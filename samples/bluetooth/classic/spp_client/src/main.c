/*
 * Copyright 2025 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/classic/classic.h>
#include <zephyr/bluetooth/classic/rfcomm.h>
#include <zephyr/bluetooth/classic/sdp.h>
#include <zephyr/settings/settings.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(spp_client, LOG_LEVEL_INF);

#define RFCOMM_MTU CONFIG_BT_RFCOMM_L2CAP_MTU
#define SPP_CLIENT_TX_MSG "Hello from SPP client"

#define SPP_STORAGE_DIR "/lfs"

#define PARTITION_NODE DT_NODELABEL(lfs1)
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);

static void spp_storage_clear(void)
{
	struct fs_dir_t dirp;
	struct fs_dirent entry;
	int rc;

	fs_dir_t_init(&dirp);

	rc = fs_opendir(&dirp, SPP_STORAGE_DIR);
	if (rc < 0) {
		return;
	}

	while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
		char path[96];

		if (entry.type == FS_DIR_ENTRY_DIR) {
			continue;
		}

		snprintf(path, sizeof(path), "%s/%s", SPP_STORAGE_DIR, entry.name);
		(void)fs_unlink(path);
	}

	fs_closedir(&dirp);
}

static void spp_storage_mount(void)
{
	int rc = fs_mount(&FS_FSTAB_ENTRY(PARTITION_NODE));

	if (rc < 0) {
		LOG_ERR("mount %s failed (err %d), file receive/ls will not work "
			"until a valid storage_partition is provided in devicetree",
			SPP_STORAGE_DIR, rc);
	} else {
		LOG_INF("storage mounted at %s", SPP_STORAGE_DIR);
		spp_storage_clear();
	}
}

#define TX_POOL_COUNT 4

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TX_POOL_COUNT, BT_RFCOMM_BUF_SIZE(RFCOMM_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

#define SDP_CLIENT_BUF_LEN 512
NET_BUF_POOL_FIXED_DEFINE(sdp_pool, 1, SDP_CLIENT_BUF_LEN, 8, NULL);

static struct bt_rfcomm_dlc rfcomm_dlc;
static bt_addr_t g_peer_addr;
static bool g_peer_valid;

static void addr_to_str(const bt_addr_t *addr, char *buf, size_t len);
static void spp_start_sdp_discover(struct bt_conn *conn);

struct file_rx_stat {
	bool active;
	bool file_open;
	bool warned_no_storage;
	struct fs_file_t file;
	char path[64];
	uint32_t total_bytes;
	int64_t first_byte_ts;
	int64_t last_byte_ts;
};
static struct file_rx_stat g_rx_stat;

/*
 * Timeout
 */
#define FILE_RX_IDLE_TIMEOUT_MS 500U
#define FILE_RX_PROGRESS_STEP   (4U * 1024U)
#define SPP_ECHO_MAX_LEN 64U

static void file_rx_idle_timeout(struct k_work *work)
{
	int64_t elapsed_ms;
	uint32_t kbps = 0;

	ARG_UNUSED(work);

	if (!g_rx_stat.active) {
		return;
	}

	elapsed_ms = g_rx_stat.last_byte_ts - g_rx_stat.first_byte_ts;
	if (elapsed_ms > 0) {
		kbps = (uint32_t)((uint64_t)g_rx_stat.total_bytes * 8U / (uint32_t)elapsed_ms);
	}

	if (g_rx_stat.file_open) {
		fs_close(&g_rx_stat.file);
		g_rx_stat.file_open = false;
		LOG_INF("[FILE RX] done: total %u bytes, %lld ms, avg speed %u kbps, "
			"saved to %s", g_rx_stat.total_bytes, elapsed_ms, kbps, g_rx_stat.path);
	} else {
		LOG_INF("[FILE RX] done: total %u bytes, %lld ms, avg speed %u kbps "
			"(not saved, storage unavailable)",
			g_rx_stat.total_bytes, elapsed_ms, kbps);
	}

	g_rx_stat.active = false;
	g_rx_stat.total_bytes = 0;
}

K_WORK_DELAYABLE_DEFINE(file_rx_idle_work, file_rx_idle_timeout);

static void file_rx_start(void)
{
	int rc;

	g_rx_stat.active = true;
	g_rx_stat.file_open = false;
	g_rx_stat.total_bytes = 0;
	g_rx_stat.first_byte_ts = k_uptime_get();

	snprintf(g_rx_stat.path, sizeof(g_rx_stat.path), "%s/spp_rx_%lld.bin",
		 SPP_STORAGE_DIR, k_uptime_get());

	fs_file_t_init(&g_rx_stat.file);
	rc = fs_open(&g_rx_stat.file, g_rx_stat.path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		if (!g_rx_stat.warned_no_storage) {
			LOG_WRN("[FILE RX] open %s failed (err %d), file will NOT be "
				"saved, only stats will be printed. Check that "
				"spp_storage_mount() succeeded at boot.",
				g_rx_stat.path, rc);
			g_rx_stat.warned_no_storage = true;
		}
		LOG_INF("[FILE RX] start receiving (no storage)...");
		return;
	}

	g_rx_stat.file_open = true;
	LOG_INF("[FILE RX] start receiving, saving to %s ...", g_rx_stat.path);
}

static void file_rx_feed(const uint8_t *data, uint32_t len)
{
	uint32_t last_reported;

	if (!g_rx_stat.active) {
		file_rx_start();
	}

	if (g_rx_stat.file_open) {
		ssize_t wr = fs_write(&g_rx_stat.file, data, len);

		if (wr < 0 || (uint32_t)wr != len) {
			LOG_ERR("[FILE RX] write failed (err %d), aborting save for this file",
				(int)wr);
			fs_close(&g_rx_stat.file);
			g_rx_stat.file_open = false;
		}
	}

	last_reported = g_rx_stat.total_bytes / FILE_RX_PROGRESS_STEP;
	g_rx_stat.total_bytes += len;
	g_rx_stat.last_byte_ts = k_uptime_get();

	if (g_rx_stat.total_bytes / FILE_RX_PROGRESS_STEP != last_reported) {
		int64_t elapsed_ms = g_rx_stat.last_byte_ts - g_rx_stat.first_byte_ts;
		uint32_t kbps = elapsed_ms > 0 ?
			(uint32_t)((uint64_t)g_rx_stat.total_bytes * 8U / (uint32_t)elapsed_ms) : 0;

		LOG_INF("[FILE RX] progress: %u bytes, %u kbps so far",
			g_rx_stat.total_bytes, kbps);
	}

	k_work_reschedule(&file_rx_idle_work, K_MSEC(FILE_RX_IDLE_TIMEOUT_MS));
}

static void rfcomm_connected(struct bt_rfcomm_dlc *dlc)
{
	ARG_UNUSED(dlc);

	LOG_INF("RFCOMM connected");
}

static void rfcomm_disconnected(struct bt_rfcomm_dlc *dlc)
{
	ARG_UNUSED(dlc);

	LOG_INF("RFCOMM disconnected");
	g_peer_valid = false;

	if (g_rx_stat.active) {
		k_work_cancel_delayable(&file_rx_idle_work);
		file_rx_idle_timeout(NULL);
	}
}

static void rfcomm_recv(struct bt_rfcomm_dlc *dlc, struct net_buf *buf)
{
	static const char msg[] = SPP_CLIENT_TX_MSG;
	struct net_buf *tx_buf;
	int err;
	bool file_data = g_rx_stat.active || buf->len > SPP_ECHO_MAX_LEN;

	if (file_data) {
		file_rx_feed(buf->data, buf->len);
		LOG_INF("RX: %u bytes (binary/file data, not echoed)", buf->len);
		return;
	}

	LOG_INF("RX: \"%.*s\" (%u bytes)", (int)buf->len, buf->data, buf->len);

	tx_buf = bt_rfcomm_create_pdu(&tx_pool);
	if (tx_buf == NULL) {
		LOG_ERR("Failed to allocate TX buffer");
		return;
	}

	if (net_buf_tailroom(tx_buf) < sizeof(msg) - 1U) {
		LOG_ERR("TX buffer too small");
		net_buf_unref(tx_buf);
		return;
	}

	net_buf_add_mem(tx_buf, msg, sizeof(msg) - 1U);
	LOG_INF("TX: \"%s\" (%u bytes)", msg, (unsigned int)(sizeof(msg) - 1U));

	err = bt_rfcomm_dlc_send(dlc, tx_buf);
	if (err < 0) {
		LOG_ERR("Reply send failed (err %d)", err);
		net_buf_unref(tx_buf);
	}
}

static struct bt_rfcomm_dlc_ops rfcomm_ops = {
	.connected = rfcomm_connected,
	.disconnected = rfcomm_disconnected,
	.recv = rfcomm_recv,
};

struct spp_pending_connect {
	bool active;
	bool auto_connect;
	bt_addr_t addr;
	struct bt_uuid_any uuid;
};
static struct spp_pending_connect g_pending;
static struct bt_sdp_discover_params dyn_sdp_discover;

static void acl_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}
	if (info.type != BT_CONN_TYPE_BR) {
		return;
	}

	if (err) {
		LOG_ERR("ACL connect failed (err 0x%02x)", err);
		g_pending.active = false;
		return;
	}

	if (info.br.dst) {
		bt_addr_copy(&g_peer_addr, info.br.dst);
		g_peer_valid = true;
	}

	{
		char astr[BT_ADDR_STR_LEN];

		addr_to_str(&g_peer_addr, astr, sizeof(astr));
		LOG_INF("ACL connected! peer addr = %s", astr);
	}

	if (g_pending.active && info.br.dst && bt_addr_cmp(info.br.dst, &g_pending.addr) == 0) {
		spp_start_sdp_discover(conn);
	}
}

static void acl_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}
	if (info.type != BT_CONN_TYPE_BR) {
		return;
	}

	LOG_INF("ACL disconnected (reason 0x%02x)", reason);
	if (info.br.dst && g_peer_valid && bt_addr_cmp(info.br.dst, &g_peer_addr) == 0) {
		g_peer_valid = false;
	}
	if (g_pending.active && info.br.dst && bt_addr_cmp(info.br.dst, &g_pending.addr) == 0) {
		g_pending.active = false;
	}
}

static void acl_security_changed(struct bt_conn *conn, bt_security_t level,
				  enum bt_security_err err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("security failed, level %u err %d", level, err);
	} else {
		LOG_INF("security level updated to %u", level);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = acl_connected,
	.disconnected = acl_disconnected,
	.security_changed = acl_security_changed,
};

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	LOG_WRN("pairing cancelled by peer or timeout");
}
static struct bt_conn_auth_cb auth_cb = {
	.cancel = auth_cancel,
};

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);
	LOG_INF("pairing complete, bonded=%d", bonded);
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	ARG_UNUSED(conn);
	LOG_ERR("pairing failed, reason=%d", reason);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = auth_pairing_complete,
	.pairing_failed = auth_pairing_failed,
};

static void bt_ready(int err)
{
	if (err != 0) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	spp_storage_mount();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_conn_cb_register(&conn_callbacks);

	err = bt_conn_auth_cb_register(&auth_cb);
	if (err != 0) {
		LOG_ERR("auth cb register failed (err %d)", err);
	}
	err = bt_conn_auth_info_cb_register(&auth_info_cb);
	if (err != 0) {
		LOG_ERR("auth info cb register failed (err %d)", err);
	}

	LOG_INF("Ready. Use `spp connect <bd_addr> <uuid_len> <uuid>` to reach "
		"a peer board running the spp_server sample.");

	{
		struct bt_br_oob oob;
		char astr[BT_ADDR_STR_LEN];

		if (bt_br_oob_get_local(&oob) == 0) {
			addr_to_str(&oob.addr, astr, sizeof(astr));
			LOG_INF("My BD address: %s", astr);
		}
	}
}

int main(void)
{
	int err;

	err = bt_enable(bt_ready);
	if (err != 0) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}

	return 0;
}

static void addr_to_str(const bt_addr_t *addr, char *buf, size_t len)
{
	snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
		 addr->val[5], addr->val[4], addr->val[3],
		 addr->val[2], addr->val[1], addr->val[0]);
}

static int spp_parse_uuid(const char *len_arg, const char *hex_arg, struct bt_uuid_any *out)
{
	unsigned long len = strtoul(len_arg, NULL, 0);
	size_t str_len = strlen(hex_arg);

	if ((len == 2U && str_len == 4U) ||
	    (len == 4U && str_len == 8U) ||
	    (len == 16U && str_len == 36U)) {
		return (bt_uuid_from_str(hex_arg, out) == 0) ? 0 : -EINVAL;
	}

	return -EINVAL;
}

static uint8_t dyn_sdp_discover_cb(struct bt_conn *conn, struct bt_sdp_client_result *response,
				   const struct bt_sdp_discover_params *params)
{
	char astr[BT_ADDR_STR_LEN];
	uint16_t channel;
	int err;

	ARG_UNUSED(params);

	addr_to_str(&g_pending.addr, astr, sizeof(astr));

	if (response == NULL || response->resp_buf == NULL) {
		LOG_WRN("[SPP SEARCH] %s: service not found", astr);
		g_pending.active = false;
		return BT_SDP_DISCOVER_UUID_STOP;
	}

	err = bt_sdp_get_proto_param(response->resp_buf, BT_SDP_PROTO_RFCOMM, &channel);
	if (err < 0) {
		LOG_WRN("[SPP SEARCH] %s: RFCOMM channel not found (err %d)", astr, err);
		g_pending.active = false;
		return BT_SDP_DISCOVER_UUID_STOP;
	}

	LOG_INF("[SPP SEARCH] %s: found RFCOMM channel %u", astr, channel);

	if (g_pending.auto_connect) {
		if (rfcomm_dlc.session != NULL) {
			LOG_WRN("[SPP CONNECT] already connected/connecting, abort");
		} else {
			rfcomm_dlc.ops = &rfcomm_ops;
			rfcomm_dlc.mtu = RFCOMM_MTU;
			rfcomm_dlc.required_sec_level = BT_SECURITY_L0;

			err = bt_rfcomm_dlc_connect(conn, &rfcomm_dlc, (uint8_t)channel);
			if (err != 0) {
				LOG_ERR("[SPP CONNECT] RFCOMM connect failed (err %d)", err);
			} else {
				LOG_INF("[SPP CONNECT] connecting RFCOMM channel %u ...",
					channel);
			}
		}
	}

	g_pending.active = false;
	return BT_SDP_DISCOVER_UUID_STOP;
}

static void spp_start_sdp_discover(struct bt_conn *conn)
{
	int err;

	dyn_sdp_discover.type = BT_SDP_DISCOVER_SERVICE_SEARCH_ATTR;
	dyn_sdp_discover.pool = &sdp_pool;
	dyn_sdp_discover.func = dyn_sdp_discover_cb;
	dyn_sdp_discover.uuid = &g_pending.uuid.uuid;

	err = bt_sdp_discover(conn, &dyn_sdp_discover);
	if (err != 0) {
		LOG_ERR("[SPP] SDP discover start failed (err %d)", err);
		g_pending.active = false;
	}
}

static int spp_search_or_connect(const struct shell *sh, size_t argc, char **argv,
				 bool auto_connect)
{
	bt_addr_t addr;
	struct bt_conn *conn;

	ARG_UNUSED(argc);

	if (bt_addr_from_str(argv[1], &addr) != 0) {
		shell_error(sh, "invalid bd_addr: %s", argv[1]);
		return -EINVAL;
	}

	if (g_pending.active) {
		shell_error(sh, "a search/connect is already in progress");
		return -EBUSY;
	}

	if (spp_parse_uuid(argv[2], argv[3], &g_pending.uuid) != 0) {
		shell_error(sh, "invalid uuid_len/uuid (uuid_len must be 2/4/16, "
			    "uuid must match that length)");
		return -EINVAL;
	}

	g_pending.active = true;
	g_pending.auto_connect = auto_connect;
	bt_addr_copy(&g_pending.addr, &addr);

	conn = bt_conn_lookup_addr_br(&addr);
	if (conn != NULL) {
		spp_start_sdp_discover(conn);
		bt_conn_unref(conn);
	} else {
		conn = bt_conn_create_br(&addr, BT_BR_CONN_PARAM_DEFAULT);
		if (conn == NULL) {
			shell_error(sh, "failed to create ACL connection to %s", argv[1]);
			g_pending.active = false;
			return -ENOTCONN;
		}
		bt_conn_unref(conn);
	}

	shell_print(sh, "%s %s uuid_len=%s uuid=%s, see log output for result",
		    auto_connect ? "connecting to" : "searching", argv[1], argv[2], argv[3]);
	return 0;
}

static int cmd_spp_search(const struct shell *sh, size_t argc, char **argv)
{
	return spp_search_or_connect(sh, argc, argv, false);
}

static int cmd_spp_connect(const struct shell *sh, size_t argc, char **argv)
{
	return spp_search_or_connect(sh, argc, argv, true);
}

static int cmd_spp_ls(const struct shell *sh, size_t argc, char **argv)
{
	const char *dir = (argc >= 2) ? argv[1] : SPP_STORAGE_DIR;
	struct fs_dir_t dirp;
	struct fs_dirent entry;
	int rc;
	int count = 0;

	fs_dir_t_init(&dirp);

	rc = fs_opendir(&dirp, dir);
	if (rc < 0) {
		shell_error(sh, "opendir %s failed (err %d) -- "
			    "check the path is an actual mount point on this board", dir, rc);
		return rc;
	}

	shell_print(sh, "%s:", dir);
	while (1) {
		rc = fs_readdir(&dirp, &entry);
		if (rc < 0) {
			shell_error(sh, "readdir failed (err %d)", rc);
			break;
		}
		if (entry.name[0] == '\0') {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			shell_print(sh, "  <DIR>  %s", entry.name);
		} else {
			char full_path[96];

			snprintf(full_path, sizeof(full_path), "%s/%s", dir, entry.name);
			shell_print(sh, "  %8u  %s", (unsigned int)entry.size, full_path);
		}
		count++;
	}

	fs_closedir(&dirp);

	if (count == 0) {
		shell_print(sh, "  (empty)");
	}
	return 0;
}

static int cmd_spp_send_data(const struct shell *sh, size_t argc, char **argv)
{
	static const char test_str[] = "This is spp test!!!";
	bt_addr_t addr;
	uint8_t channel;
	struct net_buf *tx_buf;
	int rc;

	if (bt_addr_from_str(argv[1], &addr) != 0) {
		shell_error(sh, "invalid bd_addr: %s", argv[1]);
		return -EINVAL;
	}

	if (!g_peer_valid || rfcomm_dlc.session == NULL || bt_addr_cmp(&addr, &g_peer_addr) != 0) {
		shell_error(sh, "no active RFCOMM connection to %s, use `spp connect` first",
			    argv[1]);
		return -ENOTCONN;
	}

	channel = (uint8_t)atoi(argv[2]);
	ARG_UNUSED(channel);

	tx_buf = bt_rfcomm_create_pdu(&tx_pool);
	if (!tx_buf) {
		shell_error(sh, "no tx buffer");
		return -ENOMEM;
	}
	if (net_buf_tailroom(tx_buf) < sizeof(test_str) - 1U) {
		net_buf_unref(tx_buf);
		shell_error(sh, "tx buffer too small");
		return -ENOSPC;
	}

	net_buf_add_mem(tx_buf, test_str, sizeof(test_str) - 1U);

	rc = bt_rfcomm_dlc_send(&rfcomm_dlc, tx_buf);
	if (rc < 0) {
		net_buf_unref(tx_buf);
		shell_error(sh, "send failed (err %d)", rc);
		return rc;
	}

	shell_print(sh, "sent %u bytes: \"%s\"", (unsigned int)(sizeof(test_str) - 1U), test_str);
	LOG_INF("[SEND_DATA] sent %u bytes to peer", (unsigned int)(sizeof(test_str) - 1U));
	return 0;
}

static int cmd_spp_send_file(const struct shell *sh, size_t argc, char **argv)
{
	bt_addr_t addr;
	uint8_t channel;
	const char *path;
	struct fs_file_t file;
	struct fs_dirent stat;
	uint8_t buf[512 < RFCOMM_MTU ? 512 : RFCOMM_MTU];
	uint32_t sent = 0;
	int64_t t_start;
	int rc;

	if (bt_addr_from_str(argv[1], &addr) != 0) {
		shell_error(sh, "invalid bd_addr: %s", argv[1]);
		return -EINVAL;
	}

	if (!g_peer_valid || rfcomm_dlc.session == NULL || bt_addr_cmp(&addr, &g_peer_addr) != 0) {
		shell_error(sh, "no active RFCOMM connection to %s, use `spp connect` first",
			    argv[1]);
		return -ENOTCONN;
	}

	channel = (uint8_t)atoi(argv[2]);
	ARG_UNUSED(channel);

	path = argv[3];
	rc = fs_stat(path, &stat);
	if (rc < 0) {
		shell_error(sh, "file not found: %s (err %d)", path, rc);
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, path, FS_O_READ);
	if (rc < 0) {
		shell_error(sh, "open failed (err %d)", rc);
		return rc;
	}

	shell_print(sh, "sending %s (%u bytes)...", path, (uint32_t)stat.size);
	LOG_INF("[FILE TX] start: %s, %u bytes", path, (uint32_t)stat.size);

	{
		size_t chunk_cap = sizeof(buf);

		if (rfcomm_dlc.mtu > 0 && rfcomm_dlc.mtu < chunk_cap) {
			chunk_cap = rfcomm_dlc.mtu;
		}
		shell_print(sh, "negotiated RFCOMM MTU = %u, chunk size = %u",
			    rfcomm_dlc.mtu, (unsigned int)chunk_cap);
	}

	t_start = k_uptime_get();

	while (sent < (uint32_t)stat.size) {
		size_t chunk_cap = sizeof(buf);
		ssize_t r;
		struct net_buf *tx_buf;

		if (rfcomm_dlc.mtu > 0 && rfcomm_dlc.mtu < chunk_cap) {
			chunk_cap = rfcomm_dlc.mtu;
		}

		r = fs_read(&file, buf, chunk_cap);
		if (r <= 0) {
			break;
		}

		tx_buf = bt_rfcomm_create_pdu(&tx_pool);
		if (!tx_buf) {
			shell_error(sh, "no tx buffer, abort at %u/%u bytes",
				    sent, (uint32_t)stat.size);
			break;
		}
		if (net_buf_tailroom(tx_buf) < (size_t)r) {
			net_buf_unref(tx_buf);
			shell_error(sh, "chunk too large for MTU, abort");
			break;
		}

		net_buf_add_mem(tx_buf, buf, r);

		rc = bt_rfcomm_dlc_send(&rfcomm_dlc, tx_buf);
		if (rc < 0) {
			shell_error(sh, "send failed (err %d), abort at %u/%u bytes",
				    rc, sent, (uint32_t)stat.size);
			net_buf_unref(tx_buf);
			break;
		}

		sent += (uint32_t)r;
	}

	fs_close(&file);

	{
		int64_t elapsed_ms = k_uptime_get() - t_start;
		uint32_t kbps = elapsed_ms > 0 ?
			(uint32_t)((uint64_t)sent * 8U / (uint32_t)elapsed_ms) : 0;

		shell_print(sh, "file send done: %u/%u bytes, %lld ms, %u kbps",
			    sent, (uint32_t)stat.size, elapsed_ms, kbps);
		LOG_INF("[FILE TX] done: %u/%u bytes, %lld ms, %u kbps",
			sent, (uint32_t)stat.size, elapsed_ms, kbps);
	}

	return (sent == (uint32_t)stat.size) ? 0 : -EIO;
}

static int cmd_spp_through_put(const struct shell *sh, size_t argc, char **argv)
{
	bt_addr_t addr;
	uint32_t total;
	uint8_t buf[512 < RFCOMM_MTU ? 512 : RFCOMM_MTU];
	uint32_t sent = 0;
	int64_t t_start;
	int rc;

	ARG_UNUSED(argc);

	if (bt_addr_from_str(argv[1], &addr) != 0) {
		shell_error(sh, "invalid bd_addr: %s", argv[1]);
		return -EINVAL;
	}

	if (!g_peer_valid || rfcomm_dlc.session == NULL || bt_addr_cmp(&addr, &g_peer_addr) != 0) {
		shell_error(sh, "no active RFCOMM connection to %s, use `spp connect` first",
			    argv[1]);
		return -ENOTCONN;
	}

	total = (uint32_t)strtoul(argv[3], NULL, 0);
	if (total == 0U) {
		shell_error(sh, "size must be > 0");
		return -EINVAL;
	}

	shell_print(sh, "sending %u bytes of random data...", total);
	LOG_INF("[THROUGH PUT] start: %u bytes", total);

	t_start = k_uptime_get();

	while (sent < total) {
		size_t chunk_cap = sizeof(buf);
		size_t chunk;
		struct net_buf *tx_buf;

		if (rfcomm_dlc.mtu > 0 && rfcomm_dlc.mtu < chunk_cap) {
			chunk_cap = rfcomm_dlc.mtu;
		}
		chunk = (chunk_cap < (size_t)(total - sent)) ? chunk_cap : (size_t)(total - sent);

		sys_rand_get(buf, chunk);

		tx_buf = bt_rfcomm_create_pdu(&tx_pool);
		if (!tx_buf) {
			shell_error(sh, "no tx buffer, abort at %u/%u bytes", sent, total);
			break;
		}
		if (net_buf_tailroom(tx_buf) < chunk) {
			net_buf_unref(tx_buf);
			shell_error(sh, "chunk too large for MTU, abort");
			break;
		}

		net_buf_add_mem(tx_buf, buf, chunk);

		rc = bt_rfcomm_dlc_send(&rfcomm_dlc, tx_buf);
		if (rc < 0) {
			shell_error(sh, "send failed (err %d), abort at %u/%u bytes",
				    rc, sent, total);
			net_buf_unref(tx_buf);
			break;
		}

		sent += (uint32_t)chunk;
	}

	{
		int64_t elapsed_ms = k_uptime_get() - t_start;
		uint32_t kbps = elapsed_ms > 0 ?
			(uint32_t)((uint64_t)sent * 8U / (uint32_t)elapsed_ms) : 0;

		shell_print(sh, "through_put done: %u/%u bytes, %lld ms, %u kbps",
			    sent, total, elapsed_ms, kbps);
		LOG_INF("[THROUGH PUT] done: %u/%u bytes, %lld ms, %u kbps",
			sent, total, elapsed_ms, kbps);
	}

	return (sent == total) ? 0 : -EIO;
}

static int cmd_spp_addr(const struct shell *sh, size_t argc, char **argv)
{
	struct bt_br_oob oob;
	char astr[BT_ADDR_STR_LEN];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (bt_br_oob_get_local(&oob) != 0) {
		shell_error(sh, "failed to get local BR/EDR address");
		return -EIO;
	}

	addr_to_str(&oob.addr, astr, sizeof(astr));
	shell_print(sh, "local BR/EDR address: %s", astr);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(spp_cmds,
	SHELL_CMD_ARG(addr, NULL,
		      "addr -- print local BD address",
		      cmd_spp_addr, 1, 0),
	SHELL_CMD_ARG(ls, NULL,
		      "ls [dir] (default: " SPP_STORAGE_DIR ")",
		      cmd_spp_ls, 1, 1),
	SHELL_CMD_ARG(search, NULL,
		      "search <bd_addr> <uuid_len> <uuid> -- board-to-board SDP search",
		      cmd_spp_search, 4, 0),
	SHELL_CMD_ARG(connect, NULL,
		      "connect <bd_addr> <uuid_len> <uuid> -- board-to-board SDP search + "
		      "RFCOMM connect",
		      cmd_spp_connect, 4, 0),
	SHELL_CMD_ARG(send_data, NULL,
		      "send_data <bd_addr> <service_channel>",
		      cmd_spp_send_data, 3, 0),
	SHELL_CMD_ARG(send_file, NULL,
		      "send_file <bd_addr> <service_channel> <file_path>",
		      cmd_spp_send_file, 4, 0),
	SHELL_CMD_ARG(through_put, NULL,
		      "through_put <bd_addr> <service_channel> <size> -- send random data "
		      "and measure throughput",
		      cmd_spp_through_put, 4, 0),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(spp, &spp_cmds, "SPP command", NULL);
