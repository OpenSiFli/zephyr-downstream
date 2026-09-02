/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_string_conv.h>
#include <zephyr/data/json.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/icmp.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/dhcpv4.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/classic/pan.h>

LOG_MODULE_REGISTER(pan_panu, LOG_LEVEL_INF);

#define PEER_IPV4_ADDR CONFIG_NET_CONFIG_PEER_IPV4_ADDR
#define WEATHER_HOST "api.seniverse.com"
#define WEATHER_KEY_ID "SO23_Gmly2oK3kMf4"
#define WEATHER_CITY_ID "chongqing"
#define WEATHER_LANGUAGE_ID "zh-Hans&unit=c"

#define NVDS_FLASH_DEVICE DEVICE_DT_GET(DT_NODELABEL(mpi2))
#define NVDS_NOR_BASE DT_REG_ADDR(DT_NODELABEL(py25q128ha_memory))
#define NVDS_FLASH_OFFSET \
	(DT_REG_ADDR(DT_NODELABEL(nvds_partition)) - NVDS_NOR_BASE)
#define NVDS_FLASH_SIZE DT_REG_SIZE(DT_NODELABEL(nvds_partition))
#define NVDS_RECORD_MAGIC 0x5344564EU
#define NVDS_TAG_BD_ADDRESS 0x01U
#define NVDS_BD_ADDRESS_LEN 6U
#define SIFLI_NVDS_RAM_ADDR 0x2040FE00U
#define SIFLI_NVDS_RAM_PATTERN 0x4E564453U

struct nvds_record {
	uint32_t magic;
	uint8_t data[2U + NVDS_BD_ADDRESS_LEN];
	uint32_t crc;
};

struct sifli_nvds_ram_header {
	uint32_t pattern;
	uint16_t used_mem;
	uint16_t writing;
};

BT_PAN_NET_DEVICE_DEFINE(pan_panu0, "pan_panu0", 0x02, 0x00, 0x00, 0x44, 0x00, 0x02);

static struct net_if *pan_iface;
static struct bt_conn *default_conn;
static struct bt_pan *active_pan;
static struct k_work_delayable ping_work;
static struct k_work_delayable discover_work;
static struct k_work_delayable reconnect_work;
static struct k_work_delayable network_check_timeout_work;
static struct net_mgmt_event_callback ipv4_cb;
static bt_addr_t last_peer_addr;
static bool last_peer_valid;
static bool discovery_active;
static bool reconnect_enabled = true;
static bool reconnect_active;
static uint8_t reconnect_attempts;
static uint8_t reconnect_max_attempts = 5U;
static uint8_t ping_sent_count;
static uint8_t ping_reply_count;
static bool ping_pending;

#define PING_CHECK_COUNT 3U
#define PING_CHECK_INTERVAL K_SECONDS(1)

#define DISCOVERY_INTERVAL K_SECONDS(2)
#define RECONNECT_DELAY    K_SECONDS(5)
#define AUTOCONNECT_INTERVAL K_SECONDS(5)
#define NETWORK_CHECK_TIMEOUT K_SECONDS(15)

static struct bt_br_discovery_param br_discover = {
	.length = 10,
	.limited = false,
};

static struct bt_br_discovery_result scan_result[8];
static uint8_t scan_count;

static bool is_nap_device(const struct bt_br_discovery_result *result)
{
	uint32_t cod;
	uint8_t major;

	cod = sys_get_le24(result->cod);
	major = (uint8_t)BT_COD_MAJOR_DEVICE_CLASS(result->cod);

	return (major == BT_COD_MAJOR_DEVICE_CLASS_LAN_NETWORK) ||
	       (major == BT_COD_MAJOR_DEVICE_CLASS_PHONE) ||
	       ((cod & BT_COD_MAJOR_SVC_CLASS_NETWORKING) != 0U);
}

static bool bd_addr_oui_is_valid(uint8_t first_octet)
{
	return (first_octet & 0x03U) == 0x00U;
}

static int controller_read_bd_addr(bt_addr_t *addr)
{
	struct bt_hci_rp_read_bd_addr *rp;
	struct net_buf *rsp;
	int err;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BD_ADDR, NULL, &rsp);
	if (err != 0) {
		return err;
	}

	rp = (void *)rsp->data;
	if (rp->status != BT_HCI_ERR_SUCCESS) {
		err = -EIO;
	} else if (bt_addr_eq(&rp->bdaddr, BT_ADDR_ANY) ||
		   bt_addr_eq(&rp->bdaddr, BT_ADDR_NONE)) {
		err = -ENODATA;
	} else {
		bt_addr_copy(addr, &rp->bdaddr);
	}

	net_buf_unref(rsp);
	return err;
}

static bool nvds_record_is_valid(const struct nvds_record *record)
{
	return record->magic == NVDS_RECORD_MAGIC &&
	       record->data[0] == NVDS_TAG_BD_ADDRESS &&
	       record->data[1] == NVDS_BD_ADDRESS_LEN &&
	       record->crc == crc32_ieee(record->data, sizeof(record->data));
}

static int nvds_record_read(struct nvds_record *record)
{
	const struct device *flash = NVDS_FLASH_DEVICE;
	int err = flash_read(flash, NVDS_FLASH_OFFSET, record, sizeof(*record));

	if (err != 0) {
		return err;
	}

	return nvds_record_is_valid(record) ? 0 : -ENOENT;
}

static int nvds_record_write(const uint8_t addr[NVDS_BD_ADDRESS_LEN])
{
	const struct device *flash = NVDS_FLASH_DEVICE;
	struct nvds_record record = {
		.magic = NVDS_RECORD_MAGIC,
		.data = {NVDS_TAG_BD_ADDRESS, NVDS_BD_ADDRESS_LEN},
	};
	int err;

	memcpy(&record.data[2], addr, NVDS_BD_ADDRESS_LEN);
	record.crc = crc32_ieee(record.data, sizeof(record.data));

	err = flash_erase(flash, NVDS_FLASH_OFFSET, NVDS_FLASH_SIZE);
	if (err == 0) {
		err = flash_write(flash, NVDS_FLASH_OFFSET, &record, sizeof(record));
	}

	return err;
}

void lcpu_nvds_config(void)
{
	struct nvds_record record;
	struct sifli_nvds_ram_header *header =
		(struct sifli_nvds_ram_header *)SIFLI_NVDS_RAM_ADDR;

	if (nvds_record_read(&record) != 0) {
		return;
	}

	header->pattern = SIFLI_NVDS_RAM_PATTERN;
	header->used_mem = sizeof(record.data);
	header->writing = 0U;
	memcpy(header + 1, record.data, sizeof(record.data));
}

static enum net_verdict ping_reply(struct net_icmp_ctx *ctx, struct net_pkt *pkt,
				   struct net_icmp_ip_hdr *hdr, struct net_icmp_hdr *icmp_hdr,
				   void *user_data)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(pkt);
	ARG_UNUSED(hdr);
	ARG_UNUSED(icmp_hdr);
	ARG_UNUSED(user_data);

	if (!ping_pending) {
		return NET_OK;
	}

	ping_pending = false;
	ping_reply_count++;
	printk("Ping reply received (%u/%u)\n", ping_reply_count, PING_CHECK_COUNT);

	if (ping_reply_count == PING_CHECK_COUNT) {
		printk("PAN network connection is normal\n");
		if (reconnect_active) {
			printk("PAN autoconnect succeeded after %u attempt(s)\n",
			       reconnect_attempts);
			(void)k_work_cancel_delayable(&reconnect_work);
			reconnect_active = false;
			reconnect_attempts = 0U;
		}
	}

	return NET_OK;
}

static struct net_icmp_ctx ping_ctx;

static void network_check_timeout_handler(struct k_work *work)
{
	struct bt_conn *conn;

	ARG_UNUSED(work);

	if (!reconnect_active || active_pan == NULL || ping_sent_count != 0U) {
		return;
	}

	printk("PAN autoconnect network setup timed out\n");
	conn = bt_pan_get_conn(active_pan);
	if (conn != NULL) {
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void send_ping(struct k_work *work)
{
	struct net_sockaddr_in dst = {
		.sin_family = AF_INET,
		.sin_port = 0,
	};
	struct net_icmp_ping_params params = {
		.identifier = 0x5678,
		.sequence = 1,
		.data_size = 32,
	};
	int ret;

	ARG_UNUSED(work);

	if (active_pan == NULL || bt_pan_get_state(active_pan) != BT_PAN_STATE_CONNECTED) {
		return;
	}

	if (ping_reply_count == PING_CHECK_COUNT) {
		return;
	}

	if (ping_sent_count == PING_CHECK_COUNT) {
		struct bt_conn *conn;

		printk("PAN network check failed: %u/%u ping replies\n", ping_reply_count,
		       PING_CHECK_COUNT);
		ping_pending = false;

		if (reconnect_active) {
			conn = bt_pan_get_conn(active_pan);
			if (conn != NULL) {
				(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			}
		}
		return;
	}

	ret = zsock_inet_pton(AF_INET, PEER_IPV4_ADDR, &dst.sin_addr);
	if (ret != 1) {
		printk("Invalid peer address\n");
		return;
	}

	params.sequence = ping_sent_count + 1U;
	ret = net_icmp_send_echo_request_no_wait(&ping_ctx, pan_iface,
						 (struct net_sockaddr *)&dst, &params, NULL);
	if (ret == 0) {
		ping_sent_count++;
		ping_pending = true;
		printk("Ping sent to %s (%u/%u)\n", PEER_IPV4_ADDR, ping_sent_count,
		       PING_CHECK_COUNT);
	} else {
		printk("Ping failed (%d)\n", ret);
	}

	(void)k_work_schedule(&ping_work, PING_CHECK_INTERVAL);
}

static void ipv4_addr_add_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				  struct net_if *iface)
{
	char buf[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(cb);

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD || iface != pan_iface) {
		return;
	}

	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
			continue;
		}

		printk("DHCP address: %s\n",
		       net_addr_ntop(NET_AF_INET,
				     &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr, buf,
				     sizeof(buf)));
		printk("DHCP gateway: %s\n",
		       net_addr_ntop(NET_AF_INET, &iface->config.ip.ipv4->gw, buf, sizeof(buf)));

		(void)k_work_cancel_delayable(&network_check_timeout_work);
		(void)k_work_schedule(&ping_work, K_SECONDS(1));
		return;
	}
}

static void pan_connected(struct bt_pan *pan)
{
	printk("PAN connected\n");

	active_pan = pan;
	ping_sent_count = 0U;
	ping_reply_count = 0U;
	ping_pending = false;
	bt_pan_net_attach(pan, pan_iface);
	net_dhcpv4_restart(pan_iface);
	if (reconnect_active) {
		(void)k_work_schedule(&network_check_timeout_work, NETWORK_CHECK_TIMEOUT);
	}
	printk("DHCPv4 client started\n");
}

static void pan_disconnected(struct bt_pan *pan)
{
	struct bt_conn *conn;

	printk("PAN disconnected\n");

	(void)k_work_cancel_delayable(&ping_work);
	(void)k_work_cancel_delayable(&network_check_timeout_work);
	ping_pending = false;
	net_dhcpv4_stop(pan_iface);
	bt_pan_net_detach(pan);
	active_pan = NULL;

	conn = bt_pan_get_conn(pan);
	if (conn != NULL) {
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static struct bt_pan_cb pan_cb = {
	.connected = pan_connected,
	.disconnected = pan_disconnected,
};

static void try_pan_connect(struct bt_conn *conn)
{
	struct bt_pan *pan;
	int ret;

	pan = bt_pan_lookup(conn);
	if (pan == NULL) {
		printk("No PAN session\n");
		return;
	}

	if (bt_pan_get_state(pan) != BT_PAN_STATE_DISCONNECTED) {
		return;
	}

	ret = bt_pan_connect(conn, pan);
	if (ret != 0) {
		printk("PAN connect failed (%d)\n", ret);
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("ACL connection failed (err %u)\n", err);

		if (conn == default_conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;
		}

		if (reconnect_active) {
			if (reconnect_enabled && reconnect_attempts < reconnect_max_attempts) {
				(void)k_work_schedule(&reconnect_work, AUTOCONNECT_INTERVAL);
			} else {
				printk("PAN autoconnect stopped after %u attempt(s)\n",
				       reconnect_attempts);
				reconnect_active = false;
			}
		} else {
			(void)k_work_schedule(&discover_work, RECONNECT_DELAY);
		}
		return;
	}

	printk("ACL %s\n", __func__);

	if (conn != default_conn) {
		printk("Unexpected ACL connection\n");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	if (bt_conn_set_security(conn, BT_SECURITY_L2) != 0) {
		printk("Security upgrade failed\n");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		return;
	}

	if (bt_conn_get_security(conn) >= BT_SECURITY_L2) {
		try_pan_connect(conn);
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	if (conn != default_conn) {
		return;
	}

	if (err != BT_SECURITY_ERR_SUCCESS) {
		printk("Pairing failed (%d)\n", err);
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		return;
	}

	printk("Security level %u\n", level);

	if (level < BT_SECURITY_L2) {
		return;
	}

	printk("Link encrypted, starting PAN\n");
	try_pan_connect(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("ACL %s (reason %u)\n", __func__, reason);

	if (default_conn == conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	if (reconnect_active) {
		if (reconnect_enabled && reconnect_attempts < reconnect_max_attempts) {
			(void)k_work_schedule(&reconnect_work, AUTOCONNECT_INTERVAL);
		} else {
			printk("PAN autoconnect stopped after %u attempt(s)\n",
			       reconnect_attempts);
			reconnect_active = false;
		}
	} else {
		(void)k_work_schedule(&discover_work, RECONNECT_DELAY);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void discovery_timeout_cb(const struct bt_br_discovery_result *results, size_t count)
{
	discovery_active = false;
	scan_count = (count > ARRAY_SIZE(scan_result)) ? ARRAY_SIZE(scan_result) : count;

	if (scan_count == 0U) {
		printk("No Bluetooth devices found\n");
		return;
	}

	for (size_t i = 0; i < scan_count; i++) {
		bool nap = is_nap_device(&results[i]);

		printk("Device[%zu]: %s, rssi %d, cod 0x%02x%02x%02x%s\n", i,
		       bt_addr_str(&results[i].addr), results[i].rssi, results[i].cod[2],
		       results[i].cod[1], results[i].cod[0], nap ? " [PAN/NET]" : "");

		if (nap) {
			bt_addr_copy(&last_peer_addr, &results[i].addr);
			last_peer_valid = true;
		}
	}

	printk("Run 'pan connect <index>' to connect to one of the devices above\n");
}

static void discover_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (default_conn != NULL) {
		return;
	}

	err = bt_br_discovery_start(&br_discover, scan_result, ARRAY_SIZE(scan_result));
	if (err != 0) {
		printk("Discovery start failed (%d)\n", err);
		(void)k_work_schedule(&discover_work, DISCOVERY_INTERVAL);
	} else {
		discovery_active = true;
	}
}

static void reconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!reconnect_active || !reconnect_enabled || default_conn != NULL) {
		return;
	}

	if (!last_peer_valid || reconnect_attempts >= reconnect_max_attempts) {
		printk("PAN autoconnect stopped after %u attempt(s)\n", reconnect_attempts);
		reconnect_active = false;
		return;
	}

	reconnect_attempts++;
	printk("PAN autoconnect attempt %u/%u to %s\n", reconnect_attempts,
	       reconnect_max_attempts, bt_addr_str(&last_peer_addr));

	default_conn = bt_conn_create_br(&last_peer_addr, BT_BR_CONN_PARAM_DEFAULT);
	if (default_conn == NULL) {
		printk("PAN autoconnect could not create ACL connection\n");
		if (reconnect_attempts < reconnect_max_attempts) {
			(void)k_work_schedule(&reconnect_work, AUTOCONNECT_INTERVAL);
		} else {
			reconnect_active = false;
			printk("PAN autoconnect attempts exhausted\n");
		}
	}
}

static struct bt_br_discovery_cb discovery_cb = {
	.timeout = discovery_timeout_cb,
};

static int pan_iface_init(void)
{
	pan_iface = bt_pan_net_get_iface(DEVICE_GET(pan_panu0));
	if (pan_iface == NULL) {
		printk("PAN network interface not found\n");
		return -ENODEV;
	}

	if (!net_if_flag_is_set(pan_iface, NET_IF_UP)) {
		net_if_up(pan_iface);
	}

	return 0;
}

static void bt_ready(int err)
{
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth PAN User initialized\n");

	{
		bt_addr_le_t id_addrs[CONFIG_BT_ID_MAX];
		size_t id_count = ARRAY_SIZE(id_addrs);

		bt_id_get(id_addrs, &id_count);
		if (id_count > 0U && !bd_addr_oui_is_valid(id_addrs[0].a.val[5])) {
			printk("WARNING: local BD_ADDR %02X:%02X:%02X:%02X:%02X:%02X has an "
			       "invalid/locally-administered OUI\n",
			       id_addrs[0].a.val[5], id_addrs[0].a.val[4],
			       id_addrs[0].a.val[3], id_addrs[0].a.val[2],
			       id_addrs[0].a.val[1], id_addrs[0].a.val[0]);
			printk("Some phones/hotspots misbehave with such addresses (DHCP "
			       "oddities, 'pan weather' failing to reach the internet, "
			       "etc). Run 'pan mac get' for details.\n");
		}
	}

	if (pan_iface_init() != 0) {
		return;
	}

	bt_br_discovery_cb_register(&discovery_cb);
	bt_pan_register(BT_PAN_ROLE_PANU, &pan_cb);

	k_work_init_delayable(&discover_work, discover_work_handler);
	k_work_init_delayable(&reconnect_work, reconnect_work_handler);
	k_work_init_delayable(&network_check_timeout_work, network_check_timeout_handler);
	(void)k_work_schedule(&discover_work, K_SECONDS(1));
}

int main(void)
{
	int err;

	k_work_init_delayable(&ping_work, send_ping);

	net_mgmt_init_event_callback(&ipv4_cb, ipv4_addr_add_handler, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	err = net_icmp_init_ctx(&ping_ctx, NET_AF_INET, NET_ICMPV4_ECHO_REPLY, 0, ping_reply);
	if (err != 0) {
		printk("ICMP init failed (%d)\n", err);
		return 0;
	}

	err = bt_enable(bt_ready);
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	return 0;
}

/*
 * Shell commands: `pan status|scan|connect|disconnect|ping|autoconnect|weather`
 *
 * Modelled after the spp_server sample's shell command system: one static
 * subcommand table registered via SHELL_STATIC_SUBCMD_SET_CREATE(), then a
 * single top-level SHELL_CMD_REGISTER(). Keeping everything under one `pan`
 * command (instead of scattering separate top-level commands) mirrors how
 * spp_server groups all of its SPP operations under `spp <subcmd>`.
 */
static int cmd_pan_status(const struct shell *sh, size_t argc, char **argv)
{
	const char *pan_state_str = "none";

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "ACL           : %s", default_conn ? "connected" : "disconnected");
	shell_print(sh, "Autoconnect   : %s, attempts %u/%u",
		    reconnect_enabled ? (reconnect_active ? "running" : "enabled") : "disabled",
		    reconnect_attempts, reconnect_max_attempts);

	if (active_pan != NULL) {
		enum bt_pan_state state = bt_pan_get_state(active_pan);

		if (state == BT_PAN_STATE_CONNECTED) {
			pan_state_str = "connected";
		} else if (state == BT_PAN_STATE_DISCONNECTED) {
			pan_state_str = "disconnected";
		} else {
			pan_state_str = "connecting/disconnecting";
		}
	}
	shell_print(sh, "PAN session   : %s", pan_state_str);

	if (pan_iface == NULL) {
		shell_print(sh, "Net interface : not initialized");
		return 0;
	}

	shell_print(sh, "Net interface : %s", net_if_is_up(pan_iface) ? "up" : "down");

	if (pan_iface->config.ip.ipv4 != NULL) {
		char buf[NET_IPV4_ADDR_LEN];
		bool has_addr = false;

		for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
			if (pan_iface->config.ip.ipv4->unicast[i].ipv4.addr_type !=
			    NET_ADDR_DHCP) {
				continue;
			}

			shell_print(sh, "IPv4 address  : %s",
				   net_addr_ntop(NET_AF_INET,
						&pan_iface->config.ip.ipv4->unicast[i]
							 .ipv4.address.in_addr,
						buf, sizeof(buf)));
			shell_print(sh, "IPv4 gateway  : %s",
				   net_addr_ntop(NET_AF_INET, &pan_iface->config.ip.ipv4->gw,
						buf, sizeof(buf)));
			has_addr = true;
			break;
		}

		if (!has_addr) {
			shell_print(sh, "IPv4 address  : none (no DHCP lease yet)");
		}
	}

	return 0;
}

static int cmd_pan_scan(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (default_conn != NULL) {
		shell_warn(sh, "ACL already connected/connecting, run 'pan disconnect' first");
		return -EBUSY;
	}

	shell_print(sh, "Restarting BR/EDR discovery...");
	(void)k_work_reschedule(&discover_work, K_NO_WAIT);
	return 0;
}

static int cmd_pan_connect(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long index;
	int err = 0;

	ARG_UNUSED(argc);

	if (default_conn != NULL) {
		shell_warn(sh, "ACL already connected/connecting, run 'pan disconnect' first");
		return -EBUSY;
	}

	if (discovery_active) {
		shell_warn(sh, "discovery still running, wait for the device list first");
		return -EBUSY;
	}

	index = shell_strtoul(argv[1], 10, &err);
	if (err != 0 || scan_count == 0U || index >= scan_count) {
		shell_error(sh, "invalid device index (0..%u)",
			    scan_count > 0U ? (unsigned int)scan_count - 1U : 0U);
		return -EINVAL;
	}

	shell_print(sh, "Connecting to Device[%lu]: %s", index,
		    bt_addr_str(&scan_result[index].addr));

	if (is_nap_device(&scan_result[index])) {
		bt_addr_copy(&last_peer_addr, &scan_result[index].addr);
		last_peer_valid = true;
	}

	default_conn = bt_conn_create_br(&scan_result[index].addr, BT_BR_CONN_PARAM_DEFAULT);
	if (default_conn == NULL) {
		shell_error(sh, "failed to create connection");
		return -EIO;
	}

	return 0;
}

static int cmd_pan_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (default_conn == NULL) {
		shell_warn(sh, "not connected");
		return -ENOTCONN;
	}

	shell_print(sh, "Disconnecting...");
	return bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static int cmd_pan_ping(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (active_pan == NULL || bt_pan_get_state(active_pan) != BT_PAN_STATE_CONNECTED) {
		shell_warn(sh, "PAN not connected yet");
		return -ENOTCONN;
	}

	shell_print(sh, "Sending ping to %s ...", PEER_IPV4_ADDR);
	(void)k_work_reschedule(&ping_work, K_NO_WAIT);
	return 0;
}

static int cmd_pan_set_retry_flag(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long enabled;
	int err = 0;

	ARG_UNUSED(argc);

	enabled = shell_strtoul(argv[1], 10, &err);
	if (err != 0 || enabled > 1UL) {
		shell_error(sh, "retry flag must be 0 or 1");
		return -EINVAL;
	}

	reconnect_enabled = enabled == 1UL;
	if (!reconnect_enabled) {
		(void)k_work_cancel_delayable(&reconnect_work);
		reconnect_active = false;
		reconnect_attempts = 0U;
	}

	shell_print(sh, "PAN autoconnect %s", reconnect_enabled ? "enabled" : "disabled");
	return 0;
}

static int cmd_pan_set_retry_time(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long attempts;
	int err = 0;

	ARG_UNUSED(argc);

	attempts = shell_strtoul(argv[1], 10, &err);
	if (err != 0 || attempts == 0UL || attempts > UINT8_MAX) {
		shell_error(sh, "retry count must be 1..%u", UINT8_MAX);
		return -EINVAL;
	}

	reconnect_max_attempts = (uint8_t)attempts;
	shell_print(sh, "PAN autoconnect max attempts: %u", reconnect_max_attempts);
	return 0;
}

static int cmd_pan_autoconnect(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!reconnect_enabled) {
		shell_error(sh, "autoconnect is disabled; run 'pan set_retry_flag 1'");
		return -EACCES;
	}

	if (!last_peer_valid) {
		shell_error(sh, "no previously discovered PAN peer");
		return -ENOENT;
	}

	if (default_conn != NULL || active_pan != NULL) {
		shell_warn(sh, "ACL/PAN is already connected or connecting");
		return -EBUSY;
	}

	(void)k_work_cancel_delayable(&discover_work);
	if (discovery_active) {
		err = bt_br_discovery_stop();
		if (err != 0) {
			shell_error(sh, "failed to stop discovery (%d)", err);
			return err;
		}
		discovery_active = false;
	}

	reconnect_attempts = 0U;
	reconnect_active = true;
	shell_print(sh, "Starting PAN autoconnect to %s", bt_addr_str(&last_peer_addr));
	(void)k_work_reschedule(&reconnect_work, K_NO_WAIT);
	return 0;
}

static int cmd_pan_mac_get(const struct shell *sh, size_t argc, char **argv)
{
	bt_addr_t addr;
	struct nvds_record record;
	const uint8_t *a;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = controller_read_bd_addr(&addr);
	if (err != 0) {
		shell_error(sh, "failed to read controller BD_ADDR (%d)", err);
		return err;
	}

	a = addr.val;

	shell_print(sh, "BD_ADDR : %02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2],
		   a[1], a[0]);

	if (nvds_record_read(&record) == 0) {
		a = &record.data[2];
		shell_print(sh, "NVDS    : %02X:%02X:%02X:%02X:%02X:%02X%s",
			    a[5], a[4], a[3], a[2], a[1], a[0],
			    (memcmp(a, addr.val, NVDS_BD_ADDRESS_LEN) == 0) ?
				"" : " (reboot pending)");
	}

	if (bd_addr_oui_is_valid(a[5])) {
		shell_print(sh, "OUI     : OK (IEEE-assigned, unicast)");
	} else {
		shell_warn(sh, "OUI     : INVALID (locally-administered and/or multicast "
			   "bit set on %02X)", a[5]);
		shell_warn(sh, "Use 'pan mac oui_reverse <OUI> [suffix]' to build the "
			   "'nvds update addr 6' argument");
	}

	return 0;
}

static int parse_hex_bytes(const char *hex, uint8_t *out, size_t out_len)
{
	char clean[16];
	size_t j = 0;

	for (size_t i = 0; hex[i] != '\0'; i++) {
		if (hex[i] == ':' || hex[i] == '-') {
			continue;
		}
		if (j >= sizeof(clean) - 1U) {
			return -EINVAL;
		}
		clean[j++] = hex[i];
	}
	clean[j] = '\0';

	if (j != out_len * 2U) {
		return -EINVAL;
	}

	for (size_t i = 0; i < out_len; i++) {
		char byte_str[3] = { clean[i * 2], clean[i * 2 + 1], '\0' };
		char *endptr = NULL;
		unsigned long val = strtoul(byte_str, &endptr, 16);

		if (endptr == byte_str || *endptr != '\0') {
			return -EINVAL;
		}
		out[i] = (uint8_t)val;
	}

	return 0;
}

static int cmd_pan_mac_oui_reverse(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t oui[3];
	uint8_t suffix[3] = { 0x00, 0x00, 0x00 };
	char nvds_arg[16];

	if (parse_hex_bytes(argv[1], oui, sizeof(oui)) != 0) {
		shell_error(sh, "OUI must be 3 hex bytes, e.g. E86A64 or E8:6A:64");
		return -EINVAL;
	}

	if (!bd_addr_oui_is_valid(oui[0])) {
		shell_warn(sh, "%02X:%02X:%02X does not look like an IEEE-assigned OUI "
			   "(locally-administered or multicast bit set on %02X)",
			   oui[0], oui[1], oui[2], oui[0]);
	}

	if (argc > 2) {
		if (parse_hex_bytes(argv[2], suffix, sizeof(suffix)) != 0) {
			shell_error(sh, "suffix must be 3 hex bytes, e.g. 52FD5C");
			return -EINVAL;
		}
	} else {
		shell_warn(sh, "no suffix given, using placeholder 000000 -- replace it "
			   "with your own 3 bytes");
	}

	snprintf(nvds_arg, sizeof(nvds_arg), "%02X%02X%02X%02X%02X%02X", suffix[0], suffix[1],
		suffix[2], oui[2], oui[1], oui[0]);

	shell_print(sh, "OUI              : %02X:%02X:%02X", oui[0], oui[1], oui[2]);
	shell_print(sh, "Reversed OUI     : %02X%02X%02X", oui[2], oui[1], oui[0]);
	shell_print(sh, "Resulting BD_ADDR: %02X:%02X:%02X:%02X:%02X:%02X", oui[0], oui[1],
		   oui[2], suffix[2], suffix[1], suffix[0]);
	shell_print(sh, "Run              : nvds update addr 6 %s", nvds_arg);

	return 0;
}

static int cmd_nvds_update(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t encoded[6];
	int err;

	ARG_UNUSED(argc);

	if (strcmp(argv[1], "addr") != 0 || strcmp(argv[2], "6") != 0) {
		shell_error(sh, "usage: nvds update addr 6 <suffix><reversed-OUI>");
		return -EINVAL;
	}

	err = parse_hex_bytes(argv[3], encoded, sizeof(encoded));
	if (err != 0) {
		shell_error(sh, "address must contain 12 hex digits");
		return err;
	}

	if (!bd_addr_oui_is_valid(encoded[5])) {
		shell_error(sh, "OUI %02X:%02X:%02X is not an IEEE unicast OUI",
			    encoded[5], encoded[4], encoded[3]);
		return -EINVAL;
	}

	if (default_conn != NULL || active_pan != NULL) {
		shell_error(sh, "disconnect Bluetooth links before changing BD_ADDR");
		return -EBUSY;
	}

	err = nvds_record_write(encoded);
	if (err != 0) {
		shell_error(sh, "failed to store NVDS BD_ADDR (%d)", err);
		return err;
	}

	shell_print(sh, "NVDS BD_ADDR stored: %02X:%02X:%02X:%02X:%02X:%02X",
		    encoded[5], encoded[4], encoded[3], encoded[2], encoded[1], encoded[0]);
	shell_print(sh, "reboot to apply, then run 'nvds get_mac' to verify");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pan_mac_cmds,
	SHELL_CMD_ARG(get, NULL,
		      "get -- print local BD_ADDR and validate its OUI",
		      cmd_pan_mac_get, 1, 0),
	SHELL_CMD_ARG(oui_reverse, NULL,
		      "oui_reverse <OUI> [suffix] -- build the 'nvds update addr 6' argument",
		      cmd_pan_mac_oui_reverse, 2, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(nvds_cmds,
	SHELL_CMD_ARG(get_mac, NULL,
		      "get_mac -- read and validate the controller BD_ADDR",
		      cmd_pan_mac_get, 1, 0),
	SHELL_CMD_ARG(update, NULL,
		      "update addr 6 <suffix><reversed-OUI> -- update controller BD_ADDR",
		      cmd_nvds_update, 4, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(nvds, &nvds_cmds, "Bluetooth controller address commands", NULL);

#define WEATHER_PORT       "80"
#define WEATHER_URI_FMT    "/v3/weather/now.json?key=%s&location=%s&language=%s"
#define WEATHER_REQUEST_BUF_SIZE  384
#define WEATHER_PATH_BUF_SIZE     256
#define WEATHER_RESPONSE_BUF_SIZE 2048
#define WEATHER_RECV_TIMEOUT_SEC  5

struct weather_now {
	const char *text;        /* condition, e.g. "多云" */
	const char *code;        /* condition code */
	const char *temperature; /* degrees, as string */
};

struct weather_location {
	const char *id;
	const char *name;
	const char *country;
	const char *path;
	const char *timezone;
	const char *timezone_offset;
};

struct weather_result {
	struct weather_location location;
	struct weather_now now;
	const char *last_update;
};

struct weather_response {
	struct weather_result results[1];
	size_t results_len;
};

static const struct json_obj_descr weather_now_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct weather_now, text, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_now, code, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_now, temperature, JSON_TOK_STRING),
};

static const struct json_obj_descr weather_location_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct weather_location, id, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_location, name, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_location, country, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_location, path, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_location, timezone, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct weather_location, timezone_offset, JSON_TOK_STRING),
};

static const struct json_obj_descr weather_result_descr[] = {
	JSON_OBJ_DESCR_OBJECT(struct weather_result, location, weather_location_descr),
	JSON_OBJ_DESCR_OBJECT(struct weather_result, now, weather_now_descr),
	JSON_OBJ_DESCR_PRIM(struct weather_result, last_update, JSON_TOK_STRING),
};

static const struct json_obj_descr weather_response_descr[] = {
	JSON_OBJ_DESCR_OBJ_ARRAY(struct weather_response, results, 1,
				  results_len, weather_result_descr,
				  ARRAY_SIZE(weather_result_descr)),
};

static char weather_request_buf[WEATHER_REQUEST_BUF_SIZE];
static char weather_response_buf[WEATHER_RESPONSE_BUF_SIZE];

static int weather_http_get(const struct shell *sh, const char *city, char *out,
			    size_t out_size)
{
	struct zsock_addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct zsock_addrinfo *res = NULL;
	struct zsock_timeval timeout = {
		.tv_sec = WEATHER_RECV_TIMEOUT_SEC,
		.tv_usec = 0,
	};
	char path[WEATHER_PATH_BUF_SIZE];
	int sock;
	int ret;
	size_t total = 0;

	ret = zsock_getaddrinfo(WEATHER_HOST, WEATHER_PORT, &hints, &res);
	if (ret != 0 || res == NULL) {
		shell_error(sh, "DNS lookup for %s failed (%d)", WEATHER_HOST, ret);
		return -EHOSTUNREACH;
	}

	sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if (sock < 0) {
		shell_error(sh, "socket() failed (%d)", errno);
		zsock_freeaddrinfo(res);
		return -errno;
	}

	(void)zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

	ret = zsock_connect(sock, res->ai_addr, res->ai_addrlen);
	zsock_freeaddrinfo(res);
	if (ret < 0) {
		shell_error(sh, "connect() to %s failed (%d)", WEATHER_HOST, errno);
		zsock_close(sock);
		return -errno;
	}

	ret = snprintf(path, sizeof(path), WEATHER_URI_FMT, WEATHER_KEY_ID, city,
		      WEATHER_LANGUAGE_ID);
	if (ret < 0 || (size_t)ret >= sizeof(path)) {
		shell_error(sh, "request path too long");
		zsock_close(sock);
		return -EMSGSIZE;
	}

	ret = snprintf(weather_request_buf, sizeof(weather_request_buf),
		      "GET %s HTTP/1.1\r\n"
		      "Host: %s\r\n"
		      "Connection: close\r\n"
		      "\r\n",
		      path, WEATHER_HOST);
	if (ret < 0 || (size_t)ret >= sizeof(weather_request_buf)) {
		shell_error(sh, "request too long");
		zsock_close(sock);
		return -EMSGSIZE;
	}

	ret = zsock_send(sock, weather_request_buf, strlen(weather_request_buf), 0);
	if (ret < 0) {
		shell_error(sh, "send() failed (%d)", errno);
		zsock_close(sock);
		return -errno;
	}

	while (total < out_size - 1) {
		ret = zsock_recv(sock, out + total, out_size - 1 - total, 0);
		if (ret < 0) {
			shell_error(sh, "recv() failed (%d)", errno);
			zsock_close(sock);
			return -errno;
		}
		if (ret == 0) {
			break;
		}
		total += (size_t)ret;
	}

	zsock_close(sock);
	out[total] = '\0';

	return (int)total;
}

static int cmd_pan_weather(const struct shell *sh, size_t argc, char **argv)
{
	const char *city = (argc > 1) ? argv[1] : WEATHER_CITY_ID;
	struct weather_response resp = {0};
	struct weather_result *r;
	char *body;
	int len;
	int ret;

	if (active_pan == NULL || bt_pan_get_state(active_pan) != BT_PAN_STATE_CONNECTED) {
		shell_warn(sh, "PAN link not connected yet, trying anyway...");
	}

	shell_print(sh, "Fetching weather for \"%s\" from %s ...", city, WEATHER_HOST);

	len = weather_http_get(sh, city, weather_response_buf, sizeof(weather_response_buf));
	if (len <= 0) {
		return -EIO;
	}

	if (strstr(weather_response_buf, "200") == NULL) {
		shell_warn(sh, "unexpected HTTP status line in response");
	}

	body = strstr(weather_response_buf, "\r\n\r\n");
	if (body == NULL) {
		shell_error(sh, "malformed HTTP response (no header/body separator)");
		return -EBADMSG;
	}
	body += 4; /* skip the "\r\n\r\n" itself */

	ret = json_obj_parse(body, strlen(body), weather_response_descr,
			     ARRAY_SIZE(weather_response_descr), &resp);
	if (ret < 0) {
		shell_error(sh, "failed to parse weather JSON (%d)", ret);
		return ret;
	}

	if (resp.results_len == 0) {
		shell_error(sh, "no weather data in response");
		return -ENOENT;
	}

	r = &resp.results[0];

	shell_print(sh, "Location   : %s, %s", r->location.name ? r->location.name : "?",
		   r->location.country ? r->location.country : "?");
	shell_print(sh, "Condition  : %s", r->now.text ? r->now.text : "?");
	shell_print(sh, "Temperature: %s C", r->now.temperature ? r->now.temperature : "?");
	shell_print(sh, "Updated    : %s", r->last_update ? r->last_update : "?");

	LOG_INF("weather: %s, %s C", r->now.text ? r->now.text : "?",
	       r->now.temperature ? r->now.temperature : "?");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pan_cmds,
	SHELL_CMD_ARG(status, NULL,
		      "status -- show ACL/PAN/IP status",
		      cmd_pan_status, 1, 0),
	SHELL_CMD_ARG(scan, NULL,
		      "scan -- scan and list nearby Bluetooth devices",
		      cmd_pan_scan, 1, 0),
	SHELL_CMD_ARG(connect, NULL,
		      "connect <index> -- connect to a device from the scan list",
		      cmd_pan_connect, 2, 0),
	SHELL_CMD_ARG(disconnect, NULL,
		      "disconnect -- drop the current ACL link",
		      cmd_pan_disconnect, 1, 0),
	SHELL_CMD_ARG(ping, NULL,
		      "ping -- send one ICMP echo request now",
		      cmd_pan_ping, 1, 0),
	SHELL_CMD_ARG(set_retry_flag, NULL,
		      "set_retry_flag <0|1> -- disable or enable PAN autoconnect",
		      cmd_pan_set_retry_flag, 2, 0),
	SHELL_CMD_ARG(set_retry_time, NULL,
		      "set_retry_time <count> -- set maximum PAN reconnect attempts",
		      cmd_pan_set_retry_time, 2, 0),
	SHELL_CMD_ARG(autoconnect, NULL,
		      "autoconnect -- reconnect to the last discovered PAN peer",
		      cmd_pan_autoconnect, 1, 0),
	SHELL_CMD(mac, &pan_mac_cmds,
		  "mac get|oui_reverse -- Bluetooth MAC/OUI helpers", NULL),
	SHELL_CMD_ARG(weather, NULL,
		      "weather [city] -- fetch current weather over the PAN link",
		      cmd_pan_weather, 1, 1),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pan, &pan_cmds, "Bluetooth PAN commands", NULL);
