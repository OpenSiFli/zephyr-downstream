/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT sifli_sf32lb_lcdc_mipi_dbi

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <register.h>

LOG_MODULE_REGISTER(mipi_dbi_sf32lb, CONFIG_MIPI_DBI_LOG_LEVEL);

#define LCD_COMMAND     offsetof(LCD_IF_TypeDef, COMMAND)
#define LCD_IRQ         offsetof(LCD_IF_TypeDef, IRQ)
#define LCDC_SETTING    offsetof(LCD_IF_TypeDef, SETTING)
#define LCD_CANVAS_TL_POS offsetof(LCD_IF_TypeDef, CANVAS_TL_POS)
#define LCD_CANVAS_BR_POS offsetof(LCD_IF_TypeDef, CANVAS_BR_POS)
#define LCD_CANVAS_BG   offsetof(LCD_IF_TypeDef, CANVAS_BG)
#define LCD_LAYER0_CONFIG offsetof(LCD_IF_TypeDef, LAYER0_CONFIG)
#define LCD_LAYER0_TL_POS offsetof(LCD_IF_TypeDef, LAYER0_TL_POS)
#define LCD_LAYER0_BR_POS offsetof(LCD_IF_TypeDef, LAYER0_BR_POS)
#define LCD_LAYER0_FILTER offsetof(LCD_IF_TypeDef, LAYER0_FILTER)
#define LCD_LAYER0_SRC  offsetof(LCD_IF_TypeDef, LAYER0_SRC)
#define LCD_LAYER0_FILL offsetof(LCD_IF_TypeDef, LAYER0_FILL)
#define LCD_CONF        offsetof(LCD_IF_TypeDef, LCD_CONF)
#define LCD_IF_CONF     offsetof(LCD_IF_TypeDef, LCD_IF_CONF)
#define TE_CONF         offsetof(LCD_IF_TypeDef, TE_CONF)
#define TE_CONF2        offsetof(LCD_IF_TypeDef, TE_CONF2)
#define LCD_WR          offsetof(LCD_IF_TypeDef, LCD_WR)
#define LCD_RD          offsetof(LCD_IF_TypeDef, LCD_RD)
#define LCD_SINGLE      offsetof(LCD_IF_TypeDef, LCD_SINGLE)
#define LCD_SPI_IF_CONF offsetof(LCD_IF_TypeDef, SPI_IF_CONF)
#define LCD_STATUS      offsetof(LCD_IF_TypeDef, STATUS)

#define LCD_INTF_SEL_DBI_TYPEB (0U)
#define LCD_INTF_SEL_SPI       (1U)
#define LCD_INTF_SEL_JDI       (4U)
#define LCD_INTF_SEL_DBI_TYPEA (6U)

#define SF32LB_QSPI_CMD_WRITE 0x02U
#define SF32LB_QSPI_CMD_READ  0x03U
#define SF32LB_QSPI_MEM_WRITE 0x32U
#define SF32LB_QSPI_HEADER(op, cmd) (((uint32_t)(op) << 24) | ((uint32_t)(cmd) << 8))
#define SF32LB_QSPI_READ_FREQUENCY_HZ 2000000U
#define SF32LB_QSPI_LAYER_TIMEOUT_MS 1000U
#define SF32LB_RGB565_BYTES_PER_PIXEL 2U
#define SF32LB_SINGLE_WR (LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE)
#define SF32LB_SINGLE_WD (LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE)
#define SF32LB_SINGLE_RD LCD_IF_LCD_SINGLE_RD_TRIG
#define SF32LB_LCDC_TRANSFER_IRQS                                                                 \
	(LCD_IF_IRQ_EOF_STAT | LCD_IF_IRQ_ICB_OF_STAT | LCD_IF_IRQ_EOF_RAW_STAT |               \
	 LCD_IF_IRQ_ICB_OF_RAW_STAT)

struct dbi_sf32lb_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pincfg;
	struct sf32lb_clock_dt_spec clock;
	void (*irq_config_func)(const struct device *dev);
};

struct dbi_sf32lb_data {
	struct k_mutex lock;
	struct k_sem transfer_done;
	const struct mipi_dbi_config *active_config;
	uint32_t qspi_write_display_header;
	uint16_t qspi_x0;
	uint16_t qspi_y0;
	uint16_t qspi_x1;
	uint16_t qspi_y1;
	int transfer_status;
	bool qspi_write_display_header_valid;
	bool qspi_column_valid;
	bool qspi_page_valid;
	bool transfer_pending;
};

static inline bool mipi_dbi_sf32lb_is_memory_write(uint8_t cmd)
{
	return cmd == MIPI_DCS_WRITE_MEMORY_START || cmd == MIPI_DCS_WRITE_MEMORY_CONTINUE;
}

static inline uint32_t mipi_dbi_sf32lb_bus_address(const void *ptr)
{
	uintptr_t addr = (uintptr_t)ptr;

	return HCPU_MPI_SBUS_ADDR(addr);
}

static void mipi_dbi_sf32lb_track_qspi_window(struct dbi_sf32lb_data *driver_data,
					      uint8_t cmd, const uint8_t *data, size_t len)
{
	if (data == NULL || len != 4U) {
		return;
	}

	if (cmd == MIPI_DCS_SET_COLUMN_ADDRESS) {
		driver_data->qspi_x0 = sys_get_be16(&data[0]);
		driver_data->qspi_x1 = sys_get_be16(&data[2]);
		driver_data->qspi_column_valid = true;
	} else if (cmd == MIPI_DCS_SET_PAGE_ADDRESS) {
		driver_data->qspi_y0 = sys_get_be16(&data[0]);
		driver_data->qspi_y1 = sys_get_be16(&data[2]);
		driver_data->qspi_page_valid = true;
	}
}

static void mipi_dbi_sf32lb_finish_transfer(const struct device *dev, int status)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;

	sys_clear_bits(config->base + LCDC_SETTING, LCD_IF_SETTING_EOF_MASK);

	if (!data->transfer_pending) {
		return;
	}

	data->transfer_status = status;
	data->transfer_pending = false;
	k_sem_give(&data->transfer_done);
}

static void mipi_dbi_sf32lb_isr(const struct device *dev)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t irq;

	irq = sys_read32(config->base + LCD_IRQ);
	if (irq == 0U) {
		return;
	}

	sys_write32(irq, config->base + LCD_IRQ);

	if ((irq & (LCD_IF_IRQ_ICB_OF_STAT | LCD_IF_IRQ_ICB_OF_RAW_STAT)) != 0U) {
		LOG_ERR("LCDC ICB overflow during display transfer");
		mipi_dbi_sf32lb_finish_transfer(dev, -EIO);
	} else if ((irq & (LCD_IF_IRQ_EOF_STAT | LCD_IF_IRQ_EOF_RAW_STAT)) != 0U) {
		mipi_dbi_sf32lb_finish_transfer(dev, 0);
	}
}

static inline void wait_busy(const struct device *dev)
{
	const struct dbi_sf32lb_config *config = dev->config;

	while (sys_test_bit(config->base + LCD_SINGLE, LCD_IF_LCD_SINGLE_LCD_BUSY_Pos) ||
	       sys_test_bit(config->base + LCD_STATUS, LCD_IF_STATUS_LCD_BUSY_Pos)) {
	}
}

static void mipi_dbi_sf32lb_spi_sequence(const struct device *dev, bool end)
{
	const struct dbi_sf32lb_config *config = dev->config;

	wait_busy(dev);

	if (end) {
		sys_set_bit(config->base + LCD_SPI_IF_CONF, LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS_Pos);
	} else {
		sys_clear_bit(config->base + LCD_SPI_IF_CONF,
			      LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS_Pos);
	}
}

static void mipi_dbi_sf32lb_send_single_cmd(const struct device *dev, uint32_t addr,
					    uint32_t addr_len)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t spi_if_conf;

	wait_busy(dev);

	spi_if_conf = sys_read32(config->base + LCD_SPI_IF_CONF);
	spi_if_conf &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk |
			 LCD_IF_SPI_IF_CONF_WR_LEN_Msk);

	if ((addr_len > 0) && (addr_len <= 4)) {
		spi_if_conf |= FIELD_PREP(LCD_IF_SPI_IF_CONF_WR_LEN_Msk, addr_len - 1);

		sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);
		sys_write32(addr, config->base + LCD_WR);
		sys_write32(SF32LB_SINGLE_WR, config->base + LCD_SINGLE);
	}
}

static void mipi_dbi_sf32lb_recv_single_data(const struct device *dev, uint8_t *buf, uint32_t len)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t spi_if_conf;
	uint32_t data;

	if (len == 0U) {
		return;
	}

	wait_busy(dev);

	spi_if_conf = sys_read32(config->base + LCD_SPI_IF_CONF);
	spi_if_conf &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk |
			 LCD_IF_SPI_IF_CONF_WR_LEN_Msk);

	spi_if_conf |= FIELD_PREP(LCD_IF_SPI_IF_CONF_RD_LEN_Msk, len - 1U) |
		       FIELD_PREP(LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk, 1U);

	sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);
	sys_write32(SF32LB_SINGLE_RD, config->base + LCD_SINGLE);

	wait_busy(dev);

	data = sys_read32(config->base + LCD_RD);

	for (uint32_t i = 0U; i < len; i++) {
		buf[i] = (uint8_t)(data >> (i * 8U));
	}
}

static void mipi_dbi_sf32lb_qspi_read_bytes(const struct device *dev, uint32_t addr,
					    uint32_t addr_len, uint8_t *buf, uint16_t len)
{
	wait_busy(dev);
	mipi_dbi_sf32lb_spi_sequence(dev, false);
	mipi_dbi_sf32lb_send_single_cmd(dev, addr, addr_len);
	mipi_dbi_sf32lb_spi_sequence(dev, true);
	mipi_dbi_sf32lb_recv_single_data(dev, buf, len);
}

static inline void wait_lcdc_single_busy(const struct device *dev)
{
	const struct dbi_sf32lb_config *config = dev->config;

	while (sys_test_bit(config->base + LCD_SINGLE, LCD_IF_LCD_SINGLE_LCD_BUSY_Pos)) {
	}
}

static void mipi_dbi_sf32lb_write_bytes(const struct device *dev, uint32_t addr, uint16_t addr_len,
					const uint8_t *buf, size_t len)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t spi_if_conf;

	wait_busy(dev);

	mipi_dbi_sf32lb_spi_sequence(dev, 0U == len);

	spi_if_conf = sys_read32(config->base + LCD_SPI_IF_CONF);
	spi_if_conf &= ~(LCD_IF_SPI_IF_CONF_RD_LEN_Msk | LCD_IF_SPI_IF_CONF_SPI_RD_MODE_Msk |
			 LCD_IF_SPI_IF_CONF_WR_LEN_Msk);

	spi_if_conf |= FIELD_PREP(LCD_IF_SPI_IF_CONF_WR_LEN_Msk, addr_len - 1U);

	sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);
	sys_write32(addr, config->base + LCD_WR);
	sys_write32(SF32LB_SINGLE_WR, config->base + LCD_SINGLE);

	size_t data_len = len;
	size_t total_len = len;

	while (data_len > 0) {
		uint32_t v, l;

		/* Convert 0xAA,0xBB,0xCC ->  0x00AABBCC */
		for (v = 0, l = 0; (l < 4) && (data_len > 0); l++) {
			v = (v << 8) | (*buf);
			data_len--;
			buf++;
		}

		wait_lcdc_single_busy(dev);

		total_len -= l;
		mipi_dbi_sf32lb_spi_sequence(dev, (0 == total_len));

		spi_if_conf = sys_read32(config->base + LCD_SPI_IF_CONF);
		spi_if_conf &= ~(LCD_IF_SPI_IF_CONF_WR_LEN_Msk);
		spi_if_conf |= FIELD_PREP(LCD_IF_SPI_IF_CONF_WR_LEN_Msk, l - 1U);

		sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);
		sys_write32(v, config->base + LCD_WR);
		sys_write32(SF32LB_SINGLE_WD, config->base + LCD_SINGLE);
	}
}

static int mipi_dbi_sf32lb_spi_set_frequency(const struct device *dev, uint32_t freq)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t clk_div;
	uint32_t lcdc_clk;
	uint32_t spi_if_conf;
	int ret;

	if (freq == 0U) {
		return -EINVAL;
	}

	ret = sf32lb_clock_control_get_rate_dt(&config->clock, &lcdc_clk);
	if (ret < 0) {
		return ret;
	}

	clk_div = (lcdc_clk + (freq - 1U)) / freq;
	if (clk_div < 2U) {
		clk_div = 2U;
	}
	if (clk_div > (LCD_IF_SPI_IF_CONF_CLK_DIV_Msk >> LCD_IF_SPI_IF_CONF_CLK_DIV_Pos)) {
		clk_div = LCD_IF_SPI_IF_CONF_CLK_DIV_Msk >> LCD_IF_SPI_IF_CONF_CLK_DIV_Pos;
	}

	spi_if_conf = sys_read32(config->base + LCD_SPI_IF_CONF);
	spi_if_conf &= ~LCD_IF_SPI_IF_CONF_CLK_DIV_Msk;
	spi_if_conf |= FIELD_PREP(LCD_IF_SPI_IF_CONF_CLK_DIV_Msk, clk_div);
	sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);

	return 0;
}

static int mipi_dbi_sf32lb_set_qspi_output_format(const struct device *dev,
						  const struct mipi_dbi_config *dbi_config)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint8_t color_coding = dbi_config->color_coding & 0xF0U;
	uint32_t lcd_conf;

	if (color_coding != MIPI_DBI_MODE_RGB565) {
		return -ENOTSUP;
	}

	lcd_conf = sys_read32(config->base + LCD_CONF);
	lcd_conf &= ~(LCD_IF_LCD_CONF_LCD_FORMAT_Msk | LCD_IF_LCD_CONF_AHB_FORMAT_Msk |
		      LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Msk | LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Msk |
		      LCD_IF_LCD_CONF_JDI_SER_FORMAT_Msk | LCD_IF_LCD_CONF_ENDIAN_Msk);
	lcd_conf |= FIELD_PREP(LCD_IF_LCD_CONF_SPI_LCD_FORMAT_Msk, 1U) |
		    FIELD_PREP(LCD_IF_LCD_CONF_DPI_LCD_FORMAT_Msk, 1U) |
		    LCD_IF_LCD_CONF_LCD_FORMAT_RGB565;
	sys_write32(lcd_conf, config->base + LCD_CONF);

	return 0;
}

static int mipi_dbi_sf32lb_freq_config(const struct device *dev,
				       const struct mipi_dbi_config *dbi_config)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint32_t freq = dbi_config->config.frequency;
	uint32_t lcdc_clk;
	uint32_t pw, pwl, pwh;
	uint32_t lcd_if_conf;
	int ret;

	ret = sf32lb_clock_control_get_rate_dt(&config->clock, &lcdc_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get LCDC clock rate");
		return ret;
	}

	pw = (lcdc_clk + (freq - 1)) / freq;
	pwl = pw / 2;
	pwh = pw - pwl;

	if (pwl < 1) {
		pwl = 1;
	}
	if (pwl > FIELD_GET(LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Msk)) {
		pwl = FIELD_GET(LCD_IF_LCD_IF_CONF_PWL_Msk, LCD_IF_LCD_IF_CONF_PWL_Msk);
	}

	if (pwh < 1) {
		pwh = 1;
	}
	if (pwh > FIELD_GET(LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Msk)) {
		pwh = FIELD_GET(LCD_IF_LCD_IF_CONF_PWH_Msk, LCD_IF_LCD_IF_CONF_PWH_Msk);
	}

	lcd_if_conf = sys_read32(config->base + LCD_IF_CONF);
	lcd_if_conf &= ~(LCD_IF_LCD_IF_CONF_PWL_Msk | LCD_IF_LCD_IF_CONF_PWH_Msk);
	lcd_if_conf |= FIELD_PREP(LCD_IF_LCD_IF_CONF_PWL_Msk, pwl) |
		       FIELD_PREP(LCD_IF_LCD_IF_CONF_PWH_Msk, pwh);
	sys_write32(lcd_if_conf, config->base + LCD_IF_CONF);

	return ret;
}

static int mipi_dbi_sf32lb_spi_config(const struct device *dev,
				      const struct mipi_dbi_config *dbi_config)
{
	const struct dbi_sf32lb_config *config = dev->config;
	const struct spi_config *spi_config = &dbi_config->config;
	uint8_t bus_type = dbi_config->mode & 0xFU;
	uint32_t spi_if_conf = 0;
	uint32_t freq = dbi_config->config.frequency;

	sys_clear_bits(config->base + LCD_SPI_IF_CONF, LCD_IF_SPI_IF_CONF_CLK_DIV);

	spi_if_conf = LCD_IF_SPI_IF_CONF_SPI_CS_AUTO_DIS | LCD_IF_SPI_IF_CONF_SPI_CLK_AUTO_DIS |
		      LCD_IF_SPI_IF_CONF_SPI_CS_NO_IDLE;

	if (bus_type == MIPI_DBI_MODE_QSPI) {
		spi_if_conf |= LCD_IF_SPI_IF_CONF_4LINE_4_DATA_LINE;
	} else if (bus_type == MIPI_DBI_MODE_SPI_3WIRE) {
		spi_if_conf |= LCD_IF_SPI_IF_CONF_3LINE;
	} else {
		return -ENOTSUP;
	}

	if (spi_config->operation & SPI_MODE_CPOL) {
		spi_if_conf &= ~LCD_IF_SPI_IF_CONF_SPI_CLK_INIT;
	} else {
		spi_if_conf |= LCD_IF_SPI_IF_CONF_SPI_CLK_INIT;
	}

	if (spi_config->operation & SPI_MODE_CPHA) {
		spi_if_conf |= LCD_IF_SPI_IF_CONF_SPI_CLK_POL;
	}

	sys_write32(spi_if_conf, config->base + LCD_SPI_IF_CONF);

	return mipi_dbi_sf32lb_spi_set_frequency(dev, freq);
}

static int mipi_dbi_sf32lb_configure(const struct device *dev,
				     const struct mipi_dbi_config *dbi_config)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	uint8_t bus_type = dbi_config->mode & 0xFU;
	uint32_t lcd_conf;
	uint32_t lcd_if_conf;
	int ret;

	if (dbi_config == data->active_config) {
		return 0;
	}

	lcd_conf = sys_read32(config->base + LCD_CONF);
	lcd_conf &= ~(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk | LCD_IF_LCD_CONF_TARGET_LCD_Msk);

	switch (bus_type) {
	case MIPI_DBI_MODE_8080_BUS_16_BIT:
	case MIPI_DBI_MODE_8080_BUS_9_BIT:
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
		lcd_conf |= FIELD_PREP(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk, LCD_INTF_SEL_DBI_TYPEB);
		lcd_conf |= FIELD_PREP(LCD_IF_LCD_CONF_TARGET_LCD_Msk, 0U);
		lcd_if_conf = sys_read32(config->base + LCD_IF_CONF);
		lcd_if_conf &= ~(LCD_IF_LCD_IF_CONF_TAS_Msk | LCD_IF_LCD_IF_CONF_TAH_Msk);
		lcd_if_conf |= FIELD_PREP(LCD_IF_LCD_IF_CONF_TAS_Msk, 1U) |
			       FIELD_PREP(LCD_IF_LCD_IF_CONF_TAH_Msk, 1U);
		sys_write32(lcd_if_conf, config->base + LCD_IF_CONF);
		mipi_dbi_sf32lb_freq_config(dev, dbi_config);
		break;

	case MIPI_DBI_MODE_SPI_3WIRE:
	case MIPI_DBI_MODE_QSPI:
		lcd_conf |= FIELD_PREP(LCD_IF_LCD_CONF_LCD_INTF_SEL_Msk, LCD_INTF_SEL_SPI);
		ret = mipi_dbi_sf32lb_spi_config(dev, dbi_config);
		if (ret < 0) {
			return ret;
		}
		break;

	default:
		return -EINVAL;
	}

	sys_write32(lcd_conf, config->base + LCD_CONF);
	if (bus_type == MIPI_DBI_MODE_QSPI) {
		ret = mipi_dbi_sf32lb_set_qspi_output_format(dev, dbi_config);
		if (ret < 0) {
			return ret;
		}
	}
	data->active_config = dbi_config;

	return 0;
}

static int mipi_dbi_reset_sf32lb(const struct device *dev, k_timeout_t delay)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	uint32_t delay_ms = k_ticks_to_ms_ceil32(delay.ticks);

	k_mutex_lock(&data->lock, K_FOREVER);
	sys_clear_bit(config->base + LCD_IF_CONF, LCD_IF_LCD_IF_CONF_LCD_RSTB_Pos);
	k_msleep(delay_ms);
	sys_set_bit(config->base + LCD_IF_CONF, LCD_IF_LCD_IF_CONF_LCD_RSTB_Pos);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int mipi_dbi_sf32lb_8080_cmd_write_bytes(const struct device *dev, uint8_t cmd,
						const uint8_t *data, size_t data_len)
{
	const struct dbi_sf32lb_config *config = dev->config;

	wait_busy(dev);
	sys_write32(cmd, config->base + LCD_WR);
	sys_write32(LCD_IF_LCD_SINGLE_WR_TRIG, config->base + LCD_SINGLE);

	while (data_len > 0) {
		uint8_t v = *data;

		wait_busy(dev);
		sys_write32(v, config->base + LCD_WR);
		sys_write32(LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE,
			    config->base + LCD_SINGLE);

		data_len--;
		data++;
	}

	return 0;
}

static int mipi_dbi_command_write_sf32lb(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					 const uint8_t *data, size_t len)
{
	struct dbi_sf32lb_data *driver_data = dev->data;
	uint8_t bus_type = dbi_config->mode & 0xFU;
	uint8_t qspi_op;
	uint32_t addr;
	int ret;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	ret = mipi_dbi_sf32lb_configure(dev, dbi_config);
	if (ret < 0) {
		goto out;
	}

	switch (bus_type) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
		ret = mipi_dbi_sf32lb_8080_cmd_write_bytes(dev, cmd, data, len);
		break;
	case MIPI_DBI_MODE_QSPI:
		mipi_dbi_sf32lb_track_qspi_window(driver_data, cmd, data, len);

		if (mipi_dbi_sf32lb_is_memory_write(cmd)) {
			qspi_op = SF32LB_QSPI_MEM_WRITE;
		} else {
			qspi_op = SF32LB_QSPI_CMD_WRITE;
		}

		addr = SF32LB_QSPI_HEADER(qspi_op, cmd);
		if (mipi_dbi_sf32lb_is_memory_write(cmd) && len == 0U) {
			driver_data->qspi_write_display_header = addr;
			driver_data->qspi_write_display_header_valid = true;
			ret = 0;
			break;
		}

		driver_data->qspi_write_display_header_valid = false;
		mipi_dbi_sf32lb_write_bytes(dev, addr, sizeof(addr), data, len);
		ret = 0;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

out:
	k_mutex_unlock(&driver_data->lock);

	return ret;
}

static int mipi_dbi_sf32lb_8080_cmd_read_bytes(const struct device *dev, uint8_t *cmd,
					       size_t num_cmds, uint8_t *data, size_t data_len)
{
	const struct dbi_sf32lb_config *config = dev->config;

	while (num_cmds > 0) {
		wait_busy(dev);
		sys_write32(*cmd, config->base + LCD_WR);
		sys_write32(LCD_IF_LCD_SINGLE_WR_TRIG, config->base + LCD_SINGLE);

		num_cmds--;
		cmd++;
	}

	while (data_len > 0) {
		wait_busy(dev);
		sys_write32(LCD_IF_LCD_SINGLE_RD_TRIG, config->base + LCD_SINGLE);

		wait_busy(dev);
		*data = sys_read8(config->base + LCD_RD);

		data_len--;
		data++;
	}

	return 0;
}

static int mipi_dbi_command_read_sf32lb(const struct device *dev,
					const struct mipi_dbi_config *dbi_config, uint8_t *cmds,
					size_t num_cmds, uint8_t *response, size_t len)
{
	struct dbi_sf32lb_data *driver_data = dev->data;
	uint32_t addr;
	uint8_t bus_type = dbi_config->mode & 0xFU;
	int ret;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	ret = mipi_dbi_sf32lb_configure(dev, dbi_config);
	if (ret < 0) {
		goto out;
	}

	switch (bus_type) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
		ret = mipi_dbi_sf32lb_8080_cmd_read_bytes(dev, cmds, num_cmds, response, len);
		break;
	case MIPI_DBI_MODE_QSPI:
		if (num_cmds != 1U) {
			ret = -ENOTSUP;
			break;
		}
		if (len > 4U) {
			ret = -ENOTSUP;
			break;
		}
		addr = SF32LB_QSPI_HEADER(SF32LB_QSPI_CMD_READ, cmds[0]);
		ret = mipi_dbi_sf32lb_spi_set_frequency(
			dev, MIN(dbi_config->config.frequency, SF32LB_QSPI_READ_FREQUENCY_HZ));
		if (ret < 0) {
			break;
		}
		mipi_dbi_sf32lb_qspi_read_bytes(dev, addr, sizeof(addr), response, len);
		ret = mipi_dbi_sf32lb_spi_set_frequency(dev, dbi_config->config.frequency);
		if (ret < 0) {
			break;
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

out:
	k_mutex_unlock(&driver_data->lock);

	return ret;
}

static int mipi_dbi_sf32lb_wait_transfer_done(const struct device *dev)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	int ret;

	ret = k_sem_take(&data->transfer_done, K_MSEC(SF32LB_QSPI_LAYER_TIMEOUT_MS));
	if (ret < 0) {
		sys_clear_bits(config->base + LCDC_SETTING, LCD_IF_SETTING_EOF_MASK);
		sys_write32(SF32LB_LCDC_TRANSFER_IRQS, config->base + LCD_IRQ);
		data->transfer_pending = false;
		return -ETIMEDOUT;
	}

	return data->transfer_status;
}

static int mipi_dbi_sf32lb_program_rgb565_layer(const struct device *dev,
						const uint8_t *framebuf,
						const struct display_buffer_descriptor *desc,
						uint16_t x0, uint16_t y0,
						bool byte_swap)
{
	const struct dbi_sf32lb_config *config = dev->config;
	uint64_t min_buf_size;
	uint32_t layer_line_bytes;
	uint32_t layer_config;
	uint32_t canvas_bg;
	uint32_t x1;
	uint32_t y1;
	uint32_t max_x = LCD_IF_LAYER0_BR_POS_X1_Msk >> LCD_IF_LAYER0_BR_POS_X1_Pos;
	uint32_t max_y = LCD_IF_LAYER0_BR_POS_Y1_Msk >> LCD_IF_LAYER0_BR_POS_Y1_Pos;
	uint32_t max_layer_width =
		LCD_IF_LAYER0_CONFIG_WIDTH_Msk >> LCD_IF_LAYER0_CONFIG_WIDTH_Pos;

	if (framebuf == NULL || desc == NULL) {
		return -EINVAL;
	}

	if (desc->width == 0U || desc->height == 0U || desc->pitch < desc->width) {
		return -EINVAL;
	}

	x1 = x0 + desc->width - 1U;
	y1 = y0 + desc->height - 1U;
	layer_line_bytes = desc->pitch * SF32LB_RGB565_BYTES_PER_PIXEL;
	min_buf_size =
		(((uint64_t)desc->height - 1U) * desc->pitch + desc->width) *
		SF32LB_RGB565_BYTES_PER_PIXEL;

	if (x1 > max_x || y1 > max_y || layer_line_bytes > max_layer_width ||
	    min_buf_size > desc->buf_size) {
		return -EINVAL;
	}

#if defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE)
	(void)sys_cache_data_flush_range((void *)framebuf, desc->buf_size);
#endif

	wait_busy(dev);

	canvas_bg = sys_read32(config->base + LCD_CANVAS_BG);
	canvas_bg &= ~(LCD_IF_CANVAS_BG_RED_Msk | LCD_IF_CANVAS_BG_GREEN_Msk |
		       LCD_IF_CANVAS_BG_BLUE_Msk);
	sys_write32(canvas_bg, config->base + LCD_CANVAS_BG);

	sys_write32(FIELD_PREP(LCD_IF_CANVAS_TL_POS_X0_Msk, x0) |
		    FIELD_PREP(LCD_IF_CANVAS_TL_POS_Y0_Msk, y0),
		    config->base + LCD_CANVAS_TL_POS);
	sys_write32(FIELD_PREP(LCD_IF_CANVAS_BR_POS_X1_Msk, x1) |
		    FIELD_PREP(LCD_IF_CANVAS_BR_POS_Y1_Msk, y1),
		    config->base + LCD_CANVAS_BR_POS);

	layer_config = LCD_IF_LAYER0_CONFIG_FORMAT_RGB565 |
		       FIELD_PREP(LCD_IF_LAYER0_CONFIG_ALPHA_Msk, 255U) |
		       FIELD_PREP(LCD_IF_LAYER0_CONFIG_WIDTH_Msk, layer_line_bytes) |
		       LCD_IF_LAYER0_CONFIG_PREFETCH_EN | LCD_IF_LAYER0_CONFIG_ACTIVE;

	sys_write32(layer_config, config->base + LCD_LAYER0_CONFIG);
	sys_write32(FIELD_PREP(LCD_IF_LAYER0_TL_POS_X0_Msk, x0) |
		    FIELD_PREP(LCD_IF_LAYER0_TL_POS_Y0_Msk, y0),
		    config->base + LCD_LAYER0_TL_POS);
	sys_write32(FIELD_PREP(LCD_IF_LAYER0_BR_POS_X1_Msk, x1) |
		    FIELD_PREP(LCD_IF_LAYER0_BR_POS_Y1_Msk, y1),
		    config->base + LCD_LAYER0_BR_POS);
	sys_write32(0U, config->base + LCD_LAYER0_FILTER);
	sys_write32(mipi_dbi_sf32lb_bus_address(framebuf), config->base + LCD_LAYER0_SRC);
	sys_write32(byte_swap ? LCD_IF_LAYER0_FILL_ENDIAN : 0U, config->base + LCD_LAYER0_FILL);

	return 0;
}

static int mipi_dbi_sf32lb_qspi_send_layer(const struct device *dev, uint32_t addr,
					   const uint8_t *framebuf,
					   struct display_buffer_descriptor *desc,
					   uint16_t x0, uint16_t y0, bool byte_swap)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	int ret;

	wait_busy(dev);
	mipi_dbi_sf32lb_spi_sequence(dev, false);
	mipi_dbi_sf32lb_send_single_cmd(dev, addr, sizeof(addr));
	mipi_dbi_sf32lb_spi_sequence(dev, true);

	ret = mipi_dbi_sf32lb_program_rgb565_layer(dev, framebuf, desc, x0, y0, byte_swap);
	if (ret < 0) {
		return ret;
	}

	k_sem_reset(&data->transfer_done);
	data->transfer_status = 0;
	sys_write32(SF32LB_LCDC_TRANSFER_IRQS, config->base + LCD_IRQ);

	data->transfer_pending = true;
	sys_set_bits(config->base + LCDC_SETTING, LCD_IF_SETTING_EOF_MASK);
	sys_write32(LCD_IF_COMMAND_START, config->base + LCD_COMMAND);

	return mipi_dbi_sf32lb_wait_transfer_done(dev);
}

static int mipi_dbi_sf32lb_qspi_write_display(const struct device *dev,
					      const uint8_t *framebuf,
					      struct display_buffer_descriptor *desc,
					      enum display_pixel_format pixfmt)
{
	struct dbi_sf32lb_data *data = dev->data;
	uint32_t addr;
	uint32_t window_width;
	uint32_t window_height;
	uint16_t x0 = 0U;
	uint16_t y0 = 0U;
	bool byte_swap;

	if (pixfmt == PIXEL_FORMAT_RGB_565) {
		byte_swap = false;
	} else if (pixfmt == PIXEL_FORMAT_RGB_565X) {
		byte_swap = true;
	} else {
		return -ENOTSUP;
	}

	addr = data->qspi_write_display_header_valid ?
		data->qspi_write_display_header :
		SF32LB_QSPI_HEADER(SF32LB_QSPI_MEM_WRITE, MIPI_DCS_WRITE_MEMORY_START);
	data->qspi_write_display_header_valid = false;

	if (data->qspi_column_valid && data->qspi_page_valid) {
		if (data->qspi_x1 < data->qspi_x0 || data->qspi_y1 < data->qspi_y0) {
			return -EINVAL;
		}

		window_width = data->qspi_x1 - data->qspi_x0 + 1U;
		window_height = data->qspi_y1 - data->qspi_y0 + 1U;
		if (window_width != desc->width || window_height != desc->height) {
			return -EINVAL;
		}

		x0 = data->qspi_x0;
		y0 = data->qspi_y0;
	}

	return mipi_dbi_sf32lb_qspi_send_layer(dev, addr, framebuf, desc, x0, y0, byte_swap);
}

static int mipi_dbi_write_display_sf32lb(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config,
					 const uint8_t *framebuf,
					 struct display_buffer_descriptor *desc,
					 enum display_pixel_format pixfmt)
{
	const struct dbi_sf32lb_config *config;
	struct dbi_sf32lb_data *driver_data = dev->data;
	uint32_t data_len;
	const uint8_t *buf;
	uint8_t bus_type = dbi_config->mode & 0xFU;
	int ret;

	if (framebuf == NULL || desc == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	ret = mipi_dbi_sf32lb_configure(dev, dbi_config);
	if (ret < 0) {
		goto out;
	}

	if (bus_type == MIPI_DBI_MODE_QSPI) {
		ret = mipi_dbi_sf32lb_qspi_write_display(dev, framebuf, desc, pixfmt);
		goto out;
	}

	ARG_UNUSED(pixfmt);
	config = dev->config;
	data_len = desc->buf_size;
	buf = framebuf;

	if (data_len == 0U) {
		ret = 0;
		goto out;
	}

	if (bus_type != MIPI_DBI_MODE_8080_BUS_16_BIT &&
	    bus_type != MIPI_DBI_MODE_8080_BUS_9_BIT &&
	    bus_type != MIPI_DBI_MODE_8080_BUS_8_BIT) {
		ret = -ENOTSUP;
		goto out;
	}

	while (data_len > 0) {
		wait_busy(dev);

		if (bus_type == MIPI_DBI_MODE_8080_BUS_16_BIT && data_len >= 2) {
			uint16_t v = sys_get_le16(buf);

			sys_write32(v, config->base + LCD_WR);
			sys_write32(LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE,
				    config->base + LCD_SINGLE);
			buf += 2;
			data_len -= 2;
		} else {
			sys_write32(*buf, config->base + LCD_WR);
			sys_write32(LCD_IF_LCD_SINGLE_WR_TRIG | LCD_IF_LCD_SINGLE_TYPE,
				    config->base + LCD_SINGLE);
			buf++;
			data_len--;
		}
	}

	ret = 0;

out:
	k_mutex_unlock(&driver_data->lock);

	return ret;
}

static int mipi_dbi_configure_te_sf32lb(const struct device *dev, uint8_t edge, k_timeout_t delay)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	uint32_t delay_us = k_ticks_to_us_ceil32(delay.ticks);
	uint32_t te_conf;
	uint32_t polarity;

	if (edge == MIPI_DBI_TE_RISING_EDGE) {
		polarity = 1U;
	} else if (edge == MIPI_DBI_TE_FALLING_EDGE) {
		polarity = 0U;
	} else {
		return -EINVAL;
	}

	te_conf = FIELD_PREP(LCD_IF_TE_CONF_ENABLE_Msk, 1) |
		  FIELD_PREP(LCD_IF_TE_CONF_FMARK_POL_Msk, polarity) |
		  FIELD_PREP(LCD_IF_TE_CONF_MODE_Msk, 0);

	k_mutex_lock(&data->lock, K_FOREVER);
	sys_write32(delay_us, config->base + TE_CONF2);
	sys_write32(te_conf, config->base + TE_CONF);
	k_mutex_unlock(&data->lock);

	return 0;
}

static DEVICE_API(mipi_dbi, dbi_sf32lb_api) = {
	.reset = mipi_dbi_reset_sf32lb,
	.command_write = mipi_dbi_command_write_sf32lb,
	.command_read = mipi_dbi_command_read_sf32lb,
	.write_display = mipi_dbi_write_display_sf32lb,
	.configure_te = mipi_dbi_configure_te_sf32lb,
};

static int mipi_dbi_init_sf32lb(const struct device *dev)
{
	const struct dbi_sf32lb_config *config = dev->config;
	struct dbi_sf32lb_data *data = dev->data;
	int err;

	k_mutex_init(&data->lock);
	k_sem_init(&data->transfer_done, 0, 1);

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	err = sf32lb_clock_control_on_dt(&config->clock);
	if (err < 0) {
		return err;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", err);
		return err;
	}

	sys_set_bit(config->base + LCDC_SETTING, LCD_IF_SETTING_AUTO_GATE_EN_Pos);
	sys_clear_bits(config->base + LCDC_SETTING, LCD_IF_SETTING_EOF_MASK);
	sys_write32(SF32LB_LCDC_TRANSFER_IRQS, config->base + LCD_IRQ);
	sys_set_bit(config->base + LCD_IF_CONF, LCD_IF_LCD_IF_CONF_LCD_RSTB_Pos);
	config->irq_config_func(dev);

	return err;
}

#define DBI_SF32LB_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct dbi_sf32lb_data dbi_sf32lb_data_##n;                                         \
                                                                                                   \
	static void dbi_sf32lb_irq_config_func_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority),       \
			    mipi_dbi_sf32lb_isr, DEVICE_DT_INST_GET(n), 0);                        \
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));                                           \
	}                                                                                          \
                                                                                                   \
	static const struct dbi_sf32lb_config dbi_sf32lb_config_##n = {                            \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),                                            \
		.clock = SF32LB_CLOCK_DT_INST_PARENT_SPEC_GET(n),                                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.irq_config_func = dbi_sf32lb_irq_config_func_##n,                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mipi_dbi_init_sf32lb, NULL, &dbi_sf32lb_data_##n,                 \
			      &dbi_sf32lb_config_##n, POST_KERNEL, CONFIG_MIPI_DBI_INIT_PRIORITY,  \
			      &dbi_sf32lb_api);                                                    \
	BUILD_ASSERT((DT_CHILD_NUM_STATUS_OKAY(DT_INST_PARENT(n)) == 1),                           \
		"LCDC only supports one operating mode");

DT_INST_FOREACH_STATUS_OKAY(DBI_SF32LB_DEFINE)
