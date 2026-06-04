/*
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT chipone_co5300_qspi

#include <zephyr/device.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(display_co5300_qspi, CONFIG_DISPLAY_LOG_LEVEL);

#define CO5300_LCD_ID              0x331100U

#define CO5300_CMD_PAGE_SWITCH     0xFEU
#define CO5300_CMD_UNLOCK_1        0xF4U
#define CO5300_CMD_UNLOCK_2        0xF5U
#define CO5300_CMD_SET_SPI_MODE    0xC4U
#define CO5300_CMD_HBM_BRIGHTNESS  0x63U

#define CO5300_PAGE_PASSWORD       0x20U
#define CO5300_PASSWORD_UNLOCK_1   0x5AU
#define CO5300_PASSWORD_UNLOCK_2   0x59U
#define CO5300_PASSWORD_LOCK       0xA5U
#define CO5300_SPI_QSPI_MODE       0x80U
#define CO5300_CONTROL_DISPLAY     0x20U
#define CO5300_BRIGHTNESS_MAX      0xFFU
#define CO5300_TE_VBLANK           0x00U
#define CO5300_PIXEL_SIZE_RGB565   2U
#define CO5300_PIXEL_FORMAT        PIXEL_FORMAT_RGB_565

struct co5300_qspi_config {
	const struct device *mipi_dbi;
	struct mipi_dbi_config dbi_config;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec avdd_enable_gpio;
	uint16_t width;
	uint16_t height;
	uint8_t te_mode;
	uint32_t te_delay;
};

struct co5300_qspi_data {
	enum display_pixel_format pixel_format;
	bool blanking_on;
};

static int co5300_qspi_command_write(const struct device *dev, uint8_t cmd, const uint8_t *data,
				     size_t len)
{
	const struct co5300_qspi_config *config = dev->config;

	return mipi_dbi_command_write(config->mipi_dbi, &config->dbi_config, cmd, data, len);
}

static int co5300_qspi_command_write_u8(const struct device *dev, uint8_t cmd, uint8_t value)
{
	return co5300_qspi_command_write(dev, cmd, &value, sizeof(value));
}

static int co5300_qspi_command_read(const struct device *dev, uint8_t cmd, uint8_t *data,
				    size_t len)
{
	const struct co5300_qspi_config *config = dev->config;

	return mipi_dbi_command_read(config->mipi_dbi, &config->dbi_config, &cmd, sizeof(cmd), data,
				     len);
}

static int co5300_qspi_set_window(const struct device *dev, uint16_t x0, uint16_t y0,
				  uint16_t x1, uint16_t y1)
{
	uint8_t caset[4];
	uint8_t raset[4];
	int ret;

	sys_put_be16(x0, &caset[0]);
	sys_put_be16(x1, &caset[2]);
	sys_put_be16(y0, &raset[0]);
	sys_put_be16(y1, &raset[2]);

	ret = co5300_qspi_command_write(dev, MIPI_DCS_SET_COLUMN_ADDRESS, caset, sizeof(caset));
	if (ret < 0) {
		return ret;
	}

	return co5300_qspi_command_write(dev, MIPI_DCS_SET_PAGE_ADDRESS, raset, sizeof(raset));
}

static int co5300_qspi_blanking_on(const struct device *dev)
{
	struct co5300_qspi_data *data = dev->data;
	int ret;

	ret = co5300_qspi_command_write(dev, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data->blanking_on = true;

	return 0;
}

static int co5300_qspi_blanking_off(const struct device *dev)
{
	struct co5300_qspi_data *data = dev->data;
	int ret;

	ret = co5300_qspi_command_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data->blanking_on = false;

	return 0;
}

static int co5300_qspi_write(const struct device *dev, const uint16_t x, const uint16_t y,
			     const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct co5300_qspi_config *config = dev->config;
	struct co5300_qspi_data *data = dev->data;
	struct display_buffer_descriptor write_desc;
	uint64_t min_buf_size;
	int ret;

	if (buf == NULL || desc == NULL) {
		return -EINVAL;
	}

	if (data->pixel_format != CO5300_PIXEL_FORMAT) {
		return -ENOTSUP;
	}

	if (desc->width == 0U || desc->height == 0U || desc->pitch < desc->width) {
		return -EINVAL;
	}

	if (x >= config->width || y >= config->height || desc->width > (config->width - x) ||
	    desc->height > (config->height - y)) {
		return -EINVAL;
	}

	min_buf_size =
		(((uint64_t)desc->height - 1U) * desc->pitch + desc->width) *
		CO5300_PIXEL_SIZE_RGB565;
	if (min_buf_size > desc->buf_size) {
		return -EINVAL;
	}

	ret = co5300_qspi_set_window(dev, x, y, x + desc->width - 1U,
				     y + desc->height - 1U);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write(dev, MIPI_DCS_WRITE_MEMORY_START, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	write_desc = *desc;
	write_desc.buf_size = min_buf_size;

	return mipi_dbi_write_display(config->mipi_dbi, &config->dbi_config, buf, &write_desc,
				      data->pixel_format);
}

static int co5300_qspi_read(const struct device *dev, const uint16_t x, const uint16_t y,
			    const struct display_buffer_descriptor *desc, void *buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(desc);
	ARG_UNUSED(buf);

	return -ENOTSUP;
}

static void co5300_qspi_get_capabilities(const struct device *dev,
					 struct display_capabilities *capabilities)
{
	const struct co5300_qspi_config *config = dev->config;
	struct co5300_qspi_data *data = dev->data;

	memset(capabilities, 0, sizeof(*capabilities));

	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->supported_pixel_formats = CO5300_PIXEL_FORMAT;
	capabilities->current_pixel_format = data->pixel_format;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int co5300_qspi_set_pixel_format(const struct device *dev,
					const enum display_pixel_format pixel_format)
{
	struct co5300_qspi_data *data = dev->data;
	uint8_t value = MIPI_DCS_PIXEL_FORMAT_16BIT;
	int ret;

	if (pixel_format != CO5300_PIXEL_FORMAT) {
		return -ENOTSUP;
	}

	ret = co5300_qspi_command_write(dev, MIPI_DCS_SET_PIXEL_FORMAT, &value, sizeof(value));
	if (ret < 0) {
		return ret;
	}

	data->pixel_format = pixel_format;

	return 0;
}

static int co5300_qspi_set_brightness(const struct device *dev, const uint8_t brightness)
{
	return co5300_qspi_command_write(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, &brightness,
					 sizeof(brightness));
}

static int co5300_qspi_set_contrast(const struct device *dev, const uint8_t contrast)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(contrast);

	return -ENOTSUP;
}

static int co5300_qspi_set_orientation(const struct device *dev,
				       const enum display_orientation orientation)
{
	ARG_UNUSED(dev);

	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}

	return -ENOTSUP;
}

static int co5300_qspi_power_on(const struct device *dev)
{
	const struct co5300_qspi_config *config = dev->config;
	int ret;

	if (config->avdd_enable_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->avdd_enable_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->avdd_enable_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}

		k_msleep(10);

		ret = gpio_pin_set_dt(&config->reset_gpio, 1);
		if (ret < 0) {
			return ret;
		}

		k_msleep(10);

		ret = gpio_pin_set_dt(&config->reset_gpio, 0);
		if (ret < 0) {
			return ret;
		}

		k_msleep(50);
	}

	return 0;
}

static int co5300_qspi_configure_panel(const struct device *dev)
{
	const struct co5300_qspi_config *config = dev->config;
	int ret;

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_PAGE_SWITCH, CO5300_PAGE_PASSWORD);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_UNLOCK_1, CO5300_PASSWORD_UNLOCK_1);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_UNLOCK_2, CO5300_PASSWORD_UNLOCK_2);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_PAGE_SWITCH, CO5300_PAGE_PASSWORD);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_UNLOCK_1, CO5300_PASSWORD_LOCK);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_UNLOCK_2, CO5300_PASSWORD_LOCK);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_PAGE_SWITCH, 0x00U);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_SET_SPI_MODE, CO5300_SPI_QSPI_MODE);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_set_pixel_format(dev, CO5300_PIXEL_FORMAT);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, MIPI_DCS_SET_TEAR_ON, CO5300_TE_VBLANK);
	if (ret < 0) {
		return ret;
	}

	if (config->te_mode != MIPI_DBI_TE_NO_EDGE) {
		ret = mipi_dbi_configure_te(config->mipi_dbi, config->te_mode, config->te_delay);
		if (ret < 0) {
			return ret;
		}
	}

	ret = co5300_qspi_command_write_u8(dev, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					   CO5300_CONTROL_DISPLAY);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write_u8(dev, CO5300_CMD_HBM_BRIGHTNESS, CO5300_BRIGHTNESS_MAX);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_set_brightness(dev, CO5300_BRIGHTNESS_MAX);
	if (ret < 0) {
		return ret;
	}

	return co5300_qspi_set_window(dev, 0U, 0U, config->width - 1U, config->height - 1U);
}

static int co5300_qspi_init(const struct device *dev)
{
	const struct co5300_qspi_config *config = dev->config;
	struct co5300_qspi_data *data = dev->data;
	uint8_t id[3];
	uint32_t display_id;
	int ret;

	if (!device_is_ready(config->mipi_dbi)) {
		return -ENODEV;
	}

	ret = co5300_qspi_power_on(dev);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_read(dev, MIPI_DCS_GET_DISPLAY_ID, id, sizeof(id));
	if (ret < 0) {
		LOG_ERR("Failed to read display ID: %d", ret);
		return ret;
	}

	display_id = ((uint32_t)id[2] << 16) | ((uint32_t)id[1] << 8) | id[0];
	if (display_id != CO5300_LCD_ID) {
		LOG_ERR("Unexpected display ID: 0x%06x (raw: %02x %02x %02x)", display_id,
			id[0], id[1], id[2]);
		return -ENODEV;
	}

	ret = co5300_qspi_configure_panel(dev);
	if (ret < 0) {
		return ret;
	}

	ret = co5300_qspi_command_write(dev, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	k_msleep(120);

	ret = co5300_qspi_command_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data->blanking_on = false;

	return 0;
}

static DEVICE_API(display, co5300_qspi_api) = {
	.blanking_on = co5300_qspi_blanking_on,
	.blanking_off = co5300_qspi_blanking_off,
	.write = co5300_qspi_write,
	.read = co5300_qspi_read,
	.get_capabilities = co5300_qspi_get_capabilities,
	.set_pixel_format = co5300_qspi_set_pixel_format,
	.set_brightness = co5300_qspi_set_brightness,
	.set_contrast = co5300_qspi_set_contrast,
	.set_orientation = co5300_qspi_set_orientation,
};

#define CO5300_QSPI_DEFINE(inst)                                                                   \
	BUILD_ASSERT(DT_INST_STRING_UPPER_TOKEN(inst, mipi_mode) == MIPI_DBI_MODE_QSPI,            \
		     "CO5300 QSPI requires mipi-mode = \"MIPI_DBI_MODE_QSPI\"");                   \
	static struct co5300_qspi_data co5300_qspi_data_##inst = {                                  \
		.pixel_format = CO5300_PIXEL_FORMAT,                                               \
	};                                                                                         \
	static const struct co5300_qspi_config co5300_qspi_config_##inst = {                        \
		.mipi_dbi = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                  \
		.dbi_config = {                                                                   \
			.mode = DT_INST_STRING_UPPER_TOKEN(inst, mipi_mode),                      \
			.color_coding = MIPI_DBI_MODE_RGB565,                                    \
			.config = {                                                               \
				.frequency = DT_INST_PROP(inst, mipi_max_frequency),             \
				.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),               \
				.slave = DT_INST_REG_ADDR(inst),                                 \
			},                                                                        \
		},                                                                                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                   \
		.avdd_enable_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, avdd_enable_gpios, {0}),       \
		.width = DT_INST_PROP(inst, width),                                               \
		.height = DT_INST_PROP(inst, height),                                             \
		.te_mode = MIPI_DBI_TE_MODE_DT_INST(inst, te_mode),                               \
		.te_delay = DT_INST_PROP(inst, te_delay),                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, co5300_qspi_init, NULL, &co5300_qspi_data_##inst,              \
			      &co5300_qspi_config_##inst, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, \
			      &co5300_qspi_api);

DT_INST_FOREACH_STATUS_OKAY(CO5300_QSPI_DEFINE)
