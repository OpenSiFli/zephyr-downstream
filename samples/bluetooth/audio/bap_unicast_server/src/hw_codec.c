/*
 * Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/audio/codec.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "hw_codec.h"

LOG_MODULE_REGISTER(codec, CONFIG_LOG_DEFAULT_LEVEL);

#define CODEC_MAX_BLOCK_SIZE 960U
#define RING_BUF_SIZE        (CODEC_MAX_BLOCK_SIZE * 20U)
#define SPEAKER_VOL          15U

static uint8_t ring_buffer[RING_BUF_SIZE];
static uint8_t block_data[CODEC_MAX_BLOCK_SIZE];
static struct ring_buf ring_buf;
static const struct device *codec_dev;
static uint32_t codec_block_size;
static bool codec_configured;

static void tx_done(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (ring_buf_size_get(&ring_buf) < codec_block_size) {
		return;
	}

	if (ring_buf_get(&ring_buf, block_data, codec_block_size) != codec_block_size) {
		return;
	}

	if (audio_codec_write(dev, block_data, codec_block_size) != 0) {
		LOG_WRN("Failed to write PCM block to audio codec");
	}
}

int hw_codec_open(void)
{
	codec_dev = DEVICE_DT_GET(DT_ALIAS(codec0));
	if (!device_is_ready(codec_dev)) {
		LOG_ERR("Codec device is not ready");
		return -ENODEV;
	}

	ring_buf_init(&ring_buf, sizeof(ring_buffer), ring_buffer);
	return 0;
}

int hw_codec_cfg(uint32_t samplerate, uint32_t block_size)
{
	const audio_property_value_t volume = {.vol = SPEAKER_VOL};
	struct audio_codec_cfg cfg = {
		.dai_type = AUDIO_DAI_TYPE_PCM,
		.dai_cfg.pcm.dir = AUDIO_DAI_DIR_TX,
		.dai_cfg.pcm.pcm_width = AUDIO_PCM_WIDTH_16_BITS,
		.dai_cfg.pcm.channels = 1U,
		.dai_cfg.pcm.block_size = block_size,
		.dai_cfg.pcm.samplerate = samplerate,
	};
	int err;

	if (block_size == 0U || block_size > CODEC_MAX_BLOCK_SIZE) {
		return -EINVAL;
	}
	if (codec_configured) {
		return -EALREADY;
	}

	codec_block_size = block_size;
	audio_codec_register_done_callback(codec_dev, tx_done, NULL, NULL, NULL);
	err = audio_codec_configure(codec_dev, &cfg);
	if (err != 0) {
		return err;
	}

	err = audio_codec_start(codec_dev, AUDIO_DAI_DIR_TX);
	if (err != 0) {
		return err;
	}

	err = audio_codec_set_property(codec_dev, AUDIO_PROPERTY_OUTPUT_VOLUME, 0, volume);
	if (err != 0) {
		(void)audio_codec_stop(codec_dev, AUDIO_DAI_DIR_TX);
		return err;
	}

	codec_configured = true;
	LOG_INF("PCM playback: %u Hz, %u-byte blocks", samplerate, block_size);
	return 0;
}

uint32_t hw_codec_write_data(const uint8_t *data, uint32_t len)
{
	return ring_buf_put(&ring_buf, data, len);
}

int hw_codec_close(void)
{
	int err;

	if (!codec_configured) {
		return 0;
	}

	err = audio_codec_stop(codec_dev, AUDIO_DAI_DIR_TX);
	if (err == 0) {
		ring_buf_reset(&ring_buf);
		codec_configured = false;
	}

	return err;
}