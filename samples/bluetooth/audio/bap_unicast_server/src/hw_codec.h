/*
 * Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAMPLE_BAP_UNICAST_SERVER_HW_CODEC_H
#define SAMPLE_BAP_UNICAST_SERVER_HW_CODEC_H

#include <stdint.h>

int hw_codec_open(void);
int hw_codec_cfg(uint32_t samplerate, uint32_t block_size);
uint32_t hw_codec_write_data(const uint8_t *data, uint32_t len);
int hw_codec_close(void);

#endif /* SAMPLE_BAP_UNICAST_SERVER_HW_CODEC_H */