/*
 * Copyright 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_mpi_qspi_nor

#include "jesd216.h"
#include "spi_nor.h"

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/dma/sf32lb.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/cache.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include <ll_mpi.h>

/*
 * NOTE: it is easy to get into a _race_ when XIP from QSPI NOR and trying
 * to do other operations with it. For this reason, most of this driver resides
 * in RAM, so that no data fetches are required when dealing with the flash.
 */

#define QSPI_NOR_MAX_3B_SIZE 0x1000000UL

#define MPI_FIFO_SIZE 64U


#define MPI_CCRX_IMODE_SINGLE  FIELD_PREP(MPI_CCR1_IMODE_Msk, 1U)
#define MPI_CCRX_ADMODE_NONE   FIELD_PREP(MPI_CCR1_ADMODE_Msk, 0U)
#define MPI_CCRX_ADMODE_SINGLE FIELD_PREP(MPI_CCR1_ADMODE_Msk, 1U)
#define MPI_CCRX_ADMODE_QUAD   FIELD_PREP(MPI_CCR1_ADMODE_Msk, 3U)
#define MPI_CCRX_ADSIZE_N(n)   FIELD_PREP(MPI_CCR1_ADSIZE_Msk, (n) - 1U)
#define MPI_CCRX_ABMODE_NONE   FIELD_PREP(MPI_CCR1_ABMODE_Msk, 0U)
#define MPI_CCRX_ABMODE_SINGLE FIELD_PREP(MPI_CCR1_ABMODE_Msk, 1U)
#define MPI_CCRX_ABMODE_QUAD   FIELD_PREP(MPI_CCR1_ABMODE_Msk, 3U)
#define MPI_CCRX_ABSIZE_N(n)   FIELD_PREP(MPI_CCR1_ABSIZE_Msk, (n) - 1U)
#define MPI_CCRX_DCYC_N(n)     FIELD_PREP(MPI_CCR1_DCYC_Msk, (n))
#define MPI_CCRX_DMODE_NONE    FIELD_PREP(MPI_CCR1_DMODE_Msk, 0U)
#define MPI_CCRX_DMODE_SINGLE  FIELD_PREP(MPI_CCR1_DMODE_Msk, 1U)
#define MPI_CCRX_DMODE_QUAD    FIELD_PREP(MPI_CCR1_DMODE_Msk, 3U)
#define MPI_CCRX_FMODE_READ    FIELD_PREP(MPI_CCR1_FMODE_Msk, 0U)
#define MPI_CCRX_FMODE_WRITE   FIELD_PREP(MPI_CCR1_FMODE_Msk, 1U)

#define MPI_CCRX_CMD_WREN  MPI_CCRX_IMODE_SINGLE
#define MPI_CCRX_CMD_RDSR  (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_RDSR2 (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_WRSR  (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_WRSR2 (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_CE    MPI_CCRX_IMODE_SINGLE
#define MPI_CCRX_CMD_BE_SE (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(3U))
#define MPI_CCRX_CMD_BE_SE_4B                                                                      \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(4U))
#define MPI_CCRX_CMD_4READ_4B                                                                      \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_QUAD | MPI_CCRX_ADSIZE_N(4U) |                    \
	 MPI_CCRX_ABMODE_QUAD | MPI_CCRX_ABSIZE_N(1U) | MPI_CCRX_DCYC_N(4U) | MPI_CCRX_DMODE_QUAD)
#define MPI_CCRX_CMD_READ_FAST_4B                                                                  \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(4U) |                  \
	 MPI_CCRX_DCYC_N(8U) | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_4READ                                                                         \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_QUAD | MPI_CCRX_ADSIZE_N(3U) |                    \
	 MPI_CCRX_ABMODE_QUAD | MPI_CCRX_ABSIZE_N(1U) | MPI_CCRX_DCYC_N(4U) | MPI_CCRX_DMODE_QUAD)
#define MPI_CCRX_CMD_READ_FAST                                                                     \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(3U) |                  \
	 MPI_CCRX_DCYC_N(8U) | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_PP_1_1_4_4B                                                                   \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(4U) |                  \
	 MPI_CCRX_DMODE_QUAD | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_PP_1_1_4                                                                      \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(3U) |                  \
	 MPI_CCRX_DMODE_QUAD | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_PP_4B                                                                         \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(4U) |                  \
	 MPI_CCRX_DMODE_SINGLE | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_PP                                                                            \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(3U) |                  \
	 MPI_CCRX_DMODE_SINGLE | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_READ_SFDP                                                                     \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_SINGLE | MPI_CCRX_ADSIZE_N(3U) |                  \
	 MPI_CCRX_DCYC_N(8U) | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_RDID (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE)

static const struct flash_parameters flash_nor_parameters = {
	.write_block_size = 1U,
	.erase_value = 0xFFU,
};

struct flash_sf32lb_mpi_qspi_nor_config {
	struct flash_pages_layout layout;
};

struct flash_sf32lb_mpi_qspi_nor_data {
	MPI_TypeDef *mpi;
	uintptr_t base;
	uint32_t size;
	struct sf32lb_clock_dt_spec clock;
	struct sf32lb_dma_dt_spec dma;
	uint8_t write_buf[SPI_NOR_PAGE_SIZE] __aligned(4);
	uint8_t lines;
	uint8_t psclr;
	bool invert_rx_clk;
	uint8_t qer;
	uint8_t addr_len;
	uint8_t cmd_read;
	uint32_t ccrx_read;
	uint8_t cmd_pp;
	uint32_t ccrx_pp;
	uint8_t cmd_be;
	uint8_t cmd_be32;
	uint8_t cmd_se;
	uint32_t ccrx_be_se;
	struct k_spinlock lock;
};

static __ramfunc void *qspi_nor_memcpy(void *dst, const void *src, size_t len)
{
	const uint8_t *csrc = src;
	uint8_t *cdst = dst;

	while (len--) {
		*cdst++ = *csrc++;
	}

	return dst;
}

static __ramfunc void qspi_nor_cinstr(const struct device *dev, uint8_t cmd)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;

	/* single-line instruction-only read */
	ll_mpi_write_command_config(data->mpi, LL_MPI_CS_1, MPI_CCRX_IMODE_SINGLE);

	/* send command and wait for completion */
	ll_mpi_set_command_byte(data->mpi, LL_MPI_CS_1, cmd);

	while (!ll_mpi_get_transfer_complete_flag(data->mpi)) {
	}

	ll_mpi_clear_transfer_complete_flag(data->mpi);
}

static __ramfunc void qspi_nor_cinstr_seq_ready_wait(const struct device *dev, uint8_t cmd,
						     uint32_t ccrx, uint32_t addr)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;

	/* configure CMD2 with status match */
	ll_mpi_write_command_config(data->mpi, LL_MPI_CS_2, MPI_CCRX_CMD_RDSR);
	ll_mpi_set_command_byte(data->mpi, LL_MPI_CS_2, SPI_NOR_CMD_RDSR);
	ll_mpi_set_data_length(data->mpi, LL_MPI_CS_2, 1U);
	ll_mpi_set_status_match_mask(data->mpi, SPI_NOR_MEM_RDY_MASK);
	ll_mpi_set_status_match_value(data->mpi, SPI_NOR_MEM_RDY_MATCH);

	ll_mpi_enable_command2(data->mpi);
	ll_mpi_set_status_match_enable(data->mpi, LL_MPI_CS_2, 1U);

	/* issue CMD1, wait for status match */
	ll_mpi_set_address(data->mpi, LL_MPI_CS_1, addr);

	ll_mpi_write_command_config(data->mpi, LL_MPI_CS_1, ccrx);
	ll_mpi_set_command_byte(data->mpi, LL_MPI_CS_1, cmd);
	while (!ll_mpi_get_status_match_flag(data->mpi)) {
	}

	ll_mpi_clear_status_match_flag(data->mpi);
	ll_mpi_clear_transfer_complete_flag(data->mpi);

	/* disable CMD2 and status match 2 */
	ll_mpi_disable_command2(data->mpi);
	ll_mpi_set_status_match_enable(data->mpi, LL_MPI_CS_2, 0U);
}

static __ramfunc void qspi_nor_read_fifo(const struct device *dev, uint8_t cmd, uint32_t ccrx,
					 uint32_t addr, void *buf, size_t len)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;
	uint8_t *cbuf = buf;

	/* configure command */
	ll_mpi_write_command_config(data->mpi, LL_MPI_CS_1, ccrx);

	/* read in FIFO max sized chunks */
	do {
		size_t chunk_len = MIN(len, MPI_FIFO_SIZE);

		/* write length, address */
		ll_mpi_set_data_length(data->mpi, LL_MPI_CS_1, chunk_len);
		ll_mpi_set_address(data->mpi, LL_MPI_CS_1, addr);

		/* send command and wait for completion */
		ll_mpi_set_command_byte(data->mpi, LL_MPI_CS_1, cmd);
		while (!ll_mpi_get_transfer_complete_flag(data->mpi)) {
		}
		ll_mpi_clear_transfer_complete_flag(data->mpi);

		/* grab data */
		for (size_t i = 0U; i < (chunk_len / 4U); i++) {
			uint32_t dr = ll_mpi_read_data(data->mpi);

			qspi_nor_memcpy(&cbuf[i * 4U], &dr, 4U);
		}

		if (chunk_len & 3U) {
			uint32_t dr = ll_mpi_read_data(data->mpi);

			qspi_nor_memcpy(&cbuf[chunk_len & ~3U], &dr, chunk_len & 3U);
		}

		len -= chunk_len;
		cbuf += chunk_len;
		addr += chunk_len;
	} while (len > 0U);
}

static __ramfunc void qspi_nor_write_fifo(const struct device *dev, uint8_t cmd, uint32_t ccrx,
					  uint32_t addr, void *buf, size_t len)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;
	uint8_t *cbuf = buf;

	/* write in FIFO max sized chunks */
	do {
		size_t chunk_len = MIN(len, MPI_FIFO_SIZE);

		/* write length */
		ll_mpi_set_data_length(data->mpi, LL_MPI_CS_1, chunk_len);

		/* push data */
		for (size_t i = 0U; i < (chunk_len / 4U); i++) {
			uint32_t dr = 0U;

			qspi_nor_memcpy(&dr, &cbuf[i * 4U], 4U);
			ll_mpi_write_data(data->mpi, dr);
		}

		if (chunk_len & 3U) {
			uint32_t dr = 0U;

			qspi_nor_memcpy(&dr, &cbuf[chunk_len & ~3U], chunk_len & 3U);
			ll_mpi_write_data(data->mpi, dr);
		}

		qspi_nor_cinstr(dev, SPI_NOR_CMD_WREN);
		qspi_nor_cinstr_seq_ready_wait(dev, cmd, ccrx, addr);

		len -= chunk_len;
		cbuf += chunk_len;
		addr += chunk_len;
	} while (len > 0U);
}

static inline void qspi_nor_rdsr(const struct device *dev, uint8_t *sr)
{
	qspi_nor_read_fifo(dev, SPI_NOR_CMD_RDSR, MPI_CCRX_CMD_RDSR, 0U, sr, 1U);
}

static inline void qspi_nor_rdsr2(const struct device *dev, uint8_t *sr)
{
	qspi_nor_read_fifo(dev, SPI_NOR_CMD_RDSR2, MPI_CCRX_CMD_RDSR2, 0U, sr, 1U);
}

static inline void qspi_nor_wrsr(const struct device *dev, uint8_t *sr, size_t len)
{
	qspi_nor_write_fifo(dev, SPI_NOR_CMD_WRSR, MPI_CCRX_CMD_WRSR, 0U, sr, len);
}

static inline void qspi_nor_wrsr2(const struct device *dev, uint8_t *sr)
{
	qspi_nor_write_fifo(dev, SPI_NOR_CMD_WRSR2, MPI_CCRX_CMD_WRSR2, 0U, sr, 1U);
}

static inline bool qspi_nor_range_is_valid(const struct flash_sf32lb_mpi_qspi_nor_data *data,
					   off_t offset, size_t size)
{
	if (offset < 0) {
		return false;
	}

	if ((uint64_t)offset > data->size) {
		return false;
	}

	return size <= ((uint64_t)data->size - (uint64_t)offset);
}

static inline bool qspi_nor_addr_add_overflow(uintptr_t addr, size_t size, uintptr_t *end)
{
	if (size > (UINTPTR_MAX - addr)) {
		return true;
	}

	*end = addr + size;

	return false;
}

static inline bool qspi_nor_ranges_overlap(uintptr_t a_start, uintptr_t a_end,
					   uintptr_t b_start, uintptr_t b_end)
{
	return (a_start < b_end) && (a_end > b_start);
}

static inline bool qspi_nor_buf_overlaps_flash(const struct flash_sf32lb_mpi_qspi_nor_data *data,
					       const void *buf, size_t size)
{
	uintptr_t buf_start = (uintptr_t)buf;
	uintptr_t buf_end;
	uintptr_t flash_end;

	if ((size == 0U) || qspi_nor_addr_add_overflow(buf_start, size, &buf_end) ||
	    qspi_nor_addr_add_overflow(data->base, data->size, &flash_end)) {
		return false;
	}

	return qspi_nor_ranges_overlap(buf_start, buf_end, data->base, flash_end);
}

/* API */

static int flash_sf32lb_mpi_qspi_nor_read(const struct device *dev, off_t offset, void *data_,
					  size_t size)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;

	if (!qspi_nor_range_is_valid(data, offset, size) || ((data_ == NULL) && (size > 0U))) {
		return -EINVAL;
	}

	if (size == 0U) {
		return 0;
	}

	memcpy(data_, (void *)(data->base + offset), size);

	return 0;
}

static int flash_sf32lb_mpi_qspi_nor_write(const struct device *dev, off_t offset,
					   const void *data_, size_t size)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;
	const uint8_t *cdata = data_;
	uintptr_t src_start = (uintptr_t)data_;
	uintptr_t src_end;
	uintptr_t dst_start;
	uintptr_t dst_end;
	bool src_overlaps_flash;

	if (!qspi_nor_range_is_valid(data, offset, size)) {
		return -EINVAL;
	}

	if (size == 0U) {
		return 0;
	}

	if ((data_ == NULL) || qspi_nor_addr_add_overflow(src_start, size, &src_end) ||
	    qspi_nor_addr_add_overflow(data->base, (size_t)offset, &dst_start) ||
	    qspi_nor_addr_add_overflow(dst_start, size, &dst_end)) {
		return -EINVAL;
	}

	src_overlaps_flash = qspi_nor_buf_overlaps_flash(data, data_, size);

	if (src_overlaps_flash && qspi_nor_ranges_overlap(src_start, src_end, dst_start, dst_end)) {
		if ((src_start == dst_start) && (src_end == dst_end)) {
			return 0;
		}

		return -EINVAL;
	}

	while (size > 0) {
		int ret;
		k_spinlock_key_t key;
		struct dma_status status;
		uint16_t chunk_len;
		const uint8_t *dma_data = cdata;

		/* limit to page size, without crossing page boundary */
		chunk_len = MIN(size, SPI_NOR_PAGE_SIZE);
		if ((offset % SPI_NOR_PAGE_SIZE) + chunk_len > SPI_NOR_PAGE_SIZE) {
			chunk_len = SPI_NOR_PAGE_SIZE - (offset % SPI_NOR_PAGE_SIZE);
		}

		key = k_spin_lock(&data->lock);

		if (src_overlaps_flash && qspi_nor_buf_overlaps_flash(data, cdata, chunk_len)) {
			memcpy(data->write_buf, cdata, chunk_len);
			dma_data = data->write_buf;
		}

		/* force flash into write mode */
		ll_mpi_write_command_config(data->mpi, LL_MPI_CS_1, data->ccrx_pp);

		/* enable DMA */
		ll_mpi_enable_dma(data->mpi);

		/* configure data length */
		ll_mpi_set_data_length(data->mpi, LL_MPI_CS_1, chunk_len);

		/* trigger DMA transfer */
		ret = sf32lb_dma_reload_dt(&data->dma, (uintptr_t)dma_data, (uintptr_t)&data->mpi->DR,
					   chunk_len);
		if (ret < 0) {
			ll_mpi_disable_dma(data->mpi);
			k_spin_unlock(&data->lock, key);
			return ret;
		}

		ret = sf32lb_dma_start_dt(&data->dma);
		if (ret < 0) {
			ll_mpi_disable_dma(data->mpi);
			k_spin_unlock(&data->lock, key);
			return ret;
		}

		/* enable write, send command and wait until ready */
		qspi_nor_cinstr(dev, SPI_NOR_CMD_WREN);
		qspi_nor_cinstr_seq_ready_wait(dev, data->cmd_pp, data->ccrx_pp, offset);

		/* wait for DMA completion (polling) */
		do {
			ret = sf32lb_dma_get_status_dt(&data->dma, &status);
		} while ((ret == 0) && (status.pending_length != 0U));

		(void)sf32lb_dma_stop_dt(&data->dma);

		/* disable DMA */
		ll_mpi_disable_dma(data->mpi);

		k_spin_unlock(&data->lock, key);

		if (ret < 0) {
			return ret;
		}

		sys_cache_data_invd_range((void *)(data->base + offset), chunk_len);

		size -= chunk_len;
		cdata += chunk_len;
		offset += chunk_len;
	}

	return 0;
}

static int flash_sf32lb_mpi_qspi_nor_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;
	uint32_t ccrx;

	/* address must be sector-aligned */
	if ((offset % SPI_NOR_SECTOR_SIZE) != 0) {
		return -EINVAL;
	}

	/* size must be a non-zero multiple of sectors */
	if ((size == 0U) || (size % SPI_NOR_SECTOR_SIZE) != 0) {
		return -EINVAL;
	}

	/* affected region should be within device */
	if ((offset < 0) || (offset + size) > data->size) {
		return -EINVAL;
	}

	do {
		uint8_t cmd;
		uint32_t adj;
		k_spinlock_key_t key;

		if (size == data->size) {
			cmd = SPI_NOR_CMD_CE;
			ccrx = MPI_CCRX_CMD_CE;
			adj = size;
		} else if (size >= SPI_NOR_BLOCK_SIZE) {
			cmd = data->cmd_be;
			ccrx = data->ccrx_be_se;
			adj = SPI_NOR_BLOCK_SIZE;
		} else if (size >= (SPI_NOR_BLOCK_SIZE / 2U)) {
			cmd = data->cmd_be32;
			ccrx = data->ccrx_be_se;
			adj = SPI_NOR_BLOCK_SIZE / 2U;
		} else if (size >= SPI_NOR_SECTOR_SIZE) {
			cmd = data->cmd_se;
			ccrx = data->ccrx_be_se;
			adj = SPI_NOR_SECTOR_SIZE;
		} else {
			return -EINVAL;
		}

		key = k_spin_lock(&data->lock);
		qspi_nor_cinstr(dev, SPI_NOR_CMD_WREN);
		qspi_nor_cinstr_seq_ready_wait(dev, cmd, ccrx, offset);
		k_spin_unlock(&data->lock, key);

		sys_cache_data_invd_range((void *)(data->base + offset), size);

		size -= adj;
		offset += adj;
	} while (size > 0U);

	return 0;
}

static const struct flash_parameters *
flash_sf32lb_mpi_qspi_nor_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_nor_parameters;
}

static int flash_sf32lb_mpi_qspi_nor_get_size(const struct device *dev, uint64_t *size)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;

	*size = data->size;

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_sf32lb_mpi_qspi_nor_page_layout(const struct device *dev,
						  const struct flash_pages_layout **layout,
						  size_t *layout_size)
{
	const struct flash_sf32lb_mpi_qspi_nor_config *config = dev->config;

	*layout = &config->layout;
	*layout_size = 1U;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#ifdef CONFIG_FLASH_JESD216_API
static int flash_sf32lb_mpi_qspi_nor_sfdp_read(const struct device *dev, off_t offset, void *buf,
					       size_t len)
{
	qspi_nor_read_fifo(dev, JESD216_CMD_READ_SFDP, MPI_CCRX_CMD_READ_SFDP, offset, buf, len);

	return 0;
}

static int flash_sf32lb_mpi_qspi_nor_read_jedec_id(const struct device *dev, uint8_t *id)
{
	qspi_nor_read_fifo(dev, SPI_NOR_CMD_RDID, MPI_CCRX_CMD_RDID, 0U, id, 3U);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static DEVICE_API(flash, flash_sf32lb_mpi_qspi_nor_api) = {
	.read = flash_sf32lb_mpi_qspi_nor_read,
	.write = flash_sf32lb_mpi_qspi_nor_write,
	.erase = flash_sf32lb_mpi_qspi_nor_erase,
	.get_parameters = flash_sf32lb_mpi_qspi_nor_get_parameters,
	.get_size = flash_sf32lb_mpi_qspi_nor_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_sf32lb_mpi_qspi_nor_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#ifdef CONFIG_FLASH_JESD216_API
	.sfdp_read = flash_sf32lb_mpi_qspi_nor_sfdp_read,
	.read_jedec_id = flash_sf32lb_mpi_qspi_nor_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

static __ramfunc int flash_sf32lb_mpi_qspi_nor_init(const struct device *dev)
{
	struct flash_sf32lb_mpi_qspi_nor_data *data = dev->data;
	struct dma_config config_dma = {0};
	struct dma_block_config block_cfg = {0};
	int ret;

	ret = clock_control_on(data->clock.dev, (clock_control_subsys_t)&data->clock.id);
	if (ret < 0) {
		return ret;
	}

	if (!sf32lb_dma_is_ready_dt(&data->dma)) {
		return -ENODEV;
	}

	ll_mpi_set_fifo_tx_slots(data->mpi, 1U);

	ll_mpi_set_rx_clk_invert(data->mpi, !!data->invert_rx_clk);

	ll_mpi_set_clock_div(data->mpi, data->psclr);

	/* enable QSPI (non-dual mode) */
	ll_mpi_set_dual_flash_mode(data->mpi, 0U);
	ll_mpi_enable(data->mpi);

	/* enable QER */
	if (data->qer != JESD216_DW15_QER_VAL_NONE) {
		uint8_t sr[2];

		switch (data->qer) {
		case JESD216_DW15_QER_VAL_S1B6:
			qspi_nor_rdsr(dev, &sr[0]);
			if ((sr[0] & BIT(6)) == 0U) {
				sr[0] |= BIT(6);
				qspi_nor_wrsr(dev, &sr[0], 1U);
			}
			break;
		case JESD216_DW15_QER_VAL_S2B1v1:
		case JESD216_DW15_QER_VAL_S2B1v4:
		case JESD216_DW15_QER_VAL_S2B1v5:
			qspi_nor_rdsr(dev, &sr[0]);
			qspi_nor_rdsr2(dev, &sr[1]);
			if ((sr[1] & BIT(1)) == 0U) {
				sr[1] |= BIT(1);
				qspi_nor_wrsr(dev, &sr[0], 2U);
			}
			break;
		case JESD216_DW15_QER_VAL_S2B1v6:
			qspi_nor_rdsr2(dev, &sr[1]);
			if ((sr[1] & BIT(1)) == 0U) {
				sr[1] |= BIT(1);
				qspi_nor_wrsr2(dev, &sr[1]);
			}
			break;
		default:
			return -ENOTSUP;
		}
	}

	if (data->addr_len == 4U) {
		/* enable 4-byte address mode */
		qspi_nor_cinstr(dev, SPI_NOR_CMD_4BA);
	}

	/* configure AHB read command */
	ll_mpi_set_ahb_read_config(data->mpi, data->ccrx_read);

	ll_mpi_set_ahb_read_command(data->mpi, data->cmd_read);

	/* perform initial DMA configuration (so we can just reload) */
	sf32lb_dma_config_init_dt(&data->dma, &config_dma);
	block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	config_dma.head_block = &block_cfg;
	config_dma.block_count = 1U;
	config_dma.channel_direction = MEMORY_TO_PERIPHERAL;
	config_dma.source_data_size = 1U;
	config_dma.dest_data_size = 1U;

	ret = sf32lb_dma_config_dt(&data->dma, &config_dma);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define QSPI_NEEDS_4B_ADDR(n)                                                                      \
	((DT_PROP(DT_INST_CHILD(n, flash_0), size) / 8U) > QSPI_NOR_MAX_3B_SIZE)
#define QSPI_IS_QUAD(n) (DT_INST_PROP_OR(n, sifli_lines, 1U) == 4U)
#define QSPI_QER(n)                                                                                \
	_CONCAT(JESD216_DW15_QER_VAL_,                                                             \
		DT_STRING_TOKEN_OR(DT_INST_CHILD(n, flash_0), quad_enable_requirements, NONE))

#define FLASH_SF32LB_MPI_QSPI_NOR_DEFINE(n)                                                        \
	BUILD_ASSERT((DT_INST_CHILD_NUM(n) == 1),                                                  \
		     "Only one memory node is supported per MPI controller");                      \
	BUILD_ASSERT((QSPI_IS_QUAD(n) && (QSPI_QER(n) != JESD216_DW15_QER_VAL_NONE)),              \
		     "Quad SPI requires a valid quad-enable-requirements");                        \
                                                                                                   \
	static const struct flash_sf32lb_mpi_qspi_nor_config config##n = {                         \
		.layout =                                                                          \
			{                                                                          \
				.pages_count = (DT_PROP(DT_INST_CHILD(n, flash_0), size) / 8U) /   \
					       SPI_NOR_SECTOR_SIZE,                                \
				.pages_size = SPI_NOR_SECTOR_SIZE,                                 \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct flash_sf32lb_mpi_qspi_nor_data data##n = {                                   \
		.mpi = (MPI_TypeDef *)DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                          \
		.base = DT_INST_REG_ADDR_BY_NAME(n, nor),                                          \
		.size = DT_PROP(DT_INST_CHILD(n, flash_0), size) / 8U,                             \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(n),                                         \
		.dma = SF32LB_DMA_DT_INST_SPEC_GET(n),                                             \
		.lines = DT_INST_PROP_OR(n, sifli_lines, 1U),                                      \
		.psclr = DT_INST_PROP(n, sifli_psclr),                                             \
		.invert_rx_clk = DT_INST_PROP(n, sifli_invert_rx_clk),                             \
		.qer = QSPI_QER(n),                                                                \
		.addr_len = QSPI_NEEDS_4B_ADDR(n) ? 4U : 3U,                                       \
		.cmd_read = QSPI_NEEDS_4B_ADDR(n) ? (QSPI_IS_QUAD(n) ? SPI_NOR_CMD_4READ_4B        \
								     : SPI_NOR_CMD_READ_FAST_4B)   \
						  : (QSPI_IS_QUAD(n) ? SPI_NOR_CMD_4READ           \
								     : SPI_NOR_CMD_READ_FAST),     \
		.ccrx_read = QSPI_NEEDS_4B_ADDR(n) ? (QSPI_IS_QUAD(n) ? MPI_CCRX_CMD_4READ_4B      \
								      : MPI_CCRX_CMD_READ_FAST_4B) \
						   : (QSPI_IS_QUAD(n) ? MPI_CCRX_CMD_4READ         \
								      : MPI_CCRX_CMD_READ_FAST),   \
		.cmd_pp =                                                                          \
			QSPI_NEEDS_4B_ADDR(n)                                                      \
				? (QSPI_IS_QUAD(n) ? SPI_NOR_CMD_PP_1_1_4_4B : SPI_NOR_CMD_PP_4B)  \
				: (QSPI_IS_QUAD(n) ? SPI_NOR_CMD_PP_1_1_4 : SPI_NOR_CMD_PP),       \
		.ccrx_pp = QSPI_NEEDS_4B_ADDR(n)                                                   \
				   ? (QSPI_IS_QUAD(n) ? MPI_CCRX_CMD_PP_1_1_4_4B                   \
						      : MPI_CCRX_CMD_PP_4B)                        \
				   : (QSPI_IS_QUAD(n) ? MPI_CCRX_CMD_PP_1_1_4 : MPI_CCRX_CMD_PP),  \
		.cmd_be = QSPI_NEEDS_4B_ADDR(n) ? SPI_NOR_CMD_BE_4B : SPI_NOR_CMD_BE,              \
		.cmd_be32 = QSPI_NEEDS_4B_ADDR(n) ? SPI_NOR_CMD_BE_32K_4B : SPI_NOR_CMD_BE_32K,    \
		.cmd_se = QSPI_NEEDS_4B_ADDR(n) ? SPI_NOR_CMD_SE_4B : SPI_NOR_CMD_SE,              \
		.ccrx_be_se = QSPI_NEEDS_4B_ADDR(n) ? MPI_CCRX_CMD_BE_SE_4B : MPI_CCRX_CMD_BE_SE,  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, flash_sf32lb_mpi_qspi_nor_init, NULL, &data##n, &config##n,       \
			      PRE_KERNEL_1, CONFIG_FLASH_INIT_PRIORITY,                            \
			      &flash_sf32lb_mpi_qspi_nor_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_SF32LB_MPI_QSPI_NOR_DEFINE)

#define FLASH_SF32LB_MPI_QSPI_NOR_CHIP_DEFINE(n)                                                   \
	DEVICE_DT_DEFINE(DT_INST_CHILD(n, flash_0), flash_sf32lb_mpi_qspi_nor_init, NULL,          \
			&data##n, &config##n, PRE_KERNEL_1, CONFIG_FLASH_INIT_PRIORITY,           \
			&flash_sf32lb_mpi_qspi_nor_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_SF32LB_MPI_QSPI_NOR_CHIP_DEFINE)
