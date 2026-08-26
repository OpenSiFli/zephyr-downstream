/*
 * Copyright (c) Core Devices LLC
 * Copyright (c) Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_usart

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <string.h>
#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma/sf32lb.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/ring_buffer.h>
#endif

#include <ll_usart.h>

LOG_MODULE_REGISTER(sf32lb_uart, CONFIG_UART_LOG_LEVEL);

#ifdef CONFIG_UART_ASYNC_API
#define UART_SF32LB_TX_DMA_BUFFER_SIZE 256U
#define UART_SF32LB_ASYNC_STATUS_TIMEOUT (DMA_STATUS_HALF_COMPLETE + 1)

#if defined(CONFIG_DCACHE)
#define UART_SF32LB_RX_DMA_ALIGN CONFIG_DCACHE_LINE_SIZE
#else
#define UART_SF32LB_RX_DMA_ALIGN sizeof(void *)
#endif

#define UART_SF32LB_RX_DMA_ALLOC_PAD (2U * UART_SF32LB_RX_DMA_ALIGN)

struct sf32lb_uart_async_tx {
	const uint8_t *buf;
	size_t len;
	size_t offset;
	size_t dma_len;
	uint8_t dma_buf[UART_SF32LB_TX_DMA_BUFFER_SIZE] __aligned(sizeof(void *));
	int32_t timeout;
	struct k_work_delayable timeout_work;
};

struct sf32lb_uart_async_rx {
	uint8_t *buf;
	size_t len;
	uint8_t *next_buf;
	size_t next_len;
	size_t offset;
	volatile size_t counter;
	size_t reported;
	uint8_t *dma_buf;
	uint8_t *dma_alloc;
	size_t dma_len;
	size_t dma_offset;
	bool irq_mode;
	struct ring_buf irq_fifo;
	uint8_t irq_fifo_data[CONFIG_UART_SF32LB_ASYNC_RX_IRQ_FIFO_SIZE];
	struct k_work irq_work;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
	bool stop_pending;
	bool stopping;
	int stop_reason;
};

struct sf32lb_uart_async_data {
	const struct device *uart_dev;
	struct sf32lb_uart_async_tx tx;
	struct sf32lb_uart_async_rx rx;
	uart_callback_t cb;
	void *user_data;
};
#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#ifdef CONFIG_UART_ASYNC_API
static inline void uart_sf32lb_async_timer_start(struct k_work_delayable *work,
						 size_t timeout);
static inline void uart_sf32lb_async_tx_done(struct sf32lb_uart_async_data *data);
static inline void uart_sf32lb_async_rx_cache_invalidate(
		struct sf32lb_uart_async_data *data, size_t offset, size_t len);
static void uart_sf32lb_async_rx_defer_stop(const struct device *dev, int reason);
static void uart_sf32lb_async_rx_flush(const struct device *dev,
							 int status);
static int uart_async_sf32lb_rx_disable(const struct device *dev);
#endif
static void uart_sf32lb_isr(const struct device *dev);
#endif

struct uart_sf32lb_data {
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	struct uart_config uart_config;
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_callback;
	void *cb_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	struct sf32lb_uart_async_data async;
#endif
};

struct uart_sf32lb_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	struct uart_config uart_cfg;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
#ifdef CONFIG_UART_ASYNC_API
	struct sf32lb_dma_dt_spec tx_dma;
	struct sf32lb_dma_dt_spec rx_dma;
#endif /* CONFIG_UART_ASYNC_API */
};

static int uart_sf32lb_err_check(const struct device *dev);

static inline USART_TypeDef *uart_sf32lb_regs(const struct uart_sf32lb_config *config)
{
	return (USART_TypeDef *)config->base;
}

#ifdef CONFIG_UART_ASYNC_API
static inline uintptr_t uart_sf32lb_rdr_addr(const struct uart_sf32lb_config *config)
{
	return (uintptr_t)&uart_sf32lb_regs(config)->RDR;
}

static inline uintptr_t uart_sf32lb_tdr_addr(const struct uart_sf32lb_config *config)
{
	return (uintptr_t)&uart_sf32lb_regs(config)->TDR;
}
#endif

static int uart_sf32lb_get_frame_config(const struct uart_config *cfg,
					ll_usart_frame_config_t *frame)
{
	enum uart_config_data_bits data_bits = cfg->data_bits;

	/* SiFli USART data width includes the parity bit. */
	if (cfg->parity != UART_CFG_PARITY_NONE) {
		data_bits++;
		if (data_bits > UART_CFG_DATA_BITS_9) {
			return -ENOTSUP;
		}
	}

	switch (data_bits) {
	case UART_CFG_DATA_BITS_6:
		frame->data_width = LL_USART_DATAWIDTH_6B;
		break;
	case UART_CFG_DATA_BITS_7:
		frame->data_width = LL_USART_DATAWIDTH_7B;
		break;
	case UART_CFG_DATA_BITS_8:
		frame->data_width = LL_USART_DATAWIDTH_8B;
		break;
	case UART_CFG_DATA_BITS_9:
		frame->data_width = LL_USART_DATAWIDTH_9B;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		frame->parity = LL_USART_PARITY_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		frame->parity = LL_USART_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		frame->parity = LL_USART_PARITY_EVEN;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		frame->stop_bits = LL_USART_STOPBITS_1;
		break;
	case UART_CFG_STOP_BITS_2:
		frame->stop_bits = LL_USART_STOPBITS_2;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int uart_sf32lb_get_hwflow_config(const struct uart_config *cfg, uint32_t *hwflow)
{
	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		*hwflow = LL_USART_HWCONTROL_NONE;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		*hwflow = LL_USART_HWCONTROL_RTS_CTS;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void uart_sf32lb_isr(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	USART_TypeDef *uart = uart_sf32lb_regs(config);
#ifdef CONFIG_UART_ASYNC_API
	const uint32_t isr = ll_usart_get_isr(uart);
	const bool rx_idle = ll_usart_is_enabled_it_idle(uart) &&
		ll_usart_is_active_flag_idle(uart);
	const bool tx_complete = ll_usart_is_enabled_it_tc(uart) &&
		ll_usart_is_active_flag_tc(uart);
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->irq_callback) {
		data->irq_callback(dev, data->cb_data);
		ll_usart_clear_flag_tc(uart);
		return;
	}
#endif

#ifdef CONFIG_UART_ASYNC_API
	if (rx_idle) {
		ll_usart_clear_flag_idle(uart);

		if (data->async.rx.enabled && !data->async.rx.irq_mode) {
			if (data->async.rx.timeout == 0 ||
			    (IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC) &&
			     data->async.rx.timeout == SYS_FOREVER_US)) {
				uart_sf32lb_async_rx_flush(dev, UART_SF32LB_ASYNC_STATUS_TIMEOUT);
			} else {
				uart_sf32lb_async_timer_start(&data->async.rx.timeout_work,
							      data->async.rx.timeout);
			}
		}
	}

	if (tx_complete && data->async.tx.buf != NULL) {
		ll_usart_disable_it_tc(uart);
		uart_sf32lb_async_tx_done(&data->async);
	}

	if (data->async.rx.irq_mode && (isr & USART_ISR_RXNE) != 0U) {
		bool fifo_overflow = false;

		do {
			uint8_t byte = ll_usart_receive_data8(uart);

			if (ring_buf_put(&data->async.rx.irq_fifo, &byte, 1U) != 1U) {
				fifo_overflow = true;
				ll_usart_request_rxdata_flush(uart);
				break;
			}
		} while (ll_usart_is_active_flag_rxne(uart));

		if (fifo_overflow) {
			uart_sf32lb_async_rx_defer_stop(dev, -ENOBUFS);
		} else {
			k_work_submit(&data->async.rx.irq_work);
		}
	} else if (ll_usart_is_enabled_it_rxne(uart) && ll_usart_is_active_flag_rxne(uart)) {
		ll_usart_request_rxdata_flush(uart);
	}

	uart_sf32lb_err_check(dev);
#endif
}
#endif

static int uart_sf32lb_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_sf32lb_config *config = dev->config;
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	struct uart_sf32lb_data *data = dev->data;
#endif
	USART_TypeDef *uart = uart_sf32lb_regs(config);
	ll_usart_frame_config_t frame;
	uint32_t hwflow;
	int ret;

	ret = uart_sf32lb_get_frame_config(cfg, &frame);
	if (ret < 0) {
		return ret;
	}

	ret = uart_sf32lb_get_hwflow_config(cfg, &hwflow);
	if (ret < 0) {
		return ret;
	}

	ll_usart_disable(uart);
	ll_usart_config_frame(uart, &frame);
	ll_usart_config_hwflow(uart, hwflow);
	ll_usart_enable(uart);
	ll_usart_enable_tx(uart);
	ll_usart_enable_rx(uart);
	ll_usart_config_baudrate(uart, cfg->baudrate);

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	data->uart_config = *cfg;
#endif
	return 0;
}

static int uart_sf32lb_poll_in(const struct device *dev, uint8_t *c)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);

	if (ll_usart_is_active_flag_rxne(uart)) {
		*c = ll_usart_receive_data8(uart);
		return 0;
	}

	return -1;
}

static void uart_sf32lb_poll_out(const struct device *dev, uint8_t c)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);

	ll_usart_clear_flag_tc(uart);
	ll_usart_transmit_data8(uart, c);

	while (!ll_usart_is_active_flag_tc(uart)) {
	}
}

static int uart_sf32lb_err_check(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);
	int err = 0;

	if (ll_usart_is_active_flag_ore(uart)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (ll_usart_is_active_flag_pe(uart)) {
		err |= UART_ERROR_PARITY;
	}

	if (ll_usart_is_active_flag_fe(uart)) {
		err |= UART_ERROR_FRAMING;
	}

	if (ll_usart_is_active_flag_ne(uart)) {
		err |= UART_ERROR_NOISE;
	}

	/* clear error flags */
	ll_usart_clear_flag_ore(uart);
	ll_usart_clear_flag_pe(uart);
	ll_usart_clear_flag_fe(uart);
	ll_usart_clear_flag_ne(uart);

	return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_sf32lb_configure_set(const struct device *dev, const struct uart_config *cfg)
{
	return uart_sf32lb_configure(dev, cfg);
}

static int uart_sf32lb_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_sf32lb_data *data = dev->data;

	*cfg = data->uart_config;
	return 0;
}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_sf32lb_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);
	int i;

	for (i = 0; i < len; i++) {
		if (!ll_usart_is_active_flag_txe(uart)) {
			break;
		}
		ll_usart_transmit_data8(uart, tx_data[i]);
	}

	return i;
}

static int uart_sf32lb_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);
	int i;

	for (i = 0; i < size; i++) {
		if (!ll_usart_is_active_flag_rxne(uart)) {
			break;
		}
		rx_data[i] = ll_usart_receive_data8(uart);
	}

	return i;
}

static void uart_sf32lb_irq_tx_enable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	ll_usart_enable_it_txe(uart_sf32lb_regs(config));
}

static void uart_sf32lb_irq_tx_disable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	ll_usart_disable_it_txe(uart_sf32lb_regs(config));
}

static int uart_sf32lb_irq_tx_ready(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	return ll_usart_is_active_flag_txe(uart_sf32lb_regs(config));
}

static int uart_sf32lb_irq_tx_complete(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	return ll_usart_is_active_flag_tc(uart_sf32lb_regs(config));
}

static int uart_sf32lb_irq_rx_ready(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	return ll_usart_is_active_flag_rxne(uart_sf32lb_regs(config));
}

static void uart_sf32lb_irq_err_enable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);

	ll_usart_enable_it_pe(uart);
	ll_usart_enable_it_error(uart);
}

static void uart_sf32lb_irq_err_disable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);

	ll_usart_disable_it_pe(uart);
	ll_usart_disable_it_error(uart);
}

static int uart_sf32lb_irq_is_pending(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	USART_TypeDef *uart = uart_sf32lb_regs(config);

	/* Aggregate ISR snapshot: any set flag means an IRQ is pending. */
	return ll_usart_get_isr(uart) == 0U ? 0 : 1;
}

static void uart_sf32lb_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					 void *user_data)
{
	struct uart_sf32lb_data *data = dev->data;

	data->irq_callback = cb;
	data->cb_data = user_data;
#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->async.cb = NULL;
	data->async.user_data = NULL;
#endif
}

static void uart_sf32lb_irq_rx_enable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	ll_usart_enable_it_rxne(uart_sf32lb_regs(config));
}

static void uart_sf32lb_irq_rx_disable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;

	ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
static inline void uart_sf32lb_async_timer_start(struct k_work_delayable *work, size_t timeout)
{
	if ((timeout != SYS_FOREVER_US) && (timeout != 0)) {
		k_work_reschedule(work, K_USEC(timeout));
	}
}

static inline void uart_sf32lb_async_tx_done(struct sf32lb_uart_async_data *data)
{
	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx = {
			.buf = data->tx.buf,
			.len = data->tx.len,
		},
	};

	data->tx.buf = NULL;
	data->tx.len = 0U;
	data->tx.offset = 0U;
	data->tx.dma_len = 0U;

	if (data->cb) {
		data->cb(data->uart_dev, &evt, data->user_data);
	}
}

static inline void uart_sf32lb_async_rx_rdy(struct sf32lb_uart_async_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = data->rx.buf,
			.len = data->rx.counter - data->rx.offset,
			.offset = data->rx.offset,
		},
	};

	data->rx.offset = data->rx.counter;
	if (evt.data.rx.len != 0U && data->cb) {
		data->cb(data->uart_dev, &evt, data->user_data);
	}
}

static inline void uart_sf32lb_async_rx_report(struct sf32lb_uart_async_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = data->rx.buf,
			.len = data->rx.counter - data->rx.reported,
			.offset = data->rx.reported,
		},
	};

	data->rx.reported = data->rx.counter;
	if (evt.data.rx.len != 0U && data->cb) {
		data->cb(data->uart_dev, &evt, data->user_data);
	}
}

static inline void uart_sf32lb_async_rx_released(struct sf32lb_uart_async_data *data,
							 uint8_t *buf)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {
			.buf = buf,
		},
	};

	if (data->cb) {
		data->cb(data->uart_dev, &evt, data->user_data);
	}
}

static inline void uart_sf32lb_async_rx_cache_invalidate(
		struct sf32lb_uart_async_data *data, size_t offset, size_t len)
{
#if defined(CONFIG_DCACHE)
	uintptr_t start;
	uintptr_t end;
	const uintptr_t mask = CONFIG_DCACHE_LINE_SIZE - 1U;

	if (data->rx.dma_buf == NULL || len == 0U) {
		return;
	}

	start = ((uintptr_t)data->rx.dma_buf + offset) & ~mask;
	end = ((uintptr_t)data->rx.dma_buf + offset + len + mask) & ~mask;
	(void)sys_cache_data_invd_range((void *)start, end - start);
#else
	ARG_UNUSED(data);
	ARG_UNUSED(offset);
	ARG_UNUSED(len);
#endif
}

static bool uart_sf32lb_async_rx_advance(struct sf32lb_uart_async_data *data)
{
	uint8_t *next_buf = data->rx.next_buf;
	size_t next_len = data->rx.next_len;

	if (next_buf == NULL || next_len == 0U) {
		data->rx.stop_pending = true;
		ll_usart_disable_dma_rx(uart_sf32lb_regs(data->uart_dev->config));
		return false;
	}

	uart_sf32lb_async_rx_released(data, data->rx.buf);
	data->rx.buf = next_buf;
	data->rx.len = next_len;
	data->rx.next_buf = NULL;
	data->rx.next_len = 0U;
	data->rx.offset = 0U;
	data->rx.counter = 0U;
	data->rx.reported = 0U;

	if (!data->rx.stopping && data->cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_REQUEST,
		};

		data->cb(data->uart_dev, &evt, data->user_data);
	}

	return true;
}

static size_t uart_sf32lb_async_rx_copy_segment(struct sf32lb_uart_async_data *data,
							 size_t dma_offset, size_t len,
							 bool report_partial)
{
	size_t copied = 0U;

	while (len != 0U && data->rx.enabled && !data->rx.stop_pending) {
		size_t copy_len;

		if (data->rx.counter >= data->rx.len && !uart_sf32lb_async_rx_advance(data)) {
			break;
		}

		copy_len = MIN(len, data->rx.len - data->rx.counter);
		uart_sf32lb_async_rx_cache_invalidate(data, dma_offset, copy_len);
		memcpy(data->rx.buf + data->rx.counter,
		       data->rx.dma_buf + dma_offset, copy_len);
		data->rx.counter += copy_len;
		if (report_partial || data->rx.counter == data->rx.len) {
			uart_sf32lb_async_rx_report(data);
		}

		dma_offset += copy_len;
		if (dma_offset == data->rx.dma_len) {
			dma_offset = 0U;
		}
		len -= copy_len;
		copied += copy_len;

		if (data->rx.counter == data->rx.len && data->rx.enabled) {
			(void)uart_sf32lb_async_rx_advance(data);
		}
	}

	return copied;
}

static inline void uart_sf32lb_async_rx_stopped(struct sf32lb_uart_async_data *data,
							 int reason)
{
	struct uart_event evt = {
		.type = UART_RX_STOPPED,
		.data.rx_stop = {
			.reason = reason,
			.data = {
				.buf = data->rx.buf,
				.offset = data->rx.offset,
				.len = 0U,
			},
		},
	};

	if (data->cb) {
		data->cb(data->uart_dev, &evt, data->user_data);
	}
}

static void uart_sf32lb_async_rx_defer_stop(const struct device *dev, int reason)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	unsigned int key = irq_lock();

	(void)k_work_cancel_delayable(&data->async.rx.timeout_work);
	ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
	ll_usart_disable_it_idle(uart_sf32lb_regs(config));
	ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
	ll_usart_disable_it_error(uart_sf32lb_regs(config));
	data->async.rx.stop_reason = reason;
	data->async.rx.stopping = true;
	(void)k_work_reschedule(&data->async.rx.timeout_work, K_TICKS(1));

	irq_unlock(key);
}

static void uart_sf32lb_async_rx_flush(const struct device *dev,
								int status)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct sf32lb_uart_async_data *data = &((struct uart_sf32lb_data *)dev->data)->async;
	struct dma_status dma_stat = {0};
	size_t received;
	size_t dma_offset;
	size_t copied;
	bool report_partial;

	if (!IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC)) {
		if (sf32lb_dma_get_status_dt(&config->rx_dma, &dma_stat) != 0 ||
		    dma_stat.pending_length > data->rx.len) {
			return;
		}

		data->rx.counter = data->rx.len - dma_stat.pending_length;
		if (data->rx.counter > data->rx.offset) {
			uart_sf32lb_async_rx_rdy(data);
		}
		return;
	}

	switch (status) {
	case DMA_STATUS_HALF_COMPLETE:
		if (data->rx.timeout != 0 && data->rx.timeout != SYS_FOREVER_US) {
			return;
		}
		received = data->rx.dma_len / 2U;
		if (received == 0U || data->rx.dma_offset >= received) {
			return;
		}
		dma_offset = data->rx.dma_offset;
		copied = uart_sf32lb_async_rx_copy_segment(data, dma_offset,
								 received - dma_offset, false);
		data->rx.dma_offset = (dma_offset + copied) % data->rx.dma_len;
		break;
	case DMA_STATUS_COMPLETE:
		dma_offset = data->rx.dma_offset;
		if (dma_offset < data->rx.dma_len) {
			copied = uart_sf32lb_async_rx_copy_segment(data, dma_offset,
								 data->rx.dma_len - dma_offset, false);
			data->rx.dma_offset = (dma_offset + copied) % data->rx.dma_len;
		}
		break;
	default:
		if (sf32lb_dma_get_status_dt(&config->rx_dma, &dma_stat) != 0 ||
		    dma_stat.pending_length > data->rx.dma_len) {
			return;
		}

		received = data->rx.dma_len - dma_stat.pending_length;
		dma_offset = data->rx.dma_offset;
		report_partial = data->rx.timeout != SYS_FOREVER_US || data->rx.stopping;
		if (received >= dma_offset) {
			copied = uart_sf32lb_async_rx_copy_segment(data, dma_offset,
								 received - dma_offset, report_partial);
			data->rx.dma_offset = (dma_offset + copied) % data->rx.dma_len;
		} else {
			copied = uart_sf32lb_async_rx_copy_segment(data, dma_offset,
								 data->rx.dma_len - dma_offset, report_partial);
			if (!data->rx.stop_pending) {
				copied += uart_sf32lb_async_rx_copy_segment(data, 0U, received,
										 report_partial);
			}
			data->rx.dma_offset = received;
		}
		if (report_partial && data->rx.counter > data->rx.reported) {
			uart_sf32lb_async_rx_report(data);
		}
		break;
	}

	if (data->rx.stop_pending && !data->rx.stopping) {
		k_work_reschedule(&data->rx.timeout_work, K_TICKS(1));
	}
}

static void uart_sf32lb_dma_tx_done(const struct device *dma_dev, void *user_data, uint32_t channel,
				    int status)
{
	struct uart_sf32lb_data *data = user_data;
	const struct device *uart_dev = data->async.uart_dev;
	const struct uart_sf32lb_config *config = uart_dev->config;
	struct dma_status dma_stat = {0};
	int dma_status_ret;
	int reload_ret;
	int start_ret;
	unsigned int key = irq_lock();

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	k_work_cancel_delayable(&data->async.tx.timeout_work);
	dma_status_ret = sf32lb_dma_get_status_dt(&config->tx_dma, &dma_stat);
	(void)sf32lb_dma_stop_dt(&config->tx_dma);
	/* Disable DMA for TX */
	ll_usart_disable_dma_tx(uart_sf32lb_regs(config));

	if (status == DMA_STATUS_COMPLETE && data->async.tx.buf != NULL &&
	    data->async.tx.offset + data->async.tx.dma_len < data->async.tx.len) {
		data->async.tx.offset += data->async.tx.dma_len;
		data->async.tx.dma_len = MIN(sizeof(data->async.tx.dma_buf),
						    data->async.tx.len - data->async.tx.offset);
		memcpy(data->async.tx.dma_buf,
		       data->async.tx.buf + data->async.tx.offset,
		       data->async.tx.dma_len);

		reload_ret = sf32lb_dma_reload_dt(&config->tx_dma,
						  (uintptr_t)data->async.tx.dma_buf,
						  uart_sf32lb_tdr_addr(config), data->async.tx.dma_len);
		start_ret = sf32lb_dma_start_dt(&config->tx_dma);
		if (reload_ret == 0 && start_ret == 0) {
			uart_sf32lb_async_timer_start(&data->async.tx.timeout_work,
						      data->async.tx.timeout);
			ll_usart_clear_flag_tc(uart_sf32lb_regs(config));
			ll_usart_enable_dma_tx(uart_sf32lb_regs(config));
			irq_unlock(key);
			return;
		}

		status = -EIO;
	}

	if (status != DMA_STATUS_COMPLETE) {
		struct uart_event evt = {
			.type = UART_TX_ABORTED,
			.data.tx = {
				.buf = data->async.tx.buf,
				.len = data->async.tx.offset,
			},
		};

		if (dma_status_ret == 0 && data->async.tx.dma_len >= dma_stat.pending_length) {
			evt.data.tx.len += data->async.tx.dma_len - dma_stat.pending_length;
		}
		evt.data.tx.len = MIN(evt.data.tx.len, data->async.tx.len);
		ll_usart_disable_it_tc(uart_sf32lb_regs(config));
		data->async.tx.buf = NULL;
		data->async.tx.len = 0U;
		data->async.tx.offset = 0U;
		data->async.tx.dma_len = 0U;

		if (data->async.cb) {
			data->async.cb(uart_dev, &evt, data->async.user_data);
		}
	}
	irq_unlock(key);
}

static int uart_sf32lb_async_rx_start_dma(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	USART_TypeDef *uart = uart_sf32lb_regs(config);
	uint8_t *dma_buf = data->async.rx.dma_buf != NULL ? data->async.rx.dma_buf :
		data->async.rx.buf;
	size_t dma_len = data->async.rx.dma_buf != NULL ? data->async.rx.dma_len :
		data->async.rx.len;
	int reload_ret;
	int start_ret;

	data->async.rx.irq_mode = false;
	ll_usart_disable_it_rxne(uart);

	reload_ret = sf32lb_dma_reload_dt(&config->rx_dma, uart_sf32lb_rdr_addr(config),
					  (uintptr_t)dma_buf, dma_len);
	if (reload_ret != 0) {
		return reload_ret;
	}

	start_ret = sf32lb_dma_start_dt(&config->rx_dma);
	if (start_ret != 0) {
		return start_ret;
	}

	ll_usart_enable_dma_rx(uart);
	return 0;
}

static void uart_sf32lb_async_rx_complete(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	uint8_t *rx_buf;
	uint8_t *next_buf;
	size_t next_len;
	int start_ret = 0;

	rx_buf = data->async.rx.buf;
	ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
	data->async.rx.counter = data->async.rx.len;
	uart_sf32lb_async_rx_rdy(&data->async);
	uart_sf32lb_async_rx_released(&data->async, rx_buf);

	next_buf = data->async.rx.next_buf;
	next_len = data->async.rx.next_len;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	data->async.rx.buf = next_buf;
	data->async.rx.len = next_len;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.reported = 0U;
	data->async.rx.irq_mode = (next_buf != NULL && next_len == 1U);

	if (next_buf != NULL && next_len != 0U) {
		if (data->async.rx.irq_mode) {
			ll_usart_enable_it_rxne(uart_sf32lb_regs(config));
		} else {
			start_ret = uart_sf32lb_async_rx_start_dma(dev);
		}

		if (start_ret == 0) {
			data->async.rx.enabled = true;
			ll_usart_clear_flag_idle(uart_sf32lb_regs(config));
			if (data->async.cb) {
				struct uart_event evt = {
					.type = UART_RX_BUF_REQUEST,
				};

				data->async.cb(dev, &evt, data->async.user_data);
			}
		}
	}

	if (start_ret != 0) {
		data->async.rx.stopping = true;
		data->async.rx.stop_reason = start_ret;
		ll_usart_disable_it_idle(uart_sf32lb_regs(config));
		ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
		ll_usart_disable_it_error(uart_sf32lb_regs(config));
		ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
		(void)sf32lb_dma_stop_dt(&config->rx_dma);
		(void)k_work_reschedule(&data->async.rx.timeout_work, K_TICKS(1));
		return;
	}

	if (next_buf == NULL || next_len == 0U) {
		data->async.rx.enabled = false;
		ll_usart_disable_it_idle(uart_sf32lb_regs(config));
		ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
		ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
		(void)sf32lb_dma_stop_dt(&config->rx_dma);
		data->async.rx.buf = NULL;
		data->async.rx.len = 0U;
		data->async.rx.stop_pending = false;
		data->async.rx.stopping = false;
		data->async.rx.stop_reason = 0;
		data->async.rx.timeout = 0;
		ring_buf_reset(&data->async.rx.irq_fifo);

		if (data->async.cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			data->async.cb(dev, &evt, data->async.user_data);
		}
	}
}

static void uart_sf32lb_async_rx_irq_work(struct k_work *work)
{
	struct sf32lb_uart_async_rx *rx = CONTAINER_OF(work, struct sf32lb_uart_async_rx,
						       irq_work);
	struct sf32lb_uart_async_data *async = CONTAINER_OF(rx, struct sf32lb_uart_async_data,
							 rx);
	unsigned int key;
	uint8_t byte;

	while (true) {
		key = irq_lock();
		if (!rx->irq_mode || rx->stopping || rx->buf == NULL ||
			ring_buf_get(&rx->irq_fifo, &byte, 1U) != 1U) {
			irq_unlock(key);
			break;
		}

		rx->buf[0] = byte;
		rx->offset = 0U;
		uart_sf32lb_async_rx_complete(async->uart_dev);
		irq_unlock(key);
	}
}


static void uart_sf32lb_dma_rx_done(const struct device *dma_dev, void *user_data, uint32_t channel,
				    int status)
{
	struct uart_sf32lb_data *data = user_data;
	const struct device *uart_dev = data->async.uart_dev;
	unsigned int key;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		uart_sf32lb_async_rx_defer_stop(uart_dev, status);
		return;
	}

	key = irq_lock();
	if (!IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC)) {
		(void)k_work_cancel_delayable(&data->async.rx.timeout_work);
	}
	if (IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC)) {
		uart_sf32lb_async_rx_flush(uart_dev, status);
		if (data->async.rx.stop_pending) {
			k_work_reschedule(&data->async.rx.timeout_work, K_TICKS(1));
		}
		irq_unlock(key);
		return;
	}

	uart_sf32lb_async_rx_complete(uart_dev);

	irq_unlock(key);
}

static int uart_async_sf32lb_callback_set(const struct device *dev, uart_callback_t callback,
					  void *user_data)
{
	struct uart_sf32lb_data *data = dev->data;

	data->async.cb = callback;
	data->async.user_data = user_data;
#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->irq_callback = NULL;
	data->cb_data = NULL;
#endif

	return 0;
}

static int uart_async_sf32lb_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				       int32_t timeout)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	uint8_t *dma_alloc = NULL;
	uint8_t *dma_buf = NULL;
	size_t dma_len = 0U;
	unsigned int key;
	struct dma_status dma_stat = {0};
	struct uart_event evt = {0};
	int start_ret;

	if (buf == NULL || len == 0U) {
		return -EINVAL;
	}

	(void)sf32lb_dma_get_status_dt(&config->rx_dma, &dma_stat);
	if (data->async.rx.enabled || dma_stat.busy) {
		return -EBUSY;
	}

	(void)k_work_cancel_delayable(&data->async.rx.timeout_work);

#if defined(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC)
	{
		dma_len = CONFIG_UART_SF32LB_ASYNC_RX_BUFFER_SIZE;
		dma_alloc = k_malloc(dma_len + UART_SF32LB_RX_DMA_ALLOC_PAD);
		if (dma_alloc == NULL) {
			LOG_ERR("%s: failed to allocate %zu-byte cyclic RX buffer", dev->name,
				dma_len);
			return -ENOMEM;
		}
		dma_buf = (uint8_t *)ROUND_UP((uintptr_t)dma_alloc,
						UART_SF32LB_RX_DMA_ALIGN);
	}
#endif

	key = irq_lock();

	k_work_cancel(&data->async.rx.irq_work);
	data->async.rx.buf = buf;
	data->async.rx.len = len;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.reported = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	data->async.rx.dma_buf = dma_buf;
	data->async.rx.dma_alloc = dma_alloc;
	data->async.rx.dma_len = dma_len;
	data->async.rx.dma_offset = 0U;
	data->async.rx.irq_mode = !IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC) &&
		(len == 1U);
	data->async.rx.stop_pending = false;
	data->async.rx.stopping = false;
	data->async.rx.stop_reason = 0;
	ring_buf_reset(&data->async.rx.irq_fifo);
	data->async.rx.timeout = timeout;
	ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
	ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
	ll_usart_disable_it_error(uart_sf32lb_regs(config));
	ll_usart_request_rxdata_flush(uart_sf32lb_regs(config));
	ll_usart_clear_flag_idle(uart_sf32lb_regs(config));

	if (data->async.rx.irq_mode) {
		ll_usart_enable_it_rxne(uart_sf32lb_regs(config));
	} else {
		start_ret = uart_sf32lb_async_rx_start_dma(dev);
		if (start_ret != 0) {
			ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
			ll_usart_disable_it_idle(uart_sf32lb_regs(config));
			ll_usart_disable_it_error(uart_sf32lb_regs(config));
			(void)sf32lb_dma_stop_dt(&config->rx_dma);
			k_free(dma_alloc);
			data->async.rx.buf = NULL;
			data->async.rx.len = 0U;
			data->async.rx.next_buf = NULL;
			data->async.rx.next_len = 0U;
			data->async.rx.offset = 0U;
			data->async.rx.counter = 0U;
			data->async.rx.reported = 0U;
			data->async.rx.dma_buf = NULL;
			data->async.rx.dma_alloc = NULL;
			data->async.rx.dma_len = 0U;
			data->async.rx.dma_offset = 0U;
			data->async.rx.irq_mode = false;
			data->async.rx.stop_pending = false;
			data->async.rx.stopping = false;
			data->async.rx.timeout = 0;
			data->async.rx.enabled = false;
			ring_buf_reset(&data->async.rx.irq_fifo);
			irq_unlock(key);
			return start_ret;
		}

		ll_usart_enable_dma_rx(uart_sf32lb_regs(config));
	}
	data->async.rx.enabled = true;
	ll_usart_enable_it_idle(uart_sf32lb_regs(config));
	ll_usart_enable_it_error(uart_sf32lb_regs(config));

	/* Request next user buffer */
	evt.type = UART_RX_BUF_REQUEST;
	if (data->async.cb) {
		data->async.cb(dev, &evt, data->async.user_data);
	}

	irq_unlock(key);

	return 0;
}

static int uart_async_sf32lb_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_sf32lb_data *data = dev->data;
	unsigned int key;
	int ret = 0;

	if (buf == NULL || len == 0U) {
		return -EINVAL;
	}

	key = irq_lock();
	if (!data->async.rx.enabled) {
		ret = -EACCES;
	} else if (data->async.rx.next_buf != NULL || data->async.rx.next_len != 0) {
		ret = -EBUSY;
	} else {
		data->async.rx.next_buf = buf;
		data->async.rx.next_len = len;
	}
	irq_unlock(key);

	return ret;
}

static int uart_sf32lb_async_rx_disable_cyclic(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	uint8_t *rx_buf;
	uint8_t *next_buf;
	uint8_t *dma_alloc;
	size_t next_len;
	unsigned int key;
	int err;

	k_work_cancel_delayable(&data->async.rx.timeout_work);
	k_work_cancel(&data->async.rx.irq_work);

	key = irq_lock();
	if (!data->async.rx.enabled || data->async.rx.buf == NULL ||
	    data->async.rx.len == 0U) {
		irq_unlock(key);
		return -EINVAL;
	}

	ll_usart_disable_it_idle(uart_sf32lb_regs(config));
	ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
	ll_usart_disable_it_error(uart_sf32lb_regs(config));
	data->async.rx.stopping = true;
	uart_sf32lb_async_rx_flush(dev, UART_SF32LB_ASYNC_STATUS_TIMEOUT);

	err = sf32lb_dma_stop_dt(&config->rx_dma);
	if (err != 0) {
		irq_unlock(key);
		return err;
	}

	rx_buf = data->async.rx.buf;
	next_buf = data->async.rx.next_buf;
	next_len = data->async.rx.next_len;
	dma_alloc = data->async.rx.dma_alloc;

	data->async.rx.buf = NULL;
	data->async.rx.len = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.reported = 0U;
	data->async.rx.dma_buf = NULL;
	data->async.rx.dma_alloc = NULL;
	data->async.rx.dma_len = 0U;
	data->async.rx.dma_offset = 0U;
	data->async.rx.irq_mode = false;
	data->async.rx.stop_pending = false;
	data->async.rx.stopping = false;
	data->async.rx.enabled = false;
	data->async.rx.stop_reason = 0;
	data->async.rx.timeout = 0;
	ring_buf_reset(&data->async.rx.irq_fifo);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	ll_usart_enable_it_rxne(uart_sf32lb_regs(config));
#endif

	uart_sf32lb_async_rx_released(&data->async, rx_buf);
	if (next_buf != NULL && next_len != 0U) {
		uart_sf32lb_async_rx_released(&data->async, next_buf);
	}
	if (dma_alloc != NULL) {
		k_free(dma_alloc);
	}

	if (data->async.cb) {
		struct uart_event evt = {
			.type = UART_RX_DISABLED,
		};

		data->async.cb(dev, &evt, data->async.user_data);
	}

	irq_unlock(key);
	return 0;
}

static int uart_async_sf32lb_rx_disable(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	unsigned int key;
	struct dma_status dma_stat = {0};
	int err = 0;
	struct uart_event evt = {0};
	int dma_status_ret;
	uint8_t *rx_buf;
	uint8_t *heap_buf;
	uint8_t *next_buf;
	size_t rx_len;
	size_t rx_offset;
	size_t rx_received;
	size_t next_len;
	bool irq_mode;

	if (IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC)) {
		return uart_sf32lb_async_rx_disable_cyclic(dev);
	}

	k_work_cancel_delayable(&data->async.rx.timeout_work);
	k_work_cancel(&data->async.rx.irq_work);

	key = irq_lock();
	if (!data->async.rx.enabled || data->async.rx.buf == NULL || data->async.rx.len == 0U) {
		err = -EINVAL;
		irq_unlock(key);
		return err;
	}

	rx_buf = data->async.rx.buf;
	heap_buf = data->async.rx.dma_alloc;
	rx_len = data->async.rx.len;
	rx_offset = MIN(data->async.rx.offset, rx_len);
	irq_mode = data->async.rx.irq_mode;
	ll_usart_disable_it_idle(uart_sf32lb_regs(config));
	ll_usart_disable_dma_rx(uart_sf32lb_regs(config));
	ll_usart_disable_it_error(uart_sf32lb_regs(config));
	if (irq_mode) {
		ll_usart_disable_it_rxne(uart_sf32lb_regs(config));
		dma_status_ret = 0;
		rx_received = 0U;
		if (ll_usart_is_active_flag_rxne(uart_sf32lb_regs(config))) {
			rx_buf[rx_received++] = ll_usart_receive_data8(uart_sf32lb_regs(config));
		}
	} else {
		dma_status_ret = sf32lb_dma_get_status_dt(&config->rx_dma, &dma_stat);
		rx_received = data->async.rx.offset;
		if (dma_status_ret == 0 && dma_stat.pending_length <= data->async.rx.len) {
			rx_received = MAX(rx_received,
					  data->async.rx.len - dma_stat.pending_length);
		}
	}
	rx_received = MIN(rx_received, rx_len);

	err = sf32lb_dma_stop_dt(&config->rx_dma);
	if (err) {
		LOG_ERR("Error stopping Rx DMA (%d)", err);
		irq_unlock(key);
		return err;
	}
	if (!irq_mode && !IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC) &&
	    rx_received < rx_len &&
	    ll_usart_is_active_flag_rxne(uart_sf32lb_regs(config))) {
		rx_buf[rx_received++] = ll_usart_receive_data8(uart_sf32lb_regs(config));
	}
	if (rx_received < rx_offset) {
		rx_offset = rx_received;
	}

	next_buf = data->async.rx.next_buf;
	next_len = data->async.rx.next_len;

	data->async.rx.buf = NULL;
	data->async.rx.dma_buf = NULL;
	data->async.rx.dma_alloc = NULL;
	data->async.rx.len = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.reported = 0U;
	data->async.rx.irq_mode = false;
	data->async.rx.dma_len = 0U;
	data->async.rx.dma_offset = 0U;
	data->async.rx.stop_pending = false;
	data->async.rx.stopping = false;
	data->async.rx.enabled = false;
	data->async.rx.stop_reason = 0;
	data->async.rx.timeout = 0;
	ring_buf_reset(&data->async.rx.irq_fifo);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	ll_usart_enable_it_rxne(uart_sf32lb_regs(config));
#endif

	/* If any bytes have been received notify RX_RDY */
	evt.type = UART_RX_RDY;
	evt.data.rx.buf = rx_buf;
	evt.data.rx.len = rx_received - rx_offset;
	evt.data.rx.offset = rx_offset;

	if (data->async.cb && evt.data.rx.len) {
		data->async.cb(dev, &evt, data->async.user_data);
	}

	/* Release current buffer */
	evt.type = UART_RX_BUF_RELEASED;
	evt.data.rx_buf.buf = rx_buf;

	if (data->async.cb) {
		data->async.cb(dev, &evt, data->async.user_data);
	}

	/* Release next buffer */
	if (next_buf != NULL && next_len != 0U) {
		evt.type = UART_RX_BUF_RELEASED;
		evt.data.rx_buf.buf = next_buf;
		if (data->async.cb) {
			data->async.cb(dev, &evt, data->async.user_data);
		}

	}
	if (heap_buf != NULL) {
		k_free(heap_buf);
	}
	/* Notify UART_RX_DISABLED */
	evt.type = UART_RX_DISABLED;
	if (data->async.cb) {
		data->async.cb(dev, &evt, data->async.user_data);
	}
	irq_unlock(key);
	return err;
}

static int uart_async_sf32lb_tx(const struct device *dev, const uint8_t *buf, size_t len,
				int32_t timeout)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	unsigned int key;
	struct dma_status dma_stat = {0};
	int reload_ret;
	int start_ret;

	if (!buf || (len == 0U)) {
		return -EINVAL;
	}

	(void)sf32lb_dma_get_status_dt(&config->tx_dma, &dma_stat);
	if (data->async.tx.buf != NULL || dma_stat.busy) {
		LOG_WRN("Tx busy");
		return -EBUSY;
	}

	key = irq_lock();

	data->async.tx.buf = buf;
	data->async.tx.len = len;
	data->async.tx.offset = 0U;
	data->async.tx.dma_len = MIN(sizeof(data->async.tx.dma_buf), len);
	data->async.tx.timeout = timeout;
	memcpy(data->async.tx.dma_buf, buf, data->async.tx.dma_len);

	reload_ret = sf32lb_dma_reload_dt(&config->tx_dma,
					  (uintptr_t)data->async.tx.dma_buf,
					  uart_sf32lb_tdr_addr(config), data->async.tx.dma_len);

	start_ret = sf32lb_dma_start_dt(&config->tx_dma);
	if (reload_ret != 0 || start_ret != 0) {
		data->async.tx.buf = NULL;
		data->async.tx.len = 0U;
		data->async.tx.offset = 0U;
		data->async.tx.dma_len = 0U;
		ll_usart_disable_it_tc(uart_sf32lb_regs(config));
		irq_unlock(key);
		return (reload_ret != 0) ? reload_ret : start_ret;
	}

	uart_sf32lb_async_timer_start(&data->async.tx.timeout_work, timeout);

	ll_usart_clear_flag_tc(uart_sf32lb_regs(config));
	ll_usart_enable_it_tc(uart_sf32lb_regs(config));

	ll_usart_enable_dma_tx(uart_sf32lb_regs(config));

	irq_unlock(key);

	return 0;
}

static int uart_async_sf32lb_tx_abort(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
	struct uart_sf32lb_data *data = dev->data;
	struct uart_event evt = {0};
	struct dma_status dma_stat = {0};
	int err = 0;
	unsigned int key;
	int dma_status_ret;
	const uint8_t *tx_buf;
	size_t tx_offset;
	size_t tx_dma_len;
	size_t tx_len;
	size_t tx_sent_len;

	if (data->async.tx.buf == NULL) {
		return -EFAULT;
	}

	key = irq_lock();
	k_work_cancel_delayable(&data->async.tx.timeout_work);

	ll_usart_disable_dma_tx(uart_sf32lb_regs(config));
	ll_usart_disable_it_tc(uart_sf32lb_regs(config));

	dma_status_ret = sf32lb_dma_get_status_dt(&config->tx_dma, &dma_stat);

	tx_buf = data->async.tx.buf;
	tx_offset = data->async.tx.offset;
	tx_dma_len = data->async.tx.dma_len;
	tx_len = data->async.tx.len;
	tx_sent_len = tx_offset;
	if (dma_status_ret == 0 && tx_dma_len >= dma_stat.pending_length) {
		tx_sent_len += tx_dma_len - dma_stat.pending_length;
	}
	tx_sent_len = MIN(tx_sent_len, tx_len);

	err = sf32lb_dma_stop_dt(&config->tx_dma);
	if (err) {
		LOG_ERR("Error stopping Tx DMA (%d)", err);
		irq_unlock(key);
		return err;
	}

	data->async.tx.buf = NULL;
	data->async.tx.len = 0U;
	data->async.tx.offset = 0U;
	data->async.tx.dma_len = 0U;

	evt.type = UART_TX_ABORTED;
	evt.data.tx.buf = tx_buf;
	evt.data.tx.len = tx_sent_len;

	if (data->async.cb) {
		data->async.cb(dev, &evt, data->async.user_data);
	}

	irq_unlock(key);
	return err;
}

static void uart_sf32lb_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct sf32lb_uart_async_tx *tx =
		CONTAINER_OF(dwork, struct sf32lb_uart_async_tx, timeout_work);
	struct sf32lb_uart_async_data *async = CONTAINER_OF(tx, struct sf32lb_uart_async_data, tx);
	struct uart_sf32lb_data *data = CONTAINER_OF(async, struct uart_sf32lb_data, async);

	uart_async_sf32lb_tx_abort(data->async.uart_dev);
}

static void uart_sf32lb_async_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct sf32lb_uart_async_rx *rx =
		CONTAINER_OF(dwork, struct sf32lb_uart_async_rx, timeout_work);
	struct sf32lb_uart_async_data *async = CONTAINER_OF(rx, struct sf32lb_uart_async_data, rx);
	const struct device *dev = async->uart_dev;
	int stop_reason;
	bool disable;
	unsigned int key;

	key = irq_lock();
	if (!async->rx.enabled) {
		irq_unlock(key);
		return;
	}

	stop_reason = async->rx.stop_reason;
	if (stop_reason != 0) {
		async->rx.stop_reason = 0;
		async->rx.stopping = true;
		irq_unlock(key);

		uart_sf32lb_async_rx_stopped(async, stop_reason);
		if (async->rx.enabled) {
			(void)uart_async_sf32lb_rx_disable(dev);
		}
		return;
	}

	disable = async->rx.stop_pending;
	if (!async->rx.irq_mode) {
		uart_sf32lb_async_rx_flush(dev, UART_SF32LB_ASYNC_STATUS_TIMEOUT);
		disable = async->rx.stop_pending;
	}
	irq_unlock(key);

	if (disable) {
		(void)uart_async_sf32lb_rx_disable(dev);
	}
}
#endif /* CONFIG_UART_ASYNC_API */

static DEVICE_API(uart, uart_sf32lb_api) = {
	.poll_in = uart_sf32lb_poll_in,
	.poll_out = uart_sf32lb_poll_out,
	.err_check = uart_sf32lb_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_sf32lb_configure_set,
	.config_get = uart_sf32lb_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sf32lb_fifo_fill,
	.fifo_read = uart_sf32lb_fifo_read,
	.irq_tx_enable = uart_sf32lb_irq_tx_enable,
	.irq_tx_disable = uart_sf32lb_irq_tx_disable,
	.irq_tx_complete = uart_sf32lb_irq_tx_complete,
	.irq_tx_ready = uart_sf32lb_irq_tx_ready,
	.irq_rx_enable = uart_sf32lb_irq_rx_enable,
	.irq_rx_disable = uart_sf32lb_irq_rx_disable,
	.irq_rx_ready = uart_sf32lb_irq_rx_ready,
	.irq_err_enable = uart_sf32lb_irq_err_enable,
	.irq_err_disable = uart_sf32lb_irq_err_disable,
	.irq_is_pending = uart_sf32lb_irq_is_pending,
	.irq_callback_set = uart_sf32lb_irq_callback_set,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_async_sf32lb_callback_set,
	.rx_enable = uart_async_sf32lb_rx_enable,
	.rx_buf_rsp = uart_async_sf32lb_rx_buf_rsp,
	.rx_disable = uart_async_sf32lb_rx_disable,
	.tx = uart_async_sf32lb_tx,
	.tx_abort = uart_async_sf32lb_tx_abort,
#endif
};

static int uart_sf32lb_init(const struct device *dev)
{
	const struct uart_sf32lb_config *config = dev->config;
#ifdef CONFIG_UART_ASYNC_API
	struct uart_sf32lb_data *data = dev->data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	struct dma_config rx_dma_cfg = {0};
	struct dma_config tx_dma_cfg = {0};
	struct dma_block_config rx_dma_blk = {0};
	struct dma_block_config tx_dma_blk = {0};
#endif
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	if (config->clock.dev != NULL) {
		if (!sf32lb_clock_is_ready_dt(&config->clock)) {
			return -ENODEV;
		}

		ret = sf32lb_clock_control_on_dt(&config->clock);
		if (ret < 0) {
			return ret;
		}
	}

	ret = uart_sf32lb_configure(dev, &config->uart_cfg);
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	data->async.uart_dev = dev;
	ll_usart_disable_it_idle(uart_sf32lb_regs(config));
	ll_usart_disable_it_tc(uart_sf32lb_regs(config));
	k_work_init_delayable(&data->async.tx.timeout_work, uart_sf32lb_async_tx_timeout);
	k_work_init_delayable(&data->async.rx.timeout_work, uart_sf32lb_async_rx_timeout);
	k_work_init(&data->async.rx.irq_work, uart_sf32lb_async_rx_irq_work);
	ring_buf_init(&data->async.rx.irq_fifo,
		      sizeof(data->async.rx.irq_fifo_data), data->async.rx.irq_fifo_data);

	sf32lb_dma_config_init_dt(&config->rx_dma, &rx_dma_cfg);

	rx_dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	rx_dma_cfg.source_data_size = 1U;
	rx_dma_cfg.dest_data_size = 1U;
	rx_dma_cfg.cyclic = IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC);
	rx_dma_cfg.half_complete_callback_en = 0U;
	rx_dma_cfg.complete_callback_en = 1U;
	rx_dma_cfg.dma_callback = uart_sf32lb_dma_rx_done;
	rx_dma_cfg.user_data = (void *)data;
	rx_dma_cfg.block_count = 1U;

	rx_dma_cfg.head_block = &rx_dma_blk;
	rx_dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	rx_dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	rx_dma_blk.source_reload_en = IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC);
	rx_dma_blk.dest_reload_en = IS_ENABLED(CONFIG_UART_SF32LB_ASYNC_RX_CYCLIC);

	ret = sf32lb_dma_config_dt(&config->rx_dma, &rx_dma_cfg);
	if (ret < 0) {
		LOG_ERR("Error configuring Rx DMA (%d)", ret);
		return ret;
	}

	sf32lb_dma_config_init_dt(&config->tx_dma, &tx_dma_cfg);

	tx_dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	tx_dma_cfg.source_data_size = 1U;
	tx_dma_cfg.dest_data_size = 1U;
	tx_dma_cfg.complete_callback_en = 1U;
	tx_dma_cfg.dma_callback = uart_sf32lb_dma_tx_done;
	tx_dma_cfg.user_data = (void *)data;
	tx_dma_cfg.block_count = 1U;

	tx_dma_cfg.head_block = &tx_dma_blk;
	tx_dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	tx_dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	ret = sf32lb_dma_config_dt(&config->tx_dma, &tx_dma_cfg);
	if (ret) {
		LOG_ERR("Error configuring Tx DMA (%d)", ret);
		return ret;
	}
#endif

	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define SF32LB_UART_IRQ_CONFIG(index)                                                            \
	static void uart_sf32lb_irq_config_func_##index(const struct device *dev)                 \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), uart_sf32lb_isr,    \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}
#define SF32LB_UART_IRQ_CONFIG_FIELD(index) .irq_config_func = uart_sf32lb_irq_config_func_##index,
#else
#define SF32LB_UART_IRQ_CONFIG(index)
#define SF32LB_UART_IRQ_CONFIG_FIELD(index)
#endif

#define SF32LB_UART_DEFINE(index)                                                                  \
	SF32LB_UART_IRQ_CONFIG(index);                                                               \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct uart_sf32lb_config uart_sf32lb_cfg_##index = {                         \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET_OR(index, {}),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.uart_cfg =                                                                        \
			{                                                                          \
				.baudrate = DT_INST_PROP(index, current_speed),                    \
				.parity =                                                          \
					DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE),  \
				.stop_bits = DT_INST_ENUM_IDX_OR(index, stop_bits,                 \
								 UART_CFG_STOP_BITS_1),            \
				.data_bits = DT_INST_ENUM_IDX_OR(index, data_bits,                 \
								 UART_CFG_DATA_BITS_8),            \
				.flow_ctrl = DT_INST_PROP(index, hw_flow_control)                  \
						     ? UART_CFG_FLOW_CTRL_RTS_CTS                  \
						     : UART_CFG_FLOW_CTRL_NONE,                    \
			},                                                                         \
		SF32LB_UART_IRQ_CONFIG_FIELD(index)                                                   \
		IF_ENABLED(CONFIG_UART_ASYNC_API,                                                  \
			(.tx_dma = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME(index, tx),                 \
			 .rx_dma = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME(index, rx),))               \
	};                                                                                         \
                                                                                                   \
	static struct uart_sf32lb_data uart_sf32lb_data_##index;                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, uart_sf32lb_init, NULL, &uart_sf32lb_data_##index,            \
			      &uart_sf32lb_cfg_##index, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_sf32lb_api);

DT_INST_FOREACH_STATUS_OKAY(SF32LB_UART_DEFINE)
