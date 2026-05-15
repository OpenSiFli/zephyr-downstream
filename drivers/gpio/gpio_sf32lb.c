/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_gpio

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/irq.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <ll_gpio.h>
#include <ll_pinmux.h>

#define PINMUX_PAD_XXYY_MODE_I2C BIT(8)

struct gpio_sf32lb_config {
	struct gpio_driver_config common;
	uintptr_t gpio;
	uintptr_t pinmux;
	uint8_t pad_base;
};

struct gpio_sf32lb_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	gpio_port_pins_t od;
};

static bool shared_initialized;
static const struct device *controllers[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_INST_PARENT(0), DEVICE_DT_GET, (,)),
};

BUILD_ASSERT((DT_NODE_HAS_COMPAT(DT_INST_PARENT(0), sifli_sf32lb_gpio_parent)) &&
		     (DT_NUM_INST_STATUS_OKAY(sifli_sf32lb_gpio_parent) == 1),
	     "Only one parent instance is supported");

static inline bool gpio_sf32lb_pad_has_mode_bit(const struct gpio_sf32lb_config *config,
						gpio_pin_t pin)
{
	uint8_t abs_pin = config->pad_base + pin;

	return (abs_pin >= 39U) && (abs_pin <= 42U);
}

static void gpio_sf32lb_irq(const void *arg)
{
	for (size_t c = 0U; c < ARRAY_SIZE(controllers); c++) {
		const struct gpio_sf32lb_config *config = controllers[c]->config;
		struct gpio_sf32lb_data *data = controllers[c]->data;
		ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
		uint8_t min, max;
		uint32_t val;

		min = u32_count_trailing_zeros(config->common.port_pin_mask);
		max = 32 - u32_count_leading_zeros(config->common.port_pin_mask);

		val = ll_gpio_bank_get_irq_pending(bank);
		for (uint8_t i = min; i < max; i++) {
			if ((val & BIT(i)) != 0U) {
				gpio_fire_callbacks(&data->callbacks, controllers[c], BIT(i));
			}
		}
		ll_gpio_bank_clear_irq_pending(bank, val);
	}
}

static inline int gpio_sf32lb_configure(const struct device *port, gpio_pin_t pin,
					gpio_flags_t flags)
{
	const struct gpio_sf32lb_config *config = port->config;
	struct gpio_sf32lb_data *data = port->data;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	HPSYS_PINMUX_TypeDef *pinmux = (HPSYS_PINMUX_TypeDef *)config->pinmux;
	bool has_mode_bit = gpio_sf32lb_pad_has_mode_bit(config, pin);
	uint32_t pin_mask = BIT(pin);
	uint32_t pull = LL_PINMUX_PULL_NONE;

	if ((flags & GPIO_OUTPUT) != 0U) {
		/* disable ISR */
		ll_gpio_bank_disable_irq(bank, pin_mask);

		if ((flags & GPIO_SINGLE_ENDED) != 0U) {
			if ((flags & GPIO_LINE_OPEN_DRAIN) == 0U) {
				return -ENOTSUP;
			}

			data->od |= BIT(pin);

			/* disable O */
			ll_gpio_bank_set_low(bank, pin_mask);

			/* set initial state (OE) */
			if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
				ll_gpio_bank_enable_output(bank, pin_mask);
			} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
				ll_gpio_bank_disable_output(bank, pin_mask);
			}
		} else {
			data->od &= ~BIT(pin);

			/* enable OE */
			ll_gpio_bank_enable_output(bank, pin_mask);

			/* set initial state (O) */
			if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
				ll_gpio_bank_set_high(bank, pin_mask);
			} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
				ll_gpio_bank_set_low(bank, pin_mask);
			}
		}
	} else if ((flags & GPIO_INPUT) != 0U) {
		data->od &= ~BIT(pin);

		/* disable OE */
		ll_gpio_bank_disable_output(bank, pin_mask);
	} else {
		return -ENOTSUP;
	}

	/* configure pad settings in PINMUX */
	ll_pinmux_set_fsel(pinmux, pin, 0U);
	if (has_mode_bit) {
		/* LL gap: PA39-PA42 reuse this bit as I2C mode, not generic slew. */
		sys_clear_bits((mem_addr_t)ll_pinmux_get_pad_reg(pinmux, pin),
			       PINMUX_PAD_XXYY_MODE_I2C);
	}

	if ((flags & GPIO_INPUT) != 0U) {
		ll_pinmux_enable_input(pinmux, pin);
	} else {
		ll_pinmux_disable_input(pinmux, pin);
	}

	if ((flags & GPIO_PULL_UP) != 0U) {
		pull = LL_PINMUX_PULL_UP;
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		pull = LL_PINMUX_PULL_DOWN;
	}

	ll_pinmux_config_pull(pinmux, pin, pull);

	return 0;
}

static int gpio_sf32lb_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_sf32lb_config *config = port->config;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;

	*value = ll_gpio_bank_read_input_port(bank);

	return 0;
}

static int gpio_sf32lb_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	const struct gpio_sf32lb_config *config = port->config;
	struct gpio_sf32lb_data *data = port->data;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	gpio_port_pins_t pp_mask, od_mask;

	pp_mask = mask & ~data->od;
	if (pp_mask != 0U) {
		ll_gpio_bank_set_high(bank, value & pp_mask);
		ll_gpio_bank_set_low(bank, (~value) & pp_mask);
	}

	od_mask = mask & data->od;
	if (od_mask != 0U) {
		ll_gpio_bank_enable_output(bank, value & od_mask);
		ll_gpio_bank_disable_output(bank, (~value) & od_mask);
	}

	return 0;
}

static int gpio_sf32lb_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_sf32lb_config *config = port->config;
	struct gpio_sf32lb_data *data = port->data;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	gpio_port_pins_t pp_pins, od_pins;

	pp_pins = pins & ~data->od;
	ll_gpio_bank_set_high(bank, pp_pins);

	od_pins = pins & data->od;
	ll_gpio_bank_enable_output(bank, od_pins);

	return 0;
}

static int gpio_sf32lb_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_sf32lb_config *config = port->config;
	struct gpio_sf32lb_data *data = port->data;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	gpio_port_pins_t pp_pins, od_pins;

	pp_pins = pins & ~data->od;
	ll_gpio_bank_set_low(bank, pp_pins);

	od_pins = pins & data->od;
	ll_gpio_bank_disable_output(bank, od_pins);

	return 0;
}

static int gpio_sf32lb_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_sf32lb_config *config = port->config;
	struct gpio_sf32lb_data *data = port->data;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	gpio_port_pins_t pp_pins, od_pins;
	uint32_t val;

	pp_pins = pins & ~data->od;
	if (pp_pins != 0U) {
		ll_gpio_bank_toggle(bank, pp_pins);
	}

	od_pins = pins & data->od;
	if (od_pins != 0U) {
		val = ll_gpio_bank_is_output_enabled(bank, od_pins);
		ll_gpio_bank_enable_output(bank, (~val) & od_pins);
		ll_gpio_bank_disable_output(bank, val & od_pins);
	}

	return 0;
}

static int gpio_sf32lb_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_sf32lb_config *config = port->config;
	ll_gpio_bank_t *bank = (ll_gpio_bank_t *)config->gpio;
	ll_gpio_irq_config_t irq_config;
	uint32_t pin_mask = BIT(pin);

	if (mode == GPIO_INT_MODE_DISABLED) {
		ll_gpio_bank_disable_irq(bank, pin_mask);
	} else if ((mode == GPIO_INT_MODE_EDGE) || (mode == GPIO_INT_MODE_LEVEL)) {
		if (mode == GPIO_INT_MODE_EDGE) {
			irq_config.type = LL_GPIO_IRQ_TYPE_EDGE;
		} else {
			irq_config.type = LL_GPIO_IRQ_TYPE_LEVEL;
		}

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			irq_config.polarity = LL_GPIO_IRQ_POL_LOW;
			break;
		case GPIO_INT_TRIG_HIGH:
			irq_config.polarity = LL_GPIO_IRQ_POL_HIGH;
			break;
		case GPIO_INT_TRIG_BOTH:
			irq_config.polarity = LL_GPIO_IRQ_POL_BOTH;
			break;
		default:
			return -ENOTSUP;
		}

		ll_gpio_bank_config_irq_trigger(bank, pin_mask, &irq_config);
		ll_gpio_bank_enable_irq(bank, pin_mask);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_sf32lb_manage_callback(const struct device *dev, struct gpio_callback *callback,
				       bool set)
{
	struct gpio_sf32lb_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static DEVICE_API(gpio, gpio_sf32lb_api) = {
	.pin_configure = gpio_sf32lb_configure,
	.port_get_raw = gpio_sf32lb_port_get_raw,
	.port_set_masked_raw = gpio_sf32lb_port_set_masked_raw,
	.port_set_bits_raw = gpio_sf32lb_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sf32lb_port_clear_bits_raw,
	.port_toggle_bits = gpio_sf32lb_port_toggle_bits,
	.pin_interrupt_configure = gpio_sf32lb_pin_interrupt_configure,
	.manage_callback = gpio_sf32lb_manage_callback,
};

static int gpio_sf32lb_init(const struct device *dev)
{
	if (!shared_initialized) {
		struct sf32lb_clock_dt_spec clk = SF32LB_CLOCK_DT_SPEC_GET(DT_INST_PARENT(0));

		if (!sf32lb_clock_is_ready_dt(&clk)) {
			return -ENODEV;
		}

		(void)sf32lb_clock_control_on_dt(&clk);

		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(0)), DT_IRQ(DT_INST_PARENT(0), priority),
			    gpio_sf32lb_irq, NULL, 0);
		irq_enable(DT_IRQN(DT_INST_PARENT(0)));

		shared_initialized = true;
	}

	return 0;
}

#define GPIO_SF32LB_DEFINE(n)                                                                      \
	static const struct gpio_sf32lb_config gpio_sf32lb_config##n = {                           \
		.common = GPIO_COMMON_CONFIG_FROM_DT_INST(n),                                      \
		.gpio = DT_INST_REG_ADDR(n),                                                       \
		.pinmux = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE(n, sifli_pinmuxs),                    \
					     DT_INST_PHA(n, sifli_pinmuxs, port)) +                \
			  DT_INST_PHA(n, sifli_pinmuxs, offset),                                   \
		.pad_base = DT_INST_PHA(n, sifli_pinmuxs, offset) / 4U,                           \
	};                                                                                         \
                                                                                                   \
	static struct gpio_sf32lb_data gpio_sf32lb_data##n;                                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_sf32lb_init, NULL, &gpio_sf32lb_data##n,                     \
			      &gpio_sf32lb_config##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &gpio_sf32lb_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_SF32LB_DEFINE)
