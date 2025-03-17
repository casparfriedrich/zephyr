#include "uart_sc16is7xx.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT nxp_sc16is7xx

typedef int (*sc16is7xx_read_t)(const struct device *dev, uint8_t cmd, uint8_t *data,
                                size_t data_len);

typedef int (*sc16is7xx_write_t)(const struct device *dev, uint8_t cmd, const uint8_t *data,
                                 size_t data_len);

typedef bool (*sc16is7xx_is_ready_t)(const struct device *dev);

union sc16is7xx_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct sc16is7xx_config {
	union sc16is7xx_bus bus;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec reset;
	int clock_frequency;
	sc16is7xx_read_t read;
	sc16is7xx_write_t write;
	sc16is7xx_is_ready_t is_ready;
	bool use_software_reset;
};

struct sc16is7xx_data {
	struct gpio_callback gpio_handler;
	sys_slist_t child_callbacks;
};

LOG_MODULE_REGISTER(sc16is7xx, CONFIG_UART_LOG_LEVEL);

static const struct sc16is7xx_config *get_config(const struct device *dev) {
	return dev->config;
}

static struct sc16is7xx_data *get_data(const struct device *dev) {
	return dev->data;
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int sc16is7xx_read_i2c(const struct device *dev, uint8_t cmd, uint8_t *data, size_t data_len) {
	return i2c_burst_read_dt(&get_config(dev)->bus.i2c, cmd, data, data_len);
}

static int sc16is7xx_write_i2c(const struct device *dev, uint8_t cmd, const uint8_t *data,
                               size_t data_len) {
	return i2c_burst_write_dt(&get_config(dev)->bus.i2c, cmd, data, data_len);
}

static bool sc16is7xx_is_ready_i2c(const struct device *dev) {
	return i2c_is_ready_dt(&get_config(dev)->bus.i2c);
}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int sc16is7xx_read_spi(const struct device *dev, uint8_t cmd, uint8_t *data, size_t data_len) {
	const struct spi_buf tx_bufs[] = {
		{.buf = &cmd, .len = sizeof(cmd)},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	const struct spi_buf rx_bufs[] = {
		{.buf = NULL, .len = sizeof(cmd)},
		{.buf = data, .len = data_len},
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	WRITE_BIT(cmd, SC16IS7XX_RW_pos, 1);

	return spi_transceive_dt(&get_config(dev)->bus.spi, &tx, &rx);
}

static int sc16is7xx_write_spi(const struct device *dev, uint8_t cmd, const uint8_t *data,
                               size_t data_len) {
	const struct spi_buf tx_bufs[] = {
		{.buf = &cmd, .len = sizeof(cmd)},
		{.buf = (void *) data, .len = data_len},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	return spi_write_dt(&get_config(dev)->bus.spi, &tx);
}

static bool sc16is7xx_is_ready_spi(const struct device *dev) {
	return spi_is_ready_dt(&get_config(dev)->bus.spi);
}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

static int sc16is7xx_read_impl(const struct device *dev, uint8_t reg, uint8_t *data,
                               size_t data_len) {
	return get_config(dev)->read(dev, reg, data, data_len);
}

static int sc16is7xx_write_impl(const struct device *dev, uint8_t reg, const uint8_t *data,
                                size_t data_len) {
	return get_config(dev)->write(dev, reg, data, data_len);
}

static void sc16is7xx_register_callback_impl(const struct device *dev,
                                             struct sc16is7xx_callback *callback) {
	sys_slist_append(&get_data(dev)->child_callbacks, &callback->node);
}

static void sc16is7xx_gpio_handler(const struct device *dev, struct gpio_callback *cb,
                                   uint32_t pins) {
	struct sc16is7xx_data *data = CONTAINER_OF(cb, struct sc16is7xx_data, gpio_handler);
	struct sc16is7xx_callback *callback;

	SYS_SLIST_FOR_EACH_CONTAINER(&data->child_callbacks, callback, node) {
		callback->handler(callback->dev);
	}
}

static int sc16is7xx_configure_interrupt(const struct device *dev) {
	int ret;

	struct sc16is7xx_data *data = dev->data;

	/*
	 * Note: Interrupt signal is optional
	 */
	if (!get_config(dev)->irq.port) {
		return 0;
	}

	if (!gpio_is_ready_dt(&get_config(dev)->irq)) {
		return -ENODEV;
	}

	gpio_init_callback(&data->gpio_handler, sc16is7xx_gpio_handler,
	                   BIT(get_config(dev)->irq.pin));

	ret = gpio_add_callback_dt(&get_config(dev)->irq, &data->gpio_handler);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&get_config(dev)->irq, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	return 0;
}

static int sc16is7xx_enable_interrupt(const struct device *dev) {
	int ret;

	ret = gpio_pin_interrupt_configure_dt(&get_config(dev)->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("`gpio_pin_interrupt_configure_dt` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int sc16is7xx_configure_reset(const struct device *dev) {
	int ret;

	/*
	 * Reset signal is optional
	 */
	if (!get_config(dev)->reset.port) {
		return 0;
	}

	if (!gpio_is_ready_dt(&get_config(dev)->reset)) {
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&get_config(dev)->reset, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	return 0;
}

static int sc16is7xx_reset_device(const struct device *dev) {
	int ret;

	if (get_config(dev)->use_software_reset) {
		const uint8_t reg = SC16IS7XX_IOCONTROL_SRESET_msk;
		ret = sc16is7xx_write_impl(dev, SC16IS7XX_REG_IOCONTROL, &reg, 1);
		if (ret) {
			// return ret;
		}
	} else if (get_config(dev)->reset.port) {
		ret = gpio_pin_set_dt(&get_config(dev)->reset, 1);
		if (ret) {
			return ret;
		}

		k_usleep(SC16IS7XX_RESET_PULSE_us);

		ret = gpio_pin_set_dt(&get_config(dev)->reset, 0);
		if (ret) {
			return ret;
		}
	} else {
		return 0;
	}

	k_usleep(SC16IS7XX_RESET_DELAY_us);

	return 0;
}

static int sc16is7xx_init(const struct device *dev) {
	int ret;

	sys_slist_init(&get_data(dev)->child_callbacks);

	if (!get_config(dev)->is_ready(dev)) {
		LOG_ERR("bus not ready");
		return -ENODEV;
	}

	ret = sc16is7xx_configure_reset(dev);
	if (ret) {
		return ret;
	}

	ret = sc16is7xx_configure_interrupt(dev);
	if (ret) {
		return ret;
	}

	ret = sc16is7xx_reset_device(dev);
	if (ret) {
		return ret;
	}

	ret = sc16is7xx_enable_interrupt(dev);
	if (ret) {
		return ret;
	}

	/*
	 * Todo: Check if device is operational (Scratchpad Register (SPR))
	 */

	return 0;
}

static const struct sc16is7xx_api api = {
	.read = sc16is7xx_read_impl,
	.write = sc16is7xx_write_impl,
	.register_callback = sc16is7xx_register_callback_impl,
};

#define SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define SPI_DELAY     0

#define DT_INST_INIT(inst)                                                                         \
	static struct sc16is7xx_data inst_##inst##_data;                                           \
	static const struct sc16is7xx_config inst_##inst##_config = {                              \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (.bus.i2c = I2C_DT_SPEC_INST_GET(inst)),    \
			    (.bus.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OPERATION, SPI_DELAY))),    \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (.read = sc16is7xx_read_i2c),               \
			    (.read = sc16is7xx_read_spi)),                                         \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (.write = sc16is7xx_write_i2c),             \
			    (.write = sc16is7xx_write_spi)),                                       \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (.is_ready = sc16is7xx_is_ready_i2c),       \
			    (.is_ready = sc16is7xx_is_ready_spi)),                                 \
		.irq = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),                             \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
		.use_software_reset = DT_INST_PROP(inst, use_software_reset),                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, sc16is7xx_init, NULL, &inst_##inst##_data,                     \
			      &inst_##inst##_config, POST_KERNEL,                                  \
			      CONFIG_UART_SC16IS7XX_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(DT_INST_INIT)

// #if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
// BUILD_ASSERT(CONFIG_UART_SC16IS7XX_INIT_PRIORITY >= CONFIG_I2C_INIT_PRIORITY);
// #endif

// #if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
// BUILD_ASSERT(CONFIG_UART_SC16IS7XX_INIT_PRIORITY >= CONFIG_SPI_INIT_PRIORITY);
// #endif
