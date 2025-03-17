#include "uart_sc16is7xx.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT nxp_sc16is7xx_gpio

struct sc16is7xx_gpio_config
{
	/* gpio_driver_config needs to be first field */
	struct gpio_driver_config common;

	const struct device *bus;
};

struct sc16is7xx_gpio_data
{
	/* gpio_driver_data needs to be first field */
	struct gpio_driver_data common;

	struct k_mutex lock;
	struct sc16is7xx_callback callback;
	sys_slist_t callbacks;

	/*
	 * Shadow register
	 */
	uint8_t iocontrol;
	uint8_t iodir;
	uint8_t iointena;
	uint8_t iostate;
};

LOG_MODULE_REGISTER(sc16is7xx_gpio, CONFIG_GPIO_LOG_LEVEL);

static const struct sc16is7xx_gpio_config *get_config(const struct device *dev)
{
	return dev->config;
}

static struct sc16is7xx_gpio_data *get_data(const struct device *dev)
{
	return dev->data;
}

static int read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return sc16is7xx_read(get_config(dev)->bus, (reg << SC16IS7XX_REG_pos), val, 1);
}

static int write(const struct device *dev, uint8_t reg, const uint8_t *val)
{
	return sc16is7xx_write(get_config(dev)->bus, (reg << SC16IS7XX_REG_pos), val, 1);
}

static int sc16is7xx_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
                                              gpio_port_value_t value)
{
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	for (int i = 0; i < 8; i++) {
		if (mask & BIT(i)) {
			WRITE_BIT(get_data(dev)->iostate, i, value & BIT(i));
		}
	}

	ret = write(dev, SC16IS7XX_REG_IOSTATE, &get_data(dev)->iostate);
	if (ret < 0) {
		k_mutex_unlock(&get_data(dev)->lock);
		return -EIO;
	}

	k_mutex_unlock(&get_data(dev)->lock);

	return 0;
}

static int sc16is7xx_gpio_pin_configure(const struct device *dev, gpio_pin_t pin,
                                        gpio_flags_t flags)
{
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	WRITE_BIT(get_data(dev)->iodir, pin, flags & GPIO_OUTPUT ? 1 : 0);

	ret = write(dev, SC16IS7XX_REG_IODIR, &get_data(dev)->iodir);
	if (ret < 0) {
		k_mutex_unlock(&get_data(dev)->lock);
		return -EIO;
	}

	if (flags & GPIO_OUTPUT) {
		ret = sc16is7xx_gpio_port_set_masked_raw(
			dev, BIT(pin), flags & GPIO_OUTPUT_INIT_HIGH ? BIT(pin) : 0);
		if (ret < 0) {
			k_mutex_unlock(&get_data(dev)->lock);
			return -EIO;
		}
	}

	k_mutex_unlock(&get_data(dev)->lock);

	return 0;
}

static int sc16is7xx_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	int ret;

	uint8_t iostate;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	ret = read(dev, SC16IS7XX_REG_IOSTATE, &iostate);
	if (ret < 0) {
		k_mutex_unlock(&get_data(dev)->lock);
		return -EIO;
	}

	*value = iostate;

	k_mutex_unlock(&get_data(dev)->lock);

	return 0;
}

static int sc16is7xx_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return sc16is7xx_gpio_port_set_masked_raw(dev, pins, pins);
}

static int sc16is7xx_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return sc16is7xx_gpio_port_set_masked_raw(dev, pins, 0);
}

static int sc16is7xx_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
                                                  enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	int ret;

	if ((mode != GPIO_INT_MODE_EDGE) && (mode != GPIO_INT_MODE_DISABLED)) {
		return -ENOTSUP;
	}

	if (trig != GPIO_INT_TRIG_HIGH) {
		return -ENOTSUP;
	}

	WRITE_BIT(get_data(dev)->iointena, pin, mode == GPIO_INT_MODE_EDGE ? 1 : 0);

	ret = write(dev, SC16IS7XX_REG_IOINTENA, &get_data(dev)->iointena);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt: %d", ret);
		return ret;
	}

	return 0;
}

static int sc16is7xx_gpio_manage_callback(const struct device *dev, struct gpio_callback *callback,
                                          bool set)
{
	return gpio_manage_callback(&get_data(dev)->callbacks, callback, set);
}

static void sc16is7xx_callback_handler(const struct device *dev)
{
	int ret;

	uint8_t iir;

	ret = read(dev, SC16IS7XX_REG_IIR, &iir);
	if (ret < 0) {
		LOG_ERR("Reading IIR failed: %d", ret);
		return;
	}

	if ((iir & SC16IS7XX_IIR_INTERRUPT_msk) != SC16IS7XX_IIR_IPCS_msk) {
		return;
	}

	uint8_t iostate;

	ret = read(dev, SC16IS7XX_REG_IOSTATE, &iostate);
	if (ret < 0) {
		LOG_ERR("Failed to read I/O state: %d", ret);
		return;
	}

	iostate &= get_data(dev)->iointena;

	gpio_fire_callbacks(&get_data(dev)->callbacks, dev, iostate);
}

static int sc16is7xx_gpio_init(const struct device *dev)
{
	int ret;

	if (!device_is_ready(get_config(dev)->bus)) {
		LOG_ERR("bus not ready");
		return -ENODEV;
	}

	sc16is7xx_register_callback(get_config(dev)->bus, &get_data(dev)->callback);

	WRITE_BIT(get_data(dev)->iocontrol, SC16IS7XX_IOCONTROL_IOLATCH_pos, 1);
	WRITE_BIT(get_data(dev)->iocontrol, SC16IS7XX_IOCONTROL_GPIO_7_4_pos, 0);
	WRITE_BIT(get_data(dev)->iocontrol, SC16IS7XX_IOCONTROL_GPIO_3_0_pos, 0);

	ret = write(dev, SC16IS7XX_REG_IOCONTROL, &get_data(dev)->iocontrol);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static const struct gpio_driver_api api = {
	.pin_configure = sc16is7xx_gpio_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	pin_get_config = NULL,
#endif
	.port_get_raw = sc16is7xx_gpio_port_get_raw,
	.port_set_masked_raw = sc16is7xx_gpio_port_set_masked_raw,
	.port_set_bits_raw = sc16is7xx_gpio_port_set_bits_raw,
	.port_clear_bits_raw = sc16is7xx_gpio_port_clear_bits_raw,
	.port_toggle_bits = NULL,
	.pin_interrupt_configure = sc16is7xx_gpio_pin_interrupt_configure,
	.manage_callback = sc16is7xx_gpio_manage_callback,
	.get_pending_int = NULL,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = NULL,
#endif /* CONFIG_GPIO_GET_DIRECTION */
};

#define DT_INST_INIT(inst)                                                                         \
	static struct sc16is7xx_gpio_data inst_##inst##_data = {                                   \
		.lock = Z_MUTEX_INITIALIZER(inst_##inst##_data.lock),                              \
		.callback = {.dev = DEVICE_DT_INST_GET(inst),                                      \
			     .handler = sc16is7xx_callback_handler}};                              \
	static const struct sc16is7xx_gpio_config inst_##inst##_config = {                         \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst)},                \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst))};                                          \
	DEVICE_DT_INST_DEFINE(inst, sc16is7xx_gpio_init, NULL, &inst_##inst##_data,                \
			      &inst_##inst##_config, POST_KERNEL,                                  \
			      CONFIG_UART_SC16IS7XX_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(DT_INST_INIT)

//BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1);
