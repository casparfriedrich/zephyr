#include "uart_sc16is7xx.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stdlib.h>

#define DT_DRV_COMPAT nxp_sc16is7xx_uart

#define ABS_DIFF(a, b) (MAX(a, b) - MIN(a, b))

struct sc16is7xx_uart_config {
	const struct device *bus;
	int channel;
	int clock_frequency;
};

struct sc16is7xx_uart_data {
	const struct device *self;
	struct k_mutex lock;
	struct sc16is7xx_callback callback;
	struct uart_config uart_config;
	struct k_work callback_work;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_callback;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	/*
	 * Shadow register
	 */
	uint8_t dlh;
	uint8_t dll;
	uint8_t efcr;
	uint8_t efr;
	uint8_t fcr;
	uint8_t ier;
	uint8_t iir;
	uint8_t lcr;
	uint8_t mcr;
};

LOG_MODULE_REGISTER(sc16is7xx_uart, CONFIG_UART_LOG_LEVEL);

static const struct sc16is7xx_uart_config *get_config(const struct device *dev) {
	return dev->config;
}

static struct sc16is7xx_uart_data *get_data(const struct device *dev) {
	return dev->data;
}

static int read(const struct device *dev, uint8_t reg, uint8_t *val) {
	const uint8_t cmd =
		(reg << SC16IS7XX_REG_pos) | (get_config(dev)->channel << SC16IS7XX_CH_pos);

	return sc16is7xx_read(get_config(dev)->bus, cmd, val, 1);
}

static int read_fifo(const struct device *dev, uint8_t *data, size_t data_len) {
	const uint8_t cmd = (SC16IS7XX_REG_RHR << SC16IS7XX_REG_pos) |
	                    (get_config(dev)->channel << SC16IS7XX_CH_pos);

	return sc16is7xx_read(get_config(dev)->bus, cmd, data, data_len);
}

static int write(const struct device *dev, uint8_t reg, const uint8_t *val) {
	const uint8_t cmd =
		(reg << SC16IS7XX_REG_pos) | (get_config(dev)->channel << SC16IS7XX_CH_pos);

	return sc16is7xx_write(get_config(dev)->bus, cmd, val, 1);
}

static int write_fifo(const struct device *dev, const uint8_t *data, size_t data_len) {
	const uint8_t cmd = (SC16IS7XX_REG_THR << SC16IS7XX_REG_pos) |
	                    (get_config(dev)->channel << SC16IS7XX_CH_pos);

	return sc16is7xx_write(get_config(dev)->bus, cmd, data, data_len);
}

static int sc16is7xx_uart_update_efr(const struct device *dev) {
	int ret;

	static const uint8_t lcr_magic = SC16IS7XX_LCR_MAGIC;

	/*
	 * Ensure enhanced features are enabled
	 */
	WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_EFE_pos, 1);

	/*
	 * Write magic value (0xbf) to LCR in order to unlock enhanced function register (EFR)
	 */
	ret = write(dev, SC16IS7XX_REG_LCR, &lcr_magic);
	if (ret < 0) {
		LOG_ERR("write LCR failed: %d", ret);
		return ret;
	}

	ret = write(dev, SC16IS7XX_REG_EFR, &get_data(dev)->efr);
	if (ret < 0) {
		LOG_ERR("write EFR failed: %d", ret);
		return ret;
	}

	/*
	 * restore LCR
	 */
	ret = write(dev, SC16IS7XX_REG_LCR, &get_data(dev)->lcr);
	if (ret < 0) {
		LOG_ERR("write LCR failed: %d", ret);
		return ret;
	}

	return 0;
}

static int sc16is7xx_uart_configure_fifo(const struct device *dev) {
	int ret;

	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_FIFO_ENABLE_pos, 1);
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_RESET_RX_FIFO_pos, 1);
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_RESET_TX_FIFO_pos, 1);

	/*
	 * Sets the trigger level for the RX FIFO to 16 characters
	 * TODO: Make configurable
	 */
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_RX_TRIGGER_LSB_pos, 1);
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_RX_TRIGGER_MSB_pos, 0);

	/*
	 * Sets the trigger level for the TX FIFO to 16 spaces
	 * TODO: Make configurable
	 */
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_TX_TRIGGER_LSB_pos, 1);
	WRITE_BIT(get_data(dev)->fcr, SC16IS7XX_FCR_TX_TRIGGER_MSB_pos, 0);

	ret = sc16is7xx_uart_update_efr(dev);
	if (ret < 0) {
		return ret;
	}

	ret = write(dev, SC16IS7XX_REG_FCR, &get_data(dev)->fcr);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int sc16is7xx_uart_configure_line(const struct device *dev, const struct uart_config *cfg) {
	int ret;

	switch (cfg->flow_ctrl) {
		case UART_CFG_FLOW_CTRL_NONE:
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_RTS_pos, 0);
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_CTS_pos, 0);
			WRITE_BIT(get_data(dev)->efcr, SC16IS7XX_EFCR_RTS_CONTROL_pos, 0);
			break;
		case UART_CFG_FLOW_CTRL_RTS_CTS:
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_RTS_pos, 1);
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_CTS_pos, 1);
			WRITE_BIT(get_data(dev)->efcr, SC16IS7XX_EFCR_RTS_CONTROL_pos, 0);
			break;
		case UART_CFG_FLOW_CTRL_DTR_DSR:
			return -ENOTSUP;
		case UART_CFG_FLOW_CTRL_RS485:
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_RTS_pos, 0);
			WRITE_BIT(get_data(dev)->efr, SC16IS7XX_EFR_CTS_pos, 0);
			WRITE_BIT(get_data(dev)->efcr, SC16IS7XX_EFCR_RTS_CONTROL_pos, 1);
			break;
		default:
			return -EINVAL;
	}

	switch (cfg->data_bits) {
		case UART_CFG_DATA_BITS_5:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_LSB_pos, 0);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_MSB_pos, 0);
			break;
		case UART_CFG_DATA_BITS_6:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_LSB_pos, 1);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_MSB_pos, 0);
			break;
		case UART_CFG_DATA_BITS_7:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_LSB_pos, 0);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_MSB_pos, 1);
			break;
		case UART_CFG_DATA_BITS_8:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_LSB_pos, 1);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_WORDLEN_MSB_pos, 1);
			break;
		case UART_CFG_DATA_BITS_9:
			return -ENOTSUP;
		default:
			return -EINVAL;
	}

	switch (cfg->parity) {
		case UART_CFG_PARITY_NONE:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_ENABLE_pos, 0);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_TYPE_pos, 0);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_FORCE_PARITY_pos, 0);
			break;
		case UART_CFG_PARITY_ODD:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_ENABLE_pos, 1);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_TYPE_pos, 0);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_FORCE_PARITY_pos, 0);
			break;
		case UART_CFG_PARITY_EVEN:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_ENABLE_pos, 1);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_PARITY_TYPE_pos, 1);
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_FORCE_PARITY_pos, 0);
			break;
		case UART_CFG_PARITY_MARK:
		case UART_CFG_PARITY_SPACE:
			return -ENOTSUP;
		default:
			return -EINVAL;
	}

	switch (cfg->stop_bits) {
		case UART_CFG_STOP_BITS_0_5:
			return -ENOTSUP;
		case UART_CFG_STOP_BITS_1:
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_STOPLEN_pos, 0);
			break;
		case UART_CFG_STOP_BITS_1_5:
			if (cfg->data_bits != UART_CFG_DATA_BITS_5) {
				return -EINVAL;
			}
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_STOPLEN_pos, 1);
			break;
		case UART_CFG_STOP_BITS_2:
			if (cfg->data_bits == UART_CFG_DATA_BITS_5) {
				return -EINVAL;
			}
			WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_STOPLEN_pos, 1);
			break;
		default:
			return -EINVAL;
	}

	ret = sc16is7xx_uart_update_efr(dev);
	if (ret < 0) {
		return ret;
	}

	ret = write(dev, SC16IS7XX_REG_EFCR, &get_data(dev)->efcr);
	if (ret < 0) {
		return ret;
	}

	ret = write(dev, SC16IS7XX_REG_LCR, &get_data(dev)->lcr);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int sc16is7xx_uart_configure_baudrate(const struct device *dev,
                                             const struct uart_config *cfg) {
	int ret;

	const int32_t clock_frequency = get_config(dev)->clock_frequency;
	const int64_t baudrate = cfg->baudrate;

	/*
	 * Calculate clock divider and effective baud rates for both prescaler configurations
	 * Note: The formula can be found in the data sheet
	 */
	const int divider1 = DIV_ROUND_CLOSEST(clock_frequency / 1.0, baudrate * 16.0);
	const int baudrate1 = DIV_ROUND_CLOSEST(clock_frequency, 1.0 * divider1 * 16.0);
	const int divider4 = DIV_ROUND_CLOSEST(clock_frequency / 4.0, baudrate * 16.0);
	const int baudrate4 = DIV_ROUND_CLOSEST(clock_frequency, 4.0 * divider4 * 16.0);

	/*
	 * Check if the effective baud rate with the extra prescaler enabled is closer to the
	 * desired one
	 */
	const bool prescaler = ABS_DIFF(baudrate, baudrate4) < ABS_DIFF(baudrate, baudrate1);

	LOG_INF("Effective baud rate: %d", prescaler ? baudrate4 : baudrate1);

	ret = sc16is7xx_uart_update_efr(dev);
	if (ret < 0) {
		return ret;
	}

	WRITE_BIT(get_data(dev)->mcr, SC16IS7XX_MCR_CLOCK_DIVISOR_pos, prescaler ? 1 : 0);

	ret = write(dev, SC16IS7XX_REG_MCR, &get_data(dev)->mcr);
	if (ret < 0) {
		return ret;
	}

	WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_DIVISOR_LATCH_ENABLE_pos, 1);

	ret = write(dev, SC16IS7XX_REG_LCR, &get_data(dev)->lcr);
	if (ret < 0) {
		return ret;
	}

	get_data(dev)->dll = prescaler ? divider4 : divider1;

	ret = write(dev, SC16IS7XX_REG_DLL, &get_data(dev)->dll);
	if (ret < 0) {
		return ret;
	}

	get_data(dev)->dlh = prescaler ? divider4 : divider1 >> 8;

	ret = write(dev, SC16IS7XX_REG_DLH, &get_data(dev)->dlh);
	if (ret < 0) {
		return ret;
	}

	WRITE_BIT(get_data(dev)->lcr, SC16IS7XX_LCR_DIVISOR_LATCH_ENABLE_pos, 0);

	ret = write(dev, SC16IS7XX_REG_LCR, &get_data(dev)->lcr);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int sc16is7xx_uart_poll_in(const struct device *dev, unsigned char *c) {
	int ret;

	uint8_t rxlvl;

	if (k_is_in_isr()) {
		LOG_ERR("Calling from ISR is not supported");
		return -1;
	}

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	ret = read(dev, SC16IS7XX_REG_RXLVL, &rxlvl);
	if (ret < 0) {
		LOG_ERR("failed to read RX level: %d", ret);
		k_mutex_unlock(&get_data(dev)->lock);
		return -1;
	}

	if (!rxlvl) {
		k_mutex_unlock(&get_data(dev)->lock);
		return -1;
	}

	ret = read_fifo(dev, c, 1);
	if (ret < 0) {
		k_mutex_unlock(&get_data(dev)->lock);
		LOG_ERR("Poll in failed: %d", ret);
		return -1;
	}

	k_mutex_unlock(&get_data(dev)->lock);

	return 0;
}

static void sc16is7xx_uart_poll_out(const struct device *dev, unsigned char c) {
	int ret;

	uint8_t txlvl;

	if (k_is_in_isr()) {
		LOG_ERR("Calling from ISR is not supported");
		return;
	}

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	do {
		ret = read(dev, SC16IS7XX_REG_TXLVL, &txlvl);
		if (ret < 0) {
			LOG_ERR("failed to read TX level: %d", ret);
			k_mutex_unlock(&get_data(dev)->lock);
			return;
		}
	} while (!txlvl);

	ret = write_fifo(dev, &c, 1);
	if (ret < 0) {
		LOG_ERR("Poll out failed: %d", ret);
	}

	k_mutex_unlock(&get_data(dev)->lock);
}

static int sc16is7xx_uart_err_check(const struct device *dev) {
	int ret;

	uint8_t lsr;

	ret = read(dev, SC16IS7XX_REG_LSR, &lsr);
	if (ret < 0) {
		LOG_ERR("failed to read line status: %d", ret);
		return ret;
	}

	if (lsr & SC16IS7XX_LSR_OVERRUN_ERROR_msk) {
		return UART_ERROR_OVERRUN;
	} else if (lsr & SC16IS7XX_LSR_PARITY_ERROR_msk) {
		return UART_ERROR_PARITY;
	} else if (lsr & SC16IS7XX_LSR_FRAMING_ERROR_msk) {
		return UART_ERROR_FRAMING;
	} else if (lsr & SC16IS7XX_LSR_BREAK_INTERRUPT_msk) {
		return UART_BREAK;
	}

	return 0;
}

static int sc16is7xx_uart_configure(const struct device *dev, const struct uart_config *cfg) {
	int ret;

	k_mutex_lock(&get_data(dev)->lock, K_FOREVER);

	ret = sc16is7xx_uart_configure_line(dev, cfg);
	if (ret < 0) {
		LOG_ERR("Line configuration failed: %d", ret);
		k_mutex_unlock(&get_data(dev)->lock);
		return ret;
	}

	ret = sc16is7xx_uart_configure_baudrate(dev, cfg);
	if (ret < 0) {
		LOG_ERR("Baud rate configuration failed: %d", ret);
		k_mutex_unlock(&get_data(dev)->lock);
		return ret;
	}

	get_data(dev)->uart_config = *cfg;

	k_mutex_unlock(&get_data(dev)->lock);

	return 0;
}

static int sc16is7xx_uart_config_get(const struct device *dev, struct uart_config *cfg) {
	*cfg = get_data(dev)->uart_config;

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int sc16is7xx_uart_fifo_fill(const struct device *dev, const uint8_t *data, int data_len) {
	int ret;

	// LOG_HEXDUMP_DBG(data, data_len, "sc16is7xx_uart_fifo_fill");

	uint8_t txlvl;

	ret = read(dev, SC16IS7XX_REG_TXLVL, &txlvl);
	if (ret < 0) {
		LOG_ERR("read TXLVL failed: %d", ret);
		return ret;
	}

	const uint8_t write_len = MIN(txlvl, data_len);

	LOG_DBG("data_len: %d, txlvl: %d, write_len: %d", data_len, txlvl, write_len);

	ret = write_fifo(dev, data, write_len);
	if (ret < 0) {
		LOG_ERR("failed to write fifo: %d", ret);
		return ret;
	}

	return write_len;
}

static int sc16is7xx_uart_fifo_read(const struct device *dev, uint8_t *data, const int data_len) {
	int ret;

	uint8_t rxlvl;

	ret = read(dev, SC16IS7XX_REG_RXLVL, &rxlvl);
	if (ret < 0) {
		LOG_ERR("read RXLVL failed: %d", ret);
		return ret;
	}

	const uint8_t read_len = MIN(rxlvl, data_len);

	ret = read_fifo(dev, data, read_len);
	if (ret < 0) {
		LOG_ERR("failed to read fifo: %d", ret);
		return ret;
	}

	return read_len;
}

static void sc16is7xx_uart_irq_tx_enable(const struct device *dev) {
	int ret;

	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_THRI_pos, 1);

	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
	if (ret < 0) {
		LOG_ERR("write IER failed: %d", ret);
		return;
	}
}

static void sc16is7xx_uart_irq_tx_disable(const struct device *dev) {
	int ret;

	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_THRI_pos, 0);

	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
	if (ret < 0) {
		LOG_ERR("write IER failed: %d", ret);
		return;
	}
}

static int sc16is7xx_uart_irq_tx_ready(const struct device *dev) {
	const uint8_t iir = get_data(dev)->iir & SC16IS7XX_IIR_INTERRUPT_msk;

	return iir == SC16IS7XX_IIR_THRI_msk;
}

static void sc16is7xx_uart_irq_rx_enable(const struct device *dev) {
	int ret;

	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_RHRI_pos, 1);

	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
	if (ret < 0) {
		LOG_ERR("write IER failed: %d", ret);
		return;
	}
}

static void sc16is7xx_uart_irq_rx_disable(const struct device *dev) {
	int ret;

	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_RHRI_pos, 0);

	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
	if (ret < 0) {
		LOG_ERR("write IER failed: %d", ret);
		return;
	}
}

static int sc16is7xx_uart_irq_rx_ready(const struct device *dev) {
	const uint8_t iir = get_data(dev)->iir & SC16IS7XX_IIR_INTERRUPT_msk;

	return iir == SC16IS7XX_IIR_RHRI_msk || iir == SC16IS7XX_IIR_RTOI_msk;
}

// static void sc16is7xx_uart_irq_err_enable(const struct device *dev)
// {
// 	int ret;
//
// 	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_RLSI_pos, 1);
//
// 	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
// 	if (ret < 0) {
// 		LOG_ERR("write IER failed: %d", ret);
// 		return;
// 	}
// }

// static void sc16is7xx_uart_irq_err_disable(const struct device *dev)
// {
// 	int ret;
//
// 	WRITE_BIT(get_data(dev)->ier, SC16IS7XX_IER_RLSI_pos, 0);
//
// 	ret = write(dev, SC16IS7XX_REG_IER, &get_data(dev)->ier);
// 	if (ret < 0) {
// 		LOG_ERR("write IER failed: %d", ret);
// 	}
// }

static int sc16is7xx_uart_irq_is_pending(const struct device *dev) {
	return get_data(dev)->iir & SC16IS7XX_IIR_INTERRUPT_STATUS_msk ? 0 : 1;
}

static int sc16is7xx_uart_irq_update(const struct device *dev) {
	int ret;

	uint8_t iir;

	ret = read(get_data(dev)->self, SC16IS7XX_REG_IIR, &iir);
	if (ret < 0) {
		LOG_ERR("Reading IIR failed: %d", ret);

		/*
		 * TODO: Do we have to clean up in case of a read error?
		 */
		// get_data(dev)->iir = SC16IS7XX_IIR_INTERRUPT_STATUS_msk;
	} else {
		get_data(dev)->iir = iir;
	}

	return 1;
}

static void sc16is7xx_uart_irq_callback_set(const struct device *dev,
                                            uart_irq_callback_user_data_t cb, void *user_data) {
	get_data(dev)->user_callback = cb;
	get_data(dev)->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static void sc16is7xx_callback_handler(const struct device *dev) {
	k_work_submit(&get_data(dev)->callback_work);
}

static void sc16is7xx_callback_work_handler(struct k_work *work) {
	struct sc16is7xx_uart_data *data =
		CONTAINER_OF(work, struct sc16is7xx_uart_data, callback_work);

	if (data->user_callback) {
		data->user_callback(data->self, data->user_data);
	}
}

static int sc16is7xx_uart_init(const struct device *dev) {
	int ret;

	if (!device_is_ready(get_config(dev)->bus)) {
		LOG_ERR("parent device not ready");
		return -ENODEV;
	}

	ret = sc16is7xx_uart_configure_fifo(dev);
	if (ret < 0) {
		LOG_ERR("failed to configure fifo: %d", ret);
		return ret;
	}

	ret = sc16is7xx_uart_configure_line(dev, &get_data(dev)->uart_config);
	if (ret < 0) {
		LOG_ERR("failed to configure line: %d", ret);
		return ret;
	}

	ret = sc16is7xx_uart_configure_baudrate(dev, &get_data(dev)->uart_config);
	if (ret < 0) {
		LOG_ERR("failed to configure baudrate: %d", ret);
		return ret;
	}

	sc16is7xx_register_callback(get_config(dev)->bus, &get_data(dev)->callback);

	return 0;
}

static const struct uart_driver_api api = {
	.poll_in = sc16is7xx_uart_poll_in,
	.poll_out = sc16is7xx_uart_poll_out,
	.err_check = sc16is7xx_uart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = sc16is7xx_uart_configure,
	.config_get = sc16is7xx_uart_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = sc16is7xx_uart_fifo_fill,
	.fifo_read = sc16is7xx_uart_fifo_read,
	.irq_tx_enable = sc16is7xx_uart_irq_tx_enable,
	.irq_tx_disable = sc16is7xx_uart_irq_tx_disable,
	.irq_tx_ready = sc16is7xx_uart_irq_tx_ready,
	.irq_rx_enable = sc16is7xx_uart_irq_rx_enable,
	.irq_rx_disable = sc16is7xx_uart_irq_rx_disable,
	.irq_rx_ready = sc16is7xx_uart_irq_rx_ready,
	// .irq_err_enable = sc16is7xx_uart_irq_err_enable,
	// .irq_err_disable = sc16is7xx_uart_irq_err_disable,
	.irq_is_pending = sc16is7xx_uart_irq_is_pending,
	.irq_update = sc16is7xx_uart_irq_update,
	.irq_callback_set = sc16is7xx_uart_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define DT_INST_INIT(inst)                                                                         \
	static struct sc16is7xx_uart_data inst_##inst##_data = {                                   \
		.self = DEVICE_DT_INST_GET(inst),                                                  \
		.lock = Z_MUTEX_INITIALIZER(inst_##inst##_data.lock),                              \
		.callback = {.dev = DEVICE_DT_INST_GET(inst),                                      \
			     .handler = sc16is7xx_callback_handler},                               \
		.uart_config = {.baudrate = DT_INST_PROP_OR(inst, current_speed, 115200),          \
				.data_bits = DT_INST_ENUM_IDX_OR(inst, data_bits,                  \
								 UART_CFG_DATA_BITS_8),            \
				.parity = DT_INST_ENUM_IDX_OR(inst, parity, UART_CFG_PARITY_NONE), \
				.stop_bits = DT_INST_ENUM_IDX_OR(inst, stop_bits,                  \
								 UART_CFG_STOP_BITS_1),            \
				.flow_ctrl = DT_INST_ENUM_IDX_OR(inst, flow_control,               \
								 UART_CFG_FLOW_CTRL_NONE)},        \
		.callback_work = Z_WORK_INITIALIZER(sc16is7xx_callback_work_handler)};             \
	static const struct sc16is7xx_uart_config inst_##inst##_config = {                         \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                                           \
		.channel = DT_INST_REG_ADDR(inst),                                                 \
		.clock_frequency = DT_PROP(DT_INST_PARENT(inst), clock_frequency),                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, sc16is7xx_uart_init, NULL, &inst_##inst##_data,                \
			      &inst_##inst##_config, POST_KERNEL,                                  \
			      CONFIG_UART_SC16IS7XX_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(DT_INST_INIT)

// BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 2);
