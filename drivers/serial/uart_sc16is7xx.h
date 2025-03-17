#ifndef DRIVERS_SERIAL_SC16IS7XX_REG_H_
#define DRIVERS_SERIAL_SC16IS7XX_REG_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define SC16IS7XX_RESET_PULSE_us 3
#define SC16IS7XX_RESET_DELAY_us 3

#define SC16IS7XX_LCR_MAGIC 0xbf

#define SC16IS7XX_CH_pos  1
#define SC16IS7XX_CH_msk  (BIT(2) | BIT(1))
#define SC16IS7XX_REG_pos 3
#define SC16IS7XX_REG_msk (BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define SC16IS7XX_RW_pos  7
#define SC16IS7XX_RW_msk  BIT(7)

/*
 * General register set
 */
#define SC16IS7XX_REG_RHR       0x00 /* read only, overlaps with THR */
#define SC16IS7XX_REG_THR       0x00 /* write only, overlaps with RHR */
#define SC16IS7XX_REG_IER       0x01
#define SC16IS7XX_REG_IIR       0x02 /* read only, overlaps with FCR */
#define SC16IS7XX_REG_FCR       0x02 /* write only, overlaps with IIR */
#define SC16IS7XX_REG_LCR       0x03
#define SC16IS7XX_REG_MCR       0x04
#define SC16IS7XX_REG_LSR       0x05
#define SC16IS7XX_REG_MSR       0x06 /* overlaps with TCR */
#define SC16IS7XX_REG_TCR       0x06 /* overlaps with MSR, accessible if MCR[2] and EFR[4] set */
#define SC16IS7XX_REG_SPR       0x07 /* overlaps with TLR */
#define SC16IS7XX_REG_TLR       0x07 /* overlaps with SPR, accessible if MCR[2] and EFR[4] set */
#define SC16IS7XX_REG_TXLVL     0x08
#define SC16IS7XX_REG_RXLVL     0x09
#define SC16IS7XX_REG_IODIR     0x0a
#define SC16IS7XX_REG_IOSTATE   0x0b
#define SC16IS7XX_REG_IOINTENA  0x0c
#define SC16IS7XX_REG_IOCONTROL 0x0e
#define SC16IS7XX_REG_EFCR      0x0f

/*
 * Special register set (only accessible if LCR[7] == 1 && LCR != 0xbf)
 */
#define SC16IS7XX_REG_DLL 0x00
#define SC16IS7XX_REG_DLH 0x01

/*
 * Enhanced register set (only accessible if LCR == 0xbf)
 */
#define SC16IS7XX_REG_EFR 0x02

/*
 * IER register bits
 */
#define SC16IS7XX_IER_RHRI_pos       0 /* Receive Holding Register interrupt */
#define SC16IS7XX_IER_THRI_pos       1 /* Transmit Holding Register interrupt */
#define SC16IS7XX_IER_RLSI_pos       2 /* Receive Line Status interrupt */
#define SC16IS7XX_IER_SLEEP_MODE_pos 4

/*
 * IIR register bits
 */
#define SC16IS7XX_IIR_INTERRUPT_STATUS_msk BIT(0)
#define SC16IS7XX_IIR_INTERRUPT_msk        (BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define SC16IS7XX_IIR_RLSE_msk             (BIT(2) | BIT(1)) /* Receiver line Status error */
#define SC16IS7XX_IIR_RTOI_msk             (BIT(3) | BIT(2)) /* Receiver time-out interrupt */
#define SC16IS7XX_IIR_RHRI_msk             BIT(2)            /* RHR interrupt */
#define SC16IS7XX_IIR_THRI_msk             BIT(1)            /* THR interrupt */
#define SC16IS7XX_IIR_IPCS_msk             (BIT(5) | BIT(4)) /* Input pin change of state */

/*
 * FCR register bits
 */
#define SC16IS7XX_FCR_FIFO_ENABLE_pos    0
#define SC16IS7XX_FCR_RESET_RX_FIFO_pos  1
#define SC16IS7XX_FCR_RESET_TX_FIFO_pos  2
#define SC16IS7XX_FCR_TX_TRIGGER_LSB_pos 4
#define SC16IS7XX_FCR_TX_TRIGGER_MSB_pos 5
#define SC16IS7XX_FCR_RX_TRIGGER_LSB_pos 6
#define SC16IS7XX_FCR_RX_TRIGGER_MSB_pos 7

/*
 * LCR register bits
 */
#define SC16IS7XX_LCR_WORDLEN_LSB_pos          0
#define SC16IS7XX_LCR_WORDLEN_MSB_pos          1
#define SC16IS7XX_LCR_STOPLEN_pos              2
#define SC16IS7XX_LCR_PARITY_ENABLE_pos        3
#define SC16IS7XX_LCR_PARITY_TYPE_pos          4 /* 0: Odd, 1: Even */
#define SC16IS7XX_LCR_FORCE_PARITY_pos         5
#define SC16IS7XX_LCR_DIVISOR_LATCH_ENABLE_pos 7

/*
 * MCR register bits
 */
#define SC16IS7XX_MCR_CLOCK_DIVISOR_pos 7

/*
 * LSR register bits
 */
#define SC16IS7XX_LSR_DATA_IN_RECEIVER_pos 0
#define SC16IS7XX_LSR_DATA_IN_RECEIVER_msk BIT(0)
#define SC16IS7XX_LSR_OVERRUN_ERROR_pos    1
#define SC16IS7XX_LSR_OVERRUN_ERROR_msk    BIT(1)
#define SC16IS7XX_LSR_PARITY_ERROR_pos     2
#define SC16IS7XX_LSR_PARITY_ERROR_msk     BIT(2)
#define SC16IS7XX_LSR_FRAMING_ERROR_pos    3
#define SC16IS7XX_LSR_FRAMING_ERROR_msk    BIT(3)
#define SC16IS7XX_LSR_BREAK_INTERRUPT_pos  4
#define SC16IS7XX_LSR_BREAK_INTERRUPT_msk  BIT(4)
#define SC16IS7XX_LSR_THR_EMPTY_pos        5
#define SC16IS7XX_LSR_THR_EMPTY_msk        BIT(5)
#define SC16IS7XX_LSR_THR_TSR_EMPTY_pos    6
#define SC16IS7XX_LSR_THR_TSR_EMPTY_msk    BIT(6)
#define SC16IS7XX_LSR_FIFO_DATA_ERROR_pos  7
#define SC16IS7XX_LSR_FIFO_DATA_ERROR_msk  BIT(7)

/*
 * EFCR register bits
 */
#define SC16IS7XX_EFCR_RTS_CONTROL_pos 4
#define SC16IS7XX_EFCR_RTS_INVERT_pos  5

/*
 * IOControl register bits
 */
#define SC16IS7XX_IOCONTROL_IOLATCH_pos  0
#define SC16IS7XX_IOCONTROL_IOLATCH_msk  BIT(0)
#define SC16IS7XX_IOCONTROL_GPIO_7_4_pos 1
#define SC16IS7XX_IOCONTROL_GPIO_7_4_msk BIT(1)
#define SC16IS7XX_IOCONTROL_GPIO_3_0_pos 2
#define SC16IS7XX_IOCONTROL_GPIO_3_0_msk BIT(2)
#define SC16IS7XX_IOCONTROL_SRESET_pos   3
#define SC16IS7XX_IOCONTROL_SRESET_msk   BIT(3)

/*
 * EFR register bits
 */
#define SC16IS7XX_EFR_EFE_pos 4
#define SC16IS7XX_EFR_RTS_pos 6
#define SC16IS7XX_EFR_CTS_pos 7

#ifdef __cplusplus
extern "C" {



#endif

enum SC16IS7XX_RX_TRIGGER_LEVEL {
	SC16IS7XX_RX_TRIGGER_LEVEL_8,
	SC16IS7XX_RX_TRIGGER_LEVEL_16,
	SC16IS7XX_RX_TRIGGER_LEVEL_56,
	SC16IS7XX_RX_TRIGGER_LEVEL_60,
};

enum SC16IS7XX_TX_TRIGGER_LEVEL {
	SC16IS7XX_TX_TRIGGER_LEVEL_8,
	SC16IS7XX_TX_TRIGGER_LEVEL_16,
	SC16IS7XX_TX_TRIGGER_LEVEL_32,
	SC16IS7XX_TX_TRIGGER_LEVEL_56,
};

typedef void (*sc16is7xx_callback_handler_t)(const struct device *dev);

struct sc16is7xx_callback {
	sys_snode_t node;
	sc16is7xx_callback_handler_t handler;
	const struct device *dev;
};

__subsystem struct sc16is7xx_api {
	int (*read)(const struct device *dev, uint8_t reg, uint8_t *data, size_t data_len);

	int (*write)(const struct device *dev, uint8_t reg, const uint8_t *data, size_t data_len);

	void (*register_callback)(const struct device *dev, struct sc16is7xx_callback *callback);
};

static inline int sc16is7xx_read(const struct device *dev, uint8_t reg, uint8_t *data,
                                 size_t data_len) {
	const struct sc16is7xx_api *api = (const struct sc16is7xx_api *) dev->api;

	return api->read(dev, reg, data, data_len);
}

static inline int sc16is7xx_write(const struct device *dev, uint8_t reg, const uint8_t *data,
                                  size_t data_len) {
	const struct sc16is7xx_api *api = (const struct sc16is7xx_api *) dev->api;

	return api->write(dev, reg, data, data_len);
}

static inline void sc16is7xx_register_callback(const struct device *dev,
                                               struct sc16is7xx_callback *callback) {
	const struct sc16is7xx_api *api = (const struct sc16is7xx_api *) dev->api;

	return api->register_callback(dev, callback);
}

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SERIAL_SC16IS7XX_REG_H_ */
