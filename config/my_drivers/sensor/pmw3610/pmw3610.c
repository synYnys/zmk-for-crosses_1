#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmw3610, CONFIG_SENSOR_LOG_LEVEL);

/* Registers */
#define PMW3610_REG_PRODUCT_ID      0x00
#define PMW3610_REG_REVISION_ID     0x01
#define PMW3610_REG_MOTION          0x02
#define PMW3610_REG_DELTA_X_L       0x03
#define PMW3610_REG_DELTA_Y_L       0x04
#define PMW3610_REG_DELTA_XY_H      0x05
#define PMW3610_REG_CONFIG1         0x0F
#define PMW3610_REG_PERFORMANCE     0x11
#define PMW3610_REG_OBSERVATION     0x15
#define PMW3610_REG_MOTION_BURST    0x16
#define PMW3610_REG_POWER_UP_RESET  0x3A
#define PMW3610_REG_SHUTDOWN        0x3B
#define PMW3610_REG_RES_STEP        0x8D

#define PMW3610_PRODUCT_ID          0x3E

struct pmw3610_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work work;
    int16_t last_x;
    int16_t last_y;
};

struct pmw3610_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
};

static int pmw3610_read_reg(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t tx_buf[] = { reg & 0x7F };
    uint8_t rx_buf[2];
    
    const struct spi_buf tx = { .buf = tx_buf, .len = 1 };
    const struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int err = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
    if (err < 0) return err;

    *val = rx_buf[1];
    return 0;
}

static int pmw3610_write_reg(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t tx_buf[] = { reg | 0x80, val };
    
    const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    return spi_write_dt(&cfg->bus, &tx_set);
}

static int pmw3610_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pmw3610_data *data = dev->data;
    int err;
    uint8_t motion_reg, xl, yl;

    /* Read Motion Status */
    err = pmw3610_read_reg(dev, PMW3610_REG_MOTION, &motion_reg);
    if (err) return err;

    /* Check if motion occurred (Bit 7 set) */
    if (!(motion_reg & 0x80)) {
        data->last_x = 0;
        data->last_y = 0;
        return 0;
    }

    /* Read Delta registers */
    pmw3610_read_reg(dev, PMW3610_REG_DELTA_X_L, &xl);
    pmw3610_read_reg(dev, PMW3610_REG_DELTA_Y_L, &yl);
    
    /* Convert to signed 8-bit */
    data->last_x = (int8_t)xl;
    data->last_y = (int8_t)yl;
    
    return 0;
}

static int pmw3610_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val) {
    struct pmw3610_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->last_x;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->last_y;
        val->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static int pmw3610_init(const struct device *dev) {
    const struct pmw3610_config *cfg = dev->config;
    struct pmw3610_data *data = dev->data;
    uint8_t chip_id;
    int err;

    data->dev = dev;

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    /* Hardware Reset Sequence */
    /* 1. Power Up Reset */
    pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, 0x5A);
    k_sleep(K_MSEC(20));

    /* 2. Read Chip ID to verify connection */
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &chip_id);
    if (err) {
        LOG_ERR("Failed to read chip ID");
        return err;
    }

    if (chip_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Invalid chip ID: 0x%02x (expected 0x%02x)", chip_id, PMW3610_PRODUCT_ID);
    } else {
        LOG_INF("PMW3610 detected (ID: 0x%02x)", chip_id);
    }

    return 0;
}

/* Device Instantiation */
#define PMW3610_DEFINE(inst)                                            \
    static struct pmw3610_data pmw3610_data_##inst;                     \
    static const struct pmw3610_config pmw3610_config_##inst = {        \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB, 0), \
        .irq_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}), \
        .cpi = 1200,                                                    \
    };                                                                  \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, pmw3610_init, NULL,              \
                                 &pmw3610_data_##inst,                  \
                                 &pmw3610_config_##inst,                \
                                 POST_KERNEL,                           \
                                 CONFIG_SENSOR_INIT_PRIORITY,           \
                                 &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
