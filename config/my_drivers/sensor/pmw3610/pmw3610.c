#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmw3610, CONFIG_SENSOR_LOG_LEVEL);

#define PMW3610_REG_PRODUCT_ID      0x00
#define PMW3610_REG_REVISION_ID     0x01
#define PMW3610_REG_MOTION          0x02
#define PMW3610_REG_DELTA_X_L       0x03
#define PMW3610_REG_DELTA_Y_L       0x04
#define PMW3610_REG_DELTA_XY_H      0x05
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

/* 3-Wire SPI Read Wrapper */
static int pmw3610_read_reg(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pmw3610_config *cfg = dev->config;
    
    // 1. Send Address (MSB 0 for read)
    uint8_t addr = reg & 0x7F;
    const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    // 2. Read Data
    const struct spi_buf rx_buf = { .buf = val, .len = 1 };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    // In HALF_DUPLEX mode, providing both TX and RX means "Write then Read"
    return spi_transceive_dt(&cfg->bus, &tx, &rx);
}

/* 3-Wire SPI Write Wrapper */
static int pmw3610_write_reg(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t buf[] = { reg | 0x80, val }; // MSB 1 for write
    
    const struct spi_buf tx_buf = { .buf = buf, .len = 2 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    return spi_write_dt(&cfg->bus, &tx);
}

/* Burst Read */
static int pmw3610_burst_read(const struct device *dev, uint8_t *buf, size_t len) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t reg = PMW3610_REG_MOTION_BURST;
    
    const struct spi_buf tx_buf = { .buf = &reg, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    
    const struct spi_buf rx_buf = { .buf = buf, .len = len };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    return spi_transceive_dt(&cfg->bus, &tx, &rx);
}

static int pmw3610_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pmw3610_data *data = dev->data;
    /* Burst buffer: Motion, Delta_X_L, Delta_Y_L, Delta_XY_H, ... */
    uint8_t burst_data[6]; 
    int err;

    err = pmw3610_burst_read(dev, burst_data, sizeof(burst_data));
    if (err) return err;

    uint8_t motion = burst_data[0];
    
    if (motion & 0x80) {
        uint8_t xl = burst_data[1];
        uint8_t yl = burst_data[2];
        
        data->last_x = (int16_t)(int8_t)xl;
        data->last_y = (int16_t)(int8_t)yl;
    } else {
        data->last_x = 0;
        data->last_y = 0;
    }
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

static const struct sensor_driver_api pmw3610_driver_api = {
    .sample_fetch = pmw3610_sample_fetch,
    .channel_get = pmw3610_channel_get,
};

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

    pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, 0x5A);
    k_sleep(K_MSEC(20));

    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &chip_id);
    if (err) return err;

    if (chip_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Invalid chip ID: 0x%02x", chip_id);
        // continue anyway
    } else {
        LOG_INF("PMW3610 detected (ID: 0x%02x)", chip_id);
    }
    
    // Force Awake
    pmw3610_write_reg(dev, PMW3610_REG_SHUTDOWN, 0xB0); 

    return 0;
}

#define PMW3610_DEFINE(inst)                                            \
    static struct pmw3610_data pmw3610_data_##inst;                     \
    static const struct pmw3610_config pmw3610_config_##inst = {        \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_HALF_DUPLEX, 0), \
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
