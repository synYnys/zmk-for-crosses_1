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
#define PMW3610_REG_MOTION          0x02
#define PMW3610_REG_DELTA_X_L       0x03
#define PMW3610_REG_DELTA_Y_L       0x04
#define PMW3610_REG_MOTION_BURST    0x16
#define PMW3610_REG_POWER_UP_RESET  0x3A

#define PMW3610_PRODUCT_ID          0x3E

struct pmw3610_data {
    const struct device *dev;
    int16_t last_x;
    int16_t last_y;
};

struct pmw3610_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq_gpio; // これを復活させました
    uint16_t cpi;                 // これも復活させました
};

static int pmw3610_read_reg(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t addr = reg & 0x7F;
    
    struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = val, .len = 1 };
    struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    // 3線式 (Half Duplex) での読み込み
    return spi_transceive_dt(&cfg->bus, &tx, &rx);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t buf[] = { reg | 0x80, val };
    struct spi_buf tx_buf = { .buf = buf, .len = 2 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    return spi_write_dt(&cfg->bus, &tx);
}

static int pmw3610_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pmw3610_data *data = dev->data;
    uint8_t motion, xl, yl;
    
    // バースト読み出しではなく、確実に動く単発読み出しを使用
    pmw3610_read_reg(dev, PMW3610_REG_MOTION, &motion);
    
    if (motion & 0x80) {
        pmw3610_read_reg(dev, PMW3610_REG_DELTA_X_L, &xl);
        pmw3610_read_reg(dev, PMW3610_REG_DELTA_Y_L, &yl);
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
    if (chan == SENSOR_CHAN_POS_DX) {
        val->val1 = data->last_x;
    } else if (chan == SENSOR_CHAN_POS_DY) {
        val->val1 = data->last_y;
    } else {
        return -ENOTSUP;
    }
    val->val2 = 0;
    return 0;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .sample_fetch = pmw3610_sample_fetch,
    .channel_get = pmw3610_channel_get,
};

static int pmw3610_init(const struct device *dev) {
    const struct pmw3610_config *cfg = dev->config;
    uint8_t chip_id;

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    /* Reset */
    pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, 0x5A);
    k_msleep(50); // リセット後の待ち時間を十分に取る

    /* Chip ID check */
    pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &chip_id);
    if (chip_id != PMW3610_PRODUCT_ID) {
        LOG_WRN("PMW3610 ID mismatch: 0x%02x", chip_id);
        // IDが違っても動作を試みる
    }
    
    return 0;
}

#define PMW3610_DEFINE(inst)                                            \
    static struct pmw3610_data pmw3610_data_##inst;                     \
    static const struct pmw3610_config pmw3610_config_##inst = {        \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_HALF_DUPLEX, 0), \
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
