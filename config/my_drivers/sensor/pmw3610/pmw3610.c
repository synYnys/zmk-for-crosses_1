#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmw3610, CONFIG_SENSOR_LOG_LEVEL);

struct pmw3610_data {};
struct pmw3610_config {};

static int pmw3610_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    return 0;
}

static int pmw3610_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val) {
    return 0;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .sample_fetch = pmw3610_sample_fetch,
    .channel_get = pmw3610_channel_get,
};

static int pmw3610_init(const struct device *dev) {
    LOG_INF("PMW3610 stub driver initialized");
    return 0;
}

#define PMW3610_DEFINE(inst)                                        \
    static struct pmw3610_data pmw3610_data_##inst;                 \
    static const struct pmw3610_config pmw3610_config_##inst = {};  \
    DEVICE_DT_INST_DEFINE(inst, pmw3610_init, NULL,                 \
                          &pmw3610_data_##inst, &pmw3610_config_##inst, \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
