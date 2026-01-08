/*
 * Copyright (c) 2023-2025 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * PIM447 trackball driver:
 * - Polls the device over I2C (no INT pin required)
 * - Emits Zephyr input events so ZMK v0.3 input listeners can consume movement
 * - Keeps a sensor API for optional mode/LED control via sensor attributes
 */

#define DT_DRV_COMPAT pimoroni_trackball_pim447

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* Use default log level unless you override globally */
LOG_MODULE_REGISTER(trackball_pim447);

/* Custom sensor attributes (private range) */
#define PIM447_ATTR_LED_RGB (SENSOR_ATTR_PRIV_START)
#define PIM447_ATTR_MODE (SENSOR_ATTR_PRIV_START + 1)

/* Register addresses (PIM447) */
#define TRACKBALL_PIM447_REG_LED_RED 0x00
#define TRACKBALL_PIM447_REG_LED_GREEN 0x01
#define TRACKBALL_PIM447_REG_LED_BLUE 0x02

#define TRACKBALL_PIM447_REG_LEFT 0x04
#define TRACKBALL_PIM447_REG_RIGHT 0x05
#define TRACKBALL_PIM447_REG_UP 0x06
#define TRACKBALL_PIM447_REG_DOWN 0x07
#define TRACKBALL_PIM447_REG_SWITCH 0x08

/* Burst read start/length (LEFT..SWITCH inclusive) */
#define TRACKBALL_PIM447_BURST_START TRACKBALL_PIM447_REG_LEFT
#define TRACKBALL_PIM447_BURST_LEN 5

/* Runtime data */
struct trackball_pim447_data {
    const struct device *dev;

    int16_t dx;
    int16_t dy;

    uint8_t button_state;
    uint8_t prev_button_state;

    bool invert_x;
    bool invert_y;

    uint8_t sensitivity;
    uint8_t move_factor;
    uint8_t scroll_factor;

    uint8_t mode; /* 0=move, 1=scroll */

    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;

    struct k_work_delayable poll_work;
};

/* Build-time configuration */
struct trackball_pim447_config {
    struct i2c_dt_spec i2c;

    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;

    bool invert_x;
    bool invert_y;

    uint8_t sensitivity;
    uint8_t move_factor;
    uint8_t scroll_factor;

    /* Optional DT property (poll-interval-ms). If not present, defaults below. */
    uint16_t poll_interval_ms;
};

static int trackball_pim447_read_reg(const struct device *dev, uint8_t reg, uint8_t *value) {
    const struct trackball_pim447_config *config = dev->config;

    int err = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), value, sizeof(*value));
    if (err) {
        LOG_ERR("Failed to read register 0x%02x: %d", reg, err);
        return err;
    }

    return 0;
}

static int trackball_pim447_write_reg(const struct device *dev, uint8_t reg, uint8_t value) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[2] = { reg, value };

    int err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to write register 0x%02x: %d", reg, err);
        return err;
    }

    return 0;
}

static int trackball_pim447_clear_motion_regs(const struct device *dev) {
    int err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_LEFT, 0);
    if (err) return err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_RIGHT, 0);
    if (err) return err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_UP, 0);
    if (err) return err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_DOWN, 0);
    if (err) return err;

    return 0;
}

static int trackball_pim447_read_motion_burst(const struct device *dev, int16_t *dx, int16_t *dy,
                                              uint8_t *btn) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[TRACKBALL_PIM447_BURST_LEN] = {0};

    int err = i2c_burst_read_dt(&config->i2c, TRACKBALL_PIM447_BURST_START, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to read motion burst: %d", err);
        return err;
    }

    const uint8_t left  = buf[0];
    const uint8_t right = buf[1];
    const uint8_t up    = buf[2];
    const uint8_t down  = buf[3];
    const uint8_t sw    = buf[4];

    *dx = (int16_t)right - (int16_t)left;
    *dy = (int16_t)down - (int16_t)up;
    *btn = sw;

    /* Prevent accumulation if the device does not auto-clear on read */
    (void)trackball_pim447_clear_motion_regs(dev);

    return 0;
}

static int trackball_pim447_set_led(const struct device *dev, uint8_t red, uint8_t green, uint8_t blue) {
    struct trackball_pim447_data *data = dev->data;
    int err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_LED_RED, red);
    if (err) return err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_LED_GREEN, green);
    if (err) return err;

    err = trackball_pim447_write_reg(dev, TRACKBALL_PIM447_REG_LED_BLUE, blue);
    if (err) return err;

    data->led_red = red;
    data->led_green = green;
    data->led_blue = blue;

    return 0;
}

static void trackball_pim447_apply_scaling(struct trackball_pim447_data *data) {
    if (data->invert_x) {
        data->dx = -data->dx;
    }
    if (data->invert_y) {
        data->dy = -data->dy;
    }

    /* sensitivity baseline 64 => 1.0x */
    data->dx = (data->dx * data->sensitivity) / 64;
    data->dy = (data->dy * data->sensitivity) / 64;

    if (data->mode == 0) {
        /* Move mode */
        data->dx = data->dx * data->move_factor;
        data->dy = data->dy * data->move_factor;
    } else {
        /* Scroll mode */
        data->dx = data->dx * data->scroll_factor;
        data->dy = data->dy * data->scroll_factor;
    }
}

/* Sensor API: fetch latest sample (we always read all channels in one burst) */
static int trackball_pim447_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct trackball_pim447_data *data = dev->data;
    int err;

    ARG_UNUSED(chan);

    err = trackball_pim447_read_motion_burst(dev, &data->dx, &data->dy, &data->button_state);
    if (err) {
        return err;
    }

    trackball_pim447_apply_scaling(data);

    return 0;
}

/* Sensor API: get cached channel values */
static int trackball_pim447_channel_get(const struct device *dev, enum sensor_channel chan,
                                        struct sensor_value *val) {
    struct trackball_pim447_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->dx;
        val->val2 = 0;
        return 0;

    case SENSOR_CHAN_POS_DY:
        val->val1 = data->dy;
        val->val2 = 0;
        return 0;

    case SENSOR_CHAN_PROX:
        val->val1 = data->button_state;
        val->val2 = 0;
        return 0;

    default:
        return -ENOTSUP;
    }
}

/* Sensor API: set custom attributes (mode + LED RGB) */
static int trackball_pim447_attr_set(const struct device *dev, enum sensor_channel chan,
                                     enum sensor_attribute attr, const struct sensor_value *val) {
    struct trackball_pim447_data *data = dev->data;

    ARG_UNUSED(chan);

    switch (attr) {
    case PIM447_ATTR_LED_RGB: {
        if (!val) {
            return -EINVAL;
        }
        /* Expect val[0], val[1], val[2] => RGB */
        uint8_t r = CLAMP(val[0].val1, 0, 255);
        uint8_t g = CLAMP(val[1].val1, 0, 255);
        uint8_t b = CLAMP(val[2].val1, 0, 255);
        return trackball_pim447_set_led(dev, r, g, b);
    }

    case PIM447_ATTR_MODE: {
        if (!val) {
            return -EINVAL;
        }
        data->mode = (val->val1 > 0) ? 1 : 0;
        LOG_INF("Trackball mode set to %s", data->mode == 0 ? "MOVE" : "SCROLL");
        return 0;
    }

    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api trackball_pim447_api = {
    .sample_fetch = trackball_pim447_sample_fetch,
    .channel_get = trackball_pim447_channel_get,
    .attr_set = trackball_pim447_attr_set,
};

/* Emit Zephyr input events (what ZMK v0.3 expects) */
static void trackball_pim447_emit_input(const struct device *dev) {
    struct trackball_pim447_data *data = dev->data;

    const uint16_t code_x = (data->mode == 0) ? INPUT_REL_X : INPUT_REL_HWHEEL;
    const uint16_t code_y = (data->mode == 0) ? INPUT_REL_Y : INPUT_REL_WHEEL;

    const bool has_x = (data->dx != 0);
    const bool has_y = (data->dy != 0);

    if (has_x) {
        (void)input_report_rel(dev, code_x, data->dx, !has_y, K_NO_WAIT);
    }
    if (has_y) {
        (void)input_report_rel(dev, code_y, data->dy, true, K_NO_WAIT);
    }

    const bool pressed = (data->button_state != 0);
    const bool prev_pressed = (data->prev_button_state != 0);

    if (pressed != prev_pressed) {
        (void)input_report_key(dev, INPUT_BTN_0, pressed ? 1 : 0, true, K_NO_WAIT);
        data->prev_button_state = data->button_state;
    }
}

static void trackball_pim447_poll_work_handler(struct k_work *work) {
    struct trackball_pim447_data *data =
        CONTAINER_OF(work, struct trackball_pim447_data, poll_work.work);
    const struct device *dev = data->dev;
    const struct trackball_pim447_config *config = dev->config;

    if (trackball_pim447_sample_fetch(dev, SENSOR_CHAN_ALL) == 0) {
        trackball_pim447_emit_input(dev);
    }

    /* Always reschedule */
    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));
}

static int trackball_pim447_init(const struct device *dev) {
    const struct trackball_pim447_config *config = dev->config;
    struct trackball_pim447_data *data = dev->data;
    int err;

    data->dev = dev;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->invert_x = config->invert_x;
    data->invert_y = config->invert_y;
    data->sensitivity = config->sensitivity;
    data->move_factor = config->move_factor;
    data->scroll_factor = config->scroll_factor;
    data->mode = 0; /* default MOVE */
    data->prev_button_state = 0;

    err = trackball_pim447_set_led(dev, config->led_red, config->led_green, config->led_blue);
    if (err) {
        LOG_ERR("Failed to set initial LED color: %d", err);
        return err;
    }

    k_work_init_delayable(&data->poll_work, trackball_pim447_poll_work_handler);

    /* Start polling immediately */
    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));

    LOG_INF("Pimoroni Trackball initialized (addr: 0x%02x, poll: %ums)",
            config->i2c.addr, config->poll_interval_ms);

    return 0;
}

/* Driver instantiation */
#define TRACKBALL_PIM447_INIT(inst)                                                       \
    static struct trackball_pim447_data trackball_pim447_data_##inst;                     \
                                                                                          \
    static const struct trackball_pim447_config trackball_pim447_config_##inst = {        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                \
        .led_red = DT_INST_PROP_OR(inst, led_red, 0),                                     \
        .led_green = DT_INST_PROP_OR(inst, led_green, 0),                                 \
        .led_blue = DT_INST_PROP_OR(inst, led_blue, 0),                                   \
        .invert_x = DT_INST_PROP_OR(inst, invert_x, false),                               \
        .invert_y = DT_INST_PROP_OR(inst, invert_y, false),                               \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 64),                            \
        .move_factor = DT_INST_PROP_OR(inst, move_factor, 1),                             \
        .scroll_factor = DT_INST_PROP_OR(inst, scroll_factor, 1),                         \
        .poll_interval_ms = DT_INST_PROP_OR(inst, poll_interval_ms, 10),                  \
    };                                                                                    \
                                                                                          \
    DEVICE_DT_INST_DEFINE(inst, trackball_pim447_init, NULL,                              \
                          &trackball_pim447_data_##inst, &trackball_pim447_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, &trackball_pim447_api);

DT_INST_FOREACH_STATUS_OKAY(TRACKBALL_PIM447_INIT)