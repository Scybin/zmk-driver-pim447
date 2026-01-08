/*
 * Pimoroni PIM447 Trackball driver (polling, no INT pin required)
 *
 * - Polls the device over I2C
 * - Emits Zephyr input events so ZMK input listeners can consume movement
 * - Controls RGBW LED via I2C registers 0x00..0x03
 * - Clears sleep bit in REG_CTRL (0xFE) if set
 * - Reads back LED/CTRL registers for debug
 *
 * Register map and CTRL sleep mask match Pimoroni reference implementation. :contentReference[oaicite:1]{index=1}
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

LOG_MODULE_REGISTER(trackball_pim447);

/* Custom sensor attributes (private range) */
#define PIM447_ATTR_LED_RGB (SENSOR_ATTR_PRIV_START)
#define PIM447_ATTR_MODE (SENSOR_ATTR_PRIV_START + 1)

/* PIM447 register map (subset) */
#define REG_LED_RED   0x00
#define REG_LED_GRN   0x01
#define REG_LED_BLU   0x02
#define REG_LED_WHT   0x03

#define REG_LEFT      0x04
#define REG_RIGHT     0x05
#define REG_UP        0x06
#define REG_DOWN      0x07
#define REG_SWITCH    0x08

#define REG_CHIP_ID_L 0xFA
#define REG_CHIP_ID_H 0xFB
#define REG_CTRL      0xFE

#define MSK_SWITCH_STATE 0x80 /* pressed bit */
#define MSK_CTRL_SLEEP   0x01 /* sleep bit */

/* Burst read start/length (LEFT..SWITCH inclusive) */
#define MOTION_BURST_START REG_LEFT
#define MOTION_BURST_LEN   5

/* Runtime data */
struct trackball_pim447_data {
    const struct device *dev;

    int16_t dx;
    int16_t dy;

    uint8_t button_state;      /* 0/1 */
    uint8_t prev_button_state; /* 0/1 */

    bool invert_x;
    bool invert_y;

    uint8_t sensitivity;
    uint8_t move_factor;
    uint8_t scroll_factor;

    uint8_t mode; /* 0=move, 1=scroll */

    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
    uint8_t led_white;

    struct k_work_delayable poll_work;
};

/* Build-time configuration */
struct trackball_pim447_config {
    struct i2c_dt_spec i2c;

    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
    uint8_t led_white;

    bool invert_x;
    bool invert_y;

    uint8_t sensitivity;
    uint8_t move_factor;
    uint8_t scroll_factor;

    /* DT: poll-interval-ms (defaults to 10 if not present) */
    uint16_t poll_interval_ms;
};

static int read_reg_u8(const struct device *dev, uint8_t reg, uint8_t *value) {
    const struct trackball_pim447_config *config = dev->config;

    int err = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), value, sizeof(*value));
    if (err) {
        LOG_ERR("read reg 0x%02x failed: %d", reg, err);
        return err;
    }
    return 0;
}

static int write_reg_u8(const struct device *dev, uint8_t reg, uint8_t value) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[2] = {reg, value};

    int err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (err) {
        LOG_ERR("write reg 0x%02x failed: %d", reg, err);
        return err;
    }
    return 0;
}

static int clear_motion_regs(const struct device *dev) {
    int err;

    err = write_reg_u8(dev, REG_LEFT, 0);
    if (err) return err;

    err = write_reg_u8(dev, REG_RIGHT, 0);
    if (err) return err;

    err = write_reg_u8(dev, REG_UP, 0);
    if (err) return err;

    err = write_reg_u8(dev, REG_DOWN, 0);
    if (err) return err;

    return 0;
}

static int read_motion_burst(const struct device *dev, int16_t *dx, int16_t *dy, uint8_t *pressed) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[MOTION_BURST_LEN] = {0};

    int err = i2c_burst_read_dt(&config->i2c, MOTION_BURST_START, buf, sizeof(buf));
    if (err) {
        LOG_ERR("motion burst read failed: %d", err);
        return err;
    }

    const uint8_t left  = buf[0];
    const uint8_t right = buf[1];
    const uint8_t up    = buf[2];
    const uint8_t down  = buf[3];
    const uint8_t sw    = buf[4];

    *dx = (int16_t)right - (int16_t)left;
    *dy = (int16_t)down  - (int16_t)up;

    /* Pimoroni reference uses bit7 for pressed state */
    *pressed = (sw & MSK_SWITCH_STATE) ? 1 : 0;

    /* Prevent accumulation if the device does not auto-clear on read */
    (void)clear_motion_regs(dev);

    return 0;
}

static int set_led_rgbw(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    const struct trackball_pim447_config *config = dev->config;
    struct trackball_pim447_data *data = dev->data;

    /* Pimoroni reference: write [REG_LED_RED, r, g, b, w] */
    uint8_t buf[5] = {REG_LED_RED, r, g, b, w};

    int err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (err) {
        LOG_ERR("set LED RGBW(%u,%u,%u,%u) failed: %d", r, g, b, w, err);
        return err;
    }

    data->led_red = r;
    data->led_green = g;
    data->led_blue = b;
    data->led_white = w;

    return 0;
}

static void apply_scaling(struct trackball_pim447_data *data) {
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

    err = read_motion_burst(dev, &data->dx, &data->dy, &data->button_state);
    if (err) {
        return err;
    }

    apply_scaling(data);
    return 0;
}

/* Sensor API: get cached channel values (optional) */
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
        /* Expect val[0], val[1], val[2] => RGB. Keep current white unchanged. */
        uint8_t r = CLAMP(val[0].val1, 0, 255);
        uint8_t g = CLAMP(val[1].val1, 0, 255);
        uint8_t b = CLAMP(val[2].val1, 0, 255);
        return set_led_rgbw(dev, r, g, b, data->led_white);
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

static void emit_input_events(const struct device *dev) {
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

    if (data->button_state != data->prev_button_state) {
        (void)input_report_key(dev, INPUT_BTN_0, data->button_state ? 1 : 0, true, K_NO_WAIT);
        data->prev_button_state = data->button_state;
    }
}

static void poll_work_handler(struct k_work *work) {
    struct trackball_pim447_data *data =
        CONTAINER_OF(work, struct trackball_pim447_data, poll_work.work);
    const struct device *dev = data->dev;
    const struct trackball_pim447_config *config = dev->config;

    if (trackball_pim447_sample_fetch(dev, SENSOR_CHAN_ALL) == 0) {
        emit_input_events(dev);
    }

    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));
}

static int wake_if_sleeping(const struct device *dev) {
    uint8_t ctrl = 0;
    int err = read_reg_u8(dev, REG_CTRL, &ctrl);
    if (err) {
        return err;
    }

    if (ctrl & MSK_CTRL_SLEEP) {
        LOG_INF("PIM447 CTRL=0x%02x (sleep set), waking", ctrl);
        ctrl &= ~MSK_CTRL_SLEEP;
        err = write_reg_u8(dev, REG_CTRL, ctrl);
        if (err) {
            return err;
        }
        k_sleep(K_MSEC(2));
    }

    return 0;
}

static void log_led_regs(const struct device *dev) {
    uint8_t r = 0, g = 0, b = 0, w = 0, ctrl = 0;
    (void)read_reg_u8(dev, REG_LED_RED, &r);
    (void)read_reg_u8(dev, REG_LED_GRN, &g);
    (void)read_reg_u8(dev, REG_LED_BLU, &b);
    (void)read_reg_u8(dev, REG_LED_WHT, &w);
    (void)read_reg_u8(dev, REG_CTRL, &ctrl);
    LOG_INF("PIM447 regs: LED R=%u G=%u B=%u W=%u | CTRL=0x%02x", r, g, b, w, ctrl);
}

static void log_chip_id(const struct device *dev) {
    uint8_t lo = 0, hi = 0;
    if (read_reg_u8(dev, REG_CHIP_ID_L, &lo) == 0 && read_reg_u8(dev, REG_CHIP_ID_H, &hi) == 0) {
        uint16_t chip_id = ((uint16_t)hi << 8) | lo;
        LOG_INF("PIM447 chip id: 0x%04x", chip_id);
    }
}

static int trackball_pim447_init(const struct device *dev) {
    const struct trackball_pim447_config *config = dev->config;
    struct trackball_pim447_data *data = dev->data;

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

    /* Log chip id (useful sanity check) */
    log_chip_id(dev);

    /* Ensure awake before LED writes */
    int err = wake_if_sleeping(dev);
    if (err) {
        LOG_ERR("Failed to read/clear CTRL sleep bit: %d", err);
        return err;
    }

    /* Set initial LED RGBW from devicetree config */
    data->led_white = config->led_white;
    err = set_led_rgbw(dev, config->led_red, config->led_green, config->led_blue, config->led_white);
    if (err) {
        LOG_ERR("Failed to set initial LED color: %d", err);
        return err;
    }

    /* Read back LED regs so you can confirm writes are landing */
    log_led_regs(dev);

    k_work_init_delayable(&data->poll_work, poll_work_handler);

    /* Start polling */
    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));

    LOG_INF("PIM447 init ok (addr 0x%02x, poll %ums)", config->i2c.addr, config->poll_interval_ms);
    return 0;
}

/* Driver instantiation */
#define TRACKBALL_PIM447_INIT(inst)                                                        \
    static struct trackball_pim447_data trackball_pim447_data_##inst;                      \
                                                                                           \
    static const struct trackball_pim447_config trackball_pim447_config_##inst = {         \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
        .led_red = DT_INST_PROP_OR(inst, led_red, 0),                                      \
        .led_green = DT_INST_PROP_OR(inst, led_green, 0),                                  \
        .led_blue = DT_INST_PROP_OR(inst, led_blue, 0),                                    \
        .led_white = DT_INST_PROP_OR(inst, led_white, 0),                                  \
        .invert_x = DT_INST_PROP_OR(inst, invert_x, false),                                \
        .invert_y = DT_INST_PROP_OR(inst, invert_y, false),                                \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 64),                             \
        .move_factor = DT_INST_PROP_OR(inst, move_factor, 1),                              \
        .scroll_factor = DT_INST_PROP_OR(inst, scroll_factor, 1),                          \
        .poll_interval_ms = DT_INST_PROP_OR(inst, poll_interval_ms, 10),                   \
    };                                                                                     \
                                                                                           \
    DEVICE_DT_INST_DEFINE(inst, trackball_pim447_init, NULL,                               \
                          &trackball_pim447_data_##inst, &trackball_pim447_config_##inst,  \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, &trackball_pim447_api);

DT_INST_FOREACH_STATUS_OKAY(TRACKBALL_PIM447_INIT)
