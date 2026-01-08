/*
 * PIM447 Trackball driver (polling, no INT required)
 *
 * - Polls motion over I2C
 * - Emits Zephyr input events (REL_X/REL_Y or WHEEL/HWHEEL)
 * - LED troubleshooting:
 *   - Clears sleep bit in REG_CTRL (0xFE)
 *   - Writes RGBW as four single-register writes (0x00..0x03)
 *   - Optional blink test (no logs needed)
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

/* ---------- LED troubleshooting switches ---------- */
/* Set to 1 to blink LEDs for visual confirmation; set to 0 once verified */
#define PIM447_LED_BLINK_TEST 1
#define PIM447_LED_BLINK_PERIOD_MS 500
/* Blink uses WHITE channel at full brightness */
#define PIM447_LED_BLINK_W 255

/* Custom sensor attributes (private range) */
#define PIM447_ATTR_LED_RGB (SENSOR_ATTR_PRIV_START)
#define PIM447_ATTR_MODE (SENSOR_ATTR_PRIV_START + 1)

/* PIM447 registers */
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

#define MSK_CTRL_SLEEP   0x01

/* Switch pressed bit varies between implementations; support both */
#define MSK_SWITCH_PRESSED_BIT7 0x80
#define MSK_SWITCH_PRESSED_BIT0 0x01

/* Burst read LEFT..SWITCH */
#define MOTION_BURST_START REG_LEFT
#define MOTION_BURST_LEN   5

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

#if PIM447_LED_BLINK_TEST
    uint32_t last_led_toggle_ms;
    bool led_on;
#endif

    struct k_work_delayable poll_work;
};

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

    uint16_t poll_interval_ms;
};

static int read_reg_u8(const struct device *dev, uint8_t reg, uint8_t *value) {
    const struct trackball_pim447_config *config = dev->config;
    int err = i2c_write_read_dt(&config->i2c, &reg, 1, value, 1);
    return err;
}

static int write_reg_u8(const struct device *dev, uint8_t reg, uint8_t value) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[2] = {reg, value};
    int err = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    return err;
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

/* Read movement + switch */
static int read_motion_burst(const struct device *dev, int16_t *dx, int16_t *dy, uint8_t *pressed) {
    const struct trackball_pim447_config *config = dev->config;
    uint8_t buf[MOTION_BURST_LEN] = {0};

    int err = i2c_burst_read_dt(&config->i2c, MOTION_BURST_START, buf, sizeof(buf));
    if (err) {
        return err;
    }

    const uint8_t left  = buf[0];
    const uint8_t right = buf[1];
    const uint8_t up    = buf[2];
    const uint8_t down  = buf[3];
    const uint8_t sw    = buf[4];

    *dx = (int16_t)right - (int16_t)left;
    *dy = (int16_t)down  - (int16_t)up;

    /* pressed: accept either bit7 or bit0 */
    *pressed = ((sw & MSK_SWITCH_PRESSED_BIT7) || (sw & MSK_SWITCH_PRESSED_BIT0)) ? 1 : 0;

    /* Clear motion accumulators */
    (void)clear_motion_regs(dev);

    return 0;
}

/* Force device awake (clear sleep bit) */
static void pim447_force_awake(const struct device *dev) {
    uint8_t ctrl = 0;
    if (read_reg_u8(dev, REG_CTRL, &ctrl) == 0) {
        if (ctrl & MSK_CTRL_SLEEP) {
            ctrl &= ~MSK_CTRL_SLEEP;
            (void)write_reg_u8(dev, REG_CTRL, ctrl);
            k_sleep(K_MSEC(2));
        }
    }
}

/*
 * LED set via FOUR single-register writes (matches Pimoroni style).
 * This avoids relying on multi-byte auto-increment behavior.
 */
static int set_led_rgbw(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    struct trackball_pim447_data *data = dev->data;
    int err;

    err = write_reg_u8(dev, REG_LED_RED, r);
    if (err) return err;

    err = write_reg_u8(dev, REG_LED_GRN, g);
    if (err) return err;

    err = write_reg_u8(dev, REG_LED_BLU, b);
    if (err) return err;

    err = write_reg_u8(dev, REG_LED_WHT, w);
    if (err) return err;

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
        data->dx = data->dx * data->move_factor;
        data->dy = data->dy * data->move_factor;
    } else {
        data->dx = data->dx * data->scroll_factor;
        data->dy = data->dy * data->scroll_factor;
    }
}

/* Sensor API (optional; used only for custom attrs) */
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

/* Custom attributes: mode + set RGB (keeps current white) */
static int trackball_pim447_attr_set(const struct device *dev, enum sensor_channel chan,
                                     enum sensor_attribute attr, const struct sensor_value *val) {
    struct trackball_pim447_data *data = dev->data;

    ARG_UNUSED(chan);

    switch (attr) {
    case PIM447_ATTR_LED_RGB: {
        if (!val) {
            return -EINVAL;
        }
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

#if PIM447_LED_BLINK_TEST
    /* Visual-only LED test: blink WHITE every 500ms, and force awake periodically */
    uint32_t now = k_uptime_get_32();
    if ((now - data->last_led_toggle_ms) >= PIM447_LED_BLINK_PERIOD_MS) {
        data->last_led_toggle_ms = now;
        data->led_on = !data->led_on;

        /* Keep device awake (in case it boots sleeping or re-sleeps) */
        pim447_force_awake(dev);

        /* Blink white channel */
        (void)set_led_rgbw(dev, 0, 0, 0, data->led_on ? PIM447_LED_BLINK_W : 0);
    }
#endif

    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));
}

static void log_chip_id_best_effort(const struct device *dev) {
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
        return -ENODEV;
    }

    data->invert_x = config->invert_x;
    data->invert_y = config->invert_y;
    data->sensitivity = config->sensitivity;
    data->move_factor = config->move_factor;
    data->scroll_factor = config->scroll_factor;

    data->mode = 0; /* default MOVE */
    data->prev_button_state = 0;

    data->led_red = config->led_red;
    data->led_green = config->led_green;
    data->led_blue = config->led_blue;
    data->led_white = config->led_white;

#if PIM447_LED_BLINK_TEST
    data->last_led_toggle_ms = k_uptime_get_32();
    data->led_on = false;
#endif

    /* Optional sanity check if logging is enabled */
    log_chip_id_best_effort(dev);

    /* Force awake, then set initial LED from devicetree */
    pim447_force_awake(dev);
    (void)set_led_rgbw(dev, data->led_red, data->led_green, data->led_blue, data->led_white);

    k_work_init_delayable(&data->poll_work, poll_work_handler);

    const uint16_t interval = MAX(config->poll_interval_ms, 1);
    (void)k_work_schedule(&data->poll_work, K_MSEC(interval));

    return 0;
}

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
