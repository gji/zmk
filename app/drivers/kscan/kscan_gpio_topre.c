/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_topre

#include <device.h>
#include <drivers/kscan.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct kscan_gpio_item_config {
    char *label;
    gpio_pin_t pin;
    gpio_flags_t flags;
};

// Helper macro
#define PWR_TWO(x) (1 << (x))

// Define GPIO cfg
#define _KSCAN_GPIO_ITEM_CFG_INIT(n, prop, idx)                                                    \
    {                                                                                              \
        .label = DT_INST_GPIO_LABEL_BY_IDX(n, prop, idx),                                          \
        .pin = DT_INST_GPIO_PIN_BY_IDX(n, prop, idx),                                              \
        .flags = DT_INST_GPIO_FLAGS_BY_IDX(n, prop, idx),                                          \
    },

#define _KSCAN_GPIO_SINGLE_ITEM_CFG_INIT(n, prop)                                                  \
    {                                                                                              \
        .label = DT_INST_GPIO_LABEL(n, prop),                                                      \
        .pin = DT_INST_GPIO_PIN(n, prop),                                                          \
        .flags = DT_INST_GPIO_FLAGS(n, prop),                                                      \
    }

// Define pin cfg
#define _KSCAN_GPIO_ROW_CFG_INIT(idx, n) _KSCAN_GPIO_ITEM_CFG_INIT(n, row_gpios, idx)
#define _KSCAN_GPIO_COL_CFG_INIT(idx, n) _KSCAN_GPIO_ITEM_CFG_INIT(n, col_gpios, idx)
#define _KSCAN_GPIO_COL_SEL_CFG_INIT(idx, n) _KSCAN_GPIO_ITEM_CFG_INIT(n, col_select_gpios, idx)
#define _KSCAN_GPIO_LED_CFG_INIT(idx, n) _KSCAN_GPIO_ITEM_CFG_INIT(n, led_gpios, idx)
#define _KSCAN_GPIO_ENABLE_CFG_INIT(n) _KSCAN_GPIO_SINGLE_ITEM_CFG_INIT(n, enable_gpios)
#define _KSCAN_GPIO_HYS_CFG_INIT(n) _KSCAN_GPIO_SINGLE_ITEM_CFG_INIT(n, hys_gpios)
#define _KSCAN_GPIO_KEY_CFG_INIT(n) _KSCAN_GPIO_SINGLE_ITEM_CFG_INIT(n, key_gpios)

// Check debounce config
#define CHECK_DEBOUNCE_CFG(n, a, b) COND_CODE_0(DT_INST_PROP(n, debounce_period), a, b)

// Define the row and column lengths
#define INST_MATRIX_ROWS(n) DT_INST_PROP_LEN(n, row_gpios)
#define INST_MATRIX_COLS(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_MATRIX_COL_SELS(n) DT_INST_PROP_LEN(n, col_select_gpios)
#define INST_LEDS(n) DT_INST_PROP_LEN(n, led_gpios)
#define INST_MATRIX_MUX_ROWS(n) PWR_TWO(INST_MATRIX_ROWS(n))
#define INST_MATRIX_MUX_COLS(n) PWR_TWO(INST_MATRIX_COLS(n)) * INST_MATRIX_COL_SELS(n)
#define ACTIVE_POLL_INTERVAL(n) DT_INST_PROP(n, active_polling_interval_msec)
#define IDLE_POLL_INTERVAL(n) DT_INST_PROP(n, idle_polling_interval_msec)
#define SLEEP_POLL_INTERVAL(n) DT_INST_PROP(n, sleep_polling_interval_msec)
#define MAX_MUX_ROW(n) COND_CODE_0(DT_INST_PROP(n, max_row),                                       \
                                   (INST_MATRIX_MUX_ROWS(n)),                                      \
                                   (DT_INST_PROP(n, max_row)))                                     \

#define GPIO_INST_INIT(n)                                                                          \
    struct kscan_gpio_irq_callback_##n {                                                           \
        struct CHECK_DEBOUNCE_CFG(n, (k_work), (k_delayed_work)) * work;                           \
        struct gpio_callback callback;                                                             \
        const struct device *dev;                                                                  \
    };                                                                                             \
                                                                                                   \
    struct kscan_gpio_config_##n {                                                                 \
        struct kscan_gpio_item_config rows[INST_MATRIX_ROWS(n)];                                   \
        struct kscan_gpio_item_config cols[INST_MATRIX_COLS(n)];                                   \
        struct kscan_gpio_item_config col_sels[INST_MATRIX_COL_SELS(n)];                           \
        struct kscan_gpio_item_config leds[INST_LEDS(n)];                                          \
        struct kscan_gpio_item_config enable;                                                      \
        struct kscan_gpio_item_config hys;                                                         \
        struct kscan_gpio_item_config key;                                                         \
    };                                                                                             \
                                                                                                   \
    struct kscan_gpio_data_##n {                                                                   \
        kscan_callback_t callback;                                                                 \
        struct k_timer poll_timer;                                                                 \
        struct CHECK_DEBOUNCE_CFG(n, (k_work), (k_delayed_work)) work;                             \
        bool matrix_state[INST_MATRIX_MUX_ROWS(n)][INST_MATRIX_MUX_COLS(n)];                       \
        const struct device *rows[INST_MATRIX_ROWS(n)];                                            \
        const struct device *cols[INST_MATRIX_COLS(n)];                                            \
        const struct device *col_sels[INST_MATRIX_COL_SELS(n)];                                    \
        const struct device *leds[INST_LEDS(n)];                                                   \
        const struct device *enable;                                                               \
        const struct device *hys;                                                                  \
        const struct device *key;                                                                  \
        const struct device *dev;                                                                  \
    };                                                                                             \
                                                                                                   \
    static struct kscan_gpio_data_##n kscan_gpio_data_##n = {                                      \
        .rows = {[INST_MATRIX_ROWS(n) - 1] = NULL},                                                \
        .cols = {[INST_MATRIX_COLS(n) - 1] = NULL},                                                \
        .col_sels = {[INST_MATRIX_COL_SELS(n) - 1] = NULL}                                         \
    };                                                                                             \
                                                                                                   \
    static const struct kscan_gpio_config_##n kscan_gpio_config_##n = {                            \
        .rows = {UTIL_LISTIFY(INST_MATRIX_ROWS(n), _KSCAN_GPIO_ROW_CFG_INIT, n)},                  \
        .cols = {UTIL_LISTIFY(INST_MATRIX_COLS(n), _KSCAN_GPIO_COL_CFG_INIT, n)},                  \
        .col_sels = {UTIL_LISTIFY(INST_MATRIX_COL_SELS(n), _KSCAN_GPIO_COL_SEL_CFG_INIT, n)},      \
        .leds = {UTIL_LISTIFY(INST_LEDS(n), _KSCAN_GPIO_LED_CFG_INIT, n)},                         \
        .enable = _KSCAN_GPIO_ENABLE_CFG_INIT(n),                                                  \
        .hys = _KSCAN_GPIO_HYS_CFG_INIT(n),                                                        \
        .key = _KSCAN_GPIO_KEY_CFG_INIT(n),                                                        \
    };                                                                                             \
                                                                                                   \
    /* POLLING SETUP */                                                                            \
    static void kscan_gpio_timer_handler(struct k_timer *timer) {                                  \
        struct kscan_gpio_data_##n *data =                                                         \
            CONTAINER_OF(timer, struct kscan_gpio_data_##n, poll_timer);                           \
        k_work_submit(&data->work.work);                                                           \
    }                                                                                              \
    /* Read the state of the input GPIOs */                                                        \
    /* This is the core matrix_scan func */                                                        \
    static int kscan_gpio_read_##n(const struct device *dev) {                                     \
        struct kscan_gpio_data_##n *data = dev->data;                                              \
        const struct kscan_gpio_config_##n *cfg = dev->config;                                     \
        static int read_state[INST_MATRIX_MUX_ROWS(n)][INST_MATRIX_MUX_COLS(n)];                   \
        for (uint8_t col = 0; col < INST_MATRIX_MUX_COLS(n); col++) {                              \
            for (uint8_t row = 0; row < MAX_MUX_ROW(n); row++) {                                   \
                /* For a given row/col, figure out row/col mux and active col_sel pins */          \
                for (uint8_t row_bit = 0; row_bit < INST_MATRIX_ROWS(n); row_bit++) {              \
                    gpio_pin_set(data->rows[row_bit],                                              \
                                 cfg->rows[row_bit].pin,                                           \
                                 (row >> row_bit) & 1);                                            \
                }                                                                                  \
                for (uint8_t col_bit = 0; col_bit < INST_MATRIX_COLS(n); col_bit++) {              \
                    gpio_pin_set(data->cols[col_bit],                                              \
                                 cfg->cols[col_bit].pin,                                           \
                                 (col >> col_bit) & 1);                                            \
                }                                                                                  \
                for (uint8_t col_sel = 0; col_sel < INST_MATRIX_COL_SELS(n); col_sel++) {          \
                    uint8_t active_col_sel = col / PWR_TWO(INST_MATRIX_COLS(n));                   \
                    gpio_pin_set(data->col_sels[col_sel],                                          \
                                 cfg->col_sels[col_sel].pin,                                       \
                                 col_sel == active_col_sel);                                       \
                }                                                                                  \
                k_busy_wait(5);                                                                    \
                gpio_pin_set(data->hys, cfg->hys.pin, data->matrix_state[row][col]);               \
                k_busy_wait(5);                                                                    \
                unsigned int lock_key = irq_lock();                                                \
                gpio_pin_set(data->enable, cfg->enable.pin, 1);                                    \
                k_busy_wait(5);                                                                    \
                bool res = gpio_pin_get(data->key, cfg->key.pin) > 0;                              \
                read_state[row][col] = res;                                                        \
                irq_unlock(lock_key);                                                              \
                gpio_pin_set(data->enable, cfg->enable.pin, 0);                                    \
            }                                                                                      \
                                                                                                   \
        }                                                                                          \
        for (int r = 0; r < INST_MATRIX_MUX_ROWS(n); r++) {                                        \
            for (int c = 0; c < INST_MATRIX_MUX_COLS(n); c++) {                                    \
                bool pressed = read_state[r][c];                                                   \
                if (pressed != data->matrix_state[r][c]) {                                         \
                    LOG_DBG("Sending event at %d,%d state %s", r, c, (pressed ? "on" : "off"));    \
                    data->matrix_state[r][c] = pressed;                                            \
                    data->callback(dev, r, c, pressed);                                            \
                }                                                                                  \
            }                                                                                      \
        }                                                                                          \
        return 0;                                                                                  \
    }                                                                                              \
                                                                                                   \
    static void kscan_gpio_work_handler_##n(struct k_work *work) {                                 \
        struct kscan_gpio_data_##n *data = CONTAINER_OF(work, struct kscan_gpio_data_##n, work);   \
        kscan_gpio_read_##n(data->dev);                                                            \
    }                                                                                              \
                                                                                                   \
    /* KSCAN API configure function */                                                             \
    static int kscan_gpio_configure_##n(const struct device *dev, kscan_callback_t callback) {     \
        LOG_DBG("KSCAN API configure");                                                            \
        struct kscan_gpio_data_##n *data = dev->data;                                              \
        if (!callback) {                                                                           \
            return -EINVAL;                                                                        \
        }                                                                                          \
        data->callback = callback;                                                                 \
        LOG_DBG("Configured GPIO %d", n);                                                          \
        return 0;                                                                                  \
    };                                                                                             \
                                                                                                   \
    /* KSCAN API enable function */                                                                \
    static int kscan_gpio_enable_##n(const struct device *dev) {                                   \
        LOG_DBG("KSCAN API enable");                                                               \
        struct kscan_gpio_data_##n *data = dev->data;                                              \
        k_timer_start(&data->poll_timer,                                                           \
                      K_MSEC(ACTIVE_POLL_INTERVAL(n)),                                             \
                      K_MSEC(ACTIVE_POLL_INTERVAL(n)));                                            \
        return 0;                                                                                  \
    };                                                                                             \
                                                                                                   \
    /* KSCAN API disable function */                                                               \
    static int kscan_gpio_disable_##n(const struct device *dev) {                                  \
        LOG_DBG("KSCAN API disable");                                                              \
        struct kscan_gpio_data_##n *data = dev->data;                                              \
        k_timer_stop(&data->poll_timer);                                                           \
        return 0;                                                                                  \
    };                                                                                             \
                                                                                                   \
    /* KSCAN modify poll rate based on activity state */                                           \
    int kscan_gpio_activity_event_handler_##n(const zmk_event_t *eh) {                             \
        int poll_interval;                                                                         \
        struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);                 \
        if (ev == NULL) {                                                                          \
            return -ENOTSUP;                                                                       \
        }                                                                                          \
        switch (ev->state) {                                                                       \
        case ZMK_ACTIVITY_ACTIVE:                                                                  \
            poll_interval = ACTIVE_POLL_INTERVAL(n);                                               \
            break;                                                                                 \
        case ZMK_ACTIVITY_IDLE:                                                                    \
            poll_interval = IDLE_POLL_INTERVAL(n);                                                 \
            break;                                                                                 \
        case ZMK_ACTIVITY_SLEEP:                                                                   \
            poll_interval = SLEEP_POLL_INTERVAL(n);                                                \
            break;                                                                                 \
        default:                                                                                   \
            LOG_WRN("Unhandled activity state: %d", ev->state);                                    \
            return -EINVAL;                                                                        \
        }                                                                                          \
        LOG_DBG("Setting poll interval to %d", poll_interval);                                     \
        k_timer_start(&kscan_gpio_data_##n.poll_timer,                                             \
                      K_MSEC(poll_interval),                                                       \
                      K_MSEC(poll_interval));                                                      \
        return 0;                                                                                  \
    }                                                                                              \
                                                                                                   \
    ZMK_LISTENER(kscan_##n, kscan_gpio_activity_event_handler_##n);                                \
    ZMK_SUBSCRIPTION(kscan_##n, zmk_activity_state_changed);                                       \
                                                                                                   \
    /* GPIO init function*/                                                                        \
    static int kscan_gpio_init_##n(const struct device *dev) {                                     \
        LOG_DBG("KSCAN GPIO init");                                                                \
        struct kscan_gpio_data_##n *data = dev->data;                                              \
        const struct kscan_gpio_config_##n *cfg = dev->config;                                     \
        int err;                                                                                   \
        const struct kscan_gpio_item_config *in_cfg = &cfg->key;                                   \
        data->key = device_get_binding(in_cfg->label);                                             \
        if (!data->key) {                                                                          \
            LOG_ERR("Unable to find input GPIO device");                                           \
            return -EINVAL;                                                                        \
        }                                                                                          \
        err = gpio_pin_configure(data->key, in_cfg->pin, GPIO_INPUT | in_cfg->flags);              \
        if (err) {                                                                                 \
            LOG_ERR("Unable to configure pin %d on %s for input", in_cfg->pin, in_cfg->label);     \
            return err;                                                                            \
        } else {                                                                                   \
            LOG_DBG("Configured pin %d on %s for input", in_cfg->pin, in_cfg->label);              \
        }                                                                                          \
        /* configure output devices*/                                                              \
        const struct device **output_device_groups[6] = {                                          \
            data->rows,                                                                            \
            data->cols,                                                                            \
            data->col_sels,                                                                        \
            data->leds,                                                                            \
            &data->enable,                                                                         \
            &data->hys,                                                                            \
        };                                                                                         \
        const struct kscan_gpio_item_config *output_device_configs[6] = {                          \
            cfg->rows,                                                                             \
            cfg->cols,                                                                             \
            cfg->col_sels,                                                                         \
            cfg->leds,                                                                             \
            &cfg->enable,                                                                          \
            &cfg->hys,                                                                             \
        };                                                                                         \
        const uint16_t output_device_group_lengths[6] = {                                          \
            INST_MATRIX_ROWS(n),                                                                   \
            INST_MATRIX_COLS(n),                                                                   \
            INST_MATRIX_COL_SELS(n),                                                               \
            INST_LEDS(n),                                                                          \
            1,                                                                                     \
            1,                                                                                     \
        };                                                                                         \
        for (int i=0; i<6; i++) {                                                                  \
            for(int j=0; j<output_device_group_lengths[i]; j++) {                                  \
                const struct kscan_gpio_item_config *out_cfg = &output_device_configs[i][j];       \
                output_device_groups[i][j] = device_get_binding(out_cfg->label);                   \
                LOG_DBG("Programming %s, i=%d, j=%d", out_cfg->label, i, j);                       \
                if (!output_device_groups[i][j]) {                                                 \
                    LOG_ERR("Unable to find output GPIO device %s", out_cfg->label);               \
                    return -EINVAL;                                                                \
                }                                                                                  \
                err = gpio_pin_configure(                                                          \
                    output_device_groups[i][j],                                                    \
                    out_cfg->pin,                                                                  \
                    GPIO_OUTPUT_INACTIVE | out_cfg->flags);                                        \
                if (err) {                                                                         \
                    LOG_ERR(                                                                       \
                        "Unable to configure pin %d on %s for output",                             \
                        out_cfg->pin,                                                              \
                        out_cfg->label                                                             \
                    );                                                                             \
                    return err;                                                                    \
                } else {                                                                           \
                    LOG_DBG("Configured pin %d on %s for output", out_cfg->pin, out_cfg->label);   \
                }                                                                                  \
            }                                                                                      \
        }                                                                                          \
        data->dev = dev;                                                                           \
                                                                                                   \
        k_timer_init(&data->poll_timer, kscan_gpio_timer_handler, NULL);                           \
                                                                                                   \
        (CHECK_DEBOUNCE_CFG(n, (k_work_init), (k_delayed_work_init)))(                             \
            &data->work, kscan_gpio_work_handler_##n);                                             \
        return 0;                                                                                  \
    }                                                                                              \
                                                                                                   \
    static const struct kscan_driver_api gpio_driver_api_##n = {                                   \
        .config = kscan_gpio_configure_##n,                                                        \
        .enable_callback = kscan_gpio_enable_##n,                                                  \
        .disable_callback = kscan_gpio_disable_##n,                                                \
    };                                                                                             \
                                                                                                   \
    DEVICE_AND_API_INIT(kscan_gpio_##n, DT_INST_LABEL(n), kscan_gpio_init_##n,                     \
                        &kscan_gpio_data_##n, &kscan_gpio_config_##n, APPLICATION,                 \
                        CONFIG_APPLICATION_INIT_PRIORITY, &gpio_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INST_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
