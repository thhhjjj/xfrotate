#include "test_mode.h"
#include "uart_communication.h"
#include "user_define.h"
#include "servo_control.h"
#include "infrared_control.h"
#include "led_control.h"
#include "key_control.h"
#include "gpio.h"
#include "log.h"
#include <string.h>

#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[test_mode]"

enum {
    TEST_STEP_IDLE = 0,
    TEST_STEP_LED,
    TEST_STEP_KEY,
    TEST_STEP_IR,
    TEST_STEP_SERVO,
    TEST_STEP_DONE,
};

u8 g_test_mode = 0;

static u8 s_step = TEST_STEP_IDLE;
static u16 s_timeout = 0;
static u8 s_ir_armed = 0;
static u8 s_sw0 = 0;
static u8 s_sw90 = 0;
static u8 s_sw180 = 0;
static u8 s_servo_started = 0;

extern SERVO_CTRL *servo_ctrl;
extern KEY_CTRL *key_ctrl;
extern LED_CTRL *led_ctrl;
extern INFRARED_CTRL *infrared_ctrl;

static void test_report(u8 item, u8 result)
{
    u8 payload[2];
    payload[0] = item;
    payload[1] = result;
    log_info("TEST_RESULT item=0x%02x result=0x%02x\n", item, result);
    send_cmd_by_uart(UART_DATA_TEST_RESULT, payload, 2, 0, 1);
}

static void test_timeout_reset(void)
{
    s_timeout = 0;
}

static u8 servo_read_sw(u32 pin)
{
    return (u8)gpio_read(pin);
}

static void servo_snapshot_switch(void)
{
    if (!servo_ctrl || !servo_ctrl->platform_data) {
        s_sw0 = s_sw90 = s_sw180 = 0xff;
        return;
    }
    s_sw0 = servo_read_sw(servo_ctrl->platform_data->angle0_switch_pins);
    s_sw90 = servo_read_sw(servo_ctrl->platform_data->angle90_switch_pins);
    s_sw180 = servo_read_sw(servo_ctrl->platform_data->angle180_switch_pins);
}

static u8 servo_switch_changed(void)
{
    if (!servo_ctrl || !servo_ctrl->platform_data) {
        return 0;
    }
    if (servo_read_sw(servo_ctrl->platform_data->angle0_switch_pins) != s_sw0) {
        return 1;
    }
    if (servo_read_sw(servo_ctrl->platform_data->angle90_switch_pins) != s_sw90) {
        return 1;
    }
    if (servo_read_sw(servo_ctrl->platform_data->angle180_switch_pins) != s_sw180) {
        return 1;
    }
    return 0;
}

static void servo_start_move(void)
{
    u16 target = 45;
    if (!servo_ctrl) {
        return;
    }
    if (servo_ctrl->cur_angle < 90) {
        target = 135;
    } else {
        target = 45;
    }
    s_servo_started = 1;
    servo_snapshot_switch();
    gd.dev_table[SERVO_DEV].dev_ioctl(servo_ctrl, SERVO_CMD_SET_ANGLE, (u32)target);
    log_info("test servo move -> %d\n", target);
}

static void test_goto_step(u8 step)
{
    s_step = step;
    test_timeout_reset();

    switch (step) {
    case TEST_STEP_LED:
        if (led_ctrl) {
            gd.dev_table[LED_DEV].dev_ioctl(led_ctrl, LED_CMD_ALL_FUNC_ON, 0);
        }
        test_report(TEST_ITEM_LED, TEST_RESULT_OK);
        test_goto_step(TEST_STEP_KEY);
        break;

    case TEST_STEP_KEY:
        log_info("test wait key\n");
        break;

    case TEST_STEP_IR:
        s_ir_armed = 0;
        if (infrared_ctrl && !infrared_ctrl->human_flag) {
            s_ir_armed = 1;
        }
        log_info("test wait ir, armed=%d\n", s_ir_armed);
        break;

    case TEST_STEP_SERVO:
        s_servo_started = 0;
        servo_start_move();
        log_info("test wait servo switch\n");
        break;

    case TEST_STEP_DONE:
        log_info("test mode done, auto exit\n");
        test_mode_exit();
        break;

    default:
        break;
    }
}

void test_mode_enter(void)
{
    if (g_ota_busy) {
        log_info("test mode reject: ota busy\n");
        return;
    }
    if (g_test_mode) {
        test_mode_exit();
    }
    log_info("test mode enter\n");
    g_test_mode = 1;
    test_goto_step(TEST_STEP_LED);
}

void test_mode_exit(void)
{
    if (!g_test_mode && s_step == TEST_STEP_IDLE) {
        return;
    }
    log_info("test mode exit\n");
    if (led_ctrl) {
        gd.dev_table[LED_DEV].dev_ioctl(led_ctrl, LED_CMD_ALL_FUNC_OFF, 0);
    }
    if (servo_ctrl && servo_ctrl->platform_data) {
        servo_ctrl->platform_data->mov_dir = STOP;
        servo_ctrl->platform_data->open_time = 0;
    }
    g_test_mode = 0;
    s_step = TEST_STEP_IDLE;
    s_timeout = 0;
    s_ir_armed = 0;
    s_servo_started = 0;
}

void test_mode_on_key(u8 key_event)
{
    (void)key_event;
    if (!g_test_mode || s_step != TEST_STEP_KEY) {
        return;
    }
    test_report(TEST_ITEM_KEY, TEST_RESULT_OK);
    test_goto_step(TEST_STEP_IR);
}

void test_mode_poll_24ms(void)
{
    if (!g_test_mode) {
        return;
    }

    if (s_step == TEST_STEP_IR) {
        if (!infrared_ctrl) {
            return;
        }
        if (!s_ir_armed) {
            if (!infrared_ctrl->human_flag) {
                s_ir_armed = 1;
            }
            return;
        }
        if (infrared_ctrl->human_flag) {
            test_report(TEST_ITEM_IR, TEST_RESULT_OK);
            test_goto_step(TEST_STEP_SERVO);
        }
        return;
    }

    if (s_step == TEST_STEP_SERVO) {
        if (!s_servo_started) {
            servo_start_move();
            return;
        }
        if (servo_switch_changed()) {
            test_report(TEST_ITEM_SERVO, TEST_RESULT_OK);
            test_goto_step(TEST_STEP_DONE);
        }
    }
}

void test_mode_poll_500ms(void)
{
    u8 item;

    if (!g_test_mode) {
        return;
    }
    if (s_step != TEST_STEP_KEY && s_step != TEST_STEP_IR && s_step != TEST_STEP_SERVO) {
        return;
    }

    s_timeout++;
    if (s_timeout < TEST_ITEM_TIMEOUT_TICKS) {
        return;
    }

    switch (s_step) {
    case TEST_STEP_KEY:
        item = TEST_ITEM_KEY;
        test_report(item, TEST_RESULT_FAIL);
        test_goto_step(TEST_STEP_IR);
        break;
    case TEST_STEP_IR:
        item = TEST_ITEM_IR;
        test_report(item, TEST_RESULT_FAIL);
        test_goto_step(TEST_STEP_SERVO);
        break;
    case TEST_STEP_SERVO:
        item = TEST_ITEM_SERVO;
        test_report(item, TEST_RESULT_FAIL);
        test_goto_step(TEST_STEP_DONE);
        break;
    default:
        break;
    }
}
