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
u8 key_test_flag = 0;
u8 ir_test_flag = 0;
u8 servo_test_flag = 0;

/* 舵机测试: 0度<->180度循环, 每个角度保持3s(500ms x n) */
#define SERVO_TEST_HOLD_TICKS   20
static u8 servo_hold_cnt = 0;
static u8 servo_angle_phase = 0; /* 0:当前0度 1:当前180度 */

/* 测试模式总超时: 60s (500ms x 120), 超时自动退出 */
#define TEST_MODE_TIMEOUT_TICKS   360
static u16 test_mode_cnt = 0;

extern SERVO_CTRL *servo_ctrl;
extern KEY_CTRL *key_ctrl;
extern LED_CTRL *led_ctrl;
#if INFRARED_EN
extern INFRARED_CTRL *infrared_ctrl;
#endif

static void test_report(u8 item, u8 result)
{
    u8 payload[2];
    payload[0] = item;
    payload[1] = result;
    log_info("TEST_RESULT item=0x%02x result=0x%02x\n", item, result);
    send_cmd_by_uart(UART_DATA_TEST_RESULT, payload, 2, 0, 1);
}
void servo_test(void)//500ms
{
    int angle = 0;
    if(servo_test_flag != 1){
        return;
    }
    servo_hold_cnt++;
    if(servo_hold_cnt < SERVO_TEST_HOLD_TICKS){
        return;
    }
    servo_hold_cnt = 0;
    if(servo_angle_phase == 0){
        angle = 180;
        servo_angle_phase = 1;
    }else{
        angle = 0;
        servo_angle_phase = 0;
    }
    gd.dev_table[SERVO_DEV].dev_write(servo_ctrl,&angle,4);
}

void test_poll_500ms(void)//500ms
{
    if(!g_test_mode){
        return;
    }
    test_mode_cnt++;
    if(test_mode_cnt >= TEST_MODE_TIMEOUT_TICKS){
        log_info("test mode timeout, exit\n");
        test_mode_exit();
        return;
    }
    servo_test();
#if INFRARED_EN
    if(ir_test_flag == 1){
        ir_test_flag = 2;
        test_report(TEST_ITEM_IR, TEST_RESULT_OK);
    }
#endif
    if(key_test_flag == 1){
        key_test_flag = 2;
        test_report(TEST_ITEM_KEY, TEST_RESULT_OK);
    }
}
static void test_goto_step(u8 step)//open device
{
    switch (step) {
    case TEST_STEP_LED:
        if (!led_ctrl) {
            led_ctrl = gd.dev_table[LED_DEV].dev_open(NULL);
        }
        gd.dev_table[LED_DEV].dev_ioctl(led_ctrl, LED_CMD_ALL_FUNC_ON, 0);
        test_report(TEST_ITEM_LED, TEST_RESULT_OK);
        test_goto_step(TEST_STEP_KEY);
        break;

    case TEST_STEP_KEY:
        log_info("test wait key\n");
        if (!key_ctrl) {
            key_ctrl = gd.dev_table[KEY_DEV].dev_open(NULL);
        }
        test_goto_step(TEST_STEP_IR);
        break;

    case TEST_STEP_IR:
#if INFRARED_EN
        log_info("test wait ir\n");
        if (!infrared_ctrl) {
            infrared_ctrl = gd.dev_table[INFRARED_DEV].dev_open(NULL);
        }
#endif
        test_goto_step(TEST_STEP_SERVO);
        break;

    case TEST_STEP_SERVO:
        log_info("test servo: 0<->180 deg every 3s, human judge result\n");
        if (!servo_ctrl) {
            servo_ctrl = gd.dev_table[SERVO_DEV].dev_open(NULL);
        }
        /* 先到0度, 再每3s在0度/180度间循环, 供人工目视判断 */
        {
            int angle = 0;
            gd.dev_table[SERVO_DEV].dev_write(servo_ctrl,&angle,4);
        }
        /* PWM舵机无反馈, 直接判定PASS, 结果由人工判断 */
        test_report(TEST_ITEM_SERVO, TEST_RESULT_OK);
        servo_test_flag = 1;
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
    test_mode_cnt = 0;
    servo_hold_cnt = 0;
    servo_angle_phase = 0;
    test_goto_step(TEST_STEP_LED);
}

void test_mode_exit(void)
{
    if (!g_test_mode) {
        return;
    }
    log_info("test mode exit\n");
    if (led_ctrl) {
        gd.dev_table[LED_DEV].dev_ioctl(led_ctrl, LED_CMD_ALL_FUNC_OFF, 0);
    }
    test_mode_cnt = 0;
    servo_hold_cnt = 0;
    servo_angle_phase = 0;
    g_test_mode = 0;
    servo_test_flag = 0;
    ir_test_flag = 0;
    key_test_flag = 0;
}


