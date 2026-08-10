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
u8 servo_left_switch = 0;
u8 servo_mid_switch = 0;
u8 servo_right_switch = 0;
static u8 tempcnt = 0;

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
void servo_test(void)//1s
{
    int angle = 0;
    if(servo_test_flag==1){
        angle = 180;
        gd.dev_table[SERVO_DEV].dev_write(servo_ctrl,&angle,4);
        servo_test_flag= 2;
    }else if(servo_test_flag==2){
        servo_test_flag= 1;
        angle = 0;
        gd.dev_table[SERVO_DEV].dev_write(servo_ctrl,&angle,4);
    }
}

void test_poll_500ms(void)//500ms
{
    if(!g_test_mode){
        return;
    }
    tempcnt++;
    if(tempcnt>=2){
        servo_test();     
    }
    if((tempcnt>=40)&&(servo_test_flag<=2)){//20s
        servo_test_flag = 4;
    }
    if(ir_test_flag == 1){
        ir_test_flag = 2;
        test_report(TEST_ITEM_IR, TEST_RESULT_OK);
    }
    if(key_test_flag == 1){
        key_test_flag = 2;
        test_report(TEST_ITEM_KEY, TEST_RESULT_OK);
    }
    if(servo_test_flag==3){
        servo_test_flag = 5;
        test_report(TEST_ITEM_SERVO, TEST_RESULT_OK);
    }else if(servo_test_flag==4){
        servo_test_flag = 5;
        test_report(TEST_ITEM_SERVO, TEST_RESULT_FAIL);
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
        log_info("test wait ir\n");
        if (!infrared_ctrl) {
            infrared_ctrl = gd.dev_table[INFRARED_DEV].dev_open(NULL);
        }
        test_goto_step(TEST_STEP_SERVO);
        break;

    case TEST_STEP_SERVO:
        log_info("test wait servo switch\n");
        if (!servo_ctrl) {
            servo_ctrl = gd.dev_table[SERVO_DEV].dev_open(NULL);
        }
        servo_test_flag=1;
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
    if (!g_test_mode) {
        return;
    }
    log_info("test mode exit\n");
    if (led_ctrl) {
        gd.dev_table[LED_DEV].dev_ioctl(led_ctrl, LED_CMD_ALL_FUNC_OFF, 0);
    }
    tempcnt = 0;
    g_test_mode = 0;
    servo_test_flag = 0;
    ir_test_flag = 0;
    key_test_flag = 0;
}


