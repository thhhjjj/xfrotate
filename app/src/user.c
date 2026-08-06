#include "mydev_manage.h"
#include "user_define.h"
#include "typedef.h"
#include "log.h"
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[user]"
#include "log.h"
#include "servo_control.h"
#include "infrared_control.h"
#include "key_control.h"
#include "led_control.h"
GD_T gd ={
    .dev_table = dev_table_t,
};

SERVO_CTRL *servo_ctrl = NULL;
KEY_CTRL *key_ctrl = NULL;
LED_CTRL *led_ctrl = NULL;
INFRARED_CTRL *infrared_ctrl = NULL;

void mydev_open(GD_T *gd)
{
    servo_ctrl = gd->dev_table[SERVO_DEV].dev_open(NULL);
    key_ctrl = gd->dev_table[KEY_DEV].dev_open(NULL);
    led_ctrl = gd->dev_table[LED_DEV].dev_open(NULL);
    infrared_ctrl = gd->dev_table[INFRARED_DEV].dev_open(NULL);
    if(servo_ctrl == NULL){
        log_error("servo_open failed");
        return;
    }
    if(key_ctrl == NULL){
        log_error("key_open failed");
        return;
    }
    if(led_ctrl == NULL){
        log_error("led_open failed");
        return;
    }
    if(infrared_ctrl == NULL){
        log_error("infrared_open failed");
        return;
    }
    log_info("all dev_open success");
}

void mydev_close(GD_T *gd)
{
    if(servo_ctrl == NULL){
        log_error("servo_close failed");
        return;
    }
    if(key_ctrl == NULL){
        log_error("key_close failed");
        return;
    }
    if(led_ctrl == NULL){
        log_error("led_close failed");
        return;
    }
    if(infrared_ctrl == NULL){
        log_error("infrared_close failed");
        return;
    }
    gd->dev_table[SERVO_DEV].dev_release(servo_ctrl);
    gd->dev_table[KEY_DEV].dev_release(key_ctrl);
    gd->dev_table[LED_DEV].dev_release(led_ctrl);
    gd->dev_table[INFRARED_DEV].dev_release(infrared_ctrl);
    servo_ctrl = NULL;
    key_ctrl = NULL;
    led_ctrl = NULL;
    infrared_ctrl = NULL;   
    log_info("all dev_close success");
}

void user_init(void)
{
    mydev_open(&gd);
}

void user_deinit(void)
{
    log_info("===user_deinit===");
    mydev_close(&gd);
}
