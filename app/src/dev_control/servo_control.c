#include "servo_control.h"
#include "mcpwm.h"
#include "gpio.h"

#include "my_malloc.h"
#include "malloc.h"
#include "log.h"
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[servo_control]"

#define IO_PWM IO_PORTA_12
#define IO_VMA_PWR IO_PORTB_05

#define IO_PWM_SET(IO_LEVEL)do{     \
    gpio_write(IO_PWM, IO_LEVEL);    \
}while(0)               

/* 统一区域速度: 每个24ms刷新周期移动的度数(可调, 2约等于83度/秒) */
#define SERVO_MOVE_SPEED       2   /* 全行程统一速度, 无减速区 */

/* 死区角度: 0~x度和y~180度为死区, 有效行程范围[x, y] */
#define SERVO_ANGLE_MIN     5   /* 死区下界x: 目标小于x按x占空比输出 */
#define SERVO_ANGLE_MAX     175 /* 死区上界y: 目标大于y按y占空比输出 */

SERVO_PLATFORM_DATA servo_platform_data = {
    .high_time = 1500,
    .pins = IO_PORTA_12,
};

static u32 servo_angle_to_high_time(u32 angle)
{
    if (angle < SERVO_ANGLE_MIN) {
        angle = SERVO_ANGLE_MIN;
    } else if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }
    return angle * 9 + 500;
}

static void servo_set_target(SERVO_CTRL *dev, u32 angle)
{
    /* 死区钳位: 小于x按x处理, 大于y按y处理 */
    if (angle < SERVO_ANGLE_MIN) {
        angle = SERVO_ANGLE_MIN;
    } else if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }
    dev->str_angle = dev->cur_angle;
    dev->obj_angle = angle;
    log_info("servo target:%d,start:%d\n", angle, dev->str_angle);
}

static void servo_gradient_update(SERVO_CTRL *dev)
{
    u32 step = SERVO_MOVE_SPEED;

    if (dev->cur_angle == dev->obj_angle) {
        return;
    }
    if (dev->cur_angle < dev->obj_angle) {
        dev->cur_angle += step;
        if (dev->cur_angle > dev->obj_angle) {
            dev->cur_angle = dev->obj_angle;
        }
    } else {
        if (dev->cur_angle > step) {
            dev->cur_angle -= step;
        } else {
            dev->cur_angle = 0;
        }
        if (dev->cur_angle < dev->obj_angle) {
            dev->cur_angle = dev->obj_angle;
        }
    }
    dev->platform_data->high_time = servo_angle_to_high_time(dev->cur_angle);
}

void servo_io_init(void)
{
    gpio_set_direction(IO_VMA_PWR, 0);      
    gpio_write(IO_VMA_PWR, 1);  
    
    gpio_set_direction(IO_PWM, 0);
    gpio_write(IO_PWM, 0);  
}
void soft_pwm_set(SERVO_CTRL *dev)
{
    if(dev == NULL){
        return;
    }
    /* 先按速度宏做角度渐变, 再输出当前角度对应的占空比 */
    servo_gradient_update(dev);
    u16 high_time = dev->platform_data->high_time;
    //log_info("high_time = %d\n",high_time);
    if((high_time < 500)||(high_time > 2500)){
        return;
    }
    local_irq_disable();
    IO_PWM_SET(1);
    udelay(high_time);
    IO_PWM_SET(0);
    local_irq_enable();
}
void *servo_open(void *dev)
{
    SERVO_CTRL *dev_ctrl = malloc(sizeof(SERVO_CTRL));
    if(dev_ctrl == NULL){
        return NULL;
    }
    dev_ctrl->str_angle = 90;
    dev_ctrl->cur_angle = 90;
    dev_ctrl->obj_angle = 90;
    dev_ctrl->platform_data = &servo_platform_data;
    dev_ctrl->platform_data->high_time = servo_angle_to_high_time(dev_ctrl->cur_angle);
    return dev_ctrl;
}

void *servo_release(void *dev)
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    if(dev_ctrl == NULL){
        return NULL;
    }
    free(dev_ctrl);
    dev_ctrl = NULL;
    return NULL;
}

void *servo_read(void *dev)
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    return &dev_ctrl->cur_angle;
}

void *servo_write(void *dev, void *data, u32 len)
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    u32 angle = *(u32 *)data;
    if (angle > 180) {
        return NULL;
    }
    servo_set_target(dev_ctrl, angle);
    return NULL;
}
void *servo_ioctl(void *dev, u32 cmd, u32 arg)
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    switch (cmd) {
        case SERVO_CMD_SET_ANGLE:
            if (arg > 180) {
                return NULL;
            }
            servo_set_target(dev_ctrl, arg);
            return NULL;
            break;
        default:
            return NULL;
    }
    return NULL;
}
