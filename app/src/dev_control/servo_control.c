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

/* 角度渐变速度: 每个24ms刷新周期移动的度数(可调, 2约等于83度/秒) */
#define SERVO_MOVE_SPEED       2   /* 常规速度(0~160度) */
#define SERVO_MOVE_SPEED_SLOW  1   /* 高端区单次步进度数 */
#define SERVO_SLOW_START_ANGLE 160 /* 当前角度达到该值后进入高端减速区 */
#define SERVO_SLOW_TICKS       2   /* 高端区每N个刷新周期走一步(越大越柔) */

SERVO_PLATFORM_DATA servo_platform_data = {
    .high_time = 1500,
    .pins = IO_PORTA_12,
};

static u32 servo_angle_to_high_time(u32 angle)
{
    if (angle > 178) {
        return 2292;
    }
    return angle * 9 + 500;
}

static void servo_set_target(SERVO_CTRL *dev, u32 angle)
{
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
    /* 高端区(160~180度)减速收尾, 低端区(0~20度)保持常规速度 */
    if (dev->cur_angle >= SERVO_SLOW_START_ANGLE) {
        dev->slow_acc++;
        if (dev->slow_acc >= SERVO_SLOW_TICKS) {
            dev->slow_acc = 0;
            step = SERVO_MOVE_SPEED_SLOW;
        } else {
            step = 0;
        }
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
    dev_ctrl->slow_acc = 0;
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
