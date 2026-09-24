#include "servo_control.h"
#include "gpio.h"
#include "sfr.h"
#include "clock.h"

#include "my_malloc.h"
#include "malloc.h"
#include "log.h"
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[servo_control]"

#define IO_PWM IO_PORTA_12
#define IO_VMA_PWR IO_PORTB_05

/* MCPWM CH3 固定占用 PA12, IOMC0[19] */
#define SERVO_MCPWM_CH      3
#define SERVO_PWM_CKPS      6       /* TCK / 2^6 = /64 */
#define SERVO_PWM_FREQ      100     /* 10ms, 比 50Hz 更能跟上 4ms 插补 */
#define SERVO_PULSE_MIN_US  500
#define SERVO_PULSE_MAX_US  2600

/* 4ms 刷新: 按脉宽微秒插补, 避免整度跳变一顿一顿 */
#define SERVO_STEP_US_CRUISE   3    /* 3us/4ms ≈ 0.33°/4ms ≈ 83°/s */
#define SERVO_STEP_US_EASE     1    /* 起停最小步 */
#define SERVO_EASE_US          90   /* 两端约 10° 加减速 */

/* 死区角度: 0~x度和y~180度为死区, 有效行程范围[x, y] */
#define SERVO_ANGLE_MIN     5   /* 死区下界x: 目标小于x按x占空比输出 */
#define SERVO_ANGLE_MAX     175 /* 死区上界y: 目标大于y按y占空比输出 */

/* 预分频后的 PWM 计数时钟, 及周期对应的 PR (手册 N = CMP/(PR+1)) */
static u32 servo_pwm_clk;
static u16 servo_pwm_pr;
static u8 servo_pwm_ready;

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

static u32 servo_ease_step(u32 traveled_us, u32 remain_us)
{
    u32 step = SERVO_STEP_US_CRUISE;
    u32 ease;

    if (traveled_us < SERVO_EASE_US) {
        ease = SERVO_STEP_US_EASE +
               traveled_us * (SERVO_STEP_US_CRUISE - SERVO_STEP_US_EASE) / SERVO_EASE_US;
        if (ease < step) {
            step = ease;
        }
    }
    if (remain_us < SERVO_EASE_US) {
        ease = SERVO_STEP_US_EASE +
               remain_us * (SERVO_STEP_US_CRUISE - SERVO_STEP_US_EASE) / SERVO_EASE_US;
        if (ease < step) {
            step = ease;
        }
    }
    if (step < 1) {
        step = 1;
    }
    if (step > remain_us) {
        step = remain_us;
    }
    return step;
}

static void servo_gradient_update(SERVO_CTRL *dev)
{
    u32 obj_us = servo_angle_to_high_time(dev->obj_angle);
    u32 cur_us = dev->platform_data->high_time;
    u32 start_us = servo_angle_to_high_time(dev->str_angle);
    u32 remain_us;
    u32 traveled_us;
    u32 step;

    if (cur_us == obj_us) {
        dev->cur_angle = dev->obj_angle;
        return;
    }

    if (cur_us < obj_us) {
        remain_us = obj_us - cur_us;
        traveled_us = (cur_us > start_us) ? (cur_us - start_us) : 0;
        step = servo_ease_step(traveled_us, remain_us);
        cur_us += step;
    } else {
        remain_us = cur_us - obj_us;
        traveled_us = (start_us > cur_us) ? (start_us - cur_us) : 0;
        step = servo_ease_step(traveled_us, remain_us);
        cur_us -= step;
    }

    dev->platform_data->high_time = cur_us;
    if (cur_us <= 500) {
        dev->cur_angle = 0;
    } else {
        dev->cur_angle = (cur_us - 500) / 9;
    }
}

static u16 servo_us_to_cmp(u32 us)
{
    u32 cmp;

    if (servo_pwm_clk == 0) {
        return 0;
    }
    /* CMP = us * pwm_clk / 1e6, 80M/64 时 1tick=0.8us */
    cmp = (us * (servo_pwm_clk / 1000)) / 1000;
    if (cmp > servo_pwm_pr) {
        cmp = servo_pwm_pr;
    }
    return (u16)cmp;
}

static void servo_hw_pwm_set_us(u32 us)
{
    if (!servo_pwm_ready) {
        return;
    }
    if (us < SERVO_PULSE_MIN_US) {
        us = SERVO_PULSE_MIN_US;
    } else if (us > SERVO_PULSE_MAX_US) {
        us = SERVO_PULSE_MAX_US;
    }
    /* CMP 带缓冲, CNT==PR 时载入, 连续改脉宽不会出毛刺 */
    JL_PWM->CH3_CMP = servo_us_to_cmp(us);
}

static void servo_hw_pwm_init(u32 us)
{
    u32 lsb = (u32)clk_get("lsb");

    if (lsb == 0) {
        lsb = 80000000;
    }
    servo_pwm_clk = lsb >> SERVO_PWM_CKPS;
    if (servo_pwm_clk < SERVO_PWM_FREQ) {
        log_error("servo mcpwm clk err lsb:%d", lsb);
        return;
    }
    servo_pwm_pr = (u16)(servo_pwm_clk / SERVO_PWM_FREQ - 1);

    gpio_set_pull_up(IO_PWM, 0);
    gpio_set_pull_down(IO_PWM, 0);
    gpio_set_die(IO_PWM, 0);
    gpio_set_direction(IO_PWM, 0);

    /* IOMC0 是 24bit: PWM_CH3_IOEN[19] 占用 PA12 */
    *(volatile u32 *)(ls_io_base + 0x20 * 4) |= BIT(16 + SERVO_MCPWM_CH);

    JL_PWM->TMR3_CON = 0;
    JL_PWM->TMR3_CNT = 0;
    JL_PWM->TMR3_PR = servo_pwm_pr;
    JL_PWM->CH3_CMP = servo_us_to_cmp(us);
    /* [2:0]=CKPS /64, [7:6]=0 计数模式 */
    JL_PWM->TMR3_CON = SERVO_PWM_CKPS;

    SFR(JL_PWM->PWMCON1, 13, 3, SERVO_MCPWM_CH); /* PWM3 时基 = TIMER3 */
    SFR(JL_PWM->PWMCON1, 12, 1, 0);

    JL_PWM->PWMCON0 |= BIT(8 + SERVO_MCPWM_CH); /* T3EN */
    JL_PWM->PWMCON0 |= BIT(SERVO_MCPWM_CH);     /* PWM3EN */
    servo_pwm_ready = 1;
    log_info("servo mcpwm lsb:%d clk:%d pr:%d cmp:%d us:%d\n",
             lsb, servo_pwm_clk, servo_pwm_pr, JL_PWM->CH3_CMP, us);
}

void servo_io_init(void)
{
    gpio_set_direction(IO_VMA_PWR, 0);
    gpio_write(IO_VMA_PWR, 1);
    servo_hw_pwm_init(1500);
}

void soft_pwm_set(SERVO_CTRL *dev)
{
    if (dev == NULL) {
        return;
    }
    /* 硬件 PWM 持续出波, 这里只做角度渐变并更新比较值 */
    servo_gradient_update(dev);
    servo_hw_pwm_set_us(dev->platform_data->high_time);
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
    servo_hw_pwm_set_us(dev_ctrl->platform_data->high_time);
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
