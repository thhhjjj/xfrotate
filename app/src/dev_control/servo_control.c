#include "servo_control.h"
#include "mcpwm.h"
#include "gpio.h"

#include "my_malloc.h"
#include "malloc.h"
#include "log.h"
#include <string.h>
#include "vm_api.h"
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[servo_control]"

#define IO_LSERVO    IO_PORTA_11
#define IO_RSERVO    IO_PORTA_12
#define IO_ANGLE0    IO_PORTA_02
#define IO_ANGLE90   IO_PORTA_01
#define IO_ANGLE180  IO_PORTB_05

#define SERVO_MOVE_SPEED 4

#define IO_SERVO_OUT(IO_SERVO,IO_LEVEL)do{     \
    gpio_write(IO_SERVO, IO_LEVEL);    \
}while(0)

#define IO_ANGLE_INIT(IO)do{      \
    gpio_set_direction(IO, 1);      \
    gpio_set_pull_up(IO, 1);          \
    gpio_set_pull_down(IO, 0);        \
    gpio_set_die(IO, 1);              \
}while(0)

// 结构体新增state_cnt解决static变量多实例冲突
SERVO_PLATFORM_DATA servo_platform_data = {
    .half_zone_flag = MID,
    .mov_dir = STOP,
    .open_time = 0,
    .lservo_pins = IO_LSERVO,
    .rservo_pins = IO_RSERVO,
    .angle0_switch_pins = IO_ANGLE0,
    .angle90_switch_pins = IO_ANGLE90,
    .angle180_switch_pins = IO_ANGLE180,
};

void servo_io_init(void)
{
    gpio_set_direction(IO_LSERVO, 0);
    gpio_write(IO_LSERVO, 0);
    gpio_set_direction(IO_RSERVO, 0);
    gpio_write(IO_RSERVO, 0);

    IO_ANGLE_INIT(IO_ANGLE0);
    IO_ANGLE_INIT(IO_ANGLE90);
    IO_ANGLE_INIT(IO_ANGLE180);
}

void servo_switch_read(void *dev) //4ms任务
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;
    static u16 state_cnt = 0;
    log_info("servo_switch_state:%d,%d,%d\n",gpio_read(IO_ANGLE0),gpio_read(IO_ANGLE90),gpio_read(IO_ANGLE180));
    if ((!gpio_read(IO_ANGLE0)) && (pdata->half_zone_flag == LEFT_ZONE)) {
        pdata->half_zone_flag = LEFT_MAX;
        state_cnt = 1;
    } else {
        state_cnt = 0;
    }

    if (!gpio_read(IO_ANGLE90)) {
        state_cnt = 1;
        pdata->half_zone_flag = MID;
    } else {
        state_cnt = 0;
        if (pdata->mov_dir == LEFT) {
            pdata->half_zone_flag = LEFT_ZONE;
        } else if (pdata->mov_dir == RIGHT) {
            pdata->half_zone_flag = RIGHT_ZONE;
        }
    }

    if ((!gpio_read(IO_ANGLE180)) && (pdata->half_zone_flag == RIGHT_ZONE)) {
        state_cnt = 1;
        pdata->half_zone_flag = RIGHT_MAX;
    } else {
        state_cnt = 0;
    }

    if ((state_cnt != 0) && (pdata->mov_dir != STOP)) {
        state_cnt++;
        if (state_cnt >= 300) {
            pdata->mov_dir = STOP;
        }
    }
}

void move_set(void *dev, u16 angle)
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;
    log_info("move_set angle:%d\n",angle);
    angle = angle > 180 ? 180 : angle;

    dev_ctrl->str_angle = dev_ctrl->cur_angle;
    dev_ctrl->obj_angle = angle;

    if (dev_ctrl->str_angle < dev_ctrl->obj_angle) {
        pdata->mov_dir = RIGHT;
        pdata->open_time = (dev_ctrl->obj_angle - dev_ctrl->str_angle) * SERVO_MOVE_SPEED;
    } else if (dev_ctrl->str_angle > dev_ctrl->obj_angle) {
        pdata->mov_dir = LEFT;
        pdata->open_time = (dev_ctrl->str_angle - dev_ctrl->obj_angle) * SERVO_MOVE_SPEED;
    } else {
        pdata->mov_dir = STOP;
        pdata->open_time = 0;
    }
}

void move_func(void *dev) //4ms执行
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;
    log_info("move_func mov_dir:%d,open_time:%d\n",pdata->mov_dir,pdata->open_time);
    if (pdata->mov_dir == STOP) {
        IO_SERVO_OUT(IO_LSERVO, 0);
        IO_SERVO_OUT(IO_RSERVO, 0);
        return;
    }

    int step_tick = SERVO_MOVE_SPEED + dev_ctrl->dyn_speed_offset + dev_ctrl->fixed_speed_offset;
    if (step_tick < 1) {
        step_tick = 1;
    }

    if (pdata->mov_dir == RIGHT) {
        if (pdata->open_time > 0) {
            pdata->open_time--;
        }
        if ((pdata->open_time % step_tick) == 0) {
            dev_ctrl->cur_angle++;
            dev_ctrl->cur_angle = dev_ctrl->cur_angle > 180 ? 180 : dev_ctrl->cur_angle;

            IO_SERVO_OUT(IO_LSERVO, 0);
            IO_SERVO_OUT(IO_RSERVO, 1);

            if (dev_ctrl->cur_angle == dev_ctrl->obj_angle) {
                pdata->mov_dir = STOP;
                dev_ctrl->dyn_speed_offset = 0;
                dev_ctrl->fixed_speed_offset = 0;
                IO_SERVO_OUT(IO_LSERVO, 0);
                IO_SERVO_OUT(IO_RSERVO, 0);
            }
        }
    } else if (pdata->mov_dir == LEFT) {
        if (pdata->open_time > 0) {
            pdata->open_time--;
        }
        if ((pdata->open_time % step_tick) == 0) {
            dev_ctrl->cur_angle--;
            dev_ctrl->cur_angle = dev_ctrl->cur_angle < 0 ? 0 : dev_ctrl->cur_angle;

            IO_SERVO_OUT(IO_RSERVO, 0);
            IO_SERVO_OUT(IO_LSERVO, 1);

            if (dev_ctrl->cur_angle == dev_ctrl->obj_angle) {
                pdata->mov_dir = STOP;
                dev_ctrl->dyn_speed_offset = 0;
                dev_ctrl->fixed_speed_offset = 0;
                IO_SERVO_OUT(IO_LSERVO, 0);
                IO_SERVO_OUT(IO_RSERVO, 0);
            }
        }
    }
}

void multi_check_setoff(void *dev) //4ms
{
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;

    dev_ctrl->dyn_speed_offset = 0;
    dev_ctrl->fixed_speed_offset = 0;

    if (pdata->half_zone_flag == MID) {
        if (pdata->mov_dir == LEFT) {
            if (dev_ctrl->cur_angle > 90) {
                if (dev_ctrl->cur_angle - 90 > 20) {
                    dev_ctrl->dyn_speed_offset = -1;
                }
            } else {
                if (90 - dev_ctrl->cur_angle > 20) {
                    dev_ctrl->dyn_speed_offset = 1;
                }
            }
        } else if (pdata->mov_dir == RIGHT) {
            if (dev_ctrl->cur_angle < 90) {
                if (90 - dev_ctrl->cur_angle > 20) {
                    dev_ctrl->dyn_speed_offset = 1;
                }
            } else {
                if (dev_ctrl->cur_angle - 90 > 20) {
                    dev_ctrl->dyn_speed_offset = -1;
                }
            }
        }
        if (pdata->mov_dir == STOP) {
            dev_ctrl->cur_angle = 90;
        }
        return;
    } else if (pdata->half_zone_flag == LEFT_MAX) {
        if (dev_ctrl->obj_angle == 0 && dev_ctrl->cur_angle > 20) {
            dev_ctrl->fixed_speed_offset = -1;
        }
        dev_ctrl->cur_angle = 0;
        return;
    } else if (pdata->half_zone_flag == RIGHT_MAX) {
        if (dev_ctrl->obj_angle == 180 && dev_ctrl->cur_angle < 160) {
            dev_ctrl->fixed_speed_offset = -1;
        }
        dev_ctrl->cur_angle = 180;
        return;
    }
}

void *servo_open(void *dev)
{
    SERVO_CTRL *dev_ctrl = malloc(sizeof(SERVO_CTRL));
    if (dev_ctrl == NULL) {
        return NULL;
    }
    memset(dev_ctrl, 0, sizeof(SERVO_CTRL));
    SERVO_SAVE_DATA temp_save_data={0};
    vm_read(SERVO_SAVE, (u8 *)&temp_save_data, sizeof(SERVO_SAVE_DATA));
    dev_ctrl->cur_angle = temp_save_data.cur_angle;
    dev_ctrl->obj_angle = temp_save_data.cur_angle;
    dev_ctrl->str_angle = temp_save_data.cur_angle;
    dev_ctrl->dyn_speed_offset = temp_save_data.dyn_speed_offset;
    dev_ctrl->fixed_speed_offset = temp_save_data.fixed_speed_offset;
    dev_ctrl->platform_data = &servo_platform_data;
    return dev_ctrl;
}

void *servo_save(void *dev)
{
    if (dev == NULL) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    SERVO_SAVE_DATA temp_save_data={0};
    temp_save_data.cur_angle = dev_ctrl->cur_angle;
    temp_save_data.dyn_speed_offset = dev_ctrl->dyn_speed_offset;
    temp_save_data.fixed_speed_offset = dev_ctrl->fixed_speed_offset;
    vm_write(SERVO_SAVE, (u8*)&temp_save_data, sizeof(SERVO_SAVE_DATA));
    return NULL;
}

void *servo_release(void *dev)
{
    if (dev == NULL) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    IO_SERVO_OUT(IO_LSERVO, 0);
    IO_SERVO_OUT(IO_RSERVO, 0);
    free(dev_ctrl);
    return NULL;
}

void *servo_read(void *dev, void *buf, u32 len)
{
    if (!dev || !buf || len < sizeof(u16)) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    *(u16 *)buf = dev_ctrl->cur_angle;
    return buf;
}


void *servo_write(void *dev, void *data, u32 len)
{
    if (!dev) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    servo_switch_read(dev_ctrl);
    multi_check_setoff(dev_ctrl);
    move_func(dev_ctrl);
    return NULL;
}

// ioctl
void *servo_ioctl(void *dev, u32 cmd, u32 arg)
{
    if (!dev) {
        return NULL;
    }
    switch (cmd) {
        case SERVO_CMD_SET_ANGLE:
            move_set(dev, arg);
            return NULL;
        default:
            return NULL;
    }
}
