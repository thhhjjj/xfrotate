#include "servo_control.h"
#include "mcpwm.h"
#include "gpio.h"
#include "test_mode.h"
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

#define DEFUALT_SERVO_MOVE_SPEED 10

#define IO_SERVO_OUT(IO_SERVO,IO_LEVEL)do{     \
    gpio_write(IO_SERVO, IO_LEVEL);            \
}while(0)

#define IO_ANGLE_INIT(IO)do{            \
    gpio_set_direction(IO, 1);          \
    gpio_set_pull_up(IO, 1);            \
    gpio_set_pull_down(IO, 0);          \
    gpio_set_die(IO, 1);                \
}while(0)

// 结构体新增state_cnt解决static变量多实例冲突
SERVO_PLATFORM_DATA servo_platform_data = {
    .half_zone_flag = MID,
    .mov_dir = STOP,
    .open_time = 0,
    .l_mov_speed = DEFUALT_SERVO_MOVE_SPEED,
    .r_mov_speed = DEFUALT_SERVO_MOVE_SPEED,
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

void servo_switch_read(SERVO_CTRL *dev_ctrl) //24ms任务
{
    static u16 state_cnt = 0;

    if (!gpio_read(IO_ANGLE90)) {
        state_cnt = 1;
        dev_ctrl->platform_data->half_zone_flag = MID;
        if (g_test_mode) {
            servo_mid_switch = 1;
            //log_info("test servo mid\n");
        } else {
            //log_info("servo mid\n");
        }
    } else if (!gpio_read(IO_ANGLE0)) {
        state_cnt = 1;
        dev_ctrl->platform_data->half_zone_flag = LEFT_MAX;
        if (g_test_mode) {
            servo_left_switch = 1;
            //log_info("test servo left max\n");
        } else {
            //log_info("servo left max\n");
        }
    } else if (!gpio_read(IO_ANGLE180)) {
        state_cnt = 1;
        dev_ctrl->platform_data->half_zone_flag = RIGHT_MAX;
        if (g_test_mode) {
            servo_right_switch = 1;
            //log_info("test servo right max\n");
        } else {
            //log_info("servo right max\n");
        }
    } else{
        state_cnt = 0;
        dev_ctrl->platform_data->half_zone_flag = NONE;
    }

    if((g_test_mode) && (servo_left_switch == 1) && (servo_mid_switch == 1) && (servo_right_switch == 1)) {
        servo_test_flag = 3;
    }

    if ((state_cnt != 0) && (dev_ctrl->platform_data->mov_dir != STOP)) {
        state_cnt++;
        if (state_cnt >= 300) {
            dev_ctrl->platform_data->mov_dir = STOP;
        }
    }

    //protect servo move out of range
    if ((dev_ctrl->platform_data->open_time) && (dev_ctrl->platform_data->mov_dir == LEFT) && (!gpio_read(IO_ANGLE0))) {
        if (dev_ctrl->obj_angle == 0) {
            dev_ctrl->cur_angle = 0;
        }
        dev_ctrl->platform_data->mov_dir = STOP;
        dev_ctrl->platform_data->open_time = 0;
    } else if ((dev_ctrl->platform_data->open_time) && (dev_ctrl->platform_data->mov_dir == RIGHT) && (!gpio_read(IO_ANGLE180))) {
        if (dev_ctrl->obj_angle == 180) {
            dev_ctrl->cur_angle = 180;
        }
        dev_ctrl->platform_data->mov_dir = STOP;
        dev_ctrl->platform_data->open_time = 0;
    }
}

void move_set(SERVO_CTRL *dev_ctrl, u16 angle)
{
    angle = angle > 180 ? 180 : angle;

    dev_ctrl->str_angle = dev_ctrl->cur_angle;
    dev_ctrl->obj_angle = angle;

    if (dev_ctrl->str_angle < dev_ctrl->obj_angle) {
        dev_ctrl->platform_data->mov_dir = RIGHT;
        dev_ctrl->platform_data->open_time = (dev_ctrl->obj_angle - dev_ctrl->str_angle) * dev_ctrl->platform_data->r_mov_speed;
    } else if (dev_ctrl->str_angle > dev_ctrl->obj_angle) {
        dev_ctrl->platform_data->mov_dir = LEFT;
        dev_ctrl->platform_data->open_time = (dev_ctrl->str_angle - dev_ctrl->obj_angle) * dev_ctrl->platform_data->l_mov_speed;
    } else {
        dev_ctrl->platform_data->mov_dir = STOP;
        dev_ctrl->platform_data->open_time = 0;
    }
    log_info("move_set angle:%d,open_time:%d,mov_dir:%d,str_angle:%d,obj_angle:%d\n",angle,dev_ctrl->platform_data->open_time,dev_ctrl->platform_data->mov_dir,dev_ctrl->str_angle,dev_ctrl->obj_angle);
}
u8 log_flag = 0;
void move_func(SERVO_CTRL *dev_ctrl) //24ms执行
{
    // SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    // SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;
    //log_info("move_func init_state:%d, mov_dir:%d,open_time:%d,cur_angle:%d,obj_angle:%d\n",
    //        dev_ctrl->init_flag,dev_ctrl->platform_data->mov_dir,dev_ctrl->platform_data->open_time,dev_ctrl->cur_angle,dev_ctrl->obj_angle);
    if (dev_ctrl->platform_data->mov_dir == STOP) {
        if(log_flag){
            log_flag = 0;
                log_info("servo stop,cur_angle:%d\n",dev_ctrl->cur_angle);
        }
        IO_SERVO_OUT(IO_LSERVO, 0);
        IO_SERVO_OUT(IO_RSERVO, 0);
        return;
    }
    int step_tick = 0;
    if(dev_ctrl->platform_data->mov_dir == RIGHT){
        step_tick = dev_ctrl->platform_data->r_mov_speed + dev_ctrl->right_dyn_speed_offset;
    }else{
        step_tick = dev_ctrl->platform_data->l_mov_speed + dev_ctrl->left_dyn_speed_offset;
    }
    if (step_tick < 1){
        step_tick = 1;
    }
    if (dev_ctrl->platform_data->mov_dir == RIGHT) {
        if (dev_ctrl->platform_data->open_time > 0) {
            log_flag = 1;
            dev_ctrl->platform_data->open_time--;
        }
        if ((dev_ctrl->platform_data->open_time % step_tick) == 0) {
            dev_ctrl->cur_angle++;
            dev_ctrl->cur_angle = dev_ctrl->cur_angle > 180 ? 180 : dev_ctrl->cur_angle;

            IO_SERVO_OUT(IO_LSERVO, 0);
            IO_SERVO_OUT(IO_RSERVO, 1);

            if (dev_ctrl->cur_angle == dev_ctrl->obj_angle) {
                if(log_flag){
                    log_flag = 0;
                    log_info("Right move done:cur_angle:%d\n",dev_ctrl->cur_angle);
                }
                dev_ctrl->platform_data->mov_dir = STOP;
                IO_SERVO_OUT(IO_LSERVO, 0);
                IO_SERVO_OUT(IO_RSERVO, 0);
            }
        }
    } else if (dev_ctrl->platform_data->mov_dir == LEFT) {
        if (dev_ctrl->platform_data->open_time > 0) {
            log_flag = 1;
            dev_ctrl->platform_data->open_time--;
        }
        if ((dev_ctrl->platform_data->open_time % step_tick) == 0) {
            dev_ctrl->cur_angle--;
            dev_ctrl->cur_angle = dev_ctrl->cur_angle > 180 ? 0 : dev_ctrl->cur_angle; //u16无符号回绕保护(0->65535)

            IO_SERVO_OUT(IO_RSERVO, 0);
            IO_SERVO_OUT(IO_LSERVO, 1);

            if (dev_ctrl->cur_angle == dev_ctrl->obj_angle) {
                if(log_flag){
                    log_flag = 0;
                    log_info("Left move done:cur_angle:%d\n",dev_ctrl->cur_angle);
                }
                dev_ctrl->platform_data->mov_dir = STOP;
                IO_SERVO_OUT(IO_LSERVO, 0);
                IO_SERVO_OUT(IO_RSERVO, 0);
            }
        }
    }
}

void multi_dyn_check_offset(SERVO_CTRL *dev_ctrl) //24ms
{
    // SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    // SERVO_PLATFORM_DATA *pdata = dev_ctrl->platform_data;
    if (dev_ctrl->platform_data->half_zone_flag == MID) {
        if (dev_ctrl->platform_data->mov_dir == LEFT) {
            if (dev_ctrl->cur_angle > 90) {
                if (dev_ctrl->cur_angle - 90 > 30) {
                    dev_ctrl->left_dyn_speed_offset = -1;
                }
            } else {
                if (90 - dev_ctrl->cur_angle > 30) {
                    dev_ctrl->left_dyn_speed_offset = 1;
                }
            }
        } else if (dev_ctrl->platform_data->mov_dir == RIGHT) {
            if (dev_ctrl->cur_angle < 90) {
                if (90 - dev_ctrl->cur_angle > 30) {
                    dev_ctrl->right_dyn_speed_offset = 1;
                }
            } else {
                if (dev_ctrl->cur_angle - 90 > 30) {
                    dev_ctrl->right_dyn_speed_offset = -1;
                }
            }
        }
    } else if (dev_ctrl->platform_data->half_zone_flag == LEFT_MAX) {
        if (dev_ctrl->obj_angle == 0 && dev_ctrl->cur_angle > 30) {
            dev_ctrl->left_dyn_speed_offset = -1;
        }
        dev_ctrl->cur_angle = 0;
        return;
    } else if (dev_ctrl->platform_data->half_zone_flag == RIGHT_MAX) {
        if (dev_ctrl->obj_angle == 180 && dev_ctrl->cur_angle < 150) {
            dev_ctrl->right_dyn_speed_offset = -1;
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
    dev_ctrl->cur_angle = 90;
    dev_ctrl->obj_angle = 90;
    dev_ctrl->str_angle = 90;
    dev_ctrl->left_dyn_speed_offset = 0;
    dev_ctrl->right_dyn_speed_offset = 0;
    dev_ctrl->platform_data = &servo_platform_data;
    return dev_ctrl;
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
u16 left_find_time = 0;
u16 right_find_time = 0;
void stc_offset_set(SERVO_CTRL *dev_ctrl)//4ms
{
    switch(dev_ctrl->init_flag){
        case INIT_START:
            move_set(dev_ctrl, 0);//left find first
            dev_ctrl->init_flag = FINDING_0;
            break;
        case FINDING_0:
            if(dev_ctrl->platform_data->half_zone_flag == LEFT_MAX){
                dev_ctrl->platform_data->mov_dir = STOP;
                dev_ctrl->platform_data->open_time = 0;
                dev_ctrl->cur_angle = 0;
                dev_ctrl->init_flag = FINDED_0;
            }else if(dev_ctrl->platform_data->mov_dir== STOP){
                dev_ctrl->cur_angle = 90;
                move_set(dev_ctrl, 0);//left find
            }
        break;
        case FINDED_0:
            move_set(dev_ctrl, 180);//count find right
            dev_ctrl->init_flag = FINDING_180;
        break;
        case FINDING_180:
            if(dev_ctrl->platform_data->half_zone_flag == RIGHT_MAX){
                dev_ctrl->platform_data->mov_dir = STOP;
                dev_ctrl->platform_data->open_time = 0;
                dev_ctrl->cur_angle = 180;
                dev_ctrl->init_flag = FINDED_180;
            }else if(dev_ctrl->platform_data->mov_dir== STOP){
                dev_ctrl->cur_angle = 90;
                move_set(dev_ctrl, 180);//right find
            }
            right_find_time++;
        break;
        case FINDED_180:
            move_set(dev_ctrl, 0);//count find left
            dev_ctrl->init_flag = FINDING_0AGIN;
        break;
        case FINDING_0AGIN:
            if(dev_ctrl->platform_data->half_zone_flag == LEFT_MAX){
                dev_ctrl->platform_data->mov_dir = STOP;
                dev_ctrl->platform_data->open_time = 0;
                dev_ctrl->cur_angle = 0;
                dev_ctrl->init_flag = FINDED_0AGIN;
            }else if(dev_ctrl->platform_data->mov_dir== STOP){
                dev_ctrl->cur_angle = 90;
                move_set(dev_ctrl, 0);//left find
            }
            left_find_time++;
        break;
        case FINDED_0AGIN:
            move_set(dev_ctrl, 90);//reset
            dev_ctrl->init_flag = BACKING_90;
        break;
        case BACKING_90:
            if(dev_ctrl->platform_data->half_zone_flag == MID){
                dev_ctrl->platform_data->mov_dir = STOP;
                dev_ctrl->platform_data->open_time = 0;
                dev_ctrl->cur_angle = 90;
                dev_ctrl->init_flag = BACKED_90;
            }else if(dev_ctrl->platform_data->mov_dir== STOP){
                dev_ctrl->cur_angle = 0;
                move_set(dev_ctrl, 90);//reset
            }
        break;
        case BACKED_90:
            dev_ctrl->platform_data->r_mov_speed = right_find_time/180;
            dev_ctrl->platform_data->l_mov_speed = left_find_time/180;
            if((dev_ctrl->platform_data->r_mov_speed == 0) && (dev_ctrl->platform_data->l_mov_speed == 0)){
                dev_ctrl->platform_data->r_mov_speed = 1;
                dev_ctrl->platform_data->l_mov_speed = 1;
            }
            log_info("init over,r_mov_speed: %d, l_mov_speed: %d", dev_ctrl->platform_data->r_mov_speed, dev_ctrl->platform_data->l_mov_speed);
            dev_ctrl->init_flag = INIT_OVER;
        break;
    }
}

void *servo_write(void *dev, void *data, u32 len)//24ms
{
    if (!dev) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    if(dev_ctrl->init_flag>=INIT_OVER){
        servo_switch_read(dev_ctrl);
        //multi_dyn_check_offset(dev_ctrl);
        move_func(dev_ctrl);
    }else{
        servo_switch_read(dev_ctrl);
        stc_offset_set(dev_ctrl);
        move_func(dev_ctrl);
    }
    return NULL;
}

// ioctl
void *servo_ioctl(void *dev, u32 cmd, u32 arg)
{
    if (!dev) {
        return NULL;
    }
    SERVO_CTRL *dev_ctrl = (SERVO_CTRL *)dev;
    switch(cmd){
        case SERVO_CMD_SET_ANGLE:
            move_set(dev_ctrl, arg);
            return NULL;
        default:
            return NULL;
    }
}
