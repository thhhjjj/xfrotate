#ifndef _SERVO_CONTRL
#define _SERVO_CONTRL
#include "typedef.h"

typedef struct _servo_platform_data {
    u8  half_zone_flag;//0:mid,1:left,2:right
    u8  mov_dir;
    u16 open_time;
    u16 l_mov_speed;
    u16 r_mov_speed;
    u32 lservo_pins;
    u32 rservo_pins;
    u32 angle0_switch_pins;
    u32 angle90_switch_pins;
    u32 angle180_switch_pins;
}SERVO_PLATFORM_DATA;

typedef struct{
    SERVO_PLATFORM_DATA *platform_data;
    int  left_dyn_speed_offset;
    int  right_dyn_speed_offset;
    u16  str_angle;//start angle of servo motor
    u16  obj_angle;//target angle of servo motor
    u16  cur_angle;//current angle of servo motor
    u8   init_flag;//init_flag
}SERVO_CTRL;

enum
{
    MID=0,
    LEFT_MAX,
    RIGHT_MAX,
    NONE,       //无任何限位开关按下
};
enum
{
    STOP=0,
    LEFT,
    RIGHT,
};
//CMD
enum
{
    SERVO_CMD_SET_ANGLE,
};
enum{//init stage
    INIT_START,
    FINDING_0,
    FINDED_0,//START
    FINDING_180,
    FINDED_180,//COUNT RIGHT
    FINDING_0AGIN,
    FINDED_0AGIN,//COUNT LEFT
    BACKING_90,
    BACKED_90,//RESET
    INIT_OVER,
};
extern void udelay(u32 us);
void local_irq_enable();
void local_irq_disable();

extern void *servo_open(void *arg);
extern void *servo_release(void *dev);
extern void *servo_read(void *dev , void *buf, u32 len);
extern void *servo_write(void *dev, void *data, u32 len);
extern void *servo_ioctl(void *dev, u32 cmd, u32 arg);
extern void *servo_save(void *dev); 

extern void soft_pwm_set(SERVO_CTRL *dev);
extern void mcpwm_set(u8 channel, u32 fre, u32 duty);
extern void pwm_frq_duty(u8 channel, u32 fre, u32 duty);
#endif