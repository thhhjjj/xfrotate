#ifndef _SERVO_CONTRL
#define _SERVO_CONTRL
#include "typedef.h"

typedef struct _servo_platform_data {
    u32 high_time;
    u32 pins;
}SERVO_PLATFORM_DATA;

typedef struct{
    SERVO_PLATFORM_DATA *platform_data;
    u32 str_angle;//起始角度
    u32 cur_angle;//当前角度
    u32 obj_angle;//目标角度
}SERVO_CTRL;
//CMD
enum
{
    SERVO_CMD_SET_ANGLE,
};
extern void *servo_open(void *arg);
extern void *servo_release(void *dev);
extern void *servo_read(void *dev);
extern void *servo_write(void *dev, void *data, u32 len);
extern void *servo_ioctl(void *dev, u32 cmd, u32 arg);

extern void servo_io_init(void);
extern void soft_pwm_set(SERVO_CTRL *dev);
#endif
