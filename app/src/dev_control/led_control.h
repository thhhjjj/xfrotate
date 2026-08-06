#ifndef _LED_CONTRL_H
#define _LED_CONTRL_H
#include "typedef.h"

enum//led_func_state
{
    OUT_LOW = 0,
    OUT_HIGH = 1,
};

enum//led_func
{
    LED_FUNC_OFF = 0,
    LED_FUNC_ON = 1,
};

enum//CMD
{
    LED_CMD_ALL_FUNC_OFF = 0,
    LED_CMD_ALL_FUNC_ON = 1,
    LED_CMD_IO_FUNC_OFF = 2,
    LED_CMD_IO_FUNC_ON = 3,
};
#define LED_DEFAULT_NUM 2
typedef struct _led_platform_data {
    u16 led_func;
    u16 led_func_state;
    u32 led_pins;
}LED_PLATFORM_DATA;

typedef struct{
    LED_PLATFORM_DATA *platform_data_table;
    u16 led_num;
}LED_CTRL;
extern void led_io_init(void);
extern void *led_open(void *arg);
extern void *led_release(void *dev);
extern void *led_read(void *dev);
extern void *led_write(void *dev, void *data, u32 len);
extern void *led_ioctl(void *dev, u32 cmd, u32 arg);
#endif