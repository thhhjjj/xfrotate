#ifndef _KEY_CONTRL_H
#define _KEY_CONTRL_H
#include "typedef.h"
enum//key_func_state
{
    HIGH_PRESS = 0,
    LOW_PRESS = 1,
};

#define IOKEY_DEFAULT_NUM 1
#define ADKEY_DEFAULT_NUM 0

typedef struct _iokey_platform_data {
    u16 key_func_state;
    u32 key_pins;
}IOKEY_PLATFORM_DATA;

typedef struct _adkey_platform_data {
    u16 adc_low_threshold;
    u16 adc_high_threshold;
    u32 key_pins;
    u32 adc_channel;
}ADKEY_PLATFORM_DATA;

typedef struct{
    IOKEY_PLATFORM_DATA *ioplatform_data_table;
    ADKEY_PLATFORM_DATA *adplatform_data_table;
    u16 key_num;
}KEY_CTRL;

extern void key_io_init(void);
extern void *key_open(void *arg);
extern void *key_release(void *dev);
extern void *key_read(void *dev);
extern void *key_write(void *dev, void *data, u32 len);
extern void *key_ioctl(void *dev, u32 cmd, u32 arg);
#endif