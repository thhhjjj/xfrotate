#ifndef INFRARED_CONTROL_H
#define INFRARED_CONTROL_H
#include "typedef.h"
#include "gpio.h"
#include "app_config.h"

#ifndef INFRARED_EN
#define INFRARED_EN     0
#endif

typedef struct _infrared_platform_data {
    u32 *REG;
    u32 *ADC;
}INFRARED_PLATFORM_DATA;

typedef struct{
    INFRARED_PLATFORM_DATA *platform_data;
    u8 human_flag:1;//0:no human,1:yes human
}INFRARED_CTRL;

#define INFRARED_SET_FUNC_MODE 0x01
//extern about

#if INFRARED_EN

#define LOW 0
#define HIGH 1
#define IO_PIR_PWR   IO_PORTB_04
#define IO_SERIN     IO_PORTB_08
#define IO_DOCI      IO_PORTA_03

#define IO_INIT(IO)do{              \
    gpio_set_direction(IO, 1);      \
    gpio_set_pull_up(IO, 0);        \
    gpio_set_pull_down(IO, 0);      \
    gpio_set_die(IO, 1);            \
}while(0)

#define SERIN_OUT(STATE)do{            \
    gpio_set_direction(IO_SERIN, 0); \
    gpio_write(IO_SERIN, STATE);     \
}while(0)

#define DOCI_OUT(STATE)do{            \
    gpio_set_direction(IO_DOCI, 0); \
    gpio_write(IO_DOCI, STATE);     \
}while(0)

#define DOCI_IN()do{                \
    gpio_set_direction(IO_DOCI, 1); \
    gpio_set_pull_up(IO_DOCI, 0);   \
    gpio_set_pull_down(IO_DOCI, 0); \
    gpio_set_die(IO_DOCI, 1);       \
}while(0)

extern void udelay(u32 us);
void local_irq_enable();
void local_irq_disable();

extern void *infrared_open(void *arg);
extern void *infrared_release(void *dev);
extern void *infrared_read(void *dev);
extern void *infrared_write(void *dev, void *data, u32 len);
extern void *infrared_ioctl(void *dev, u32 cmd, u32 arg);

#endif /* INFRARED_EN */
#endif // INFRARED_CONTROL_H
