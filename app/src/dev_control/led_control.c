#include "led_control.h"
#include "gpio.h"

#include "my_malloc.h"
#include "malloc.h"
#define IO_RED      IO_PORTA_05
#define IO_WHITE    IO_PORTA_06
LED_PLATFORM_DATA led_platform_data_table[LED_DEFAULT_NUM] = {
    {
        .led_func = LED_FUNC_OFF,
        .led_func_state = OUT_HIGH,
        .led_pins = IO_RED,//RED
    },
    {
        .led_func = LED_FUNC_OFF,
        .led_func_state = OUT_HIGH,
        .led_pins = IO_WHITE,//WHITE
    },  
};
void led_io_init(void)
{
    gpio_set_direction(IO_RED, 0);
    gpio_set_die(IO_RED, 1);
    gpio_write(IO_RED, 0);
    gpio_set_direction(IO_WHITE, 0);
    gpio_set_die(IO_WHITE, 1);
    gpio_write(IO_WHITE, 0);
}   
void *led_open(void *dev)
{
    LED_CTRL *dev_ctrl = malloc(sizeof(LED_CTRL));
    if(dev_ctrl == NULL){
        return NULL;
    }
    memset(dev_ctrl, 0, sizeof(LED_CTRL));
    dev_ctrl->led_num = LED_DEFAULT_NUM;
    dev_ctrl->platform_data_table = led_platform_data_table;
    dev_ctrl = dev_ctrl;
    return dev_ctrl;
}

void *led_release(void *dev)
{
    if(dev == NULL){
        return NULL;
    }
    LED_CTRL *dev_ctrl = (LED_CTRL *)dev;
    free(dev_ctrl);
    return NULL;
}

void *led_read(void *dev)
{
    return NULL;
}

void *led_write(void *dev, void *data, u32 len)
{
    return NULL;
}

void *led_ioctl(void *dev, u32 cmd, u32 arg)
{
    if(dev == NULL){
        return NULL;
    }
    LED_CTRL *dev_ctrl = (LED_CTRL *)dev;
    switch (cmd) {
        case LED_CMD_ALL_FUNC_OFF:
            gpio_write(IO_WHITE, 0);
            gpio_write(IO_RED, 0);
            dev_ctrl->platform_data_table[0].led_func = LED_FUNC_OFF;
            dev_ctrl->platform_data_table[1].led_func = LED_FUNC_OFF;
            break;
        case LED_CMD_ALL_FUNC_ON:
            gpio_write(IO_WHITE, 1);
            gpio_write(IO_RED, 1);
            dev_ctrl->platform_data_table[0].led_func = LED_FUNC_ON;
            dev_ctrl->platform_data_table[1].led_func = LED_FUNC_ON;
            break;
        case LED_CMD_IO_FUNC_OFF:
            switch (arg) {
                case 0:
                    gpio_write(IO_RED, 0);
                    dev_ctrl->platform_data_table[0].led_func = LED_FUNC_OFF;
                    break;
                case 1:
                    gpio_write(IO_WHITE, 0);
                    dev_ctrl->platform_data_table[1].led_func = LED_FUNC_OFF;
                    break;
            }
            break;
        case LED_CMD_IO_FUNC_ON:    
            switch (arg) {
                case 0:
                    gpio_write(IO_RED, 1);
                    dev_ctrl->platform_data_table[0].led_func = LED_FUNC_ON;
                    break;
                case 1:
                    gpio_write(IO_WHITE, 1);
                    dev_ctrl->platform_data_table[1].led_func = LED_FUNC_ON;
                    break;
            }
            break;
        default:
            break;
    }
    return NULL;
}