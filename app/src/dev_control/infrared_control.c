#include "adc_drv.h"
#include "gpio.h"
#include "typedef.h"
#include "infrared_control.h"
#include "log.h"

#include "my_malloc.h"
#include "malloc.h"
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[infrared_control]"
u32 REG = 0;
u32 ADC_REG = 0;
u16 read_delay = 0;
u16 debounce_cnt = 0;
INFRARED_PLATFORM_DATA infrared_platform_data = {0};
void infrared_io_init(void)
{
    gpio_set_direction(IO_PIR_PWR, 0);
    gpio_write(IO_PIR_PWR, HIGH);
}

void reg_send(u32 REG_IN)
{
    SERIN_OUT(LOW);
    udelay(100);
    u8 b = 24;
    for(u16 a = 25; a > 0; a--)
    {
        SERIN_OUT(HIGH);
        udelay(2);
        if ((REG_IN & (1U << b)) == 0){
            SERIN_OUT(LOW);
        }
        udelay(100);
        SERIN_OUT(LOW);
        udelay(2);
        b--;
    }
}

u8 reg_read(void)
{
    u8 triggered = 0;
    local_irq_disable();

    // 先输出高 — 给传感器 DOCI 脚一个电平跳变
    DOCI_OUT(HIGH);
    udelay(5);
    DOCI_IN();                          // 切输入+上拉
    udelay(3);                          // 等电平稳定

    if(gpio_read(IO_DOCI))              // 引脚为高 = 有人触发
    {
        DOCI_OUT(LOW);                  // 拉低复位传感器
        udelay(500);                    // 保持 500µs
        DOCI_IN();                      // 释放回输入
        triggered = 1;
    }

    local_irq_enable();
    return triggered;
}


void *infrared_open(void* arg)
{
    INFRARED_CTRL *dev_ctrl = malloc(sizeof(INFRARED_CTRL));
    if(dev_ctrl == NULL){
        return NULL;
    }
    memset(dev_ctrl, 0, sizeof(INFRARED_CTRL));
    dev_ctrl->platform_data = &infrared_platform_data;
    // initial pin
    SERIN_OUT(LOW);
    DOCI_OUT(LOW);
    //infrared_io_init();
    REG = 0x01A0110U;//0x640910U;
    local_irq_disable();
    // init register
    udelay(1000);
    reg_send(REG);//standard human detect mode
    udelay(1000);
    local_irq_enable();
    SERIN_OUT(HIGH);
    IO_INIT(IO_DOCI);
    read_delay = 100;
    dev_ctrl->platform_data->REG = &REG;
    dev_ctrl->platform_data->ADC = &ADC_REG;
    return dev_ctrl;
}

void *infrared_release(void *dev)
{
    INFRARED_CTRL *dev_ctrl = (INFRARED_CTRL *)dev;
    if(dev_ctrl == NULL){
        return NULL;
    }
    free(dev_ctrl);
    dev_ctrl = NULL;
    return NULL;
}

void *infrared_read(void *dev)
{
    if(read_delay){
        read_delay--;
        return NULL;
    }
    if(dev == NULL){
        return NULL;
    }
    INFRARED_CTRL *dev_ctrl = (INFRARED_CTRL *)dev;
    if(debounce_cnt){
        debounce_cnt--;
        if(!debounce_cnt){
            dev_ctrl->human_flag = 0;
        }
    }
    if(gpio_read(IO_DOCI)){
        debounce_cnt=50;
        dev_ctrl->human_flag = 1;
        DOCI_OUT(LOW);
        udelay(1000);
        gpio_set_direction(IO_DOCI, 1);
        gpio_set_pull_up(IO_DOCI, 0);
        gpio_set_pull_down(IO_DOCI, 1);
        ADC_REG = 1;
        read_delay = 20;
    }else{
        ADC_REG = 0;
    }

    dev_ctrl->platform_data->ADC = &ADC_REG;
    return dev_ctrl->platform_data->ADC;
}

void *infrared_write(void *dev, void *data, u32 len)
{
    if((dev == NULL) || (len != 4)){
        return NULL;
    }
    INFRARED_CTRL *dev_ctrl = (INFRARED_CTRL *)dev;
    u8 *buf = (u8 *)data;
    u32 cfg_reg = 0;
    cfg_reg |= (u32)buf[0] << 0;
    cfg_reg |= (u32)buf[1] << 8;
    cfg_reg |= (u32)buf[2] << 16;
    cfg_reg |= (u32)buf[3] << 24;
    reg_send(cfg_reg);
    REG = cfg_reg;
    return dev;
}

void *infrared_ioctl(void *dev, u32 cmd, u32 arg)
{
    if(dev == NULL){
        return NULL;
    }
    INFRARED_CTRL *dev_ctrl = (INFRARED_CTRL *)dev;
    switch(cmd){
        case INFRARED_SET_FUNC_MODE:
            break;
        default:
            break;
    }
    return dev_ctrl;
}

