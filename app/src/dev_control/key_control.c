#include "key_control.h"
#include "gpio.h"
#include "msg.h"
#include "log.h"

#include "my_malloc.h"
#include "malloc.h"

#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[key_control]"

IOKEY_PLATFORM_DATA iokey_platform_data_table[IOKEY_DEFAULT_NUM] = {
    {
        .key_func_state = LOW_PRESS,
        .key_pins = IO_PORTB_02,
    }, 
};

void key_io_init(void)
{
    gpio_set_direction(IO_PORTB_02, 1);
    gpio_set_pull_up(IO_PORTB_02, 1);
    gpio_set_pull_down(IO_PORTB_02, 0);
    gpio_set_die(IO_PORTB_02, 1);
}

void *key_open(void *dev)
{
    KEY_CTRL *dev_ctrl = malloc(sizeof(KEY_CTRL));
    if(dev_ctrl == NULL){
        return NULL;
    }
    dev_ctrl->key_num = IOKEY_DEFAULT_NUM;
    dev_ctrl->ioplatform_data_table = iokey_platform_data_table;
    return dev_ctrl;
}

void *key_release(void *dev)
{
    KEY_CTRL *dev_ctrl = (KEY_CTRL *)dev;
    if(dev_ctrl == NULL){
        return NULL;
    }
    return NULL;
}
#define Used_B              0x0004U//100
#define Port_B              JL_PORTB->IN
#define DebounceT           8U
#define KEY_LONG_TICK_CNT   750U//3S   
#define DOUBLE_CHECK_TICK   50U//200ms     

static uint8_t  long_flag      = 0U;
static uint8_t  double_flag    = 0U;
static uint8_t  rel_not_flag   = 0U;
static uint16_t DebounceCNT    = 0U;
static uint16_t TmpKey         = 0U;
static uint16_t OldKey         = 0U;
static uint16_t NewKey         = 0U;
static uint16_t RelKey         = 0U;
static uint16_t long_cnt       = 0U;
static uint16_t double_cnt     = 0U;

void *key_read(void *dev) //4ms
{
    if (DebounceCNT != 0U)
    {
        uint16_t curr_sample = (~Port_B) & Used_B;
        if (TmpKey == curr_sample)
        {
            DebounceCNT++;
            if (DebounceCNT >= DebounceT)
            {
                NewKey = (OldKey ^ TmpKey) & TmpKey;
                RelKey = (OldKey ^ TmpKey) & OldKey;
                OldKey = TmpKey;
                DebounceCNT = 0U;
            }
        }
        else
        {
            DebounceCNT = 0U; 
        }
    }
    else
    {
        TmpKey = (~Port_B) & Used_B;
        if (OldKey != TmpKey)
        {
            DebounceCNT++;
        }
    }

    if (double_flag != 0U)
    {
        if (double_cnt < DOUBLE_CHECK_TICK)
        {
            double_cnt++;
        }
        else
        {
            long_flag    = 0U;
            long_cnt     = 0U;
            double_flag  = 0U;
            double_cnt   = 0U;
            rel_not_flag = 0U;
            //log_info("PB2 IOKEY PRESS\n");
            post_msg(2, MSG_KEY, PRESS);
        }
    }

    if (long_flag != 0U)
    {
        if (long_cnt < KEY_LONG_TICK_CNT)
        {
            long_cnt++;
            if (long_cnt >= KEY_LONG_TICK_CNT)
            {
                long_flag    = 0U;
                long_cnt     = 0U;
                double_flag  = 0U;
                double_cnt   = 0U;
                rel_not_flag = 0U;
                //log_info("PB2 IOKEY LONG PRESS\n");
                post_msg(2, MSG_KEY, LONG_PRESS);
            }
        }
    }

    if (NewKey != 0U)
    {
        if ((NewKey & BIT(2)) != 0U)
        {
            NewKey &= ~BIT(2);
            rel_not_flag = 0U;
            long_flag    = 1U;

            if (double_flag && (double_cnt < DOUBLE_CHECK_TICK))
            {
                long_flag    = 0U;
                long_cnt     = 0U;
                double_flag  = 0U;
                double_cnt   = 0U;
                rel_not_flag = 1U;
                //log_info("PB2 IOKEY DOUBLE PRESS\n");
                post_msg(2, MSG_KEY, DOUBLE_PRESS);
            }
        }
    }
    else if (RelKey != 0U)
    {
        if ((RelKey & BIT(2)) != 0U)
        {
            //log_info("PB2 IOKEY RELEASE\n");
            RelKey &= ~BIT(2);
            if ((long_flag == 1U) && (long_cnt < KEY_LONG_TICK_CNT) && (!rel_not_flag))
            {
                double_flag = 1U;
                double_cnt  = 0U;
            }
            long_flag = 0U;
            long_cnt  = 0U;
        }
    }
    return NULL;
}


void *key_write(void *dev, void *data, u32 len)
{
    return NULL;
}

void *key_ioctl(void *dev, u32 cmd, u32 arg)
{
    return NULL;
}