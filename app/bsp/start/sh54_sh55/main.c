/*********************************************************************************************
    *   Filename        : main.c

    *   Description     :

    *   Author          :

    *   Email           :

    *   Last modifiled  :

    *   Copyright:(c)JIELI  2011-2017  @ , All Rights Reserved.
*********************************************************************************************/
#include "config.h"
#include "common.h"
#include "maskrom.h"
#include "asm/power/p33.h"
#include "app_config.h"
#include "init.h"
#include "vm_api.h"
#include "asm/power_interface.h"
#include "power_api.h"

#define LOG_TAG_CONST       MAIN
#define LOG_TAG             "[main]"
#include "log.h"

#define  APP_CODE_DEFAULT     0
extern void led_io_init(void);
extern void key_io_init(void);
extern void infrared_io_init(void);
extern void servo_io_init(void);
extern void app(void);
int c_main(int cfg_addr)
{
    VDC13_LOAD_EN(1);

#if 1
    ///暂时调低p33跑的波特率
    JL_P33->SPI_CON |= (BIT(2) | BIT(3));
#endif

    /* spi_cache_way_switch(1); */
    /* request_irq(1, 7, exception_irq_handler, 0); */
    mask_init(exception_analyze, putchar);
    all_init_isr();

    log_init(921600);

    log_info("---------sh5x apps------------ \n");
    // r_printf("---------NEW apps------------ \n");

    p33_tx_1byte(P3_PINR_CON, 0);

    reset_source_dump();
    power_reset_source_dump();

    //LVD电压配置，默认关闭
    /* p33_vlvd(LVLD_SEL_25V,0);//0:reset , 1:interrupt(wkup)*/

    //注:soft_off会Latch io, 唤醒之后电源初始化才会释放io，所以在电源初始化之后才能翻io/打印
    sys_power_init();

    //--- OSC CL  12M
    SFR(JL_CLK->CON0, 19, 2, 3);

    system_init();
    
    led_io_init();
    key_io_init();
    infrared_io_init();
    servo_io_init();
//     extern void get_dual_bank_info(void);
//     get_dual_bank_info();
//     extern void vfs_demo(void);
//     vfs_demo();

//     u8 test_data = 0;
//     vm_read(VM_INDEX_USER_TEST, &test_data, sizeof(test_data));

//     // 地址说明 
// #if APP_CODE_DEFAULT 
//     r_printf("---------app_defualt------------ \n");
//     if(test_data == 0){
//         test_data = 0xaa;
//         vm_write(VM_INDEX_USER_TEST, &test_data, sizeof(test_data));
//     }
//     vm_read(VM_INDEX_USER_TEST, &test_data, sizeof(test_data));
//     r_printf("test_data:%d  ",test_data);    
//     extern void dual_bank_test();
//     dual_bank_test();
// #else
//     r_printf("---------app_NEW------------ \n");
//     r_printf("test_data:%x  ",test_data);    //测试VM
// #endif
    app();
    while (1) {
        wdt_clear();
    }
}


