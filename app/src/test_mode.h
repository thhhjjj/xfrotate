#ifndef _TEST_MODE_H
#define _TEST_MODE_H
#include "typedef.h"

/* protocol item id */
#define TEST_ITEM_KEY     0x01
#define TEST_ITEM_IR      0x02
#define TEST_ITEM_LED     0x03
#define TEST_ITEM_SERVO   0x04

#define TEST_RESULT_OK    0x00
#define TEST_RESULT_FAIL  0x01

/* per-item timeout: 60 * 500ms = 30s */
#define TEST_ITEM_TIMEOUT_TICKS  60

extern u8 g_test_mode;
extern u8 key_test_flag;
extern u8 ir_test_flag;
extern u8 servo_test_flag;
extern u8 servo_left_switch;
extern u8 servo_mid_switch;
extern u8 servo_right_switch;

void test_mode_enter(void);
void test_mode_exit(void);
extern void test_poll_500ms(void);

#endif
