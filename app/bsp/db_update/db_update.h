#ifndef DB_UPDATE_H
#define DB_UPDATE_H
#include "typedef.h"
#define USE_SD_SIMP_FAT 1   //simple_fs
#define PACK_CNT        8   //pkg_num
#define BYTE_CNT        512 //byte_num

//UP_DATE
typedef struct
{
    //data write
    u8 *data_buf;
    u32 data_size;
    u32 data_offset;
    u32 total_size;
    //u8  cur_pkg_cnt;    //0~7
    u32 data_crc;
    //flash
    void *flash_dev;    //internal flash devp
    u32 start_addr;     //write start addr
    u32 flash_cmd;
    u32 flash_offset;
}UP_DATE;

extern UP_DATE *ctrl_updatep;//update control pointer
//low level interface
u16 chip_crc16_with_init(void *ptr, u32  len, u32 init);
u32 jlfs_get_idle_bank_info(u32 *bank_addr, u32 *bank_size);
u32 get_flash_alignsize(void);
u32 jlfs_updata_dual_bank_info(u32 bank_addr, u16 data_crc);
u32 jlfs_check_dual_bank_info(u32 update_bank_addr);
//upper interface
extern UP_DATE *db_update_buf_malloc(void);
extern u32 db_update_init(u32 total_size,u32 version,UP_DATE *ctrl);
extern u32 db_update_write(UP_DATE *ctrl);
extern u32 db_update_verify(UP_DATE *ctrl);
extern void db_update_deinit(u8 opt, UP_DATE *ctrl);
#endif