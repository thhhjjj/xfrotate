//programmer:Moments
#include "vfs.h"
#include "my_malloc.h"
#include "device.h"
#include "update.h"
#include "db_update.h"
#include "malloc.h"
#include "asm/power_interface.h"
#include "uart_communication.h"
#include <stdlib.h>
#include <string.h>
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"

UP_DATE *ctrl_updatep = NULL;
u32 save_version = 0;
u32 temp_version = 0;

/// @brief  ctrl pointer init
/// @param  NULL
/// @return UP_DATE *
UP_DATE *db_update_buf_malloc(void)
{
    // exsist ctrl_updatep
    if(ctrl_updatep != NULL){
        return ctrl_updatep;
    }
    // allocate mem
    ctrl_updatep = (UP_DATE *)malloc(sizeof(UP_DATE));
    if(ctrl_updatep == NULL){
        return NULL;
    }
    memset(ctrl_updatep, 0, sizeof(UP_DATE));
    ctrl_updatep->flash_dev = NULL;
    
    return ctrl_updatep;
}

/// @brief update init,need ctrl pointer
/// @param total_size need update total size
/// @param version    update.bin version//7.62=762
/// @param ctrl       control handle
/// @return error code,0: success
u32 db_update_init(u32 total_size,u32 version,UP_DATE *ctrl)
{
    log_info("===db_update init===");
    log_info("total_size:%d,version:%d",total_size,version);
    u32 res = 0;
    if(ctrl != ctrl_updatep){
        log_error("only support global update handle\n");
        res = PARAM_ERR;
        return res;
    }
    //read
    //version check
    if(save_version==version){
        log_error("same version !!!! \n");
        res = NO_NEED_UPGRADE;
        return res;
    }
    temp_version = version;
    //ctrl_updatep date init
    ctrl->total_size = total_size;
    // ctrl->cur_pkg_cnt = 0;
    ctrl->data_crc = 0;
    ctrl->data_offset = 0;
    //open internal flash
    ctrl->flash_dev = dev_open("sfc", NULL);
    if (ctrl->flash_dev == NULL) {
        log_error("flash_dev null !!!! \n");
        res = DEVIVE_OPEN_ERROR;
        goto __close_flash;
    }

    //get bank info
    u32 bank_start_addr = 0;
    u32 bank_size = 0;
    res = jlfs_get_idle_bank_info(&bank_start_addr, &bank_size);
    if(res != 0){
        log_error("get idle bank info fail !!!! \n");
        res = FILE_OPEN_ERROR;
        goto __close_flash;
    }
    log_info("addr 0x%x,size 0x%x\n", bank_start_addr, bank_size);
    ctrl->start_addr = bank_start_addr;
    ctrl->flash_offset = ctrl->start_addr;
    //check size
    u32 total_align_size = total_size + PACK_CNT*BYTE_CNT;//need bank size
    if(bank_size < total_align_size){
        log_error("bank size not enough !!!! \n");
        res = SPACE_NOT_ENOUGH;
        goto __close_flash;
    }

    //align size
    u32 cur_align_size = get_flash_alignsize();
    ctrl->flash_cmd = IOCTL_ERASE_SECTOR;
    if(cur_align_size == 256){
        ctrl->flash_cmd = IOCTL_ERASE_PAGE;
    }

    log_info("erase from 0x%x total align size:0x%x",ctrl->start_addr,total_align_size);
    //erase bank date
    u32 erase_cnt = total_align_size / cur_align_size;
    for(u32 i = 0; i < erase_cnt; i++)
    {
        u32 erase_addr = ctrl->start_addr + i * cur_align_size;
        res = dev_ioctl(ctrl->flash_dev, ctrl->flash_cmd, erase_addr);
        if(res){
            log_error("erase addr 0x%x fail !!!! \n", erase_addr);
            goto __close_flash;
        }
        wdt_clear();//clr wdt
    }

    log_info("db_update_init success,dev");//:rw%d",rw);
    return res;

__close_flash:
    if (ctrl->flash_dev != NULL){
        dev_close(ctrl->flash_dev);
        ctrl->flash_dev = NULL;
    }
    log_error("db_update_init fail, ret: %d", res);
    return res;
}

/// @brief write data
/// @param ctrl     control handle
/// @return error code,0: success,0xffffffff: recive over
u32 db_update_write(UP_DATE *ctrl)
{
    u32 res = 0;
    u32 total_len = ctrl->total_size;
    u32 single_offset = 0;

    if (ctrl->flash_dev == NULL || ctrl->data_buf == NULL){
        log_error("update dev or buf is null\n");
        return DEVIVE_OPEN_ERROR;
    }
    if (total_len == 0){
        log_error("firmware total size invalid\n");
        return GET_BUF_ERR;
    }
    // write 512 byte
    log_info("=======write_buf========");
    u32 pkg_len = ctrl->data_size;//
    // count crc16
    ctrl->data_crc = chip_crc16_with_init(ctrl->data_buf, pkg_len, ctrl->data_crc);
    dev_byte_write(ctrl->flash_dev, ctrl->data_buf, ctrl->flash_offset, pkg_len);
    if (res != 0){
        log_error("write error data size:%d",res);
        res = WRITE_BUF_ERR;
        log_error("write flash addr:0x%x,data offset:%d,fail code:%d\n", ctrl->flash_offset,ctrl->data_offset,res);
        return res;
    }
    ctrl->data_offset += pkg_len;
    ctrl->flash_offset = ctrl->start_addr + ctrl->data_offset;
    if(ctrl_updatep->data_offset == ctrl_updatep->total_size){//recive over
        res = 0xffffffff;
    }
    wdt_clear();//clr wdt
    if(res == 0xffffffff){
        log_info("total data write finish,total data size:%d, total crc:0x%x",ctrl->data_offset,ctrl->data_crc);
    }else{
        log_info("write addr:0x%x len:%d crc:%x",ctrl->flash_offset, pkg_len, ctrl->data_crc);
    }
    return res;
}

/// @brief check crc
/// @param ctrl       control handle
/// @return error code,0: success
u32 db_update_verify(UP_DATE *ctrl)
{
    u32 res = 0;
    if (ctrl->flash_dev == NULL){
        log_error("flash dev is null\n");
        return DEVIVE_OPEN_ERROR;
    }

    // set flash read only
    dev_ioctl(ctrl->flash_dev, IOCTL_SET_SFC_READ, 0);

    // write dual bank info
    res = jlfs_updata_dual_bank_info(ctrl->start_addr, ctrl->data_crc);
    if (res != 0){
        log_error("update dual bank info fail\n");
        goto __verify_err;
    }
    dev_ioctl(ctrl->flash_dev, IOCTL_SET_SFC_READ, 1);
    // check bank header info
    res = jlfs_check_dual_bank_info(ctrl->start_addr);
    if (res != 0){
        log_error("bank header check fail\n");
        goto __verify_err;
    }
    //check bank flash just check head and tail
    //reserve

    log_info("all data verify success\n");
    dev_ioctl(ctrl->flash_dev, IOCTL_SET_SFC_READ, 0);
    return 0;

__verify_err:
    // erase error bank
    log_info("all data verify fail\n");
    dev_ioctl(ctrl->flash_dev, IOCTL_SET_SFC_READ, 0);
    return res;
}


/// @brief deinit
/// @param opt 1:wdt reset,0:not reset
/// @param ctrl      control handle
void db_update_deinit(u8 opt, UP_DATE *ctrl)
{
    if (ctrl == NULL){
        log_error("update ctrl pointer is null\n");
        return;
    }

    if (ctrl->flash_dev != NULL)
    {
        dev_ioctl(ctrl->flash_dev, IOCTL_SET_SFC_READ, 0);
        log_info("sfc cache close");
    }

    // close flash device
    if (ctrl->flash_dev != NULL){
        dev_close(ctrl->flash_dev);
        ctrl->flash_dev = NULL;
    }

    log_info("update deinit finish, wait reset\n");
    // free update control struct
    ctrl_updatep = NULL;
    free(ctrl);

    // maybe wdt reset
    if(opt){    
        save_version = temp_version;
        log_info("update reset");
        extern void chip_reset();
        chip_reset();
        while(1);
    }
}
