#include "gpio.h"
#include "uart_communication.h"
#include "malloc.h"
#include "my_malloc.h"
#include "db_update.h"
#include <stdlib.h>
#include <string.h>
#define LOG_TAG_CONST       UTD
#define LOG_TAG             "[uart communication]"
#include "log.h"
#include "msg.h"
#include "jiffies.h"
#include "vm_api.h"
#include "servo_control.h"
#include "infrared_control.h"
#include "user_define.h"
#include "test_mode.h"
#include "printf.h"
UART_BUF Uart_buf = {0x00};
ZC_UART *ZcUart = NULL;
// u8 recv_buf[13]={0x00};
u8 serial = 0;
u8 off = 0;
u8 g_ota_busy = 0;
static u32 ota_lost_pkg = 0;
static u16 ota_total_pkg = 0;

extern SERVO_CTRL *servo_ctrl;
extern void uart1_tx_send(u8 *buf, int buf_size);
static unsigned short checksum_frame(u8 *buf, u32 need_byte);
//=====================================================================================================================
//                                                      Others about
//=====================================================================================================================
/* 应答/推送专用; SendBuff 只留给 send_cmd_by_uart, 供 zc_await_reply 重发 */
static u8 ReplyBuff[UART_SEND_BUFF_LEN];
static u32 rx_last_jiffies;
/* 残帧超过该时间没有新数据, 认为是坏帧头, 丢掉重找 */
#define UART_RX_STALE_JIFFIES   (100 / (1000 / HZ))

//data send buf
/* 同步发送: 返回时 buf 已发完, 调用方可以马上复用 */
void data_send(u8 *buf,int buf_size)
{
    if (!g_ota_busy) {
        log_info("===========send data===========");
        put_buf(buf, buf_size);
    }
    uart1_tx_send(buf, buf_size);
}

static void data_send_quiet(u8 *buf, int buf_size)
{
    uart1_tx_send(buf, buf_size);
}

static int uart_frame_build(u8 *out, u32 out_size,
                            unsigned char Type,
                            unsigned char *data,
                            unsigned short len,
                            unsigned short Fuid,
                            unsigned short total_num)
{
    UART_FRAME_HEAD FrameHead = {0};
    unsigned short templen = HEAD_LEN + len + 1;
    unsigned short Chk = 0;

    if (templen > out_size) {
        log_info("uart frame too long:%d", templen);
        return 0;
    }
    FrameHead.flag = UART_FRAM_FLAG;
    FrameHead.len = 1 + len;
    FrameHead.cur_num = Fuid;
    FrameHead.total_num = total_num;
    memcpy(&out[0], &FrameHead, HEAD_LEN);
    out[HEAD_LEN] = Type;
    if (len && data) {
        memcpy(&out[HEAD_LEN + 1], data, len);
    }
    Chk = checksum_frame(out, templen);
    out[8] = Chk;//low bit
    out[9] = (Chk >> 8) & 0xff;//high bit
    return templen;
}

/* build reply frame then TX immediately (for OTA finish before reset) */
static int send_reply_by_uart_sync(unsigned char Type,
                                   unsigned char *data,
                                   unsigned short len,
                                   unsigned short Fuid,
                                   unsigned short total_num)
{
    int templen;
    if (!ZcUart)
        return -1;
    templen = uart_frame_build(ReplyBuff, sizeof(ReplyBuff), Type, data, len, Fuid, total_num);
    if (!templen)
        return -1;
    uart1_tx_send(ReplyBuff, templen);
    return 0;
}
//reset recv buff
void reset_recv_states(void)
{
    //log_info("check fail maybe head or checksum error");
    memset(RecvBuff, 0, UART_RECV_BUFF_LEN);
    if (ZcUart->RecvData){
        ZcUart->RecvData = NULL;
    }
    ZcUart->rx_rev_len = 0;
    ZcUart->rx_recving = 0;
}
//frame head check: flag / len / cur_num / 整帧能放进 RecvBuff
static int uart_head_valid(const UART_FRAME_HEAD *head)
{
    if (head->flag != UART_FRAM_FLAG) {
        return 0;
    }
    if (head->len == 0 || head->cur_num >= head->total_num) {
        return 0;
    }
    if ((u32)HEAD_LEN + head->len > UART_RECV_BUFF_LEN) {
        return 0;
    }
    return 1;
}
//check sum
static unsigned short checksum_frame(u8 *buf, u32 need_byte)
{
    unsigned int retsum = 0;
    // 遍历该帧所有字节
    for (u32 i = 0; i < need_byte; i++)             //flag      0 1
    {                                               //len       2 3
        if (i == 8 || i == 9)//去掉checksum byte    //cur_num   4 5
            continue;                               //total_num 6 7
        retsum += buf[i];                           //checksum  8 9
    }
    return retsum&(0xffff);
}

//recive type check
u8 recive_frame_check(u8 type)
{
    switch(type)
    {
        //judge whether it is receive cmd?
        case OTA_INIT:
        case OTA_RECIVE:
        case OTA_READ_VERSION:
        case UART_DATA_GET_UUID:
        case UART_DATA_GET_SN:
        case UART_DATA_SET_SN:
        case UART_DATA_IR_ONOFF:
        case UART_DATA_SR_ONOFF:
        case UART_DATA_IR_STATUS:
        case UART_DATA_SR_SET_STATUS:
        case UART_DATA_TEST_MODE:
        case UART_DATA_SR_CUR_ANGLE:
            return 2;
        break;
        //judge whether it is receive callback cmd?
        case OTA_READ_VERSION|0x80:
        case OTA_SUCCESS|0x80:
        case OTA_FAIL|0x80:
        case UART_DATA_GET_UUID|0x80:
        case UART_DATA_GET_SN|0x80:
        case UART_DATA_SET_SN|0x80:
        case UART_DATA_IR_ONOFF|0x80:
        case UART_DATA_SR_ONOFF|0x80:
        case UART_DATA_IR_STATUS|0x80:
        case UART_DATA_SR_SET_STATUS|0x80:
        case UART_DATA_TEST_MODE|0x80:
        case UART_DATA_TEST_RESULT|0x80:
            return 1;
        break;
        default:
            return 0;
    }
}

//=====================================================================================================================
//                                                      UART init about
//=====================================================================================================================
int zc_uart_init(void)
{
    log_info("zc_uart_init\r\n");
    if (!ZcUart) {
        ZcUart = malloc(sizeof(ZC_UART));
        if (!ZcUart) {
            log_info("uart malloc err\r\n");
            return -1;
        }
        memset(ZcUart, 0, sizeof(ZC_UART));
        //uart_tr_init();
    }
    return 0;
}

int zc_uart_deinit(void)
{
    log_info("zc_uart_deinit\r\n");
    if (ZcUart) {
        free(ZcUart);
        ZcUart = NULL;
        //uart_tr_suspend();
    }
    return 0;
}
//=====================================================================================================================
//                                                     UART send about
//=====================================================================================================================
u8 reply_buf = 0x00;
int send_reply_by_uart(unsigned char Type,       // 1. 命令类型
                       unsigned char *data,      // 2. 发送的数据buff
                       unsigned short len,       // 3. 数据长度
                       unsigned short Fuid,      // 4. 包序号//0 start
                       unsigned short total_num) // 5. 总包数
{
    int templen;
    if (!ZcUart)
        return -1;
    templen = uart_frame_build(ReplyBuff, sizeof(ReplyBuff), Type, data, len, Fuid, total_num);
    if (!templen)
        return -1;
    data_send(ReplyBuff, templen);
    return 0;
}


int send_cmd_by_uart(unsigned char Type,       // 1. 命令类型
                     unsigned char *data,      // 2. 发送的数据buff
                     unsigned short len,       // 3. 数据长度
                     unsigned short Fuid,      // 4. 包序号//0 start
                     unsigned short total_num) // 5. 总包数
{
    int templen;
    if (!ZcUart)
        return -1;
    templen = uart_frame_build(SendBuff, sizeof(SendBuff), Type, data, len, Fuid, total_num);
    if (!templen)
        return -1;
    ZcUart->tx_send_len = templen;
    data_send(SendBuff,ZcUart->tx_send_len);
    ZcUart->tx_sending = 1;
    ZcUart->tx_sending_cnt = 0;
    return 0;
}

// push frame: send once per state change, no auto retransmit
int send_push_by_uart(unsigned char Type, unsigned char *data, unsigned short len, unsigned short Fuid, unsigned short total_num)
{
    int templen;

    if (!ZcUart) {
        return -1;
    }
    templen = uart_frame_build(ReplyBuff, sizeof(ReplyBuff), Type, data, len, Fuid, total_num);
    if (!templen) {
        return -1;
    }
    data_send_quiet(ReplyBuff, templen);
    return 0;
}

// 没有接收到回复，连发三次
void zc_await_reply(void)//300ms
{
    if (!ZcUart) {
        return;
    }
    if (ZcUart->tx_sending)
    {
        ZcUart->tx_sending_cnt++;
        if (ZcUart->tx_sending_cnt > 10)
        {
            ZcUart->tx_sending_cnt = 0;
            data_send(SendBuff,ZcUart->tx_send_len);
            ZcUart->tx_sending++;
            if (ZcUart->tx_sending > 2)
                ZcUart->tx_sending = 0;
        }
    }
}
//=====================================================================================================================
//                                                     UART recv about
//=====================================================================================================================
int err_code = 0;
#if INFRARED_EN
extern INFRARED_CTRL *infrared_ctrl;
#endif
extern SERVO_CTRL *servo_ctrl;
extern void nostalgia_write_userid(void);
extern char sn[33];
void bytes_to_hex_str(uint8_t *data, char *str) {
    for (int i = 0; i < 16; i++) {
        sprintf(str + i * 2, "%02X", data[i]);
    }
    str[32] = '\0';  // 添加字符串结束符
}
static void uart_recv_handle(char *FrameData)
{
//safe check
    if(FrameData == NULL) return;
    switch (recive_frame_check(FrameData[HEAD_LEN]))
    {
        case 0:
        log_info("FrameData[HEAD_LEN]=%x",FrameData[HEAD_LEN]);
        log_info("it is not receive frame");
        return;

        case 1:
        ZcUart->tx_sending_cnt = 0;
        ZcUart->tx_sending = 0;
        log_info("call back frame");
        return;//回复帧不再重发
    }

//malloc orderbuf
    UART_CB_DATA *orderbuf = malloc(sizeof(UART_CB_DATA));
    if (orderbuf == NULL) {
        log_info("orderbuf malloc failed\r\n");
        return;
    }
    UART_FRAME_HEAD *head = &ZcUart->FrameHead;

//load
    orderbuf->Type    = FrameData[HEAD_LEN];
    orderbuf->Fuid    = head->cur_num;
    orderbuf->allFuid = head->total_num;
    orderbuf->data_len= head->len - 1;
    orderbuf->Data    = (unsigned char *)(FrameData + HEAD_LEN + 1);

//check data length
    if ((head->len == 0)||(orderbuf->Data == NULL)) {
        log_info("frame length only type without data\r\n");
        free(orderbuf);
        orderbuf = NULL;
        return;
    }

//receive cmd need reply
    switch(orderbuf->Type){//判断是需要回复类型
        // case OTA_INIT:
        //      send_reply_by_uart(orderbuf->Type,NULL,0,0,1);
        //      break;
        default:
             break;
    }

//func
    if(orderbuf==NULL){
        return;
    }
    if (!g_ota_busy) {
        log_info("=========type:%x==========",orderbuf->Type);
    }
    //put_buf(orderbuf->Data,orderbuf->data_len);
    switch (orderbuf->Type)
    {
        case UART_DATA_GET_UUID:{
            u8 tmpData[32+1] = {0};
            bytes_to_hex_str((uint8_t*)gd.save_data.uuid, (char*)tmpData);
            send_reply_by_uart(UART_DATA_GET_UUID,tmpData,32,0,1);
            break;
        }
        case UART_DATA_GET_SN:{
            u8 tmpData[32] = {0};
            if(gd.save_data.sn_exist){
                memcpy(tmpData, gd.save_data.sn, strlen(gd.save_data.sn));
            }               
            send_reply_by_uart(UART_DATA_GET_SN,(unsigned char*)tmpData,strlen((char*)tmpData),0,1);
            break;
        }
        case UART_DATA_SET_SN:{
            gd.save_data.sn_exist = 1;
            sn[0] = 1;
            memcpy(sn+1, orderbuf->Data, orderbuf->data_len);
            gd.save_data.sn=&sn[1];
            vm_write(SN_INFO_SAVE,(u8*)sn, sizeof(sn));
            send_reply_by_uart(UART_DATA_SET_SN,(unsigned char*)&sn[0],1,0,1);
            break;
        }
        case UART_DATA_IR_ONOFF:{
            u8 tmpData = 0;
#if INFRARED_EN
            if(infrared_ctrl){
                tmpData = 0x01;
            }
#endif
            send_reply_by_uart(UART_DATA_IR_ONOFF,&tmpData,1,0,1);
            break;
        }
        case UART_DATA_SR_ONOFF:
            u8 tmpData = 0;
            if(servo_ctrl){
                tmpData = 0x01;
            }else{
                tmpData = 0x00;
            }
            send_reply_by_uart(UART_DATA_SR_ONOFF,&tmpData,1,0,1);
            break;
        case UART_DATA_IR_STATUS:{
            u8 tmpData = 0;
#if INFRARED_EN
            if(infrared_ctrl){
                if(infrared_ctrl->human_flag){
                    tmpData = infrared_ctrl->human_flag;
                }
            }
#endif
            send_reply_by_uart(UART_DATA_IR_STATUS,&tmpData,1,0,1);
            break;
        }
        case UART_DATA_SR_SET_STATUS:{
             u32 angle =  orderbuf->Data[0]        |
                         (orderbuf->Data[1] << 8)  |
                         (orderbuf->Data[2] << 16) |
                         (orderbuf->Data[3] << 24);
            log_info("||===============================================||",angle);   
            log_info("||====================angle:%d===================||",angle); 
            log_info("||===============================================||",angle);             
            put_buf(orderbuf->Data,4);
            if(servo_ctrl && orderbuf->data_len >= 4){
                gd.dev_table[SERVO_DEV].dev_ioctl(servo_ctrl, SERVO_CMD_SET_ANGLE, (u32)angle);
            }
            send_reply_by_uart(UART_DATA_SR_SET_STATUS,&orderbuf->Data[0],4,0,1);
            break;
        }
        case UART_DATA_TEST_MODE:{
            u8 reply = 0x00;
            u8 do_enter = 0;
            if (orderbuf->data_len < 1) {
                send_reply_by_uart(UART_DATA_TEST_MODE, &reply, 1, 0, 1);
                break;
            }
            if (orderbuf->Data[0] == 0x01) {
                if (!g_ota_busy) {
                    reply = 0x01;
                    do_enter = 1;
                }
            } else {
                test_mode_exit();
                reply = 0x00;
            }
            send_reply_by_uart(UART_DATA_TEST_MODE, &reply, 1, 0, 1);
            if (do_enter) {
                test_mode_enter();
            }
            break;
        }
        case UART_DATA_SR_CUR_ANGLE:{
            u8 tmpData[4] = {0};
            tmpData[0]=servo_ctrl->obj_angle;//target angle of servo motor
            tmpData[1]=servo_ctrl->obj_angle>>8;
            tmpData[2]=0x00;
            tmpData[3]=0x00;
            send_reply_by_uart(UART_DATA_SR_CUR_ANGLE,tmpData,4,0,1);
            break;
        }
        case OTA_READ_VERSION:{
            u8 tmpData[6] = {0};
            memcpy(tmpData, gd.save_data.version, 6);
            send_reply_by_uart(OTA_READ_VERSION,tmpData,6,0,1);
            break;
        }
        case OTA_INIT:{
            int t_size = orderbuf->Data[0]        |
                        (orderbuf->Data[1] << 8)  |
                        (orderbuf->Data[2] << 16) |
                        (orderbuf->Data[3] << 24);
            u32 c_ver = 0xffffffffU;
            ota_lost_pkg = 0;
            ota_total_pkg = 0;
            ctrl_updatep = db_update_buf_malloc();
            if(ctrl_updatep == NULL){
                u8 send_buf = 0x01;
                g_ota_busy = 0;
                send_reply_by_uart(OTA_INIT,&send_buf,1,0,1);
                break;
            }
            err_code = db_update_init(t_size,c_ver,ctrl_updatep);
            if(err_code){
                db_update_deinit(0,ctrl_updatep);
                u8 send_buf = 0x01;
                g_ota_busy = 0;
                send_reply_by_uart(OTA_INIT,&send_buf,1,0,1);//initial fail
            }else{
                u8 send_buf = 0x00;
                g_ota_busy = 1;
                send_reply_by_uart(OTA_INIT,&send_buf,1,0,1);//initial success
            }
            break;
        }
        case OTA_RECIVE:{
            if((orderbuf->allFuid != ota_total_pkg) && (ota_total_pkg)){
                goto __break;
            }
            if(orderbuf->data_len > UART_OTA_MAX_DATA_LEN){
                goto __break;
            }
            if (!ota_total_pkg){
                ota_total_pkg = orderbuf->allFuid;
            }
            if((orderbuf->Fuid == ota_lost_pkg) || ((!ota_lost_pkg) && (!orderbuf->Fuid))){
                ota_lost_pkg++;
                ctrl_updatep->data_buf = orderbuf->Data;
                ctrl_updatep->data_size = orderbuf->data_len;
                err_code = db_update_write(ctrl_updatep);

                if(err_code==0xffffffff){//recive over
                    ota_lost_pkg = 0;
                    ota_total_pkg = 0;
                    u8 send_buf = 0x00;
                    send_reply_by_uart_sync(OTA_RECIVE,&send_buf,1,0,1);
                    goto __recive_ok;
                }else{
                    u8 send_buf = 0x00;
                    if(err_code)
                        send_buf = 0x01;
                    /* OTA path: sync TX so host can ACK-next immediately */
                    if (g_ota_busy)
                        send_reply_by_uart_sync(OTA_RECIVE,&send_buf,1,0,1);
                    else
                        send_reply_by_uart(OTA_RECIVE,&send_buf,1,0,1);
                }
            }else{//丢包，请求重传当前lost_pkg
                u8 send_buf[5] = {0};
                send_buf[0] = 0x01;
                if (g_ota_busy)
                    send_reply_by_uart_sync(OTA_RECIVE, send_buf, 1, 0, 1);
                else
                    send_reply_by_uart(OTA_RECIVE, send_buf, 1, 0, 1);
            }
            break;
__recive_ok:
            {
                u8 send_buf = 0x00;
                err_code = db_update_verify(ctrl_updatep);
                if(err_code){
                    g_ota_busy = 0;
                    send_reply_by_uart_sync(OTA_FAIL,&send_buf,1,0,1);
                    db_update_deinit(0,ctrl_updatep);
                }else{
                    send_reply_by_uart_sync(OTA_SUCCESS,&send_buf,1,0,1);
                    g_ota_busy = 0;
                    db_update_deinit(1,ctrl_updatep);// success -> reset
                }
            }
            break;
__break:
            {
                u8 send_buf = 0x02;
                if (g_ota_busy)
                    send_reply_by_uart_sync(OTA_RECIVE,&send_buf,1,0,1);
                else
                    send_reply_by_uart(OTA_RECIVE,&send_buf,1,0,1);
            }
            break;
        }
//         case OTA_RECIVE:{
//                 static u32 lost_pkg = 0;
//                 static u16 total_pkg = 0;
//                 static u16 pkg_offset = 0;
//                 static u8 *temp_buf = NULL;
//                 static u16 temp_buf_size = 0;
//                 if((orderbuf->allFuid != total_pkg)&&(total_pkg)){//total pkg error
//                     goto __break;
//                 }
//                 if(orderbuf->data_len>512){//pkg length error
//                     goto __break;
//                 }
//                 if (!total_pkg){
//                     total_pkg = orderbuf->allFuid;
//                 }
//                 if((orderbuf->Fuid==lost_pkg)||((!lost_pkg)&&(!orderbuf->Fuid))){//pkg recive success
//                     lost_pkg++;
//                     u8 send_buf = 0x00;
//                     send_reply_by_uart(OTA_RECIVE,&send_buf,1,0,1);
//                     if((!pkg_offset)&&(temp_buf!=NULL)&&(temp_buf_size)){
//                         memcpy(ctrl_updatep->data_buf,temp_buf,temp_buf_size);
//                         pkg_offset+=temp_buf_size;
//                         free(temp_buf);
//                         temp_buf_size = 0;
//                         temp_buf = NULL;
//                     }
//                     if(pkg_offset+orderbuf->data_len>4096){
//                         memcpy(ctrl_updatep->data_buf+pkg_offset,orderbuf->Data,4096-pkg_offset);
//                         temp_buf_size = pkg_offset+orderbuf->data_len-4096;
//                         if(temp_buf != NULL){
//                             free(temp_buf);
//                             temp_buf = NULL;
//                             temp_buf_size = 0;
//                         }
//                         temp_buf = malloc(temp_buf_size);
//                         memcpy(temp_buf,orderbuf->Data+(4096-pkg_offset),temp_buf_size);
//                         goto __recive_ok;
//                     }else if(pkg_offset+orderbuf->data_len==4096){
//                         memcpy(ctrl_updatep->data_buf+pkg_offset,orderbuf->Data,orderbuf->data_len);
//                         goto __recive_ok;
//                     }else{
//                         memcpy(ctrl_updatep->data_buf+pkg_offset,orderbuf->Data,orderbuf->data_len);
//                         pkg_offset+=orderbuf->data_len;
//                     }
//                 }else{//lost pkg
//                     u8 send_buf[5] = {0};
//                     send_buf[0]=0x01;
//                     send_buf[1]=lost_pkg&0xff;
//                     send_buf[2]=(lost_pkg>>8)&0xff;
//                     send_buf[3]=(lost_pkg>>16)&0xff;
//                     send_buf[4]=(lost_pkg>>24)&0xff;
//                     send_reply_by_uart(OTA_RECIVE,send_buf,5,0,1);
//                 }
//                 break;
// __recive_ok:
//         total_pkg = 0;
//         pkg_offset = 0;
//         if(orderbuf->Fuid==total_pkg){
//             if(temp_buf!=NULL){
//                 free(temp_buf);
//                 temp_buf = NULL;
//                 temp_buf_size = 0;
//             }
//         }
//         err_code = db_update_write(ctrl_updatep);
//         if(ctrl_updatep->data_offset == ctrl_updatep->total_size){//all ota package recive over
//             lost_pkg = 0;
//             err_code = db_update_verify(ctrl_updatep);
//             u8 send_buf = (u8)err_code;
//             send_reply_by_uart(OTA_FEEDBACK,&send_buf,1,0,1);//0x00 sucess others error code
//             db_update_deinit(!send_buf,ctrl_updatep);
//         }
//         break;
// __break:
//         {u8 send_buf = 0x02;
//         send_reply_by_uart(OTA_RECIVE,&send_buf,1,0,1);}
//         }break;
    }

__free_orderbuf:
    //free orderbuf
    if(orderbuf!=NULL){
        free(orderbuf);
        orderbuf = NULL;
    }
}

/*
 * 从 RecvBuff 头部开始连续解析, 一次可处理多帧, 解析完把剩余字节挪到头部.
 * flush=1: 残帧已超时, 不再等它补齐, 逐字节往后重找帧头.
 */
static void uart_recv_parse(u8 flush)
{
    int total = ZcUart->rx_rev_len;
    int rd = 0;
    int skip = 0;
    u32 need_byte;
    u16 flag;
    unsigned short chk;
    UART_FRAME_HEAD head;

    while (total - rd >= (int)HEAD_LEN) {
        memcpy(&flag, RecvBuff + rd, sizeof(flag));
        if (flag != UART_FRAM_FLAG) {
            rd++;
            skip++;
            continue;
        }
        memcpy(&head, RecvBuff + rd, HEAD_LEN);
        if (!uart_head_valid(&head)) {
            rd++;
            skip++;
            continue;
        }
        need_byte = HEAD_LEN + head.len;
        if ((u32)(total - rd) < need_byte) {
            if (!flush) {
                break;
            }
            rd++;
            skip++;
            continue;
        }
        chk = checksum_frame(RecvBuff + rd, need_byte);
        if (chk != head.checksum) {
            log_info("checksum fail,expected:%04x,actual:%04x", chk, head.checksum);
            rd++;
            skip++;
            continue;
        }
        if (!g_ota_busy) {
            log_info("checksum pass");
        }
        memcpy(&ZcUart->FrameHead, &head, HEAD_LEN);
        ZcUart->RecvData = RecvBuff + rd;
        ZcUart->rx_recving = 1;
        uart_recv_handle((char *)(RecvBuff + rd));
        rd += need_byte;
    }
    if (flush) {
        rd = total;
    }
    if (skip && !g_ota_busy) {
        log_info("uart rx skip %d byte", skip);
    }

    if (rd >= total) {
        ZcUart->rx_rev_len = 0;
    } else if (rd) {
        memmove(RecvBuff, RecvBuff + rd, total - rd);
        ZcUart->rx_rev_len = total - rd;
    }
    ZcUart->rx_recving = 0;
    ZcUart->RecvData = NULL;
}

void uart_recv_task(char *buf, int len)
{
    if (!ZcUart || buf == NULL || len <= 0) return;
    if (len > UART_RECV_BUFF_LEN) {
        buf += len - UART_RECV_BUFF_LEN;
        len = UART_RECV_BUFF_LEN;
    }
    if (ZcUart->rx_rev_len + len > UART_RECV_BUFF_LEN) {
        /* 放不下时丢最旧的字节, 保留刚收到的数据 */
        int drop = ZcUart->rx_rev_len + len - UART_RECV_BUFF_LEN;
        memmove(RecvBuff, RecvBuff + drop, ZcUart->rx_rev_len - drop);
        ZcUart->rx_rev_len -= drop;
        log_info("uart rx overflow, drop %d byte", drop);
    }
    if (!g_ota_busy) {
        log_info("===========recv data,len:%d===========",len);
        put_buf((u8 *)buf, len);
    }
    memcpy(RecvBuff + ZcUart->rx_rev_len, buf, len);
    ZcUart->rx_rev_len += len;
    rx_last_jiffies = jiffies;
    uart_recv_parse(0);
}

/* 主循环定时调用: 残帧长时间补不齐时清掉, 避免挡住后面的帧 */
void uart_recv_poll(void)
{
    if (!ZcUart || !ZcUart->rx_rev_len) {
        return;
    }
    if ((u32)(jiffies - rx_last_jiffies) < UART_RX_STALE_JIFFIES) {
        return;
    }
    uart_recv_parse(1);
}

