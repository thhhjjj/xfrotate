#include "typedef.h"

#ifndef _UART_COMMUNICATION_H
#define _UART_COMMUNICATION_H
#define USE_UART_RECV
#define USE_UART_SEND
// ========================= 配置 =========================
//macro align protocols
#define UART_FRAM_FLAG          0x4658

//#define MAX_FRAME_LEN           1024
//#define MAX_OTA_BYTE            1024
#define UART_RECV_BUFF_LEN      (700)
#define UART_SEND_BUFF_LEN      (256)

// ======================== 命令枚举 =======================
// command class
typedef enum {
    UART_NOT_DONE             = 0x00,
    //Uart OTA:
    OTA_INIT,                  
    OTA_RECIVE,                
    OTA_FAIL, 
    OTA_SUCCESS,
    OTA_READ_VERSION,
    //Dev contorl:
    UART_DATA_GET_UUID,
    UART_DATA_GET_SN,
    UART_DATA_SET_SN,
    UART_DATA_IR_ONOFF,
    UART_DATA_SR_ONOFF,
    UART_DATA_IR_STATUS,
    UART_DATA_SR_SET_STATUS,
} UART_DATA_TYPE;
// ======================= 帧头结构体 =======================
// frame head define
#pragma pack(1)
typedef struct {
    unsigned short flag;       // UART_FRAM_FLAG 小端：0x46 0x58
    unsigned short len;        // 数据区长度 = 1(类型) + N(数据)
    unsigned short cur_num;    // 当前帧序号
    unsigned short total_num;  // 总帧数
    unsigned short checksum;   // 除自身外所有数据和
    unsigned char reserve[6];  // 6字节预留
} UART_FRAME_HEAD;
#pragma pack()
#define HEAD_LEN    sizeof(UART_FRAME_HEAD)//size

// ========================= 数据结构 ========================
typedef struct {
    UART_DATA_TYPE Type;
    u16 Fuid;
    u16 allFuid;
    u16 data_len;
    u8 *Data;
} UART_CB_DATA;//data func call back

typedef struct zc_uart {
    // ==================== 接收端状态 RX ====================
    int rx_recving;        // 【接收状态】
    int rx_rev_len;        // 【已接收长度】RecvBuff里的字节数
    // ==================== 发送端状态 TX ====================
    int tx_sending;        // 【发送状态】计次数
    int tx_send_len;       // 【发送总长度】本次发送字节数
    u16 tx_sending_cnt;    // 【发送超时计数】超时计数
    u32 tx_total_len;      // 【已发送总长度】发送总字节计数
    // ==================== 数据指针 ====================
    u8 *RecvData;          // 【指向接收缓存】
    u8 *SendData;          // 【指向发送缓存】
    // ==================== 当前帧信息 ====================
    UART_FRAME_HEAD FrameHead; // 【当前帧头】
    // ==================== 重发机制 ====================
    u32 retry_cnt;         // 【重发次数】重发计数
}ZC_UART;//uart state

// ======================== 缓冲区 =========================
//extern u8 receive_buf[512];   //single_package_buf
//extern u8 temp_buf[4*1024];   //ota_buf

typedef struct {
    u8 *buf;
    int size;
}UART_BUF;

#ifdef USE_UART_RECV
u8 RecvBuff[UART_RECV_BUFF_LEN];//Receive total buf
#endif
#ifdef USE_UART_SEND
u8 SendBuff[UART_SEND_BUFF_LEN];//Send total buf
#endif

extern u8 send_replycmd(char *FrameData);
extern int send_cmd_by_uart(unsigned char Type, unsigned char *data, unsigned short len, unsigned short Fuid, unsigned short total_num);
extern void uart_recv_task(char *buf,int len);
extern int zc_uart_init(void);
extern void zc_await_reply(void);
#endif
