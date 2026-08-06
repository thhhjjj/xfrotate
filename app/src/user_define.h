#ifndef _USER_DEFINE_H
#define _USER_DEFINE_H
//=================================================================device define================================================
typedef uint32_t u32;
typedef uint32_t TYPE_U;
typedef uint32_t ID_U;
// ID
enum {
    SERVO_DEV,
    LED_DEV,
    KEY_DEV,
    INFRARED_DEV,
    DEV_MAX
};
// TYPE
enum {
    char_dev,
    block_dev,
    net_dev,
};

#define DEV_DEBUG        0
#define DEV_TABLE_MAX    DEV_MAX
#define DEV_EMPTY_FLAG   0xFFFFFFFFU
typedef struct _dev_manage {
    TYPE_U dev_type;
    ID_U   dev_id;

    void *(*dev_open)(void *arg);
    void *(*dev_read)(void *dev);
    void *(*dev_write)(void *dev, void *data, u32 len);
    void *(*dev_ioctl)(void *dev, u32 cmd, u32 arg);
    void *(*dev_release)(void *dev);
}DEV_MANAGE;
//==================================================================================================================================
typedef struct {
    char *version;
    char *uuid;
    char *sn;
}SAVE_DATA;

typedef struct{
    DEV_MANAGE *dev_table;
    SAVE_DATA save_data;
}GD_T;

extern GD_T gd;
extern void user_init(void);
extern void user_deinit(void);
#endif
