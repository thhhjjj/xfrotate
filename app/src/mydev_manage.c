#include "mydev_manage.h"
#include "dev_control/servo_control.h"
#include "dev_control/led_control.h"
#include "dev_control/key_control.h"
#include "dev_control/infrared_control.h"
#include "user_define.h"
#include "typedef.h"
// device table
DEV_MANAGE dev_table_t[DEV_TABLE_MAX] = {
    DEV_ENTRY(char_dev, SERVO_DEV, servo_open, servo_read, servo_write, servo_ioctl, servo_release),
    DEV_ENTRY(char_dev, LED_DEV, led_open, led_read, led_write, led_ioctl, led_release),
    DEV_ENTRY(char_dev, KEY_DEV, key_open, key_read, key_write, key_ioctl, key_release),
    DEV_ENTRY(char_dev, INFRARED_DEV, infrared_open, infrared_read, infrared_write, infrared_ioctl, infrared_release),
};
//==================== stub functions ====================
void *dev_stub_open(void *arg)
{
#if DEV_DEBUG
    printf("WARN: dev_stub_open not implement!\r\n");
#endif
    return NULL;
}

void *dev_stub_read(void *dev)
{
#if DEV_DEBUG
    printf("WARN: dev_stub_read not implement!\r\n");
#endif
    return NULL;
}

void *dev_stub_write(void *dev, void *data, u32 len)
{
#if DEV_DEBUG
    printf("WARN: dev_stub_write not implement!\r\n");
#endif
    return NULL;
}

void *dev_stub_ioctl(void *dev, u32 cmd, u32 arg)
{
#if DEV_DEBUG
    printf("WARN: dev_stub_ioctl not implement!\r\n");
#endif
    return NULL;
}

void *dev_stub_release(void *dev)
{
#if DEV_DEBUG
    printf("WARN: dev_stub_release not implement!\r\n");
#endif
    return NULL;
}

//==================== dev assist functions ====================
DEV_MANAGE *dev_find(GD_T *gd, TYPE_U type, ID_U id)
{
    if (gd == NULL || gd->dev_table == NULL)
        return NULL;

    DEV_MANAGE *p = gd->dev_table;
    DEV_MANAGE *p_end = gd->dev_table + DEV_TABLE_MAX;

    while (p < p_end)
    {
        // skip empty slot
        if (p->dev_type == DEV_EMPTY_FLAG && p->dev_id == DEV_EMPTY_FLAG){
            p++;
            continue;
        }
        // match type and id
        if (p->dev_type == (TYPE_U)type && p->dev_id == (ID_U)id)
            return p;

        p++;
    }
    return NULL;
}

DEV_MANAGE *dev_dyn_reg(GD_T *gd, TYPE_U type, ID_U id,
                       void *(*open)(void *arg), void *(*read)(void *dev),
                       void *(*write)(void *dev, void *data, u32 len),
                       void *(*ioctl)(void *dev, u32 cmd, u32 arg), void *(*release)(void *dev))
{
    if (gd == NULL || gd->dev_table == NULL)
        return NULL;

    //find if already exist ,return exist
    DEV_MANAGE *tmp = dev_find(gd, type, id);
    if (tmp != NULL)
        return tmp;

    DEV_MANAGE *p = gd->dev_table;
    DEV_MANAGE *p_end = gd->dev_table + DEV_TABLE_MAX;
    // find empty slot
    while (p < p_end)
    {
        if (p->dev_type == DEV_EMPTY_FLAG && p->dev_id == DEV_EMPTY_FLAG)
            break;
        p++;
    }

    // full, no empty slot return NULL
    if (p >= p_end)
        return NULL;

    // register new device
    p->dev_type    = (TYPE_U)type;
    p->dev_id      = (ID_U)id;
    p->dev_open    = open    ? open    : dev_stub_open;
    p->dev_read    = read    ? read    : dev_stub_read;
    p->dev_write   = write   ? write   : dev_stub_write;
    p->dev_ioctl   = ioctl   ? ioctl   : dev_stub_ioctl;
    p->dev_release = release ? release : dev_stub_release;

    return p;
}

DEV_MANAGE *dev_change(GD_T *gd, TYPE_U type, ID_U id,
                       void *(*open)(void *arg),
                       void *(*read)(void *dev),
                       void *(*write)(void *dev, void *data, u32 len),
                       void *(*ioctl)(void *dev, u32 cmd, u32 arg),
                       void *(*release)(void *dev))
{
    if (gd == NULL || gd->dev_table == NULL)
        return NULL;

    DEV_MANAGE *entry = dev_find(gd, type, id);
    if (entry == NULL)
        return NULL;

    // change the function pointers if provided
    if (open)    entry->dev_open    = open;
    if (read)    entry->dev_read    = read;
    if (write)   entry->dev_write   = write;
    if (ioctl)   entry->dev_ioctl   = ioctl;
    if (release) entry->dev_release = release;

    return entry;
}
