#ifndef _MYDEV_MANAGE_H
#define _MYDEV_MANAGE_H
#include <stdint.h>
#include "user_define.h"
void *dev_stub_open(void *arg);
void *dev_stub_read(void *dev);
void *dev_stub_write(void *dev, void *data, u32 len);
void *dev_stub_ioctl(void *dev, u32 cmd, u32 arg);
void *dev_stub_release(void *dev);

#define DEV_ENTRY(type, id, op, rd, wr, ctl, rel)         \
{                                                         \
    .dev_type    = (TYPE_U)(type),                        \
    .dev_id      = (ID_U)(id),                            \
    .dev_open    = (op)  ? (op)  : dev_stub_open,         \
    .dev_read    = (rd)  ? (rd)  : dev_stub_read,         \
    .dev_write   = (wr)  ? (wr)  : dev_stub_write,        \
    .dev_ioctl   = (ctl) ? (ctl) : dev_stub_ioctl,        \
    .dev_release = (rel) ? (rel) : dev_stub_release       \
}

// API
extern DEV_MANAGE dev_table_t[DEV_TABLE_MAX];
extern DEV_MANAGE *dev_find(GD_T *gd, TYPE_U type, ID_U id);
extern DEV_MANAGE *dev_dyn_reg(GD_T *gd, TYPE_U type, ID_U id,
                       void *(*open)(void *arg), void *(*read)(void *dev),
                       void *(*write)(void *dev, void *data, u32 len),
                       void *(*ioctl)(void *dev, u32 cmd, u32 arg),
                       void *(*release)(void *dev));
extern DEV_MANAGE *dev_change(GD_T *gd, TYPE_U type, ID_U id,
                       void *(*open)(void *arg),
                       void *(*read)(void *dev),
                       void *(*write)(void *dev, void *data, u32 len),
                       void *(*ioctl)(void *dev, u32 cmd, u32 arg),
                       void *(*release)(void *dev));             
#endif
