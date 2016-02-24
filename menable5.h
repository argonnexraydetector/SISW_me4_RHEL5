/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#ifndef MENABLE5_H
#define MENABLE5_H

#include <linux/sched.h>

#include "menable.h"

#define ME5_NUMDMA	(0x002 * 4)
#define ME5_IRQSTATUS	(0x100 * 4)
#define ME5_IRQACK	(0x102 * 4)
#define ME5_IRQTYPE	(0x104 * 4)
#define ME5_IRQENABLE	(0x106 * 4)

#define ME5_DMAOFFS	(0x110 * 4)
#define ME5_DMASZ	(0x010 * 4)

#define ME5_DMACTRL	(0x000 * 4)
#define ME5_DMAADDR	(0x002 * 4)
#define ME5_DMAACTIVE	(0x004 * 4)
#define ME5_DMALENGTH	(0x006 * 4)
#define ME5_DMACOUNT	(0x008 * 4)
#define ME5_DMAMAXLEN	(0x00a * 4)
#define ME5_DMATYPE	(0x00c * 4)
#define ME5_DMATAG	(0x00e * 4)

#define ME5_IRQQUEUE	(0x080 * 4)
#define ME5_IRQQ_LOW	16
#define ME5_IRQQ_HIGH	29

#define ME5_FULLOFFSET     (0x2000 * 4)
#define ME5_IFCONTROL      (0x0004 * 4)
#define ME5_UIQCNT         (0x0006 * 4)
#define ME5_FIRSTUIQ       (0x0007 * 4)
#define ME5_FPGACONTROL    (0x1010 * 4)
#define ME5_PCIECONFIG0    (0x0010 * 4)
#define ME5_PCIECONFIGMAX  (0x001f * 4)

#define ME5_LED_CONTROL                 (0x0012 * 4)
#define LED_W_CONTROL_ENABLE_MASK       0x1
#define LED_W_VALUE_MASK                0x1E
#define LED_R_REGISTER_VALID            0x1
#define LED_R_PRESENT_MASK              0x1E
#define LED_R_CONTROL_ENABLED_MASK      0x10000
#define LED_R_VALUE_MASK                0x1E0000

#define ME5_RECONFIGURE_CONTROL (0x0013 * 4)
#define RECONFIGURE_FLAG    0x1

/* set when IRQs from next higher FPGA are set */
#define ME5_CHAIN_MASK 0x40000000

#define ME5_DMA_FIFO_DEPTH	16LL

struct menable_uiq;

struct me5_sgl {
    __le64 addr[7];
    __le64 next;
} __attribute__ ((packed));


extern struct class *menable_notify_class;

struct me5_data {
    struct siso_menable * men;
    struct menable_dmabuf dummybuf;
    void *dummypage;
    uint32_t irq_wanted;	                /* protected by irqmask_lock */
    uint32_t design_crc;
    char design_name[65];	                /* user supplied name of design (for IPC) */
    spinlock_t irqmask_lock;	                /* to protect IRQ enable register */

    // Notifications
    spinlock_t notify_lock;                     /* to protect notification register */
    uint32_t notify_wait_count;                 /* completions waiting for a notification */
    unsigned long notification;
    unsigned long notify_time_stamp;            /* Current notification time stamp */
    uint32_t fired_alarms;
    struct list_head notify_handler_heads;     /* all buffer heads */
    unsigned int notify_handler_cnt;
    spinlock_t notify_handler_headlock;

    // Temperature Alarm
    struct work_struct temperature_alarm_work;  /* called when temperature alarm is set */
    struct delayed_work temperature_alarm_timer_work;   /* called 1 second later after temperature alarm is set */
    unsigned int temperatureAlarmPeriod;        /* Minimum period between two consecutive temperature alarms */
};

#define NOTIFICATION_DRIVER_CLOSED  0x01
#define NOTIFICATION_DEVICE_REMOVED 0x02
#define NOTIFICATION_DEVICE_ADDED   0x04
#define NOTIFICATION_DEVICE_ALARM   0x08

struct me5_notify_handler {
    struct list_head node;          /* entry in notify_handler heads *_list */
    pid_t pid;                      /* process id */
    pid_t ppid;                     /* parent process id */
    unsigned long notify_time_stamp;
    struct completion notification_available;
    int quit_requested;
};

//Alarms
#define INT_ME5_ALARMS_COUNT                7
#define INT_MASK_ALARMS                     0xBF00

#define INT_MASK_TEMPERATURE_ALARM          0x8000
#define INT_MASK_POE                        0x2000
#define INT_MASK_PHY_MANAGEMENT             0x1000
#define INT_MASK_ACTION_CMD_LOST_CHAN_0     0x0100
#define INT_MASK_ACTION_CMD_LOST_CHAN_1     0x0200
#define INT_MASK_ACTION_CMD_LOST_CHAN_2     0x0400
#define INT_MASK_ACTION_CMD_LOST_CHAN_3     0x0800
#define INT_MASK_ACTION_CMD_LOST            0x0F00

#if 0 /* BITS_PER_LONG > 32 */
#define w64(v, a) writeq(cpu_to_le64(v), a)
#else
static inline void
w64(uint64_t v, void __iomem *a)
{
    iowrite32((unsigned int)(v & 0xffffffff), a);
    wmb();
    iowrite32((unsigned int)((v >> 32) & 0xffffffff), a + 4);
}
#endif

#endif
