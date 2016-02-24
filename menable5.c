/************   ************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
//!!! added tmd
 #include "list.h"


#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <stdbool.h>
#include "menable5.h"
#include "menable.h"
#include "menable_ioctl.h"
#include "uiq.h"

#include "linux_version.h"

static DEVICE_ATTR(design_crc, 0660, men_get_des_val, men_set_des_val);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
static inline void reinit_completion(struct completion *x)
{
    INIT_COMPLETION(*x);
}
#endif

static ssize_t
men_get_boardinfo(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct siso_menable *men = container_of(dev, struct siso_menable, dev);
    ssize_t ret = 0;
    int i;

    for (i = 0; i < 4; i++) {
        u32 tmp = ioread32(men->runtime_base + 4 * i);
        ssize_t r = sprintf(buf + ret, "0x%08x\n", tmp);

        if (r < 0) {
            return r;
        }

        ret += r;
    }

    return ret;
}

static DEVICE_ATTR(board_info, 0440, men_get_boardinfo, NULL);

/**
 * me5_add_uiqs - scan for new UIQs in one FPGA
 * @men: board to scan
 * @fpga: fpga index to scan
 * @count: how many UIQs to add
 * @uiqoffs: register offset of the first UIQ
 *
 * This will keep all UIQs from lower FPGAs.
 */
static int
me5_add_uiqs(struct siso_menable *men, const unsigned int fpga,
        const unsigned int count, const unsigned int uiqoffs)
{
    struct menable_uiq **nuiqs;
    unsigned int i;
    int ret;
    uint32_t uiqtype = ioread32(men->runtime_base + ME5_IRQTYPE);

    WARN_ON(!men->design_changing);
    for (i = fpga; i < MAX_FPGAS; i++) {
        WARN_ON(men->uiqcnt[i] != 0);
    }

    nuiqs = kcalloc(fpga * MEN_MAX_UIQ_PER_FPGA + count, sizeof(*men->uiqs),
            GFP_KERNEL);

    if (nuiqs == NULL) {
        return -ENOMEM;
    }

    for (i = 0; i < count; i++) {
        struct menable_uiq *uiq;
        void __iomem *basereg = men->runtime_base + uiqoffs;
        int chan = fpga * MEN_MAX_UIQ_PER_FPGA + i;
        int t = uiqtype & (1 << (i + ME5_IRQQ_LOW));

        uiq = men_uiq_init(chan, basereg + 8 * i, men, t, 16);

        if (IS_ERR(uiq)) {
            int j;
            ret = PTR_ERR(uiq);

            for (j = fpga * MEN_MAX_UIQ_PER_FPGA; j < chan; j++) {
                men_uiq_remove(nuiqs[j]);
            }

            kfree(nuiqs);
            return ret;
        }

        uiq->irqack = men->runtime_base + ME5_IRQACK;
        uiq->ackbit = i + ME5_IRQQ_LOW;

        nuiqs[chan] = uiq;
    }

    kfree(men->uiqs);
    men->uiqs = nuiqs;
    men->uiqcnt[fpga] = count;

    return 0;
}

static void
acknowledge_alarm (struct siso_menable *men, unsigned long alarm)
{
    unsigned long flags = 0;

    //Acknowledge and re-enable the interrupt
    spin_lock_irqsave(&men->d5->notify_lock, flags);

    men->d5->fired_alarms &= ~alarm;
    men->d5->irq_wanted |= alarm;
    iowrite32(alarm, men->runtime_base + ME5_IRQACK);
    if (men->d5->fired_alarms == 0) {
        men->d5->notification &= ~NOTIFICATION_DEVICE_ALARM;
    }
    iowrite32(men->d5->irq_wanted, men->runtime_base + ME5_IRQENABLE);

    printk(KERN_INFO "[%d]: Alarm 0x%lX is acknowledged (Fired Alarm = 0x%X, irq_wanted = 0x%X)\n",
            current->pid, alarm, men->d5->fired_alarms, men->d5->irq_wanted);

    spin_unlock_irqrestore(&men->d5->notify_lock, flags);
}

/**
 * men5_reset_vlink - reset link between bridge FPGA and upper FPGA
 * @men: board to reset
 * @upper: reset upper FPGA part or not
 * returns: 0 on success, error code else
 */
static int
men5_reset_vlink(struct siso_menable *men, const bool upper)
{
    return 0;
}

struct me5_notify_handler *
me5_create_notify_handler (struct siso_menable *men)
{
    struct me5_notify_handler *bh;

    bh = kzalloc(sizeof(*bh), GFP_USER);
    if (bh == NULL) {
        return NULL;
    }

    INIT_LIST_HEAD(&bh->node);
    spin_lock(&men->d5->notify_handler_headlock);

    men->d5->notify_handler_cnt++;
    bh->pid = current->pid;
    bh->ppid = current->parent->pid;
    bh->quit_requested = 0;
    bh->notify_time_stamp = men->d5->notify_time_stamp;
    init_completion(&bh->notification_available);
    list_add_tail(&bh->node, &men->d5->notify_handler_heads);

    spin_unlock(&men->d5->notify_handler_headlock);
    printk(KERN_INFO "[%d %d]: Notify Handler is created\n", bh->ppid, bh->pid);

    return bh;
}

void
me5_free_notify_handler(struct siso_menable *men, struct me5_notify_handler *bh)
{
    BUG_ON(in_interrupt());

    printk(KERN_INFO "[%d %d]: Notify Handler is removed \n", bh->ppid, bh->pid);

    spin_lock(&men->d5->notify_handler_headlock);
    men->d5->notify_handler_cnt--;
    __list_del_entry(&bh->node);
    spin_unlock(&men->d5->notify_handler_headlock);
    kfree(bh);
}

struct me5_notify_handler *
me5_get_notify_handler(struct siso_menable *men, const unsigned int ppid, const unsigned int pid)
{
    struct me5_notify_handler *res;
    //printk(KERN_INFO "[%d %d]: Getting notify_handler\n", ppid, pid);

    spin_lock(&men->d5->notify_handler_headlock);

    list_for_each_entry(res, &men->d5->notify_handler_heads, node)
    {
        if ((res->pid == pid) && (res->ppid == ppid)) {
            spin_unlock(&men->d5->notify_handler_headlock);
            return res;
        }
    }
    spin_unlock(&men->d5->notify_handler_headlock);

    //printk(KERN_INFO "[%d %d]: No notify_handler found\n", ppid, pid);
    return NULL;
}

static int
men5_ioctl(struct siso_menable *men, const unsigned int cmd,
        const unsigned int size, unsigned long arg)
{
    unsigned long flags = 0;

    switch (cmd) {
    case IOCTL_BOARD_INFO:
        {
            unsigned int a[4];
            int i;

            if (size != sizeof(a)) {
                warn_wrong_iosize(men, cmd, sizeof(a));
                return -EINVAL;
            }

            for (i = 0; i < ARRAY_SIZE(a); i++) {
                a[i] = ioread32(men->runtime_base + 4 * i);
            }
            if (copy_to_user((void __user *) arg,
                    a, sizeof(a))) {
                return -EFAULT;
            }
            return 0;
        }
    case IOCTL_PP_CONTROL:
        {
            int ret = 0;
            unsigned long flags = 0;

            if (size != 0) {
                warn_wrong_iosize(men, cmd, 0);
                return -EINVAL;
            }

            spin_lock_irqsave(&men->designlock, flags);
            if (men->design_changing) {
                spin_unlock_irqrestore(&men->designlock, flags);
                return -EBUSY;
            }

            men->design_changing = true;
            spin_unlock_irqrestore(&men->designlock, flags);

            switch (arg) {
            case 0:
            case 1:
                men->active_fpgas = 1;
                break;
            default:
                ret = -EINVAL;
            }

            spin_lock_irqsave(&men->designlock, flags);
            men->design_changing = false;

            spin_unlock_irqrestore(&men->designlock, flags);
            return ret;
        }
    case IOCTL_RESSOURCE_CONTROL:
    case IOCTL_GET_EEPROM_DATA:
    case IOCTL_DESIGN_SETTINGS:
        return -EINVAL;

    case IOCTL_EX_DEVICE_CONTROL:
        {
            struct men_device_control_i ctrl;
            struct men_device_control_o_v2 reply;
            unsigned long leds;
            struct me5_notify_handler * handler;
            int result = 0;
            long wakeup_time = msecs_to_jiffies(250);

            if (size < sizeof(ctrl)) {
                warn_wrong_iosize(men, cmd, sizeof(ctrl));
                return -EINVAL;
            }

            if (copy_from_user(&ctrl, (void __user *) arg, sizeof(ctrl))) {
                return -EFAULT;
            }

            if (ctrl._size > size) {
                warn_wrong_iosize(men, cmd, sizeof(ctrl));
                return -EINVAL;
            }

            switch (ctrl.command) {
                // Not yet supported
//             case DEVCTRL_RECONFIGURE_FPGA:
//                 break;

            case DEVCTRL_SET_LEDS:
                iowrite32(ctrl.args.set_leds.led_status,
                        men->runtime_base + ME5_LED_CONTROL);
                //printk(KERN_INFO "[%d]: Set LEDs 0x%X\n", men->board, ctrl.args.set_leds.led_status);

                break;

            case DEVCTRL_GET_LEDS:
                if (size < sizeof(reply)) {
                    warn_wrong_iosize(men, cmd, sizeof(reply));
                    return -EINVAL;
                }

                leds = ioread32(men->runtime_base + ME5_LED_CONTROL);
                reply.args.get_leds.led_present = leds & 0xFFFF;
                reply.args.get_leds.led_status = (leds >> 16) & 0xFFFF;

                //printk(KERN_INFO "[%d]: Get LEDs 0x%lX\n", men->board, leds);

                if (copy_to_user((void __user *) arg, &reply, sizeof(reply))) {
                    return -EFAULT;
                }

                break;

            case DEVCTRL_GET_ASYNC_NOTIFY:

                if (size < sizeof(reply)) {
                    warn_wrong_iosize(men, cmd, sizeof(reply));
                    return -EINVAL;
                }

                handler = me5_get_notify_handler(men, current->parent->pid, current->pid);
                if (handler == NULL) {
                    handler = me5_create_notify_handler(men);
                    if (handler == NULL) {
                        return -ENOMEM;
                    }
                }

                spin_lock_irqsave(&men->d5->notify_lock, flags);

//                if (((men->d5->notification == DEVCTRL_ASYNC_NOTIFY_DEVICE_ALARM) && ((men->d5->fired_alarms & men->d5->irq_wanted) == 0))
//                        || (men->d5->notification == DEVCTRL_ASYNC_NO_NOTIFY)) {

                men->d5->notify_wait_count++;
                if ((!handler->quit_requested)
                        && ((handler->notify_time_stamp == men->d5->notify_time_stamp) || (!men->d5->notification))) {
                    printk(KERN_INFO "[%d %d]: Waiting for notifications (notify_wait_count = %d)\n", current->parent->pid, current->pid, men->d5->notify_wait_count);
                    // Wait until a notification is received
                    spin_unlock_irqrestore(&men->d5->notify_lock, flags);
                    wakeup_time = wait_for_completion_interruptible_timeout(&handler->notification_available, msecs_to_jiffies(250));
                    printk(KERN_INFO "[%d]: Waking up\n", current->pid);
                    spin_lock_irqsave(&men->d5->notify_lock, flags);
                }

                reply.args.get_async_event.pl = 0;
                reply.args.get_async_event.ph = 0;

                if (handler->quit_requested != 0) {
                    printk(KERN_INFO "[%d]: Received Quit Signal, quitting.\n", current->pid);

                    reply.args.get_async_event.event = DEVCTRL_ASYNC_NOTIFY_DRIVER_CLOSED;
                    if (copy_to_user((void __user *) arg, &reply, sizeof(reply))) {
                        return -EFAULT;
                    }
                    me5_free_notify_handler(men, handler);
                } else if (handler->notify_time_stamp != men->d5->notify_time_stamp) {

                    handler->notify_time_stamp = men->d5->notify_time_stamp;
                    // We might have received 10000 completions and just started
                    // to handle them now, so that we must reset the completion struct now.
                    // Otherwise, we will handle the same interrupt 10000 times.
                    reinit_completion(&handler->notification_available);

                    if (men->d5->notification) {
                        printk(KERN_INFO "[%d]: Notifications available (current time stamp = %ld; Hanlder time stamp = %ld; notifications = 0x%lX; alarms = 0x%X)\n",
                            current->pid, men->d5->notify_time_stamp, handler->notify_time_stamp,
                            men->d5->notification, men->d5->fired_alarms);
                        if (men->d5->notification & NOTIFICATION_DEVICE_REMOVED) {
                            reply.args.get_async_event.event = DEVCTRL_ASYNC_NOTIFY_DEVICE_REMOVED;
                        } else if (men->d5->notification & NOTIFICATION_DEVICE_ALARM) {
                            reply.args.get_async_event.event = DEVCTRL_ASYNC_NOTIFY_DEVICE_ALARM;
                            reply.args.get_async_event.pl |= (men->d5->fired_alarms & INT_MASK_TEMPERATURE_ALARM) ? DEVCTRL_DEVICE_ALARM_TEMPERATURE : 0x0;
                            reply.args.get_async_event.pl |= (men->d5->fired_alarms & INT_MASK_ACTION_CMD_LOST) >> 4;
                            reply.args.get_async_event.pl |= (men->d5->fired_alarms & INT_MASK_PHY_MANAGEMENT) ? DEVCTRL_DEVICE_ALARM_PHY : 0x0;
                            reply.args.get_async_event.pl |= (men->d5->fired_alarms & INT_MASK_POE) ? DEVCTRL_DEVICE_ALARM_POE : 0x0;
                        } else if (men->d5->notification & NOTIFICATION_DRIVER_CLOSED) {
                            reply.args.get_async_event.event = DEVCTRL_ASYNC_NOTIFY_DRIVER_CLOSED;
                        } else if (men->d5->notification & NOTIFICATION_DEVICE_ADDED) {
                            reply.args.get_async_event.event = DEVCTRL_ASYNC_NOTIFY_DEVICE_ADDED;
                        }
                    } else { // No notification happened, but only timestamp updated.
                        // No notification is availble, so that it's better to return timeout
                        printk(KERN_INFO "[%d]: NO Notifications available (current time stamp = %ld; Hanlder time stamp = %ld; notifications = 0x%lX; alarms = 0x%X)\n",
                            current->pid, men->d5->notify_time_stamp, handler->notify_time_stamp,
                            men->d5->notification, men->d5->fired_alarms);
                        result = -ETIMEDOUT;
                    }
                } else if (wakeup_time == 0) { // Timeout
                    printk(KERN_INFO "[%d]: Notify_Handler timed out.\n", current->pid);

                    result = -ETIMEDOUT;
                } else if (wakeup_time > 0) { // someone sent a complete, but we don't know who ! (Should never happen)
                    printk(KERN_INFO "[%d]: Unexpected wakeup!!!!!!\n", current->pid);

                    result = -ETIMEDOUT;
                } else { // Sleeping process is interrupted !
                    printk(KERN_INFO "[%d]: Interrupted!!!!!!\n", current->pid);

                    result = -EFAULT;
                }

                if (result == 0) {
                    if (copy_to_user((void __user *) arg, &reply, sizeof(reply))) {
                        return -EFAULT;
                    }
                }

                men->d5->notify_wait_count--;
                printk(KERN_INFO "[%d]: Notifications handled (notify_wait_count = %d)\n", current->pid, men->d5->notify_wait_count);

                spin_unlock_irqrestore(&men->d5->notify_lock, flags);

                break;

            case DEVCTRL_RESET_ASYNC_NOTIFY:

                //printk(KERN_INFO "[%d]: DEVCTRL_RESET_ASYNC_NOTIFY \n", men->board);
                if (ctrl.args.reset_async_event.event == DEVCTRL_ASYNC_NOTIFY_DRIVER_CLOSED) {
                    //printk(KERN_INFO "[%d %d %d]: Driver Closed Notification \n", current->real_parent->pid, current->parent->pid, current->pid);
                    if (ctrl.args.reset_async_event.ph == DEVCTRL_CLOSE_DRIVER_MAGIC) {
                        struct me5_notify_handler * notify_handler;

                        spin_lock_irqsave(&men->d5->notify_lock, flags);

                        //men->d5->notification |= NOTIFICATION_DRIVER_CLOSED;
                        //men->d5->notify_time_stamp++;

                        list_for_each_entry(notify_handler, &men->d5->notify_handler_heads, node)
                        {
                            if (current->parent->pid == notify_handler->ppid) {
                                notify_handler->quit_requested = 1;
                                complete(&notify_handler->notification_available);
                                //printk(KERN_INFO "[%d %d]: 1 Process is signaled\n", notify_handler->ppid, notify_handler->pid);
                            }
                        }

                        spin_unlock_irqrestore(&men->d5->notify_lock, flags);
                    }
                }
                else if (ctrl.args.reset_async_event.event == DEVCTRL_ASYNC_NOTIFY_DEVICE_ALARM) {
                    if (ctrl.args.reset_async_event.pl & DEVCTRL_DEVICE_ALARM_TEMPERATURE) {
                        if (ctrl.args.reset_async_event.ph != 0 &&
                                ctrl.args.reset_async_event.ph < LONG_MAX) {
                            men->d5->temperatureAlarmPeriod = ctrl.args.reset_async_event.ph;
                            acknowledge_alarm(men, INT_MASK_TEMPERATURE_ALARM);
                            printk(KERN_INFO "[%d]: Temperature alarm period set to %d ms\n", men->board, men->d5->temperatureAlarmPeriod);
                        }
                    }
                    if (ctrl.args.reset_async_event.pl & DEVCTRL_DEVICE_ALARM_PHY) {
                        acknowledge_alarm(men, INT_MASK_PHY_MANAGEMENT);
                        printk(KERN_INFO "[%d]: PHY interrupt is reset \n", men->board);
                    }
                    if (ctrl.args.reset_async_event.pl & DEVCTRL_DEVICE_ALARM_ACL) {
                        acknowledge_alarm(men, ((ctrl.args.reset_async_event.pl & DEVCTRL_DEVICE_ALARM_ACL) << 4) & INT_MASK_ACTION_CMD_LOST);
                        printk(KERN_INFO "[%d]: ACL interrupt is reset \n", men->board);
                    }
                    if (ctrl.args.reset_async_event.pl & DEVCTRL_DEVICE_ALARM_POE) {
                        acknowledge_alarm(men, INT_MASK_POE);
                        printk(KERN_INFO "[%d]: POE interrupt is reset \n", men->board);
                    }
                }

                break;

            default:
                printk(KERN_INFO "ENOSYS\n");
                return -ENOSYS;
            }
            return result;
        }
    default:
        return -ENOIOCTLCMD;
    }
}

static void
me5_free_sgl(struct siso_menable *men, struct menable_dmabuf *sb)
{
    struct men_dma_chain *res = sb->dmat;
    dma_addr_t dma = sb->dma;

    while (res) {
        struct men_dma_chain *n;
        dma_addr_t ndma;

        n = res->next;
        ndma = (dma_addr_t)(le64_to_cpu(res->pcie5->next) & ~(3ULL));
        if (dma == ndma) {
            break;
        }
        dma_pool_free(men->pool, res->pcie5, dma);
        kfree(res);
        res = n;
        dma = ndma;
    }
}

static void
me5_queue_sb(struct menable_dmachan *db, struct menable_dmabuf *sb)
{
    w64(sb->buf_length / 4, db->iobase + ME5_DMAMAXLEN);
    wmb();
    w64(sb->dma, db->iobase + ME5_DMAADDR);
    wmb();
}

#define to_delayed_work(_work)  container_of(_work, struct delayed_work, work)

static void
temperature_alarm_timer(struct work_struct *work)
{
    struct siso_menable *men;
    struct me5_data *men_d5;

    men_d5 = container_of(to_delayed_work(work), struct me5_data, temperature_alarm_timer_work);
    men = men_d5->men;

    acknowledge_alarm(men, INT_MASK_TEMPERATURE_ALARM);

//    //Acknowledge and re-enable the interrupt
//    spin_lock(&men->d5->notify_lock);
//
//    men_d5->fired_alarms &= ~TEMPERATURE_ALARM_MASK;
//    iowrite32(TEMPERATURE_ALARM_MASK, men->runtime_base + ME5_IRQACK);
//
//    men_d5->irq_wanted |= TEMPERATURE_ALARM_MASK;
//    iowrite32(men_d5->irq_wanted, men->runtime_base + ME5_IRQENABLE);
//
//    spin_unlock(&men->d5->notify_lock);
}

void
alarm_work(struct work_struct *work)
{
    struct siso_menable *men;
    struct me5_data *men_d5;

    men_d5 = container_of(work, struct me5_data, temperature_alarm_work);
    men = men_d5->men;

    schedule_delayed_work(&men_d5->temperature_alarm_timer_work, msecs_to_jiffies(men_d5->temperatureAlarmPeriod));
}

static irqreturn_t
me5_irq(int irq, void *dev_id)
{
    uint32_t sr;
    struct siso_menable *men = dev_id;
    struct timespec timeStamp;
    bool haveTimeStamp = false;

    int dma;
    uint32_t badmask = 0;
    uint32_t st; /* tmp status */

    if (pci_channel_offline(men->pdev)) {
        return IRQ_HANDLED;
    }

    sr = ioread32(men->runtime_base + ME5_IRQSTATUS);
    if (unlikely(sr == 0)) {
        return IRQ_NONE;
    }

    if (men->active_fpgas == 0) {
        return IRQ_HANDLED;
    }

    if (unlikely(sr == 0xffffffff)) {
        dev_warn(&men->dev, "IRQ status register %i read returned -1\n", 0);
        iowrite32(0, men->runtime_base + ME5_IRQENABLE);
        iowrite32(0xffffffff, men->runtime_base + ME5_IRQACK);
        return IRQ_HANDLED;
    }

    spin_lock(&men->d5->irqmask_lock);
    badmask = sr & ~men->d5->irq_wanted;
    if (unlikely(badmask != 0)) {
        iowrite32(men->d5->irq_wanted, men->runtime_base + ME5_IRQENABLE);
        iowrite32(badmask, men->runtime_base + ME5_IRQACK);
        sr &= men->d5->irq_wanted;
    }
    spin_unlock(&men->d5->irqmask_lock);

    /* Handle DMA Interrupts */
    for (dma = 0; dma < men->dmacnt[0]; dma++) {
        struct menable_dmachan *db;
        void __iomem *dmabase;
        void __iomem *lenaddr;
        void __iomem *tagaddr;
        struct menable_dma_wait *waitstr;
        uint32_t ic, delta;
        int i;

        if ((sr & (0x1 << dma)) == 0) {
            continue;
        }

        if (!haveTimeStamp) {
            haveTimeStamp = true;
            timeStamp = current_kernel_time();
        }

        db = men_dma_channel(men, dma);
        BUG_ON(db == NULL);
        dmabase = db->iobase;
        lenaddr = dmabase + ME5_DMALENGTH;
        tagaddr = dmabase + ME5_DMATAG;

        spin_lock(&db->chanlock);
        iowrite32(1 << db->ackbit, db->irqack);
        ic = ioread32(dmabase + ME5_DMACOUNT);
        spin_lock(&db->listlock);
        if (unlikely(db->active == NULL)) {
            for (i = ic - db->imgcnt; i > 0; i--) {
                uint32_t tmp = ioread32(lenaddr);
                tmp = ioread32(tagaddr);
                db->lost++;
            }
            spin_unlock(&db->listlock);
            spin_unlock(&db->chanlock);
            continue;
        }

        delta = ic - db->imgcnt;
        for (i = delta; i > 0; i--) {
            struct menable_dmabuf *sb = men_move_hot(db, &timeStamp);
            uint32_t len = ioread32(lenaddr);
            uint32_t tag = ioread32(tagaddr);

            if (unlikely(sb != NULL)) {
                sb->dma_length = len;
                sb->dma_tag = tag;
            }
        }

        list_for_each_entry(waitstr, &db->wait_list, node) {
            if (waitstr->frame <= db->goodcnt)
                complete(&waitstr->cpl);
        }

        if (likely(db->transfer_todo > 0)) {
            unsigned int sbcnt = min(ME5_DMA_FIFO_DEPTH - db->hot,
                    db->transfer_todo - db->hot);

            if (delta)
                me_queue_dma(db, sbcnt);
            spin_unlock(&db->listlock);
            mod_timer(&db->timer, jiffies + db->timeout);
        } else {
            spin_unlock(&db->listlock);
            db->running = 3;
            schedule_work(&db->dwork);
        }
        spin_unlock(&db->chanlock);
    }

    /* Handle User Queue IRQ */
    st = (sr & 0x3fff0000);
    if (st != 0) {
        uint32_t bit;
        for (bit = ME5_IRQQ_LOW; (bit <= ME5_IRQQ_HIGH) && st; bit++) {
            if (st & (1 << bit)) {
                uiq_irq(men->uiqs[bit - ME5_IRQQ_LOW], &timeStamp,
                        &haveTimeStamp);
                st ^= (1 << bit);
            }
        }
    }


    /* Handle Alarms*/
    st = (sr & INT_MASK_ALARMS);
    if (st & men->d5->irq_wanted) {
        struct me5_notify_handler *notify_handler;
        printk(KERN_INFO "[%i]: (Fired Alarm = 0x%X, irq_wanted = 0x%X)\n", men->board, men->d5->fired_alarms, men->d5->irq_wanted);
        printk(KERN_INFO "[%i]: Alarm(s) received (IRQ_Status = 0x%X, IRQ_Wanted = 0x%X)\n", men->board, st, men->d5->irq_wanted);

        spin_lock(&men->d5->notify_lock);

        men->d5->notification |= NOTIFICATION_DEVICE_ALARM;
        men->d5->fired_alarms |= st & men->d5->irq_wanted;
        men->d5->irq_wanted &= ~men->d5->fired_alarms;
        men->d5->notify_time_stamp++;

        printk(KERN_INFO "[%i]: -> (Fired Alarm = 0x%X, irq_wanted = 0x%X)\n", men->board, men->d5->fired_alarms, men->d5->irq_wanted);

        // Disable all alarms until they are handled
        iowrite32(men->d5->irq_wanted, men->runtime_base + ME5_IRQENABLE);

        /* Handle Temperature alarm */
        if (men->d5->fired_alarms & INT_MASK_TEMPERATURE_ALARM) {
            // Acknowledge & re-enable Temperature alarm after 1 second
            // (it might be already handled within this time)
            schedule_work(&men->d5->temperature_alarm_work);
        }

        list_for_each_entry(notify_handler, &men->d5->notify_handler_heads, node)
        {
            complete(&notify_handler->notification_available);
        }

        spin_unlock(&men->d5->notify_lock);
    }

    return IRQ_HANDLED;
}

static void
men5_abort_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
    iowrite32(2, dc->iobase + ME5_DMACTRL);
    wmb();
    iowrite32(0, dc->iobase + ME5_DMACTRL);
    wmb();
}

static void
men5_stop_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
    uint32_t irqreg;
    unsigned long flags;

    irqreg = ioread32(dc->irqenable);
    irqreg &= ~(1 << dc->enablebit);
    iowrite32(irqreg, dc->irqenable);

    iowrite32(0, dc->iobase + ME5_DMACTRL);
    wmb();

    spin_lock_irqsave(&men->d5->irqmask_lock, flags);
    men->d5->irq_wanted &= ~(1 << dc->enablebit);
    spin_unlock_irqrestore(&men->d5->irqmask_lock, flags);
}

static int
me5_create_userbuf(struct siso_menable *men, struct menable_dmabuf *db)
{
    struct men_dma_chain *cur;
    int i;

    db->dmat->pcie5 = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
    if (!db->dmat->pcie5)
        goto fail_pcie;
    memset(db->dmat->pcie5, 0, sizeof(*db->dmat->pcie5));

    cur = db->dmat;

    for (i = 0; i < db->nents; i++) {
        int idx = i % ARRAY_SIZE(cur->pcie5->addr);

        cur->pcie5->addr[idx] = cpu_to_le64(sg_dma_address(db->sg + i) + 0x1);

        if ((idx == ARRAY_SIZE(cur->pcie5->addr) - 1) && (i + 1 < db->nents)) {
            dma_addr_t next;

            cur->next = kzalloc(sizeof(*cur->next), GFP_USER);
            if (!cur->next)
                goto fail;

            cur->next->pcie5 = dma_pool_alloc(men->pool, GFP_USER, &next);
            if (!cur->next->pcie5) {
                kfree(cur->next);
                cur->next = NULL;
                goto fail;
            }
            cur->pcie5->next = cpu_to_le64(next + 0x2);
            cur = cur->next;
            memset(cur->pcie5, 0, sizeof(*cur->pcie5));
        }
    }
    cur->pcie5->next = men->d5->dummybuf.dmat->pcie5->next;

    return 0;
fail:
    me5_free_sgl(men, db);
    return -ENOMEM;
fail_pcie:
    kfree(db->dmat);
    return -ENOMEM;
}

static int
men5_create_dummybuf(struct siso_menable *men)
{
    struct men_dma_chain *cur;
    struct menable_dmabuf *db = &men->d5->dummybuf;
    int i;
    dma_addr_t pagedma;

    db->index = -1;
    db->dmat = kzalloc(sizeof(*db->dmat), GFP_KERNEL);
    if (!db->dmat) {
        goto fail_dmat;
    }

    db->dmat->pcie5 = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
    if (!db->dmat->pcie5) {
        goto fail_pcie;
    }
    memset(db->dmat->pcie5, 0, sizeof(*db->dmat->pcie5));

    men->d5->dummypage = pci_alloc_consistent(men->pdev, 4096, &pagedma);
    if (men->d5->dummypage == NULL) {
        goto fail_page;
    }

    cur = db->dmat;

    for (i = 0; i < ARRAY_SIZE(cur->pcie5->addr); i++) {
        cur->pcie5->addr[i] = cpu_to_le64(pagedma + 0x1);
    }

    cur->pcie5->next = cpu_to_le64(db->dma + 0x2);

    db->buf_length = -1;

    return 0;
fail_page:
    dma_pool_free(men->pool, db->dmat->pcie5, db->dma);
fail_pcie:
    kfree(db->dmat);
fail_dmat:
    return -ENOMEM;
}

static void
men5_destroy_dummybuf(struct siso_menable *men)
{
    uint64_t pg = le64_to_cpu(men->d5->dummybuf.dmat->pcie5->addr[0]) & ~(3ULL);
    dma_addr_t dmaaddr = (dma_addr_t) pg;

    pci_free_consistent(men->pdev, 4096, men->d5->dummypage, dmaaddr);
    dma_pool_free(men->pool, men->d5->dummybuf.dmat->pcie5,
            men->d5->dummybuf.dma);
    kfree(men->d5->dummybuf.dmat);
}

static void
me5_exit(struct siso_menable *men)
{
    men5_destroy_dummybuf(men);
    kfree(men->uiqs);
    kfree(men->d5);
}

static unsigned int
me5_query_dma(struct siso_menable *men, const unsigned int arg)
{
    uint32_t u;

    BUG_ON(men->active_fpgas <= 0);

    u = ioread32(men->runtime_base + ME5_NUMDMA);
    if (unlikely(u == 0xffffffff)) {
        dev_warn(&men->dev, "Reading DMACNT from FPGA %i failed\n", 0);
        u = 0;
    } else {
        dev_dbg(&men->dev, "%i DMA channels detected in FPGA %i\n", u, 0);
    }

    return u;
}

static int
men5_startdma(struct menable_dmachan *dmac)
{
    uint32_t tmp, dir;
    unsigned long flags;
    struct siso_menable *men = dmac->parent;

    men5_abort_dma(men, dmac);

    dir = (dmac->direction == PCI_DMA_TODEVICE) ? 2 : 1;

    tmp = ioread32(dmac->iobase + ME5_DMATYPE);
    if (!(tmp & dir)) {
        return -EACCES;
    }
    iowrite32(dir, dmac->iobase + ME5_DMATYPE);

    /* clear IRQ */
    iowrite32(1 << dmac->ackbit, dmac->irqack);

    dmac->imgcnt = ioread32(dmac->iobase + ME5_DMACOUNT);

    me_queue_dma(dmac, min(dmac->transfer_todo, ME5_DMA_FIFO_DEPTH));

    spin_lock_irqsave(&men->d5->irqmask_lock, flags);
    men->d5->irq_wanted |= (1 << dmac->enablebit);
    iowrite32(men->d5->irq_wanted, dmac->irqenable);

    if ((men->d5->irq_wanted & ME5_CHAIN_MASK) == 0) {
        men->d5->irq_wanted |= ME5_CHAIN_MASK;
        iowrite32(men->d5->irq_wanted, men->runtime_base + ME5_IRQENABLE);
    }

    spin_unlock_irqrestore(&men->d5->irqmask_lock, flags);

    iowrite32(1, dmac->iobase + ME5_DMAACTIVE);
    ioread32(dmac->iobase + ME5_DMAACTIVE);
    iowrite32(0, dmac->iobase + ME5_DMAACTIVE);
    ioread32(dmac->iobase + ME5_DMAACTIVE);
    iowrite32(1, dmac->iobase + ME5_DMACTRL);
    ioread32(dmac->iobase + ME5_DMACTRL);

    return 0;
}

static void
men5_dmabase(struct siso_menable *men, struct menable_dmachan *dc)
{
    void __iomem *addrbase = men->runtime_base;
    unsigned int skipdma = 0;
    unsigned int i;

    for (i = 0; i < dc->fpga; i++) {
        skipdma += men->dmacnt[i];
    }

    dc->ackbit = dc->number - skipdma;

    dc->iobase = addrbase + ME5_DMAOFFS + ME5_DMASZ * dc->ackbit;
    dc->irqack = addrbase + ME5_IRQACK;
    dc->irqenable = addrbase + ME5_IRQENABLE;
    dc->enablebit = dc->ackbit;
}

static void
men5_stopirq(struct siso_menable *men)
{
    unsigned int i;

    if (men->active_fpgas > 0) {
        void __iomem *addrbase = men->runtime_base;

        iowrite32(0, addrbase + ME5_IRQENABLE);
        iowrite32(0xffffffff, addrbase + ME5_IRQACK);
        iowrite32(0, addrbase + ME5_IFCONTROL);
    }
    men->active_fpgas = 1;

    for (i = 0; i < men->uiqcnt[0]; i++) {
        if (men->uiqs[i] == NULL) {
            continue;
        }
        men->uiqs[i]->running = false;
    }

    //complete_all(&men->d5->notification_available);
}

static void
men5_startirq(struct siso_menable *men)
{
    // Enable IRQs
    uint32_t mask = ((1 << men->uiqcnt[0]) - 1) << ME5_IRQQ_LOW;

    // Enable all Notifications
    mask |= INT_MASK_ALARMS;

    men->d5->irq_wanted = mask;

    iowrite32(0xffffffff, men->runtime_base + ME5_IRQACK);
    iowrite32(mask, men->runtime_base + ME5_IRQENABLE);
}

/**
 * men5_reset_core - reset state machines near the PCIe core
 * @men: board to reset
 *
 * This will reset the state machines and logic directly connected to the
 * PCIe core.
 */
static void
men5_reset_core(struct siso_menable *men)
{
    int i;

    iowrite32(0xa, men->runtime_base + ME5_IFCONTROL);
    iowrite32(0xa, men->runtime_base + ME5_IFCONTROL);
    for (i = 0; i < 4; i++) {
        iowrite32(0x8, men->runtime_base + ME5_IFCONTROL);
    }
    for (i = 0; i < 6; i++) {
        iowrite32(0, men->runtime_base + ME5_IFCONTROL);
    }

    men5_reset_vlink(men, false);
}

static void
me5_cleanup(struct siso_menable *men)
{
    unsigned long flags;

    spin_lock_irqsave(&men->designlock, flags);
    men->design_changing = true;
    spin_unlock_irqrestore(&men->designlock, flags);

    men_del_uiqs(men, 1);
    memset(men->desname, 0, men->deslen);

    spin_lock_irqsave(&men->designlock, flags);
    men->design_changing = false;
    spin_unlock_irqrestore(&men->designlock, flags);
}

static struct menable_dmabuf *
me5_dummybuf(struct menable_dmachan *dc)
{
    return &dc->parent->d5->dummybuf;
}

static struct lock_class_key me5_irqmask_lock;

static struct attribute *me5_attributes[6] = { &dev_attr_design_crc.attr,
        &dev_attr_board_info.attr, NULL };

static struct attribute_group me5_attribute_group = { .attrs = me5_attributes };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
const
#endif /* LINUX >= 2.6.31 */
struct attribute_group *me5_attribute_groups[2] = {
    &me5_attribute_group,
    NULL
};

int
me5_probe(struct siso_menable *men)
{
    int ret = -ENOMEM;
    unsigned int uiqoffs;
    unsigned int uiqcnt;
    unsigned long flags = 0;

    men->d5 = kzalloc(sizeof(*men->d5), GFP_KERNEL);
    if (men->d5 == NULL) {
        goto fail;
    }

    men->d5->men = men;

    spin_lock_init(&men->d5->irqmask_lock);
    lockdep_set_class(&men->d5->irqmask_lock, &me5_irqmask_lock);

    spin_lock_init(&men->d5->notify_lock);
    spin_lock_init(&men->d5->notify_handler_headlock);

    men->active_fpgas = 1;

    men5_reset_core(men);
    men5_stopirq(men);

    if (pci_set_dma_mask(men->pdev, DMA_BIT_MASK(64))) {
        dev_err(&men->dev, "No suitable DMA available.\n");
        goto fail_mask;
    }
    pci_set_consistent_dma_mask(men->pdev, DMA_BIT_MASK(64));
    men->pool = dmam_pool_create("me5_sgl", &men->pdev->dev,
            sizeof(struct me5_sgl), 128, 4096);
    if (!men->pool) {
        dev_err(&men->dev, "can not allocate DMA pool\n");
        goto fail_pool;
    }

    ret = men5_create_dummybuf(men);
    if (ret) {
        dev_err(&men->dev, "can not allocate dummy buffer\n");
        goto fail_dummy;
    }

    men->create_buf = me5_create_userbuf;
    men->free_buf = me5_free_sgl;
    men->startdma = men5_startdma;
    men->abortdma = men5_abort_dma;
    men->stopdma = men5_stop_dma;
    men->stopirq = men5_stopirq;
    men->startirq = men5_startirq;
    men->ioctl = men5_ioctl;
    men->exit = me5_exit;
    men->cleanup = me5_cleanup;
    men->query_dma = me5_query_dma;
    men->dmabase = men5_dmabase;
    men->queue_sb = me5_queue_sb;
    men->dummybuf = me5_dummybuf;

    uiqcnt = ioread32(men->runtime_base + ME5_UIQCNT);
    uiqoffs = ioread32(men->runtime_base + ME5_FIRSTUIQ);

    if ((uiqcnt == 0) && (uiqoffs == 0)) {
        /* old firmware versions did not provide this */
        uiqcnt = ME5_IRQQ_HIGH - ME5_IRQQ_LOW + 1;
        uiqoffs = ME5_IRQQUEUE;
    }

    if (uiqcnt != 0) {
        ret = me5_add_uiqs(men, 0, uiqcnt, uiqoffs);
        if (ret != 0) {
            goto fail_uiqs;
        }
    }

    spin_lock_irqsave(&men->d5->notify_lock, flags);
    men->d5->temperatureAlarmPeriod = 1000;
    men->d5->notification = 0;
    men->d5->notify_time_stamp = 0;
    men->d5->fired_alarms = 0;
    men->d5->notify_wait_count = 0;
    spin_unlock_irqrestore(&men->d5->notify_lock, flags);
    INIT_WORK(&men->d5->temperature_alarm_work, alarm_work);
    INIT_DELAYED_WORK(&men->d5->temperature_alarm_timer_work, temperature_alarm_timer);
    INIT_LIST_HEAD(&men->d5->notify_handler_heads);

#if 0
    ret = pci_enable_msi(men->pdev);
    if (ret)
    dev_info(&men->dev, "can't enable MSI\n");
#endif

    men->desname = men->d5->design_name;
    men->deslen = sizeof(men->d5->design_name);

    ret = devm_request_irq(&men->pdev->dev, men->pdev->irq, me5_irq,
            IRQF_SHARED, DRIVER_NAME, men);
    if (ret) {
        dev_err(&men->dev, "can't request interrupt\n");
        goto fail_irq;
    }


    return 0;
fail_irq:
    men_del_uiqs(men, 0);
    kfree(men->uiqs);
fail_uiqs:
    men5_destroy_dummybuf(men);
fail_dummy: fail_pool: fail_mask:
    kfree(men->d5);
fail:
    return ret;
}
