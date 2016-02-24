/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include "uiq.h"
#include "menable.h"
#include "linux_version.h"

#define UIQ_CONTROL_EOT		0x00010000	/* the FIFO will be empty after this value */
#define UIQ_CONTROL_DATALOSS	0x00020000	/* data was lost before this value */
#define UIQ_CONTROL_INSERT_TS	0x00040000	/* insert timestamp after this word */
#define UIQ_CONTROL_INVALID	0x80000000	/* the current value is invalid and must be discarded */

static bool
men_uiq_pop(struct menable_uiq *uiq, struct timespec *ts, bool *have_ts)
{
	unsigned int windex;
	uint32_t val;
	bool was_empty;
	int transfer = 0;

	if (unlikely(uiq->length == 0)) {
		do {
			val = ioread32(uiq->reg);
			if (!(val & UIQ_CONTROL_INVALID))
				uiq->lost++;
		} while (!(val & UIQ_CONTROL_EOT));
		return false;
	}

	windex = (uiq->rindex + uiq->fill) % uiq->length;
	was_empty = (uiq->fill == 0);
	do {
		val = ioread32(uiq->reg);

		if (!(val & UIQ_CONTROL_INVALID)) {
			if (uiq->fill < uiq->length) {
				if (unlikely(uiq->loss_error)) {
					val |= UIQ_CONTROL_DATALOSS;
					uiq->loss_error = false;
				}
				if (unlikely((val & UIQ_CONTROL_INSERT_TS) &&
						(uiq->fill + 4 < uiq->length))) {
					uint32_t eflag = val & UIQ_CONTROL_EOT;

					if (!*have_ts) {
						*ts = current_kernel_time();
						*have_ts = true;
					}

					uiq->data[windex] = val & ~UIQ_CONTROL_EOT;
					windex = (windex + 1) % uiq->length;
					uiq->fill++;
					uiq->data[windex] = ts->tv_sec & 0xffff;
					windex = (windex + 1) % uiq->length;
					uiq->fill++;
					uiq->data[windex] = (ts->tv_sec >> 16) & 0xffff;
					windex = (windex + 1) % uiq->length;
					uiq->fill++;
					uiq->data[windex] = ts->tv_nsec & 0xffff;
					windex = (windex + 1) % uiq->length;
					uiq->fill++;
					uiq->data[windex] = (ts->tv_nsec >> 16) & 0xffff;
					uiq->data[windex] |= eflag;
					windex = (windex + 1) % uiq->length;
				} else {
					uiq->data[windex] = val;
				}
				windex = (windex + 1) % uiq->length;
				uiq->fill++;
				transfer++;
			} else {
				uiq->lost++;
				uiq->loss_error = true;
			}
		} else {
			break; // don't loop on empty FIFO!
		}
	} while (!(val & UIQ_CONTROL_EOT));
	if (uiq->cpltodo && (uiq->cpltodo <= uiq->fill))
		complete(&uiq->cpl);
	uiq->irqcnt++;

	return (was_empty && (uiq->fill > 0));
}

static bool
men_uiq_push(struct menable_uiq *uiq)
{
	unsigned int num, i;

	num = min_t(unsigned int, uiq->burst, uiq->fill);
	for (i = 0; i < num - 1; i++) {
		iowrite32(uiq->data[uiq->rindex] & 0xffff, uiq->reg);
		uiq->rindex = (uiq->rindex + 1) % uiq->length;
	}
	iowrite32(uiq->data[uiq->rindex] | UIQ_CONTROL_EOT, uiq->reg);
	uiq->rindex = (uiq->rindex + 1) % uiq->length;
	uiq->irqcnt++;
	uiq->fill -= num;
	if (uiq->cpltodo && (uiq->cpltodo <= uiq->length - uiq->fill))
		complete(&uiq->cpl);

	if (uiq->fill == 0) {
		/* avoid wraparound */
		uiq->rindex = 0;
		return true;
	} else {
		return false;
	}
}

static void
uiq_cpy(struct menable_uiq *uiq, void *buf, const unsigned int num)
{
	if (unlikely(uiq->rindex + num >= uiq->length)) {
		unsigned int wrap = uiq->length - uiq->rindex;

		memcpy(buf, uiq->data + uiq->rindex,
			wrap * sizeof(*uiq->data));
		memcpy(buf + wrap * sizeof(*uiq->data), uiq->data,
			(num - wrap) * sizeof(*uiq->data));
	} else {
		memcpy(buf, uiq->data + uiq->rindex,
			num * sizeof(*uiq->data));
	}
}

static ssize_t
uiq_read(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		struct file *filp,
#endif /* LINUX >= 2.6.35 */
		struct kobject *kobj, struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	unsigned long flags;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);
	unsigned int cnt = count / sizeof(*uiq->data);
	ssize_t num = 0;

	spin_lock_irqsave(&uiq->lock, flags);
	if (unlikely(uiq->cpltodo)) {
		spin_unlock_irqrestore(&uiq->lock, flags);
		return -EBUSY;
	}
	if (uiq->fill < cnt) {
		unsigned long t = uiq->cpltimeout;
		uiq->cpltodo = cnt;
		do {
			spin_unlock_irqrestore(&uiq->lock, flags);
			t = wait_for_completion_interruptible_timeout(&uiq->cpl,
					t);
			spin_lock_irqsave(&uiq->lock, flags);
			if (uiq->fill >= cnt)
				break;
		} while ((t > 0) && !fatal_signal_pending(current));
		uiq->cpltodo = 0;
	}

	if (uiq->fill > 0) {
		unsigned int c = min(uiq->fill, cnt);

		uiq_cpy(uiq, buf, c);
		uiq->fill -= c;
		if (uiq->fill == 0)
			uiq->rindex = 0;
		else
			uiq->rindex = (uiq->rindex + c) % uiq->length;
		num = c * sizeof(*uiq->data);
	} else {
		num = -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&uiq->lock, flags);
	return num;
}

static ssize_t
uiq_write(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		struct file *filp,
#endif /* LINUX >= 2.6.35 */
		struct kobject *kobj, struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	unsigned long flags;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);
	const unsigned int cnt = count / sizeof(*uiq->data);
	unsigned int wrap;
	unsigned int windex;
	bool notify = false;

	if (unlikely(count == 0))
		return 0;

	if (unlikely(count % sizeof(*uiq->data) != 0))
		return -EINVAL;

	spin_lock_irqsave(&uiq->lock, flags);
	if (unlikely((uiq->length == 0) || (uiq->length < cnt))) {
		spin_unlock_irqrestore(&uiq->lock, flags);
		return -ENOSPC;
	}
	if (unlikely(uiq->cpltodo)) {
		spin_unlock_irqrestore(&uiq->lock, flags);
		return -EBUSY;
	}
	if (unlikely(uiq->reg == NULL)) {
		spin_unlock_irqrestore(&uiq->lock, flags);
		dev_err(&uiq->dev, "no register set\n");
		return -ENODEV;
	}
	if (uiq->length - uiq->fill < cnt) {
		unsigned long t = uiq->cpltimeout;
		uiq->cpltodo = cnt;
		do {
			spin_unlock_irqrestore(&uiq->lock, flags);
			t = wait_for_completion_interruptible_timeout(&uiq->cpl,
					t);
			spin_lock_irqsave(&uiq->lock, flags);
			if (uiq->cpltodo == -1) {
				/* someone send a clear request while we were
				 * waiting. We act as if the data of this request
				 * would already have been queued: we silently
				 * drop it. */
				uiq->cpltodo = 0;
				spin_unlock_irqrestore(&uiq->lock, flags);
				return count;
			}
			if (uiq->length - uiq->fill >= cnt)
				break;
		} while ((t > 0) && !fatal_signal_pending(current));
		uiq->cpltodo = 0;
	}
	if (uiq->length - uiq->fill < cnt) {
		spin_unlock_irqrestore(&uiq->lock, flags);
		return -EBUSY;
	}

	windex = (uiq->rindex + uiq->fill) % uiq->length;
	wrap = uiq->length - windex;

	if (wrap > cnt) {
		memcpy(uiq->data + windex, buf,
			cnt * sizeof(*uiq->data));
	} else {
		memcpy(uiq->data + windex, buf,
			wrap * sizeof(*uiq->data));
		memcpy(uiq->data, buf + wrap * sizeof(*uiq->data),
			(cnt - wrap) * sizeof(*uiq->data));
	}

	uiq->fill += cnt;

	if (!uiq->running) {
		notify = men_uiq_push(uiq);
		uiq->running = true;
	}

	spin_unlock_irqrestore(&uiq->lock, flags);

#if 0
	if (notify)
		sysfs_notify(&uiq->dev.kobj, NULL, "data");
#endif

	return count;
}

static ssize_t
men_uiq_readsize(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);

	/* This is not synchronized as this may change anyway
	 * until the user can use the information */
	return sprintf(buf, "%zi\n", uiq->length * sizeof(*uiq->data));
}

static ssize_t
men_uiq_writesize(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);
	unsigned int sz;
	int ret;

	ret = buf_get_uint(buf, count, &sz);
	if (ret)
		return ret;

	ret = men_scale_uiq(uiq, sz);

	return ret ? ret : count;
}

static ssize_t
men_uiq_readlost(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);

	/* This is not synchronized as this may change anyway
	 * until the user can use the information */
	return sprintf(buf, "%i\n", uiq->lost);
}

static ssize_t
men_uiq_readfill(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);

	/* This is not synchronized as this may change anyway
	 * until the user can use the information */
	return sprintf(buf, "%i\n", uiq->fill);
}

static ssize_t
men_uiq_writefill(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);
	unsigned long flags;

	if (strcmp(buf, "0"))
		return -EINVAL;

	spin_lock_irqsave(&uiq->lock, flags);
	uiq->fill = 0;
	uiq->rindex = 0;
	uiq->lost = 0;
	if (uiq->cpltodo) {
		uiq->cpltodo = -1;
		complete(&uiq->cpl);
	}
	spin_unlock_irqrestore(&uiq->lock, flags);

	return count;
}

static ssize_t
men_uiq_readirqcnt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);

	return sprintf(buf, "%i\n", uiq->irqcnt);
}

static ssize_t
men_uiq_writetimeout(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);
	unsigned long flags;
	unsigned int t;
	int ret;
	struct timespec timeout;
	unsigned long to;

	ret = buf_get_uint(buf, count, &t);
	if (ret)
		return ret;

	timeout.tv_nsec = (t % 1000) * 1000000;
	timeout.tv_sec = t / 1000;
	to = timespec_to_jiffies(&timeout);

	spin_lock_irqsave(&uiq->lock, flags);
	uiq->cpltimeout = to;
	spin_unlock_irqrestore(&uiq->lock, flags);

	return count;
}

struct bin_attribute bin_attr_uiq_data_r = {
	.attr = {
		.name = "data",
		.mode = S_IFREG | S_IRUSR | S_IRGRP
	},
	.size = 4096,
	.read = uiq_read
};

struct bin_attribute bin_attr_uiq_data_w = {
	.attr = {
		.name = "data",
		.mode = S_IFREG | S_IWUSR | S_IWGRP
	},
	.size = 4096,
	.write = uiq_write
};

struct device_attribute men_uiq_attributes[6] = {
	__ATTR(irqcnt, 0440, men_uiq_readirqcnt, NULL),
	__ATTR(fill, 0660, men_uiq_readfill, men_uiq_writefill),
	__ATTR(lost, 0440, men_uiq_readlost, NULL),
	__ATTR(size, 0660, men_uiq_readsize, men_uiq_writesize),
	__ATTR(timeout, 0220, NULL, men_uiq_writetimeout),
	__ATTR_NULL
};

static void menable_uiq_release(struct device *dev)
{
	struct menable_uiq *uiq = container_of(dev, struct menable_uiq, dev);

	kfree(uiq->data);
	kfree(uiq);
}

void
men_uiq_remove(struct menable_uiq *uiq)
{
	char symlinkname[16];

	if (uiq == NULL)
		return;

	snprintf(symlinkname, sizeof(symlinkname), "uiq%i", uiq->chan);
	sysfs_remove_link(&uiq->parent->dev.kobj, symlinkname);

	if (uiq->write_queue) {
		device_remove_bin_file(&uiq->dev, &bin_attr_uiq_data_w);
	} else {
		device_remove_bin_file(&uiq->dev, &bin_attr_uiq_data_r);
	}

	device_unregister(&uiq->dev);
}

static struct lock_class_key men_uiq_lock;
static struct lock_class_key men_uiqcpl_lock;

struct class *menable_uiq_class = NULL;

/**
 * men_uiq_init - create a UIQ object for the given queue
 * @chan UIQ channel number
 * @addr address of the queue I/O register
 * @parent grabber this UIQ belongs to
 * @write if queue is a read or write queue
 * @burst FIFO depth for write queues
 *
 * This will allocate the memory for the UIQ object and return
 * the allocated pointer or an ERR_PTR() value on failure.
 */
struct menable_uiq *
men_uiq_init(int chan, void __iomem *addr, struct siso_menable *parent,
		bool write, unsigned char burst)
{
	int ret;
	struct menable_uiq *uiq;
	char symlinkname[16];

	uiq = kzalloc(sizeof(*uiq), GFP_KERNEL);
	if (uiq == NULL)
		return ERR_PTR(-ENOMEM);

	uiq->chan = chan;
	uiq->reg = addr;
	uiq->parent = parent;
	uiq->dev.driver = parent->dev.driver;
	uiq->dev.class = menable_uiq_class;
	init_completion(&uiq->cpl);
	spin_lock_init(&uiq->lock);
	lockdep_set_class(&uiq->lock, &men_uiq_lock);
	lockdep_set_class(&uiq->cpl.wait.lock, &men_uiqcpl_lock);
	uiq->write_queue = write;
	uiq->loss_error = false;
	
	dev_set_name(&uiq->dev, "%s_uiq%i", dev_name(&parent->dev), chan);

	uiq->dev.parent = &parent->dev;
	uiq->dev.release = menable_uiq_release;
	uiq->dev.groups = 0;
	
	device_initialize(&uiq->dev);
	dev_set_uevent_suppress(&uiq->dev, 1);
	ret = device_add(&uiq->dev);
	if (ret)
		goto err;
	if (write) {
		uiq->burst = burst;
		ret = sysfs_create_bin_file(&uiq->dev.kobj, &bin_attr_uiq_data_w);
	} else {
	  	ret = sysfs_create_bin_file(&uiq->dev.kobj, &bin_attr_uiq_data_r);
	}
	if (ret)
		goto err_data;
	dev_set_uevent_suppress(&uiq->dev, 0);
	kobject_uevent(&uiq->dev.kobj, KOBJ_ADD);

	snprintf(symlinkname, sizeof(symlinkname), "uiq%i", chan);
	ret = sysfs_create_link(&parent->dev.kobj, &uiq->dev.kobj,
			symlinkname);
	if (ret)
		goto err_data;

	return uiq;
err_data:
	device_unregister(&uiq->dev);
err:
	kfree(uiq);
	return ERR_PTR(ret);
}

int
men_scale_uiq(struct menable_uiq *uiq, const unsigned int len)
{
	void *mem, *old;
	unsigned long flags;
	size_t cp;
	const unsigned int newlen = len / sizeof(*uiq->data);

	if (uiq->length == newlen)
		return 0;

	if (len == 0) {
		mem = NULL;
	} else {
		mem = kmalloc(len, GFP_USER);
		if (unlikely(mem == NULL))
			return -ENOMEM;
	}

	spin_lock_irqsave(&uiq->lock, flags);
	cp = min(uiq->fill, newlen);
	uiq_cpy(uiq, mem, cp);
	uiq->fill = cp;
	uiq->rindex = 0;
	old = uiq->data;
	uiq->data = mem;
	uiq->length = newlen;
	spin_unlock_irqrestore(&uiq->lock, flags);
	kfree(old);

	return 0;
}

void
uiq_irq(struct menable_uiq *uiq, struct timespec *ts, bool *have_ts)
{
	bool notify = false;

	BUG_ON(uiq == NULL);
	spin_lock(&uiq->lock);

	/* unconfigured board */
	if (unlikely(uiq->reg == NULL)) {
		spin_unlock(&uiq->lock);
		return;
	}

	if (uiq->write_queue == 0) {
		notify = men_uiq_pop(uiq, ts, have_ts);
		iowrite32(1 << uiq->ackbit, uiq->irqack);
	} else {
		WARN_ON(!uiq->running);

		iowrite32(1 << uiq->ackbit, uiq->irqack);
		if (uiq->fill == 0) {
			uiq->running = false;
			uiq->rindex = 0;
		} else {
			notify = men_uiq_push(uiq);
		}
	}

	spin_unlock(&uiq->lock);

#if 0
	if (notify)
		sysfs_notify(&uiq->dev.kobj, NULL, "data");
#endif
}

/**
 * men_del_uiqs - delete UIQs from given FPGA
 * @men: board to use
 * @fpga: fpga index to use
 *
 * This will also remove all UIQs from higher FPGAs.
 */
void
men_del_uiqs(struct siso_menable *men, const unsigned int fpga)
{
	unsigned int i;

	WARN_ON(!men->design_changing);

	for (i = fpga; i < MAX_FPGAS; i++) {
		unsigned int j;
		for (j = 0; j < men->uiqcnt[i]; j++) {
			unsigned int chan = i * MEN_MAX_UIQ_PER_FPGA + j;
			men_uiq_remove(men->uiqs[chan]);
			men->uiqs[chan] = NULL;
		}
		men->uiqcnt[i] = 0;
	}
}
