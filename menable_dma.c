/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "menable.h"
#include "linux_version.h"

/**
 * men_dma_channel - get DMA channel on the given board
 * @men: board to query
 * @dma: dma channel index
 *
 * This will return the DMA channel with the given index if
 * it exists, otherwise NULL.
 */
struct menable_dmachan *
men_dma_channel(struct siso_menable *men, const unsigned int index)
{
	unsigned int i;
	unsigned int j = 0;	/* the highest valid DMA index +1 */

	for (i = 0; i < MAX_FPGAS; i++) {
		j += men->dmacnt[i];
	}

	WARN_ON(j > MEN_MAX_DMA);

	if (index < j)
		return men->dmachannels[index];
	else
		return NULL;
}

/**
 * men_get_dmaimg - print current picture number to sysfs
 * @dev: device to query
 * @attr: device attribute of the channel file
 * @buf: buffer to print information to
 *
 * The result will be printed in decimal form into the buffer.
 */
static ssize_t
men_get_dmaimg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_dmachan *d = container_of(dev,
			struct menable_dmachan, dev);

	return sprintf(buf, "%lli\n", d->goodcnt);
}

/**
 * men_get_dmalost - print current number of lost pictures to sysfs
 * @dev: device to query
 * @attr: device attribute of the channel file
 * @buf: buffer to print information to
 *
 * The result will be printed in decimal form into the buffer.
 */
static ssize_t
men_get_dmalost(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct menable_dmachan *d = container_of(dev,
			struct menable_dmachan, dev);

	return sprintf(buf, "%i\n", d->lost);
}

struct device_attribute men_dma_attributes[3] = {
	__ATTR(lost, 0440, men_get_dmalost, NULL),
	__ATTR(img, 0440, men_get_dmaimg, NULL),
	__ATTR_NULL
};

struct class *menable_dma_class = NULL;

static struct lock_class_key men_dmacpl_lock;

/**
 * men_wait_dmaimg - wait until the given image is grabbed
 * @d: the DMA channel to watch
 * @img: the image number to wait for
 * @timeout: wait limit
 *
 * This function blocks until the current image number is at least the one
 * requested in @buf.
 *
 * Returns: current picture number on success, error code on failure
 */
int
men_wait_dmaimg(struct menable_dmachan *d, const uint64_t img,
			const struct timespec *timeout, uint64_t *foundframe)
{
	uint64_t cur;
	struct device *dv;
	unsigned long flags;
	struct menable_dma_wait waitstr;

	dv = get_device(&d->dev);
	init_completion(&waitstr.cpl);
	lockdep_set_class(&waitstr.cpl.wait.lock, &men_dmacpl_lock);
	INIT_LIST_HEAD(&waitstr.node);
	waitstr.frame = img;

	spin_lock_irqsave(&d->listlock, flags);
	if (d->goodcnt >= img) {
		spin_unlock_irqrestore(&d->listlock, flags);
		put_device(dv);
		*foundframe = d->goodcnt;
		return 0;
	}
	if (unlikely(d->running != 1)) {
		spin_unlock_irqrestore(&d->listlock, flags);
		put_device(dv);
		return -ETIMEDOUT;
	}
	list_add_tail(&waitstr.node, &d->wait_list);
	spin_unlock_irqrestore(&d->listlock, flags);
	wait_for_completion_interruptible_timeout(&waitstr.cpl,
				timespec_to_jiffies(timeout));

	spin_lock_irqsave(&d->listlock, flags);
	list_del(&waitstr.node);
	cur = d->goodcnt;
	spin_unlock_irqrestore(&d->listlock, flags);

	put_device(dv);
	if (cur < img)
		return -ETIMEDOUT;

	*foundframe = d->goodcnt;
	return 0;
}

static void
men_dma_timeout(unsigned long arg)
{
	unsigned long flags;
	struct menable_dmachan *dc = (struct menable_dmachan *)arg;
	struct menable_dma_wait *waitstr;

	spin_lock(&dc->parent->headlock);
	spin_lock_irqsave(&dc->chanlock, flags);
	if (!spin_trylock(&dc->timerlock)) {
		/* If this fails someone else tries to modify this timer.
		 * This means the timer will either get restarted or deleted.
		 * In both cases we don't need to do anything here as the
		 * other function will take care of everything. */
		spin_unlock_irqrestore(&dc->chanlock, flags);
		spin_unlock(&dc->parent->headlock);
		return;
	}

	dc->parent->abortdma(dc->parent, dc);
	dc->running = 2;
	spin_lock(&dc->listlock);
	list_for_each_entry(waitstr, &dc->wait_list, node) {
		complete(&waitstr->cpl);
	}
	spin_unlock(&dc->listlock);

	spin_unlock(&dc->timerlock);
	spin_unlock_irqrestore(&dc->chanlock, flags);
	spin_unlock(&dc->parent->headlock);
}

static void
menable_chan_release(struct device *dev)
{
	struct menable_dmachan *d = container_of(dev, struct menable_dmachan, dev);

	kfree(d);
}

static struct lock_class_key men_dma_lock;
static struct lock_class_key men_dmachan_lock;
static struct lock_class_key men_dmatimer_lock;

/**
 * dma_clean_sync - put DMA channel in stopped state
 * @db: channel to work
 *
 * This resets everything in the DMA channel to stopped state, e.g. clearing
 * association of a memory buffer.
 *
 * context: IRQ (headlock and chanlock must be locked and released from caller)
 *          the channel must be in "stop pending" state (running == 3)
 */
void
dma_clean_sync(struct menable_dmachan *db)
{
	struct menable_dma_wait *waitstr;

	BUG_ON(db->running != 3);
	del_timer(&db->timer);
	db->running = 2;
	db->transfer_todo = 0;
	spin_lock(&db->listlock);
	list_for_each_entry(waitstr, &db->wait_list, node) {
		complete(&waitstr->cpl);
	}
	spin_unlock(&db->listlock);
}

void
dma_done_work(struct work_struct *work)
{
	struct menable_dmachan *db = container_of(work,
			struct menable_dmachan, dwork);
	struct siso_menable *men = db->parent;
	unsigned long flags;

	spin_lock_bh(&men->headlock);
	spin_lock_irqsave(&db->chanlock, flags);
	/* the timer might have cancelled everything in the mean time */
	if (db->running == 3)
		dma_clean_sync(db);
	spin_unlock_irqrestore(&db->chanlock, flags);
	spin_unlock_bh(&men->headlock);
}

static struct menable_dmachan *
men_create_dmachan(struct siso_menable *parent, const unsigned char index, const unsigned char fpga)
{
	char symlinkname[16];
	struct menable_dmachan *res = kzalloc(sizeof(*res), GFP_KERNEL);
	int r = 0;

	if (!res)
		return ERR_PTR(ENOMEM);

	/* our device */
	res->dev.parent = &parent->dev;
	res->dev.release = menable_chan_release;
	res->dev.driver = parent->dev.driver;
	res->dev.class = menable_dma_class;
	res->dev.groups = 0;
	res->number = index;
	res->fpga = fpga;
	parent->dmabase(parent, res);
	dev_set_name(&res->dev, "%s_dma%i", dev_name(&parent->dev), index);

	/* misc setup */
	res->parent = parent;
	spin_lock_init(&res->listlock);
	spin_lock_init(&res->chanlock);
	spin_lock_init(&res->timerlock);
	lockdep_set_class(&res->listlock, &men_dma_lock);
	lockdep_set_class(&res->chanlock, &men_dmachan_lock);
	lockdep_set_class(&res->timerlock, &men_dmatimer_lock);
	INIT_LIST_HEAD(&res->free_list);
	INIT_LIST_HEAD(&res->grabbed_list);
	INIT_LIST_HEAD(&res->hot_list);
	INIT_LIST_HEAD(&res->wait_list);
	init_timer(&res->timer);
	res->timer.function = men_dma_timeout;
	res->timer.data = (unsigned long)res;
	INIT_WORK(&res->dwork, dma_done_work);

	r = device_register(&res->dev);
	if (r != 0)
		goto err;

	snprintf(symlinkname, sizeof(symlinkname), "dma%i", index);
	r = sysfs_create_link(&parent->dev.kobj, &res->dev.kobj, symlinkname);
	if (r)
		goto err_data;

	return res;
err_data:
	device_unregister(&res->dev);
err:
	kfree(res);
	return ERR_PTR(r);
}

static void
men_dma_remove(struct menable_dmachan *d)
{
	char symlinkname[16];

	del_timer_sync(&d->timer);
	flush_scheduled_work();

	snprintf(symlinkname, sizeof(symlinkname), "dma%i", d->number);
	sysfs_remove_link(&d->parent->dev.kobj, symlinkname);

	device_unregister(&d->dev);
}

/**
 * men_add_dmas - add more DMA channels
 * @men: board to pimp
 */
int
men_add_dmas(struct siso_menable *men)
{
	unsigned int i;
	struct menable_dmachan **old, **nc;
	unsigned long flags;
	unsigned int count = 0;
	unsigned int oldsum = 0;
	unsigned int oldcnt[MAX_FPGAS];
	unsigned int newcnt[MAX_FPGAS];
	unsigned int newindex;

	for (i = 0; i < MAX_FPGAS; i++) {
		oldsum += men->dmacnt[i];
		oldcnt[i] = men->dmacnt[i];
	}

	/* DMA count may only change if that FPGA previously had none */
	for (i = 0; i < men->active_fpgas; i++) {
		unsigned int tmp = men->query_dma(men, i);
		WARN_ON((tmp != oldcnt[i]) && (oldcnt[i] != 0));
		count += tmp;
		newcnt[i] = tmp;
	}

	WARN_ON(count < oldsum);
	if (unlikely(count <= oldsum))
		return 0;

	nc = kcalloc(count, sizeof(*nc), GFP_KERNEL);

	if (unlikely(nc == NULL))
		return -ENOMEM;

	newindex = oldsum;
	for (i = 0; i < men->active_fpgas; i++) {
		unsigned int j;

		if (oldcnt[i] == newcnt[i])
			continue;

		/* allocate control structs for all new channels */
		for (j = 0; j < newcnt[i]; j++) {
			nc[newindex] = men_create_dmachan(men, newindex, i);

			if (unlikely(IS_ERR(nc[newindex]))) {
				int ret = PTR_ERR(nc[newindex]);
				unsigned int j;

				for (j = oldsum; j < newindex; j++)
					men_dma_remove(nc[j]);

				kfree(nc);
				return ret;
			}
			newindex++;
		}
	}

	spin_lock_bh(&men->headlock);
	spin_lock_irqsave(&men->boardlock, flags);
	/* copy old array contents to new one */
	old = men->dmachannels;
	for (i = 0; i < oldsum; i++)
		nc[i] = old[i];

	men->dmachannels = nc;
	for (i = 0; i < men->active_fpgas; i++)
		men->dmacnt[i] = men->query_dma(men, i);
	spin_unlock_irqrestore(&men->boardlock, flags);
	spin_unlock_bh(&men->headlock);

	kfree(old);

	return 0;
}

/**
 * men_alloc_dma - change number of DMA channels of this board
 * @men: board
 * @count: new channel count
 *
 * count must not be greater than the current DMA count. Use men_add_dmas()
 * instead.
 */
int
men_alloc_dma(struct siso_menable *men, unsigned int count)
{
	struct menable_dmachan *old[MEN_MAX_DMA];
	struct menable_dmachan **delold = NULL;
	int i;
	unsigned int oldcnt = 0;
	unsigned long flags;

	spin_lock_irqsave(&men->boardlock, flags);

	if (unlikely(men->releasing))
		count = 0;

	for (i = 0; i < MAX_FPGAS; i++)
		oldcnt += men->dmacnt[i];
	BUG_ON(oldcnt > MEN_MAX_DMA);

	if (count == oldcnt) {
		spin_unlock_irqrestore(&men->boardlock, flags);
		spin_unlock_bh(&men->headlock);
		return 0;
	} else if (count > oldcnt) {
		spin_unlock_irqrestore(&men->boardlock, flags);
		spin_unlock_bh(&men->headlock);
		WARN_ON(count > oldcnt);
		return 0;
	}

	for (i = count; i < oldcnt; i++) {
		struct menable_dmachan *dc = men_dma_channel(men, i);
		spin_lock(&dc->chanlock);
		men_stop_dma_locked(dc);
		spin_unlock(&dc->chanlock);
	}

	memset(old, 0, sizeof(old));

	/* copy those we will free */
	for (i = count; i < oldcnt; i++) {
		old[i] = men->dmachannels[i];
		men->dmachannels[i] = NULL;
	}
	/* don't bother shrinking the array here, it's
	 * not worth the effort */

	for (i = 0; i < MAX_FPGAS; i++) {
		unsigned int oldcnt = men->dmacnt[i];

		men->dmacnt[i] = min(count, oldcnt);
		if (count > 0)
			count -= oldcnt;
	}
	if (count == 0) {
		delold = men->dmachannels;
		men->dmachannels = NULL;
	}
	spin_unlock_irqrestore(&men->boardlock, flags);
	spin_unlock_bh(&men->headlock);

	kfree(delold);

	for (i = oldcnt - 1; i >= 0; --i) {
		if (old[i] == NULL)
			continue;

		men_dma_remove(old[i]);
	}

	return 0;
}

ssize_t
men_get_dmas(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);
	unsigned int i;
	unsigned int cnt = 0;

	for (i = 0; i < men->active_fpgas; i++)
		cnt += men->dmacnt[i];

	return sprintf(buf, "%i\n", cnt);
}

static void
men_clean_bh(struct menable_dmachan *dc, const unsigned int startbuf)
{
	long i;
	struct menable_dmahead *active = dc->active;

	dc->free = 0;
	dc->grabbed = 0;
	dc->locked = 0;
	dc->lost = 0;
	dc->hot = 0;
	dc->goodcnt = 0;
	INIT_LIST_HEAD(&dc->free_list);
	INIT_LIST_HEAD(&dc->hot_list);
	INIT_LIST_HEAD(&dc->grabbed_list);

	for (i = startbuf; i < active->num_sb; i++) {
		if (active->bufs[i] == NULL)
			continue;
		list_add_tail(&active->bufs[i]->node, &dc->free_list);
		dc->free++;
	}

	if (startbuf > 0) {
		for (i = 0; i < startbuf; i++) {
			if (active->bufs[i] == NULL)
				continue;
			list_add_tail(&active->bufs[i]->node, &dc->free_list);
			dc->free++;
		}
	}

	active->chan = dc;
}

int
men_start_dma(struct menable_dmachan *dc, struct menable_dmahead *newh,
		const unsigned int startbuf)
{
	int ret = 0;

	if ((newh->chan != NULL) &&
			(((dc->running != 2) && (dc->running != 0)) ||
			((newh->chan->running != 2) &&
					(newh->chan->running != 0)))) {
		return -EBUSY;
	}

	if ((dc->active != NULL) && (dc->active != newh))
		dc->active->chan = NULL;

	dc->active = newh;
	dc->running = 1;

	spin_lock(&dc->listlock);
	men_clean_bh(dc, startbuf);
	if (dc->free == 0)
		ret = -ENODEV;
	spin_unlock(&dc->listlock);
	if (ret) {
		dc->running = 0;
		return ret;
	}

	ret = dc->parent->startdma(dc);

	if (ret == 0) {
		spin_lock(&dc->timerlock);
		add_timer(&dc->timer);
		spin_unlock(&dc->timerlock);
	}
	return ret;
}
