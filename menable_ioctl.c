/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include "menable_ioctl.h"
#include "menable.h"

/**
 * warn_wrong_iosize - print warning about bad ioctl argument
 * @men device that happened on
 * @cmd ioctl command sent in
 * @expsize size expected by this ioctl
 *
 * Basically this should never happen: all ioctls are sent in by the
 * Silicon Software runtime environment and should do this right. This
 * is basically to make internal debugging easier when someone
 * accidentially messes up some struct layout.
 */
void
warn_wrong_iosize(struct siso_menable *men, unsigned int cmd,
		const size_t expsize)
{
	dev_dbg(&men->dev,
		"ioctl %i called with buffer length %i but expected %zi\n",
		_IOC_NR(cmd), _IOC_SIZE(cmd), expsize);
}

static long
men_ioctl_fgstart(struct siso_menable *men, unsigned int cmd, unsigned long arg)
{
	struct fg_ctrl fg;
	struct fg_ctrl_s fgr;

	if (unlikely(_IOC_SIZE(cmd) != sizeof(fgr))) {
		warn_wrong_iosize(men, cmd, sizeof(fgr));
		return -EINVAL;
	}

	if (copy_from_user(&fgr, (void __user *)arg, sizeof(fgr)))
		return -EFAULT;

	fg.mode = fgr.u.fg_start.mode;
	fg.timeout = fgr.u.fg_start.timeout;
	fg.transfer_todo = fgr.u.fg_start.transfer_todo;
	fg.chan = fgr.u.fg_start.chan;
	fg.head = fgr.u.fg_start.head;
	fg.start_buf = fgr.u.fg_start.start_buf;
	fg.dma_dir = fgr.u.fg_start.dma_dir;

	return fg_start_transfer(men, &fg, fgr.u.fg_start.act_size);
}

static long
men_ioctl_unlock_buffer(struct siso_menable *men, const unsigned int head,
		const long index)
{
	struct menable_dmahead *dh;
	struct menable_dmabuf *sb;
	struct menable_dmachan *dc;
	int ret;
	unsigned long flags;

	if (unlikely(index < -1))
		return -EINVAL;

	dh = me_get_bh(men, head);

	if (dh == NULL)
		return -EINVAL;

	if (unlikely(index >= dh->num_sb)) {
		spin_unlock_bh(&men->headlock);
		return -EINVAL;
	}

	dc = dh->chan;
	if (unlikely(dc == NULL)) {
		spin_unlock_bh(&men->headlock);
		return 0;
	}

	spin_lock_irqsave(&dc->listlock, flags);
	if (index == -1) {
		long i;

		for (i = 0; i < dh->num_sb; i++) {
			sb = dh->bufs[i];
			men_unblock_buffer(dc, sb);
		}
		BUG_ON(dc->grabbed);
		BUG_ON(dc->locked);
		ret = 0;
	} else {
		sb = dh->bufs[index];
		if (!sb) {
			ret = -EINVAL;
		} else {
			men_unblock_buffer(dc, sb);
			ret = 0;
		}
	}
	spin_unlock_irqrestore(&dc->listlock, flags);
	spin_unlock_bh(&men->headlock);
	return ret;
}

long menable_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct siso_menable *men = file->private_data;

	if (unlikely(men == NULL)) {
		WARN_ON(!men);
		return -EFAULT;
	}

	if (unlikely(_IOC_TYPE(cmd) != 'm'))
		return -EINVAL;

	switch (_IOC_NR(cmd)) {
	case IOCTL_ALLOCATE_VIRT_BUFFER: {
		struct mm_create_s mm_cmd;

		if (_IOC_SIZE(cmd) != sizeof(mm_cmd)) {
			warn_wrong_iosize(men, cmd, sizeof(mm_cmd));
			return -EINVAL;
		}

		if (copy_from_user(&mm_cmd,
				(void __user *) arg,
				sizeof(mm_cmd)))
			return -EFAULT;

#if BITS_PER_LONG < 64
		if (mm_cmd.maxsize > 0xffffffffULL)
			return -EINVAL;
#endif

		return men_create_buf_head(men,
				mm_cmd.maxsize,
				mm_cmd.subbufs);
	}
	case IOCTL_ADD_VIRT_USER_BUFFER: {
		struct men_io_range range;

		if (_IOC_SIZE(cmd) != sizeof(range)) {
			warn_wrong_iosize(men, cmd, sizeof(range));
			return -EINVAL;
		}

		if (copy_from_user(&range,
				(void __user *) arg,
				sizeof(range)))
			return -EFAULT;
		if (range.length == 0)
			return -EFAULT;
		return men_create_userbuf(men, &range);
	}
	case IOCTL_DEL_VIRT_USER_BUFFER: {
		struct men_io_bufidx num;
		struct menable_dmahead *dh;
		int r;

		if (_IOC_SIZE(cmd) != sizeof(num)) {
			warn_wrong_iosize(men, cmd, sizeof(num));
			return -EINVAL;
		}

		if (copy_from_user(&num,
				(void __user *) arg,
				sizeof(num)))
			return -EFAULT;

		dh = me_get_bh(men, num.headnr);

		if (dh == NULL) {
			r = -EINVAL;
		} else {
			struct menable_dmabuf *sb = me_get_sb_by_head(dh, num.index);
			r = men_free_userbuf(men, dh, num.index);
			spin_unlock_bh(&men->headlock);
			if (r == 0)
				men_destroy_sb(men, sb);
		}
		return r;
	}
	case IOCTL_FREE_VIRT_BUFFER: {
		struct menable_dmahead *bh;
		int ret;

		if (_IOC_SIZE(cmd) != 0) {
			warn_wrong_iosize(men, cmd, 0);
			return -EINVAL;
		}

		bh = me_get_bh(men, (unsigned int) arg);
		if (bh == NULL)
			return 0;
		ret = men_release_buf_head(men, bh);
		spin_unlock_bh(&men->headlock);
		if (ret == 0)
			men_free_buf_head(men, bh);
		return ret;
	}
	case IOCTL_DMA_LENGTH: {
		struct men_io_bufidx binfo;
		struct menable_dmabuf *sb;
		uint64_t ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(binfo))) {
			warn_wrong_iosize(men, cmd, sizeof(binfo));
			return -EINVAL;
		}

		if (copy_from_user(&binfo, (void __user *) arg,
				sizeof(binfo)))
			return -EFAULT;

		sb = me_get_sb(men, binfo.headnr, binfo.index);
		if (unlikely(sb == NULL))
			return -EINVAL;
		ret = sb->dma_length;
		spin_unlock_bh(&men->headlock);

		if (copy_to_user((void __user *) arg, &ret, sizeof(ret)))
			return -EFAULT;

		if (ret >= INT_MAX)
			return INT_MAX;
		else
			return (int)ret;
	}
	case IOCTL_DMA_TAG: {
		struct men_io_bufidx binfo;
		struct menable_dmabuf *sb;
		uint32_t ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(binfo))) {
			warn_wrong_iosize(men, cmd, sizeof(binfo));
			return -EINVAL;
		}

		if (copy_from_user(&binfo, (void __user *) arg,
				sizeof(binfo)))
			return -EFAULT;

		sb = me_get_sb(men, binfo.headnr, binfo.index);
		if (unlikely(sb == NULL))
			return -EINVAL;
		ret = sb->dma_tag;
		spin_unlock_bh(&men->headlock);

		if (copy_to_user((void __user *) arg, &ret, sizeof(ret)))
			return -EFAULT;

		return ret & 0x7ffffff;
	}
	case IOCTL_DMA_TIME_STAMP: {
		struct dma_timestamp ts;
		struct menable_dmabuf *sb;
		struct timespec tmp;
		int ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(ts))) {
			warn_wrong_iosize(men, cmd, sizeof(ts));
			return -EINVAL;
		}

		if (copy_from_user(&ts, (void __user *) arg, sizeof(ts)))
			return -EFAULT;

		sb = me_get_sb(men, ts.head, ts.buf);
		if (unlikely(sb == NULL))
			return -EINVAL;
		tmp = sb->timestamp;
		spin_unlock_bh(&men->headlock);

		ret = copy_to_user(((void __user *) arg) +
				offsetof(typeof(ts), stamp),
				&tmp, sizeof(tmp));
		return ret ? -EFAULT : 0;
	}
	case IOCTL_FG_WAIT_FOR_SUBBUF: {
		struct men_io_bufwait ctrl;
		struct menable_dmachan *db;
		struct timespec timeout;
		uint64_t foundframe;
		int ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(ctrl))) {
			warn_wrong_iosize(men, cmd, sizeof(ctrl));
			return -EINVAL;
		}

		if (copy_from_user(&ctrl, (void __user *) arg, sizeof(ctrl)))
			return -EFAULT;

		if (unlikely(ctrl.index < 0))
			return -EINVAL;

		db = men_dma_channel(men, ctrl.dmachan);
		if (unlikely(db == NULL))
			return -ECHRNG;

		timeout.tv_nsec = 0;
		timeout.tv_sec = ctrl.timeout;

		ret = men_wait_dmaimg(db, ctrl.index, &timeout, &foundframe);

		if (ret < 0)
			return ret;

		ctrl.index = foundframe;

		if (copy_to_user((void __user *) arg, &ctrl, sizeof(ctrl)))
			return -EFAULT;

		if (foundframe >= INT_MAX)
			return INT_MAX;
		else
			return foundframe;
	}
	case IOCTL_FG_STOP_CMD: {
		struct menable_dmachan *dc;

		if (_IOC_SIZE(cmd) != 0) {
			warn_wrong_iosize(men, cmd, 0);
			return -EINVAL;
		}

		dc = men_dma_channel(men, arg);
		if (dc == NULL)
			return -ECHRNG;
		men_stop_dma(dc);
		return 0;
	}
	case IOCTL_FG_START_TRANSFER: {
#if BITS_PER_LONG == 32
		return men_ioctl_fgstart(men, cmd, arg);
#else /* BITS_PER_LONG == 32 */
		struct fg_ctrl fgr;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(fgr))) {
			warn_wrong_iosize(men, cmd, sizeof(fgr));
			return -EINVAL;
		}

		if (copy_from_user(&fgr, (void __user *)arg, sizeof(fgr)))
			return -EFAULT;

		return fg_start_transfer(men, &fgr, 0);
#endif /* BITS_PER_LONG == 32 */
	}
	case IOCTL_GET_BUFFER_STATUS: {
		struct bufstatus data;
		struct menable_dmahead *dh;
		struct menable_dmachan *dc;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(data))) {
			warn_wrong_iosize(men, cmd, sizeof(data));
			return -EINVAL;
		}

		if (copy_from_user(&data, (void __user *) arg, sizeof(data)))
			return -EFAULT;

		dh = me_get_bh(men, data.idx.head);

		if (unlikely(dh == NULL))
			return -EINVAL;

		if (unlikely((data.idx.index >= dh->num_sb) || (data.idx.index < -1))) {
			spin_unlock_bh(&men->headlock);
			return -EINVAL;
		}

		if (data.idx.index == -1) {
			data.status.is_locked = 0;
		} else {
			struct menable_dmabuf *sb = dh->bufs[data.idx.index];
			if (unlikely(sb == NULL)) {
				spin_unlock_bh(&men->headlock);
				return -EINVAL;
			}
			data.status.is_locked = (sb->listname != FREE_LIST);
		}

		dc = dh->chan;
		if (unlikely(dc == NULL)) {
			spin_unlock_bh(&men->headlock);
			return -EINVAL;
		}

		data.status.free = dc->free;
		data.status.grabbed = dc->grabbed;
		data.status.locked = dc->locked;
		data.status.lost = dc->lost;

		spin_unlock_bh(&men->headlock);

		if (copy_to_user((void __user *) arg, &data, sizeof(data)))
			return -EFAULT;

		return 0;
	}
	case IOCTL_UNLOCK_BUFFER_NR: {
		struct men_io_bufidx data;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(data))) {
			warn_wrong_iosize(men, cmd, sizeof(data));
			return -EINVAL;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

		return men_ioctl_unlock_buffer(men, data.headnr, data.index);
	}
	case IOCTL_GET_HANDSHAKE_DMA_BUFFER: {
		struct handshake_frame data;
		struct menable_dmahead *dh;
		struct menable_dmabuf *sb;
		struct menable_dmachan *dc;
		unsigned long flags;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(data))) {
			warn_wrong_iosize(men, cmd, sizeof(data));
			return -EINVAL;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

		switch (data.mode) {
		case SEL_ACT_IMAGE:
		case SEL_NEXT_IMAGE:
			break;
		default:
			return -EINVAL;
		}

		dh = me_get_bh(men, data.head);

		if (unlikely(dh == NULL))
			return -EINVAL;

		dc = dh->chan;
		if (unlikely(dc == NULL)) {
			spin_unlock_bh(&men->headlock);
			return -EINVAL;
		}

		spin_lock_irqsave(&dc->listlock, flags);
		switch (data.mode) {
		case SEL_ACT_IMAGE:
			sb = men_last_blocked(men, dc);
			break;
		case SEL_NEXT_IMAGE:
			sb = men_next_blocked(men, dc);
			break;
		default:
			BUG();
		}
		spin_unlock_irqrestore(&dc->listlock, flags);
		spin_unlock_bh(&men->headlock);
		if (unlikely(sb == NULL)) {
			return -ENOENT;
		} else {
			data.frame = sb->index;
			if (copy_to_user((void __user *)arg,
					&data, sizeof(data)))
				return -EFAULT;

			if (sb->index >= INT_MAX)
				return INT_MAX;
			else
				return sb->index;
		}
	}
	default:
		return men->ioctl(men, _IOC_NR(cmd), _IOC_SIZE(cmd), arg);
	}
}

long menable_compat_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct siso_menable *men = file->private_data;

	if (unlikely(men == NULL)) {
		WARN_ON(!men);
		return -EFAULT;
	}

	if (unlikely(_IOC_TYPE(cmd) != 'm'))
		return -EINVAL;

	switch (_IOC_NR(cmd)) {
	case IOCTL_ALLOCATE_VIRT_BUFFER32: {
		struct mm_create_s32 mm_cmd;

		if (_IOC_SIZE(cmd) != sizeof(mm_cmd)) {
			warn_wrong_iosize(men, cmd, sizeof(mm_cmd));
			return -EINVAL;
		}

		if (copy_from_user(&mm_cmd,
				(void __user *) arg,
				sizeof(mm_cmd)))
			return -EFAULT;

		if (mm_cmd.maxsize > 0xffffffffULL)
			return -EINVAL;

		return men_create_buf_head(men,
				mm_cmd.maxsize,
				mm_cmd.subbufs);
	}
	case IOCTL_ADD_VIRT_USER_BUFFER32: {
		struct men_io_range32 range;
		struct men_io_range range64;

		if (_IOC_SIZE(cmd) != sizeof(range)) {
			warn_wrong_iosize(men, cmd, sizeof(range));
			return -EINVAL;
		}

		if (copy_from_user(&range,
			(void __user *) arg,
					sizeof(range)))
			return -EFAULT;
		if (range.length == 0)
			return -EFAULT;

		range64.start = range.start;
		range64.length = range.length;
		range64.subnr = range.subnr;
		range64.headnr = range.headnr;
		return men_create_userbuf(men, &range64);
	}
	case IOCTL_DEL_VIRT_USER_BUFFER: {
		struct men_io_bufidx32 num;
		struct menable_dmahead *dh;
		int r;

		if (_IOC_SIZE(cmd) != sizeof(num)) {
			warn_wrong_iosize(men, cmd, sizeof(num));
			return -EINVAL;
		}

		if (copy_from_user(&num,
				(void __user *) arg,
				sizeof(num)))
			return -EFAULT;

		dh = me_get_bh(men, num.headnr);

		if (dh == NULL) {
			r = -EINVAL;
		} else {
			struct menable_dmabuf *sb = me_get_sb_by_head(dh, num.index);
			r = men_free_userbuf(men, dh, num.index);
			spin_unlock_bh(&men->headlock);
			if (r == 0)
				men_destroy_sb(men, sb);
		}
		return r;
	}
	case IOCTL_FG_START_TRANSFER32:
		return men_ioctl_fgstart(men, cmd, arg);
	case IOCTL_DMA_LENGTH: {
		struct men_io_bufidx32 binfo;
		struct menable_dmabuf *sb;
		int ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(binfo))) {
			warn_wrong_iosize(men, cmd, sizeof(binfo));
			return -EINVAL;
		}

		if (copy_from_user(&binfo, (void __user *) arg,
				sizeof(binfo)))
			return -EFAULT;

		sb = me_get_sb(men, binfo.headnr, binfo.index);
		if (unlikely(sb == NULL))
			return -EINVAL;
		ret = sb->dma_length;
		spin_unlock_bh(&men->headlock);
		return ret;
	}
	case IOCTL_DMA_TAG: {
		struct men_io_bufidx32 binfo;
		struct menable_dmabuf *sb;
		int ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(binfo))) {
			warn_wrong_iosize(men, cmd, sizeof(binfo));
			return -EINVAL;
		}

		if (copy_from_user(&binfo, (void __user *) arg,
				sizeof(binfo)))
			return -EFAULT;

		sb = me_get_sb(men, binfo.headnr, binfo.index);
		if (unlikely(sb == NULL))
			return -EINVAL;
		ret = sb->dma_tag;
		spin_unlock_bh(&men->headlock);
		return ret;
	}
	case IOCTL_DMA_TIME_STAMP: {
		struct dma_timestamp32 ts;
		struct menable_dmabuf *sb;
		struct timespec tmp;
		int ret;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(ts))) {
			warn_wrong_iosize(men, cmd, sizeof(ts));
			return -EINVAL;
		}

		if (copy_from_user(&ts, (void __user *) arg, sizeof(ts)))
			return -EFAULT;

		sb = me_get_sb(men, ts.head, ts.buf);
		if (unlikely(sb == NULL))
			return -EINVAL;
		tmp = sb->timestamp;
		spin_unlock_bh(&men->headlock);

		ret = copy_to_user(((void __user *) arg) +
				offsetof(typeof(ts), stamp),
				&tmp, sizeof(tmp));
		return ret ? -EFAULT : 0;
	}
	case IOCTL_FG_WAIT_FOR_SUBBUF32: {
		struct men_io_bufwait32 ctrl;
		struct menable_dmachan *db;
		struct timespec timeout;
		int ret;
		uint64_t foundframe;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(ctrl))) {
			warn_wrong_iosize(men, cmd, sizeof(ctrl));
			return -EINVAL;
		}

		if (copy_from_user(&ctrl, (void __user *) arg, sizeof(ctrl)))
			return -EFAULT;

		if (unlikely(ctrl.index < 0))
			return -EINVAL;

		db = men_dma_channel(men, ctrl.dmachan);
		if (unlikely(db == NULL))
			return -ECHRNG;

		timeout.tv_nsec = 0;
		timeout.tv_sec = ctrl.timeout;

		ret = men_wait_dmaimg(db, ctrl.index, &timeout, &foundframe);

		if (ret < 0)
			return ret;

		ctrl.index = foundframe;

		if (copy_to_user((void __user *) arg, &ctrl, sizeof(ctrl)))
			return -EFAULT;

		if (foundframe >= INT_MAX)
			return INT_MAX;
		else
			return foundframe;
	}
	case IOCTL_UNLOCK_BUFFER_NR: {
		struct men_io_bufidx32 data;

		if (unlikely(_IOC_SIZE(cmd) != sizeof(data))) {
			warn_wrong_iosize(men, cmd, sizeof(data));
			return -EINVAL;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

		return men_ioctl_unlock_buffer(men, data.headnr, data.index);
	}
	default:
		return -ENOIOCTLCMD;
	}
}
