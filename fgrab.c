/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "menable.h"
#include "menable_ioctl.h"

int
fg_start_transfer(struct siso_menable *men, struct fg_ctrl *fgr,
		const size_t tsize)
{
	struct menable_dmachan *dc;
	struct menable_dmahead *bh;
	struct menable_dmabuf *sb;
	unsigned long flags;
	int ret = -EINVAL;

	bh = me_get_bh(men, fgr->head);
	if (bh == NULL)
		return ret;

	if ((fgr->start_buf >= bh->num_sb) || (fgr->start_buf < 0))
		goto out_err;

	sb = bh->bufs[fgr->start_buf];
	if (sb == NULL)
		goto out_err;

	if ((tsize != 0) && (sb->buf_length < tsize))
		goto out_err;

	dc = men_dma_channel(men, fgr->chan);

	if (dc == NULL) {
		ret = -ECHRNG;
		goto out_err;
	}

	if (fgr->dma_dir > 1)
		goto out_err;

	/* The channel is already running. Don't touch
	 * any of it's state variables */
	if (dc->running == 1) {
		spin_unlock_bh(&men->headlock);
		return -EBUSY;
	}

	if (fgr->mode == DMA_HANDSHAKEMODE) {
		if (fgr->dma_dir) {
			ret = -EINVAL;
			goto out_err;
		}

		if (men->dummyscale) {
			// FIXME: the channel is not locked here: race condition
			size_t dummylen = tsize ? tsize : sb->buf_length;
			ret = men->dummyscale(dc, dummylen);
			if (ret)
				goto out_err;
		}
	}

	if (dc->running == 3) {
		int run;
		struct menable_dmahead *nbh;

		/* channel is waiting for shutdown */
		spin_unlock_bh(&men->headlock);
		run = cancel_work_sync(&dc->dwork);

		nbh = me_get_bh(men, fgr->head);

		if (nbh == NULL) {
			WARN_ON(run);
			return -EBUSY;
		}

		spin_lock_irqsave(&dc->chanlock, flags);
		if (run && (dc->running == 3)) {
			dma_clean_sync(dc);
			if (dc->active) {
				dc->active->chan = NULL;
				dc->active = NULL;
			}
		}

		if ((dc->running != 2) && (dc->running != 0)) {
			spin_unlock_irqrestore(&dc->chanlock, flags);
			spin_unlock_bh(&men->headlock);
			return -EBUSY;
		}

		/* Someone has changed the buffer while we were waiting */
		if (nbh != bh) {
			spin_unlock_irqrestore(&dc->chanlock, flags);
			spin_unlock_bh(&men->headlock);
			return -EBUSY;
		}
	} else {
		spin_lock_irqsave(&dc->chanlock, flags);
	}

	if (fgr->transfer_todo == -1)
		dc->transfer_todo = LLONG_MAX;
	else
		dc->transfer_todo = fgr->transfer_todo;

	dc->timeout = min_t(unsigned long, msecs_to_jiffies(fgr->timeout * 1000), MAX_JIFFY_OFFSET);
	dc->timer.expires = jiffies + dc->timeout;

	if (fgr->dma_dir)
		dc->direction = PCI_DMA_TODEVICE;
	else
		dc->direction = PCI_DMA_FROMDEVICE;

	INIT_WORK(&dc->dwork, dma_done_work);
	dc->mode = fgr->mode;

	ret = men_start_dma(dc, bh, fgr->start_buf);
	spin_unlock_irqrestore(&dc->chanlock, flags);
	spin_unlock_bh(&men->headlock);
	return ret;
out_err:
	spin_unlock_bh(&men->headlock);
	return ret;
}

void
men_stop_dma_locked(struct menable_dmachan *dc)
{
	spin_lock(&dc->listlock);
	if (dc->transfer_todo > 0) {
		dc->transfer_todo = 0;
		spin_unlock(&dc->listlock);
		dc->running = 3;
		dc->parent->stopdma(dc->parent, dc);
		schedule_work(&dc->dwork);
	} else {
		spin_unlock(&dc->listlock);
	}
	/* Delete it here: abort has finished, the device will _not_ touch
	 * the buffer anymore. Make sure that there are no more references
	 * on it so we can free it. */
	if (dc->active)
		dc->active->chan = NULL;
	dc->active = NULL;
}

void
men_stop_dma(struct menable_dmachan *dc)
{
	unsigned long flags;

	spin_lock_bh(&dc->parent->headlock);
	spin_lock_irqsave(&dc->chanlock, flags);
	if (dc->running == 1)
		men_stop_dma_locked(dc);
	spin_unlock_irqrestore(&dc->chanlock, flags);
	spin_unlock_bh(&dc->parent->headlock);
}
