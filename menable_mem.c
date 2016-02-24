/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include "menable.h"
#include "menable_ioctl.h"
#include "linux_version.h"

/**
 * get_page_addresses - get addresses of user pages
 * @addr: start address
 * @end: end address
 * @pg: list of pages will be allocated here
 * @len: number of pages in list
 *
 * This is more or less a copy of mm/memory.c::make_pages_present
 */
static int
get_page_addresses(unsigned long addr, const size_t length, struct page ***pg)
{
	int ret, write, len;
	struct vm_area_struct *vma;

	vma = find_vma(current->mm, addr);
	if (!vma)
		return -EFAULT;
	if (vma->vm_start + length > vma->vm_end)
		return -EFAULT;
	write = (vma->vm_flags & VM_WRITE) ? 1 : 0;
	len = DIV_ROUND_UP(length + (addr % PAGE_SIZE), PAGE_SIZE);

	*pg = vmalloc(len * sizeof(*pg));
	if (!*pg)
		return -ENOMEM;
	ret = get_user_pages(current, current->mm, addr,
			     len, write, 0, *pg, NULL);
	if (ret < 0) {
		kfree(*pg);
		return ret;
	}
	return len;
}

/**
 * men_create_userbuf - do generic initialisation of user buffer
 * @men: device to use this buffer
 * @range: user address range
 *
 * Context: User context
 *
 * returns: 0 on success, negative error code otherwise
 */
int
men_create_userbuf(struct siso_menable *men, struct men_io_range *range)
{
	struct page **pages;
	struct menable_dmabuf *dmab;
	int ret = -ENOMEM, l, i;
	uint64_t len;
	struct menable_dmahead *bh;
	struct menable_dmachan *dc;

#if BITS_PER_LONG > 32
	if (range->length > 16UL * 1024UL * 1024UL * 1024UL)
		return -EINVAL;
#endif

	/* force 32 bit alignment of DMA memory */
	if ((range->start & 0x3) != 0)
		return -EINVAL;

	l = get_page_addresses(range->start, range->length, &pages);
	if (l < 0)
		return l;

	dmab = kzalloc(sizeof(*dmab), GFP_USER);
	if (!dmab)
		goto fail_dmab;

	dmab->sg = vmalloc(l * sizeof(*dmab->sg));
	if (!dmab->sg)
		goto fail_sg;
	sg_init_table(dmab->sg, l);

	dmab->dmat = kzalloc(sizeof(*dmab->dmat), GFP_USER);
	if (!dmab->dmat)
		goto fail_dmat;

	dmab->buf_length = len = range->length;
	sg_set_page(dmab->sg, pages[0],
			min((PAGE_SIZE - (range->start % PAGE_SIZE)),
					range->length),
			range->start % PAGE_SIZE);
	len -= dmab->sg[0].length;

	for (i = 1; i < l; i++) {
		sg_set_page(dmab->sg + i, pages[i], PAGE_SIZE, 0);
		len -= dmab->sg[i].length;
	}

	vfree(pages);
	pages = NULL;
	dmab->rents = l;
	dmab->sg[l - 1].length = len + PAGE_SIZE;
	BUG_ON(dmab->sg[l - 1].length > PAGE_SIZE);

	dmab->nents = pci_map_sg(men->pdev, dmab->sg, dmab->rents,
				PCI_DMA_BIDIRECTIONAL);
	if (dmab->nents == 0)
		goto fail_map;

	dmab->index = range->subnr;
	/* this will also free dmab->dmat on error */
	ret = men->create_buf(men, dmab);
	if (ret)
		goto fail_create;

	dmab->listname = FREE_LIST;
	INIT_LIST_HEAD(&dmab->node);
	ret = -EINVAL;

	bh = me_get_bh(men, range->headnr);
	if (bh == NULL)
		goto fail_bh;

	if (range->subnr >= bh->num_sb)
		goto fail_cnt;

	if (bh->bufs[range->subnr]) {
		ret = -EBUSY;
		goto fail_cnt;
	}

	bh->bufs[range->subnr] = dmab;
	/* now everything is fine. Go and add this buffer to the free list */
	dc = bh->chan;
	if (dc != NULL) {
		unsigned long flags;

		spin_lock_irqsave(&dc->listlock, flags);
		list_add_tail(&dmab->node, &dc->free_list);
		dc->free++;
		spin_unlock_irqrestore(&dc->listlock, flags);
	}
	spin_unlock_bh(&men->headlock);

	return 0;
fail_cnt:
	spin_unlock_bh(&men->headlock);
fail_bh:
	men_destroy_sb(men, dmab);
	return ret;
fail_create:
	pci_unmap_sg(men->pdev, dmab->sg, dmab->rents, PCI_DMA_BIDIRECTIONAL);
fail_map:
	for (i = dmab->rents - 1; i >= 0; i--)
		put_page(sg_page(dmab->sg + i));
fail_dmat:
	vfree(dmab->sg);
fail_sg:
	kfree(dmab);
fail_dmab:
	if (pages) {
		for (i = l - 1; i >= 0; i--)
			put_page(pages[i]);
		vfree(pages);
	}
	return ret;
}

void
men_destroy_sb(struct siso_menable *men, struct menable_dmabuf *sb)
{
	int i;

	BUG_ON(in_interrupt());

	men->free_buf(men, sb);

	pci_unmap_sg(men->pdev, sb->sg, sb->rents, PCI_DMA_BIDIRECTIONAL);
	for (i = sb->rents - 1; i >= 0; i--)
		put_page(sg_page(sb->sg + i));
	vfree(sb->sg);
	kfree(sb);
}

/**
 * men_free_userbuf - remove the DMA buffer from it's head
 * @men: board buffer belongs to
 * @db: DMA head buffer belongs to
 * @index: buffer index
 * returns: 0 on success, error code otherwise
 *
 * The caller must get and release &men->headlock if neccessary.
 */
int
men_free_userbuf(struct siso_menable *men, struct menable_dmahead *db,
		long index)
{
	struct menable_dmabuf *sb;
	struct menable_dmachan *dc;
	unsigned long flags;

	if ((index < 0) || (index > db->num_sb))
		return -EINVAL;

	sb = db->bufs[index];
	if (sb == NULL)
		return -EINVAL;

	dc = db->chan;
	if (dc == NULL) {
		/* The channel is not active: nobody but us knows about the
		 * buffer. Just kill it. */
		db->bufs[index] = NULL;
		return 0;
	}

	spin_lock_irqsave(&dc->chanlock, flags);
	spin_lock(&dc->listlock);
	if ((dc->running != 0) && (dc->running != 2) &&
			(sb->listname == HOT_LIST)) {
		/* The buffer is active, that means we would have to wait
		 * until the board is finished with it. Users problem. */
		spin_unlock(&dc->listlock);
		spin_unlock_irqrestore(&dc->chanlock, flags);
		return -EBUSY;
	}

	if (sb->listname != NO_LIST)
		list_del(&sb->node);
	switch (sb->listname) {
	case FREE_LIST:
		dc->free--;
		break;
	case GRABBED_LIST:
		dc->grabbed--;
		break;
	case NO_LIST:
		dc->locked--;
		break;
	case HOT_LIST:
		dc->hot--;
		break;
	default:
		BUG();
	}
	db->bufs[index] = NULL;
	spin_unlock(&dc->listlock);
	spin_unlock_irqrestore(&dc->chanlock, flags);

	return 0;
}

int
men_create_buf_head(struct siso_menable *men, const size_t maxsize,
		const long subbufs)
{
	struct menable_dmahead *bh, *tmp;
	int i;

	if (subbufs <= 0)
		return -EINVAL;

	bh = kzalloc(sizeof(*bh), GFP_USER);
	if (bh == NULL)
		return -ENOMEM;

	bh->bufs = kcalloc(subbufs, sizeof(*bh->bufs), GFP_USER);

	if (!bh->bufs) {
		kfree(bh);
		return -ENOMEM;
	}

	bh->num_sb = subbufs;

	i = 0;
	INIT_LIST_HEAD(&bh->node);
	spin_lock_bh(&men->headlock);

	list_for_each_entry(tmp, &men->heads, node) {
		if (tmp->id >= i)
			i = tmp->id + 1;
	}
	men->headcnt++;
	bh->id = i;
	list_add_tail(&bh->node, &men->heads);
	spin_unlock_bh(&men->headlock);
	return i;
}

int
men_release_buf_head(struct siso_menable *men, struct menable_dmahead *bh)
{
	int r;

	if (bh->chan != NULL) {
		unsigned long flags;

		spin_lock_irqsave(&bh->chan->chanlock, flags);
		if (bh->chan->running == 1) {
			r = -EBUSY;
		} else {
			r = 0;
			bh->chan->active = NULL;
		}
		spin_unlock_irqrestore(&bh->chan->chanlock, flags);
		if (r)
			return r;
	}
	list_del(&bh->node);
	men->headcnt--;
	return 0;
}

void
men_free_buf_head(struct siso_menable *men, struct menable_dmahead *bh)
{
	long i;

	BUG_ON(in_interrupt());

	for (i = 0; i < bh->num_sb; i++) {
		if (bh->bufs[i])
			men_destroy_sb(men, bh->bufs[i]);
	}
	kfree(bh->bufs);
	kfree(bh);
}

struct menable_dmabuf *
men_move_hot(struct menable_dmachan *db, const struct timespec *ts)
{
	struct menable_dmabuf *sb;

	if (db->active == NULL) {
		/* Something in the IRQ reset did not block this one.
		 * Flush them out. */
		db->lost++;
		db->transfer_todo = 0;
		return NULL;
	}

	if (db->hot != 0) {
		sb = list_first_entry(&db->hot_list, typeof(*sb), node);
		if (sb->index == -1) {
			/* this is the dummy buffer */
			list_del(&sb->node);
			db->lost++;
		} else {
			if (db->mode == DMA_HANDSHAKEMODE) {
				list_move_tail(&sb->node, &db->grabbed_list);
				db->grabbed++;
				sb->listname = GRABBED_LIST;
			} else {
				list_move_tail(&sb->node, &db->free_list);
				db->free++;
				sb->listname = FREE_LIST;
			}
			db->goodcnt++;
			db->transfer_todo--;
		}
		db->hot--;
		db->imgcnt++;
		sb->timestamp = *ts;
		pci_dma_sync_sg_for_cpu(db->parent->pdev, sb->sg, sb->nents,
				db->direction);
	} else {
		sb = NULL;
		db->lost++;
	}

	return sb;
}

struct menable_dmahead *
me_get_bh(struct siso_menable *men, const unsigned int num)
{
	struct menable_dmahead *res;

	spin_lock_bh(&men->headlock);
	list_for_each_entry(res, &men->heads, node) {
		if (res->id == num)
			return res;
	}
	spin_unlock_bh(&men->headlock);
	return NULL;
}

struct menable_dmabuf *
me_get_sb_by_head(struct menable_dmahead *head, const long bufidx)
{
	if (head == NULL)
		return NULL;

	if ((bufidx < 0) || (bufidx >= head->num_sb))
		return NULL;

	return head->bufs[bufidx];

}

struct menable_dmabuf *
me_get_sb(struct siso_menable *men, const unsigned int headnum,
		const long bufidx)
{
	struct menable_dmahead *head = me_get_bh(men, headnum);
	struct menable_dmabuf *res;

	if (!head)
		return NULL;

	res = me_get_sb_by_head(head, bufidx);

	if (res == NULL)
		spin_unlock_bh(&men->headlock);

	return res;
}

void
me_queue_dma(struct menable_dmachan *dc, const unsigned int max)
{
	struct menable_dmabuf *sb;

	if ((dc->mode == DMA_HANDSHAKEMODE) && (dc->free == 0)) {
		/* if there are still buffers in queue simply do nothing */
		if (dc->hot == 0) {
			sb = dc->parent->dummybuf(dc);
			INIT_LIST_HEAD(&sb->node);
			dc->parent->queue_sb(dc, sb);
			list_add(&sb->node, &dc->hot_list);
			dc->hot++;
		}
	} else {
		unsigned int i;

		WARN_ON(!dc->free);
		i = min(max, dc->free);

		while (i--) {
			sb = list_first_entry(&dc->free_list,
					struct menable_dmabuf, node);

			pci_dma_sync_sg_for_device(dc->parent->pdev, sb->sg,
					sb->nents, dc->direction);

			dc->parent->queue_sb(dc, sb);

			list_move_tail(&sb->node, &dc->hot_list);
			dc->free--;
			dc->hot++;
			sb->listname = HOT_LIST;
		}
	}
}

struct menable_dmabuf *
men_next_blocked(struct siso_menable *men, struct menable_dmachan *dc)
{
	struct menable_dmabuf *ret;

	if (dc->grabbed == 0) {
		ret = NULL;
	} else {
		ret = list_first_entry(&dc->grabbed_list, typeof(*ret), node);
		list_del(&ret->node);
		dc->grabbed--;
		dc->locked++;
		ret->listname = NO_LIST;
	}

	return ret;
}

struct menable_dmabuf *
men_last_blocked(struct siso_menable *men, struct menable_dmachan *dc)
{
	struct menable_dmabuf *ret;

	if (dc->grabbed == 0) {
		ret = NULL;
	} else {
		while (dc->grabbed > 1) {
			ret = list_first_entry(&dc->grabbed_list,
					struct menable_dmabuf, node);
			list_move_tail(&ret->node, &dc->free_list);
			dc->free++;
			dc->grabbed--;
			ret->listname = FREE_LIST;
		}
		ret = list_first_entry(&dc->grabbed_list,
				struct menable_dmabuf, node);
		list_del(&ret->node);
		dc->grabbed = 0;
		dc->locked++;
		ret->listname = NO_LIST;
	}

	return ret;
}

void
men_unblock_buffer(struct menable_dmachan *dc, struct menable_dmabuf *sb)
{
	if (sb == NULL)
		return;

	switch (sb->listname) {
	case HOT_LIST:
	case FREE_LIST:
		return;
	case GRABBED_LIST:
		sb->listname = FREE_LIST;
		list_move_tail(&sb->node, &dc->free_list);
		dc->grabbed--;
		dc->free++;
		break;
	case NO_LIST:
		sb->listname = FREE_LIST;
		INIT_LIST_HEAD(&sb->node);
		list_add_tail(&sb->node, &dc->free_list);
		dc->locked--;
		dc->free++;
		break;
	default:
		BUG();
	}
}
