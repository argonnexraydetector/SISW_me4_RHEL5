/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "menable4.h"
#include "menable.h"
#include "menable_ioctl.h"
#include "uiq.h"

#include "linux_version.h"

static DEVICE_ATTR(design_crc, 0660, men_get_des_val, men_set_des_val);

static ssize_t
men_get_boardinfo(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);
	ssize_t ret = 0;
	int i;

	for (i = 0; i < 4; i++) {
		u32 tmp = ioread32(men->runtime_base + 4 * i);
		ssize_t r = sprintf(buf + ret, "0x%08x\n", tmp);

		if (r < 0)
			return r;

		ret += r;
	}

	return ret;
}

static DEVICE_ATTR(board_info, 0440, men_get_boardinfo, NULL);

/**
 * me4_fpga_control - return the base of the per-FPGA control registers
 * @men: board
 * @fpga: fpga index
 */
static void __iomem *
me4_fpga_control(struct siso_menable *men, const unsigned int fpga)
{
	BUG_ON(fpga >= MAX_FPGAS);
	BUG_ON((men == NULL) || (men->runtime_base == NULL));

	return men->runtime_base + fpga * ME4_FULLOFFSET;
}

/**
 * me4_add_uiqs - scan for new UIQs in one FPGA
 * @men: board to scan
 * @fpga: fpga index to scan
 * @count: how many UIQs to add
 * @uiqoffs: register offset of the first UIQ
 *
 * This will keep all UIQs from lower FPGAs.
 */
static int
me4_add_uiqs(struct siso_menable *men, const unsigned int fpga,
		const unsigned int count, const unsigned int uiqoffs)
{
	struct menable_uiq **nuiqs;
	unsigned int i;
	int ret;
	uint32_t uiqtype = ioread32(me4_fpga_control(men, fpga) + ME4_IRQTYPE);

	WARN_ON(!men->design_changing);
	for (i = fpga; i < MAX_FPGAS; i++)
		WARN_ON(men->uiqcnt[i] != 0);

	nuiqs = kcalloc(fpga * MEN_MAX_UIQ_PER_FPGA + count,
			sizeof(*men->uiqs), GFP_KERNEL);

	if (nuiqs == NULL)
		return -ENOMEM;

	for (i = 0; i < fpga; i++) {
		unsigned int j;
		for (j = 0; j < men->uiqcnt[i]; j++)
			nuiqs[i * MEN_MAX_UIQ_PER_FPGA + j] =
					men->uiqs[i * MEN_MAX_UIQ_PER_FPGA + j];
	}

	for (i = 0; i < count; i++) {
		struct menable_uiq *uiq;
		void __iomem *basereg = men->runtime_base + uiqoffs;
		int chan = fpga * MEN_MAX_UIQ_PER_FPGA + i;
		int t = uiqtype & (1 << (i + ME4_IRQQ_LOW));

		uiq = men_uiq_init(chan, basereg + 8 * i, men, t, 16);

		if (IS_ERR(uiq)) {
			int j;
			ret = PTR_ERR(uiq);

			for (j = fpga * MEN_MAX_UIQ_PER_FPGA; j < chan; j++)
				men_uiq_remove(nuiqs[j]);

			kfree(nuiqs);
			return ret;
		}

		uiq->irqack = me4_fpga_control(men, fpga) + ME4_IRQACK;
		uiq->ackbit = i + ME4_IRQQ_LOW;

		nuiqs[chan] = uiq;
	}

	kfree(men->uiqs);
	men->uiqs = nuiqs;
	men->uiqcnt[fpga] = count;

	return 0;
}

/**
 * men4_reset_vlink - reset link between bridge FPGA and upper FPGA
 * @men: board to reset
 * @upper: reset upper FPGA part or not
 * returns: 0 on success, error code else
 */
static int
men4_reset_vlink(struct siso_menable *men, const bool upper)
{
	uint32_t cplid;
	int i;
	void __iomem *ifctrl = me4_fpga_control(men, 0) + ME4_IFCONTROL;
	void __iomem *uifctrl = me4_fpga_control(men, 1) + ME4_IFCONTROL;

	/* multiple register accesses are here to
	 * ensure the value passes the register
	 * pipeline */

	iowrite32(2, ifctrl);
	iowrite32(0, ifctrl);
	iowrite32(0, ifctrl);

	if (!upper)
		return 0;

	for (i = 0; i < 5; i++)
		iowrite32(1, ifctrl);

	cplid = ioread32(men->runtime_base + ME4_FPGACONTROL) & 0xffff0000;
	iowrite32(cplid | 0x2, uifctrl);
	for (i = 0; i < 4; i++)
		iowrite32(cplid, uifctrl);

	for (i = ME4_PCIECONFIG0; i < ME4_PCIECONFIGMAX; i += 4) {
		uint32_t v = ioread32(me4_fpga_control(men, 0) + i);
		iowrite32(v, me4_fpga_control(men, 1) + i);
	}

	iowrite32(cplid | 0x1, uifctrl);

	return 0;
}

static int
men4_ioctl(struct siso_menable *men, const unsigned int cmd,
		const unsigned int size, unsigned long arg)
{
	switch (cmd) {
	case IOCTL_BOARD_INFO: {
		unsigned int a[4];
		int i;

		if (size != sizeof(a)) {
			warn_wrong_iosize(men, cmd, sizeof(a));
			return -EINVAL;
		}

		for (i = 0; i < ARRAY_SIZE(a); i++)
			a[i] = ioread32(men->runtime_base + 4 * i);
		if (copy_to_user((void __user *) arg,
				a, sizeof(a)))
			return -EFAULT;
		return 0;
	}
	case IOCTL_PP_CONTROL: {
		int ret;
		unsigned long flags;

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
			spin_lock_bh(&men->headlock);
			if (men->active_fpgas > 1)
				iowrite32(0, me4_fpga_control(men, 1) +
						ME4_IFCONTROL);
			iowrite32(0, me4_fpga_control(men, 0) + ME4_IFCONTROL);
			men_del_uiqs(men, 1);
			men->active_fpgas = 1;

			ret = men_alloc_dma(men, men->dmacnt[0]);
			break;
		case 1:
			/* DCM reset */
			iowrite32(0x10, men->runtime_base + ME4_IFCONTROL);
			udelay(10);
			iowrite32(0, men->runtime_base + ME4_IFCONTROL);
			iowrite32(0, men->runtime_base + ME4_IFCONTROL);
			msleep(5);
			if (ioread32(men->runtime_base + ME4_IFCONTROL) & 0x10)
				return -EBUSY;

			ret = men4_reset_vlink(men, true);
			if (ret == 0) {
				men->active_fpgas = 2;

				ret = men_add_dmas(men);
				if (ret == 0) {
					unsigned int cnt = ioread32(me4_fpga_control(men, 1) + ME4_UIQCNT);
					unsigned int offs = ioread32(me4_fpga_control(men, 1) + ME4_FIRSTUIQ);
					ret = me4_add_uiqs(men, 1, cnt, offs);
				}
			}
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
	default:
		return -ENOIOCTLCMD;
	}
}

static void
me4_free_sgl(struct siso_menable *men, struct menable_dmabuf *sb)
{
	struct men_dma_chain *res = sb->dmat;
	dma_addr_t dma = sb->dma;

	while (res) {
		struct men_dma_chain *n;
		dma_addr_t ndma;

		n = res->next;
		ndma = (dma_addr_t) (le64_to_cpu(res->pcie4->next) & ~(3ULL));
		if (dma == ndma)
			break;
		dma_pool_free(men->pool, res->pcie4, dma);
		kfree(res);
		res = n;
		dma = ndma;
	}
}

static void
me4_queue_sb(struct menable_dmachan *db, struct menable_dmabuf *sb)
{
	w64(sb->buf_length / 4, db->iobase + ME4_DMAMAXLEN);
	wmb();
	w64(sb->dma, db->iobase + ME4_DMAADDR);
	wmb();
}

static irqreturn_t
me4_irq(int irq, void *dev_id)
{
	uint32_t sr;
	struct siso_menable *men = dev_id;
	unsigned int fpga;
	unsigned char skipdma = 0;	/* DMA channels in lower FPGAs */
	unsigned char skipuiq = 0;	/* UIQs in lower FPGAs */
	struct timespec ts;
	bool have_ts = false;

	if (pci_channel_offline(men->pdev))
		return IRQ_HANDLED;

	sr = ioread32(men->runtime_base + ME4_IRQSTATUS);

	if (unlikely(sr == 0))
		return IRQ_NONE;

	for (fpga = 0; fpga < men->active_fpgas; fpga++) {
		int dma;
		uint32_t badmask = 0;
		uint32_t st;	/* tmp status */

		if (unlikely(sr == 0xffffffff)) {
			dev_warn(&men->dev, "IRQ status register %i read returned -1\n", fpga);
			iowrite32(0, me4_fpga_control(men, fpga) + ME4_IRQENABLE);
			iowrite32(0xffffffff, me4_fpga_control(men, fpga) + ME4_IRQACK);
			return IRQ_HANDLED;
		}

		spin_lock(&men->d4->irqmask_lock);
		badmask = sr & ~men->d4->irq_wanted[fpga];
		if (unlikely(badmask != 0)) {
			iowrite32(men->d4->irq_wanted[fpga],
					me4_fpga_control(men, fpga) + ME4_IRQENABLE);
			iowrite32(badmask, me4_fpga_control(men, fpga) + ME4_IRQACK);
			sr &= men->d4->irq_wanted[fpga];
		}
		spin_unlock(&men->d4->irqmask_lock);

		for (dma = 0; dma < men->dmacnt[fpga]; dma++) {
			struct menable_dmachan *db;
			void __iomem *dmabase;
			void __iomem *lenaddr;
			void __iomem *tagaddr;
			struct menable_dma_wait *waitstr;
			uint32_t ic, delta;
			int i;

			if ((sr & (0x1 << dma)) == 0)
				continue;

			if (!have_ts) {
				have_ts = true;
				ts = current_kernel_time();
			}

			db = men_dma_channel(men, dma + skipdma);
			BUG_ON(db == NULL);
			dmabase = db->iobase;
			lenaddr = dmabase + ME4_DMALENGTH;
			tagaddr = dmabase + ME4_DMATAG;

			spin_lock(&db->chanlock);
			iowrite32(1 << db->ackbit, db->irqack);
			ic = ioread32(dmabase + ME4_DMACOUNT);
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
				struct menable_dmabuf *sb = men_move_hot(db, &ts);
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
				unsigned int sbcnt = min(
						ME4_DMA_FIFO_DEPTH - db->hot,
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

		st = (sr & 0x3fff0000);
		if (st != 0) {
			uint32_t bit;
			for (bit = ME4_IRQQ_LOW; (bit <= ME4_IRQQ_HIGH) && st; bit++) {
				if (st & (1 << bit)) {
					uiq_irq(men->uiqs[bit - ME4_IRQQ_LOW + skipuiq], &ts, &have_ts);
					st ^= (1 << bit);
				}
			}
		}

		if (sr & ME4_CHAIN_MASK) {
			WARN_ON(fpga == men->active_fpgas - 1);

			if (likely(fpga < men->active_fpgas - 1)) {
				sr = ioread32(me4_fpga_control(men, fpga + 1) + ME4_IRQSTATUS);
			} else {
				sr = 0;
			}

			iowrite32(ME4_CHAIN_MASK, me4_fpga_control(men, fpga) + ME4_IRQACK);

			skipdma += men->dmacnt[fpga];
			skipuiq += men->uiqcnt[fpga];
		} else {
			break;
		}
	}

	return IRQ_HANDLED;
}

static void
men4_abort_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
	iowrite32(2, dc->iobase + ME4_DMACTRL);
	wmb();
	iowrite32(0, dc->iobase + ME4_DMACTRL);
	wmb();
}

static void
men4_stop_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
	uint32_t irqreg;
	unsigned long flags;
	unsigned char fpga;

	irqreg = ioread32(dc->irqenable);
	irqreg &= ~(1 << dc->enablebit);
	iowrite32(irqreg, dc->irqenable);

	iowrite32(0, dc->iobase + ME4_DMACTRL);
	wmb();

	spin_lock_irqsave(&men->d4->irqmask_lock, flags);
	fpga = dc->fpga;
	men->d4->irq_wanted[fpga] &= ~(1 << dc->enablebit);
	while ((fpga > 0) && (men->d4->irq_wanted[fpga] == 0)) {
		fpga--;
		men->d4->irq_wanted[fpga] &= ~ME4_CHAIN_MASK;
	}
	spin_unlock_irqrestore(&men->d4->irqmask_lock, flags);
}

static int
me4_create_userbuf(struct siso_menable *men, struct menable_dmabuf *db)
{
	struct men_dma_chain *cur;
	int i;

	db->dmat->pcie4 = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
	if (!db->dmat->pcie4)
		goto fail_pcie;
	memset(db->dmat->pcie4, 0, sizeof(*db->dmat->pcie4));

	cur = db->dmat;

	for (i = 0; i < db->nents; i++) {
		int idx = i % ARRAY_SIZE(cur->pcie4->addr);

		cur->pcie4->addr[idx] =
				cpu_to_le64(sg_dma_address(db->sg + i) + 0x1);

		if ((idx == ARRAY_SIZE(cur->pcie4->addr) - 1) &&
						(i + 1 < db->nents)) {
			dma_addr_t next;

			cur->next = kzalloc(sizeof(*cur->next), GFP_USER);
			if (!cur->next)
				goto fail;

			cur->next->pcie4 = dma_pool_alloc(men->pool,
					GFP_USER, &next);
			if (!cur->next->pcie4) {
				kfree(cur->next);
				cur->next = NULL;
				goto fail;
			}
			cur->pcie4->next = cpu_to_le64(next + 0x2);
			cur = cur->next;
			memset(cur->pcie4, 0, sizeof(*cur->pcie4));
		}
	}
	cur->pcie4->next = men->d4->dummybuf.dmat->pcie4->next;

	return 0;
fail:
	me4_free_sgl(men, db);
	return -ENOMEM;
fail_pcie:
	kfree(db->dmat);
	return -ENOMEM;
}

static int
men4_create_dummybuf(struct siso_menable *men)
{
	struct men_dma_chain *cur;
	struct menable_dmabuf *db = &men->d4->dummybuf;
	int i;
	dma_addr_t pagedma;

	db->index = -1;
	db->dmat = kzalloc(sizeof(*db->dmat), GFP_KERNEL);
	if (!db->dmat)
		goto fail_dmat;

	db->dmat->pcie4 = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
	if (!db->dmat->pcie4)
		goto fail_pcie;
	memset(db->dmat->pcie4, 0, sizeof(*db->dmat->pcie4));

	men->d4->dummypage = pci_alloc_consistent(men->pdev, 4096, &pagedma);
	if (men->d4->dummypage == NULL)
		goto fail_page;

	cur = db->dmat;

	for (i = 0; i < ARRAY_SIZE(cur->pcie4->addr); i++)
		cur->pcie4->addr[i] = cpu_to_le64(pagedma + 0x1);

	cur->pcie4->next = cpu_to_le64(db->dma + 0x2);
	
	db->buf_length = -1;
	
	return 0;
fail_page:
	dma_pool_free(men->pool, db->dmat->pcie4, db->dma);
fail_pcie:
	kfree(db->dmat);
fail_dmat:
	return -ENOMEM;
}

static void
men4_destroy_dummybuf(struct siso_menable *men)
{
	uint64_t pg = le64_to_cpu(men->d4->dummybuf.dmat->pcie4->addr[0]) & ~(3ULL);
	dma_addr_t dmaaddr = (dma_addr_t) pg;

	pci_free_consistent(men->pdev, 4096, men->d4->dummypage, dmaaddr);
	dma_pool_free(men->pool, men->d4->dummybuf.dmat->pcie4,
					men->d4->dummybuf.dma);
	kfree(men->d4->dummybuf.dmat);
}

static void
me4_exit(struct siso_menable *men)
{
	men4_destroy_dummybuf(men);
	kfree(men->uiqs);
	kfree(men->d4);
}

static unsigned int
me4_query_dma(struct siso_menable *men, const unsigned int fpga)
{
	uint32_t u;

	BUG_ON(fpga >= men->active_fpgas);

	u = ioread32(me4_fpga_control(men, fpga) + ME4_NUMDMA);
	if (unlikely(u == 0xffffffff)) {
		dev_warn(&men->dev,
			"Reading DMACNT from FPGA %i failed\n", fpga);
		u = 0;
	} else {
		dev_dbg(&men->dev, "%i DMA channels detected in FPGA %i\n",
				u, fpga);
	}

	return u;
}

static int
men4_startdma(struct menable_dmachan *dmac)
{
	uint32_t tmp, dir;
	unsigned long flags;
	unsigned char fpga;
	struct siso_menable *men = dmac->parent;

	men4_abort_dma(men, dmac);

	dir = (dmac->direction == PCI_DMA_TODEVICE) ? 2 : 1;

	tmp = ioread32(dmac->iobase + ME4_DMATYPE);
	if (!(tmp & dir))
		return -EACCES;
	iowrite32(dir, dmac->iobase + ME4_DMATYPE);

	/* clear IRQ */
	iowrite32(1 << dmac->ackbit, dmac->irqack);

	dmac->imgcnt = ioread32(dmac->iobase + ME4_DMACOUNT);

	me_queue_dma(dmac, min(dmac->transfer_todo, ME4_DMA_FIFO_DEPTH));

	spin_lock_irqsave(&men->d4->irqmask_lock, flags);
	men->d4->irq_wanted[dmac->fpga] |= (1 << dmac->enablebit);
	iowrite32(men->d4->irq_wanted[dmac->fpga], dmac->irqenable);
	for (fpga = 0; fpga < dmac->fpga; fpga++) {
		if ((men->d4->irq_wanted[fpga] & ME4_CHAIN_MASK) == 0) {
			men->d4->irq_wanted[fpga] |= ME4_CHAIN_MASK;
			iowrite32(men->d4->irq_wanted[fpga],
					me4_fpga_control(men, fpga) + ME4_IRQENABLE);
		}
	}
	spin_unlock_irqrestore(&men->d4->irqmask_lock, flags);

	iowrite32(1, dmac->iobase + ME4_DMAACTIVE);
	ioread32(dmac->iobase + ME4_DMAACTIVE);
	iowrite32(0, dmac->iobase + ME4_DMAACTIVE);
	ioread32(dmac->iobase + ME4_DMAACTIVE);
	iowrite32(1, dmac->iobase + ME4_DMACTRL);
	ioread32(dmac->iobase + ME4_DMACTRL);

	return 0;
}

static void
men4_dmabase(struct siso_menable *men, struct menable_dmachan *dc)
{
	void __iomem *addrbase = me4_fpga_control(men, dc->fpga);
	unsigned int skipdma = 0;
	unsigned int i;

	for (i = 0; i < dc->fpga; i++)
		skipdma +=men->dmacnt[i];

	dc->ackbit = dc->number - skipdma;

	dc->iobase =  addrbase + ME4_DMAOFFS + ME4_DMASZ * dc->ackbit;
	dc->irqack = addrbase + ME4_IRQACK;
	dc->irqenable = addrbase + ME4_IRQENABLE;
	dc->enablebit = dc->ackbit;
}

static void
men4_stopirq(struct siso_menable *men)
{
	unsigned int i;

	for (i = men->active_fpgas; i > 0; i--) {
		void __iomem *addrbase = me4_fpga_control(men, i - 1);
		unsigned int skipuiq = MEN_MAX_UIQ_PER_FPGA * (i - 1);
		int j;

		for (j = 0; j < men->uiqcnt[i]; j++) {
			if (men->uiqs[j + skipuiq] == NULL)
				continue;
			men->uiqs[j + skipuiq]->running = false;
		}

		iowrite32(0, addrbase + ME4_IRQENABLE);
		iowrite32(0xffffffff, addrbase + ME4_IRQACK);
		iowrite32(0, addrbase + ME4_IFCONTROL);
	}
	men->active_fpgas = 1;

	for (i = 0; i < men->uiqcnt[0]; i++) {
		if (men->uiqs[i] == NULL)
			continue;
		men->uiqs[i]->running = false;
	}
}

static void
men4_startirq(struct siso_menable *men)
{
	uint32_t mask = ((1 << men->uiqcnt[0]) - 1) << ME4_IRQQ_LOW;
	unsigned int i;

	for (i = 1; i < MAX_FPGAS; i++)
		men->d4->irq_wanted[i] = 0;
	men->d4->irq_wanted[0] = mask;
	iowrite32(0xffffffff, men->runtime_base + ME4_IRQACK);
	iowrite32(mask, men->runtime_base + ME4_IRQENABLE);
}

/**
 * men4_reset_core - reset state machines near the PCIe core
 * @men: board to reset
 *
 * This will reset the state machines and logic directly connected to the
 * PCIe core.
 */
static void
men4_reset_core(struct siso_menable *men)
{
	int i;

	iowrite32(0xa, men->runtime_base + ME4_IFCONTROL);
	iowrite32(0xa, men->runtime_base + ME4_IFCONTROL);
	for (i = 0; i < 4; i++)
		iowrite32(0x8, men->runtime_base + ME4_IFCONTROL);
	for (i = 0; i < 6; i++)
		iowrite32(0, men->runtime_base + ME4_IFCONTROL);

	men4_reset_vlink(men, false);
}

static void
me4_cleanup(struct siso_menable *men)
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
me4_dummybuf(struct menable_dmachan *dc)
{
	return &dc->parent->d4->dummybuf;
}

static struct lock_class_key me4_irqmask_lock;

static struct attribute *me4_attributes[6] = {
	&dev_attr_design_crc.attr,
	&dev_attr_board_info.attr,
	NULL
};

static struct attribute_group me4_attribute_group = {
        .attrs = me4_attributes
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
const
#endif /* LINUX >= 2.6.31 */
struct attribute_group *me4_attribute_groups[2] = {
	&me4_attribute_group,
	NULL
};

int
me4_probe(struct siso_menable *men)
{
	int ret = -ENOMEM;
	unsigned int uiqoffs;
	unsigned int uiqcnt;

	men->d4 = kzalloc(sizeof(*men->d4), GFP_KERNEL);
	if (men->d4 == NULL)
		goto fail;

	spin_lock_init(&men->d4->irqmask_lock);
	lockdep_set_class(&men->d4->irqmask_lock, &me4_irqmask_lock);

	men->active_fpgas = 1;

	men4_reset_core(men);
	men4_stopirq(men);

	if (pci_set_dma_mask(men->pdev, DMA_BIT_MASK(64))) {
		dev_err(&men->dev, "No suitable DMA available.\n");
		goto fail_mask;
	}
	pci_set_consistent_dma_mask(men->pdev, DMA_BIT_MASK(64));
	men->pool = dmam_pool_create("me4_sgl", &men->pdev->dev,
			sizeof(struct me4_sgl), 128, 4096);
	if (!men->pool) {
		dev_err(&men->dev, "can not allocate DMA pool\n");
		goto fail_pool;
	}

	ret = men4_create_dummybuf(men);
	if (ret) {
		dev_err(&men->dev, "can not allocate dummy buffer\n");
		goto fail_dummy;
	}

	men->create_buf = me4_create_userbuf;
	men->free_buf = me4_free_sgl;
	men->startdma = men4_startdma;
	men->abortdma = men4_abort_dma;
	men->stopdma = men4_stop_dma;
	men->stopirq = men4_stopirq;
	men->startirq = men4_startirq;
	men->ioctl = men4_ioctl;
	men->exit = me4_exit;
	men->cleanup = me4_cleanup;
	men->query_dma = me4_query_dma;
	men->dmabase = men4_dmabase;
	men->queue_sb = me4_queue_sb;
	men->dummybuf = me4_dummybuf;

	uiqcnt = ioread32(men->runtime_base + ME4_UIQCNT);
	uiqoffs = ioread32(men->runtime_base + ME4_FIRSTUIQ);

	if ((uiqcnt == 0) && (uiqoffs == 0)) {
		/* old firmware versions did not provide this */
		uiqcnt = ME4_IRQQ_HIGH - ME4_IRQQ_LOW + 1;
		uiqoffs = ME4_IRQQUEUE;
	}

	if (uiqcnt != 0) {
		ret = me4_add_uiqs(men, 0, uiqcnt, uiqoffs);
		if (ret != 0)
			goto fail_uiqs;
	}

#if 0
	ret = pci_enable_msi(men->pdev);
	if (ret)
		dev_info(&men->dev, "can't enable MSI\n");
#endif

	men->desname = men->d4->design_name;
	men->deslen = sizeof(men->d4->design_name);

	ret = devm_request_irq(&men->pdev->dev, men->pdev->irq, me4_irq,
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
	men4_destroy_dummybuf(men);
fail_dummy:
fail_pool:
fail_mask:
	kfree(men->d4);
fail:
	return ret;
}
