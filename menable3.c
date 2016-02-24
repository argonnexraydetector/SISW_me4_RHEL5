/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */

/* the forward declaration is missing in dmapool.h until at least 2.6.37-rc5 */
struct device;

#include <linux/delay.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/pfn.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include "menable.h"
#include "menable3.h"
#include "menable_ioctl.h"
#include "plx.h"
#include "uiq.h"

#include "linux_version.h"

static DEVICE_ATTR(design_clk, 0660, men_get_des_val, men_set_des_val);

static void
toggle24(void __iomem *reg)
{
	uint32_t data;

	data = ioread32(reg);
	data &= ~(1 << 24);
	iowrite32(data, reg);

	data = ioread32(reg);
	data |= (1 << 24);
	iowrite32(data, reg);

	udelay(2);
}

static void
me3_ee_read(struct siso_menable *men, struct men_eeprom *buf)
{
	uint32_t val;
	uint32_t data;
	int i, word;
	void __iomem *reg = men->runtime_base + 0x6c;

	/* chipselect the EEPROM */
	val = ioread32(reg) | (1 << 25);
	iowrite32(val, reg);

	/* write READ instruction into EEPROM */
	for (i = 0; i < 11; i++) {
		uint32_t value;

		data = (i < 2) ? 1 : 0;

		value = ioread32(reg) & ~(1 << 26);
		value |= (data << 26);

		iowrite32(value, reg);

		toggle24(reg);
	}

	data = ioread32(reg);
	data |= (1 << 31);
	iowrite32(data, reg);

	/* now the sequential readout of the EEPROM starts */
	for (word = 0; word < 35; word++) {
		int bit;
		uint32_t u, value;

		u = 0;
		for (bit = 31; bit >= 0; bit--) {
			toggle24(reg);

			value = ioread32(reg) & (1 << 27);

			u <<= 1;
			if (value)
				u |= 1;
		}
		buf->buf[word] = u;
	}
	data = ioread32(reg);
	data &= ~(1 << 31);
	iowrite32(data, reg);

	/* turn chip select CS of EEPROM off */
	data = ioread32(reg) & ~(1 << 25);
	iowrite32(data, reg);
}

static ssize_t
men_get_eeprom(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);
	ssize_t ret = 0;
	struct men_eeprom content;
	int i;

	me3_ee_read(men, &content);

	for (i = 0; i < ARRAY_SIZE(content.buf); i++) {
		ssize_t r = sprintf(buf + ret, "0x%08x\n", content.buf[i]);

		if (r < 0)
			return r;

		ret += r;
	}

	return ret;
}

static DEVICE_ATTR(eeprom, 0440, men_get_eeprom, NULL);

/**
 * plx_abort_dma - abort DMA transfer on given channel
 * @men: board to stop
 * @chan: DMA channel number
 *
 * Context: must be called from IRQ context. PLX is chip's abort is weird,
 *          no way to do this better.
 *
 * returns: 0 on success, 1 on error
 */
static int
plx_abort_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
	int wait;
	uint32_t old;
	unsigned char ch;

	/* disable channel */
	ch = ioread8(dc->iobase);
	iowrite8(ch & 0xfe, dc->iobase);

	if (ch & 0x10) {
		/* shut up IRQ */
		iowrite8(0x8, dc->iobase);
		if (dc->irqack != NULL)
			iowrite32(0x1 << dc->ackbit, dc->irqack);
		return 0;
	}

	wmb();
	/* give PLX some time to settle down */
	for (wait = 2; wait > 0; wait--) {
		read_barrier_depends();
		ch = ioread8(dc->iobase);

		if (ch & 0x10) {
			/* shut up IRQ */
			iowrite8(0x8, dc->iobase);
			if (dc->irqack != NULL)
				iowrite32(0x1 << dc->ackbit, dc->irqack);
			return 0;
		}
	}

	iowrite8(ch | 0x4, dc->iobase);
	wmb();

	wait = 20;
	while (wait > 0) {
		uint32_t flag = 0x200000 << dc->number;

		read_barrier_depends();
		ch = ioread8(dc->iobase);
		if (ch & 0x10)
			wait = 0;

		wait--;
		read_barrier_depends();
		old = ioread32(men->runtime_base + PLX_INT_CSR);
		if (old & flag) {		/* INT active? */
			iowrite8(0x8, dc->iobase);	/* clear */
			wmb();
		}
	}
	if (wait == 0) {
		read_barrier_depends();
		old = ioread32(men->runtime_base + PLX_INT_CSR);
		iowrite32(old & 0xfffff7ff, men->runtime_base + PLX_INT_CSR);
		dev_dbg(&men->dev, "transfer abort on "
						"channel %i\n", dc->number);
	}
	if (dc->irqack != NULL)
		iowrite32(0x1 << dc->ackbit, dc->irqack);
	return !wait;
}

static void
me3_queue_sb(struct menable_dmachan *dc, struct menable_dmabuf *sb)
{
	void __iomem *runtime_base = dc->parent->runtime_base;
	unsigned int offs;
	uint32_t addr;

	offs = (dc->number == 0) ? 0x80 : 0x94;

	iowrite32(0, runtime_base + offs + 0x4);
	iowrite32(64, runtime_base + offs + 0xc); /* DMA size */

	addr = sb->dma | 0x1;
	if (dc->direction == PCI_DMA_FROMDEVICE)
		addr |= 0x8;
	iowrite32(addr, runtime_base + offs + 0x10);
}

static void
me3_dma_irq(struct siso_menable *men, const unsigned int chan, const struct timespec *ts)
{
	struct menable_dmachan *db = men_dma_channel(men, chan);
	struct menable_dmabuf *sb;
	int off2;
	uint32_t old;
	struct menable_dma_wait *waitstr;

	BUG_ON(db == NULL);
	if (chan == 0)
		off2 = 0x80;
	else
		off2 = 0x94;

	spin_lock(&db->listlock);
	sb = men_move_hot(db, ts);
	/* Acquired image is saved.
	 * Now get statistical data of this transfer. */

	if (sb) {
		sb->dma_length = ioread32(men->d3->design_base +
				men->d3->design.dma_len[chan]);
		if (men->d3->design.dmatag[chan])
			sb->dma_tag = ioread32(men->d3->design_base +
					men->d3->design.dmatag[chan]);
	}
	iowrite32(0x1 << db->ackbit, db->irqack);
	/* if transmission isn't finished, abort */

	old = ioread8(db->iobase);
	/* abort needed? */
	if ((old & 0x10) == 0) {
		if (plx_abort_dma(men, db)) {
			spin_unlock(&db->listlock);
			del_timer(&db->timer);
			return;
		}
	}
	iowrite8(0x8, db->iobase);	/* clear */

	list_for_each_entry(waitstr, &db->wait_list, node) {
		if (waitstr->frame <= db->goodcnt)
			complete(&waitstr->cpl);
	}

	if (unlikely(db->transfer_todo <= 0)) {
		spin_unlock(&db->listlock);
		db->running = 3;
		schedule_work(&db->dwork);
	} else {
		/* (re)start DMA Channel */
		iowrite32(0x00021fc3, men->runtime_base + off2);

		me_queue_dma(db, 1);

		iowrite8(1, db->iobase);	/* enable */
		wmb();
		iowrite8(3, db->iobase);	/* start xfer */
		spin_unlock(&db->listlock);

		/* restart design */
		iowrite32(1 << db->enablebit, db->irqenable);
		mod_timer(&db->timer, jiffies + db->timeout);
	}
}

static void
men3_stopirq(struct siso_menable *men)
{
	iowrite32(0, men->runtime_base + PLX_INT_CSR);
}

static irqreturn_t
me3_irq(int irq, void *dev_id)
{
	uint32_t sr, ir;
	struct siso_menable *men = dev_id;
	unsigned int i;
	struct timespec ts;
	bool have_ts = false;

	spin_lock(&men->boardlock);

	sr = ioread32(men->runtime_base + PLX_INT_CSR);
	/* fix up dorky logic of PLX chip: bits 24..27 are active low while
	 * all others are active high */
	sr ^= 0xf000000;
	/* mask off the enable/disable bits, they don't care as status */
	sr &= ~ME3_IRQMASK;

	if (unlikely(sr == 0)) {
		spin_unlock(&men->boardlock);
		return IRQ_NONE;
	}

	if (sr & PLX_IRQ_PARITY_ERR) {
		// FIXME: deaktivieren
//		dev_warn(&men->dev, "parity error\n");
		sr ^= PLX_IRQ_PARITY_ERR;
		iowrite32(PLX_IRQ_PARITY_ERR | ME3_IRQMASK, men->runtime_base + 0x68);
		if (sr == 0) {
			spin_unlock(&men->boardlock);
			return IRQ_HANDLED;
		}
	}

	if (unlikely(sr & ~ME3_IRQ_INP)) {
		dev_err(&men->dev,
				"unexpected irq (status 0x%x)\n", sr);
		iowrite32(ME3_IRQMASK, men->runtime_base + PLX_INT_CSR);
		sr &= ME3_IRQ_INP;
		if (sr == 0) {
			spin_unlock(&men->boardlock);
			return IRQ_HANDLED;
		}
	}

	if (unlikely((men->d3->design_base == NULL) || men->releasing)) {
		if (men->d3->design_base == NULL)
			dev_err(&men->dev, "unexpected irq (status 0x%x):"
					" no design loaded\n", sr);
		men3_stopirq(men);
		spin_unlock(&men->boardlock);
		return IRQ_HANDLED;
	}

	ir = ioread32(men->d3->design_base + men->d3->design.irq_read);

	/* DMA interrupts */
	for (i = 0; i < ME3_DMACHANS; i++) {
		if (!(ir & (1 << i)))
			continue;

		if (!have_ts) {
			have_ts = true;
			ts = current_kernel_time();
		}

		/* delete local interrupt */
		me3_dma_irq(men, i, &ts);
		ir &= ~(1 << i);
	}

	/* User interrupts */
	if (ir & 0xff0) {
		uint32_t bit;
		uint32_t st = ir & 0xff0;

		spin_unlock(&men->boardlock);
		for (bit = 4; st; bit++) {
			if (st & (1 << bit)) {
				unsigned int uiqchan = bit - 4;
				if (unlikely((uiqchan >= men->uiqcnt[0]) ||
						(men->uiqs[uiqchan] == NULL))) {
					dev_warn(&men->dev, "IRQ on unused UIQ %i\n", uiqchan);
				} else {
					uiq_irq(men->uiqs[uiqchan], &ts, &have_ts);
				}
				st ^= (1 << bit);
			}
		}
		return IRQ_HANDLED;
	}
	spin_unlock(&men->boardlock);
	if (unlikely(ir))
		dev_err(&men->dev, "unexpected local irq 0x%x\n", ir);
	return IRQ_HANDLED;
}

static void
me3_free_plx(struct siso_menable *men, struct men_dma_chain *res,
		dma_addr_t dma)
{
	while (res) {
		struct men_dma_chain *n = res->next;
		dma_addr_t ndma = le32_to_cpu(res->plx->next);

		dma = dma & 0xfffffff0;
		dma_pool_free(men->pool, res->plx, dma);
		kfree(res);
		res = n;
		dma = ndma;
	}
}

void
me3_free_sb(struct siso_menable *men, struct menable_dmabuf *sb)
{
	me3_free_plx(men, sb->dmat, sb->dma);
}

static void
sg_to_plx(struct scatterlist *sg, struct plx_chain *plx)
{
	dma_addr_t da = sg_dma_address(sg);

	plx->local_address = 0;
	plx->length = cpu_to_le32(sg_dma_len(sg));
	plx->pci_address = cpu_to_le32(da);
	/* end of list. If not it will be overwritten anway */
	plx->next = cpu_to_le32(0xb);
}

static int
men3_create_userbuf(struct siso_menable *men, struct menable_dmabuf *db)
{
	struct men_dma_chain *cur, *prev = NULL;
	int i;

	cur = db->dmat;
	cur->plx = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
	if (!cur->plx)
		goto fail_plx;

	sg_to_plx(db->sg, cur->plx);
	prev = cur;

	for (i = 1; i < db->nents; i++) {
		dma_addr_t da;
		u32 prevlen = le32_to_cpu(prev->plx->length);

		if (le32_to_cpu(prev->plx->pci_address) + prevlen ==
					sg_dma_address(db->sg + i)) {
			prevlen += sg_dma_len(db->sg + i);
			prev->plx->length = cpu_to_le32(prevlen);
			continue;
		}
		cur = kzalloc(sizeof(*cur), GFP_USER);
		if (!cur)
			goto fail_next;
		cur->plx = dma_pool_alloc(men->pool,
					GFP_USER, &da);
		if (!cur->plx) {
			kfree(cur);
			goto fail_next;
		}

		/* control bits for PLX DMA chain */
		prev->plx->next = cpu_to_le32(da + 0x9);
		prev->next = cur;

		sg_to_plx(db->sg + i, cur->plx);

		prev = cur;
	}
	return 0;
fail_next:
	me3_free_sb(men, db);
	return -ENOMEM;
fail_plx:
	kfree(db->dmat);
	return -ENOMEM;
}

static void
men3_abort_dma(struct siso_menable *men, struct menable_dmachan *dc)
{
	plx_abort_dma(men, dc);
}

static void
men3_startirq(struct siso_menable *men)
{
	if (men->d3->design_name[0] == '\0')
		return;

	iowrite32(ME3_IRQMASK, men->runtime_base + PLX_INT_CSR);
}

static void
me3_clear_des_settings(struct siso_menable *men, const bool force)
{
	int i;
	unsigned long flags;
	void __iomem *oldmap;
	bool oldchanging;

	spin_lock_irqsave(&men->designlock, flags);
	if (men->design_changing && !force) {
		spin_unlock_irqrestore(&men->designlock, flags);
		return;
	}

	men3_stopirq(men);
	oldmap = men->d3->design_base;
	men->d3->design_base = NULL;

	oldchanging = men->design_changing;
	men->design_changing = true;
	spin_unlock_irqrestore(&men->designlock, flags);

	men->d3->expansion_active = 0;
	men->d3->design.irq_read = 0;
	men->d3->design.irq_ack = 0;
	for (i = 0; i <= 1; i++) {
		struct menable_dmachan *dc = men_dma_channel(men, i);
		dc->irqack = NULL;
		dc->irqenable = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(men->d3->design.dma_len); i++)
		men->d3->design.dma_len[i] = 0;

	men->d3->design.localirq = 0;

	for (i = 0; i < ARRAY_SIZE(men->d3->design.dmatag); i++)
		men->d3->design.dmatag[i] = 0;

	men_del_uiqs(men, 0);
	kfree(men->uiqs);
	men->uiqs = NULL;

	if (!oldchanging) {
		spin_lock_irqsave(&men->designlock, flags);
		men->design_changing = oldchanging;
		spin_unlock_irqrestore(&men->designlock, flags);
	}

	if (oldmap != NULL)
		pcim_iounmap(men->pdev, oldmap);
}

static int
me3_set_des_settings(struct siso_menable *men, struct l_design_settings *ds)
{
	unsigned long flags;
	uint32_t maplen;
	int i;
	void __iomem *newmap;
	unsigned int newuiqcnt = 0;
	struct menable_uiq **nuiqs = NULL;

	/* find the last register in the area so we know what
	 * memory range to map */
	maplen = max(ds->irq_read, ds->irq_ack);

	maplen = max(maplen, ds->irq_start);
	for (i = 0; i < ARRAY_SIZE(ds->dma_len); i++)
		if (ds->dma_len[i] != 0)
			maplen = max(maplen, ds->dma_len[i]);
	for (i = 0; i < ARRAY_SIZE(ds->dmatag); i++)
		if (ds->dmatag[i] != 0)
			maplen = max(maplen, ds->dmatag[i]);
	for (i = 0; i < ARRAY_SIZE(ds->userintqueue); i++) {
		if (ds->userintqueue[i].address == 0)
			continue;

		newuiqcnt = i + 1;
		maplen = max(maplen, ds->userintqueue[i].address);

		if ((ds->userintqueue[i].tag == 0) &&
				(ds->userintqueue[i].shift != 0)) {
			dev_warn(&men->dev, "shifted UIQ registers are not "
					"supported.\n");
			return -EINVAL;
		}
	}
	maplen = max(maplen, ds->localirq);
	maplen = PFN_ALIGN(maplen);

	spin_lock_irqsave(&men->designlock, flags);
	if (men->design_changing) {
		spin_unlock_irqrestore(&men->designlock, flags);
		return -EBUSY;
	}

	if ((men->d3->design.localirq != 0) && (men->d3->design_base))
		iowrite32(0, men->d3->design_base + men->d3->design.localirq);

	men->design_changing = true;
	spin_unlock_irqrestore(&men->designlock, flags);

	me3_clear_des_settings(men, true);

	if (newuiqcnt > 0) {
		nuiqs = kcalloc(newuiqcnt, sizeof(*men->uiqs), GFP_KERNEL);

		if (nuiqs == NULL) {
			spin_lock_irqsave(&men->designlock, flags);
			men->design_changing = false;
			spin_unlock_irqrestore(&men->designlock, flags);
			return -ENOMEM;
		}
	}

	newmap = pcim_iomap(men->pdev, 2, maplen);
	if (newmap == NULL) {
		kfree(nuiqs);
		spin_lock_irqsave(&men->designlock, flags);
		men->design_changing = false;
		spin_unlock_irqrestore(&men->designlock, flags);
		return -ENOMEM;
	}

	men->d3->design.irq_read = ds->irq_read;
	men->d3->design.irq_ack = ds->irq_ack;
	for (i = 0; i <= 1; i++) {
		struct menable_dmachan *dc = men_dma_channel(men, i);
		dc->irqack = newmap + ds->irq_ack;
		dc->irqenable = newmap + ds->irq_start;
	}
	men->d3->design.dma_len[0] = ds->dma_len[0];
	men->d3->design.dma_len[1] = ds->dma_len[1];
	men->d3->design.dmatag[0] = ds->dmatag[0];
	men->d3->design.dmatag[1] = ds->dmatag[1];
	men->d3->design.localirq = ds->localirq;

	men->d3->design_base = newmap;

	for (i = 0; i < newuiqcnt; i++) {
		struct menable_uiq *uiq;

		if (ds->userintqueue[i].address == 0)
			continue;

		uiq = men_uiq_init(i,
				men->d3->design_base + ds->userintqueue[i].address,
				men, ds->userintqueue[i].tag != 0,
				ds->userintqueue[i].burstlen);

		if (IS_ERR(uiq)) {
			int ret = PTR_ERR(uiq);
			int j;

			spin_lock_irqsave(&men->designlock, flags);
			men->design_changing = false;
			spin_unlock_irqrestore(&men->designlock, flags);

			for (j = 0; j < i; j++)
				men_uiq_remove(nuiqs[j]);

			kfree(nuiqs);
			return ret;
		}

		uiq->irqack = men->d3->design_base + men->d3->design.irq_ack;
		uiq->ackbit = 4 + i;
		nuiqs[i] = uiq;

		/* the lib will scale the uiqs to the right length */
	}

	men->uiqs = nuiqs;
	men->uiqcnt[0] = newuiqcnt;

	spin_lock_irqsave(&men->designlock, flags);
	men->design_changing = false;

	men->d3->expansion_active = 1;
	if (ds->localirq != 0) {
		men3_startirq(men);

		if (men->d3->design_base)
			iowrite32(1, men->d3->design_base + men->d3->design.localirq);
	}

	spin_unlock_irqrestore(&men->designlock, flags);

	return 0;
}

static int
men3_ioctl(struct siso_menable *men, const unsigned int cmd,
		const unsigned int size, unsigned long arg)
{
	switch (cmd) {
	case IOCTL_GET_EEPROM_DATA: {
		struct men_eeprom ee;

		if (size != sizeof(ee)) {
			warn_wrong_iosize(men, cmd, sizeof(ee));
			return -EINVAL;
		}

		me3_ee_read(men, &ee);
		return copy_to_user((uint32_t __user *) arg, &ee, sizeof(ee));
	}
	case IOCTL_DESIGN_SETTINGS: {
		if ((arg == 0) && (size == 0)) {
			me3_clear_des_settings(men, false);
			return 0;
		} else {
			struct l_design_settings ds;

			if (size != sizeof(ds)) {
				warn_wrong_iosize(men, cmd, sizeof(ds));
				return -EINVAL;
			}

			if (copy_from_user(&ds, (void __user *) arg,
						sizeof(ds)))
				return -EFAULT;
			return me3_set_des_settings(men, &ds);
		}
	}
	case IOCTL_RESSOURCE_CONTROL: {
		unsigned long flags;
		int res;

		if (size != 0) {
			warn_wrong_iosize(men, cmd, 0);
			return -EINVAL;
		}

		spin_lock_irqsave(&men->designlock, flags);
		if (!men->d3->expansion_active) {
			spin_unlock_irqrestore(&men->designlock, flags);
			return 0;
		}
		switch (arg) {
		case 0:
		case 1:
			if (men->d3->design_base && men->d3->design.localirq) {
				iowrite32((uint32_t)(arg & 0xff),
						men->d3->design_base +
						men->d3->design.localirq);
				res = 0;
			} else {
				res = -EINVAL;
			}
			break;
		default:
			res = -EINVAL;
		}
		spin_unlock_irqrestore(&men->designlock, flags);
		return res;
	}
	case IOCTL_PP_CONTROL:
	case IOCTL_BOARD_INFO:
		return -EINVAL;
	default:
		return -ENOIOCTLCMD;
	}
}

static unsigned int
men3_query_dma(struct siso_menable *men, const unsigned int fpga)
{
	BUG_ON(fpga > 0);
	return ME3_DMACHANS;
}

static void
me3_fix_plxsgl(struct menable_dmabuf *sb, const int dir, const __le32 locaddr)
{
	struct men_dma_chain *sgl = sb->dmat;

	/* this entry is already correct */
	if (sgl->plx->local_address == locaddr) {
		if (dir == PCI_DMA_FROMDEVICE) {
			if (le32_to_cpu(sgl->plx->next) & 0x8)
				return;
		} else {
			if (!(le32_to_cpu(sgl->plx->next) & 0x8))
				return;
		}
	}

	while (sgl) {
		uint32_t plxnext;
		sgl->plx->local_address = locaddr;
		if (dir == PCI_DMA_FROMDEVICE)
			plxnext = (le32_to_cpu(sgl->plx->next) | 0x8);
		else
			plxnext = (le32_to_cpu(sgl->plx->next) & ~0x8);

		sgl->plx->next = cpu_to_le32(plxnext);
		sgl = sgl->next;
	}
}

static int
men3_startdma(struct menable_dmachan *dmac)
{
	uint32_t ir;
	struct siso_menable *men = dmac->parent;
	void __iomem *runtime_base = dmac->parent->runtime_base;
	unsigned int offs;
	const __le32 locaddr = dmac->number ?
			cpu_to_le32(0x218000) :
			cpu_to_le32(0x210000);
	long i;

	if (!men->d3->expansion_active || (men->d3->design_base == NULL))
		return -EINVAL;

	BUG_ON(dmac->number >= 2);
	ir = ioread32(men->runtime_base + PLX_INT_CSR);
	ir |= 0x900;	/* enable local IRQ and Pci Interrupts */
	iowrite32(ir, men->runtime_base + PLX_INT_CSR);

// FIXME: dma_mode missing
	ir = 0x21fc3;
/*	if((dma_mode[chan] & 0x1) == 0)
		ir = 0x00021fc3; // demand + no increment
	else
		ir = 0x00020fc3; // no demand + no increment*/
	offs = (dmac->number == 0) ? 0x80 : 0x94;
	iowrite32(ir, runtime_base + offs);

	/* initialize local address of all SGL entries */
	for (i = 0; i < dmac->active->num_sb; i++)
		me3_fix_plxsgl(dmac->active->bufs[i], dmac->direction, locaddr);

	if (dmac->mode == DMA_HANDSHAKEMODE)
		me3_fix_plxsgl(men->dummybuf(dmac), dmac->direction, locaddr);

	me_queue_dma(dmac, 1);

	iowrite32(1 << dmac->enablebit, dmac->irqenable);

	iowrite8(1, dmac->iobase);	/* enable */
	wmb();
	iowrite8(3, dmac->iobase);	/* start xfer */
	return 0;
}

static void
men3_free_dummybuf(struct siso_menable *men, const unsigned int port)
{
	me3_free_sb(men, &(men->d3->dummybuf[port]));
}

static struct attribute *me3_attributes[6] = {
	&dev_attr_design_clk.attr,
	&dev_attr_eeprom.attr,
	NULL
};

static struct attribute_group me3_attribute_group = {
        .attrs = me3_attributes
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
const
#endif /* LINUX >= 2.6.31 */
struct attribute_group *me3_attribute_groups[2] = {
	&me3_attribute_group,
	NULL
};

static void
men3_exit(struct siso_menable *men)
{
	int i;

	for (i = 0; i < ME3_DMACHANS; i++)
		men3_free_dummybuf(men, i);
	pci_free_consistent(men->pdev, men->d3->dummy_len, men->d3->dummy_va,
			men->d3->dummy_dma);
	kfree(men->d3);
}

static void
men3_dmabase(struct siso_menable *men, struct menable_dmachan *dc)
{
	BUG_ON(dc->fpga != 0);
	dc->iobase = men->runtime_base + (dc->number ? 0xa9 : 0xa8);
	dc->ackbit = dc->number;
	dc->enablebit = dc->number;
	/* dc->irqack and dc->irqenable will be set on FPGA configure */
}

static struct menable_dmabuf *
me3_dummybuf(struct menable_dmachan *dc)
{
	return &(dc->parent->d3->dummybuf[dc->number]);
}

static int
me3_dummyscale(struct menable_dmachan *dc, const size_t len)
{
	struct siso_menable *men = dc->parent;
	struct menable_dmabuf *dummy = men->dummybuf(dc);
	unsigned int newpg = DIV_ROUND_UP(len, men->d3->dummy_len);
	unsigned int oldpg;
	struct men_dma_chain *ch;

	oldpg = DIV_ROUND_UP((uint32_t)dummy->buf_length, men->d3->dummy_len);

	if (newpg == oldpg)
		return 0;

	if (oldpg > newpg) {
		ch = dummy->dmat;
		newpg = oldpg - newpg;

		while (newpg-- > 0) {
			ch = ch->next;
			BUG_ON(ch == NULL);
		}

		me3_free_plx(men, ch->next, le32_to_cpu(ch->plx->next));
		ch->next = NULL;
		ch->plx->next = cpu_to_le32(0xb);
		return 0;
	}

	newpg -= oldpg;
	ch = dummy->dmat;
	while (ch->next)
		ch = ch->next;
	while (newpg--) {
		struct scatterlist sg;

		ch->next = kzalloc(sizeof(*ch->next), GFP_USER);

		if (!ch->next)
			return -ENOMEM;

		ch->next->plx = dma_pool_alloc(men->pool, GFP_USER,
				&sg_dma_address(&sg));
		if (!ch->next->plx) {
			kfree(ch->next);
			ch->next = NULL;
			return -ENOMEM;
		}
		sg.offset = 0;
		sg.length = men->d3->dummy_len;

		sg_to_plx(&sg, ch->next->plx);
		ch->plx->next = cpu_to_le32(sg_dma_address(&sg) | 0x9);
		ch = ch->next;
	}

	return 0;
}

static int
men3_create_dummybuf(struct siso_menable *men, const unsigned int port)
{
	struct scatterlist sg;
	struct menable_dmabuf *db = &men->d3->dummybuf[port];

	db->index = -1;

	db->dmat = kzalloc(sizeof(db->dmat), GFP_KERNEL);
	if (db->dmat == NULL)
		return -ENOMEM;

	db->dmat->plx = dma_pool_alloc(men->pool, GFP_USER, &db->dma);
	if (db->dmat->plx == NULL) {
		kfree(db->dmat);
		return -ENOMEM;
	}

	sg_dma_address(&sg) = men->d3->dummy_dma;
	sg_dma_len(&sg) = men->d3->dummy_len;
	sg.offset = 0;
	sg_to_plx(&sg, db->dmat->plx);
	db->buf_length = men->d3->dummy_len;

	return 0;
}

int
me3_probe(struct siso_menable *men)
{
	uint32_t u;
	int ret = -ENOMEM;

	men->d3 = kzalloc(sizeof(*men->d3), GFP_KERNEL);
	if (men->d3 == NULL)
		goto err;

	men->design_changing = false;
	men->d3->rom_base = pcim_iomap(men->pdev, 3, 0);
	if (!men->d3->rom_base) {
		dev_err(&men->dev, "can't map BAR 3\n");
		goto err_d3;
	}

	if (pci_set_dma_mask(men->pdev, DMA_BIT_MASK(32))) {
		dev_err(&men->dev, "No suitable DMA available\n");
		goto err_mask;
	}
	pci_set_consistent_dma_mask(men->pdev, DMA_BIT_MASK(32));

	men->pool = dmam_pool_create("men_3", &men->pdev->dev,
			sizeof(struct plx_chain), 16, 0);
	if (!men->pool) {
		dev_err(&men->dev, "can not allocate DMA pool\n");
		goto err_pool;
	}

	men->d3->dummy_len = 128 * 1024;
	do {
		men->d3->dummy_va = pci_alloc_consistent(men->pdev,
				men->d3->dummy_len, &men->d3->dummy_dma);
		if (men->d3->dummy_va == NULL)
			men->d3->dummy_len /= 2;
	} while ((men->d3->dummy_va == NULL) &&
			(men->d3->dummy_len >= PAGE_SIZE));

	if (men->d3->dummy_va == NULL)
		goto err_dummy;
	memset(men->d3->dummy_va, 0, men->d3->dummy_len);
	dev_info(&men->dev, "allocated dummy DMA area of %zi kiB\n",
			men->d3->dummy_len / 1024);

	ret = men3_create_dummybuf(men, 0);
	if (ret != 0)
		goto err_dummy_sb;
	ret = men3_create_dummybuf(men, 1);
	if (ret != 0)
		goto err_dummy_plx;

	men->create_buf = men3_create_userbuf;
	men->free_buf = me3_free_sb;
	men->startdma = men3_startdma;
	men->abortdma = men3_abort_dma;
	men->stopdma = men3_abort_dma;
	men->stopirq = men3_stopirq;
	men->startirq = men3_startirq;
	men->ioctl = men3_ioctl;
	men->exit = men3_exit;
	men->query_dma = men3_query_dma;
	men->dmabase = men3_dmabase;
	men->queue_sb = me3_queue_sb;
	men->dummybuf = me3_dummybuf;
	men->dummyscale = me3_dummyscale;

	men->desname = men->d3->design_name;
	men->deslen = sizeof(men->d3->design_name);

	iowrite32(0x100800, men->runtime_base + PLX_MISC);

	u = PLX_DMA_MODE_LBUS32 | PLX_DMA_MODE_TARDY | PLX_DMA_MODE_CONTBUR |
			PLX_DMA_MODE_LBUR;

	/* init of DMA controllers */
	iowrite32(u, men->runtime_base + PLX_DMA0_MODE);
	iowrite32(0, men->runtime_base + PLX_DMA0_LADDR);
	iowrite32(2048, men->runtime_base + PLX_DMA0_LEN);
	iowrite32(0, men->runtime_base + PLX_DMA0_DESCR);
	iowrite32(u, men->runtime_base + PLX_DMA1_MODE);
	iowrite32(0, men->runtime_base + PLX_DMA1_LADDR);
	iowrite32(2048, men->runtime_base + PLX_DMA1_LEN);
	iowrite32(0, men->runtime_base + PLX_DMA1_DESCR);
	iowrite32(0x101, men->runtime_base + PLX_DMA_CS);

	ret = devm_request_irq(&men->pdev->dev, men->pdev->irq, me3_irq,
				IRQF_SHARED, DRIVER_NAME, men);
	if (ret) {
		dev_err(&men->dev, "can't request interrupt\n");
		goto fail_uiq;
	}

	men->active_fpgas = 1;

	return 0;
fail_uiq:
	men_del_uiqs(men, 0);
	men3_free_dummybuf(men, 1);
err_dummy_plx:
	men3_free_dummybuf(men, 0);
err_dummy_sb:
	pci_free_consistent(men->pdev, men->d3->dummy_len, men->d3->dummy_va,
			men->d3->dummy_dma);
err_dummy:
err_pool:
err_mask:
err_d3:
	kfree(men->d3);
err:
	return ret;
}
