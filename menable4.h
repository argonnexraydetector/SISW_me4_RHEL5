/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#ifndef MENABLE4_H
#define MENABLE4_H

#include <linux/spinlock.h>

#include "menable.h"

#define ME4_NUMDMA	(0x002 * 4)
#define ME4_IRQSTATUS	(0x100 * 4)
#define ME4_IRQACK	(0x102 * 4)
#define ME4_IRQTYPE	(0x104 * 4)
#define ME4_IRQENABLE	(0x106 * 4)

#define ME4_DMAOFFS	(0x110 * 4)
#define ME4_DMASZ	(0x010 * 4)

#define ME4_DMACTRL	(0x000 * 4)
#define ME4_DMAADDR	(0x002 * 4)
#define ME4_DMAACTIVE	(0x004 * 4)
#define ME4_DMALENGTH	(0x006 * 4)
#define ME4_DMACOUNT	(0x008 * 4)
#define ME4_DMAMAXLEN	(0x00a * 4)
#define ME4_DMATYPE	(0x00c * 4)
#define ME4_DMATAG	(0x00e * 4)

#define ME4_IRQQUEUE	(0x080 * 4)
#define ME4_IRQQ_LOW	16
#define ME4_IRQQ_HIGH	29

#define ME4_FULLOFFSET     (0x2000 * 4)
#define ME4_IFCONTROL      (0x0004 * 4)
#define ME4_UIQCNT         (0x0006 * 4)
#define ME4_FIRSTUIQ       (0x0007 * 4)
#define ME4_FPGACONTROL    (0x1010 * 4)
#define ME4_PCIECONFIG0    (0x0010 * 4)
#define ME4_PCIECONFIGMAX  (0x001f * 4)

/* set when IRQs from next higher FPGA are set */
#define ME4_CHAIN_MASK 0x40000000

#define ME4_DMA_FIFO_DEPTH	16LL

struct menable_uiq;

struct me4_sgl {
	__le64 addr[7];
	__le64 next;
} __attribute__ ((packed));

struct me4_data {
	struct menable_dmabuf dummybuf;
	void *dummypage;
	uint32_t irq_wanted[MAX_FPGAS];	/* protected by irqmask_lock */
	uint32_t design_crc;
	char design_name[65];	/* user supplied name of design (for IPC) */
	spinlock_t irqmask_lock;	/* to protect IRQ enable register */
};

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
