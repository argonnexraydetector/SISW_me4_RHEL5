/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#ifndef MENABLE3_H
#define MENABLE3_H

struct plx_chain {
	__le32 pci_address;
	__le32 local_address;
	__le32 length;
	__le32 next;
#ifdef USE64
	__le32 pci_high;
#endif
} __attribute__ ((packed));

#define ME3_DMACHANS 2

struct me3_lock {
	int dres;		/* design resource */
	int dresid;
	int dlock;		/* config lock */
	int dlockid;		/* owner of dlock */
	int ulock;
	int ulockid;		/* owner of ulock */
};

struct k_design_settings {
	unsigned int irq_read;
	unsigned int irq_ack;
	unsigned int dma_len[ME3_DMACHANS];

	unsigned int dmatag[ME3_DMACHANS];

	unsigned int localirq;
};

struct me3_data {
	char design_name[1024];	/* user supplied name of design (for IPC) */
	struct k_design_settings design;
	int expansion_active;
	struct me3_lock lock;
	void __iomem *rom_base;
	void __iomem *design_base;

	void *dummy_va;
	dma_addr_t dummy_dma;
	size_t dummy_len;
	struct menable_dmabuf dummybuf[ME3_DMACHANS];
};

/* only PLX local interrupt allowed */
#define ME3_IRQMASK (PLX_IRQ_PCI_EN | PLX_IRQ_PCI_LIRQ_EN)
#define ME3_IRQ_INP (PLX_IRQ_PCI_LIRQ | PLX_IRQ_PCI_ABORT)

extern void me3_free_sb(struct siso_menable *, struct menable_dmabuf *);

/* USER Interrupt Defines */
#define HAP_USERINTQUEUEREAD	0x08151060
#define HAP_USERINTQUEUEWRITE	0x08151070

#endif
