/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 *
 * define constants for PLX 9056 register offsets and bit masks
 *
 * This is incomplete, I only defined those I needed...
 */
#ifndef PLX_H
#define PLX_H

#define PLX_MISC	0x0C
#define PLX_INT_CSR	0x68

#define PLX_DMA0_MODE	0x80
/* warning: the next 3 registers cycle around depending on operation mode */
#define PLX_DMA0_PCIL	0x84
#define PLX_DMA0_LADDR	0x88
#define PLX_DMA0_LEN	0x8C
#define PLX_DMA0_DESCR	0x90

#define PLX_DMA1_MODE	0x9a
/* warning: the next 3 registers cycle around depending on operation mode */
#define PLX_DMA1_PCIL	0x98
#define PLX_DMA1_LADDR	0x9C
#define PLX_DMA1_LEN	0xA0
#define PLX_DMA1_DESCR	0xA4

#define PLX_DMA_CS	0xA8
#define PLX_DMA0_CS	0xAB
#define PLX_DMA1_CS	0xAA

#define PLX_DMA_ARBIT	0xAC
#define PLX_DMA_THRES	0xB0

#define PLX_DMA0_PCIH	0xB4
#define PLX_DMA1_PCIH	0xB8

#define PLX_IRQ_PARITY_ERR	0x00000080
#define PLX_IRQ_PCI_EN		0x00000100
#define PLX_IRQ_PCI_LIRQ_EN	0x00000800
#define PLX_IRQ_PCI_LIRQ	0x00008000
#define PLX_IRQ_PCI_ABORT	0x00020000

#define PLX_DMA_MODE_LBUS8	0x00000000
#define PLX_DMA_MODE_LBUS16	0x00000001
#define PLX_DMA_MODE_LBUS32	0x00000002

#define PLX_DMA_MODE_TARDY	0x00000040
#define PLX_DMA_MODE_CONTBUR	0x00000080
#define PLX_DMA_MODE_LBUR	0x00000100

#define PLX_DMA_MODE_DAC	0x00040000

#endif
