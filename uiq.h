/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#ifndef UIQ_H
#define UIQ_H

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct siso_menable;

struct menable_uiq {
	struct device dev;
	struct siso_menable *parent;

	/* simple ringbuffer. Memory is allocated on demand */
	void __iomem *reg;	/* address of hardware register */
	void __iomem *irqack;	/* address of IRQ ACK register */
	uint32_t *data;		/* data buffer */
	unsigned int rindex;	/* read index */
	unsigned int fill;	/* number of entries occupied starting at rindex */
	unsigned int length;	/* index wraparound (sort of ARRAY_SIZE(data)) */
	unsigned int lost;	/* number of entries lost by overflow */
	unsigned int irqcnt;	/* number of interrupts */
	unsigned int cpltodo;	/* number of entries cpl still waits for */
	unsigned char chan;	/* own channel number */
	unsigned char burst;	/* max number of entries to write on burst */
	unsigned char ackbit;	/* bit in irqack */
	bool running;		/* still waiting for ACK */
	bool write_queue;	/* queue direction */
	bool loss_error;	/* the last value(s) were lost */
	unsigned long cpltimeout;	/* timeout for read */
	struct completion cpl;		/* wait for timeout */
	spinlock_t lock;
};

extern struct menable_uiq *men_uiq_init(int chan, void __iomem *addr,
		struct siso_menable *parent, bool write, unsigned char burst);
extern int men_scale_uiq(struct menable_uiq *uiq, const unsigned int len);
extern void men_uiq_remove(struct menable_uiq *);
extern void uiq_irq(struct menable_uiq *uiq, struct timespec *ts, bool *have_ts);
extern void men_del_uiqs(struct siso_menable *men, const unsigned int fpga);

extern struct class *menable_uiq_class;
extern struct device_attribute men_uiq_attributes[6];

#endif /* UIQ_H */
