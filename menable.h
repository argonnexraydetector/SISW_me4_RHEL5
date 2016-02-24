/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#ifndef MENABLE_H
#define MENABLE_H

#define DRIVER_NAME	"menable"
#define DRIVER_DESCRIPTION "microEnable III/IV/5 driver"
#define DRIVER_VENDOR "Silicon Software GmbH"
#define DRIVER_VERSION "4.1.5"

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
#error This driver requires at least kernel 2.6.23
#endif

/* Maximum number of devices the driver can handle. Only
 * effect is the size of the device number region. Increase
 * it to any size you need. */
#define MEN_MAX_NUM	8

#define PCI_VENDOR_PLX 0x10b5
#define PCI_VENDOR_SISO 0x1ae8

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define FREE_LIST 0
#define GRABBED_LIST 1
#define HOT_LIST 2
#define NO_LIST 3

/* no device currently supports more than 8 DMA channels */
#define MEN_MAX_DMA 8
#define MEN_MAX_UIQ_PER_FPGA 16
/* currently at most 2 FPGAs implement IRQs */
#define MAX_FPGAS 2

struct men_dma_chain {
	union {
		struct me4_sgl *pcie4;
		struct me5_sgl *pcie5;
		struct plx_chain *plx;
	};
	struct men_dma_chain *next;
};

struct menable_dmabuf {
	struct men_dma_chain *dmat;	/* dma descriptor list of buffer */
	dma_addr_t dma;			/* dma address of dmat */
	struct scatterlist *sg;		/* sg-list of DMA buffer */
	int nents;			/* number of used entries in dmasg */
	int rents;			/* number of real entries in dmasg */
	struct list_head node;		/* entry in dmaheads *_list */
	unsigned int listname;		/* which list are we currently in? */
	uint64_t dma_length;		/* length of valid data */
	uint64_t buf_length;		/* length of buffer */
	uint32_t dma_tag;		/* last tag sent by DMA channel */
	long index;			/* index in DMA channels bufs[] */
	struct timespec timestamp;	/* time when "grabbed" irq was handled */
};

struct menable_dmachan;

struct menable_dmahead {
	struct list_head node;		/* entry in parents heads list*/
	unsigned int id;		/* id number to be used by userspace */
	struct menable_dmabuf **bufs;	/* array of buffers */
	long num_sb;			/* length of bufs */
	struct menable_dmachan *chan;	/* channel this head is currently linked to */
};

struct completion;
struct siso_menable;

struct menable_dma_wait {
	struct list_head node;		/* entry in the parents wait_list */
	struct completion cpl;		/* used to wait for specific imgcnt */
	uint64_t frame;		/* image number to wait for, 0 if none */
};

struct menable_dmachan {
	struct device dev;
	struct siso_menable *parent;
	spinlock_t chanlock;		/* lock to protect administrative changes */
	struct menable_dmahead *active;	/* active dma_head */
	unsigned char number;		/* number of DMA channel on device */
	unsigned char fpga;		/* FPGA index this channel belongs to */
	unsigned int mode:6;		/* streaming or controlled */
	unsigned int direction:2;	/* PCI_DMA_{TO,FROM]DEVICE */
	unsigned int running:2;		/* 0: stopped, 1: active, 2: finished, 3: in shutdown */
	unsigned int ackbit:5;		/* bit in irqack */
	unsigned int enablebit:5;	/* bit in irqenable */
	void __iomem *iobase;		/* base address for register space */
	void __iomem *irqack;		/* IRQ ACK register */
	void __iomem *irqenable;	/* IRQ enable register */

	spinlock_t listlock;		/* lock to protect list changes */
	uint64_t imgcnt;		/* absolute image number of this DMA */
	uint64_t goodcnt;		/* number of transfers to real buffers */
	unsigned int free;		/* entries in free_list */
	unsigned int grabbed;		/* entries in grabbed_list */
	unsigned int hot;		/* count of hot_list */
	unsigned int lost;		/* lost pictures */
	unsigned int locked;		/* locked pictures */
	struct list_head free_list;	/* list of free buffers */
	struct list_head grabbed_list;	/* list of filled buffers */
	struct list_head hot_list;	/* entries currently "in hardware" */
	struct list_head wait_list;	/* completions waiting for a frame */
	long long transfer_todo;	/* number of image still to transfer */

	spinlock_t timerlock;		/* lock to protect timer */
	struct timer_list timer;	/* DMA timeouts */
	unsigned long timeout;		/* delay for timer restarts (in jiffies) */

	struct work_struct dwork;	/* called when all pictures grabbed */
};

struct menable_uiq;

struct siso_menable {
	/* kernel stuff */
	struct device dev;
	struct module *owner;
	struct pci_dev *pdev;
	spinlock_t boardlock;
	void __iomem *runtime_base;
	struct dma_pool *pool;
	struct cdev cdev;

	int board;				/* type of board: 0: ? 1: meIII 2: me4 */
	int idx;				/* driver-internal number of board */
	unsigned char active_fpgas;		/* how many FPGAs are active */
	unsigned char dmacnt[MAX_FPGAS];	/* number of active DMA channels (per FPGA) */
	unsigned char uiqcnt[MAX_FPGAS];	/* number of UIQs (per FPGA) */
	struct menable_uiq **uiqs;		/* UIQ control structs */
	struct menable_dmachan **dmachannels;	/* array of DMA channels */
	int use;			/* number of open fds on this board */
	spinlock_t designlock;
	char *desname;			/* design name */
	size_t deslen;			/* length of desname buffer */
	uint32_t desval;		/* design clock on meIII, design CRC on meIV */
	bool releasing;			/* board is about to be destroyed */
	bool design_changing;		/* the device is reconfigured and changes it's properties */
	union {
		struct me3_data *d3;
		struct me4_data *d4;
		struct me5_data *d5;
	};

	spinlock_t headlock;
	unsigned int headcnt;
	struct list_head heads;		/* all buffer heads */

	int (*open)(struct siso_menable *, struct file *);
	int (*release)(struct siso_menable *, struct file *);
	int (*create_buf)(struct siso_menable *, struct menable_dmabuf *);
	void (*free_buf)(struct siso_menable *, struct menable_dmabuf *);
	int (*startdma)(struct menable_dmachan *);
	void (*abortdma)(struct siso_menable *, struct menable_dmachan *);
	void (*stopdma)(struct siso_menable *, struct menable_dmachan *);
	int (*ioctl)(struct siso_menable *, const unsigned int, const unsigned int, unsigned long);
	void (*exit)(struct siso_menable *);
	void (*cleanup)(struct siso_menable *);	/* garbage collection if last handle is closed */
	unsigned int (*query_dma)(struct siso_menable *, const unsigned int);
	void (*dmabase)(struct siso_menable *, struct menable_dmachan *);
	void (*stopirq)(struct siso_menable *);
	void (*startirq)(struct siso_menable *);
	void (*queue_sb)(struct menable_dmachan *, struct menable_dmabuf *);
	struct menable_dmabuf *(*dummybuf)(struct menable_dmachan *);
	int (*dummyscale)(struct menable_dmachan *, const size_t);
};

struct men_io_range;
struct fg_ctrl;

extern int men_create_userbuf(struct siso_menable *, struct men_io_range *);
extern int men_free_userbuf(struct siso_menable *, struct menable_dmahead *, long index);
extern int men_alloc_dma(struct siso_menable *men, unsigned int count) __releases(&men->headlock);
extern int men_add_dmas(struct siso_menable *men);
extern int buf_get_uint(const char *, size_t, unsigned int *);
extern int men_start_dma(struct menable_dmachan *dc, struct menable_dmahead *, const unsigned int startbuf);
extern int men_wait_dmaimg(struct menable_dmachan *d, const uint64_t imt, const struct timespec *timeout, uint64_t *foundframe);
extern int men_create_buf_head(struct siso_menable *, const size_t maxsize, const long subbufs);
extern int men_release_buf_head(struct siso_menable *, struct menable_dmahead *);
extern void men_free_buf_head(struct siso_menable *, struct menable_dmahead *);
extern struct menable_dmabuf *men_move_hot(struct menable_dmachan *db, const struct timespec *ts);
extern void men_destroy_sb(struct siso_menable *, struct menable_dmabuf *);
extern void men_stop_dma(struct menable_dmachan *);
extern void men_stop_dma_locked(struct menable_dmachan *);
extern struct menable_dmahead *me_get_bh(struct siso_menable *men, const unsigned int);
extern struct menable_dmabuf *me_get_sb(struct siso_menable *men, const unsigned int headnum, const long bufidx);
extern struct menable_dmabuf *me_get_sb_by_head(struct menable_dmahead *head, const long bufidx);
extern int fg_start_transfer(struct siso_menable *, struct fg_ctrl *, const size_t tsize);
extern void me_queue_dma(struct menable_dmachan *, const unsigned int);
extern long menable_ioctl(struct file *, unsigned int, unsigned long);
extern long menable_compat_ioctl(struct file *, unsigned int, unsigned long);
extern void dma_clean_sync(struct menable_dmachan *db);
extern void dma_done_work(struct work_struct *);
extern void men_unblock_buffer(struct menable_dmachan *dc, struct menable_dmabuf *sb);
extern struct menable_dmabuf *men_next_blocked(struct siso_menable *men, struct menable_dmachan *dc);
extern struct menable_dmabuf *men_last_blocked(struct siso_menable *men, struct menable_dmachan *dc);
extern struct menable_dmachan *men_dma_channel(struct siso_menable *men, const unsigned int index);

int me3_probe(struct siso_menable *men);
int me4_probe(struct siso_menable *men);
int me5_probe(struct siso_menable *men);

ssize_t men_get_des_val(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t men_set_des_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t men_get_dmas(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t men_get_des_name(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t men_set_des_name(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
extern struct attribute_group *me5_attribute_groups[2];
extern struct attribute_group *me4_attribute_groups[2];
extern struct attribute_group *me3_attribute_groups[2];
#else /* LINUX < 2.6.31 */
extern const struct attribute_group *me5_attribute_groups[2];
extern const struct attribute_group *me4_attribute_groups[2];
extern const struct attribute_group *me3_attribute_groups[2];
#endif /* LINUX < 2.6.31 */

extern struct class *menable_dma_class;
extern struct device_attribute men_dma_attributes[3];

static inline int
is_me3(const struct siso_menable *men)
{
	if ((men->board > 0) && (men->board <= 2)) return 1;
	return 0;
}

static inline int
is_me4(const struct siso_menable *men)
{
	if ((men->board > 2) && (men->board <= 9)) return 1;
	return 0;
}

#endif /* MENABLE_H */
