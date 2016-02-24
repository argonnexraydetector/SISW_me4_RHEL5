/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
#include <linux/spinlock_types.h>
#include <asm/cmpxchg.h>
#endif

#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/vmalloc.h>

#include "menable.h"
#include "uiq.h"
#include "linux_version.h"

static dev_t devr;
static DEFINE_SPINLOCK(idxlock);

static struct lock_class_key men_idx_lock;
static struct lock_class_key men_board_lock;
static struct lock_class_key men_design_lock;
static struct lock_class_key men_head_lock;
static int maxidx;

static void menable_obj_release(struct device *dev)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);

	spin_lock(&idxlock);
	if (men->idx == maxidx - 1)
		maxidx--;
	spin_unlock(&idxlock);
	kfree(men);
}

static void
men_cleanup_channels(struct siso_menable *men)
{
	int j;
	unsigned int skipdma = 0;

	for (j = 0; j < men->active_fpgas; j++) {
		int i;
		for (i = 0; i < men->dmacnt[j]; i++) {
			struct menable_dmachan *dc = men_dma_channel(men, skipdma + i);
			if (dc->running == 1)
				men_stop_dma_locked(dc);
			if (dc->active) {
				dc->active->chan = NULL;
				dc->active = NULL;
			}
		}
		skipdma += men->dmacnt[j];
	}

	men->stopirq(men);

	for (j = 0; j < men->active_fpgas; j++) {
		int i;
		for (i = 0; i < men->uiqcnt[j]; i++) {
			struct menable_uiq *uiq = men->uiqs[j * MEN_MAX_UIQ_PER_FPGA + i];

			if (uiq == NULL)
				continue;

			spin_lock(&uiq->lock);
			uiq->running = false;
			spin_unlock(&uiq->lock);
		}
	}
}

static void
men_cleanup_mem(struct siso_menable *men)
{
	struct menable_dmahead *dh, *tmp;
	struct list_head heads;

	INIT_LIST_HEAD(&heads);
	list_for_each_entry_safe(dh, tmp, &men->heads, node) {
		int r = men_release_buf_head(men, dh);
		WARN_ON(r != 0);
		INIT_LIST_HEAD(&dh->node);
		list_add_tail(&dh->node, &heads);
	}

	WARN_ON(men->headcnt != 0);
	spin_unlock_bh(&men->headlock);

	list_for_each_entry_safe(dh, tmp, &heads, node) {
		men_free_buf_head(men, dh);
	}
}

static int menable_open(struct inode *inode, struct file *file)
{
	struct siso_menable *men = container_of(inode->i_cdev, struct siso_menable, cdev);
	int ret;
	unsigned long flags;

	file->private_data = men;

	if (men->open) {
		ret = men->open(men, file);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&men->boardlock, flags);
	if (men->use == 0)
		men->startirq(men);
	men->use++;
	spin_unlock_irqrestore(&men->boardlock, flags);
	return 0;
}

static int menable_release(struct inode *inode, struct file *file)
{
	struct siso_menable *men = file->private_data;
	unsigned long flags;

	spin_lock_bh(&men->headlock);
	spin_lock_irqsave(&men->boardlock, flags);

	if (--men->use == 0) {
		men_cleanup_channels(men);
		spin_unlock_irqrestore(&men->boardlock, flags);
		men_cleanup_mem(men);

		if (men->cleanup != NULL)
			men->cleanup(men);
	} else {
		spin_unlock_irqrestore(&men->boardlock, flags);
		spin_unlock_bh(&men->headlock);
	}

	return 0;
}

static const struct file_operations menable_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = menable_ioctl,
	.compat_ioctl = menable_compat_ioctl,
	.open = menable_open,
	.release = menable_release,
};

static struct class *menable_class;

static DEFINE_PCI_DEVICE_TABLE(menable_pci_table) = {
	/* microEnable III/XXL */
	{ .vendor = PCI_VENDOR_PLX, .device = 0x9656,
	  .subvendor = PCI_VENDOR_PLX, .subdevice = 0x1087, .driver_data = 1 },
	/* microEnable III */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a30), .driver_data = 1 },
	/* microEnable III-XXL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a31), .driver_data = 2 },
	/* microEnable IV AD1-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a40), .driver_data = 3 },
	/* microEnable IV VD1-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a41), .driver_data = 4 },
	/* microEnable IV AD4-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a42), .driver_data = 5 },
	/* microEnable IV VD4-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a44), .driver_data = 6 },
	/* microEnable IV AS1-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a45), .driver_data = 7 },
	/* microEnable IV AQ4-GE */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0e42), .driver_data = 8 },
	/* microEnable IV VQ4-GE */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0e44), .driver_data = 9 },
	/* microEnable V AQ8-CXP6B */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a53), .driver_data = 10 },
	/* microEnable V VQ8-CXP6B */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a54), .driver_data = 11 },
	/* microEnable V AD8-CLHS */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a55), .driver_data = 12 },
	/* microEnable V VQ8-CXP6D */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a56), .driver_data = 13 },
	/* microEnable V AQ8-CXP6D */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a57), .driver_data = 14 },
	/* microEnable V VD8-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a58), .driver_data = 15 },
	/* microEnable V AD8-CL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0a5a), .driver_data = 16 },
	/* LightBridge Prototype */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0750), .driver_data = 17 },
	/* LightBridge/Marathon VCL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0751), .driver_data = 18 },
	/* Marathon AF2 */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0752), .driver_data = 19 },
	/* Marathon ACX QP */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0753), .driver_data = 20 },
	/* LightBridge/Marathon ACL */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0754), .driver_data = 21 },
	/* Marathon ACX SP */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0755), .driver_data = 22 },
	/* Marathon ACX DP */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0756), .driver_data = 23 },
	/* Marathon VCX QP */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0757), .driver_data = 24 },
	/* Marathon AF2 */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0758), .driver_data = 25 },
	/* Abacus 4G */
	{ PCI_DEVICE(PCI_VENDOR_SISO, 0x0b51), .driver_data = 26 },
	/* EOT */
	{ 0 },
};

MODULE_DEVICE_TABLE(pci, menable_pci_table);

static const char __devinitdata *
men_boards[] = {
	"unknown",
	"microEnable III",
	"microEnable III-XXL",
	"microEnable IV AD1-CL",
	"microEnable IV VD1-CL",
	"microEnable IV AD4-CL",
	"microEnable IV VD4-CL",
	"microEnable IV AS1-CL",
	"microEnable IV AQ4-GE",
	"microEnable IV VQ4-GE",
	"microEnable 5 AQ8-CXP6B",
	"microEnable 5 VQ8-CXP6B",
	"microEnable 5 AD8-CLHS",
	"microEnable 5 VQ8-CXP6D",
	"microEnable 5 AQ8-CXP6D",
	"microEnable 5 VD8-CL",
	"microEnable 5 AD8-CL",
	"LightBridge Prototype",
	"LightBridge / mE5 marathon VCL",
	"mE5 marathon AF2",
	"mE5 marathon ACX QP",
	"LightBridge / mE5 marathon ACL",
	"mE5 marathon ACX SP",
	"mE5 marathon ACX DP",
	"mE5 marathon VCX QP",
	"mE5 marathon VF2",
	"microEnable 5 Abacus 4G",
};

static void __devexit menable_pci_remove(struct pci_dev *pdev)
{
	struct siso_menable *men = pci_get_drvdata(pdev);
	int i;
	unsigned long flags;

	get_device(&men->dev);

	sysfs_remove_link(&men->dev.kobj, "pci_dev");

	spin_lock_bh(&men->headlock);
	spin_lock_irqsave(&men->designlock, flags);
	while (men->design_changing) {
		spin_unlock_irqrestore(&men->designlock, flags);
		spin_unlock_bh(&men->headlock);
		schedule();
		spin_lock_bh(&men->headlock);
		spin_lock_irqsave(&men->designlock, flags);
	}
	men->design_changing = true;
	spin_unlock_irqrestore(&men->designlock, flags);

	spin_lock_irqsave(&men->boardlock, flags);
	men->stopirq(men);
	men->releasing = true;
	spin_unlock_irqrestore(&men->boardlock, flags);

	i = men_alloc_dma(men, 0);
	BUG_ON(i != 0);

	devm_free_irq(&men->pdev->dev, men->pdev->irq, men);

	men_del_uiqs(men, 0);

	men->exit(men);

	device_unregister(&men->dev);
	cdev_del(&men->cdev);

	pci_set_drvdata(pdev, NULL);
	put_device(&men->dev);
}

static int __devinit menable_pci_probe(struct pci_dev *,
				const struct pci_device_id *);

static struct pci_driver menable_pci_driver = {
	.name =		DRIVER_NAME,
	.id_table =	menable_pci_table,
	.probe =	menable_pci_probe,
	.remove =	__devexit_p(menable_pci_remove),
};

static int __devinit menable_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct siso_menable *men;
	int ret;
	int board;
	unsigned long flags;

	printk(KERN_INFO "%s: probing device %i", DRIVER_NAME, maxidx);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pcim_enable_device() failed\n");
		return ret;
	}

	board = id->driver_data;

	men = kzalloc(sizeof(*men), GFP_KERNEL);
	if (men == NULL)
		return -ENOMEM;

	men->owner = menable_class->owner;
	men->dev.parent = &pdev->dev;
	men->pdev = pdev;
	men->dev.release = menable_obj_release;
	men->dev.class = menable_class;
	men->dev.driver = &menable_pci_driver.driver;
	spin_lock(&idxlock);
	men->idx = maxidx++;
	spin_unlock(&idxlock);
	dev_set_name(&men->dev, "menable%i", men->idx);

	men->board = board;
	men->releasing = false;
	men->design_changing = true;

	pci_set_master(pdev);
	pci_set_drvdata(pdev, men);
	spin_lock_init(&men->boardlock);
	lockdep_set_class(&men->boardlock, &men_board_lock);
	spin_lock_init(&men->designlock);
	lockdep_set_class(&men->designlock, &men_design_lock);
	spin_lock_init(&men->headlock);
	lockdep_set_class(&men->headlock, &men_head_lock);
	INIT_LIST_HEAD(&men->heads);

	cdev_init(&men->cdev, &menable_fops);
	men->cdev.owner = men->dev.class->owner;
	kobject_set_name(&men->cdev.kobj, "menablec%i", men->idx);

	ret = cdev_add(&men->cdev, devr + men->idx, 1);
	if (ret)
		goto err_release;

	men->dev.devt = men->cdev.dev;

	if (is_me3(men))
		men->dev.groups = me3_attribute_groups;
	else if (is_me4(men))
		men->dev.groups = me4_attribute_groups;
	else
    {   
    /*!! tjm removed tbelow...*/
		/*men->dev.groups = me5_attribute_groups;*/
        /* tim added below*/
        printk(KERN_INFO "ERROR:No me5 support on RHEL\n");
        men->dev.groups = me4_attribute_groups;
        /*doen tim madden*/
    }
	ret = device_register(&men->dev);
	if (ret)
		goto err_cdev;

	dev_info(&men->dev, "device is a %s\n", men_boards[board]);

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret)
		goto err_dev;

	men->runtime_base = pcim_iomap(pdev, 0, 0);
	if (men->runtime_base == NULL)
		goto err_dev;

	if (is_me3(men))
		ret = me3_probe(men);
	else if (is_me4(men))
		ret = me4_probe(men);
	else
    {
       
		/*ret = me5_probe(men);*/
        /*tim madden add*/
        ret = me4_probe(men);
        printk(KERN_ERR "ERROR- no ME5 support RHEL\n");
        /*tim madden done*/
    }

	if (ret)
		goto err_board_dev;

	BUG_ON(men->active_fpgas == 0);
	BUG_ON(men->active_fpgas > MAX_FPGAS);

	ret = sysfs_create_link(&men->dev.kobj, &men->pdev->dev.kobj,
			"pci_dev");
	if (ret)
		goto err_sysfs_link;

	ret = men_add_dmas(men);
	if (ret)
		goto err_dma_scale;

	spin_lock_irqsave(&men->boardlock, flags);
	men->design_changing = false;
	spin_unlock_irqrestore(&men->boardlock, flags);

	return 0;
err_dma_scale:
	sysfs_remove_link(&men->dev.kobj, "pci_dev");
err_sysfs_link:
	men_del_uiqs(men, 0);
	men->exit(men);
err_board_dev:
err_dev:
	device_unregister(&men->dev);
err_cdev:
	cdev_del(&men->cdev);
err_release:
	pci_set_drvdata(pdev, NULL);
	spin_lock(&idxlock);
	if (men->idx == maxidx - 1)
		maxidx--;
	spin_unlock(&idxlock);
	kfree(men);
	return ret;
}

static struct device_attribute men_device_attributes[3] = {
	__ATTR(dma_channels, 0444, men_get_dmas, NULL),
	__ATTR(design_name, 0660, men_get_des_name, men_set_des_name),
	__ATTR_NULL,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)

static struct attribute * men_device_attrs[3] = {
	&men_device_attributes[0].attr,
	&men_device_attributes[1].attr,
	NULL
};

ATTRIBUTE_GROUPS(men_device);

static struct attribute * men_dma_attrs[3] = {
	&men_dma_attributes[0].attr,
	&men_dma_attributes[1].attr,
	NULL
};

ATTRIBUTE_GROUPS(men_dma);

static struct attribute * men_uiq_attrs[6] = {
	&men_uiq_attributes[0].attr,
	&men_uiq_attributes[1].attr,
	&men_uiq_attributes[2].attr,
	&men_uiq_attributes[3].attr,
	&men_uiq_attributes[4].attr,
	NULL
};

ATTRIBUTE_GROUPS(men_uiq);

#endif /* LINUX >= 3.12.0 */

static int __init menable_init(void)
{
	int ret;

	// Some kernels don't initialise statics to zero ...
	maxidx = 0;
	printk(KERN_INFO  "%s: loading %s %s Version %s\n", DRIVER_NAME, DRIVER_VENDOR, DRIVER_DESCRIPTION, DRIVER_VERSION);
	
	lockdep_set_class(&idxlock, &men_idx_lock);

	ret = alloc_chrdev_region(&devr, 0, MEN_MAX_NUM, DRIVER_NAME);
	if (ret) {
		printk(KERN_ERR "%s: failed to register device numbers\n",
				DRIVER_NAME);
		return ret;
	}

	menable_class = class_create(THIS_MODULE, "menable");
	if (IS_ERR(menable_class)) {
		ret = PTR_ERR(menable_class);
		goto err_class;
	}

	menable_uiq_class = class_create(THIS_MODULE, "menable_uiq");
	if (IS_ERR(menable_uiq_class)) {
		ret = PTR_ERR(menable_uiq_class);
		goto err_uiq_class;
	}

	menable_dma_class = class_create(THIS_MODULE, "menable_dma");
	if (IS_ERR(menable_dma_class)) {
		ret = PTR_ERR(menable_dma_class);
		goto err_dma_class;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
	menable_class->dev_attrs = men_device_attributes;
	menable_uiq_class->dev_attrs = men_uiq_attributes;
	menable_dma_class->dev_attrs = men_dma_attributes;
#else /* LINUX < 3.12.0 */
	menable_class->dev_groups = men_device_groups;
	menable_uiq_class->dev_groups = men_uiq_groups;
	menable_dma_class->dev_groups = men_dma_groups;
#endif /* LINUX < 3.12.0 */

	ret = pci_register_driver(&menable_pci_driver);
	if (ret)
		goto err_reg;

	return 0;
err_reg:
	class_destroy(menable_dma_class);
err_dma_class:
	class_destroy(menable_uiq_class);
err_uiq_class:
	class_destroy(menable_class);
err_class:
	unregister_chrdev_region(devr, MEN_MAX_NUM);
	return ret;
}

static void __exit menable_exit(void)
{
	pci_unregister_driver(&menable_pci_driver);
	class_destroy(menable_dma_class);
	class_destroy(menable_uiq_class);
	class_destroy(menable_class);
	unregister_chrdev_region(devr, MEN_MAX_NUM);
}

module_init(menable_init);
module_exit(menable_exit);

MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_AUTHOR(DRIVER_VENDOR);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
