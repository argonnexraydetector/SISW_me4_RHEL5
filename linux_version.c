/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 *
 * This adds some functions of newer kernels for older versions so
 * no #ifdef fiddling is required in-source.
 */

#include <linux/device.h>
#include <linux/version.h>
#include "linux_version.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 25)

/* taken from lib/vsprintf.c from kernel 2.6.32 */

int
strict_strtoul(const char *cp, unsigned int base, unsigned long *res)
{
	char *tail;
	unsigned long val;
	size_t len;

	*res = 0;
	len = strlen(cp);
	if (len == 0)
		return -EINVAL;

	val = simple_strtoul(cp, &tail, base);
	if (tail == cp)
		return -EINVAL;
	if ((*tail == '\0') ||
		((len == (size_t)(tail - cp) + 1) && (*tail == '\n'))) {
		*res = val;
		return 0;
	}

	return -EINVAL;
}

#endif /* LINUX < 2.6.25 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)

int dev_set_name(struct device *dev, const char *fmt, ...)
{
	va_list vargs;
	int err;

	va_start(vargs, fmt);
	err = vsnprintf(dev->bus_id, BUS_ID_SIZE, fmt, vargs);
	va_end(vargs);
	return err;
}

const char *dev_name(const struct device *dev)
{
	return dev->bus_id;
}

#endif /* LINUX < 2.6.26 */
