/************************************************************************
 * Copyright 2006-2011 Silicon Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2) as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include "menable.h"
#include "linux_version.h"

int
buf_get_uint(const char *buf, size_t count, uint32_t *res)
{
	unsigned long tmp;
	int err;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
	err = strict_strtoul(buf, 0, &tmp);
#else
	err = kstrtoul(buf, 0, &tmp);
#endif

	if (err != 0)
		return err;

#if BITS_PER_LONG > 32
	if (tmp > UINT_MAX)
		return -EINVAL;
#endif
	*res = (uint32_t) tmp;
	return 0;
}

ssize_t
men_get_des_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);

	return sprintf(buf, "%s\n", men->desname);
}

ssize_t
men_set_des_name(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);

	if (count > men->deslen - 1)
		return -EINVAL;

	memcpy(men->desname, buf, count);
	if (buf[count - 1] == '\n')
		count--;
	men->desname[count] = '\0';
	men->startirq(men);

	return count;
}

ssize_t
men_get_des_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);

	return sprintf(buf, "%i\n", men->desval);
}

ssize_t
men_set_des_val(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct siso_menable *men = container_of(dev, struct siso_menable, dev);
	uint32_t v;
	int ret;

	ret = buf_get_uint(buf, count, &v);
	if (ret)
		return ret;

	men->desval = v;

	return count;
}
