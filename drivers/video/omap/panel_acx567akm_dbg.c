/*
 * drivers/video/omap/panel_acx657akm_dbg.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Author: john chen (jchen1996@gmail.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/spi/acx567akm.h>

#define ACX567AKM_DBG_FLD(_name, _last, _first)	\
	.name = #_name,				\
	.last = _last,				\
	.first = _first,

#define ACX567AKM_DBG_FLD_TERM()	\
	.name = NULL,

#define ACX567AKM_DBG_FLDS(_name)				\
	static struct acx567akm_dbg_field _name ## _fields[]

#define ACX567AKM_DBG_I2C_ATTR(_name, _wr, _rd)			\
	.attr = {						\
		.attr = {					\
			.name = #_name,				\
			.owner = THIS_MODULE,			\
			.mode = S_IRUGO | S_IWUGO,		\
		},						\
		.show = acx567akm_dbg_spi_attribute_show,	\
		.store = acx567akm_dbg_spi_attribute_store,	\
	},							\
	.rd_index = _rd,					\
	.wr_index = _wr,					\
	.fields = _name ## _fields,

#define acx567akm_dbg_spi_field_mask(_last, _first)		\
	((((u8)-1) >> (7 - _last)) & (((u8)-1) << _first))

#define acx567akm_dbg_field_nibbles(_last, _first)	\
	(((_last - _first) / 4) + 1)

#define acx567akm_dbg_field_format "%%s[%%d:%%d]=0x%%0%dX\n"

struct acx567akm_dbg_field {
	const char	*name;
	u8		last;
	u8		first;
};

struct acx567akm_dbg_attribute {
	struct device_attribute		attr;
	u8				rd_index;
	u8				wr_index;
	struct acx567akm_dbg_field	*fields;
};

static ssize_t acx567akm_dbg_spi_attribute_show(struct device *dev,
						struct device_attribute *attr,
						char *begin)
{
	u8 val;
	u8 mask;
	int shift;
	int nibbles;
	char *end = begin;
	char fmt[sizeof(acx567akm_dbg_field_format)+1];
	struct spi_device *spi;
	struct acx567akm_dbg_field *fld;
	struct acx567akm_dbg_attribute *reg;

	spi = to_spi_device(dev);
	reg = (struct acx567akm_dbg_attribute *)attr;

	acx567akm_spi_read_u8(spi, reg->rd_index, &val);

	end += sprintf(end, "|---------------|\n");
	end += sprintf(end, "|7 6 5 4 3 2 1 0|\n");
	end += sprintf(end, "|---------------|\n");

	for (shift = 7; 0 <= shift; --shift)
	{
		end += sprintf(end, " %d", (val >> shift) & 0x1);
	}

	end += sprintf(end, "\n\n");

	for (fld = reg->fields; fld->name; ++fld)
	{
		mask = acx567akm_dbg_spi_field_mask(fld->last, fld->first);
		nibbles = acx567akm_dbg_field_nibbles(fld->last, fld->first);

		sprintf(fmt, acx567akm_dbg_field_format, nibbles);
		end += sprintf(end, fmt, fld->name, fld->last, fld->first,
				(val & mask) >> fld->first);
	}

	return (end - begin);
}

static ssize_t acx567akm_dbg_spi_attribute_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u8 val;
	u8 mask;
	unsigned int bits;
	unsigned int last;
	unsigned int first;
	struct spi_device *spi;
	struct acx567akm_dbg_attribute *reg;

	spi = to_spi_device(dev);
	reg = (struct acx567akm_dbg_attribute *)attr;

	if (3 != sscanf(buf, "[%u:%u]=0x%x", &last, &first, &bits)
		&& 3 != sscanf(buf, "[%u:%u]=%u", &last, &first, &bits))
		goto exit;

	if (7 < last || last < first)
		goto exit;

	mask = acx567akm_dbg_spi_field_mask(last, first);
	acx567akm_spi_read_u8(spi, reg->rd_index, &val);
	val &= ~mask;
	val |= (bits << first) & mask;
	acx567akm_spi_write_u8(spi, reg->wr_index, val);
exit:
	return (count);
}

ACX567AKM_DBG_FLDS(LCDBV) = {
	{ ACX567AKM_DBG_FLD(LBV, 7, 0) },
	{ ACX567AKM_DBG_FLD_TERM() },
};

ACX567AKM_DBG_FLDS(CTRLL) = {
	{ ACX567AKM_DBG_FLD(BCTRL, 5, 5) },
	{ ACX567AKM_DBG_FLD(LD, 3, 3) },
	{ ACX567AKM_DBG_FLD(BL, 2, 2) },
	{ ACX567AKM_DBG_FLD_TERM() },
};

static struct acx567akm_dbg_attribute acx567akm_dbg_attrs[] = {
	{ ACX567AKM_DBG_I2C_ATTR(LCDBV, 0x51, 0x52) },
	{ ACX567AKM_DBG_I2C_ATTR(CTRLL, 0x53, 0x54) },
};

int acx567akm_dbg_register_spi_device(struct spi_device *spi)
{
	int i;
	int rc;
	struct device_attribute *attr;

	for (i = 0, rc = 0; ARRAY_SIZE(acx567akm_dbg_attrs) > i; ++i) {
		attr = &acx567akm_dbg_attrs[i].attr;
		if ((rc = device_create_file(&spi->dev, attr))) {
			while (i--) {
				attr = &acx567akm_dbg_attrs[i].attr;
				device_remove_file(&spi->dev, attr);
			}
			break;
		}
	}

	return (rc);
}

EXPORT_SYMBOL(acx567akm_dbg_register_spi_device);

void acx567akm_dbg_unregister_spi_device(struct spi_device *spi)
{
	int i;
	struct device_attribute *attr;

	for (i = 0; ARRAY_SIZE(acx567akm_dbg_attrs) > i; ++i) {
		attr = &acx567akm_dbg_attrs[i].attr;
		device_remove_file(&spi->dev, attr);
	}
}

EXPORT_SYMBOL(acx567akm_dbg_unregister_spi_device);
