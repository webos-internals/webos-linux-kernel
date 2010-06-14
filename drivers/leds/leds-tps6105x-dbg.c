/*
 * drivers/leds/leds-tps6105x-dbg.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c_tps6105x_led.h>

#define TPS6105X_DBG_FLD(_name, _last, _first) \
	.name = #_name, \
	.last = _last, \
	.first = _first,

#define TPS6105X_DBG_FLD_TERM() \
	.name = NULL,

#define TPS6105X_DBG_FLDS(_name) \
	static struct tps6105x_dbg_field _name ## _fields[]

#define TPS6105X_DBG_I2C_ATTR(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO | S_IWUGO, \
		}, \
		.show = tps6105x_dbg_i2c_attribute_show, \
		.store = tps6105x_dbg_i2c_attribute_store, \
	}, \
	.idx = _idx, \
	.flds = _name ## _fields,

#define tps6105x_dbg_i2c_field_mask(_last, _first) \
	((((u8)-1) >> (7 - _last)) & (((u8)-1) << _first))

#define tps6105x_dbg_field_nibbles(_last, _first) \
	(((_last - _first) / 4) + 1)

#define tps6105x_dbg_field_format "%%s[%%d:%%d]=0x%%0%dX\n"

struct tps6105x_dbg_field {
	const char	*name;
	u8		last;
	u8		first;
};

struct tps6105x_dbg_attribute {
	struct device_attribute		attr;
	u8				idx;
	struct tps6105x_dbg_field	*flds;
};

static ssize_t
tps6105x_dbg_i2c_attribute_show(struct device *dev,
				struct device_attribute *attr, char *begin)
{
	u8 rval;
	u8 mask;
	int shift;
	int nibbles;
	char *end = begin;
	char fmt[sizeof(tps6105x_dbg_field_format)+1];
	struct i2c_client *i2c;
	struct tps6105x_dbg_field *fld;
	struct tps6105x_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct tps6105x_dbg_attribute *)attr;

	tps6105x_i2c_read_u8(i2c, reg->idx, &rval);

	end += sprintf(end, "|---------------|\n");
	end += sprintf(end, "|0 0 0 0 0 0 0 0|\n");
	end += sprintf(end, "|7 6 5 4 3 2 1 0|\n");
	end += sprintf(end, "|---------------|\n");

	for (shift = 7; 0 <= shift; --shift)
	{
		end += sprintf(end, " %d", (rval >> shift) & 0x1);
	}

	end += sprintf(end, "\n\n");

	for (fld = reg->flds; fld->name; ++fld)
	{
		mask = tps6105x_dbg_i2c_field_mask(fld->last, fld->first);
		nibbles = tps6105x_dbg_field_nibbles(fld->last, fld->first);

		sprintf(fmt, tps6105x_dbg_field_format, nibbles);
		end += sprintf(end, fmt, fld->name, fld->last, fld->first,
				(rval & mask) >> fld->first);
	}

	return (end - begin);
}

static ssize_t
tps6105x_dbg_i2c_attribute_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	u8 rval;
	u8 mask;
	unsigned int fval;
	unsigned int last;
	unsigned int first;
	struct i2c_client *i2c;
	struct tps6105x_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct tps6105x_dbg_attribute *)attr;

	if (3 != sscanf(buf, "[%u:%u]=0x%x", &last, &first, &fval)
		&& 3 != sscanf(buf, "[%u:%u]=%u", &last, &first, &fval))
		goto exit;

	if (7 < last || last < first)
		goto exit;

	mask = tps6105x_dbg_i2c_field_mask(last, first);
	tps6105x_i2c_read_u8(i2c, reg->idx, &rval);
	rval &= ~mask;
	rval |= (fval << first) & mask;
	tps6105x_i2c_write_u8(i2c, reg->idx, rval);

exit:
	return (count);
}

TPS6105X_DBG_FLDS(REGISTER0) = {
	{ TPS6105X_DBG_FLD(MODE_CTRL, 7, 6) },
	{ TPS6105X_DBG_FLD(OV, 5, 4) },
	{ TPS6105X_DBG_FLD(DIM, 3, 3) },
	{ TPS6105X_DBG_FLD(TC, 2, 0) },
	{ TPS6105X_DBG_FLD_TERM() },
};

TPS6105X_DBG_FLDS(REGISTER1) = {
	{ TPS6105X_DBG_FLD(MODE_CTRL, 7, 6) },
	{ TPS6105X_DBG_FLD(TO, 5, 5) },
	{ TPS6105X_DBG_FLD(STT, 4, 4) },
	{ TPS6105X_DBG_FLD(SFT, 3, 3) },
	{ TPS6105X_DBG_FLD(FC, 2, 0) },
	{ TPS6105X_DBG_FLD_TERM() },
};

TPS6105X_DBG_FLDS(REGISTER2) = {
	{ TPS6105X_DBG_FLD(OVERTEMP, 7, 7) },
	{ TPS6105X_DBG_FLD(LF, 6, 6) },
	{ TPS6105X_DBG_FLD(ILIM, 6, 5) },
	{ TPS6105X_DBG_FLD(ADC, 5, 3) },
	{ TPS6105X_DBG_FLD(Tx-MASK, 2, 2) },
	{ TPS6105X_DBG_FLD(GPIO, 1, 1) },
	{ TPS6105X_DBG_FLD(DIR, 0, 0) },
	{ TPS6105X_DBG_FLD_TERM() },
};

TPS6105X_DBG_FLDS(REGISTER3) = {
	{ TPS6105X_DBG_FLD(DCTIM, 7, 5) },
	{ TPS6105X_DBG_FLD(STIM, 4, 0) },
	{ TPS6105X_DBG_FLD_TERM() },
};

static struct tps6105x_dbg_attribute tps6105x_dbg_attrs[] = {
	{ TPS6105X_DBG_I2C_ATTR(REGISTER0, 0x00) },
	{ TPS6105X_DBG_I2C_ATTR(REGISTER1, 0x01) },
	{ TPS6105X_DBG_I2C_ATTR(REGISTER2, 0x02) },
	{ TPS6105X_DBG_I2C_ATTR(REGISTER3, 0x03) },
};

int
tps6105x_dbg_register_i2c_client(struct i2c_client *i2c)
{
	int i;
	int rc;
	struct device_attribute *attr;

	for (i = 0, rc = 0; ARRAY_SIZE(tps6105x_dbg_attrs) > i; ++i) {
		attr = &tps6105x_dbg_attrs[i].attr;
		if ((rc = device_create_file(&i2c->dev, attr))) {
			while (i--) {
				attr = &tps6105x_dbg_attrs[i].attr;
				device_remove_file(&i2c->dev, attr);
			}
			break;
		}
	}

	return (rc);
}

EXPORT_SYMBOL(tps6105x_dbg_register_i2c_client);

void
tps6105x_dbg_unregister_i2c_client(struct i2c_client *i2c)
{
	int i;
	struct device_attribute *attr;

	for (i = 0; ARRAY_SIZE(tps6105x_dbg_attrs) > i; ++i) {
		attr = &tps6105x_dbg_attrs[i].attr;
		device_remove_file(&i2c->dev, attr);
	}
}

EXPORT_SYMBOL(tps6105x_dbg_unregister_i2c_client);
