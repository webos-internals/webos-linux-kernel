/*
 * Copyright (C) 2008-2009 Palm, Inc
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
 *
 */


#define PIOC_REQ_DATA_SIZE	16
#define PIOC_NOTIF_DATA_SIZE	16

struct pioc_cdc_control_request {
	unsigned char bmRequestType;	/* 0x21 or 0xA1 */
	unsigned char bRequest;
	unsigned short wValue;		/* Value (LE) */
	unsigned short wIndex;		/* Interface (LE) */
	unsigned short wLength;		/* Length of Data (LE) */
	unsigned char data[PIOC_REQ_DATA_SIZE];
} __attribute__ ((packed));

struct pioc_cdc_notification {
	unsigned char bmRequestType;	/* 0xA1 */
	unsigned char bNotificationType;
	unsigned short wValue;		/* Value (LE) */
	unsigned short wIndex;		/* Interface (LE) */
	unsigned short wLength;		/* Length of Data (LE) */
	unsigned char data[PIOC_NOTIF_DATA_SIZE];
} __attribute__ ((packed));

#define PIOCSENDCTLREQ	_IOW('p', 101, struct pioc_cdc_control_request)
#define PIOCRECVNOTIF	_IOR('p', 102, struct pioc_cdc_notification)

#define PIOCSENDNOTIF	_IOW('p', 103, struct pioc_cdc_control_request)
#define PIOCRECVCTLREQ	_IOR('p', 104, struct pioc_cdc_notification)
