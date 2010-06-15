/*
 * Passthru Ioctls
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
