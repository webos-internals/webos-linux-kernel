/*
 * MUSB OTG driver - support for Mentor's DMA controller
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2007 by Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include "musb_core.h"
#include <asm/arch/dma.h>

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
#include "omap2430.h"
#endif

#ifdef CONFIG_MACH_SIRLOIN_3630
/* OMAP3630 v1.1 bug "1.85 USB OTG DMA may stall under specific condition"
 *
 * Workaround:
 * 1) always use INCR16 burst mode (0x3)
 * 2) Tx DMA ADDR should be 32 bytes aligned
 * 3) Tx DMA COUNT > 16
 */
#define BUG_OMAP3630_MENTOR_DMA
#endif

#define MUSB_HSDMA_BASE		0x200
#define MUSB_HSDMA_INTR		(MUSB_HSDMA_BASE + 0)
#define MUSB_HSDMA_CONTROL		0x4
#define MUSB_HSDMA_ADDRESS		0x8
#define MUSB_HSDMA_COUNT		0xc

#define MUSB_HSDMA_CHANNEL_OFFSET(_bChannel, _offset)		\
		(MUSB_HSDMA_BASE + (_bChannel << 4) + _offset)

/* control register (16-bit): */
#define MUSB_HSDMA_ENABLE_SHIFT		0
#define MUSB_HSDMA_TRANSMIT_SHIFT		1
#define MUSB_HSDMA_MODE1_SHIFT		2
#define MUSB_HSDMA_IRQENABLE_SHIFT		3
#define MUSB_HSDMA_ENDPOINT_SHIFT		4
#define MUSB_HSDMA_BUSERROR_SHIFT		8
#define MUSB_HSDMA_BURSTMODE_SHIFT		9
#define MUSB_HSDMA_BURSTMODE		(3 << MUSB_HSDMA_BURSTMODE_SHIFT)
#define MUSB_HSDMA_BURSTMODE_UNSPEC	0
#define MUSB_HSDMA_BURSTMODE_INCR4	1
#define MUSB_HSDMA_BURSTMODE_INCR8	2
#define MUSB_HSDMA_BURSTMODE_INCR16	3

#define MUSB_HSDMA_CHANNELS		8

#define MUSB_FIFO_ADDRESS(epnum)	\
	((unsigned long) (OMAP_HSOTG_BASE + MUSB_FIFO_OFFSET(epnum)))

struct musb_dma_controller;

struct musb_dma_channel {
	struct dma_channel		Channel;
	struct musb_dma_controller	*controller;
	u32				dwStartAddress;
	u32				len;
	u16				wMaxPacketSize;
	u8				bIndex;
	u8				epnum;
	u8				transmit;
	int				sysdma_channel;
};

struct musb_dma_controller {
	struct dma_controller		Controller;
	struct musb_dma_channel		aChannel[MUSB_HSDMA_CHANNELS];
	void				*pDmaPrivate;
	void __iomem			*pCoreBase;
	u8				bChannelCount;
	u8				bmUsedChannels;
	u8				irq;
};

static int dma_controller_start(struct dma_controller *c)
{
	/* nothing to do */
	return 0;
}

#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
void musb_sysdma_completion(int lch, u16 ch_status, void *data);
#endif

static void dma_channel_release(struct dma_channel *pChannel);

static int dma_controller_stop(struct dma_controller *c)
{
	struct musb_dma_controller *controller =
		container_of(c, struct musb_dma_controller, Controller);
	struct musb *musb = (struct musb *) controller->pDmaPrivate;
	struct dma_channel *pChannel;
	u8 bBit;

	if (controller->bmUsedChannels != 0) {
		dev_err(musb->controller,
			"Stopping DMA controller while channel active\n");

		for (bBit = 0; bBit < MUSB_HSDMA_CHANNELS; bBit++) {
			if (controller->bmUsedChannels & (1 << bBit)) {
				pChannel = &controller->aChannel[bBit].Channel;
				dma_channel_release(pChannel);

				if (!controller->bmUsedChannels)
					break;
			}
		}
	}
	return 0;
}

static struct dma_channel *dma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 transmit)
{
	u8 bBit;
	struct dma_channel *pChannel = NULL;
	struct musb_dma_channel *pImplChannel = NULL;
	struct musb_dma_controller *controller =
			container_of(c, struct musb_dma_controller, Controller);

	for (bBit = 0; bBit < MUSB_HSDMA_CHANNELS; bBit++) {
		if (!(controller->bmUsedChannels & (1 << bBit))) {
			controller->bmUsedChannels |= (1 << bBit);
			pImplChannel = &(controller->aChannel[bBit]);
			pImplChannel->controller = controller;
			pImplChannel->bIndex = bBit;
			pImplChannel->epnum = hw_ep->epnum;
			pImplChannel->transmit = transmit;
			pChannel = &(pImplChannel->Channel);
			pChannel->private_data = pImplChannel;
			pChannel->status = MUSB_DMA_STATUS_FREE;
			pChannel->max_len = 0x10000;
			/* Tx => mode 1; Rx => mode 0 */
			pChannel->desired_mode = transmit;
			pChannel->actual_len = 0;
			pImplChannel->sysdma_channel = -1;

#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
			if (!transmit) {
				int ret;
				ret = omap_request_dma(OMAP24XX_DMA_NO_DEVICE,
					"MUSB SysDMA", musb_sysdma_completion,
					(void *) pImplChannel,
					&(pImplChannel->sysdma_channel));
/* FIXME: Decide on the correct private data to use */

				if (ret) {
					printk(KERN_ERR "request_dma failed:"
							" %d\n", ret);
					controller->bmUsedChannels &=
								~(1 << bBit);
					pChannel->status =
							MUSB_DMA_STATUS_UNKNOWN;
					pImplChannel->sysdma_channel = -1;
					pChannel = NULL;
				}
			}
#endif

			break;
		}
	}
	return pChannel;
}

static void dma_channel_release(struct dma_channel *pChannel)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->private_data;

	pChannel->actual_len = 0;
	pImplChannel->dwStartAddress = 0;
	pImplChannel->len = 0;

	pImplChannel->controller->bmUsedChannels &=
		~(1 << pImplChannel->bIndex);

	pChannel->status = MUSB_DMA_STATUS_UNKNOWN;

#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
	if (pImplChannel->sysdma_channel != -1) {
		omap_stop_dma(pImplChannel->sysdma_channel);
		omap_free_dma(pImplChannel->sysdma_channel);
		pImplChannel->sysdma_channel = -1;
	}
#endif
}

static void configure_channel(struct dma_channel *pChannel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->private_data;
	struct musb_dma_controller *controller = pImplChannel->controller;
	void __iomem *mbase = controller->pCoreBase;
	u8 bChannel = pImplChannel->bIndex;
	u16 csr = 0;

	DBG(4, "%p, pkt_sz %d, addr 0x%x, len %d, mode %d\n",
			pChannel, packet_sz, dma_addr, len, mode);

#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
	if (pImplChannel->sysdma_channel != -1) {
	/* System DMA */
	/* RX: set src = FIFO */

		if (len == 0) {
			/*
			 * No need to do a transfer of length 0, not to mention
			 * that it causes a DMA misalignment error.  Just notify
			 * the musb core that the transfer has completed.
			 */
			struct musb *musb = controller->pDmaPrivate;

			pChannel->actual_len = 0;
			pChannel->status = MUSB_DMA_STATUS_FREE;
			musb_dma_completion(musb, pImplChannel->epnum,
				pImplChannel->transmit);
			return;
		}

		omap_set_dma_transfer_params(pImplChannel->sysdma_channel,
					OMAP_DMA_DATA_TYPE_S8,
					len, 1, /* One frame */
					OMAP_DMA_SYNC_ELEMENT,
					OMAP24XX_DMA_NO_DEVICE,
					0); /* Src Sync */

		omap_set_dma_src_params(pImplChannel->sysdma_channel, 0,
					OMAP_DMA_AMODE_CONSTANT,
					MUSB_FIFO_ADDRESS(pImplChannel->epnum),
					0, 0);

		omap_set_dma_dest_params(pImplChannel->sysdma_channel, 0,
					OMAP_DMA_AMODE_POST_INC, dma_addr,
					0, 0);

/* FIXME: Packing and burst mode on src side
 * Verify if this actually adds value
 */
		omap_set_dma_src_data_pack(pImplChannel->sysdma_channel, 1);
		omap_set_dma_src_burst_mode(pImplChannel->sysdma_channel,
					OMAP_DMA_DATA_BURST_16);

		omap_set_dma_dest_data_pack(pImplChannel->sysdma_channel, 1);
		omap_set_dma_dest_burst_mode(pImplChannel->sysdma_channel,
					OMAP_DMA_DATA_BURST_16);

		omap_start_dma(pImplChannel->sysdma_channel);

	} else
#endif
	{ /* Mentor DMA */
		if (mode) {
			csr |= 1 << MUSB_HSDMA_MODE1_SHIFT;
			BUG_ON(len < packet_sz);

#if !defined(BUG_OMAP3630_MENTOR_DMA)
			if (packet_sz >= 64) {
				csr |= MUSB_HSDMA_BURSTMODE_INCR16
					<< MUSB_HSDMA_BURSTMODE_SHIFT;
			} else if (packet_sz >= 32) {
				csr |= MUSB_HSDMA_BURSTMODE_INCR8
					<< MUSB_HSDMA_BURSTMODE_SHIFT;
			} else if (packet_sz >= 16) {
				csr |= MUSB_HSDMA_BURSTMODE_INCR4
					<< MUSB_HSDMA_BURSTMODE_SHIFT;
			}
#endif
		}
#ifdef BUG_OMAP3630_MENTOR_DMA
		/* always use INCR16 */
		csr |= MUSB_HSDMA_BURSTMODE_INCR16
			<< MUSB_HSDMA_BURSTMODE_SHIFT;
#endif
		csr |= (pImplChannel->epnum << MUSB_HSDMA_ENDPOINT_SHIFT)
			| (1 << MUSB_HSDMA_ENABLE_SHIFT)
			| (1 << MUSB_HSDMA_IRQENABLE_SHIFT)
			| (pImplChannel->transmit
					? (1 << MUSB_HSDMA_TRANSMIT_SHIFT)
					: 0);

		/* address/count */
		musb_writel(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_ADDRESS),
			dma_addr);
		musb_writel(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_COUNT),
			len);

		/* control (this should start things) */
		musb_writew(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_CONTROL),
			csr);
	} /* Mentor DMA */
}

#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
void musb_sysdma_completion(int lch, u16 ch_status, void *data)
{
	u32 dwAddress;
	unsigned long flags;

	struct dma_channel *pChannel;

	struct musb_dma_channel *pImplChannel =
					(struct musb_dma_channel *) data;
	struct musb_dma_controller *controller = pImplChannel->controller;
	struct musb *musb = controller->pDmaPrivate;
	pChannel = &pImplChannel->Channel;

	DBG(2, "lch = 0x%d, ch_status = 0x%x\n", lch, ch_status);
	spin_lock_irqsave(&musb->lock, flags);

	dwAddress = (u32) omap_get_dma_dst_pos(pImplChannel->sysdma_channel);
	pChannel->actual_len = dwAddress - pImplChannel->dwStartAddress;

	DBG(2, "ch %p, 0x%x -> 0x%x (%d / %d) %s\n",
		pChannel, pImplChannel->dwStartAddress, dwAddress,
		pChannel->actual_len, pImplChannel->len,
		(pChannel->actual_len < pImplChannel->len) ?
		"=> reconfig 0": "=> complete");

	pChannel->status = MUSB_DMA_STATUS_FREE;
	musb_dma_completion(musb, pImplChannel->epnum, pImplChannel->transmit);

	spin_unlock_irqrestore(&musb->lock, flags);
	return;
}
#endif

static int dma_channel_program(struct dma_channel *pChannel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct musb_dma_channel *pImplChannel =
			(struct musb_dma_channel *) pChannel->private_data;
	struct musb_dma_controller *controller = pImplChannel->controller;
	struct musb *musb = controller->pDmaPrivate;

	DBG(2, "ep%d-%s pkt_sz %d, dma_addr 0x%x length %d, mode %d\n",
		pImplChannel->epnum,
		pImplChannel->transmit ? "Tx" : "Rx",
		packet_sz, dma_addr, len, mode);

	BUG_ON(pChannel->status == MUSB_DMA_STATUS_UNKNOWN ||
		pChannel->status == MUSB_DMA_STATUS_BUSY);

	/* On MUSB:RTL1.8 and above, DMA has to be word aligned */
	if ((dma_addr % 4) && (musb->hwvers >= MUSB_HWVERS_1800)) {
		/* Fail DMA for unaligned buffers:
		 * Use PIO for such buffers
		 */
		return false;
	}

#ifdef BUG_OMAP3630_MENTOR_DMA
	/* Tx DMA COUNT has to be >16 */
	if (pImplChannel->transmit && len <= 16)
		return false;
#endif

	pChannel->actual_len = 0;
	pImplChannel->dwStartAddress = dma_addr;
	pImplChannel->len = len;
	pImplChannel->wMaxPacketSize = packet_sz;
	pChannel->status = MUSB_DMA_STATUS_BUSY;

	if ((mode == 1) && (len >= packet_sz))
		configure_channel(pChannel, packet_sz, 1, dma_addr, len);
	else
		configure_channel(pChannel, packet_sz, 0, dma_addr, len);

	return true;
}

static int dma_channel_abort(struct dma_channel *pChannel)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->private_data;
	u8 bChannel = pImplChannel->bIndex;
	void __iomem *mbase = pImplChannel->controller->pCoreBase;
	u16 csr;

	if (pChannel->status == MUSB_DMA_STATUS_BUSY) {
		if (pImplChannel->transmit) {

			csr = musb_readw(mbase,
				MUSB_EP_OFFSET(pImplChannel->epnum, MUSB_TXCSR));
			csr &= ~(MUSB_TXCSR_AUTOSET |
				 MUSB_TXCSR_DMAENAB |
				 MUSB_TXCSR_DMAMODE);
			musb_writew(mbase,
				MUSB_EP_OFFSET(pImplChannel->epnum, MUSB_TXCSR),
				csr);
		} else {
#ifdef CONFIG_USB_USE_SYSTEM_DMA_RX
			if (pImplChannel->sysdma_channel != -1) {
				omap_stop_dma(pImplChannel->sysdma_channel);
				omap_free_dma(pImplChannel->sysdma_channel);
				pImplChannel->sysdma_channel = -1;
			}
#endif
			csr = musb_readw(mbase,
				MUSB_EP_OFFSET(pImplChannel->epnum, MUSB_RXCSR));
			csr &= ~(MUSB_RXCSR_AUTOCLEAR |
				 MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(mbase,
				MUSB_EP_OFFSET(pImplChannel->epnum, MUSB_RXCSR),
				csr);
		}

		musb_writew(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_CONTROL),
			0);
		musb_writel(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_ADDRESS),
			0);
		musb_writel(mbase,
			MUSB_HSDMA_CHANNEL_OFFSET(bChannel, MUSB_HSDMA_COUNT),
			0);

		pChannel->status = MUSB_DMA_STATUS_FREE;
	}
	return 0;
}


/*
 * This is a hack to enable KGDB over ethernet over USB.
 *
 * We are using the kgdboe module to connect to the target. Typically the
 * target will use net_poll_xxx to poll the ethernet device to detect gdb UDP
 * packets. net_poll_xxx functions tickle the interrupt handler of the ethernet
 * device in a tight loop in order to send/receive UDP packets to/from the
 * host. In the case of ethernet devices this works well as the net_poll_xxx
 * functions sit directly on top of the ethernet driver for the hardware.
 *
 * In the case of ethernet over USB however, the net_poll_xxx functions sit on
 * top of the USB ethernet gadget driver which has no control over the USB
 * interrupt handler. In order to make net_poll_xxx work for this case we need
 * to add this hook to tickle the USB interrupt handler directly.
 *
 * This is not a clean design and requires the USB ether gadget layer to know
 * about the underlying USB driver. It also requires that the USB driver
 * remembers its pointer to the private driver data, as the net_poll_xxx layer
 * has not way to provide it. Oh well...
 */
#if defined(CONFIG_NET_POLL_CONTROLLER) && defined(CONFIG_KGDBOE_OVER_USB)
#include <linux/kgdb.h>
static irqreturn_t dma_controller_irq(int irq, void *private_data);

/* Pointer to the driver's private data. Will be initialized in
 * dma_controller_create(). */
static void *private_data_net_poll = NULL;

void netpoll_eth_over_usb_poll(void)
{
	if (!private_data_net_poll)
		return;

	(irqreturn_t) dma_controller_irq(0, private_data_net_poll);
	/*                               |
	 * IRQ number is ignored in dma_controller_irq(), pass 0.
	 */
}
#endif


static irqreturn_t dma_controller_irq(int irq, void *private_data)
{
	struct musb_dma_controller *controller =
		(struct musb_dma_controller *)private_data;
	struct musb_dma_channel *pImplChannel;
	struct musb *musb = controller->pDmaPrivate;
	void __iomem *mbase = controller->pCoreBase;
	struct dma_channel *pChannel;
	u8 bChannel;
	u32 count;
	u16 csr;
	u32 dwAddress;
	u8 int_hsdma;
	irqreturn_t retval = IRQ_NONE;
	unsigned long flags;

	spin_lock_irqsave(&musb->lock, flags);
#ifdef CONFIG_PM
	/* REVISIT do we need this? */
	if (musb->asleep) {
		/* we might get a Bus Error interrupt 
		 * after dma_channel_abort
		 */
		retval = IRQ_HANDLED;
		goto done;
	}
#endif

	int_hsdma = musb_readb(mbase, MUSB_HSDMA_INTR);

#if defined(CONFIG_NET_POLL_CONTROLLER) && defined(CONFIG_KGDBOE_OVER_USB)
	/* If the system is sitting in the GDB breakpoint loop, then we are
	 * being called by the NETPOLL layer. In this case we "simulate"
	 * interrupts on endpoints 1 and 2 (RX/TX) so the interrupt handler
	 * does spin at least once.
	 */
	if (atomic_read(&debugger_active) != 0) {
		int_hsdma |= ((1<<1) | (1<<2));
	}
#endif

	if (!int_hsdma) {
		DBG(2, "spurious DMA irq\n");

		for (bChannel = 0; bChannel < MUSB_HSDMA_CHANNELS; bChannel++) {
			pImplChannel = (struct musb_dma_channel *)
						&(controller->aChannel[bChannel]);
			pChannel = &pImplChannel->Channel;
			if (pChannel->status == MUSB_DMA_STATUS_BUSY) {
				count = musb_readl(mbase, MUSB_HSDMA_CHANNEL_OFFSET
							(bChannel, MUSB_HSDMA_COUNT));

				if (count == 0)
					int_hsdma |= (1 << bChannel);
			}
		}

		DBG(2, "int_hsdma = 0x%x\n", int_hsdma);

		if (!int_hsdma)
			goto done;
	}

	for (bChannel = 0; bChannel < MUSB_HSDMA_CHANNELS; bChannel++) {
		if (int_hsdma & (1 << bChannel)) {
			pImplChannel = (struct musb_dma_channel *)
					&(controller->aChannel[bChannel]);
			pChannel = &pImplChannel->Channel;

			csr = musb_readw(mbase,
					MUSB_HSDMA_CHANNEL_OFFSET(bChannel,
							MUSB_HSDMA_CONTROL));

			if (csr & (1 << MUSB_HSDMA_BUSERROR_SHIFT))
				pImplChannel->Channel.status =
					MUSB_DMA_STATUS_BUS_ABORT;
			else {
				u8 devctl;

				dwAddress = musb_readl(mbase,
						MUSB_HSDMA_CHANNEL_OFFSET(
							bChannel,
							MUSB_HSDMA_ADDRESS));
				pChannel->actual_len = dwAddress
					- pImplChannel->dwStartAddress;

				DBG(2, "ch %p, 0x%x -> 0x%x (%d / %d) %s\n",
					pChannel, pImplChannel->dwStartAddress,
					dwAddress, pChannel->actual_len,
					pImplChannel->len,
					(pChannel->actual_len
						< pImplChannel->len) ?
					"=> reconfig 0": "=> complete");

				devctl = musb_readb(mbase, MUSB_DEVCTL);

				pChannel->status = MUSB_DMA_STATUS_FREE;

				/* completed */
				if ((devctl & MUSB_DEVCTL_HM)
					&& (pImplChannel->transmit)
					&& ((pChannel->desired_mode == 0)
					    || (pChannel->actual_len &
					    (pImplChannel->wMaxPacketSize - 1)))
					 ) {
					/* Send out the packet */
					musb_ep_select(mbase,
						pImplChannel->epnum);
					musb_writew(mbase, MUSB_EP_OFFSET(
							pImplChannel->epnum,
							MUSB_TXCSR),
						MUSB_TXCSR_TXPKTRDY);
				} else
					musb_dma_completion(
						musb,
						pImplChannel->epnum,
						pImplChannel->transmit);
			}
		}
	}
	retval = IRQ_HANDLED;
done:
	spin_unlock_irqrestore(&musb->lock, flags);
	return retval;
}

void dma_controller_destroy(struct dma_controller *c)
{
	struct musb_dma_controller *controller;

	controller = container_of(c, struct musb_dma_controller, Controller);
	if (!controller)
		return;

	if (controller->irq)
		free_irq(controller->irq, c);

	kfree(controller);
}

struct dma_controller *__init
dma_controller_create(struct musb *musb, void __iomem *pCoreBase)
{
	struct musb_dma_controller *controller;
	struct device *dev = musb->controller;
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 1);

	if (irq == 0) {
		dev_err(dev, "No DMA interrupt line!\n");
		return NULL;
	}

	controller = kzalloc(sizeof(struct musb_dma_controller), GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->bChannelCount = MUSB_HSDMA_CHANNELS;
	controller->pDmaPrivate = musb;
	controller->pCoreBase = pCoreBase;

	controller->Controller.start = dma_controller_start;
	controller->Controller.stop = dma_controller_stop;
	controller->Controller.channel_alloc = dma_channel_allocate;
	controller->Controller.channel_release = dma_channel_release;
	controller->Controller.channel_program = dma_channel_program;
	controller->Controller.channel_abort = dma_channel_abort;

	if (request_irq(irq, dma_controller_irq, IRQF_DISABLED,
			musb->controller->bus_id, &controller->Controller)) {
		dev_err(dev, "request_irq %d failed!\n", irq);
		dma_controller_destroy(&controller->Controller);
		return NULL;
	}

	controller->irq = irq;

#if defined(CONFIG_NET_POLL_CONTROLLER) && defined(CONFIG_KGDBOE_OVER_USB)
	pPrivateDataNetPoll = &controller->Controller;
#endif

	return &controller->Controller;
}
