/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pcan_fops.c - all file operation functions, exports only struct fops
 *
 * Copyright (C) 2001-2020 PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Stephane Grosjean <s.grosjean@peak-system.com>
 * Contributors: Klaus Hitschler <klaus.hitschler@gmx.de>
 *               Edouard Tisserant <edouard.tisserant@lolitech.fr> XENOMAI
 *               Laurent Bessard <laurent.bessard@lolitech.fr> XENOMAI
 *               Oliver Hartkopp <oliver.hartkopp@volkswagen.de> socket-CAN
 *               Marcel Offermans <marcel.offermans@luminis.nl>
 *               Arno <a.vdlaan@hccnet.nl>
 *               John Privitera <JohnPrivitera@dciautomation.com>
 */
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"	/* must always be the 1st include */

#include <linux/kernel.h>
#include <linux/slab.h>     // pcan_malloc()
#include <linux/fs.h>       // everything...
#include <linux/errno.h>    // error codes
#include <linux/types.h>    // size_t
#include <linux/proc_fs.h>  // proc
#include <linux/fcntl.h>    // O_ACCMODE
#include <linux/pci.h>      // all about pci
#include <linux/capability.h> // all about restrictions
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#include <asm/system.h>     // cli(), *_flags
#endif
#include <asm/uaccess.h>    // copy_...
#include <linux/delay.h>    // mdelay()
#include <linux/poll.h>     // poll() and select()

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
#include <linux/moduleparam.h>
#endif

#include "src/pcan_main.h"
#include "src/pcan_pci.h"
#include "src/pcan_isa.h"
#include "src/pcan_dongle.h"
#include "src/pcan_sja1000.h"
#include "src/pcan_fifo.h"
#include "src/pcan_fops.h"
#include "src/pcan_parse.h"
#include "src/pcan_usb.h"
#include "src/pcan_filter.h"

#include "src/pcanfd_core.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"
#endif

#ifdef DEBUG
#define DEBUG_RELEASE
#else
//#define DEBUG_RELEASE
#endif

#ifndef MODULE_LICENSE
#define MODULE_LICENSE(x)
#endif
#ifndef MODULE_VERSION
#define MODULE_VERSION(x)
#endif

#ifdef DEBUG
#define DEBUG_ALLOC_FIFOS
#else
//#define DEBUG_ALLOC_FIFOS
#endif

#define READ_MESSAGE_COUNT	500	/* max read message count */
#define WRITE_MESSAGE_COUNT	500	/* max write message count */

MODULE_AUTHOR("s.grosjean@peak-system.com");
MODULE_AUTHOR("klaus.hitschler@gmx.de");
MODULE_DESCRIPTION("Driver for PEAK-System CAN interfaces");
MODULE_VERSION(CURRENT_RELEASE);
MODULE_SUPPORTED_DEVICE("PCAN-ISA, PCAN-PC/104, PCAN-Dongle, PCAN-PCI(e), PCAN-ExpressCard, PCAN-PCCard, PCAN-USB (compilation dependent)");
MODULE_LICENSE("GPL");

#if defined(module_param_array) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
extern char *type[8];
extern ulong io[8];
extern char irq[8];
extern ushort btr0btr1;
extern char *bitrate;
extern char *dbitrate;

ushort rxqsize = READ_MESSAGE_COUNT;
ushort txqsize = WRITE_MESSAGE_COUNT;

module_param_array(type, charp, NULL, 0444);
module_param_array(io, ulong, NULL, 0444);
module_param_array(irq, byte,  NULL, 0444);
module_param(btr0btr1, ushort, 0444);
module_param(bitrate, charp, 0444);
module_param(dbitrate, charp, 0444);
module_param(rxqsize, ushort, 0444);
module_param(txqsize, ushort, 0444);
#else
MODULE_PARM(type, "0-8s");
MODULE_PARM(io, "0-8h");
MODULE_PARM(irq, "0-8b");
MODULE_PARM(btr0btr1, "h");
MODULE_PARM(bitrate, "s");
MODULE_PARM(dbitrate, "s");
MODULE_PARM(rxqsize, "h");
MODULE_PARM(txqsize, "h");
#endif

MODULE_PARM_DESC(type, "type of PCAN interface (isa, sp, epp)");
MODULE_PARM_DESC(io, "io-port address for either PCAN-ISA, PC/104 or Dongle");
MODULE_PARM_DESC(irq, "interrupt number for either PCAN-ISA, PC/104 or Dongle");
MODULE_PARM_DESC(btr0btr1, "initial bitrate (BTR0BTR1 format) for all channels");
MODULE_PARM_DESC(bitrate, "initial nominal bitrate for all channels");
MODULE_PARM_DESC(dbitrate, "initial data bitrate for all CAN-FD channels");

MODULE_PARM_DESC(rxqsize, " size of the Rx FIFO of a channel (def="
				__stringify(READ_MESSAGE_COUNT) ")");
MODULE_PARM_DESC(txqsize, " size of the Tx FIFO of a channel (def="
				__stringify(WRITE_MESSAGE_COUNT) ")");

#if defined(LINUX_24)
EXPORT_NO_SYMBOLS;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,18) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#define minor(x)	MINOR(x)
#endif

static struct pcandev *pcan_get_dev(struct pcan_udata *dev_priv)
{
	struct pcandev *dev;

	if (!dev_priv)
		return NULL;

	/* check whether this device is always linked. */
	dev = dev_priv->dev;
	if (!pcan_is_device_in_list(dev))
		return NULL;

	/* if the device is plugged out */
	if (!dev->is_plugged) {

#ifdef PCAN_USER_FREE_DEV
		/* if device was in global devices list, then it has
		 * been initialized, then it can be destroyed */
		if (pcan_remove_dev_from_list(dev))
			pcan_destroy_dev(dev);

		pcan_free_dev(dev);
#endif
		return NULL;
	}

	return dev;
}

/* find the pcandev according to given major,minor numbers
 * returns NULL pointer in the case of no success */
static struct pcandev* pcan_search_dev(int major, int minor)
{
	struct pcandev *dev = (struct pcandev *)NULL;
	struct list_head *ptr;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	pcan_lock_get_irqsave(&pcan_drv.devices_lock, flags);
#endif
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): major,minor=%d,%d\n",
		__func__, major, minor);
#endif

	if (list_empty(&pcan_drv.devices)) {
#ifdef HANDLE_HOTPLUG
		pcan_lock_put_irqrestore(&pcan_drv.devices_lock, flags);
#endif
#ifdef DEBUG
		pr_info(DEVICE_NAME ": no devices to select from!\n");
#endif
		return NULL;
	}

	/* loop through my devices */
	for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices;
							ptr = ptr->next) {
		dev = (struct pcandev *)ptr;

#ifndef XENOMAI3
		if (dev->nMajor == major)
#endif
			if (dev->nMinor == minor)
				break;
	}

#ifdef HANDLE_HOTPLUG
	pcan_lock_put_irqrestore(&pcan_drv.devices_lock, flags);
#endif
	if (ptr == &pcan_drv.devices) {
#ifdef DEBUG
		pr_info(DEVICE_NAME ": didn't find any pcan devices (%d,%d)\n",
			major, minor);
#endif
		return NULL;
	}

	return dev;
}

static long pcan_put_dev(struct pcandev *dev, long err)
{
	return err;
}

/* Indicate that a CAN frame is not a CAN 2.0 frame */
static int pcan_is_fd(struct pcanfd_msg *pf)
{
        return (pf->type == PCANFD_TYPE_CANFD_MSG);
}

/* Convert old-style TPCANMsg CAN 2.0 type into the new one
 *
 * WARNING: msg->LEN field MUST be <= 64
 */
static struct pcanfd_msg *pcan_msg_to_fd(struct pcanfd_msg *pf,
					 const TPCANMsg *msg)
{
	pf->type = PCANFD_TYPE_CAN20_MSG;	/* obviously */
	pf->id = msg->ID;
	pf->flags = msg->MSGTYPE;
	pf->data_len = msg->LEN;
	memcpy(pf->data, msg->DATA, msg->LEN);

	return pf;
}

/* Convert CAN 2.0 frame into old-style TPCANRdMsg type
 *
 * Warning: it's caller's responsibility to check whether pf->data_len is <= 8
 */
TPCANRdMsg *pcan_fd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf)
{
	switch (pf->type) {

	case PCANFD_TYPE_STATUS:
		msg->Msg.ID = pf->id;
		msg->Msg.MSGTYPE = MSGTYPE_STATUS;
		msg->Msg.LEN = 4;

		memset(msg->Msg.DATA, CAN_ERR_OK, msg->Msg.LEN);

		switch (pf->id) {
		case PCANFD_ERROR_WARNING:
			msg->Msg.DATA[3] |= CAN_ERR_BUSLIGHT;
			break;
		case PCANFD_ERROR_PASSIVE:
			msg->Msg.DATA[3] |= CAN_ERR_BUSHEAVY;
			break;
		case PCANFD_ERROR_BUSOFF:
			msg->Msg.DATA[3] |= CAN_ERR_BUSOFF;
			break;
		case PCANFD_RX_EMPTY:
			msg->Msg.DATA[3] |= CAN_ERR_QRCVEMPTY;
			break;
		case PCANFD_RX_OVERFLOW:
			msg->Msg.DATA[3] |= CAN_ERR_OVERRUN;
			break;
		case PCANFD_TX_OVERFLOW:
			msg->Msg.DATA[3] |= CAN_ERR_QXMTFULL;
			break;

		default:
		case PCANFD_TX_EMPTY:
			msg->Msg.DATA[3] |= CAN_ERR_RESOURCE;

		case PCANFD_ERROR_ACTIVE:
			break;
		}
		break;

	case PCANFD_TYPE_CAN20_MSG:
		msg->Msg.ID = pf->id;
		msg->Msg.MSGTYPE = (BYTE )(pf->flags & 0xff);
		msg->Msg.LEN = pf->data_len;

		/* Warning: pf->data_len MUST be <= 8 */
		memcpy(&msg->Msg.DATA, pf->data, pf->data_len);
		break;

	default:
		return NULL;
	}

	/* TODO: should check whether PCANFD_TIMESTAMP is always set */
	if (pf->flags & PCANFD_TIMESTAMP) {
		msg->dwTime = pf->timestamp.tv_sec * 1000;
		msg->dwTime += pf->timestamp.tv_usec / 1000;
		msg->wUsec = pf->timestamp.tv_usec % 1000;
	}

	return msg;
}

/* opens a data path with a pcan device.
 * This function is called by:
 * - pcan_open()
 * - pcan_open_rt()
 * - pcan_netdev_open()
 */
int pcan_open_path(struct pcandev *dev, struct pcan_udata *irq_arg)
{
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p): minor=%d, opened path=%d\n",
		__func__, dev, dev->nMinor, dev->nOpenPaths);
#endif
	/* only the 1st open to this device makes a default init  */
	if (dev->nOpenPaths > 0) {
		dev->nOpenPaths++;
		goto lbl_unlock_exit;
	}

	dev->wMsg = pcan_malloc(sizeof(dev->wMsg[0]) * txqsize, GFP_KERNEL);
	if (!dev->wMsg) {
		err = -ENOMEM;
		goto lbl_unlock_exit;
	}

	/* init Tx fifo even in NETDEV mode (writing is always possible) */
	pcan_fifo_init(&dev->writeFifo, dev->wMsg, dev->wMsg + txqsize - 1,
			txqsize, sizeof(dev->wMsg[0]));
#ifdef DEBUG_ALLOC_FIFOS
	pr_info(DEVICE_NAME ": %s CAN%u: %u items Tx FIFO allocated\n",
		dev->adapter->name, dev->can_idx+1, txqsize);
#endif

#ifdef NETDEV_SUPPORT
	/* in NETDEV, Rx FIFO is useless, since events are routed towards the
	 * socket buffer */
#else
	dev->rMsg = pcan_malloc(sizeof(dev->rMsg[0]) * rxqsize, GFP_KERNEL);
	if (!dev->rMsg) {
		err = -ENOMEM;
		goto lbl_unlock_free_w;
	}

	/* init Rx fifos */
	pcan_fifo_init(&dev->readFifo, dev->rMsg, dev->rMsg + rxqsize - 1,
			rxqsize, sizeof(dev->rMsg[0]));
#ifdef DEBUG_ALLOC_FIFOS
	pr_info(DEVICE_NAME ": %s CAN%u: %u items Rx FIFO allocated\n",
		dev->adapter->name, dev->can_idx+1, rxqsize);
#endif
#endif

	/* open the interface special parts */
	if (dev->open) {
		err = dev->open(dev);
		if (err) {
			pr_err("%s: can't open interface specific!\n",
				DEVICE_NAME);
			goto lbl_unlock_free_all;
		}
	}

	/* special handling: probe here only for dongle devices,
	 * because connect after init is possible */
	switch (dev->wType) {

	case HW_DONGLE_SJA:
	case HW_DONGLE_SJA_EPP:

		/* no usb here, generic sja1000 call for dongle */
		err = sja1000_probe(dev);
		if (err) {
			pr_err("%s: %s-dongle minor %d (io=0x%04x, irq=%d) "
				"not found (err %d)\n",
				DEVICE_NAME, dev->type, dev->nMinor,
				dev->dwPort, dev->wIrq, err);
			dev->release(dev);
			goto lbl_unlock_free_all;
		}
		break;
	}

	/* initialize here the sync mechanism between ISR and fifo */
#ifndef NETDEV_SUPPORT
	pcan_event_init(&dev->in_event, 0);
#endif
	pcan_event_init(&dev->out_event, 1);

	/* install irq */
	if (dev->req_irq) {
		err = dev->req_irq(dev, irq_arg);
		if (err) {
			pr_err("%s: can't request irq from device (err %d)\n",
				DEVICE_NAME, err);
			goto lbl_unlock_free_all;
		}
	}

	dev->opened_index = dev->adapter->opened_count++;

	/* inc nOpenPath BEFORE calling _open() because some devices (USB for
	 * ex) may start sending notifications (interrupt based) before
	 * returning from the function. */
	dev->nOpenPaths = 1;

	pcanfd_dev_reset(dev);

	//pcan_mutex_unlock(&dev->mutex);

	/* open the device itself */
	err = pcanfd_dev_open(dev, &dev->init_settings);

	//pcan_mutex_lock(&dev->mutex);

	if (!err) {
		return 0;
	}

	dev->nOpenPaths--;
	dev->adapter->opened_count--;

	if (dev->free_irq)
		dev->free_irq(dev, irq_arg);

	pr_err(DEVICE_NAME ": can't open device hardware itself (err %d)!\n",
		err);

lbl_unlock_free_all:
#ifndef NETDEV_SUPPORT
	dev->rMsg = pcan_free(dev->rMsg);
lbl_unlock_free_w:
#endif
	dev->wMsg = pcan_free(dev->wMsg);
lbl_unlock_exit:

	return err;
}

static inline int pcan_tx_fifo_empty(struct pcandev *dev)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": closing %s CAN%u: "
		"tx_fifo=%u is_plugged=%u bus_state=%d engine_state=%u\n",
		dev->adapter->name, dev->can_idx+1,
		dev->writeFifo.nStored, dev->is_plugged,
		dev->bus_state,
		dev->locked_tx_engine_state
		);
#endif
	return !dev->is_plugged ||
		dev->bus_state == PCANFD_ERROR_BUSOFF ||
		pcan_fifo_empty(&dev->writeFifo);
}

/* is called by pcan_release() and pcan_netdev_close() */
void pcan_release_path(struct pcandev *dev, struct pcan_udata *irq_arg)
{
	pcan_lock_irqsave_ctxt flags;
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_RELEASE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u): minor=%d, path=%d\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		dev->nMinor, dev->nOpenPaths);
#endif
	switch (dev->nOpenPaths) {

	case 1:

		/* mutex not needed: we're the only single one! */
		//pcan_mutex_unlock(&dev->mutex);

		dev->flags |= PCAN_DEV_CLOSING;

		/* Note: always true in non-RT */
		if (pcan_task_can_wait()) {
#ifdef DEBUG_RELEASE
			pr_info(DEVICE_NAME ": %s CAN%u preparing to wait: "
				"is_plugged=%u tx_fifo=%u "
				"tx_engine_state=%d to=%d\n",
				dev->adapter->name, dev->can_idx+1,
				dev->is_plugged,
				dev->writeFifo.nStored,
				dev->locked_tx_engine_state,
				MAX_WAIT_UNTIL_CLOSE);
#endif

			/* Now, wait for the tx fifo to be empty */
			err = pcan_event_wait_timeout(dev->out_event,
					pcan_tx_fifo_empty(dev),
					MAX_WAIT_UNTIL_CLOSE);

#ifdef DEBUG_RELEASE
			pr_info(DEVICE_NAME ": %s CAN%u finished waiting: "
				"is_plugged=%u tx_fifo=%u "
				"tx_engine_state=%d err=%d\n",
				dev->adapter->name, dev->can_idx+1,
				dev->is_plugged,
				dev->writeFifo.nStored,
				dev->locked_tx_engine_state,
				err);
#endif
#ifndef NO_RT
		} else if (!pcan_tx_fifo_empty(dev)) {

			/* Note: this else part only makes sense in RT:
			 * simply wait a bit for the tx fifo to empty */
			pcan_msleep_interruptible(MAX_WAIT_UNTIL_CLOSE);
#endif
		}

		/* tell the world that the device is being closed now */
		dev->nOpenPaths--;

		//pcan_lock_get_irqsave(&dev->isr_lock, flags);
		dev->lock_irq(dev, &flags);
		pcan_set_tx_engine(dev, TX_ENGINE_CLOSED);

		//pcan_lock_put_irqrestore(&dev->isr_lock, flags);
		dev->unlock_irq(dev, &flags);
		/* release the device itself */
		if (dev->device_release)
			dev->device_release(dev);

		dev->flags &= ~(PCAN_DEV_OPENED|PCAN_DEV_CLOSING);
		pcan_set_bus_state(dev, PCANFD_UNKNOWN);

		if (dev->release)
			dev->release(dev);

		/* release the device irq */
		if (dev->free_irq)
			dev->free_irq(dev, irq_arg);

		/* mutex not needed: we're the only single one! */
		//pcan_mutex_lock(&dev->mutex);

		if (dev->adapter->opened_count > 0)
			dev->adapter->opened_count--;

#if 1
		/* don't restore default init settings, otherwise
		 * "echo "i 0x0014" > /dev/pcanX" is inoperative */
#else
		/* restore default settings */
		pcanfd_copy_init(&dev->init_settings, &dev->def_init_settings);
#endif

		/* destroy useless syncs (initialized at open()) */
		pcan_event_free(&dev->out_event);

#ifndef NETDEV_SUPPORT
		pcan_event_free(&dev->in_event);
#endif

		/* fall through */
	case 0:
		/* case 0 does exist when the device has been unplugged 
		 * but not opened */
	
		/* destroy useless Rx/Tx fifos */
		dev->wMsg = pcan_free(dev->wMsg);
#ifdef DEBUG_ALLOC_FIFOS
		pr_info(DEVICE_NAME ": %s CAN%u Tx FIFO released\n",
			dev->adapter->name, dev->can_idx+1);
#endif

#ifndef NETDEV_SUPPORT
		dev->rMsg = pcan_free(dev->rMsg);
#ifdef DEBUG_ALLOC_FIFOS
		pr_info(DEVICE_NAME ": %s CAN%u Rx FIFO released\n",
			dev->adapter->name, dev->can_idx+1);
#endif
#endif

		/* fall through */
	default:
		if (dev->nOpenPaths >= 0)
			dev->nOpenPaths--;
	}
}

/* is called at user ioctl() with cmd = PCAN_GET_STATUS */
int pcan_ioctl_status_common(struct pcandev *dev, TPSTATUS *local)
{
	local->wErrorFlag = dev->wCANStatus;

	/* get infos for friends of polling operation */
#ifndef NETDEV_SUPPORT
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;
#endif
	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->nLastError = dev->nLastError;

	return 0;
}

/* is called at user ioctl() with cmd = PCAN_GET_EXT_STATUS */
int pcan_ioctl_extended_status_common(struct pcandev *dev,
					TPEXTENDEDSTATUS *local)
{
	local->wErrorFlag = dev->wCANStatus;

#ifndef NETDEV_SUPPORT
	local->nPendingReads = dev->readFifo.nStored;

	/* get infos for friends of polling operation */
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;
#else
	local->nPendingReads = 0;
#endif
	local->nPendingWrites = dev->writeFifo.nStored;

	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->nLastError = dev->nLastError;

	return 0;
}

/* is called at user ioctl() with cmd = PCAN_DIAG */
int pcan_ioctl_diag_common(struct pcandev *dev, TPDIAG *local)
{
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (dev->netdev) ?
		pcan_netdev_get_stats(dev->netdev) : NULL;
#endif

	local->wType = dev->wType;

	switch (dev->wType) {
	case HW_USB:
	case HW_USB_FD:
	case HW_USB_PRO:
	case HW_USB_PRO_FD:
	case HW_USB_X6:
#ifdef USB_SUPPORT 
		local->dwBase = pcan_usb_get_if(dev)->dwSerialNumber;
		local->wIrqLevel = dev->port.usb.ucHardcodedDevNr;
#endif
		break;
	default:
		local->dwBase = dev->dwPort;
		local->wIrqLevel = dev->wIrq;
		break;
	}

#ifdef NETDEV_SUPPORT
	local->dwReadCounter = (stats) ? stats->rx_packets : 0;
#else
	local->dwReadCounter = dev->readFifo.dwTotal;

	/* get infos for friends of polling operation */
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;
#endif

	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->dwWriteCounter = dev->writeFifo.dwTotal;
	local->dwIRQcounter = dev->dwInterruptCounter;
	local->dwErrorCounter = dev->dwErrorCounter;
	local->wErrorFlag = dev->wCANStatus;

	local->nLastError = dev->nLastError;
	local->nOpenPaths = dev->nOpenPaths;

	strncpy(local->szVersionString,
			pcan_drv.szVersionString, VERSIONSTRING_LEN);
	return 0;
}

static int handle_pcanfd_send_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_msgs_0 __user *plu = (struct pcanfd_msgs_0 *)up;
	struct pcanfd_txmsgs txs, *pl;
	int i, l, err;

	l = sizeof(*plu);
	err = pcan_copy_from_user(&txs, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* ok. Nothing to send. So nothing done. Perfect. */
	if (!txs.count)
		return 0;

	l += txs.count * sizeof(txs.list[0]);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": %s(): failed to alloc msgs list\n",
			__func__);
		return -ENOMEM;
	}

	/* copy count of items, then items */
	for (i = 0; i < txs.count; i++) {
		err = copy_from_user(&pl->list[i].msg, &plu->list[i],
				     sizeof(pl->list[i].msg));
		if (err) {
			pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
				__func__);
			err = -EFAULT;
			goto lbl_free;
		}
#if 0
		pr_info(DEVICE_NAME ": id=%x len=%u\n",
			pl->list[i].msg.id,
			pl->list[i].msg.data_len);
			
		dump_mem("data", pl->list[i].msg.data,
			 pl->list[i].msg.data_len);
#endif
	}

	pl->count = i;
	err = pcanfd_ioctl_send_msgs(dev, pl, dev_priv);

	/* copy the count of msgs really sent (= pl->count) */
	if (pcan_copy_to_user(plu, pl, sizeof(*plu), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

lbl_free:
	pcan_free(pl);

	return err;
}

static int handle_pcanfd_recv_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_msgs_0 __user *plu = (struct pcanfd_msgs_0 *)up;
	struct pcanfd_rxmsgs rxs, *pl;
	int i, l, err;

	l = sizeof(*plu);
	err = pcan_copy_from_user(&rxs, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* ok! no room for saving rcvd msgs!? Thus, nothing returned */
	if (!rxs.count)
		return 0;

	l += rxs.count * sizeof(rxs.list[0]);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": failed to alloc msgs list\n");
		return -ENOMEM;
	}

	pl->count = rxs.count;
	err = pcanfd_ioctl_recv_msgs(dev, pl, dev_priv);

	/* copy the count then the msgs received */
	if (pcan_copy_to_user(plu, pl, sizeof(*plu), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
		goto lbl_free;
	}

	for (i = 0; i < pl->count; i++) {
		err = pcan_copy_to_user(&plu->list[i], &pl->list[i],
					sizeof(plu->list[i]), c);
		if (err) {
			pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
				__func__);
			err = -EFAULT;
			break;

		}
	}

lbl_free:
	pcan_free(pl);

	return err;
}

static int handle_pcanfd_get_av_clocks(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					void *c)
{
	struct pcanfd_available_clocks avclks;
	int l = sizeof(struct pcanfd_available_clocks_0);
	const void *kp;
	int err;

	err = pcan_copy_from_user(&avclks, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* copy only the count of clocks of the device */
	if (avclks.count >= dev->clocks_list->count) {
		kp = dev->clocks_list;
		l += dev->clocks_list->count *
				sizeof(struct pcanfd_available_clock);

	/* copy only the count of clocks requested by user */
	} else {
		up += l;
		kp = &dev->clocks_list->list;
		l += avclks.count *
				sizeof(struct pcanfd_available_clock);
	}

	if (pcan_copy_to_user(up, kp, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_bittiming_ranges(struct pcandev *dev,
						void __user *up,
						struct pcan_udata *dev_priv,
						void *c)
{
	struct __array_of_struct(pcanfd_bittiming_range, 2) fdbtr;
	int l = sizeof(struct pcanfd_bittiming_ranges_0);
	//int l = sizeof(fdbtr.count);
	int err = pcan_copy_from_user(&fdbtr, up, l, c);
	u32 user_count;

	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* keep in memory the max given by user */
	user_count = fdbtr.count;

	/* CAN-FD: max of 2 bittiming ranges */
	memset(&fdbtr, '\0', sizeof(fdbtr));

	if (fdbtr.count < user_count) {
		fdbtr.list[fdbtr.count++] = *dev->bittiming_caps;

		if (dev->dbittiming_caps)
			if (fdbtr.count < user_count)
				fdbtr.list[fdbtr.count++] =
						*dev->dbittiming_caps;
	}

	/* copy the count of bittiming ranges read from the device */
	l += fdbtr.count * sizeof(struct pcanfd_bittiming_range);
	if (pcan_copy_to_user(up, &fdbtr, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_option opt;
	const int l = sizeof(opt);

	int err = pcan_copy_from_user(&opt, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (opt.name >= PCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt.name);
		return -EINVAL;
	}

	if (!dev->option[opt.name].get) {
		return -EOPNOTSUPP;
	}

	if (dev->option[opt.name].req_size > 0)

		/* if user option buffer size is too small, return the 
		 * requested size with -ENOSPC */
		if (opt.size < dev->option[opt.name].req_size) {
			pr_warn(DEVICE_NAME
				": invalid option size %d < %d for option %d\n",
				opt.size, dev->option[opt.name].req_size,
				opt.name);
			opt.size = dev->option[opt.name].req_size;
			err = -ENOSPC;
			goto lbl_cpy_size;
		}

	err = dev->option[opt.name].get(dev, &opt, c);
	if (err && err != -ENOSPC)
		return err;

lbl_cpy_size:
	/* update 'size' field */
	if (pcan_copy_to_user(up+offsetof(struct pcanfd_option, size),
			 &opt.size, sizeof(opt.size), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_set_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_option opt;
	int l = sizeof(opt);

	int err = pcan_copy_from_user(&opt, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (opt.name >= PCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt.name);
		return -EINVAL;
	}

	if (!dev->option[opt.name].set) {
		return -EOPNOTSUPP;
	}

	if (dev->option[opt.name].req_size > 0)

		/* if user option buffer size is too small, return the 
		 * requested size with -ENOSPC */
		if (opt.size < dev->option[opt.name].req_size) {
			pr_warn(DEVICE_NAME
				": invalid option size %d < %d for option %d\n",
				opt.size, dev->option[opt.name].req_size,
				opt.name);
			opt.size = dev->option[opt.name].req_size;
			err = -ENOSPC;
			goto lbl_cpy_size;
		}

	return dev->option[opt.name].set(dev, &opt, c);

lbl_cpy_size:
	/* update 'size' field */
	if (pcan_copy_to_user(up+offsetof(struct pcanfd_option, size),
			 &opt.size, sizeof(opt.size), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

/*
 * Inculde system specific entry points:
 */
#ifdef NO_RT
#include "pcan_fops_linux.c"
#else
#include "pcan_fops_rt.c"
#endif
