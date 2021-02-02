/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pcan_usbfd.c - the inner parts for PCAN-USB (Pro) FD support
 *
 * Copyright (C) 2014-2020 PEAK System-Technik GmbH <www.peak-system.com>
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
 * Author:       Stephane Grosjean <s.grosjean@peak-system.com>
 */
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"

#ifdef USB_SUPPORT

#include "src/pcan_fifo.h"
#include "src/pcanfd_usb.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"		/* for hotplug pcan_netdev_register() */
#else
#include <linux/can/dev.h>
#endif

#include "src/pcan_timing.h"		/* timing conversion */
#include "src/pcanfd_usb_fw.h"		/* PCAN-USB (Pro) FD fw structures */
#include "src/pcan_usbpro_fw.h"

/* if defined, calibration msgs are started when 1st CAN is opened,
 * and stopped when last CAN is closed. But it looks like it's not the best
 * way to do and this generates problems with timestamps and PCAN-Chip.
 * This is how pcan worked like, until 8.6.
 * If not defined, (since pcan 8.6+) then CM are started when the device is
 * plugged in...
 */
//#define UCAN_USB_START_CM_AT_OPEN

/* if defined, bus_load event timestamp is cooked: since BUS_LOAD event is
 * now handled by the driver itself which doesn't forward teh device bus load
 * notifications "as is", cooking the ebus load timestamp is useless.
 */
//#define PCAN_DECODE_BUS_LOAD_TS

/* device state flags */
#define UCAN_USB_SHOULD_WAKEUP		0x00000001UL

/*
 * Private Data Structures
 */
struct pcan_usbfd_fw_info {
	u16	size_of;	/* sizeof this */
	u16	type;		/* type of this structure */
	u8	hw_type;	/* Type of hardware (HW_TYPE_xxx) */
	u8	bl_version[3];	/* Bootloader version */
	u8	hw_version;	/* Hardware version (PCB) */
	u8	fw_version[3];	/* Firmware version */
	__le32	dev_id[2];	/* "device id" per CAN */
	__le32	ser_no;		/* S/N */
	u32	flags;		/* special functions */
} __attribute__ ((packed));

#ifdef UCAN_USB_OPTION_FAST_FWD
/* enable to globally set the fast-forward option for PCAN-USB FD adapters */
static ushort fast_fwd = 0;
module_param(fast_fwd, ushort, 0444);
#endif

static int pcan_chip_devices = 0;
static int pcan_usbfd_devices = 0;
static int pcan_usbprofd_devices = 0;
static int pcan_usbx6_devices = 0;

static const char *str_pcan_usb_x6 = "PCAN-USB X6";

#define PCAN_UFD_CMD_EP			0x01
#define PCAN_UFD_LOSPD_PKT_SIZE		64

#ifdef DEBUG_TRACE
static struct pcan_timeval *pcan_usbfd_decode_timestamp(struct pcandev *dev,
						   u32 ts_low, u32 ts_high,
						   struct pcan_timeval *tv)
{
	pr_info(DEVICE_NAME ": %s(%s CAN%u, ts=%u)\n",
		__func__, dev->adapter->name, dev->can_idx+1, ts_low);

	return pcan_usbpro_timestamp_decode(dev, ts_low, tv);
}
#else
#define pcan_usbfd_decode_timestamp(a, b, c, d)	\
				pcan_usbpro_timestamp_decode(a, b, d)
#endif

/* static int pcan_usbfd_send_ucan_cmd(struct pcandev *dev)
 */
static int pcan_usbfd_send_ucan_cmd(struct pcandev *dev)
{
	int i = 0, err = 0, packet_len;
	void *packet_ptr;
#ifdef PCAN_USB_CMD_PER_DEV
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u): ->EP#%02X\n",
		__func__, dev->can_idx+1,
		usb_if->pipe_cmd_out.ucNumber);
#endif
	/* don't do anything with non-existent hardware */
	if (!dev->is_plugged)
		return -ENODEV;

	/* if a packet is not filled completely by commands, the command list
	 * is terminated with an "end of collection" record. */
	if (dev->ucan.cmd_len < dev->ucan.cmd_size) {
#if 1
		ucan_add_cmd(dev, UCAN_CMD_END_OF_COLLECTION);
#else
		void *cmd = ucan_add_cmd(dev, UCAN_CMD_END_OF_COLLECTION);
		memset(cmd, 0xff, sizeof(u64));
#endif
	}

#ifdef DEBUG
	//dump_mem("sent cmd", dev->ucan.cmd_head, 32); //dev->ucan.cmd_len);
	dump_mem("sent cmd", dev->ucan.cmd_head, dev->ucan.cmd_len);

	/* firmware is not able to re-assemble 512 bytes buffer in full-speed */
	if ((usb_if->usb_dev->speed != USB_SPEED_HIGH) &&
				(dev->ucan.cmd_len > PCAN_UFD_LOSPD_PKT_SIZE)) {
		pr_warn(DEVICE_NAME
			": large cmd (%dB) is cut for non full-speed USB\n",
			dev->ucan.cmd_len);
	}
#endif

	packet_ptr = dev->ucan.cmd_head;
	packet_len = dev->ucan.cmd_len;

	/* firmware is not able to re-assemble 512 bytes buffer in full-speed */
	if (unlikely((usb_if->usb_dev->speed != USB_SPEED_HIGH) &&
			(packet_len > PCAN_UFD_LOSPD_PKT_SIZE))) {
		packet_len = PCAN_UFD_LOSPD_PKT_SIZE;

#ifdef DEBUG
		pr_warn(DEVICE_NAME
			": packet_len reset to %u bytes due to non-high speed "
			"controler\n", PCAN_UFD_LOSPD_PKT_SIZE);
#endif
	}

	do {
		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_sndbulkpipe(usb_if->usb_dev,
					   usb_if->pipe_cmd_out.ucNumber),
				   packet_ptr, packet_len, NULL,
				   PCAN_USB_CMD_TIMEOUT);
		if (err) {
			pr_err(DEVICE_NAME
			       ": err %d submitting cmd %03xh to %s (%d/3)\n",
			       err, le16_to_cpu(*(__le16 *)dev->ucan.cmd_head),
			       dev->adapter->name, ++i);

			if (err == -ETIMEDOUT && i < 3)
				continue;
			break;
		}

		packet_ptr += packet_len;
		dev->ucan.cmd_len -= packet_len;

		if (dev->ucan.cmd_len < PCAN_UFD_LOSPD_PKT_SIZE)
			packet_len = dev->ucan.cmd_len;

	} while (packet_len > 0);

#else
	/* TODO */
#endif
	return err;
}

/*
 * Hardware Callbacks
 */

/* int ucan_usb_set_can_led(struct pcandev *dev, u8 mode )
 */
static int ucan_usb_set_can_led(struct pcandev *dev, u8 mode)
{
	struct ucan_usb_led *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_LED_SET);
	if (cmd)
		cmd->mode = mode;

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_set_en_option(struct pcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_set_en_option(struct pcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	cmd = ucan_add_cmd_set_en_option(ucan_init_cmd(dev), mask);
	if (cmd)
		cmd->usb_mask = cpu_to_le16(usb_mask);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_clr_dis_option(struct pcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_clr_dis_option(struct pcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	cmd = ucan_add_cmd_clr_dis_option(ucan_init_cmd(dev), mask);
	if (cmd)
		cmd->usb_mask = cpu_to_le16(usb_mask);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int pcan_usbfd_get_fw_info(struct pcandev *dev, int should_print,
 *				u32 *bl_version)
 */
static int pcan_usbfd_get_fw_info(struct pcan_usb_interface *usb_if,
				  int should_print, u32 *bl_version)
{
	struct pcandev *dev0 = usb_if_dev(usb_if, 0);
	struct pcan_usbfd_fw_info * const pfi = \
			(struct pcan_usbfd_fw_info *)dev0->ucan.cmd_head;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	err = pcan_usbpro_request(usb_if,
				USB_VENDOR_REQUEST_INFO,
				USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
				pfi, sizeof(*pfi));
	if (err >= 0) {

		int i;

		if (should_print)
			dev_info(&usb_if->usb_intf->dev,
				"%s (%02xh PCB%02Xh) "
				"fw v%d.%d.%d "
				"bl v%d.%d.%d\n",
				usb_if->adapter->name,
				pfi->hw_type, pfi->hw_version,
				pfi->fw_version[0], pfi->fw_version[1],
				pfi->fw_version[2],
				pfi->bl_version[0], pfi->bl_version[1],
				pfi->bl_version[2]);

		/* save device id and serial num info for further read */
		usb_if->dwSerialNumber = le32_to_cpu(pfi->ser_no);
		usb_if->ucRevision = pfi->hw_version;
		usb_if->hw_ver.major = pfi->fw_version[0];
		usb_if->hw_ver.minor = pfi->fw_version[1];
		usb_if->hw_ver.subminor = pfi->fw_version[2];

		for (i = 0; i < usb_if->can_count; i++) {
			struct pcandev *dev = usb_if_dev(usb_if, i);
			dev->port.usb.ucHardcodedDevNr =
				le32_to_cpu(pfi->dev_id[i]);
		}

		if (bl_version)
			*bl_version = VER_NUM(pfi->bl_version[0],
					      pfi->bl_version[1],
					      pfi->bl_version[2]);
		return 0;
	}

	return err;
}

/*
 * int pcan_usbfd_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
 */
static int pcan_usbfd_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usbfd_fw_info * const pfi = \
				(struct pcan_usbfd_fw_info *)dev->ucan.cmd_head;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	err = pcan_usbpro_request(usb_if,
				USB_VENDOR_REQUEST_INFO,
				USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
				pfi, sizeof(*pfi));
	if (!err) {

		u32 tmp32 = le32_to_cpu(pfi->dev_id[dev->can_idx]);

		dev->flags &= ~PCAN_DEV_USES_ALT_NUM;
		if (tmp32 != 0xffffffff)
			dev->flags |= PCAN_DEV_USES_ALT_NUM;

		if (p_device_nr)
			*p_device_nr = tmp32;
	}

	return err;
}

/*
 * int pcan_usbfd_set_device_nr(struct pcandev *dev, u32 device_nr)
 */
static int pcan_usbfd_set_device_nr(struct pcandev *dev, u32 device_nr)
{
	struct ucan_usb_device_id *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, 0x%x)\n",
		__func__, dev->can_idx+1, device_nr);
#endif

	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_DEVID_SET);
	if (cmd)
		cmd->device_id = cpu_to_le32(device_nr);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/*
 * int pcan_usbfd_get_serial_nr(struct pcan_usb_interface *usb_if,
 *					u32 *p_serial_num)
 *
 * Retrieve serial number from bootloader info
 */
static int pcan_usbfd_get_serial_nr(struct pcan_usb_interface *usb_if,
					u32 *p_serial_num)
{
#if 0
	int err;

	err = pcan_usbfd_get_fw_info(usb_if, 0, NULL);
	if (!err) {
		if (p_serial_num)
			*p_serial_num = usb_if->dwSerialNumber;
	}

	return err;
#else
	/* used cached value read from pcan_usbfd_init() */
	if (p_serial_num)
		*p_serial_num = usb_if->dwSerialNumber;

	return 0;
#endif
}

/* int ucan_usb_set_clck_domain(struct pcandev *dev, u32 clk_mode)
 */
static int ucan_usb_set_clck_domain(struct pcandev *dev, u32 clk_mode)
{
	struct ucan_usb_clock *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, 0x%x)\n",
		__func__, dev->can_idx+1, clk_mode);
#endif
	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_CLK_SET);
	if (cmd)
		cmd->mode = (u8 )clk_mode;

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/*
 * PCAN-Chip FW v3.3.0 specific:
 */

#if 0
struct api_context {
	struct completion	done;
	int			status;
};

static void pcanfd_usb_completion(struct urb *urb)
{
	struct api_context *ctx = urb->context;

	ctx->status = urb->status;
	complete(&ctx->done);
}
#endif
static int pcan_usb_chip_get_io(struct pcandev *dev,
				struct pcanfd_option *opt, void *c,
				u16 op_req)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct ucan_usb_io_ctrl *cmd = ucan_add_cmd(ucan_init_cmd(dev), op_req);

	int i, l, err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, op_req=%04xh)\n",
		__func__, dev->can_idx+1, op_req);
#endif
	err = pcan_usbfd_send_ucan_cmd(dev);
	if (err)
		return err;

	for (i = 0; i < 3; i++) {

		u16 opcode;
		u32 tmp32;

		/* wait for response */
		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_in.ucNumber),
				   dev->ucan.cmd_head, dev->ucan.cmd_size, &l,
				   PCAN_USB_CMD_TIMEOUT);
		if (err < 0) {
			pr_err(DEVICE_NAME
				": %s(): usb_bulk_msg() err %d\n",
				__func__, err);
			break;
		}

		if (!l) {
			pr_warn(DEVICE_NAME ": abnormal null response (err=%d)."
				" Retrying...\n",
				err);
			continue;
		}

		/* dummy read to complete the above read */
		usb_bulk_msg(usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
					PCAN_UFD_CMD_EP),
				dev->ucan.cmd_head, dev->ucan.cmd_size, &l,
				PCAN_USB_CMD_TIMEOUT);
#ifdef DEBUG
		dump_mem("rcvd rsp", cmd, l);
#endif

		/* check if it's the good one */
		opcode = le16_to_cpu(cmd->opcode);
		if (opcode != (op_req+1)) {
			pr_warn(DEVICE_NAME
				": %s(): got abnormal rsp 0x%x to req 0x%x\n",
				__func__, opcode, op_req);
			continue;
		}

		opt->size = sizeof(u32);
		tmp32 = le32_to_cpu(cmd->io_val);
		if (pcan_copy_to_user(opt->value, &tmp32, opt->size, c)) {
			pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
				__func__);
			err = -EFAULT;
		}

		break;
	}

	return err;
}

static int pcan_usb_chip_set_io(struct pcandev *dev,
				struct pcanfd_option *opt, void *c,
				u16 op_req)
{
	struct ucan_usb_io_ctrl *cmd;
	u32 io_cfg;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, op_req=%04xh)\n",
		__func__, dev->can_idx+1, op_req);
#endif
	err = pcan_copy_from_user(&io_cfg, opt->value, sizeof(u32), c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	cmd = ucan_add_cmd(ucan_init_cmd(dev), op_req);
	if (!cmd)
		return -ENOMEM;

	cmd->io_val = cpu_to_le32(io_cfg);
	return pcan_usbfd_send_ucan_cmd(dev);
}

int pcan_usb_chip_get_dig_cfg(struct pcandev *dev,
			      struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_DPIN_CFG_REQ);
}

int pcan_usb_chip_set_dig_cfg(struct pcandev *dev,
			      struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_CFG_SET);
}

int pcan_usb_chip_get_dig_val(struct pcandev *dev,
			      struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_DPIN_VAL_REQ);
}

int pcan_usb_chip_set_dig_val(struct pcandev *dev,
				     struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_VAL_SET);
}

int pcan_usb_chip_set_dig_bit(struct pcandev *dev,
				     struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_SET_HIGH);
}

int pcan_usb_chip_clr_dig_bit(struct pcandev *dev,
				     struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_SET_LOW);
}

int pcan_usb_chip_get_ana_val(struct pcandev *dev,
				     struct pcanfd_option *opt, void *c)
{
	return pcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_ANAL_VAL_REQ);
}

/*
 * static int pcan_usbfd_set_clk_domain(struct pcandev *dev,
 *						struct pcanfd_init *pfdi)
 */
static int pcan_usbfd_set_clk_domain(struct pcandev *dev,
						struct pcanfd_init *pfdi)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	/* select the clock for the CAN */
	switch (pfdi->clock_Hz) {
	case 20000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_20MHZ);
		break;
	case 24000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_24MHZ);
		break;
	case 30000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_30MHZ);
		break;
	case 40000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_40MHZ);
		break;
	case 60000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_60MHZ);
		break;
	case 80000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_80MHZ);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int pcan_usbfd_open_complete(struct pcan_usb_interface *usb_if,
				    struct pcandev *dev,
				    struct pcanfd_init *pfdi,
				    int err)
{
	u16 opt_mask = UCAN_OPTION_ERROR, usb_mask = 0;

	if (err) {
		pr_err(DEVICE_NAME ": failed to open %s CAN%u: err %d\n",
			dev->adapter->name, dev->can_idx+1, err);
		goto fail;
	}

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): opened_count=%d\n",
		__func__, usb_if->opened_count);
#endif

#ifdef UCAN_USB_START_CM_AT_OPEN
	/* Note: if 'opened_count == 0' then this is the first device
	 * opened, ask for being notified of calibration msgs.
	 * We MUST fo this test because setting (again) this bit when opening
	 * another CAN channel of the same device will reset the timestamp
	 * of the next CM received!
	 * If this bit should be set eventhough, then all .time_sync members
	 * fo all channels of the same usb_if MUST be reset too. */
	if (usb_if->opened_count < 1)
		usb_mask |= UCAN_USB_OPTION_CALIBRATION;
#endif
#ifdef UCAN_USB_OPTION_FAST_FWD
	/* setup fast-forward option */
	if (fast_fwd) {
		pr_info("%s: fast-forward option set for %s CAN%u channel\n",
				DEVICE_NAME, dev->adapter->name,
				dev->can_idx+1);
		usb_mask |= UCAN_USB_OPTION_FAST_FWD;
	} else {
		/* be sure to clear any fast-forward option */
		err = ucan_usb_clr_dis_option(dev, 0, UCAN_USB_OPTION_FAST_FWD);
		if (err)
			goto fail;
	}
#endif

	if (pfdi) {
		if (pfdi->flags & PCANFD_INIT_BUS_LOAD_INFO)
			opt_mask |= UCAN_OPTION_BUSLOAD;
	}

	if (!(opt_mask & UCAN_OPTION_BUSLOAD)) {
		err = ucan_usb_clr_dis_option(dev, UCAN_OPTION_BUSLOAD, 0);
		if (err)
			goto fail;
	}

	/* ask for being notified of error messages */
	err = ucan_usb_set_en_option(dev, opt_mask, usb_mask);

fail:
	return err;
}

/* int pcan_usbfd_open_fd(struct pcandev *dev, struct pcanfd_init *pfdi)
 */
static int pcan_usbfd_open_fd(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_IRQ)
	pr_info(DEVICE_NAME ": %s(CAN%u, clk=%u Hz)\n",
		__func__, dev->can_idx+1, pfdi->clock_Hz);
#endif

	err = ucan_device_open_fd(dev, pfdi);

	return pcan_usbfd_open_complete(usb_if, dev, pfdi, err);
}

/* int pcan_usbfd_close(struct pcandev *dev)
 */
static int pcan_usbfd_close(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	u16 usb_mask = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u): opened_count=%d\n",
		__func__, dev->can_idx+1, usb_if->opened_count);
#endif

	/* Note: when 'opened_count == 1' then this is the last device
	 * opened. We can clear anything regarding the whole interface
	 */
	if (usb_if->opened_count <= 1) {
		struct pcandev *dev0 = usb_if_dev(usb_if, 0);

		/* now we can reset sync for the next time for all
		 * devices */
		dev0->time_sync.ts_us = 0;

#ifdef UCAN_USB_START_CM_AT_OPEN
		usb_mask |= UCAN_USB_OPTION_CALIBRATION;
#endif
	}

#ifdef UCAN_USB_OPTION_FAST_FWD
	/* clear fast-forward option */
	if (fast_fwd)
		usb_mask |= UCAN_USB_OPTION_FAST_FWD;
#endif

	/* turn off notifications */
	return ucan_usb_clr_dis_option(dev,
			UCAN_OPTION_ERROR|UCAN_OPTION_BUSLOAD,
			usb_mask);
}

/* do blink LED */
static int pcan_usbfd_identify(struct pcandev *dev, u32 delay_ms)
{
	int err;

	if (delay_ms) {

		/* set (FAST) blinking mode */
		err = ucan_usb_set_can_led(dev, UCAN_USB_LED_FAST);
		if (err)
			return err;

		pcan_msleep_interruptible(delay_ms);
	}

	/* restore "normal" mode */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_DEF);

	return !err ? UCAN_USB_LED_DEF : err;
}

/* handle uCAN Rx CAN message */
static int pcan_usbfd_decode_canrx(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_rx_msg *rm = (struct ucan_rx_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct pcan_timeval tv;
	int err;

	err = ucan_post_canrx_msg(dev, rm, 
			pcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(rm->ts_low),
						    le32_to_cpu(rm->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN error message */
static int pcan_usbfd_decode_error(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_error_msg *er = (struct ucan_error_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct pcan_timeval tv;
	int err;

	err = ucan_post_error_msg(dev, er,
			pcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(er->ts_low),
						    le32_to_cpu(er->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN bus_load message */
static int pcan_usbfd_decode_bus_load(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_bus_load_msg *bl = (struct ucan_bus_load_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;

#ifdef PCAN_DECODE_BUS_LOAD_TS
	struct pcan_timeval tv;

	int err = ucan_post_bus_load_msg(dev, bl,
			pcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(bl->ts_low),
						    le32_to_cpu(bl->ts_high),
						    &tv));
#else
	int err = ucan_post_bus_load_msg(dev, bl,
			/* don't lose time to decode timestamp: bus load events
			 * frequency is very high so use time of day instead */
			NULL);
#endif

	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN status message */
static int pcan_usbfd_decode_status(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_status_msg *st = (struct ucan_status_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct pcan_timeval tv;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": got status msg: ts=0x%08x-%08x RB=%u EP=%u EW=%u BO=%u\n",
		le32_to_cpu(st->ts_low), le32_to_cpu(st->ts_high),
		!!UCAN_STMSG_RB(st), !!UCAN_STMSG_PASSIVE(st),
		!!UCAN_STMSG_WARNING(st), !!UCAN_STMSG_BUSOFF(st));
#endif
	err = ucan_post_status_msg(dev, st,
			pcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(st->ts_low),
						    le32_to_cpu(st->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	/* bus is ok: if tx_engine idle, set it to STOPPED so that user will 
	 * initiate writing on it */
	if ((dev->bus_state != PCANFD_ERROR_BUSOFF) &&
			(dev->locked_tx_engine_state == TX_ENGINE_IDLE))
		pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

	return err;
}

/* handle uCAN USB overrun message */
static int pcan_usbfd_decode_overrun(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_usb_ovr_msg *ov = (struct ucan_usb_ovr_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct pcan_timeval tv;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": got overrun msg: ts=0x%08x-%08x\n",
		le32_to_cpu(ov->ts_low), le32_to_cpu(ov->ts_high));
#endif
	err = ucan_post_overflow_msg(dev,
			pcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(ov->ts_low),
						    le32_to_cpu(ov->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle USB calibration message:
 */
static int pcan_usbfd_decode_calibration(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct pcan_usb_interface *usb_if = (void *)arg;
	struct ucan_usb_ts_msg *ts = (struct ucan_usb_ts_msg *)rx_msg;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s, ts=%u, fi=%u)\n",
		__func__, usb_if->adapter->name, 
				le32_to_cpu(ts->ts_low),
				le16_to_cpu(ts->usb_frame_index));
#endif

	return pcan_usbpro_handle_calibration(usb_if,
				le32_to_cpu(ts->ts_low),
				le16_to_cpu(ts->usb_frame_index));
}

/* int pcan_usbfd_msg_decode(struct pcan_usb_interface *usb_if,
 *				u8 *msg_addr, int msg_len)
 *
 * Decode a message received from PCAN-USB (Pro) FD
 */
static int pcan_usbfd_msg_decode(struct pcan_usb_interface *usb_if,
					u8 *msg_addr, int msg_len)
{
	struct pcandev *dev;
	int err = 0, d;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%d)\n", __func__, msg_len);
	dump_mem("received msg", msg_addr, msg_len);
#endif

	/* do some init for each controller */
	for (d = 0; d < usb_if->can_count; d++) {
		dev = usb_if_dev(usb_if, d);
		dev->port.usb.state &= ~UCAN_USB_SHOULD_WAKEUP;
	}

	/* call default uCAN rx CAN messages handler */
	dev = usb_if_dev(usb_if, 0);
	err = ucan_handle_msgs_buffer(&dev->ucan, msg_addr, msg_len);

	/* check if something must be awake */
	for (d = 0; d < usb_if->can_count; d++) {
		dev = usb_if_dev(usb_if, d);

		if (dev->port.usb.state & UCAN_USB_SHOULD_WAKEUP) {

			dev->dwInterruptCounter++;
#ifndef NETDEV_SUPPORT
#ifdef DEBUG
			pr_info(DEVICE_NAME
				": wakeup task reading CAN%u\n", d+1);
#endif
			pcan_event_signal(&dev->in_event);
#endif
		}
	}

	return err;
}

#define USB_VENDOR_REQUEST_wVALUE_SETFKT_MASS_STORAGE	0

/*
 * Switch the device in Mass Storage Mode
 */
static int pcan_usbfd_set_mass_storage_mode(struct pcan_usb_interface *usb_if)
{
	return pcan_usbpro_request(usb_if,
			USB_VENDOR_REQUEST_FKT,
			USB_VENDOR_REQUEST_wVALUE_SETFKT_MASS_STORAGE,
			NULL, 0);
}

/*
 * void pcan_usbfd_cleanup(struct pcandev *dev)
 *
 * Last chance to submit URB before driver removal.
 */
static void pcan_usbfd_cleanup(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif
	/* Sometimes, bus off request can't be submit when driver is removed
	 * so, when the device was not properly closed. So, move the bus off
	 * request here to be sure it is sent.  */
	err = ucan_set_bus_off(dev);
	if (err)
		goto lbl_end;

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* No more need to switch off the LEDs by ourselves!
	 * Fw does it when we notify it from driver unload!  */
#else
	/* Switch LED off */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_OFF);
	if (err)
		goto lbl_end;
#endif

lbl_end:
	/* If last device, tell module that driver is unloaded */
	if (dev->can_idx == (usb_if->can_count-1)) {

#ifndef UCAN_USB_START_CM_AT_OPEN
		/* just to be sure to ignore pending CM */
		usb_if->cm_ignore_count = 1000;

		/* stop calibration msgs when cleaning up last channel */
		if (!err)
			ucan_usb_set_en_option(dev, 0,
						UCAN_USB_OPTION_CALIBRATION);
#endif
		/* Tell module the CAN driver is unloaded */
		pcan_usbpro_driver_loaded(usb_if, 0, 0);
	}
}

/*
 * int pcan_usbfd_ctrl_init(struct pcandev *dev)
 *
 * Do CAN controller specific initialization.
 */
static int pcan_usbfd_ctrl_init(struct pcandev *dev)
{
	const struct pcan_usb_interface *usb_if = dev->port.usb.usb_if;
	int err;

#if 0
	const int c = dev->can_idx + (usb_if->index * usb_if->can_count) + 1;

	pr_info(DEVICE_NAME ": %s channel %d device number=%u\n",
		dev->adapter->name, c, dev->device_alt_num);
#endif
	/* set LED in default state (end of init phase) */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_DEF);

	/* sending data is not allowed for the moment */

	/* can do this here because no isr is running at the moment, and only
	 * one process can do this (actually, the 1st one) */
	pcan_set_tx_engine(dev, TX_ENGINE_IDLE);

#ifndef UCAN_USB_START_CM_AT_OPEN
	if (dev->can_idx == (usb_if->can_count-1)) {

		/* start calibration msgs when initializing last channel */
		if (!err)
			ucan_usb_set_en_option(dev, 0,
						UCAN_USB_OPTION_CALIBRATION);
	}
#endif

	return err;
}

/*
 * void pcan_usbfd_free(struct pcan_usb_interface *usb_if)
 */
static void pcan_usbfd_free(struct pcan_usb_interface *usb_if)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case PCAN_USBX6_PRODUCT_ID:
		if (usb_if->usb_dev->portnum < 3)
			return;

		pcan_usbx6_devices--;
		break;
	case PCAN_USBPROFD_PRODUCT_ID:
		pcan_usbprofd_devices--;
		break;
	case PCAN_USBFD_PRODUCT_ID:
		pcan_usbfd_devices--;
		break;
	case PCAN_USBCHIP_PRODUCT_ID:
		pcan_chip_devices--;
		break;

	default:
		break;
	}
	/* release dynamic memory only once */
	usb_if->adapter = pcan_free_adapter(usb_if->adapter);
}

/* interface functions used to send commands / handle msgs to USB/uCAN */
static int (*pcan_usbfd_ucan_handlers[])(struct ucan_engine *,
					 struct ucan_msg *,
					 void *) = {
	[UCAN_MSG_CAN_RX] = pcan_usbfd_decode_canrx,
#if 0
	/* this handler "only" keeps a copy of rx/tx errors counters BUT don't
	 * push any msgs into the rx queue */
	[UCAN_MSG_ERROR] = ucan_handle_error,
	[UCAN_MSG_BUSLOAD] = ucan_handle_bus_load,
#else
	/* use this one if rx/tx errors counters are pushed into rx queue
	 * as STATUS messages */
	[UCAN_MSG_ERROR] = pcan_usbfd_decode_error,
	[UCAN_MSG_BUSLOAD] = pcan_usbfd_decode_bus_load,
#endif
	[UCAN_MSG_STATUS] = pcan_usbfd_decode_status,

	[UCAN_USB_MSG_CALIBRATION] = pcan_usbfd_decode_calibration,
	[UCAN_USB_MSG_OVERRUN] = pcan_usbfd_decode_overrun,
};

static struct ucan_ops pcan_usbfd_ucan_ops = {
	.set_clk_domain = pcan_usbfd_set_clk_domain,
	.send_cmd = pcan_usbfd_send_ucan_cmd,
	.handle_msg_table = pcan_usbfd_ucan_handlers,
	.handle_msg_size = ARRAY_SIZE(pcan_usbfd_ucan_handlers),
};

static struct pcan_usb_interface *
		pcan_usbfd_get_same_if(struct pcan_usb_interface *usb_if)
{
	struct pcan_usb_interface *usb_dev_if = NULL;
	struct pcandev *dev;
	struct list_head *ptr;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	pcan_lock_get_irqsave(&pcan_drv.devices_lock, flags);
#endif

	list_for_each(ptr, &pcan_drv.devices) {

		dev = (struct pcandev *)ptr;
		if (dev->wType != HW_USB_X6)
			goto lbl_continue;

		usb_dev_if = pcan_usb_get_if(dev);
		if (!usb_dev_if) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL usb_if for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				dev->can_idx+1);
			continue;
		}

		if (!usb_dev_if->usb_dev) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL usb_dev for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				dev->can_idx+1);
			goto lbl_continue;
		}
		if (!usb_dev_if->usb_dev->bus) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL bus for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				dev->can_idx+1);
			goto lbl_continue;
		}

		if (usb_dev_if->usb_dev->bus->busnum !=
					usb_if->usb_dev->bus->busnum)
			goto lbl_continue;

		/* see drivers/usb/core/usb.c usb_alloc_dev():
		 *
		 * if devices share the same bus, and the same parent, so they
		 * share the same interface.
		 *
		 * if devices share the same bus and the same port, so they
		 * share the same interface.
		 */
		if (usb_dev_if->usb_dev->route) {
			if (!usb_dev_if->usb_dev->parent) {
				pr_err(DEVICE_NAME
					": %s(L%u): %s: abnormal NULL bus for "
					"CAN%u\n",
					__func__, __LINE__,
					usb_if->adapter->name,
					dev->can_idx+1);
				goto lbl_continue;
			}

			if (usb_dev_if->usb_dev->parent->portnum ==
					usb_if->usb_dev->parent->portnum)
				break;
		} else {
			if (usb_dev_if->usb_dev->portnum ==
					usb_if->usb_dev->portnum)
				break;
		}
lbl_continue:
		usb_dev_if = NULL;
	}

#ifdef HANDLE_HOTPLUG
	pcan_lock_put_irqrestore(&pcan_drv.devices_lock, flags);
#endif
	return usb_dev_if;
}

/*
 * int pcan_usbfd_init(struct pcan_usb_interface *usb_if)
 *
 * Do device specific initialization.
 */
int pcan_usbfd_init(struct pcan_usb_interface *usb_if)
{
	int c, err, should_print = 1;
	u32 bl_version;

	if (!usb_if) {
		pr_info(DEVICE_NAME ": NULL usb_if!\n");
		return -ENODEV;
	}
	if (!usb_if->usb_dev) {
		pr_info(DEVICE_NAME ": NULL usb_if->usb_dev!\n");
		return -ENODEV;
	}

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s(bus=%d port=%d parent_port=%d can_count=%d)\n",
		__func__,
		usb_if->usb_dev->bus->busnum,
		usb_if->usb_dev->portnum,
		usb_if->usb_dev->parent->portnum,
		usb_if->can_count);
#endif

	usb_if->cm_ignore_count = 3;
	usb_if->dev_frame_index = -1;

	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case PCAN_USBPROFD_PRODUCT_ID:
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB Pro FD",
						pcan_usbprofd_devices++,
						usb_if->can_count);
		break;

	case PCAN_USBX6_PRODUCT_ID:
		if (usb_if->usb_dev->portnum == 1) {
			usb_if->adapter = pcan_alloc_adapter(str_pcan_usb_x6,
						pcan_usbx6_devices++,
						6);
		} else {
			struct pcan_usb_interface *same_if =
					pcan_usbfd_get_same_if(usb_if);

			if (!same_if) {
				pr_err(DEVICE_NAME
					": %s #%u: unable to find same usb "
					"interface.\n",
					str_pcan_usb_x6, pcan_usbx6_devices);
				return -ENODEV;
			}

			usb_if->adapter = same_if->adapter;
			should_print = 0;
		}
		break;

	case PCAN_USBFD_PRODUCT_ID:
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB FD",
						pcan_usbfd_devices++,
						usb_if->can_count);
		break;

	case PCAN_USBCHIP_PRODUCT_ID:

		/* PCAN-Chip workaround: ignore calibration messages while
		 * both sync and timestamps clocks aren't synced... */
		usb_if->cm_ignore_count = -1;
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB Chip",
						pcan_chip_devices++,
						usb_if->can_count);
		break;
	}

	if (!usb_if->adapter)
		return -ENOMEM;

	/* Set PCAN-USB (Pro) FD hardware specific callbacks */
	usb_if->device_ctrl_init = pcan_usbfd_ctrl_init;
	usb_if->device_get_snr = pcan_usbfd_get_serial_nr;
	usb_if->device_msg_decode = pcan_usbfd_msg_decode;
	usb_if->device_free = pcan_usbfd_free;

	usb_if->device_ctrl_cleanup = pcan_usbfd_cleanup;
	usb_if->device_ctrl_open_fd = pcan_usbfd_open_fd;
	usb_if->device_ctrl_close = pcan_usbfd_close;
	usb_if->device_ctrl_set_bus_on = ucan_set_bus_on;
	usb_if->device_ctrl_set_bus_off = ucan_set_bus_off;
	usb_if->device_ctrl_set_dnr = pcan_usbfd_set_device_nr;
	usb_if->device_ctrl_get_dnr = pcan_usbfd_get_device_nr;
	usb_if->device_ctrl_msg_encode = ucan_encode_msgs_buffer;

	for (c = 0; c < usb_if->can_count; c++) {
		struct pcandev *dev = usb_if_dev(usb_if, c);
		if (!dev) {
			pr_err(DEVICE_NAME
				": %s: ABNORMAL NULL dev #%u/%u\n",
				usb_if->adapter->name, c, usb_if->can_count);
			return -ENODEV;
		}

		/* setup identification callback */
		dev->device_identify = pcan_usbfd_identify;

		/* set uCAN interface function to send cmds and handle msgs */
		dev->ucan.ops = &pcan_usbfd_ucan_ops;

		/* remember the list of channels in each channel */
		dev->ucan.devs = usb_if->devs;
		dev->ucan.devs_count = usb_if->can_count;

		/* use the allocated commands buffer for building uCAN cmds */
		dev->ucan.cmd_head = dev->port.usb.cout_baddr;
		dev->ucan.cmd_size = dev->port.usb.cout_bsize;
	}

	/* Tell module the CAN driver is loaded */
	err = pcan_usbpro_driver_loaded(usb_if, 0, 1);
	if (err) {
		pr_err(DEVICE_NAME
			": unable to tell %s that driver is loaded (err %d)\n",
			usb_if->adapter->name, err);
		return err;
	}

	/* read fw info */
	err = pcan_usbfd_get_fw_info(usb_if, should_print, &bl_version);
	if (err) {
		pr_err(DEVICE_NAME
			": unable to read fw info from %s (err %d)\n",
			usb_if->adapter->name, err);
		return err;
	}

	/* init adapter fw version with (only) first interface fw version 
	 * (PCAN-USB X6 incldues 3x USB interfaces, each one might run a
	 * separate fw version) */
	if (!usb_if->index)
		usb_if->adapter->hw_ver = usb_if->hw_ver;

	/* setting Mass Storage Mode available for all USB FD devices, except
	 * for PCAN-USB FD and PCAN-USB Pro FD with bootloader version < 2 */
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case PCAN_USBFD_PRODUCT_ID:
	case PCAN_USBPROFD_PRODUCT_ID:
		if (bl_version < VER_NUM(2, 0, 0))
			break;

		/* fall through */
	default:
		usb_if->device_set_mass_storage_mode =
			pcan_usbfd_set_mass_storage_mode;
		break;
	}

	return 0;
}
#endif /* USB_SUPPORT */
