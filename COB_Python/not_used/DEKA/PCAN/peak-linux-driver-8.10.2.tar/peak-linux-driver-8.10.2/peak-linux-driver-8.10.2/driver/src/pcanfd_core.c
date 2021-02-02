/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAN-FD extension to PEAK-System CAN products.
 *
 * Copyright (C) 2015-2020 PEAK System-Technik GmbH <www.peak-system.com>
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
/*#define DEBUG*/
/*#undef DEBUG*/

#include "src/pcanfd_core.h"
#include "src/pcan_filter.h"

#ifdef DEBUG
#define DEBUG_WAIT_RD
#define DEBUG_WAIT_WR
#define DEBUG_OPEN
#define DEBUG_BITRATE
#else
//#define DEBUG_WAIT_RD
//#define DEBUG_WAIT_WR
//#define DEBUG_OPEN
//#define DEBUG_BITRATE
#endif

/* Timeout set to task waiting for room in the tx queue.
 * 0 means infinite.
 * != 0 implies that the wait might end with -ETIMEDOUT. */
//#define PCANFD_TIMEOUT_WAIT_FOR_WR	100
#define PCANFD_TIMEOUT_WAIT_FOR_WR	0

/* if defined, default init settings will be used in case any error in user
 * bittiming is found. */
//#define PCAN_USE_DEFBT_ON_ERROR

extern u16 btr0btr1;
extern u32 pcan_def_dbitrate;

/*
 * Compute bitrate according to bittiming spec and Clock frequency
 */
static int pcan_bittiming_to_bitrate(struct pcan_bittiming *pbt, u32 clk_Hz)
{
	/* avoid div by 0 */
	if (pbt->brp) {
		u64 v64;

		if (!pbt->sjw)
			pbt->sjw = 1; /* ??? */

		pbt->sample_point = (PCAN_SAMPT_SCALE *
			(1 + pbt->tseg1)) / (1 + pbt->tseg1 + pbt->tseg2);

		pbt->bitrate = pcan_get_bps(clk_Hz, pbt);

		v64 = (u64 )pbt->brp * GHz;
		do_div(v64, clk_Hz);
		pbt->tq = (u32 )v64;

		return 0;
	}

	pr_warn(DEVICE_NAME
		": %s(): cannot compute bitrate from invalid brp=%d\n",
		__func__, pbt->brp);

	return -EINVAL;
}

int pcan_bittiming_normalize(struct pcan_bittiming *pbt,
			u32 clock_Hz, const struct pcanfd_bittiming_range *caps)
{
	int err = -EINVAL;

#ifdef DEBUG_BITRATE
	pcanfd_dump_bittiming(pbt, clock_Hz);
#endif

	/* NEW 8.2: always trust BRP/TEGx first:
	 * if brp valid, use these for computing the bitrate field */
	if (pbt->brp) {
		if (pbt->brp < caps->brp_min)
			pbt->brp = caps->brp_min;
		else if (pbt->brp > caps->brp_max)
			pbt->brp = caps->brp_max;

		if (pbt->tseg1 < caps->tseg1_min)
			pbt->tseg1 = caps->tseg1_min;
		else if (pbt->tseg1 > caps->tseg1_max)
			pbt->tseg1 = caps->tseg1_max;

		if (pbt->tseg2 < caps->tseg2_min)
			pbt->tseg2 = caps->tseg2_min;
		else if (pbt->tseg2 > caps->tseg2_max)
			pbt->tseg2 = caps->tseg2_max;

		err = pcan_bittiming_to_bitrate(pbt, clock_Hz);

	} else if (pbt->bitrate) {
		err = pcan_bitrate_to_bittiming(pbt, caps, clock_Hz);

#ifdef DEBUG_BITRATE
	/* else, if any of them is valid, it's an error! */
	} else {
		pr_info("%s: invalid bittiming specs: unable to normalize\n",
							DEVICE_NAME);
#endif
	}

	/* real bit-rate */
	if (!err)
		pbt->bitrate_real = clock_Hz /
		 	(pbt->brp * (pbt->tseg1 + pbt->tseg2 + 1));

#ifdef DEBUG_BITRATE
	pcanfd_dump_bittiming(pbt, clock_Hz);
#endif
	return err;
}

/*
 * Convert SJA1000 BTR0BTR1 16-bits value into a generic bittiming
 * representation
 */
struct pcan_bittiming *pcan_btr0btr1_to_bittiming(struct pcan_bittiming *pbt,
						  u16 btr0btr1)
{
	pbt->sjw = 1 + ((btr0btr1 & 0xc000) >> 14);
	pbt->brp = 1 + ((btr0btr1 & 0x3f00) >> 8);
	pbt->tsam = (btr0btr1 & 0x0080) >> 7;
	pbt->tseg2 = 1 + ((btr0btr1 & 0x0070) >> 4);
	pbt->tseg1 = 1 + (btr0btr1 & 0x000f);
	pbt->bitrate = 0;

	pcan_bittiming_to_bitrate(pbt, 8*MHz);

#if 0//def DEBUG_BITRATE
	pr_info(DEVICE_NAME ": %s(btr0btr1=%04xh) =>\n", __func__, btr0btr1);
	pcanfd_dump_bittiming(pbt, 8*MHz);
#endif

	return pbt;
}

/* Convert old CAN 2.0 init object into new-style CAN-FD init object. */
struct pcanfd_init *pcan_init_to_fd(struct pcandev *dev,
				    struct pcanfd_init *pfdi,
				    const TPCANInit *pi)
{
#if 1
	/* DON'T memset('\0') the struct pcanfd_init since it may already 
	 * contain data (or other CANFD specific values). Caller HAS TO
	 * initialize the struct pcanfd_init by himself! */
	memset(&pfdi->data, '\0', sizeof(struct pcan_bittiming));
#else

	memset(pfdi, '\0', sizeof(*pfdi));
#endif

	if (!pfdi->clock_Hz)
		pfdi->clock_Hz = dev->sysclock_Hz;

	if (!(pi->ucCANMsgType & MSGTYPE_EXTENDED))
		pfdi->flags |= PCANFD_INIT_STD_MSG_ONLY;

	if (pi->ucListenOnly)
		pfdi->flags |= PCANFD_INIT_LISTEN_ONLY;

	if (pi->wBTR0BTR1) {
		pcan_btr0btr1_to_bittiming(&pfdi->nominal, pi->wBTR0BTR1);

		if (pfdi->clock_Hz != 8*MHz) {

			/* compute new bittiming according to the real clock */
			pcan_bitrate_to_bittiming(&pfdi->nominal,
						  dev->bittiming_caps,
						  pfdi->clock_Hz);
		}
	}

#ifdef DEBUG_BITRATE
	pr_info(DEVICE_NAME ": %s(): 8xMHz btr0btr1=%04xh =>\n",
		__func__, pi->wBTR0BTR1);
	pcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
#endif
	return pfdi;
}

/* reset fifo queues and counters of a pcan device and release it.
 * WARNING: caller should normally wait for the output fifo to be empty
 *          before calling pcanfd_dev_reset()
 */
int pcanfd_dev_reset(struct pcandev *dev)
{
	int err;

	/* close Tx engine BEFORE device_release() so that device Tx resources
	 * will be able to be safety released from writing task.
	 * */
	pcan_lock_irqsave_ctxt flags;

	//pcan_lock_get_irqsave(&dev->isr_lock, flags);
	dev->lock_irq(dev, &flags);
	pcan_set_tx_engine(dev, TX_ENGINE_CLOSED);
	//pcan_lock_put_irqrestore(&dev->isr_lock, flags);
	dev->unlock_irq(dev, &flags);

	/* release the device (if it has been opened) */
	if (dev->nOpenPaths && dev->device_release)
		dev->device_release(dev);

	dev->flags &= ~PCAN_DEV_OPENED;
	pcan_set_bus_state(dev, PCANFD_UNKNOWN);

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s CAN%u rx=%u tx=%u\n",
			dev->adapter->name, dev->can_idx+1,
			dev->rx_frames_counter, dev->tx_frames_counter);
#endif
	/* flush fifo contents */
	err = pcan_fifo_reset(&dev->writeFifo);
	if (err)
		goto reset_fail;

#ifndef NETDEV_SUPPORT
	err = pcan_fifo_reset(&dev->readFifo);
	if (err)
		goto reset_fail;
#endif

	dev->wCANStatus &= ~(CAN_ERR_OVERRUN|CAN_ERR_XMTFULL);

reset_fail:
	return err;
}

/* do a smart copy to avoid setting dbitrate to 0 for CAN-FD capable devices
 * Note that nominal and data bitrate SHOULD be normalized... */
void pcanfd_copy_init(struct pcanfd_init *pd, struct pcanfd_init *ps)
{
#if 1
	/* back to old behaviour: do a full copy of user settings so that
	 * outside world is aware that the CAN-FD device is open in CAN 2.0
	 * mode only if data_bitrate equals 0!*/
	*pd = *ps;
#else
	pd->flags = ps->flags;

	if (ps->clock_Hz)
		pd->clock_Hz = ps->clock_Hz;

	pd->nominal = ps->nominal;
	if (pd->flags & PCANFD_INIT_FD)
		pd->data = ps->data;
#endif
}

/* default allowed msgs mask equals all messages except ERR_MSG */
#define PCANFD_ALLOWED_MSG_DEFAULT      (PCANFD_ALLOWED_MSG_CAN|\
					 PCANFD_ALLOWED_MSG_RTR|\
					 PCANFD_ALLOWED_MSG_EXT|\
					 PCANFD_ALLOWED_MSG_STATUS)

void pcanfd_dev_open_init(struct pcandev *dev)
{
	/* nofilter */
	dev->acc_11b.code = 0;
	dev->acc_11b.mask = CAN_MAX_STANDARD_ID;
	dev->acc_29b.code = 0;
	dev->acc_29b.mask = CAN_MAX_EXTENDED_ID;

	dev->tx_iframe_delay_us = 0;
	dev->allowed_msgs = PCANFD_ALLOWED_MSG_DEFAULT;

#ifndef PCAN_USE_BUS_LOAD_TIMER
	dev->bus_load_ind_last_ts = jiffies;
#endif
	/* Nope! Since pcan v8.6, time sync is handled as soon as the device
	 * hw is probed (mainly USB devices): starting/stopping CM when
	 * opening/closing the PCAN-Chip does not work, since ts in CM are not
	 * based like ts in CAN msgs...
	 * (see also: UCAN_USB_START_CM_AT_OPEN) */
	//pcan_sync_init(dev);

	dev->rx_frames_counter = 0;
	dev->tx_frames_counter = 0;

	/* New: reset these counters too */
	dev->dwErrorCounter = 0;
	dev->dwInterruptCounter = 0;
}

static int pcanfd_fix_init_clock(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	const struct pcanfd_available_clocks *pc = dev->clocks_list;
	int i, err;

	for (i = 0; i < pc->count; i++)
		if (pfdi->clock_Hz == pc->list[i].clock_Hz)
			break;

	/* user clock not found in device clocks list */
	if (i >= pc->count) {

		if (pfdi->nominal.brp) {

			/* to be compatible with old API, accept 8*MHz clocks
			 * and consider that BRP values and so on come from
			 * SJA1000 BTR0BTR1. */
			if (pfdi->clock_Hz == 8*MHz)
				return 0;

			/* otherwise, try to convert bittiming into bitrate
			 * using the user (wrong) clock */
			err = pcan_bittiming_to_bitrate(&pfdi->nominal,
							pfdi->clock_Hz);
			if (err) {
				pr_err(DEVICE_NAME ": unable to convert"
				       " user nominal bittiming with wrong clk"
				       "=%uMHz (err %d)\n",
				       pfdi->clock_Hz, err);

				return err;
			}

			/* rst BRP to force using bitrate value next */
			pfdi->nominal.brp = 0;
		}

		/* do the same for data bittiming */
		if (pfdi->flags & PCANFD_INIT_FD) {
			if (pfdi->data.brp) {
				err = pcan_bittiming_to_bitrate(&pfdi->data,
								pfdi->clock_Hz);
				if (err) {
					pr_err(DEVICE_NAME ": unable to convert"
					       " user data bittiming with wrong"
					       "clk=%uMHz (err %d)\n",
					       pfdi->clock_Hz, err);

					return err;
				}
			}
		}

		/* fix user clock with device default one to convert
		 * the bitrate value in valid bittiming */
		pfdi->clock_Hz = dev->sysclock_Hz;
	}

	return 0;
}

/* consider pfdi ok */
static int __pcanfd_dev_open(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	struct pcanfd_init tmp_init;
	int err;

	/* sanitize */
	if (!(dev->features & PCAN_DEV_BUSLOAD_RDY))
		pfdi->flags &= ~PCANFD_INIT_BUS_LOAD_INFO;

	/* do this BEFORE calling open callbacks, to be ready to handle
	 * timestamps conversion if any msg is posted by them. These two init
	 * steps are made again at the end, as usual. */
	pcan_gettimeofday(&dev->init_timestamp);

	pcanfd_copy_init(&tmp_init, &dev->init_settings);
	pcanfd_copy_init(&dev->init_settings, pfdi);

	pcanfd_dev_open_init(dev);

	/* use old APi entry (with wBTR0BTR1) if CAN controller clock
	 * is 8 MHz. */
	if (pfdi->clock_Hz == 8*MHz) {
		TPCANInit init;

		pfdi->flags &= ~(PCANFD_INIT_FD|PCANFD_INIT_FD_NON_ISO);
		memset(&pfdi->data, '\0', sizeof(struct pcan_bittiming));

		init.ucCANMsgType = (pfdi->flags & PCANFD_INIT_STD_MSG_ONLY) ?
						0 : MSGTYPE_EXTENDED;
		init.ucListenOnly = !!(pfdi->flags & PCANFD_INIT_LISTEN_ONLY);

		/* we're sure that the bittiming are ok: no need to check nor
		 * convert them again. */
		init.wBTR0BTR1 = pcan_bittiming_to_btr0btr1(&pfdi->nominal);
		if (!init.wBTR0BTR1) {
			init.wBTR0BTR1 = sja1000_bitrate(
				dev->def_init_settings.nominal.bitrate,
				dev->def_init_settings.nominal.sample_point,
				dev->def_init_settings.nominal.sjw);
#ifdef DEBUG
			pr_err(DEVICE_NAME
				": %s CAN%u using default BTR0BTR1\n",
				dev->adapter->name, dev->can_idx+1);
#endif
			//return -EINVAL;
		}

#ifdef DEBUG_BITRATE
		pr_info(DEVICE_NAME
			": %s(CAN%d): time=%u.%06us: opening with "
			"BTR0BTR1=%04xh (bitrate=%u sp=%u) flags=%08xh\n",
			dev->adapter->name, dev->can_idx+1,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec,
			init.wBTR0BTR1, pfdi->nominal.bitrate,
			pfdi->nominal.sample_point, pfdi->flags);
		pr_info(DEVICE_NAME
			": nominal [brp=%d tseg1=%d tseg2=%d sjw=%d]\n",
			pfdi->nominal.brp, pfdi->nominal.tseg1,
			pfdi->nominal.tseg2, pfdi->nominal.sjw);
#endif

		/* device is not CAN-FD capable: forward to (old) CAN 2.0 API */
		err = dev->device_open(dev, init.wBTR0BTR1,
						init.ucCANMsgType,
						init.ucListenOnly);

	/* use the new API entry point (erroneously called "open_fd") that 
	 * enable to setup the true bitimings according to a given clock, as
	 * well as definig a data bitrate (CAN-FD) */
	} else {

#ifdef DEBUG_BITRATE
		pr_info(DEVICE_NAME
			": %s(CAN%d): time=%u.%06us: opening with "
			"clk=%u bitrate=%u dbitrate=%u (flags=%08xh)\n",
			dev->adapter->name, dev->can_idx+1,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec,
			pfdi->clock_Hz, pfdi->nominal.bitrate,
			pfdi->data.bitrate, pfdi->flags);
		pr_info(DEVICE_NAME
			": nominal [brp=%d tseg1=%d tseg2=%d sjw=%d sp=%u]\n",
			pfdi->nominal.brp, pfdi->nominal.tseg1,
			pfdi->nominal.tseg2, pfdi->nominal.sjw,
			pfdi->nominal.sample_point);
		pr_info(DEVICE_NAME
			": data [brp=%d tseg1=%d tseg2=%d sjw=%d sp=%u]\n",
			pfdi->data.brp, pfdi->data.tseg1, pfdi->data.tseg2,
			pfdi->data.sjw, pfdi->data.sample_point);
#endif
		err = dev->device_open_fd(dev, pfdi);
	}

	if (!err) {
		pcan_lock_irqsave_ctxt flags;

		dev->flags |= PCAN_DEV_OPENED;

		pcan_gettimeofday(&dev->init_timestamp);

		/* remember the init settings for further usage */
		pcanfd_copy_init(&dev->init_settings, pfdi);

		/* default tx engine state: ready to start! */
		dev->lock_irq(dev, &flags);

		if (dev->locked_tx_engine_state == TX_ENGINE_CLOSED)
			pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

		dev->unlock_irq(dev, &flags);

#ifdef PCAN_USE_BUS_LOAD_TIMER
		/* start STATUS[PCANFD_BUS_LOAD] timer */
		if (dev->init_settings.flags & PCANFD_INIT_BUS_LOAD_INFO) {
			mod_timer(&dev->bus_load_timer,
					jiffies + dev->bus_load_ind_period);
		}
#endif
		return 0;
	}

	/* since these settings are bad, should undo the above 
	 * pcanfd_copy_init() */
	pcanfd_copy_init(&dev->init_settings, &tmp_init);

	return err;
}

int pcanfd_dev_open(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, dev->can_idx+1);
#endif

	if (pfdi->flags & PCANFD_INIT_USER) {

#ifdef DEBUG_BITRATE
		pcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
		if (pfdi->flags & PCANFD_INIT_FD)
			pcanfd_dump_bittiming(&pfdi->data, pfdi->clock_Hz);
#endif

		/* check init settings: */
		if (pfdi->flags & PCANFD_INIT_FD)
			if (!dev->dbittiming_caps ||
			    !(dev->features & PCAN_DEV_FD_RDY)) {

				pr_err(DEVICE_NAME ": %s CAN%u: "
					"can't be opened in CAN FD mode\n",
					dev->adapter->name, dev->can_idx+1);

				return -EINVAL;
			}

		/* if user has given no clock, use device current clock */
		if (!pfdi->clock_Hz)
			pfdi->clock_Hz = dev->sysclock_Hz;

		/* otherwise, check if user clock is valid. If not AND
		 * if bittiming are used, then convert them into values that
		 * match the device default clock. */
		else {
			err = pcanfd_fix_init_clock(dev, pfdi);
			if (err)
				return err;
		}

		/* if user has not given any bitrate nor BRP, setup default
		 * settings for the nominal bitrate */
		if (!pfdi->nominal.bitrate && !pfdi->nominal.brp)
			pfdi->nominal = dev->def_init_settings.nominal;

		/* be sure that bitrate and brp,tsegx,sjw are set */
		err = pcan_bittiming_normalize(&pfdi->nominal,
					pfdi->clock_Hz, dev->bittiming_caps);
		if (err) {
#ifdef PCAN_USE_DEFBT_ON_ERROR
			pr_err(DEVICE_NAME
				": %s CAN%u: error %d in user nominal "
				"bittiming: using default\n",
				dev->adapter->name, dev->can_idx+1, err);
			pfdi->nominal = dev->def_init_settings.nominal;
#else
			pr_err(DEVICE_NAME
				": %s CAN%u: error %d in user nominal "
				"bittiming:\n",
				dev->adapter->name, dev->can_idx+1, err);

			pcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);

			return err;
#endif
		}

#ifdef DEBUG_OPEN
		pr_info(DEVICE_NAME
			": opening %s CAN%u with nominal bittiming:\n",
			dev->adapter->name, dev->can_idx+1);

		pcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
#endif

		/* do the same if CAN-FD */
		if (pfdi->flags & PCANFD_INIT_FD) {

			/* if user has not given any bitrate nor BRP,
			 * setup default settings for the data bitrate */
			if (!pfdi->data.bitrate && !pfdi->data.brp)
				pfdi->data = dev->def_init_settings.data;

			/* be sure that bitrate and brp,tsegx,sjw are set */
			err = pcan_bittiming_normalize(&pfdi->data,
					pfdi->clock_Hz, dev->dbittiming_caps);
			if (err) {
#ifdef PCAN_USE_DEFBT_ON_ERROR
				pr_err(DEVICE_NAME
					": %s CAN%u: error %d in user data "
					"bittiming: using default\n",
					dev->adapter->name, dev->can_idx+1,
					err);

				pfdi->data = dev->def_init_settings.data;
#else
				pr_err(DEVICE_NAME
					": %s CAN%u: error %d in user data "
					"bittiming\n",
					dev->adapter->name, dev->can_idx+1,
					err);

				pcanfd_dump_bittiming(&pfdi->data,
						      pfdi->clock_Hz);

				return err;
#endif
			}

			/* For CAN FD the data bitrate has to be >= the
			 * nominal bitrate */
			if (pfdi->data.bitrate < pfdi->nominal.bitrate) {
				pr_err(DEVICE_NAME ": %s CAN%u data bitrate "
					"(%u bps) should be greater than "
					"nominal bitrate (%u bps)\n",
					dev->adapter->name, dev->can_idx+1,
					pfdi->data.bitrate,
					pfdi->nominal.bitrate);

				return -EINVAL;
			}
#ifdef DEBUG_OPEN
			pr_info(DEVICE_NAME
				": opening %s CAN%u with data bittiming:\n",
				dev->adapter->name, dev->can_idx+1);

			pcanfd_dump_bittiming(&pfdi->data, pfdi->clock_Hz);
#endif
		}

		/* no need to check them next */
		pfdi->flags &= ~PCANFD_INIT_USER;
	}

	return __pcanfd_dev_open(dev, pfdi);
}

int pcanfd_ioctl_set_init(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	err = pcanfd_dev_reset(dev);
	if (err)
		goto lbl_unlock;

	/* force bittiming checking */
	pfdi->flags |= PCANFD_INIT_USER;

	err = pcanfd_dev_open(dev, pfdi);

lbl_unlock:
	return err;
}

int pcanfd_ioctl_get_init(struct pcandev *dev, struct pcanfd_init *pfdi)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	memcpy(pfdi, &dev->init_settings, sizeof(*pfdi));

	return 0;
}

/* add a message filter_element into the filter chain or delete all
 * filter_elements
 */
int pcanfd_ioctl_add_filter(struct pcandev *dev, struct pcanfd_msg_filter *pf)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!pf) {
		pcan_delete_filter_all(dev->filter);
		return 0;
	}

	return pcan_add_filter(dev->filter,
		               pf->id_from, pf->id_to, pf->msg_flags);
}

/* add several message filter_element into the filter chain.
 */
int pcanfd_ioctl_add_filters(struct pcandev *dev,
						struct pcanfd_msg_filters *pfl)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!pfl) {
		pcan_delete_filter_all(dev->filter);
		return 0;
	}

	return pcan_add_filters(dev->filter, pfl->list, pfl->count);
}

/* get several message filter_element from the filter chain.
 */
int pcanfd_ioctl_get_filters(struct pcandev *dev,
						struct pcanfd_msg_filters *pfl)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif

	/* filter == NULL -> return the current nb of filters in the chain */
	if (!pfl)
		return pcan_get_filters_count(dev->filter);

	err = pcan_get_filters(dev->filter, pfl->list, pfl->count);
	if (err < 0) {
		pfl->count = 0;
		return err;
	}

	pfl->count = err;
	return 0;
}

int pcanfd_ioctl_get_state(struct pcandev *dev, struct pcanfd_state *pfds)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, dev->can_idx+1);
#endif

	pfds->ver_major = PCAN_VERSION_MAJOR;
	pfds->ver_minor = PCAN_VERSION_MINOR;
	pfds->ver_subminor = PCAN_VERSION_SUBMINOR;

	pfds->tv_init = dev->init_timestamp;

	pfds->bus_state = dev->bus_state;
	pfds->device_id = dev->device_alt_num;

	pfds->open_counter = dev->nOpenPaths;
	pfds->filters_counter = pcan_get_filters_count(dev->filter);

	pfds->hw_type = dev->wType;
	pfds->channel_number = dev->can_idx;

#ifdef USB_SUPPORT
	if (dev->wType == HW_USB_X6) {
		struct pcan_usb_interface *usb_if;

		usb_if = pcan_usb_get_if(dev);

		pfds->channel_number += usb_if->index * usb_if->can_count;
	}
#endif
 
	pfds->can_status = dev->wCANStatus;
	pfds->bus_load = dev->bus_load;

	pfds->tx_max_msgs = dev->writeFifo.nCount;
	pfds->tx_pending_msgs = dev->writeFifo.nStored;

#ifndef NETDEV_SUPPORT
	pfds->rx_max_msgs = dev->readFifo.nCount;
	pfds->rx_pending_msgs = dev->readFifo.nStored;
#else
	pfds->rx_max_msgs = 0;
	pfds->rx_pending_msgs = 0;
#endif
	pfds->tx_error_counter = dev->tx_error_counter;
	pfds->rx_error_counter = dev->rx_error_counter;
	pfds->tx_frames_counter = dev->tx_frames_counter;
	pfds->rx_frames_counter = dev->rx_frames_counter;

	pfds->host_time_ns = dev->time_sync.tv_ns;
	pfds->hw_time_ns = dev->time_sync.ts_us * 1000;

	return 0;
}

static int pcanfd_recv_msg(struct pcandev *dev, struct pcanfd_rxmsg *pf,
		                                        struct pcan_udata *ctx)
{
#ifdef NETDEV_SUPPORT
	return -EAGAIN;		/* be compatible with old behaviour */
#else
	FIFO_MANAGER *rx_fifo = &dev->readFifo;
	int err;

	do {
		/* if the device has been plugged out while waiting,
		 * or if any task is closing it */
		if (!dev->is_plugged || !dev->nOpenPaths) {
			err = -ENODEV;
			break;
		}

		/* get data from fifo */
		err = pcan_fifo_get(rx_fifo, pf);
		if (err >= 0) {

			pcan_clear_status_bit(dev, CAN_ERR_OVERRUN);

			pcan_sync_timestamps(dev, pf);
#if 0
			if (pf->msg.type == PCANFD_TYPE_CAN20_MSG)
				pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

#ifdef DEBUG_WAIT_RD
			pr_info("%s: %s(%u): still %u items in Rx queue\n",
				DEVICE_NAME, __func__, __LINE__,
				rx_fifo->nStored);
#endif
#ifdef pcan_event_clear
			/* if rx fifo is now empty, then the corresponding event
			 * should be cleared now.
			 * This section MUST be atomic regarding the rx fifo.
			 * Note: if (ctx->open_flags & O_NONBLOCK) { ? */
			{
				pcan_lock_irqsave_ctxt flags;

				pcan_lock_get_irqsave(&rx_fifo->lock, flags);

				if (pcan_fifo_empty(rx_fifo))
					pcan_event_clear(&dev->in_event);

				pcan_lock_put_irqrestore(&rx_fifo->lock, flags);
			}
#endif
			err = 0;
			break;
		}

		/* support nonblocking read if requested */
		if (ctx->open_flags & O_NONBLOCK) {
			err = -EAGAIN;
			break;
		}

		/* check whether the task is able to wait:
		 * Linux: always!
		 * RT: depends on the RT context of the running task */
		if (!pcan_task_can_wait()) {
			pr_info(DEVICE_NAME
				": %s(%u): ABNORMAL task unable to wait!\n",
				__func__, __LINE__);
			err = -EAGAIN;
			break;
		}

		/* sleep until some msg is available. */
#ifdef DEBUG_WAIT_RD
		pr_info("%s: %s(%u): waiting for some msgs to read...\n",
			DEVICE_NAME, __func__, __LINE__);
#endif

#ifdef NO_RT
		/* task might go to sleep: unlock current dev */
		pcan_mutex_unlock(&dev->mutex);
#endif
		/* wait for some msg in the Rx queue.
		 *
		 * Note: ^C may occur while waiting. In RT, preemption can 
		 * schedule another task that might call close() while we're
		 * always waiting here.
		 * - If the event is destroyed by some other task, the below
		 *   call fails with err=-EIDRM(43).
		 * - if some other task deletes this waiting task, this tasks
		 *   is first unblocked, thus err=-EINTR(4).*/
		err = pcan_event_wait(dev->in_event,
					!dev->is_plugged ||
					!pcan_fifo_empty(rx_fifo));

#ifdef NO_RT
		pcan_mutex_lock(&dev->mutex);
#endif

#ifdef DEBUG_WAIT_RD
		pr_info(DEVICE_NAME
			": end of waiting for rx fifo not empty: err=%d\n",
			err);
#endif

	} while (err >= 0);

	/* Note: ERESTARTSYS == 512 */
	return (err == -ERESTARTSYS) ? -EINTR : err;
#endif
}

/* this function SHOULD be used with dev->isr_lock locked */
int __pcan_dev_start_writing(struct pcandev *dev, struct pcan_udata *ctx)
{
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u bus_state=%u)\n",
		__func__, dev->adapter->name, dev->can_idx+1, dev->bus_state);
#endif

#ifdef PCAN_WRITES_ON_ACTIVE
	/* Hem... this should be tested here. But PCAN-USB takes up to ~900 ms
	 * to notify from ERROR_ACTIVE. See also handle_error_active() in
	 * src/pcan_main.c */
	if (dev->bus_state == PCANFD_UNKNOWN) {
		return 0;
	}
#endif
	/* if we just put the 1st message (=the fifo was empty), we can start
	 * writing on hardware if it is ready for doing this. */
	if (dev->locked_tx_engine_state == TX_ENGINE_STOPPED) {

		//pr_info(DEVICE_NAME ": [%u] TX_ENGINE_STOPPED => start writing\n", task_pid_nr(current));
		err = dev->device_write(dev, ctx);
	}

	/* since v8.8, device_write() should not return -ENODATA except if no
	 * data has been read from the Tx fifo. Since __pcan_dev_start_writing()
	 * is called after having put a frame into the Tx fifo, err
	 * cannot be -ENODATA. */
	return (err == -ENODATA) ? 0 : err;
	//return err;
}

static int pcanfd_start_tx_engine(struct pcandev *dev, struct pcan_udata *ctx)
{
	pcan_lock_irqsave_ctxt lck_ctx;
	int err;

	//pcan_lock_get_irqsave(&dev->isr_lock, lck_ctx);
	dev->lock_irq(dev, &lck_ctx);

		/* if can device ready to send, start writing */
		err = __pcan_dev_start_writing(dev, ctx);

	//pcan_lock_put_irqrestore(&dev->isr_lock, lck_ctx);
	dev->unlock_irq(dev, &lck_ctx);

	return err;
}

/*
 * Return:
 * > 0		Tx fifo number of items
 * 0		if nothing done in Tx fifo.
 * < 0		An error code:
 *		-EBADMSG	if sending CAN FD msg on CAN 2.0 settings
 *				if sending extended id while std msg allowed
 *		-ENODEV		if device no more plugged
 *		-ENETDOWN	if bus off
 *		-EAGAIN		if Tx fifo full
 *		-EINTR		if wait() has been interrupted
 */
static int pcanfd_send_msg(struct pcandev *dev, struct pcanfd_txmsg *ptx,
			   struct pcan_udata *ctx)
{
	FIFO_MANAGER *tx_fifo = &dev->writeFifo;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(type=%d) is_deferred_msg=%u\n",
		__func__, ptx->msg.type, is_deferred_msg);
#endif

	switch (ptx->msg.type) {

	case PCANFD_TYPE_CANFD_MSG:

		/* accept such messages for devices that have been initialized
		 * for CAN-FD. */
		if ((dev->init_settings.flags & PCANFD_INIT_FD) &&
				(ptx->msg.data_len <= PCANFD_MAXDATALEN))
			break;

		/* Ok to be permissive *BUT* force the message to be
		 * CAN 2.0 only, that is, CAN-FD specific flags won't be
		 * taken into account next. */
		ptx->msg.type = PCANFD_TYPE_CAN20_MSG;

		/* fall through */
	case PCANFD_TYPE_CAN20_MSG:
		if (ptx->msg.data_len <= PCAN_MAXDATALEN)
			break;

		/* fall through */
	default:
		pr_err(DEVICE_NAME
			": trying to send invalid msg (type=%xh len=%d)\n",
			ptx->msg.type, ptx->msg.data_len);

		return -EBADMSG;
	}

	/* filter extended data if initialized to standard only
	 * SGr note: no need to wait for doing such a test... */
	if ((dev->init_settings.flags & PCANFD_INIT_STD_MSG_ONLY)
	   && ((ptx->msg.flags & PCANFD_MSG_EXT) || (ptx->msg.id > 2047))) {

		pr_err(DEVICE_NAME
			": trying to send ext msg %xh while not setup for\n",
			ptx->msg.id);
		return -EBADMSG;
	}

	do {
		/* if the device has been plugged out while waiting,
		 * or if any task is closing it */
		if (!dev->is_plugged || !dev->nOpenPaths) {
			err = -ENODEV;
			break;
		}

		/* no need to write in case of BUS_OFF */
		if (dev->bus_state == PCANFD_ERROR_BUSOFF) {
			err = -ENETDOWN;
			break;
		}

#if 1
		/* useless */
#else
		/* get the time when msg is queued */
		pcan_gettimeofday(&ptx->tv);
#endif
		/* put data into fifo */
		err = pcan_fifo_put(tx_fifo, ptx);
		if (err > 0) {

			/* if FIFO was full, build a STATUS msg to clear */
			pcan_clear_status_bit(dev, CAN_ERR_XMTFULL);
#ifdef DEBUG
			pr_info(DEVICE_NAME
				": %s(%u): still %u free items in Tx queue\n",
				__func__, __LINE__,
				tx_fifo->nCount - tx_fifo->nStored);
#endif
#ifdef pcan_event_clear
			/* if tx fifo is now full, then the corresponding event
			 * should be cleared now.
			 * This section MUST be atomic regarding the tx fifo.
			 * Note: if (ctx->open_flags & O_NONBLOCK) { ? */
			{
				pcan_lock_irqsave_ctxt flags;

				pcan_lock_get_irqsave(&tx_fifo->lock, flags);

				if (pcan_fifo_full(tx_fifo))
					pcan_event_clear(&dev->out_event);

				pcan_lock_put_irqrestore(&tx_fifo->lock, flags);
			}
#endif
			//err = 0;
			break;
		}

		/* support nonblocking write if requested */
		if (!ctx || (ctx->open_flags & O_NONBLOCK)) {
			err = -EAGAIN;
			break;
		}

		if (!pcan_task_can_wait()) {
			pr_info(DEVICE_NAME
				": %s(%u): ABNORMAL task unable to wait!\n",
				__func__, __LINE__);
			err = -EAGAIN;
			break;
		}

		/* do it here, AFTER above both break; since err=-EAGAIN will
		 * generate an internal error that will set this bit too */
		pcan_set_status_bit(dev, CAN_ERR_XMTFULL);

		/* check Tx engine whether it is running before going asleep
		 * (Note: useful only if one has sent more msgs than Tx fifo
		 * size, at once) */
		pcanfd_start_tx_engine(dev, ctx);

		/* sleep until space is available. */
#ifdef DEBUG_WAIT_WR
		pr_info(DEVICE_NAME
			": %s CAN%u waiting %u ms. for some free space "
			"to write...\n",
			dev->adapter->name, dev->can_idx+1,
			PCANFD_TIMEOUT_WAIT_FOR_WR);
#endif

		/* task might go to sleep: unlock current dev */
		pcan_mutex_unlock(&dev->mutex);

		/* wait up to 100 ms. for some room in the Tx queue.
		 *
		 * some logs:
		 *
[ 7977.396005] pcan: pcanfd_send_msg(359): waiting for some free space to write...
...
[ 7977.400974] pcan: CAN1 lnk=1 signaling writing task...
...
[ 7977.400977] pcan: end of waiting for tx fifo not full: err=0
		 *
		 * Note: ^C may occur while waiting. In RT, preemption can 
		 * schedule another task that might call close() while we're
		 * always waiting here.
		 * - If the event is destroyed by some other task, the below
		 *   call fails with err=-EIDRM(43).
		 * - if some other task deletes this waiting task, this tasks
		 *   is first unblocked, thus err=-EINTR(4). */
		err = pcan_event_wait_timeout(dev->out_event,
					!dev->is_plugged ||
					!pcan_fifo_full(tx_fifo) ||
					dev->bus_state == PCANFD_ERROR_BUSOFF,
					PCANFD_TIMEOUT_WAIT_FOR_WR);

		/* lock the device again */
		pcan_mutex_lock(&dev->mutex);

#ifdef DEBUG_WAIT_WR
		pr_info(DEVICE_NAME
			": end of waiting for tx fifo not full: err=%d\n",
			err);
#endif

	} while (err >= 0);

	switch (err) {

	case -ERESTARTSYS:
		err = -EINTR;
		break;

	case -EAGAIN:

		/* remember the status: if FIFO was not full, build a STATUS
		 * msg and put it into the Rx FIFO. Note that this STATUS msg
		 * is put in Rx FIFO *ONLY* when in non-blocking mode
		 * (err == -EAGAIN). */
		if (!(dev->wCANStatus & CAN_ERR_XMTFULL)) {
			struct pcanfd_rxmsg f;

			pcan_handle_error_internal(dev, &f, PCANFD_TX_OVERFLOW);
#ifndef NETDEV_SUPPORT
			if (pcan_chardev_rx(dev, &f) > 0)
				pcan_event_signal(&dev->in_event);
#endif
		}

		/* fall through */
	default:
		break;
	}

	return err;
}

int pcanfd_ioctl_send_msg(struct pcandev *dev, struct pcanfd_txmsg *ptx,
			  struct pcan_udata *ctx)
{
	int err = pcanfd_send_msg(dev, ptx, ctx);

	/* start Tx engine only if Tx fifo is not empty */
	if (err > 0)
		err = pcanfd_start_tx_engine(dev, ctx);

	return err >= 0 ? 0 : err;
}

int pcanfd_ioctl_send_msgs(struct pcandev *dev, struct pcanfd_txmsgs *pl,
			   struct pcan_udata *ctx)
{
	struct pcanfd_txmsg *ptx;
	int err = 0, msgs_queued = 0, n = pl->count;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u)\n", __func__, n);
#endif

	ptx = pl->list;
	for (pl->count = 0; pl->count < n; pl->count++) {
		err = pcanfd_send_msg(dev, ptx, ctx);

		/* don't stop sending if error is related to the msg only */
		if (err < 0) {
			if (err != -EBADMSG)
				break;
		} else {
			msgs_queued += err;
		}

		ptx++;
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(count=%u): queued %u msgs\n",
		__func__, n, msgs_queued);
#endif

	/* if at least ONE message has been enqueued */
	//if (pl->count) {
	if (msgs_queued > 0) {

		/* if we just put the 1st message (=the fifo was empty),
		 * we can start writing on hardware if it is ready for doing
		 * this. */
		err = pcanfd_start_tx_engine(dev, ctx);
	}

	//return err;
	return msgs_queued ? 0 : err;
}

int pcanfd_ioctl_send_msgs_nolock(struct pcandev *dev, struct pcanfd_txmsgs *pl,
				  struct pcan_udata *ctx)
{
	struct pcanfd_txmsg *ptx;
	int err = 0, msgs_queued = 0, n = pl->count;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u)\n", __func__, n);
#endif

	ptx = pl->list;
	for (pl->count = 0; pl->count < n; pl->count++) {
		err = pcanfd_send_msg(dev, ptx, ctx);

		/* don't stop sending if error is related to the msg only */
		if (err < 0) {
			if (err != -EBADMSG)
				break;
		} else {
			msgs_queued += err;
		}

		ptx++;
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(count=%u): sent %u msgs\n",
			__func__, n, pl->count);
#endif

	/* if at least ONE message has been enqueued */
	if (err > 0) {


		/* if we just put the 1st message (=the fifo was empty),
		 * we can start writing on hardware if it is ready for doing
		 * this. */
		err = __pcan_dev_start_writing(dev, ctx);
	}

	//return err;
	return msgs_queued ? 0 : err;
}

int pcanfd_ioctl_recv_msg(struct pcandev *dev, struct pcanfd_rxmsg *prx,
			  struct pcan_udata *ctx)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	err = pcanfd_recv_msg(dev, prx, ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): returns %d\n", __func__, err);
#endif
	return err;
}

int pcanfd_ioctl_recv_msgs(struct pcandev *dev, struct pcanfd_rxmsgs *pl,
			   struct pcan_udata *ctx)
{
	struct pcanfd_rxmsg *prx;
	int err = 0, n = pl->count, saved_flags = ctx->open_flags;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u)\n", __func__, n);
#endif

	prx = pl->list;

	for (pl->count = 0; pl->count < n; pl->count++) {
		err = pcanfd_recv_msg(dev, prx, ctx);
		if (err)
			break;

		/* the task won't block anymore since at least one msg has been
		 * read. */
		ctx->open_flags |= O_NONBLOCK;
		prx++;
	}

	/* restore original flags asap */
	ctx->open_flags = saved_flags;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u): got %u msgs (err %d)\n",
			__func__, n, pl->count, err);
#endif
	return (pl->count > 0) ? 0 : err;
}
