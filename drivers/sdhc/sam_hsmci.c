/*
 * Copyright 2023 Nikhef
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_hsmci

#include <zephyr/sd/sd_spec.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#ifdef CONFIG_SAM_HSMCI_XDMAC
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>
#endif

/* S-01b: per-command-type counters for BCC throughput analysis */
static atomic_t s_cmd17_count;  /* SD_READ_SINGLE_BLOCK  */
static atomic_t s_cmd18_count;  /* SD_READ_MULTIPLE_BLOCK */
static atomic_t s_cmd24_count;  /* SD_WRITE_SINGLE_BLOCK  */
static atomic_t s_cmd25_count;  /* SD_WRITE_MULTIPLE_BLOCK */

void sam_hsmci_get_cmd_counts(uint32_t *c17, uint32_t *c18,
			      uint32_t *c24, uint32_t *c25)
{
	*c17 = (uint32_t)atomic_get(&s_cmd17_count);
	*c18 = (uint32_t)atomic_get(&s_cmd18_count);
	*c24 = (uint32_t)atomic_get(&s_cmd24_count);
	*c25 = (uint32_t)atomic_get(&s_cmd25_count);
}

LOG_MODULE_REGISTER(hsmci, CONFIG_SDHC_LOG_LEVEL);

#ifdef HSMCI_MR_PDCMODE
#ifdef CONFIG_SAM_HSMCI_PDCMODE
#define _HSMCI_PDCMODE
#endif
#endif

#ifdef CONFIG_SAM_HSMCI_PWRSAVE
#if (CONFIG_SAM_HSMCI_PWRSAVE_DIV < 0) || (CONFIG_SAM_HSMCI_PWRSAVE_DIV > 7)
#error "CONFIG_SAM_HSMCI_PWRSAVE_DIV must be 0 to 7"
#endif
#endif

#define _HSMCI_DEFAULT_TIMEOUT	5000
#define _HSMCI_MAX_FREQ			(SOC_ATMEL_SAM_MCK_FREQ_HZ >> 1)
#define _HSMCI_MIN_FREQ			(_HSMCI_MAX_FREQ / 0x200)
#define _MSMCI_MAX_DIVISOR		0x1FF
#define _HSMCI_SR_ERR			 (HSMCI_SR_RINDE \
								| HSMCI_SR_RDIRE \
								| HSMCI_SR_RCRCE \
								| HSMCI_SR_RENDE \
								| HSMCI_SR_RTOE \
								| HSMCI_SR_DCRCE \
								| HSMCI_SR_DTOE \
								| HSMCI_SR_CSTOE \
								| HSMCI_SR_OVRE \
								| HSMCI_SR_UNRE)

static const uint8_t _resp2size[] = {
	[SD_RSP_TYPE_NONE] = HSMCI_CMDR_RSPTYP_NORESP,
	[SD_RSP_TYPE_R1]   = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R1b]  = HSMCI_CMDR_RSPTYP_R1B,
	[SD_RSP_TYPE_R2]   = HSMCI_CMDR_RSPTYP_136_BIT,
	[SD_RSP_TYPE_R3]   = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R4]   = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R5]   = 0 /* SDIO not supported */,
	[SD_RSP_TYPE_R5b]  = 0 /* SDIO not supported */,
	[SD_RSP_TYPE_R6]   = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R7]   = HSMCI_CMDR_RSPTYP_48_BIT,
};

/* timeout multiplier shift (actual value is 1 << _mul_shift[*]) */
static const uint8_t _mul_shift[] = {0, 4, 7, 8, 10, 12, 16, 20};
static const uint8_t _mul_shift_size = 8;

struct sam_hsmci_config {
	Hsmci *base;
	const struct atmel_sam_pmc_config clock_cfg;
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec carrier_detect;
#ifdef CONFIG_SAM_HSMCI_XDMAC
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_perid;
#endif
};

struct sam_hsmci_data {
	bool open_drain;
	uint8_t cmd_in_progress;
	struct k_mutex mtx;
	/* S-01b BM: when true, bypass DMA and use CPU-polled transfer */
	bool force_manual;
#ifdef CONFIG_SAM_HSMCI_XDMAC
	struct k_sem xfer_done;
	int xfer_status;
#endif
};

void sam_hsmci_set_force_manual(bool enable)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(hsmci));
	struct sam_hsmci_data *d = dev->data;

	d->force_manual = enable;
}

static int sam_hsmci_reset(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;
	Hsmci *hsmci = config->base;

	uint32_t mr = hsmci->HSMCI_MR;
	uint32_t dtor = hsmci->HSMCI_DTOR;
	uint32_t sdcr = hsmci->HSMCI_SDCR;
	uint32_t cstor = hsmci->HSMCI_CSTOR;
	uint32_t cfg = hsmci->HSMCI_CFG;

	hsmci->HSMCI_CR = HSMCI_CR_SWRST;
	hsmci->HSMCI_MR = mr;
	hsmci->HSMCI_DTOR = dtor;
	hsmci->HSMCI_SDCR = sdcr;
	hsmci->HSMCI_CSTOR = cstor;
	hsmci->HSMCI_CFG = cfg;

	hsmci->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIEN;
	return 0;
}

static int sam_hsmci_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	memset(props, 0, sizeof(*props));

	props->f_max = _HSMCI_MAX_FREQ;
	props->f_min = _HSMCI_MIN_FREQ;
	/* high-speed not working yet due to limitations of the SDHC sm */
	props->host_caps.high_spd_support = false;
	props->host_caps.vol_330_support = true;
	props->power_delay = 500;
	props->is_spi = false;
	props->max_current_330 = 4;

	return 0;
}

static int sam_hsmci_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sam_hsmci_config *config = dev->config;
	struct sam_hsmci_data *data = dev->data;
	Hsmci *hsmci = config->base;
	uint32_t frequency;
	uint32_t div_val;
	int ret;

	LOG_DBG("%s(clock=%d, bus_width=%d, timing=%d, mode=%d)", __func__, ios->clock,
		ios->bus_width, ios->timing, ios->bus_mode);

	if (ios->clock > 0) {
		if (ios->clock > _HSMCI_MAX_FREQ) {
			return -ENOTSUP;
		}

		ret = clock_control_get_rate(SAM_DT_PMC_CONTROLLER,
					     (clock_control_subsys_t)&config->clock_cfg,
					     &frequency);

		if (ret < 0) {
			LOG_ERR("Failed to get clock rate, err=%d", ret);
			return ret;
		}

		div_val = frequency / ios->clock - 2;

		if (div_val < 0) {
			div_val = 0;
		}

		if (div_val > _MSMCI_MAX_DIVISOR) {
			div_val = _MSMCI_MAX_DIVISOR;
		}

		LOG_DBG("divider: %d (freq=%d)", div_val, frequency / (div_val + 2));

		hsmci->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
		hsmci->HSMCI_MR |=
			((div_val & 1) ? HSMCI_MR_CLKODD : 0) | HSMCI_MR_CLKDIV(div_val >> 1);
	}

	if (ios->bus_width)	{
		hsmci->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;

		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			hsmci->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_1;
			break;
		case SDHC_BUS_WIDTH4BIT:
			hsmci->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_4;
			break;
		default:
			return -ENOTSUP;
		}
	}

	data->open_drain = (ios->bus_mode == SDHC_BUSMODE_OPENDRAIN);

	if (ios->timing) {
		switch (ios->timing) {
		case SDHC_TIMING_LEGACY:
			hsmci->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;
			break;
		case SDHC_TIMING_HS:
			hsmci->HSMCI_CFG |= HSMCI_CFG_HSMODE;
			break;
		default:
			return -ENOTSUP;
		}
	}

	return 0;
}

static int sam_hsmci_init(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;
	int ret;

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state() => %d", ret);
		return ret;
	}
	/* Enable module's clock */
	(void)clock_control_on(SAM_DT_PMC_CONTROLLER, (clock_control_subsys_t)&config->clock_cfg);

#ifdef CONFIG_SAM_HSMCI_XDMAC
	{
		struct sam_hsmci_data *d = dev->data;

		k_sem_init(&d->xfer_done, 0, 1);
		if (!device_is_ready(config->dma_dev)) {
			LOG_ERR("XDMAC device not ready");
			return -ENODEV;
		}
	}
#endif

	/* init carrier detect (if set) */
	if (config->carrier_detect.port != NULL) {
		if (!gpio_is_ready_dt(&config->carrier_detect)) {
			LOG_ERR("GPIO port for carrier-detect pin is not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->carrier_detect, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Couldn't configure carrier-detect pin; (%d)", ret);
			return ret;
		}
	}

	Hsmci *hsmci = config->base;

	/* reset the device */
	hsmci->HSMCI_CR = HSMCI_CR_SWRST;
	hsmci->HSMCI_CR = HSMCI_CR_PWSDIS;
	hsmci->HSMCI_CR = HSMCI_CR_MCIEN;
#ifdef CONFIG_SAM_HSMCI_PWRSAVE
	hsmci->HSMCI_MR =
		HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF | HSMCI_MR_PWSDIV(CONFIG_SAM_HSMCI_PWRSAVE_DIV);
	hsmci->HSMCI_CR = HSMCI_CR_PWSEN;
#else
	hsmci->HSMCI_MR = HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF;
#endif

	return 0;
}

static int sam_hsmci_get_card_present(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;

	if (config->carrier_detect.port == NULL) {
		return 1;
	}

	return gpio_pin_get_dt(&config->carrier_detect);
}

static int sam_hsmci_card_busy(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;
	Hsmci *hsmci = config->base;

	return (hsmci->HSMCI_SR & HSMCI_SR_NOTBUSY) == 0;
}

static void sam_hsmci_send_clocks(Hsmci *hsmci)
{
	hsmci->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
	hsmci->HSMCI_ARGR = 0;
	hsmci->HSMCI_CMDR =
		HSMCI_CMDR_RSPTYP_NORESP | HSMCI_CMDR_SPCMD_INIT | HSMCI_CMDR_OPDCMD_OPENDRAIN;
	while (!(hsmci->HSMCI_SR & HSMCI_SR_CMDRDY)) {
		;
	}
	hsmci->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
}

static int sam_hsmci_send_cmd(Hsmci *hsmci, struct sdhc_command *cmd, uint32_t cmdr,
			      struct sam_hsmci_data *data)
{
	uint32_t sr;

	hsmci->HSMCI_ARGR = cmd->arg;

	cmdr |= HSMCI_CMDR_CMDNB(cmd->opcode) | HSMCI_CMDR_MAXLAT_64;
	if (data->open_drain) {
		cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
	}

	uint8_t nrt = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (nrt > SD_RSP_TYPE_R7) {
		return -ENOTSUP;
	}

	cmdr |= _resp2size[nrt];
	hsmci->HSMCI_CMDR = cmdr;
	do {
		sr = hsmci->HSMCI_SR;

		/* special case ,ignore CRC status if response is R3 to clear it */
		if (nrt == SD_RSP_TYPE_R3 || nrt == SD_RSP_TYPE_NONE) {
			sr &= ~HSMCI_SR_RCRCE;
		}

		if ((sr & _HSMCI_SR_ERR) != 0) {
			LOG_DBG("Status register error bits: %08x", sr & _HSMCI_SR_ERR);
			if (sr & HSMCI_SR_RTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_CMDRDY));

	if (nrt == SD_RSP_TYPE_R1b) {
		do {
			sr = hsmci->HSMCI_SR;
		} while (!((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0)));
	}

	/* RSPR is just a FIFO, index is of no consequence */
	cmd->response[3] = hsmci->HSMCI_RSPR[0];
	cmd->response[2] = hsmci->HSMCI_RSPR[0];
	cmd->response[1] = hsmci->HSMCI_RSPR[0];
	cmd->response[0] = hsmci->HSMCI_RSPR[0];
	return 0;
}

static int sam_hsmci_wait_write_end(Hsmci *hsmci)
{
	uint32_t sr = 0;

#ifdef _HSMCI_PDCMODE
	/* Timeout is included in HSMCI, see DTOE bit, not required explicitly. */
	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x error", sr);
			if (sr & HSMCI_SR_DTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_TXBUFE));
#endif

	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x last transfer error", sr);
			if (sr & HSMCI_SR_DTOE) {
					LOG_WRN("DTOE SR=0x%08x NOTBUSY=%d",
						sr, (sr & HSMCI_SR_NOTBUSY) ? 1 : 0);
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_NOTBUSY));

	if (!(hsmci->HSMCI_SR & HSMCI_SR_FIFOEMPTY)) {
		return -EIO;
	}
	return 0;
}

static int sam_hsmci_wait_read_end(Hsmci *hsmci)
{
	uint32_t sr;

#ifdef _HSMCI_PDCMODE
	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x error", sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE |
							     HSMCI_SR_DTOE | HSMCI_SR_DCRCE));
			if (sr & HSMCI_SR_DTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_RXBUFF));
#endif

	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			if (sr & HSMCI_SR_DTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_XFRDONE));
	return 0;
}

static int sam_hsmci_write_timeout(Hsmci *hsmci, int timeout_ms)
{
	/* convert to clocks (coarsely) */
	int clocks = ATMEL_SAM_DT_CPU_CLK_FREQ_HZ / 1000 * timeout_ms;
	int mul, max_clock;

	for (int i = 0; i < _mul_shift_size; i++) {
		mul = 1 << _mul_shift[i];
		max_clock = 15 * mul;
		if (max_clock > clocks) {
			hsmci->HSMCI_DTOR = ((i << HSMCI_DTOR_DTOMUL_Pos) & HSMCI_DTOR_DTOMUL_Msk) |
					    HSMCI_DTOR_DTOCYC((clocks + mul - 1) / mul);
			return 0;
		}
	}
	/*
	 * So, if it is > maximum timeout... we'll just put it on the maximum the driver supports
	 * its not nice.. but it should work.. what else is there to do?
	 */
	hsmci->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_Msk | HSMCI_DTOR_DTOCYC_Msk;
	return 0;
}

static inline int wait_write_transfer_done(Hsmci *hsmci)
{
	int sr;

	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			if (sr & HSMCI_SR_DTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_TXRDY));
	return 0;
}

static inline int wait_read_transfer_done(Hsmci *hsmci)
{
	int sr;

	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			if (sr & HSMCI_SR_DTOE) {
					return -ETIMEDOUT;
			}
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_RXRDY));
	return 0;
}

#ifdef CONFIG_SAM_HSMCI_XDMAC
static void hsmci_dma_done(const struct device *dma_dev, void *user_data,
			   uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct sam_hsmci_data *d = dev->data;

	d->xfer_status = status;
	k_sem_give(&d->xfer_done);
}
#endif /* CONFIG_SAM_HSMCI_XDMAC */

#ifndef _HSMCI_PDCMODE

static int hsmci_do_manual_transfer(Hsmci *hsmci, bool byte_mode, bool is_write, void *data,
				    int transfer_count)
{
	int ret;

	if (is_write) {
		if (byte_mode) {
			const uint8_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = wait_write_transfer_done(hsmci);
				if (ret != 0) {
					return ret;
				}
				hsmci->HSMCI_TDR = *ptr;
				ptr++;
			}
		} else {
			const uint32_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = wait_write_transfer_done(hsmci);
				if (ret != 0) {
					return ret;
				}
				hsmci->HSMCI_TDR = *ptr;
				ptr++;
			}
		}
		ret = sam_hsmci_wait_write_end(hsmci);
	} else {
		if (byte_mode) {
			uint8_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = wait_read_transfer_done(hsmci);
				if (ret != 0) {
					return ret;
				}
				*ptr = hsmci->HSMCI_RDR;
				ptr++;
			}
		} else {
			uint32_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = wait_read_transfer_done(hsmci);
				if (ret != 0) {
					return ret;
				}
				*ptr = hsmci->HSMCI_RDR;
				ptr++;
			}
		}
		ret = sam_hsmci_wait_read_end(hsmci);
	}
	return ret;
}

#endif /* !_HSMCI_PDCMODE */

static int sam_hsmci_request_inner(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *sd_data)
{
	const struct sam_hsmci_config *config = dev->config;
	struct sam_hsmci_data *data = dev->data;
	Hsmci *hsmci = config->base;
	uint32_t sr;
	uint32_t size;
	uint32_t transfer_count;
	uint32_t cmdr = 0;
	int ret;
	bool is_write, byte_mode;
#ifdef CONFIG_SAM_HSMCI_XDMAC
	bool use_xdmac = false;
#endif

	LOG_DBG("%s(opcode=%d, arg=%08x, data=%08x, rsptype=%d)", __func__, cmd->opcode, cmd->arg,
		(uint32_t)sd_data, cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		/* send 74 clocks, as required by SD spec */
		sam_hsmci_send_clocks(hsmci);
	}

	if (sd_data) {
		cmdr |= HSMCI_CMDR_TRCMD_START_DATA;

		ret = sam_hsmci_write_timeout(hsmci, cmd->timeout_ms);
		if (ret != 0) {
			return ret;
		}

		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
			atomic_inc(&s_cmd24_count);
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			is_write = true;
			break;
		case SD_WRITE_MULTIPLE_BLOCK:
			atomic_inc(&s_cmd25_count);
			is_write = true;
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			break;
		case SD_APP_SEND_SCR:
		case SD_SWITCH:
		case SD_READ_SINGLE_BLOCK:
			atomic_inc(&s_cmd17_count);
			is_write = false;
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_READ_MULTIPLE_BLOCK:
			atomic_inc(&s_cmd18_count);
			is_write = false;
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_APP_SEND_NUM_WRITTEN_BLK:
			is_write = false;
			break;
		case MMC_SEND_EXT_CSD:
            is_write = false;
            cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
            cmdr |= HSMCI_CMDR_TRDIR_READ;
            break;			
		default:
			return -ENOTSUP;
		}

		if ((sd_data->block_size & 0x3) == 0 && (((uint32_t)sd_data->data) & 0x3) == 0) {
			size = (sd_data->block_size + 3) >> 2;
			hsmci->HSMCI_MR &= ~HSMCI_MR_FBYTE;
			byte_mode = false;   /* ← was true, word-aligned = word mode = uint32_t */
		} else {
			size = sd_data->block_size;
			hsmci->HSMCI_MR |= HSMCI_MR_FBYTE;
			byte_mode = true;    /* ← was false, unaligned = byte mode = uint8_t */
		}

		hsmci->HSMCI_BLKR =
			HSMCI_BLKR_BLKLEN(sd_data->block_size) | HSMCI_BLKR_BCNT(sd_data->blocks);

		transfer_count = size * sd_data->blocks;

#if defined(CONFIG_SAM_HSMCI_XDMAC) && !defined(_HSMCI_PDCMODE)
		/* Configure and start XDMAC before sending the command.
		 * HSMCI_DMA.DMAEN must also be set before CMDR is written —
		 * see SAMS70 datasheet errata (handshake starts at command issue). */
		if (!byte_mode && !data->force_manual) {
			uint32_t byte_count = transfer_count * 4U;
			uint32_t fifo_addr = (uint32_t)&hsmci->HSMCI_FIFO[0];
			struct dma_block_config blk = {
				.source_address  = is_write ? (uint32_t)sd_data->data : fifo_addr,
				.dest_address    = is_write ? fifo_addr : (uint32_t)sd_data->data,
				.block_size      = byte_count,
				.source_addr_adj = is_write ? DMA_ADDR_ADJ_INCREMENT
							    : DMA_ADDR_ADJ_NO_CHANGE,
				.dest_addr_adj   = is_write ? DMA_ADDR_ADJ_NO_CHANGE
							    : DMA_ADDR_ADJ_INCREMENT,
			};
			struct dma_config dma_cfg = {
				.dma_slot            = config->dma_perid,
				.channel_direction   = is_write ? MEMORY_TO_PERIPHERAL
							         : PERIPHERAL_TO_MEMORY,
				.source_data_size    = 4U,
				.dest_data_size      = 4U,
				/* burst=4 → find_msb_set(4)-1=2 → XDMAC CSIZE=2 = CHK_4
				 * matches HSMCI_DMA_CHKSIZE_4 (val=2, 4 data/request) */
				.source_burst_length = 4U,
				.dest_burst_length   = 4U,
				.block_count         = 1U,
				.head_block          = &blk,
				.dma_callback        = hsmci_dma_done,
				.user_data           = (void *)dev,
				.complete_callback_en = 1,
			};

			if (is_write) {
				sys_cache_data_flush_range(sd_data->data, byte_count);
			}

			k_sem_reset(&data->xfer_done);

			ret = dma_config(config->dma_dev, config->dma_channel, &dma_cfg);
			if (ret == 0) {
				ret = dma_start(config->dma_dev, config->dma_channel);
			}
			if (ret == 0) {
				hsmci->HSMCI_DMA = HSMCI_DMA_DMAEN | HSMCI_DMA_CHKSIZE_4;
				use_xdmac = true;
			} else {
				LOG_ERR("DMA setup failed (%d), falling back to manual", ret);
				ret = 0;
			}
		}
#endif /* CONFIG_SAM_HSMCI_XDMAC && !_HSMCI_PDCMODE */

#ifdef _HSMCI_PDCMODE
		/* S-01b BM: skip PDC when force_manual is set */
		if (!data->force_manual) {
			hsmci->HSMCI_MR |= HSMCI_MR_PDCMODE;

			hsmci->HSMCI_RNCR = 0;

			if (is_write) {
				hsmci->HSMCI_TCR = transfer_count;
				hsmci->HSMCI_TPR = (uint32_t)sd_data->data;
			} else {
				hsmci->HSMCI_RCR = transfer_count;
				hsmci->HSMCI_RPR = (uint32_t)sd_data->data;
				hsmci->HSMCI_PTCR = HSMCI_PTCR_RXTEN;
			}
		} else {
			hsmci->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
		}

	} else {
		hsmci->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
#endif /* _HSMCI_PDCMODE */
	}

	ret = sam_hsmci_send_cmd(hsmci, cmd, cmdr, data);

	if (sd_data) {
#ifdef _HSMCI_PDCMODE
		if (!data->force_manual) {
			if (ret == 0) {
				if (is_write) {
					hsmci->HSMCI_PTCR = HSMCI_PTCR_TXTEN;
					ret = sam_hsmci_wait_write_end(hsmci);
				} else {
					ret = sam_hsmci_wait_read_end(hsmci);
				}
			}
			hsmci->HSMCI_PTCR = HSMCI_PTCR_TXTDIS | HSMCI_PTCR_RXTDIS;
			hsmci->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
		} else {
			/* S-01b BM: manual path for DMA-vs-CPU comparison */
			if (ret == 0) {
				ret = hsmci_do_manual_transfer(hsmci, byte_mode, is_write,
							       sd_data->data, transfer_count);
			}
		}
#else  /* !_HSMCI_PDCMODE */
#ifdef CONFIG_SAM_HSMCI_XDMAC
		if (use_xdmac) {
			if (ret == 0) {
				int wait_ret = k_sem_take(&data->xfer_done,
							  K_MSEC(cmd->timeout_ms));

				if (wait_ret != 0) {
					dma_stop(config->dma_dev, config->dma_channel);
					hsmci->HSMCI_DMA = 0;
					ret = -ETIMEDOUT;
				} else if (data->xfer_status != 0) {
					hsmci->HSMCI_DMA = 0;
					ret = -EIO;
				} else {
					if (!is_write) {
						sys_cache_data_invd_range(
							sd_data->data,
							transfer_count * 4U);
						ret = sam_hsmci_wait_read_end(hsmci);
					} else {
						ret = sam_hsmci_wait_write_end(hsmci);
					}
					hsmci->HSMCI_DMA = 0;
				}
			} else {
				dma_stop(config->dma_dev, config->dma_channel);
				hsmci->HSMCI_DMA = 0;
			}
		} else {
			if (ret == 0) {
				ret = hsmci_do_manual_transfer(hsmci, byte_mode, is_write,
							       sd_data->data, transfer_count);
			}
		}
#else  /* !CONFIG_SAM_HSMCI_XDMAC */
		if (ret == 0) {
			ret = hsmci_do_manual_transfer(hsmci, byte_mode, is_write, sd_data->data,
						       transfer_count);
		}
#endif /* CONFIG_SAM_HSMCI_XDMAC */
#endif /* _HSMCI_PDCMODE */
	}

	sr = hsmci->HSMCI_SR;

	LOG_DBG("RSP0=%08x, RPS1=%08x, RPS2=%08x,RSP3=%08x, SR=%08x", cmd->response[0],
		cmd->response[1], cmd->response[2], cmd->response[3], sr);

	return ret;
}

static void sam_hsmci_abort(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;
	Hsmci *hsmci = config->base;

#ifdef _HSMCI_PDCMODE
	hsmci->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;
#endif /* _HSMCI_PDCMODE */

#ifdef CONFIG_SAM_HSMCI_XDMAC
	dma_stop(config->dma_dev, config->dma_channel);
	hsmci->HSMCI_DMA = 0;
#endif

	struct sdhc_command cmd = {
		.opcode = SD_STOP_TRANSMISSION, .arg = 0, .response_type = SD_RSP_TYPE_NONE};
	sam_hsmci_request_inner(dev, &cmd, NULL);
}

static int sam_hsmci_request(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *sd_data)
{
	struct sam_hsmci_data *dev_data = dev->data;
	int ret;

	ret = k_mutex_lock(&dev_data->mtx, K_MSEC(cmd->timeout_ms));
	if (ret) {
		LOG_ERR("Could not access card");
		return -EBUSY;
	}

#ifdef CONFIG_SAM_HSMCI_PWRSAVE
	const struct sam_hsmci_config *config = dev->config;
	Hsmci *hsmci = config->base;

	hsmci->HSMCI_CR = HSMCI_CR_PWSDIS;
#endif /* CONFIG_SAM_HSMCI_PWRSAVE */

	do {
		ret = sam_hsmci_request_inner(dev, cmd, sd_data);
		if (sd_data && (ret || sd_data->blocks > 1)) {
			sam_hsmci_abort(dev);
			/* Post-CMD12 busy wait. Erase-boundary writes can hold DAT0 low
			 * for several seconds on some eMMC chips — longer than the
			 * hardware DTOR limit (~629ms at 25 MHz). Use wall-clock timeout
			 * with msleep so the CPU is not monopolised. Minimum 30 s floor
			 * covers worst-case erase latencies. */
			int busy_ms = cmd->timeout_ms > 30000 ? cmd->timeout_ms : 30000;
			int64_t t0 = k_uptime_get();
			int64_t deadline = t0 + busy_ms;
			int64_t log_next = t0 + 1000;
			bool card_idle = false;
			const struct sam_hsmci_config *cfg = dev->config;

			while (k_uptime_get() < deadline) {
				if (!sam_hsmci_card_busy(dev)) {
					card_idle = true;
					break;
				}
				if (k_uptime_get() >= log_next) {
					uint32_t sr_now = cfg->base->HSMCI_SR;
					LOG_WRN("busy wait +%u ms SR=0x%08x NOTBUSY=%d",
						(uint32_t)(k_uptime_get() - t0),
						sr_now,
						(sr_now & HSMCI_SR_NOTBUSY) ? 1 : 0);
					log_next += 5000;
				}
				k_msleep(1);
			}
			if (!card_idle) {
				LOG_ERR("Card did not idle after CMD12");
				ret = -ETIMEDOUT;
			} else if (ret == -ETIMEDOUT) {
				/* DTOE fired during post-transfer busy wait; all data was
				 * transferred to the card before the hardware timeout, so
				 * the write completed successfully once the card went idle. */
				ret = 0;
			}
		}
	} while (ret != 0 && (cmd->retries-- > 0));

#ifdef CONFIG_SAM_HSMCI_PWRSAVE
	hsmci->HSMCI_CR = HSMCI_CR_PWSEN;
#endif /* CONFIG_SAM_HSMCI_PWRSAVE */

	k_mutex_unlock(&dev_data->mtx);

	return ret;
}

static DEVICE_API(sdhc, hsmci_api) = {
	.reset = sam_hsmci_reset,
	.get_host_props = sam_hsmci_get_host_props,
	.set_io = sam_hsmci_set_io,
	.get_card_present = sam_hsmci_get_card_present,
	.request = sam_hsmci_request,
	.card_busy = sam_hsmci_card_busy,
};

#ifdef CONFIG_SAM_HSMCI_XDMAC
#define SAM_HSMCI_DMA_CFG(N)                                                                       \
	.dma_dev     = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(N, rxtx)),                         \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(N, rxtx, channel),                               \
	.dma_perid   = DT_INST_DMAS_CELL_BY_NAME(N, rxtx, perid),
#else
#define SAM_HSMCI_DMA_CFG(N)
#endif

#define SAM_HSMCI_INIT(N)                                                                          \
	PINCTRL_DT_INST_DEFINE(N);                                                                 \
	static const struct sam_hsmci_config hsmci_##N##_config = {                                \
		.base = (Hsmci *)DT_INST_REG_ADDR(N),                                              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(N),                                       \
		.clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(N),                                         \
		.carrier_detect = GPIO_DT_SPEC_INST_GET_OR(N, cd_gpios, {0}),                      \
		SAM_HSMCI_DMA_CFG(N)                                                                \
	};                                                                                         \
	static struct sam_hsmci_data hsmci_##N##_data = {};                                        \
	DEVICE_DT_INST_DEFINE(N, &sam_hsmci_init, NULL, &hsmci_##N##_data, &hsmci_##N##_config,    \
			      POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, &hsmci_api);

DT_INST_FOREACH_STATUS_OKAY(SAM_HSMCI_INIT)
