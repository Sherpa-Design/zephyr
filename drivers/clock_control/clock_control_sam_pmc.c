/*
 * Copyright (c) 2023 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_pmc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

static int atmel_sam_clock_control_on(const struct device *dev,
				      clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const struct atmel_sam_pmc_config *cfg = (const struct atmel_sam_pmc_config *)sys;

	if (cfg == NULL) {
		LOG_ERR("The PMC config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("Type: %x, Id: %d", cfg->clock_type, cfg->peripheral_id);

	switch (cfg->clock_type) {
	case PMC_TYPE_PERIPHERAL:
		soc_pmc_peripheral_enable(cfg->peripheral_id);
		break;
	default:
		LOG_ERR("The PMC clock type is not implemented.");
		return -ENODEV;
	}

	return 0;
}

static int atmel_sam_clock_control_off(const struct device *dev,
				       clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const struct atmel_sam_pmc_config *cfg = (const struct atmel_sam_pmc_config *)sys;

	if (cfg == NULL) {
		LOG_ERR("The PMC config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("Type: %x, Id: %d", cfg->clock_type, cfg->peripheral_id);

	switch (cfg->clock_type) {
	case PMC_TYPE_PERIPHERAL:
		soc_pmc_peripheral_disable(cfg->peripheral_id);
		break;
	default:
		LOG_ERR("The PMC clock type is not implemented.");
		return -ENODEV;
	}

	return 0;
}

static int atmel_sam_clock_control_get_rate(const struct device *dev,
					    clock_control_subsys_t sys,
					    uint32_t *rate)
{
	ARG_UNUSED(dev);

	const struct atmel_sam_pmc_config *cfg = (const struct atmel_sam_pmc_config *)sys;

	if (cfg == NULL) {
		LOG_ERR("The PMC config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("Type: %x, Id: %d", cfg->clock_type, cfg->peripheral_id);

	switch (cfg->clock_type) {
	case PMC_TYPE_PERIPHERAL: {
		uint32_t mckr = PMC->PMC_MCKR;

		/* Decode PRES: encoded field → integer divisor */
		uint32_t pres;

		switch (mckr & PMC_MCKR_PRES_Msk) {
		case PMC_MCKR_PRES_CLK_2:  pres = 2U;  break;
		case PMC_MCKR_PRES_CLK_3:  pres = 3U;  break;
		case PMC_MCKR_PRES_CLK_4:  pres = 4U;  break;
		case PMC_MCKR_PRES_CLK_8:  pres = 8U;  break;
		case PMC_MCKR_PRES_CLK_16: pres = 16U; break;
		case PMC_MCKR_PRES_CLK_32: pres = 32U; break;
		case PMC_MCKR_PRES_CLK_64: pres = 64U; break;
		default:                   pres = 1U;  break; /* CLK_1: no division */
		}

		/* MDIV: SAMX7X only (MCK = HCLK / MDIV) */
		uint32_t mdiv = 1U;

#ifdef CONFIG_SOC_SERIES_SAMX7X
		switch (mckr & PMC_MCKR_MDIV_Msk) {
		case PMC_MCKR_MDIV_PCK_DIV2: mdiv = 2U; break;
		case PMC_MCKR_MDIV_PCK_DIV3: mdiv = 3U; break;
		case PMC_MCKR_MDIV_PCK_DIV4: mdiv = 4U; break;
		default: break; /* EQ_PCK → mdiv stays 1 */
		}
#endif

		/* MAINCK from PLLA Kconfig + DTS cpu0.clock-frequency.
		 * Assumes PRES/MDIV match their Kconfig values (true for SAMX7X
		 * where neither field changes after SOC init). */
		const uint32_t mainck_hz =
			(uint32_t)((uint64_t)SOC_ATMEL_SAM_HCLK_FREQ_HZ
				    * CONFIG_SOC_ATMEL_SAM_MCK_PRESCALLER
				    * CONFIG_SOC_ATMEL_SAM_PLLA_DIVA
				    / (CONFIG_SOC_ATMEL_SAM_PLLA_MULA + 1U));

		switch (mckr & PMC_MCKR_CSS_Msk) {
		case PMC_MCKR_CSS_PLLA_CLK: {
			uint32_t pllar = PMC->CKGR_PLLAR;
			uint32_t mula = (pllar & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos;
			uint32_t diva = (pllar & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos;
			uint32_t src_hz = (diva > 0U)
				? (uint32_t)((uint64_t)mainck_hz * (mula + 1U) / diva)
				: mainck_hz;
			*rate = src_hz / pres / mdiv;
			break;
		}
		case PMC_MCKR_CSS_MAIN_CLK:
			*rate = mainck_hz / pres / mdiv;
			break;
		default:
			/* SLOW, UPLL: not decoded; return compile-time MCK */
			*rate = SOC_ATMEL_SAM_MCK_FREQ_HZ;
			break;
		}
		break;
	}
	default:
		LOG_ERR("The PMC clock type is not implemented.");
		return -ENODEV;
	}

	LOG_DBG("Rate: %d", *rate);

	return 0;
}

static enum clock_control_status
atmel_sam_clock_control_get_status(const struct device *dev,
				   clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);

	const struct atmel_sam_pmc_config *cfg = (const struct atmel_sam_pmc_config *)sys;
	enum clock_control_status status;

	if (cfg == NULL) {
		LOG_ERR("The PMC config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("Type: %x, Id: %d", cfg->clock_type, cfg->peripheral_id);

	switch (cfg->clock_type) {
	case PMC_TYPE_PERIPHERAL:
		status = soc_pmc_peripheral_is_enabled(cfg->peripheral_id) > 0
		       ? CLOCK_CONTROL_STATUS_ON
		       : CLOCK_CONTROL_STATUS_OFF;
		break;
	default:
		LOG_ERR("The PMC clock type is not implemented.");
		return -ENODEV;
	}

	return status;
}

static DEVICE_API(clock_control, atmel_sam_clock_control_api) = {
	.on = atmel_sam_clock_control_on,
	.off = atmel_sam_clock_control_off,
	.get_rate = atmel_sam_clock_control_get_rate,
	.get_status = atmel_sam_clock_control_get_status,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &atmel_sam_clock_control_api);
