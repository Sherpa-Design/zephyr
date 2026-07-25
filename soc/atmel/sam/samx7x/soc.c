/*
 * Copyright (c) 2016 Piotr Mienkowski
 * Copyright (c) 2019-2024 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Atmel SAM E70/S70/V70/V71 MCU series initialization code
 *
 * This file provides routines to initialize and support board-level hardware
 * for the Atmel SAM E70/S70/V70/V71 MCU series.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/arch/cache.h>
#include <soc.h>
#include <soc_pmc.h>
#include <soc_supc.h>
#include <cmsis_core.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

/**
 * @brief Setup various clocks on SoC at boot time.
 *
 * Setup Slow, Main, PLLA, Processor and Master clocks during the device boot.
 * It is assumed that the relevant registers are at their reset value.
 */
static ALWAYS_INLINE void clock_init(void)
{
	/*
	 * MCUboot runs this same clock_init() from POR state (4 MHz RC → 150 MHz
	 * PLLA). When the app starts after MCUboot, clocks are already at 150 MHz
	 * PLLA and the I-cache is about to be enabled (done before this call in
	 * soc_reset_hook). Re-running the sequence would drop MCK from 150 MHz
	 * back to 12 MHz RC with the I-cache live — the downward PMC_MCKR
	 * transition glitches the instruction fetch and corrupts the PC.
	 * Detect "already at PLLA" and return immediately; MCUboot has already
	 * set EFC wait states, SUPC crystal, and PLLA to the same target values.
	 */
	if ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK) {
		return;
	}

	/* Switch the main clock to the internal OSC with 12MHz */
	soc_pmc_switch_mainck_to_fastrc(SOC_PMC_FAST_RC_FREQ_12MHZ);

	/* Switch MCK (Master Clock) to the main clock */
	soc_pmc_mck_set_source(SOC_PMC_MCK_SRC_MAIN_CLK);

	EFC->EEFC_FMR = EEFC_FMR_FWS(0) | EEFC_FMR_CLOE;

	soc_pmc_enable_clock_failure_detector();

	if (IS_ENABLED(CONFIG_SOC_ATMEL_SAM_EXT_SLCK)) {
		soc_supc_slow_clock_select_crystal_osc();
	}

	if (IS_ENABLED(CONFIG_SOC_ATMEL_SAM_EXT_MAINCK)) {
		/*
		 * Setup main external crystal oscillator.
		 */

		/* We select maximum setup time.
		 * While start up time could be shortened
		 * this optimization is not deemed
		 * critical now.
		 */
		bool bypass = IS_ENABLED(CONFIG_SOC_ATMEL_SAM_EXT_MAINCK_BYPASS);

		soc_pmc_switch_mainck_to_xtal(bypass, 0xff);
	}

	/*
	 * Set FWS = 6 before switching MCK to 150 MHz (PLLA).
	 * CHIP_FREQ_FWS_6 = 150,000,000 in same70n20.h; CHIP_FREQ_FWS_5 only
	 * covers <= 138 MHz — FWS=5 is one wait state short at 150 MHz.
	 */
	EFC->EEFC_FMR = EEFC_FMR_FWS(6) | EEFC_FMR_CLOE;

	/*
	 * Setup PLLA
	 */

	/*
	 * PLL clock = Main * (MULA + 1) / DIVA
	 *
	 * By default, MULA == 24, DIVA == 1.
	 * With main crystal running at 12 MHz,
	 * PLL = 12 * (24 + 1) / 1 = 300 MHz
	 *
	 * With Processor Clock prescaler at 1
	 * Processor Clock (HCLK)=300 MHz.
	 */
	soc_pmc_enable_pllack(CONFIG_SOC_ATMEL_SAM_PLLA_MULA, 0x3Fu,
			      CONFIG_SOC_ATMEL_SAM_PLLA_DIVA);

	soc_pmc_enable_upllck(0x3Fu);

	/*
	 * Final setup of the Master Clock
	 */

	/* Setting PLLA as MCK, first prescaler, then divider and source last */
	soc_pmc_mck_set_prescaler(CONFIG_SOC_ATMEL_SAM_MCK_PRESCALLER);
	soc_pmc_mck_set_divider(CONFIG_SOC_ATMEL_SAM_MDIV);
	soc_pmc_mck_set_source(SOC_PMC_MCK_SRC_PLLA_CLK);

	/* Disable internal fast RC if we have an external crystal oscillator */
	if (IS_ENABLED(CONFIG_SOC_ATMEL_SAM_EXT_MAINCK)) {
		soc_pmc_osc_disable_fastrc();
	}
}

void soc_reset_hook(void)
{
	if (IS_ENABLED(CONFIG_SOC_ATMEL_SAM_WAIT_MODE)) {
		/*
		 * Instruct CPU to enter Wait mode instead of Sleep mode to
		 * keep Processor Clock (HCLK) and thus be able to debug
		 * CPU using JTAG.
		 */
		soc_pmc_enable_waitmode();
	}

	/*
	 * Enable DTCM and ITCM.  soc_prep_hook() (called from z_prep_c just
	 * before arch_bss_zero) also enables them; this call is a belt-and-
	 * suspenders duplicate that ensures TCM is live even if the prep hook
	 * ordering ever changes.  ARM TRM requires DSB+ISB after enabling TCM.
	 */
	SCB->DTCMCR |= SCB_DTCMCR_EN_Msk;
	SCB->ITCMCR |= SCB_ITCMCR_EN_Msk;
	__DSB();
	__ISB();

	/*
	 * Cortex-M7 L1 D-cache maintenance.  soc_reset_hook() is entered under TWO
	 * cache states and the correct op differs — branch on CCR.DC:
	 *
	 *   DC=1  -> warm soft-jump (bootloader -> app; jump_to_app leaves D-cache
	 *            ENABLED on handoff).  The prologue push {r4,r5,r6,lr} wrote the
	 *            return address into a DIRTY, write-back D-cache line not yet in
	 *            SRAM.  Must CLEAN+invalidate (DCCISW, via SCB_DisableDCache) to
	 *            flush LR to SRAM before disabling, or the epilogue pop {pc}
	 *            fetches stale SRAM and jumps to a garbage address.
	 *
	 *   DC=0  -> hardware reset (POR / backup-exit VROFF / SYSRESETREQ).  Cache
	 *            was off, so the prologue push went straight to SRAM (LR safe).
	 *            After a backup-exit the cache RAM lost power and comes up with
	 *            RANDOM tag/valid/dirty bits: a clean-by-set/way (DCCISW) would
	 *            read random tags and scatter random data to random SRAM
	 *            addresses (corruption -> mpsc_pbuf/log_buffer HardFault on the
	 *            bootloader wake).  Must INVALIDATE-only (DCISW) — it never
	 *            derives an address from a tag, so it is safe on random RAM.
	 *
	 * DC=1 with random cache RAM is impossible (losing power forces DC=0), so
	 * the two arms are exhaustive.  See this fork's README.md for the full
	 * two-cache-state rationale.
	 */
	if (SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_DisableDCache();     /* DC=1: clean+invalidate, preserves dirty LR */
	} else {
		SCB_InvalidateDCache();  /* DC=0: invalidate-only, safe on random RAM */
	}

	/*
	 * I-cache invalidation sequence for Cortex-M7 (SAM S70):
	 *
	 * 1. Pre-enable ICIALLU handles the case where a prior stage (bootloader)
	 *    left IC=1: SCB_EnableICache() returns early if IC is set, skipping
	 *    its internal ICIALLU.  Writing ICIALLU here while IC=1 is DEFINED
	 *    per ARM DDI0489 §4.2.3 and clears any stale lines the prior stage
	 *    left behind.
	 *
	 * 2. Post-enable ICIALLU handles the cold-boot (IC=0) case: ICIALLU is
	 *    UNPREDICTABLE on Cortex-M7 when IC=0 (DDI0489 §4.2.3) — on this
	 *    implementation it is a NOP.  SYSRESETREQ clears CCR.IC but does NOT
	 *    clear the cache tag SRAM, so valid bits from a prior erase/reprogram
	 *    cycle survive the reset.  After sys_cache_instr_enable() sets IC=1
	 *    we issue ICIALLU again with IC=1 (guaranteed defined) to clear those
	 *    stale lines before any instruction fetch can use them.
	 */
	SCB->ICIALLU = 0UL;   /* pre-enable: clears stale lines if IC was already 1 */
	__DSB();
	__ISB();

	sys_cache_instr_enable();   /* sets IC=1 (ICIALLU inside is with IC=0: NOP) */

	SCB->ICIALLU = 0UL;   /* post-enable: ICIALLU with IC=1 — defined, guaranteed */
	__DSB();
	__ISB();

	sys_cache_data_enable();

	/* Setup system clocks */
	clock_init();
}

/**
 * @brief Enable TCM immediately before arch_bss_zero() runs.
 *
 * Called from z_prep_c() before arch_bss_zero().  arch_bss_zero() writes
 * to __dtcm_bss_start (0x20000000) when DT_CHOSEN(zephyr_dtcm) is defined.
 * On samx7x that address is unmapped until DTCM is enabled, causing a bus
 * fault.  soc_reset_hook() also enables TCM earlier, but soc_prep_hook()
 * is the guaranteed last call-site before the first DTCM write.
 */
void soc_prep_hook(void)
{
	SCB->DTCMCR |= SCB_DTCMCR_EN_Msk;
	SCB->ITCMCR |= SCB_ITCMCR_EN_Msk;
	__DSB();
	__ISB();
}

extern void atmel_samx7x_config(void);
/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run at the very beginning.
 */
void soc_early_init_hook(void)
{
	/* Disable WDT immediately at reset before any driver touches it.
	 * Hardware default WDT_MR = 0x00010300 (3s timeout, reset enabled).
	 * WDT_MR is write-once per power cycle — must be done here before
	 * the Zephyr WDT driver PRE_KERNEL_1 init runs.
	 */
	WDT->WDT_MR = WDT_MR_WDDIS | WDT_MR_WDV(0xFFF) | WDT_MR_WDD(0xFFF);

	/* Drive PD19 (UART4 TX) HIGH before the UART driver claims the pin.
	 * After SYSRESETREQ, PD19 floats as GPIO-input; TeraTerm's UART
	 * receiver sees noise and loses framing.  A few ms of TX=HIGH gives
	 * the USB-UART dongle time to clear and resync.
	 * At 150 MHz MCK (inherited from bootloader) ~200 000 loop iterations
	 * ≈ 4 ms, well over 20 character periods at 57600 baud (174 µs each). */
	PMC->PMC_PCER0 = (1U << 16);   /* PIOD peripheral clock on (PID 16) */
	PIOD->PIO_PER  = (1U << 19);   /* PIO takes control of PD19        */
	PIOD->PIO_OER  = (1U << 19);   /* PD19 = output                    */
	PIOD->PIO_SODR = (1U << 19);   /* PD19 = HIGH (UART idle = 1)      */
	for (volatile uint32_t i = 0; i < 200000U; i++) { }  /* ~4 ms      */

	/* S-06: Cortex-M7 AHBS arbitration — favour DMA over CPU for TCM.
	 * CM7_AHBSCR @ 0xE000EFA0:
	 *   CTL=0b10  priority-aware demotion (AHBS demoted only when CPU
	 *             execution priority < TPRI threshold)
	 *   TPRI=0x40 demote AHBS only when CPU is below priority 0x40
	 *             (ISRs at priority < 0x40 still win arbitration)
	 *   INITCOUNT=1 round-robin fairness when not demoted
	 * Combined value: 0x00000902  (TRM rev F section 5.7.3)
	 */
	*((volatile uint32_t *)0xE000EFA0) = 0x00000902;

	/* S-06: Errata DS80000767M §2.22.1 — set RSTC_MR.ERSTL >= 1 to
	 * prevent infinite WDT-reset loop on external reset assertion.
	 */
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_ERSTL(1);

	/* Check that the CHIP CIDR matches the HAL one */
	#ifdef CONFIG_BOARD_EXPECTED_CIDR
		uint32_t expected = CONFIG_BOARD_EXPECTED_CIDR;
	#else
		uint32_t expected = CHIP_CIDR;
	#endif
	if (CHIPID->CHIPID_CIDR != expected) {
		LOG_WRN("CIDR mismatch: chip = 0x%08x vs HAL = 0x%08x",
			(uint32_t)CHIPID->CHIPID_CIDR, expected);
	}
	atmel_samx7x_config();
}

