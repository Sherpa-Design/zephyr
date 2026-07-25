# Zephyr fork — Microchip SAM E70 / ATSAMx7x platform fixes

A downstream fork of Zephyr RTOS carrying **Microchip SAM E70 (`samx7x`) platform
fixes** — eMMC/HSMCI, USBHS/MSD, I²C (TWIHS), clock control, SoC startup, and
Cortex-M7 cache/boot hand-off. For upstream Zephyr's README, licensing, and
getting-started, see **[`README.rst`](README.rst)**.

Most commits here are candidate upstream contributions. The item below is a
**deliberate divergence** from upstream that must not be lost on a re-sync.

---

## Deliberate divergence — read before re-syncing `soc.c` to upstream

### `soc/atmel/sam/samx7x/soc.c` · `soc_reset_hook()` — `CCR.DC`-gated D-cache maintenance

Upstream `soc_reset_hook()` invalidates the D-cache **unconditionally**, which is
correct upstream because the hook is only ever entered from a hardware reset
(`CCR.DC = 0`). When the same SoC image is chain-loaded by a first-stage
bootloader that performs a **warm software jump** (rather than a hardware reset),
the hook is entered under a **second** cache state, and the correct L1 D-cache
maintenance operation differs. This fork therefore **branches on `CCR.DC`**:

- **`CCR.DC = 1` — warm software jump** (a first-stage bootloader jumps to this
  image with the D-cache still enabled). The function-prologue `push {…, lr}` has
  written the return address into a **dirty** write-back cache line that is not yet
  in SRAM. The hook must **clean+invalidate** (`SCB_DisableDCache()`, DCCISW) to
  flush `lr` to SRAM before disabling — otherwise the epilogue `pop {pc}` returns
  to a garbage address.

- **`CCR.DC = 0` — hardware reset** (POR / **backup-mode (VROFF) exit wake** /
  SYSRESETREQ). The cache was off, so `lr` went straight to SRAM. After a
  **backup-mode (VROFF) exit** the cache tag/data RAM **lost power and comes up
  with random** tag/valid/dirty bits — a clean-by-set/way (DCCISW) would read those
  random tags and **scatter random data to random SRAM addresses**, corrupting
  `.data`/`.bss` before the C-runtime finishes initializing them. The hook must
  **invalidate-only** (`SCB_InvalidateDCache()`, DCISW), which never derives an
  address from a tag and is safe on random cache RAM.

`DC=1` with random cache RAM is impossible (losing power forces `DC=0`), so the two
arms are exhaustive.

**Symptom this fixed:** an intermittent Cortex-M7 HardFault on the **first boot
after a low-power backup-mode (VROFF) wake** — a BusFault in Zephyr's deferred-log
allocator (`mpsc_pbuf_alloc`) dereferencing an uninitialized `log_buffer->buf`,
because `.data`/`.bss` initialization had been corrupted by the unconditional clean
over powered-up-random cache RAM. Never seen on POR or warm software-reset (SRAM
defined/preserved); only the VROFF wake exposed it.

**Companion pattern (bootloader side, not in this repo):** a first-stage bootloader
should call `SCB_DisableDCache()` before jumping, so the chain-loaded image enters
`soc_reset_hook()` with `CCR.DC = 0` ("caches off before chain-load"). That makes
the image take the `DC=0` invalidate-only arm. The `CCR.DC` gate above still stands
on its own and both are worth keeping (defense in depth). **If this fork's `soc.c`
is ever re-synced to upstream, do NOT drop the `CCR.DC` gate** unless the
bootloader's disable-before-jump invariant is proven to hold on every path —
otherwise the warm-jump `DC=1` case loses `lr` and returns to garbage. The `soc.c`
comment block at the branch carries the same reasoning inline.

---

## Other `samx7x` fixes on this branch

Candidate-upstream SAM E70 platform corrections; see the individual commit messages
for details:

- **eMMC / HSMCI** — CLKDIV ceiling rounding, XDMAC hardware-handshaked transfer
  path, 4-bit bus.
- **USBHS / MSD** — SAM E70 mass-storage enumeration, drain-ISR cleanup.
- **I²C (TWIHS)** — write+read repeated-start stall via `TWIHS_IADRSZ`, live-bitrate
  tracking, SWRST state-machine reset, bus recovery.
- **Clock control** — `clock_control_sam_pmc` decodes the **live** `PMC_MCKR` for
  peripheral clock rate under runtime MCK switching; EEFC flash wait-states (FWS=6
  at 150 MHz).
- **SoC startup** — DTCM/ITCM enable, WDT/RSTC errata handling, AHBS arbitration.
- **BMI270** — re-enable Advanced Power Save after init.
