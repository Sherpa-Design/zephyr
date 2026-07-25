# Sherpa VEET — Zephyr fork

Sherpa Design's fork of Zephyr RTOS for the **VEET** data logger
(Microchip **ATSAMS70N20A**, Arm Cortex-M7 / SAM E70). For upstream Zephyr's
README, licensing, and getting-started, see **[`README.rst`](README.rst)**.

- **Fork branch:** `veet-samx7x-fixes` (remote `sherpa`)
- **Scope:** VEET-specific SAM E70 (`samx7x`) platform fixes — eMMC/HSMCI, USBHS/MSD,
  I²C (TWIHS), clock control, SoC startup, and bootloader hand-off. Most are candidate
  upstream contributions; the item below is a **deliberate divergence** that must not be
  lost on a re-sync.

---

## Deliberate divergence — read before re-syncing `soc.c` to upstream

### `soc/atmel/sam/samx7x/soc.c` · `soc_reset_hook()` — `CCR.DC`-gated D-cache maintenance

**Fork commit:** `42f29473e65`

Upstream `soc_reset_hook()` invalidates the D-cache **unconditionally** — which is correct
upstream because the hook is only ever entered from a hardware reset (`CCR.DC = 0`). On VEET
the same hook is entered under **two** cache states, and the correct L1 D-cache maintenance
operation differs. The fork therefore **branches on `CCR.DC`**:

- **`CCR.DC = 1` — warm software jump** (custom VEET bootloader → application; our
  `jump_to_app()` hands off with the D-cache still enabled). The function-prologue
  `push {…, lr}` has written the return address into a **dirty** write-back cache line that is
  not yet in SRAM. The hook must **clean+invalidate** (`SCB_DisableDCache()`, DCCISW) to flush
  `lr` to SRAM before disabling — otherwise the epilogue `pop {pc}` returns to a garbage address.

- **`CCR.DC = 0` — hardware reset** (POR / **backup-mode (VROFF) exit wake** / SYSRESETREQ).
  The cache was off, so `lr` went straight to SRAM. After a **backup-mode (VROFF) exit** the
  cache tag/data RAM **lost power and comes up with random** tag/valid/dirty bits — a
  clean-by-set/way (DCCISW) would read those random tags and **scatter random data to random
  SRAM addresses**, corrupting `.data`/`.bss` before the C-runtime finishes initializing them.
  The hook must **invalidate-only** (`SCB_InvalidateDCache()`, DCISW), which never derives an
  address from a tag and is safe on random cache RAM.

`DC=1` with random cache RAM is impossible (losing power forces `DC=0`), so the two arms are
exhaustive.

**Symptom this fixed:** an intermittent Cortex-M7 HardFault in the bootloader on the **first
boot after a low-power backup-mode (VROFF) wake** — a BusFault in Zephyr's deferred-log
allocator (`mpsc_pbuf_alloc`) dereferencing an uninitialized `log_buffer->buf`, because
`.data`/`.bss` init had been corrupted by the unconditional clean over powered-up-random cache
RAM. Never seen on POR or warm software-reset (SRAM defined/preserved); only the VROFF wake —
a reset class new to VEET's deep-sleep feature — exposed it.

**Companion change (application side — in the VEET app repo, not this fork):** the custom
bootloader's `jump_to_app()` now calls `SCB_DisableDCache()` before the jump, so the
application enters `soc_reset_hook()` with `CCR.DC = 0` ("caches off before chain-load"). That
makes the app take the `DC=0` invalidate-only arm. The two are retained together as defense in
depth. **If this fork's `soc.c` is ever re-synced to upstream, do NOT drop the `CCR.DC` gate**
unless the bootloader's disable-before-jump invariant is proven to hold on every path — else
the warm-jump `DC=1` case loses `lr` and returns to garbage. The `soc.c` comment block at the
branch carries the same reasoning inline.

**Full investigation** — register-level evidence, the zeros→garbage SRAM proof, alternatives
considered, and the four-path (backup-exit / POR / SYSRESETREQ / soft-jump) test matrix — lives
in the VEET app repo: `docs/handoff_opus_2026-07-24_bootloader_backup_wake_hardfault.md`.

---

## Other `samx7x` fixes on this branch

Candidate-upstream SAM E70 platform corrections; see the individual commit messages and the
VEET app repo's `BUGS.md` for rationale:

- **eMMC / HSMCI** — CLKDIV ceiling rounding, XDMAC hardware-handshaked transfer path, 4-bit bus.
- **USBHS / MSD** — SAM E70 mass-storage enumeration, drain-ISR cleanup.
- **I²C (TWIHS)** — write+read repeated-start stall via `TWIHS_IADRSZ`, live-bitrate tracking,
  SWRST state-machine reset, bus recovery.
- **Clock control** — `clock_control_sam_pmc` decodes the **live** `PMC_MCKR` for peripheral
  clock rate under runtime MCK switching; EEFC flash wait-states (FWS=6 at 150 MHz).
- **SoC startup** — DTCM/ITCM enable, WDT/RSTC errata handling, AHBS arbitration.
- **BMI270** — re-enable Advanced Power Save after init.
