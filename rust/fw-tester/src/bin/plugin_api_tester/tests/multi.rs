// Copyright (c) 2026 William Athing
//
// MIT License

//! Tests for this fork's local plugin API extensions, which reach the chips of
//! a multi-ROM slot beyond chip 0:
//!
//! - `ORA_ID_LOCAL_READ_RAM_ROM_SLOT_CHIP` and
//!   `ORA_ID_LOCAL_REPROGRAM_RAM_ROM_SLOT_CHIP` read and write one chip.
//! - `ORA_ID_LOCAL_DEMANGLE_ADDR_CHIP` says which chip a capture read.
//! - `ORA_ID_LOCAL_SET_X_PIN_ENABLED` stops One ROM answering an X pin's chip
//!   select, so another device on that select can.
//!
//! What a host reads is the oracle, so the writes are checked by serving: a RAM
//! slot is filled with junk, every chip is written through the API, and each
//! is then read through the emulated PIO.  A write that missed any table entry
//! a real read can reach leaves junk there to be served.  The reads drive the
//! address window's unused GPIOs and a smaller chip's missing address lines
//! both low and high, since a host leaves those at whatever level it likes.
//!
//! The chip numbering is the API's: 0 is the chip One ROM sits in place of,
//! and chip n is selected by X pin n.

use std::path::Path;

use onerom_config::chip::ChipType;
use onerom_config::fw::FirmwareVersion;
use onerom_config::hw::Board;
use onerom_fw_emulator::{Emulator, ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS, OraResult, ffi};
use onerom_fw_tester::driver;
use onerom_fw_tester::geometry;
use onerom_fw_tester::oracle;
use onerom_fw_tester::pin_cache::{ControlLine, PinCache, per_chip_select};
use onerom_fw_tester::runner::{addr_before_cs_cycles, run_mode};
use onerom_fw_tester::timing;
use onerom_gen::{ChipSetType, Config};

/// Where the demangle test's address monitor ring sits: above the RAM slots,
/// as the address-monitor tester places it.
const RING_BASE: u32 = 0x2008_1000;
const RING_ENTRIES_LOG2: u8 = 6;
const RING_DATA_SIZE: u8 = 32;

/// How long a read the monitor is to capture holds its select.  The monitor
/// waits for the select to read the same three times before it samples the
/// address, so a read as short as the serving tests' can end before the sample.
/// A CoCo's reads are several times this long.
const CAPTURE_ACTIVE_CYCLES: u32 = 32;

/// One chip of the slot, as the tests drive it.
struct ChipRig {
    chip_type: ChipType,
    cache: PinCache,
    image: Vec<u8>,
    /// The other chips' selects, held deasserted.
    background: (u64, u64),
    /// chip0's address GPIOs this chip doesn't have.
    extra_gpios: u64,
}

/// A multi-ROM slot's chips, and the address window's unused GPIOs.
pub struct MultiRig {
    set_idx: usize,
    chips: Vec<ChipRig>,
    /// GPIOs in the address window that are no line of any chip, no select and
    /// not held at one level by the firmware - VPP on a 27128, say.
    gap_gpios: u64,
}

fn mask_of(gpios: impl IntoIterator<Item = u8>) -> u64 {
    gpios.into_iter().fold(0u64, |m, g| m | (1u64 << g))
}

impl MultiRig {
    /// Build the rig for chip set `set_idx`, which must be a multi-ROM set.
    pub fn build(
        config: &Config,
        board: Board,
        fw_version: FirmwareVersion,
        base_dir: &Path,
        set_idx: usize,
    ) -> Result<Self, String> {
        let chip_set = config
            .chip_sets
            .get(set_idx)
            .ok_or_else(|| format!("config has no chip set {set_idx}"))?;
        let primary_config = chip_set
            .chips
            .first()
            .ok_or_else(|| format!("chip set {set_idx} has no chips"))?;
        let primary_type = primary_config.chip_type.resolved();
        let primary = PinCache::build(primary_type, primary_config, board);

        // Each secondary's X pin, at the level that asserts its select.
        let mut x_lines = Vec::new();
        for (i, chip_config) in chip_set.chips.iter().skip(1).enumerate() {
            let (_, gpios) = board
                .x_pin_map()
                .get(i)
                .ok_or_else(|| format!("{} has no X{}", board.name(), i + 1))?;
            let (_, assert_high) = per_chip_select(chip_config)
                .ok_or_else(|| format!("chip {} has no per-chip select", i + 1))?;
            x_lines.push(ControlLine {
                name: "x_cs",
                gpios: gpios.to_vec(),
                assert_high,
                commoned: false,
            });
        }
        let x_deassert: Vec<(u64, u64)> = x_lines
            .iter()
            .map(|line| driver::ctrl_mask(std::slice::from_ref(line), false))
            .collect();
        let others_deasserted = |skip: Option<usize>| {
            x_deassert
                .iter()
                .enumerate()
                .filter(|(i, _)| Some(*i) != skip)
                .fold((0u64, 0u64), |acc, (_, &m)| driver::merge(acc, m))
        };
        let primary_deassert = driver::ctrl_mask(&primary.control_lines, false);
        let primary_addr = mask_of(primary.addr_gpios.iter().flatten().copied());

        let mut chips = Vec::new();
        for (k, chip_config) in chip_set.chips.iter().enumerate() {
            let chip_type = chip_config.chip_type.resolved();
            let image = oracle::load(chip_config, chip_type, base_dir);
            let chip = if k == 0 {
                // chip0's control lines other than the per-chip select are
                // commoned: they qualify a read rather than choose the chip,
                // so the runner's tristate sweep holds them deasserted, as the
                // pio-tester does.
                let mut cache = PinCache::build(primary_type, primary_config, board);
                if let Some((select, _)) = chip_set.chips.get(1).and_then(per_chip_select) {
                    for line in &mut cache.control_lines {
                        if line.name != select {
                            line.commoned = true;
                        }
                    }
                }
                ChipRig {
                    chip_type,
                    cache,
                    image,
                    background: others_deasserted(None),
                    extra_gpios: 0,
                }
            } else {
                let line = &x_lines[k - 1];
                let cache = PinCache::build_secondary(
                    chip_type,
                    &primary,
                    board,
                    line.gpios.clone(),
                    line.assert_high,
                );
                let own = mask_of(cache.addr_gpios.iter().flatten().copied());
                ChipRig {
                    chip_type,
                    cache,
                    image,
                    background: driver::merge(primary_deassert, others_deasserted(Some(k - 1))),
                    extra_gpios: primary_addr & !own,
                }
            };
            chips.push(chip);
        }

        let geometry = geometry::slot_geometry(
            config,
            board,
            fw_version,
            base_dir,
            crate::flash_slot(config, set_idx),
        )?;
        let window = (0..geometry.addr_window_len)
            .map(|i| geometry.addr_window_base + i)
            .fold(0u64, |m, g| m | (1u64 << g));
        let used = primary_addr
            | mask_of(
                primary
                    .control_lines
                    .iter()
                    .flat_map(|l| l.gpios.iter().copied()),
            )
            | mask_of(geometry.x_pin_gpios.iter().flatten().copied())
            | mask_of(geometry.forced_low_gpios.iter().copied())
            | mask_of(primary.data_gpios.iter().copied());

        Ok(Self {
            set_idx,
            chips,
            gap_gpios: window & !used,
        })
    }

    /// The chips in the slot.
    pub fn chip_count(&self) -> usize {
        self.chips.len()
    }
}

/// Drive one read of `addr` from `chip`, holding the select for `active`
/// cycles, with `extra` on top of the chip's own background.  Returns whether
/// every data line was driven, and the byte on them.
fn read_once(
    emu: &Emulator,
    chip: &ChipRig,
    addr: usize,
    extra: (u64, u64),
    active: u32,
) -> (bool, u8) {
    let background = driver::merge(chip.background, extra);
    let a = driver::addr_mask(addr, &chip.cache.addr_gpios);
    let off = driver::merge(
        driver::merge(a, driver::ctrl_mask(&chip.cache.control_lines, false)),
        background,
    );
    let on = driver::merge(
        driver::merge(a, driver::ctrl_mask(&chip.cache.control_lines, true)),
        background,
    );
    let data = &chip.cache.data_gpios[..8.min(chip.cache.data_gpios.len())];

    emu.drive_gpios(off.0, off.1);
    emu.step_cycles(addr_before_cs_cycles(chip.chip_type));
    emu.drive_gpios(on.0, on.1);
    emu.step_cycles(active);
    let pins = emu.read_pin_states();
    let driven = emu.read_driven_pins();
    emu.drive_gpios(off.0, off.1);
    emu.step_cycles(timing::CYCLES_AFTER_READ);

    let all_driven = data.iter().all(|&g| driven & (1u64 << g) != 0);
    (all_driven, driver::extract_byte(pins, data))
}

/// Serve every address of chip `k` through the PIO and compare with
/// `expected`, with the gap GPIOs and the chip's missing address lines all low
/// and then all high.
fn verify_serving(emu: &Emulator, rig: &MultiRig, k: usize, expected: &[u8]) -> Result<(), String> {
    let chip = &rig.chips[k];
    let floating = rig.gap_gpios | chip.extra_gpios;
    let levels: &[u64] = if floating == 0 { &[0] } else { &[0, u64::MAX] };

    let mut errors = Vec::new();
    for &level in levels {
        let background = driver::merge(chip.background, (floating, floating & level));
        let (_, failures, bus_failures, _) = run_mode(
            emu,
            &chip.cache,
            expected,
            8,
            addr_before_cs_cycles(chip.chip_type),
            timing::CYCLES_CS_TO_DATA_MULTI,
            rig.set_idx,
            k,
            background,
            &[],
        );
        if failures != 0 || bus_failures != 0 {
            errors.push(format!(
                "chip {k} with unused GPIOs {}: {failures} data failure(s), {bus_failures} bus \
                 violation(s)",
                if level == 0 { "low" } else { "high" }
            ));
        }
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

fn note(errors: &mut Vec<String>, what: &str, got: OraResult, want: OraResult) {
    if got != want {
        errors.push(format!("{what}: got {got:?}, want {want:?}"));
    }
}

fn verdict(errors: Vec<String>) -> Result<(), String> {
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

/// The slot as booted serves every chip's image.  The baseline: if this fails,
/// the serving path is broken before any of the calls under test run.
pub fn test_initial_serving(emu: &Emulator, rig: &MultiRig) -> Result<(), String> {
    let mut errors = Vec::new();
    for (k, chip) in rig.chips.iter().enumerate() {
        if let Err(e) = verify_serving(emu, rig, k, &chip.image) {
            errors.push(e);
        }
    }
    if errors.is_empty() {
        println!(
            "  {} chips served, unused GPIOs {:#x}",
            rig.chips.len(),
            rig.gap_gpios
        );
    }
    verdict(errors)
}

/// Reading each chip of the booted slot gives that chip's image, which is what
/// the generator laid out - so the read and the generator agree on where each
/// chip's bytes are.
pub fn test_read_chip(emu: &Emulator, rig: &MultiRig, slot: u8) -> Result<(), String> {
    let mut errors = Vec::new();
    for (k, chip) in rig.chips.iter().enumerate() {
        let mut buf = vec![0u8; chip.image.len()];
        let result = emu.read_ram_rom_slot_chip(slot, k as u8, 0, &mut buf);
        if !result.is_ok() {
            errors.push(format!("read of chip {k} got {result:?}"));
            continue;
        }
        let wrong = buf.iter().zip(&chip.image).filter(|(g, e)| g != e).count();
        if wrong != 0 {
            let first = buf
                .iter()
                .zip(&chip.image)
                .position(|(g, e)| g != e)
                .unwrap_or(0);
            errors.push(format!(
                "chip {k}: {wrong} byte(s) differ from its image, first at 0x{first:04X}"
            ));
        }
    }
    if errors.is_empty() {
        println!("  {} chips read back as their images", rig.chips.len());
    }
    verdict(errors)
}

/// Every way the chip calls and the X pin call refuse, and the edges they
/// accept.
pub fn test_refusals(emu: &Emulator, rig: &MultiRig, active_slot: u8) -> Result<(), String> {
    let n = rig.chips.len() as u8;
    let ram_count = emu.get_ram_slot_count();
    let mut errors = Vec::new();
    let mut byte = [0u8; 1];
    let mut two = [0u8; 2];

    note(
        &mut errors,
        &format!("read chip {n}, past the last"),
        emu.read_ram_rom_slot_chip(active_slot, n, 0, &mut byte),
        OraResult::InvalidArg,
    );
    note(
        &mut errors,
        &format!("reprogram chip {n}, past the last"),
        emu.reprogram_ram_rom_slot_chip(active_slot, n, 0, &byte, true),
        OraResult::InvalidArg,
    );
    note(
        &mut errors,
        &format!("read RAM slot {ram_count}, past the last"),
        emu.read_ram_rom_slot_chip(ram_count, 0, 0, &mut byte),
        OraResult::InvalidSlot,
    );
    note(
        &mut errors,
        "reprogram the active slot without allow_active",
        emu.reprogram_ram_rom_slot_chip(active_slot, 0, 0, &byte, false),
        OraResult::SlotActive,
    );
    note(
        &mut errors,
        "read of no bytes",
        emu.read_ram_rom_slot_chip(active_slot, 0, 0, &mut []),
        OraResult::InvalidArg,
    );

    for (k, chip) in rig.chips.iter().enumerate() {
        let size = chip.image.len() as u32;
        let k = k as u8;
        note(
            &mut errors,
            &format!("read chip {k}'s last byte"),
            emu.read_ram_rom_slot_chip(active_slot, k, size - 1, &mut byte),
            OraResult::Ok,
        );
        note(
            &mut errors,
            &format!("read across chip {k}'s end"),
            emu.read_ram_rom_slot_chip(active_slot, k, size - 1, &mut two),
            OraResult::InvalidArg,
        );
        note(
            &mut errors,
            &format!("reprogram past chip {k}'s end"),
            emu.reprogram_ram_rom_slot_chip(active_slot, k, size, &byte, true),
            OraResult::InvalidArg,
        );
    }

    for (x_pin, enabled, want, what) in [
        (0u8, 1u8, OraResult::InvalidArg, "X pin 0"),
        (
            n,
            1,
            OraResult::InvalidArg,
            "an X pin past the slot's chips",
        ),
        (3, 1, OraResult::InvalidArg, "X pin 3"),
        (1, 2, OraResult::InvalidArg, "enabled 2"),
        (1, 1, OraResult::Ok, "enabling an enabled X1"),
    ] {
        note(
            &mut errors,
            what,
            emu.set_x_pin_enabled(x_pin, enabled),
            want,
        );
    }

    verdict(errors)
}

/// Junk in every table entry of `slot`, then each chip written through the
/// API: every chip serves its image.  So each write reached every entry a read
/// can land on, and no write landed on another chip's.
pub fn test_reprogram_chip_serves(emu: &Emulator, rig: &MultiRig, slot: u8) -> Result<(), String> {
    let (result, info) = emu.get_ram_slot_info(slot);
    let info = match (result, info) {
        (OraResult::Ok, Some(info)) => info,
        (result, _) => return Err(format!("get_ram_slot_info({slot}) got {result:?}")),
    };

    // SAFETY: the RAM slot lies within epio's SRAM buffer, which
    // `sram_host_ptr` points into, and nothing else holds a reference to it.
    let region = unsafe {
        std::slice::from_raw_parts_mut(emu.sram_host_ptr(info.addr) as *mut u8, info.size as usize)
    };
    for (i, b) in region.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37).wrapping_add(11);
    }

    for (k, chip) in rig.chips.iter().enumerate() {
        let result = emu.reprogram_ram_rom_slot_chip(slot, k as u8, 0, &chip.image, true);
        if !result.is_ok() {
            return Err(format!("reprogram of chip {k} got {result:?}"));
        }
    }

    let mut errors = Vec::new();
    for (k, chip) in rig.chips.iter().enumerate() {
        if let Err(e) = verify_serving(emu, rig, k, &chip.image) {
            errors.push(e);
        }
    }
    if errors.is_empty() {
        println!("  {} chips written over junk and served", rig.chips.len());
    }
    verdict(errors)
}

/// The GPIO input overrides on `gpios`, as the apio emulation records them:
/// (inverted, forced low, forced high).
fn overrides(gpios: &[u8]) -> Vec<(u8, u8, u8)> {
    // SAFETY: `_apio_emulated_gpios` is a plain C global written by the
    // firmware under emulation; this reads it through a raw pointer without
    // forming a reference to the `static mut`.
    let state = unsafe { &*core::ptr::addr_of!(ffi::_apio_emulated_gpios) };
    gpios
        .iter()
        .map(|&g| {
            let g = g as usize;
            (
                state.inverted[g],
                state.force_input_low[g],
                state.force_input_high[g],
            )
        })
        .collect()
}

/// A disabled X pin's chip is not served while the other chips are, and
/// enabling it again serves it and restores the overrides the slot booted with.
pub fn test_x_pin_disable(emu: &Emulator, rig: &MultiRig, board: Board) -> Result<(), String> {
    let mut errors = Vec::new();

    for x_pin in 1..rig.chips.len() as u8 {
        let k = x_pin as usize;
        let chip = &rig.chips[k];
        let gpios = board.gpios_for_x_pin(x_pin);
        let booted = overrides(gpios);

        note(
            &mut errors,
            &format!("disable X{x_pin}"),
            emu.set_x_pin_enabled(x_pin, 0),
            OraResult::Ok,
        );
        note(
            &mut errors,
            &format!("disable X{x_pin} again"),
            emu.set_x_pin_enabled(x_pin, 0),
            OraResult::Ok,
        );
        if overrides(gpios).iter().any(|&o| o != (0, 1, 0)) {
            errors.push(format!(
                "X{x_pin} GPIOs {gpios:?} not all forced low: {:?}",
                overrides(gpios)
            ));
        }

        let size = chip.image.len();
        let served = [0, 1, size / 2, size - 1]
            .into_iter()
            .filter(|&addr| read_once(emu, chip, addr, (0, 0), timing::CYCLES_CS_TO_DATA_MULTI).0)
            .collect::<Vec<_>>();
        if !served.is_empty() {
            errors.push(format!(
                "chip {k} still driven the bus with X{x_pin} disabled, at {served:?}"
            ));
        }
        for (other, other_chip) in rig.chips.iter().enumerate() {
            if other != k
                && let Err(e) = verify_serving(emu, rig, other, &other_chip.image)
            {
                errors.push(format!("with X{x_pin} disabled: {e}"));
            }
        }

        note(
            &mut errors,
            &format!("enable X{x_pin}"),
            emu.set_x_pin_enabled(x_pin, 1),
            OraResult::Ok,
        );
        if overrides(gpios) != booted {
            errors.push(format!(
                "X{x_pin} overrides {:?} after enabling, booted with {booted:?}",
                overrides(gpios)
            ));
        }
        if let Err(e) = verify_serving(emu, rig, k, &chip.image) {
            errors.push(format!("after enabling X{x_pin}: {e}"));
        }
    }

    if errors.is_empty() {
        println!("  {} X pin(s) disabled and enabled", rig.chips.len() - 1);
    }
    verdict(errors)
}

/// The ring entry the address monitor writes for one read of `addr` from
/// `chip`, or `None` where it captured nothing.
fn capture(emu: &Emulator, chip: &ChipRig, addr: usize) -> Result<Option<u32>, String> {
    let slot = emu.get_address_monitor_ring_write_pos();
    if slot.is_null() {
        return Err("get_address_monitor_ring_write_pos returned NULL".to_string());
    }
    // SAFETY: the firmware keeps the write position slot valid while the
    // monitor runs, and it points into epio SRAM.
    let before = unsafe { *slot };
    read_once(emu, chip, addr, (0, 0), CAPTURE_ACTIVE_CYCLES);
    let after = unsafe { *slot };
    if after == before {
        return Ok(None);
    }
    Ok(Some(unsafe { *before }))
}

/// With the address monitor capturing every chip, a capture of a read from
/// each chip demangles to that chip and the address read.  A capture selecting
/// two chips, or none, is refused, and a disabled X pin's chip is not captured
/// at all.
///
/// Starts the address monitor, so it runs last.  The upstream monitor, chip 0
/// alone, is the address-monitor tester's.
pub fn test_demangle_chip(emu: &Emulator, rig: &MultiRig) -> Result<(), String> {
    emu.arm_monitor();
    let ring = emu.sram_host_ptr(RING_BASE);
    // SAFETY: `ring` is a correctly aligned ring of 2^RING_ENTRIES_LOG2 entries
    // in epio SRAM, above the RAM slots, live for the rest of the slot's tests.
    let result = unsafe {
        emu.setup_address_monitor_all_chips(
            ring,
            RING_ENTRIES_LOG2,
            ffi::ora_monitor_mode_t_ORA_MONITOR_MODE_CONTROL,
            RING_DATA_SIZE,
            core::ptr::null(),
        )
    };
    if result != OraResult::Ok {
        return Err(format!("setup_address_monitor got {result:?}"));
    }
    emu.update_from_apio();
    emu.start_address_monitor();
    emu.update_from_apio();

    let mut errors = Vec::new();
    let mut first_entries = Vec::new();
    for (k, chip) in rig.chips.iter().enumerate() {
        let size = chip.image.len();
        for addr in [0, 1, 0x55, size / 2 + 0x2A, size - 1] {
            let entry = match capture(emu, chip, addr)? {
                Some(entry) => entry,
                None => {
                    errors.push(format!("chip {k} read of 0x{addr:04X} not captured"));
                    continue;
                }
            };
            if addr == 0x55 {
                first_entries.push(entry);
            }
            let (result, logical, got_chip) = emu.demangle_addr_chip(entry);
            if result != OraResult::Ok || logical != addr as u32 || got_chip != k as u8 {
                errors.push(format!(
                    "chip {k} read of 0x{addr:04X} demangled to {result:?}, chip {got_chip}, \
                     0x{logical:04X}"
                ));
            }
        }
    }

    // Two chips' captures of one address differ only in their select bits, so
    // their union selects both and their intersection selects neither.
    if let [chip0, chip1, ..] = first_entries[..] {
        for (entry, what) in [(chip0 | chip1, "two chips"), (chip0 & chip1, "no chip")] {
            let (result, _, _) = emu.demangle_addr_chip(entry);
            note(
                &mut errors,
                &format!("a capture selecting {what}"),
                result,
                OraResult::ControlPinActive,
            );
        }
    }

    for x_pin in 1..rig.chips.len() as u8 {
        note(
            &mut errors,
            &format!("disable X{x_pin}"),
            emu.set_x_pin_enabled(x_pin, 0),
            OraResult::Ok,
        );
        if capture(emu, &rig.chips[x_pin as usize], 0x55)?.is_some() {
            errors.push(format!("chip {x_pin} captured with X{x_pin} disabled"));
        }
        note(
            &mut errors,
            &format!("enable X{x_pin}"),
            emu.set_x_pin_enabled(x_pin, 1),
            OraResult::Ok,
        );
    }

    if errors.is_empty() {
        println!("  {} chips' captures demangled", rig.chips.len());
    }
    verdict(errors)
}

/// The single-chip sets in `config` a chip of the rig could be loaded from:
/// for each chip, the first such set whose chip is the same size, with its
/// image.  Empty where the config has none.
pub fn copy_sources(
    config: &Config,
    base_dir: &Path,
    rig: &MultiRig,
) -> Vec<(usize, usize, Vec<u8>)> {
    let mut sources = Vec::new();
    for (k, chip) in rig.chips.iter().enumerate() {
        let found = config
            .chip_sets
            .iter()
            .enumerate()
            .find_map(|(set_idx, set)| {
                let source = set.chips.first()?;
                let chip_type = source.chip_type.resolved();
                (set.set_type == ChipSetType::Single
                    && set_idx >= crate::plugin_sets(config)
                    && chip_type.size_bytes() == chip.chip_type.size_bytes())
                .then(|| (set_idx, oracle::load(source, chip_type, base_dir)))
            });
        if let Some((set_idx, image)) = found {
            sources.push((k, set_idx, image));
        }
    }
    sources
}

/// Each chip of the served slot loaded from a single-chip flash slot of its
/// size serves that slot's ROM; loading the boot slot's own chips back serves
/// the slot as booted.  So a ROM stored compactly becomes a chip of a
/// multi-ROM slot, and a multi-ROM slot's chips copy between slots of one
/// shape.  Then every refusal.
pub fn test_copy_flash_chip(
    emu: &Emulator,
    rig: &MultiRig,
    config: &Config,
    sources: &[(usize, usize, Vec<u8>)],
    slot: u8,
) -> Result<(), String> {
    const FLAGS: u32 = ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS;
    let mut errors = Vec::new();
    let booted = crate::flash_slot(config, rig.set_idx) as u8;

    for (k, set_idx, image) in sources {
        let flash_slot = crate::flash_slot(config, *set_idx) as u8;
        note(
            &mut errors,
            &format!("copy flash slot {flash_slot} into chip {k}"),
            emu.copy_flash_chip_to_ram_chip(flash_slot, FLAGS, 0, slot, *k as u8, true),
            OraResult::Ok,
        );
        if let Err(e) = verify_serving(emu, rig, *k, image) {
            errors.push(format!("loaded from flash slot {flash_slot}: {e}"));
        }
    }

    for (k, chip) in rig.chips.iter().enumerate() {
        note(
            &mut errors,
            &format!("copy the boot slot's chip {k} back"),
            emu.copy_flash_chip_to_ram_chip(booted, FLAGS, k as u8, slot, k as u8, true),
            OraResult::Ok,
        );
        if let Err(e) = verify_serving(emu, rig, k, &chip.image) {
            errors.push(format!("copied back: {e}"));
        }
    }

    let n = rig.chips.len() as u8;
    let flash_count = emu.get_flash_slot_count(FLAGS);
    for (what, got, want) in [
        (
            format!("flash slot {flash_count}, past the last"),
            emu.copy_flash_chip_to_ram_chip(flash_count, FLAGS, 0, slot, 0, true),
            OraResult::InvalidSlot,
        ),
        (
            format!("source chip {n}, past the boot slot's last"),
            emu.copy_flash_chip_to_ram_chip(booted, FLAGS, n, slot, 0, true),
            OraResult::InvalidArg,
        ),
        (
            format!("destination chip {n}, past the last"),
            emu.copy_flash_chip_to_ram_chip(booted, FLAGS, 0, slot, n, true),
            OraResult::InvalidArg,
        ),
        (
            "into the active slot without allow_active".to_string(),
            emu.copy_flash_chip_to_ram_chip(booted, FLAGS, 0, slot, 0, false),
            OraResult::SlotActive,
        ),
    ] {
        note(&mut errors, &what, got, want);
    }

    // A chip of another size, where the config has one.
    let mismatched = sources.iter().find_map(|(_, set_idx, _)| {
        let size = config.chip_sets[*set_idx].chips[0]
            .chip_type
            .resolved()
            .size_bytes();
        rig.chips
            .iter()
            .position(|chip| chip.chip_type.size_bytes() != size)
            .map(|k| (crate::flash_slot(config, *set_idx) as u8, k as u8))
    });
    if let Some((flash_slot, k)) = mismatched {
        note(
            &mut errors,
            &format!("flash slot {flash_slot} into chip {k}, another size"),
            emu.copy_flash_chip_to_ram_chip(flash_slot, FLAGS, 0, slot, k, true),
            OraResult::InvalidSize,
        );
    }

    if errors.is_empty() {
        println!(
            "  {} chip(s) loaded from single-chip slots, {} copied back",
            sources.len(),
            rig.chips.len()
        );
    }
    verdict(errors)
}

/// Outside a multi-ROM slot there is only chip 0: the chip calls match the
/// plain ones on it and refuse any other, and the X pin call isn't supported.
pub fn test_single_slot(emu: &Emulator, slot: u8, boot_image: &[u8]) -> Result<(), String> {
    let mut errors = Vec::new();

    let mut buf = vec![0u8; boot_image.len()];
    let result = emu.read_ram_rom_slot_chip(slot, 0, 0, &mut buf);
    if !result.is_ok() {
        errors.push(format!("read of chip 0 got {result:?}"));
    } else if buf != boot_image {
        errors.push("chip 0 doesn't read back as the boot image".to_string());
    }

    let byte = [0u8; 1];
    let mut one = [0u8; 1];
    note(
        &mut errors,
        "read chip 1",
        emu.read_ram_rom_slot_chip(slot, 1, 0, &mut one),
        OraResult::InvalidArg,
    );
    note(
        &mut errors,
        "reprogram chip 1",
        emu.reprogram_ram_rom_slot_chip(slot, 1, 0, &byte, true),
        OraResult::InvalidArg,
    );
    for enabled in [0u8, 1] {
        note(
            &mut errors,
            &format!("set X1 enabled {enabled}"),
            emu.set_x_pin_enabled(1, enabled),
            OraResult::NotSupported,
        );
    }

    for addr in [0u32, 1, 0x55, boot_image.len() as u32 - 1] {
        let phys = emu.map_addr_to_phys(addr);
        let plain = emu.demangle_addr(phys, true);
        let (result, logical, chip) = emu.demangle_addr_chip(phys);
        if (result, logical) != plain || (result.is_ok() && chip != 0) {
            errors.push(format!(
                "0x{addr:04X}: demangle_addr_chip gave {result:?} 0x{logical:04X} chip {chip}, \
                 demangle_addr {plain:?}"
            ));
        }
    }

    verdict(errors)
}
