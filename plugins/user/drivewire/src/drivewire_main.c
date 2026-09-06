// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License
//
// DriveWire bridge plugin for One ROM.
//
// Bridges a DriveWire session between a CoCo/Dragon disk controller ROM and
// a real DriveWire server, over One ROM's RP2350 UART1 hardware (GPIO40 TX,
// GPIO41 RX - the One ROM Fire 28 Rev C SEL_A/SEL_B pads, which must be left
// unpopulated on this board for those pins to be free - see README) rather
// than the CoCo's own bit-banger serial port.
//
// Signalling rides the ROM address bus, the same idiom the host-control
// (RBCP) plugin uses: reads at addresses whose low byte carries a value are
// how the CPU "sneaks" data out over a bus it can otherwise only read from.
// Two knock sequences pick the session direction (no RBCP-style group/cmd
// byte is needed, since the direction is the only thing to select), followed
// by a 16-bit count N, high byte first (both directions) - matching the
// 0-65535 range dw.h's dwwrite()/dwread() are themselves typed for, so a
// caller can never ask this transport for more than it can carry:
//
//   write session (CoCo -> server): knock, count N, then N further
//   address-encoded reads, forwarded byte for byte to UART1 TX.  Streamed
//   with no buffering, so N's full 16-bit range is always safe here.
//
//   read session (server -> CoCo): knock, count N, capped in practice at
//   sizeof(s_read_buf) - see the main loop's own comment on why a larger N
//   is refused rather than served.  For each of the N bytes,
//   the plugin blocks on UART1 RX, writes the byte to a fixed logical "data"
//   address, then flips a fixed logical "status" address to a ready sentinel.
//   The CoCo spin-reads status until ready, then reads data to collect the
//   byte.  The plugin does not prepare the next byte until it observes the
//   CoCo's read of the data address, which is what stops it overwriting a
//   byte the CoCo has not yet collected.
//
// The status/data addresses are ordinary ROM offsets ($C100/$C101 in CPU
// space for this 8K 2764 image) - live hdbdos code, not spare padding.  What
// makes that safe is that reprogramming them is the *only* thing that
// touches served ROM content, and it is fully reversible: the plugin reads
// and caches the real bytes once at startup and restores them the moment
// each read session ends, so outside an active session the image is exactly
// what it was assembled as.  The knock sequences themselves are pure reads
// and never touch served content at all.
//
// This plugin does not support yielding: a pause mid-knock or mid-session
// would be indistinguishable from the CoCo simply pausing, and there is no
// way to resynchronise cleanly, so DriveWire sessions must not be interrupted
// by another plugin's exclusive-mode request.
//
// Build with DRIVEWIRE_TEST_PATTERN defined (make TEST_PATTERN=1) for a
// bring-up test build that skips the real protocol - see drivewire_main().

#include <stdint.h>
#include <stdbool.h>
#include "plugin.h"

#define RP235X
#define MCU_FLASH_SIZE_KB 2048
#define MCU_RAM_SIZE_KB 520
#define RP2350A
#include "onerom_metadata.h"
#include "reg-rp235x.h"

// ---------------------------------------------------------------------------
// Plugin header
// ---------------------------------------------------------------------------

void drivewire_main(
    ora_lookup_fn_t ora_lookup_fn,
    ora_plugin_type_t plugin_type,
    const ora_entry_args_t *entry_args
);
ORA_SECTION(".plugin_header")
const ora_plugin_header_t ora_plugin_header = {
    .magic         = ORA_PLUGIN_MAGIC,
    .api_version   = ORA_PLUGIN_VERSION_1,
    .major_version = MAJOR_VERSION,
    .minor_version = MINOR_VERSION,
    .patch_version = PATCH_VERSION,
    .build_version = BUILD_VERSION,
    .entry         = drivewire_main,
    .plugin_type   = ORA_PLUGIN_TYPE_USER,
    .sam_usage     = 255,
    .overrides1    = 0,
    .properties1   = 0,  // does not support yielding - see file header comment
    .min_fw_major_version = 0,
    .min_fw_minor_version = 7,
    .min_fw_patch_version = 1,
    .reserved = {0},
};

// ---------------------------------------------------------------------------
// Ring buffer
// ---------------------------------------------------------------------------

#define RING_ENTRIES_LOG2   6u                               // 64 entries
#define RING_DATA_SIZE      32u                               // 32 bits per entry
#define RING_MASK           ((1u << RING_ENTRIES_LOG2) - 1u)
#define RING_BUF_TYPE        uint32_t
_Static_assert(sizeof(RING_BUF_TYPE) * 8 == RING_DATA_SIZE, "RING_BUF_TYPE must match RING_DATA_SIZE");

ORA_SECTION(".ring_buf")
ORA_RING_BUF_DECLARE_32BIT(ring_buf, RING_ENTRIES_LOG2);

#define RING_BUF_CUR_READ_INDEX()   s_read_idx
#define RING_BUF_ADV_READ_INDEX()   s_read_idx = (s_read_idx + 1u) & RING_MASK
#define RING_BUF_CUR_WRITE_INDEX() \
    ((uint32_t)((volatile RING_BUF_TYPE *)*s_write_pos_ptr - \
                (volatile RING_BUF_TYPE *)ring_buf) & RING_MASK)
#define RING_BUF_GET_ENTRY(X) ((volatile RING_BUF_TYPE *)ring_buf)[(X)]

static volatile uint32_t * volatile *s_write_pos_ptr;
static uint32_t            s_read_idx;

// ---------------------------------------------------------------------------
// Knock sequences
// ---------------------------------------------------------------------------

#define KNOCK_LEN 8u
static const uint8_t s_knock_write[KNOCK_LEN] = "!DWSEND!";  // CoCo -> server
#ifndef DRIVEWIRE_TEST_PATTERN
static const uint8_t s_knock_read[KNOCK_LEN]  = "!DWRECV!";  // server -> CoCo
#endif

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

// Logical (chip) byte offsets used as the status/data channel - $C100/$C101
// in this 2764's CPU-visible address space.  Live hdbdos code; see the file
// header comment for why overwriting it here is safe.
#define DW_STATUS_ADDR   0x0100u
#define DW_DATA_ADDR     0x0101u

#define DW_STATUS_PENDING 0x00u
#define DW_STATUS_READY   0xFFu

// A third live ROM byte, deliberately in a different 256-byte page from
// DW_STATUS_ADDR/DW_DATA_ADDR above.  Written to DW_KNOCK_ACK_VALUE the
// instant a write-session knock matches (see drivewire_wait_for_knock()), so
// the CoCo's own bounded poll-and-retry in dwonewrite.asm's DWWrite can tell
// its knock actually landed and resend if it doesn't see this in time - only
// write knocks get this treatment, since the read direction's own knock has
// never been observed to go missing, unlike the write knock this guards.
//
// This cannot reuse DW_STATUS_ADDR/DW_DATA_ADDR themselves: the count and
// data bytes that follow a knock are sent the same way the knock is - as an
// address whose low byte IS the value, anywhere across the full $C100-$C1FF
// page (a count of 256 wire-encodes as 0, landing on $C100 exactly).  Only a
// full-address comparison against a byte outside that whole page can tell
// "the CoCo is still polling for the ack" apart from "the CoCo is now
// sending a real count/data byte that happens to have this low byte" - the
// same reason drivewire_wait_for_data_read() below compares full addresses,
// not low bytes, against DW_DATA_ADDR.
#define DW_KNOCK_ACK_ADDR  0x0200u
#define DW_KNOCK_ACK_VALUE 0xA5u

#define DW_UART_BASE     UART1_BASE
#define DW_UART_TX_GPIO  40u
#define DW_UART_RX_GPIO  41u
#ifndef DW_UART_BAUD
#define DW_UART_BAUD     921600u
#endif

// Bound on how long drivewire_uart_getc() will wait for a single RX byte -
// see its own comment for why an unbounded wait here can wedge core 1
// permanently.  A fixed iteration count, not derived from s_get_sysclk_mhz():
// getting this exactly right would need scaling by clock speed to keep the
// wall-clock duration constant, but the two failure modes are wildly
// asymmetric - too long merely delays recovery from a genuine non-response,
// while too short risks a spurious abort on a slow-but-working one (e.g.
// FujiNet's SD card access taking a while under wear-levelling GC).  Sized
// generously - tens of millions of iterations, comfortably multiple seconds
// even on an overclocked RP2350 - so it only ever fires when nothing is
// coming rather than shaving margin off any legitimately slow response.
#define DW_UART_GETC_TIMEOUT 300000000ul

typedef enum {
    SESSION_WRITE,
    SESSION_READ,
} session_dir_t;

// ---------------------------------------------------------------------------
// Looked-up API functions
// ---------------------------------------------------------------------------

static ora_log_fn_t                         s_log;
static ora_err_log_fn_t                     s_err_log;
static ora_demangle_addr_fn_t               s_demangle;
static ora_reprogram_ram_rom_slot_fn_t      s_reprogram;
static ora_read_ram_rom_slot_fn_t           s_read_slot;
static ora_get_active_ram_slot_fn_t         s_get_active_slot;
static ora_get_sysclk_mhz_fn_t              s_get_sysclk_mhz;
static ora_start_address_monitor_fn_t       s_start_monitor;
static ora_get_address_monitor_ring_write_pos_fn_t s_get_write_pos;
static ora_set_status_led_fn_t              s_set_status_led;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t s_active_slot;
static uint8_t s_orig_status_byte;
static uint8_t s_orig_data_byte;
static uint8_t s_orig_ack_byte;

// Holds a full read session's response, drained from UART1 before any of the
// CoCo handshake starts - see drivewire_do_read().  256 is the protocol's
// maximum single-session count (0 on the wire means 256).
static uint8_t s_read_buf[256];

// How many bytes at the front of s_read_buf were opportunistically drained
// from UART1 by drivewire_next_addr() before drivewire_do_read() was even
// entered - see its own comment.  Reset once a session finishes with it (a
// read session in drivewire_do_read() itself; a write session explicitly, in
// drivewire_wait_for_knock() below, before the next wait starts) so stale
// bytes from one session's window can never bleed into the next.
static uint16_t s_read_buf_filled;

// Main session loop's own state, one level up from drivewire_do_read()'s own
// variables above - these cover write sessions and knock-waiting too, so a
// stall outside drivewire_do_read() entirely (e.g. waiting for whatever
// knock is supposed to follow a just-completed session) is still visible.
// phase: 0=waiting for knock, 1=reading the count byte, 2=in a write
// session, 3=in a read session.  session_count increments once per detected
// knock (so once per write or read session - a full sector "block" is
// several of these: the request, the 256-byte data, the 2-byte checksum,
// and the 1-byte ack, each a separate session).
static volatile uint8_t  s_debug_main_phase;
static volatile uint32_t s_debug_session_count;
// Increments on every single call to drivewire_next_addr(), the most
// fundamental, frequently-hit function in the plugin - a definitive "is
// anything at all still executing" signal, independent of which higher-level
// function (do_read, do_write, wait_for_knock, wait_for_data_read) is
// currently active.  If two peeks a few seconds apart show the exact same
// value here, the plugin's core (core 0) is not merely busy-looping
// somewhere - it has genuinely stopped running any code at all.
static volatile uint32_t s_debug_heartbeat;

// Bring-up diagnostic only: every matched knock's data, for reading back via
// `onerom inspect peek memory` instead of manually transcribing UART output
// (error-prone - single-character transcription slips, e.g. a stray '9' for
// a byte that can only ever be '0' or '1', are otherwise indistinguishable
// from a genuine mismatch).  session is which session this knock started -
// same numbering as s_debug_session_count once the caller increments it.
//
// TEMPORARILY REMOVED (2026-09) to free RAM budget for the checksum-error
// investigation's own instrumentation - see s_debug_sticky_already's own
// comment and the "~20-byte margin" note on init_data_bss() for why every
// byte here is contested.  Re-add once that investigation no longer needs
// the room: static volatile uint32_t s_debug_knock_session; static volatile
// uint8_t s_debug_knock_match_write; static volatile uint8_t
// s_debug_knock_match_read; static volatile uint8_t
// s_debug_knock_window[KNOCK_LEN]; plus the write-site in
// drivewire_wait_for_knock() (removed alongside, same reason).

// Bring-up diagnostic only: the byte count of the most recently *completed*
// write or read session (s_debug_main_phase says which kind), read back via
// peek instead of a UART dump - a UART dump here previously corrupted the
// live protocol on the wire (confirmed: a checksum error of exactly 0x0D0A,
// i.e. "\r\n", turned up server-side, matching this dump's own line prefix).
static volatile uint16_t s_debug_last_count;

// Set the instant a session's count byte is known, before drivewire_do_write()
// /drivewire_do_read() is even called - unlike s_debug_last_count above,
// this is visible while a session is still stuck in progress, to see what
// count a hung read/write was actually trying to handle.
static volatile uint16_t s_debug_current_count;

// Checksum of what drivewire_do_read() actually received over UART1 into
// s_read_buf, computed right after its own initial drain completes and
// before any of it is relayed to the CoCo - same add-with-carry algorithm as
// dwoneread.asm's DWRCODE.  Compares against the DriveWire server's own
// reported checksum to tell which half of the pipeline a mismatch is in:
// matching means the bug is in how this side relays s_read_buf to the CoCo
// over the ROM bus; not matching means the bug is upstream, in what UART1
// actually received.
static volatile uint16_t s_debug_received_checksum;

// Bring-up diagnostic only: a "sticky" copy of s_debug_received_checksum
// (plus which session it belongs to), latched only when count==256 - the
// sector-read case this investigation cares about.  s_debug_received_checksum
// itself gets overwritten by *any* read session, including the small 1-byte
// "get error code" read HREAD issues immediately after a checksum mismatch -
// by the time a peek lands, that follow-up (or the next thing after it) has
// usually already clobbered the value this needs to see.  This survives
// exactly those follow-up exchanges, so a peek only has to beat the *next*
// 256-byte read, not the next read of any size - a far more generous window.
static volatile uint16_t s_debug_sticky_received_checksum;
static volatile uint32_t s_debug_sticky_session;

// Bring-up diagnostic only: how many bytes were already sitting in
// s_read_buf from drivewire_next_addr()'s opportunistic pre-drain at the
// moment this session's own top-off loop is about to run - i.e. "already"
// itself, latched before that (blocking) drivewire_uart_getc() loop can run.
// Sticky and count==256-gated for the same reason as the checksum above:
// distinguishes a shortfall that already exists before topping off even
// starts (implicating UART1 reception or the pre-drain itself) from one that
// only appears after topping off completes without blocking (implicating
// something in how the completed buffer gets summed or relayed instead).
static volatile uint16_t s_debug_sticky_already;

// Bring-up diagnostic only: UART1's own receive-status flags (RSR - framing/
// parity/break/overrun, see UART_RSR_OE and friends in reg-rp235x.h), snapshot
// right after this session's own drain finishes, latched alongside the sticky
// checksum above (same count==256 gate, same reasoning for why it needs to be
// sticky).  RSR is cumulative and sticky in the hardware too - cleared once at
// the top of the main loop, before this session's knock-wait even starts (see
// that clear's own comment) - so a non-zero value here means UART1 flagged at
// least one of these errors somewhere between this session's knock and the
// end of its drain.  drivewire_uart_getc() and the opportunistic drain in
// drivewire_next_addr() both discard the PL011's per-byte error bits by
// casting UART_DR straight to uint8_t, so this is currently the only way to
// see that anything went wrong at the UART hardware level at all - a missing
// byte with no corresponding flag here means the loss did not happen there.
static volatile uint32_t s_debug_sticky_rsr;

// Bring-up diagnostic only, the write-session counterpart to
// s_debug_received_checksum above: what drivewire_do_write() actually
// forwarded to UART1 TX, captured as the ROM-bus relay puts each byte on the
// wire (not what the CoCo intended to send, which this side has no
// independent way to know) - same running-sum algorithm as
// drivewire_checksum() on the FujiNet side and dwoneread.asm's DWRCODE, so it
// is directly comparable against whatever the server reports back for a
// write it complains about (e.g. the 2-byte read-checksum echo in HREAD).
// The raw bytes matter as much as the sum here: a small write (the common
// case - an opcode+subcommand, or that same 2-byte echo) can be read back
// verbatim instead of just its sum, which pins down a single corrupted byte
// exactly rather than merely detecting that the sum is off.  Fixed-size and
// deliberately small: a longer write (e.g. OPEN_DIRECTORY's 259 bytes) only
// has its first few bytes captured, which is exactly as informative for
// spotting *whether* the relay corrupts something as capturing all of it
// would be - the open question here is root cause, not a full byte dump.
#define DEBUG_WRITE_BYTES 8u
static volatile uint8_t  s_debug_write_bytes[DEBUG_WRITE_BYTES];
static volatile uint16_t s_debug_write_byte_count;
static volatile uint16_t s_debug_write_checksum;

// ---------------------------------------------------------------------------
// UART1
// ---------------------------------------------------------------------------

// Brings up UART1 on GPIO40 (TX) / GPIO41 (RX) at DW_UART_BAUD, 8-N-1.
//
// GPIO40/41 are this board's SEL_A/SEL_B image-select pads.  The firmware's
// own disable_sel_pins() clears their pull-ups/downs once boot's jumper read
// is done, and the core firmware deliberately excludes them from its GPIO
// use tracking (see firmware/src/plugin.c) - they are meant to be reclaimed
// by soldering a wire to a pad whose jumper has been removed.  That physical
// removal is a precondition this plugin cannot verify or enforce: if the
// jumper is still populated, driving these pins as UART1 will contend with
// it.
static void drivewire_uart_init(void) {
    // clk_peri feeds UART1's baud rate generator, and the baud divisor below
    // is computed against clk_sys - so this must guarantee AUXSRC selects
    // clk_sys (value 0), not merely assume it already does because that is
    // its reset value.  A plain write (not an atomic alias) is correct and
    // sufficient here: nothing else in this codebase touches CLOCK_PERI_CTRL,
    // so there is no concurrent writer to race with, and writing the whole
    // register is what lets this pin down every field - AUXSRC included -
    // rather than only the enable bit.  Its integer divider resets to /1 and
    // nothing touches it either, so clk_peri ends up equal to clk_sys.
    //
    // This must happen *before* the reset dance below: UART1's RESET_DONE bit
    // synchronises through the peripheral's own clock domain (clk_peri), so
    // waiting on it while clk_peri is still stopped hangs forever.  The SDK
    // never has to think about this ordering because its own boot sequence
    // already has clk_peri running long before any call to uart_init().
    CLOCK_PERI_CTRL = CLOCK_PERI_CTRL_ENABLE;

    // Force UART1 through a full reset cycle rather than just clearing the
    // reset bit and assuming it was already set.  That assumption only holds
    // at cold power-on; by the time a plugin runs, the core firmware has
    // been executing for a while and UART1's actual state is not guaranteed.
    // The SDK's own uart_init() does the same reset-then-unreset cycle.
    // RESETS is shared chip-wide, so both directions must go through the
    // atomic SET/CLR aliases, never a plain read-modify-write on RESET_RESET
    // - see reg-rp235x.h.
    RESET_RESET_SET = RESET_UART1;
    RESET_RESET_CLR = RESET_UART1;
    while (!(RESET_DONE & RESET_UART1)) { }

    // Route the pins to UART1.
    GPIO_CTRL(DW_UART_TX_GPIO) = GPIO_CTRL_FUNC_UART;
    GPIO_CTRL(DW_UART_RX_GPIO) = GPIO_CTRL_FUNC_UART;

    // Clear any leftover jumper-sensing pulls (disable_sel_pins() already did
    // this at boot, but that ran before we knew these pins would become a
    // UART) and make sure RX's input buffer is enabled.  GPIO40/41 are
    // 3.3V-only ADC pins - whatever is on the other end of this link must be
    // 3.3V logic, never 5V TTL.
    //
    // Also explicitly clear PAD_ISO on both pins.  disable_sel_pins() never
    // touches it (it only clears the pulls), and nothing else in the core
    // firmware manages these two pads at all - they are deliberately
    // excluded from its normal GPIO handling.  If ISO is still set from
    // power-up, the pad stays electrically isolated no matter how correctly
    // UART1 itself is configured: funcsel, baud, everything else can be
    // exactly right and still never reach the physical pin.
    GPIO_PAD(DW_UART_RX_GPIO) = (GPIO_PAD(DW_UART_RX_GPIO) | PAD_INPUT) & ~(uint32_t)(PAD_PU | PAD_PD | PAD_OUTPUT_DISABLE | PAD_ISO);

    // TX also gets slew rate and drive strength pinned down explicitly,
    // rather than left at whatever setup_sel_pins() configured this pad to
    // for jumper sensing.  Slow slew + a modest 4mA is deliberately gentle:
    // this pad was never designed as an active driver, and fast edges into
    // an unterminated/unexpected trace are a classic way to get ringing
    // that looks nothing like clean UART framing.
    //
    // PAD_INPUT (the pad's input-enable bit) is set here too, even though TX
    // is output-only - confirmed against the SDK's own gpio_set_function(),
    // which sets it unconditionally on every pin it configures, output or
    // not.  Nothing upstream of this function ever sets it on this pin, so
    // without this it stays whatever setup_sel_pins() left it as.
    GPIO_PAD(DW_UART_TX_GPIO) = (GPIO_PAD(DW_UART_TX_GPIO) &
        ~(uint32_t)(PAD_PU | PAD_PD | PAD_OUTPUT_DISABLE | PAD_ISO | PAD_SLEW_FAST | PAD_DRIVE(PAD_DRIVE_MASK)))
        | PAD_DRIVE(PAD_DRIVE_4MA) | PAD_INPUT;

    // BAUDDIV = clk_peri / (16 * baud).  IBRD is its integer part; FBRD is
    // the fractional part scaled to 64ths.  Compute both via a single
    // 64-scaled quantity (BAUDDIV * 64 == 4 * clk_peri / baud), rounded to
    // the nearest 64th rather than truncated.
    uint32_t clk_peri_hz = s_get_sysclk_mhz() * 1000000u;
    uint64_t scaled = ((uint64_t)clk_peri_hz * 4u + (DW_UART_BAUD / 2u)) / DW_UART_BAUD;
    uint32_t ibrd = (uint32_t)(scaled >> 6);
    uint32_t fbrd = (uint32_t)(scaled & 0x3Fu);
    if (ibrd == 0u) {
        // Requested baud is unreachable at this clk_peri - clamp to the
        // fastest representable rate rather than program a divide-by-zero.
        ibrd = 1u;
        fbrd = 0u;
    }

    UART_REG(DW_UART_BASE, UART_CR_OFFSET) = 0;  // disable while reconfiguring
    UART_REG(DW_UART_BASE, UART_IBRD_OFFSET) = ibrd;
    UART_REG(DW_UART_BASE, UART_FBRD_OFFSET) = fbrd;
    // 8 data bits, no parity, one stop bit (LCR_H bits not set below default
    // to 0: STP2 clear = 1 stop bit, PEN clear = no parity), FIFOs enabled.
    UART_REG(DW_UART_BASE, UART_LCR_H_OFFSET) = UART_LCR_H_WLEN_8 | UART_LCR_H_FEN;
    UART_REG(DW_UART_BASE, UART_CR_OFFSET) = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;

    // Clear any framing/break/overrun flags RSR latched from line noise
    // while the pad was still settling into its final electrical state
    // above (writing this address, ECR, clears rather than reads) - RSR is
    // sticky, so without this, startup glitches unrelated to any real
    // session would sit there indefinitely and look identical to a live
    // error later on.
    UART_REG(DW_UART_BASE, UART_RSR_OFFSET) = 0;
}

static void drivewire_uart_putc(uint8_t b) {
    while (UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_TXFF) { }
    UART_REG(DW_UART_BASE, UART_DR_OFFSET) = b;
}

#ifndef DRIVEWIRE_TEST_PATTERN
// Returns false on timeout (see DW_UART_GETC_TIMEOUT) without touching *out -
// the caller must treat a false return as "no byte", not as byte 0x00, since
// every value 0-255 is a valid received byte and cannot itself signal timeout.
static bool drivewire_uart_getc(uint8_t *out) {
    uint32_t timeout = DW_UART_GETC_TIMEOUT;
    while (UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_RXFE) {
        if (--timeout == 0u) {
            return false;
        }
    }
    *out = (uint8_t)UART_REG(DW_UART_BASE, UART_DR_OFFSET);
    return true;
}
#endif

// ---------------------------------------------------------------------------
// Address bus signalling
// ---------------------------------------------------------------------------

// Blocks until the next CS-active address is captured, and returns its
// demangled logical (byte) address.  Discards captures that fail to
// demangle (CS glitches etc).
//
// This board only ever serves a 2764 on the 28-pin variant, which has no
// unobserved address bits, so the logical (chip-image) address space and the
// observed (bus) address space this signalling rides in are identical - see
// ora_demangle_addr_fn_t / ora_demangle_observed_addr_fn_t in api.h.  A
// 40-pin target would need to use the latter for signalling.
#ifndef DRIVEWIRE_TEST_PATTERN
static uint32_t drivewire_next_addr(void) {
    for (;;) {
        s_debug_heartbeat++;

        // Opportunistically drain any UART1 RX byte that has already
        // arrived, on every pass through this loop - not just while the
        // ring buffer is empty (the CoCo's own instruction fetches, e.g.
        // while copying DWRead's relocatable body onto the stack, are ROM
        // reads too, so the buffer is rarely actually empty during exactly
        // the window this matters, and gating on that meant this almost
        // never ran).  Not just once a read session officially starts in
        // drivewire_do_read() either: the DriveWire server responds to a
        // request as soon as it parses it, and at 921600 baud its entire
        // 256-byte response can arrive and overrun the 32-byte hardware RX
        // FIFO well before the CoCo even finishes sending the knock that (as
        // far as this plugin previously knew) starts a read session.
        if (s_read_buf_filled < sizeof(s_read_buf) &&
            !(UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_RXFE)) {
            s_read_buf[s_read_buf_filled++] =
                (uint8_t)UART_REG(DW_UART_BASE, UART_DR_OFFSET);
        }

        if (RING_BUF_CUR_READ_INDEX() == RING_BUF_CUR_WRITE_INDEX()) {
            ORA_TEST_YIELD();
            continue;
        }
        uint32_t phys = (uint32_t)RING_BUF_GET_ENTRY(s_read_idx);
        RING_BUF_ADV_READ_INDEX();

        uint32_t logical;
        if (s_demangle(phys, &logical, 1) == ORA_RESULT_OK) {
            return logical;
        }
        // CS inactive or demangle error: discard and try next entry.
    }
}
#endif // !DRIVEWIRE_TEST_PATTERN

#ifdef DRIVEWIRE_TEST_PATTERN
// Bring-up-test-only variant of drivewire_next_addr(): if the ring buffer
// sits empty for a while, emits a fixed heartbeat marker byte before going
// back to waiting.  Without this, "the CoCo isn't sending anything" and
// "the address monitor isn't capturing anything at all" both look
// identical from outside (LED off, UART silent forever) - the caller's
// relay loop can't emit a byte it never receives.  A repeating marker with
// nothing else means the address monitor itself isn't seeing captures;
// total silence (not even the marker) means something else broke.
#define DRIVEWIRE_TEST_IDLE_MARKER   0xAAu
#define DRIVEWIRE_TEST_IDLE_SPINS    2000000u
static uint32_t drivewire_test_next_addr(void) {
    uint32_t idle_spins = 0;
    for (;;) {
        if (RING_BUF_CUR_READ_INDEX() == RING_BUF_CUR_WRITE_INDEX()) {
            if (++idle_spins == DRIVEWIRE_TEST_IDLE_SPINS) {
                drivewire_uart_putc(DRIVEWIRE_TEST_IDLE_MARKER);
                idle_spins = 0;
            }
            ORA_TEST_YIELD();
            continue;
        }
        uint32_t phys = (uint32_t)RING_BUF_GET_ENTRY(s_read_idx);
        RING_BUF_ADV_READ_INDEX();

        uint32_t logical;
        if (s_demangle(phys, &logical, 1) == ORA_RESULT_OK) {
            return logical;
        }
        // CS inactive or demangle error: discard and try next entry.
    }
}
#endif

#ifndef DRIVEWIRE_TEST_PATTERN
// Waits for either knock sequence, returning which one matched.
//
// The ORA knock API (ora_wait_for_knock_fn_t) matches exactly one configured
// sequence per call and blocks until it does, so it cannot itself watch for
// either of two alternatives - whichever sequence we asked it for would never
// return if the other one arrived instead.  This does the same masked-window
// matching by hand, over the same demangled address stream, against both
// sequences at once.
static session_dir_t drivewire_wait_for_knock(void) {
    // Deliberately does not touch s_read_buf_filled: drivewire_next_addr()'s
    // opportunistic drain runs continuously, including during the write
    // session that precedes a read (e.g. the checksum write ahead of an
    // ack-read), so the server's reply can already be sitting in s_read_buf
    // by the time this knock is even seen.  Discarding it here would lose it
    // permanently - drivewire_do_read() would then block forever on a fresh
    // UART byte that already came and went.  s_read_buf itself stays
    // consistent without help: drivewire_do_write() never touches it, and
    // drivewire_do_read() zeroes the count only once it has consumed exactly
    // that many bytes.
    uint8_t window[KNOCK_LEN] = {0};
    for (;;) {
        uint8_t b = (uint8_t)(drivewire_next_addr() & 0xFFu);
        for (unsigned i = 0; i < KNOCK_LEN - 1u; i++) {
            window[i] = window[i + 1u];
        }
        window[KNOCK_LEN - 1u] = b;

        bool match_write = true, match_read = true;
        for (unsigned i = 0; i < KNOCK_LEN; i++) {
            if (window[i] != s_knock_write[i]) match_write = false;
            if (window[i] != s_knock_read[i])  match_read = false;
        }
        if (match_write) {
            // Acknowledge as fast as possible - dwonewrite.asm's DWWrite
            // polls DW_KNOCK_ACK_ADDR for this exact value, with a timeout
            // that resends the whole knock if it never sees it (see
            // DW_KNOCK_ACK_ADDR's own comment).  drivewire_do_write()
            // restores the real ROM content here once the session ends,
            // matching how drivewire_do_read() already restores both
            // channel addresses at the end of a read session.
            uint8_t ack = DW_KNOCK_ACK_VALUE;
            s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &ack, 1u, 1u);
            return SESSION_WRITE;
        }
        if (match_read)  return SESSION_READ;
    }
}
#endif // !DRIVEWIRE_TEST_PATTERN

#ifndef DRIVEWIRE_TEST_PATTERN
// Blocks until the CoCo reads the data channel address - the read session's
// acknowledgement that it has collected the current byte, and thus safe to
// serve the next one.
static void drivewire_wait_for_data_read(void) {
    for (;;) {
        if (drivewire_next_addr() == DW_DATA_ADDR) return;
    }
}

// Like drivewire_next_addr(), but silently discards any address exactly
// equal to DW_KNOCK_ACK_ADDR before returning - see that macro's own
// comment for why this must be a full-address compare, not the usual
// low-byte mask.  Only a write session's own knock ever writes an ack there,
// so this is a no-op for read sessions: the CoCo never generates that
// address at all outside DWWrite's own poll loop.
static uint32_t drivewire_next_addr_skip_ack(void) {
    uint32_t addr;
    do {
        addr = drivewire_next_addr();
    } while (addr == DW_KNOCK_ACK_ADDR);
    return addr;
}

// ---------------------------------------------------------------------------
// Sessions
// ---------------------------------------------------------------------------

static void drivewire_do_write(uint16_t count) {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < count; i++) {
        uint8_t b = (uint8_t)(drivewire_next_addr() & 0xFFu);
        if (i < DEBUG_WRITE_BYTES) {
            s_debug_write_bytes[i] = b;
        }
        checksum = (uint16_t)(checksum + b);
        drivewire_uart_putc(b);
    }
    s_debug_write_byte_count = (count < DEBUG_WRITE_BYTES) ? count : DEBUG_WRITE_BYTES;
    s_debug_write_checksum   = checksum;

    // Restore the real ROM content at the ack address now the session is
    // over - drivewire_wait_for_knock() reprogrammed it to ack this
    // session's own knock, and a write session never touches
    // DW_STATUS_ADDR/DW_DATA_ADDR at all, so only this one address needs it.
    s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u, 1u);
}

static void drivewire_do_read(uint16_t count) {
    // Finish draining the whole response from UART1 before any of the CoCo
    // handshake below - drivewire_next_addr() may already have opportunistically
    // pre-drained some of it into s_read_buf while this session's knock was
    // still being waited for (see its own comment for why that matters).
    // Whatever it's got is already correct and in order; only read fresh
    // bytes for the rest.  The remaining reads have no handshake overhead
    // (drivewire_uart_getc() is a handful of cycles), so this drains far
    // faster than bytes can arrive even at 921600 baud.
    uint16_t already = (s_read_buf_filled < count) ? s_read_buf_filled : count;
    // Bring-up diagnostic only - see s_debug_sticky_already's own comment.
    // Latched here, before the topping-off loop below (which blocks) can run.
    if (count == 256u) {
        s_debug_sticky_already = already;
    }
    for (uint16_t i = already; i < count; i++) {
        if (!drivewire_uart_getc(&s_read_buf[i])) {
            // Timed out: FujiNet never sent the rest of this response - most
            // plausibly because an earlier, corrupted write left it still
            // blocked reading a request that will never complete (see
            // DW_UART_GETC_TIMEOUT's own comment).  Bail out to the next
            // knock rather than wedging core 1 here forever: safe to do with
            // no cleanup, since DW_STATUS_ADDR/DW_DATA_ADDR are still exactly
            // as the previous session's own cleanup left them - this session
            // has not touched either yet.  s_read_buf_filled is reset so a
            // partially-filled buffer from this abort cannot be mistaken for
            // a genuine pre-drain by whatever session starts next.
            s_read_buf_filled = 0;
            s_err_log("DriveWire: timed out waiting for UART1 RX, aborting session");
            return;
        }
    }
    s_read_buf_filled = 0;

    // Bring-up diagnostic only - see s_debug_sticky_rsr's own comment.  Read
    // right after the drain above finishes, before anything else touches
    // UART1, so this reflects only this session's own reception.
    uint32_t rsr = UART_REG(DW_UART_BASE, UART_RSR_OFFSET);

    // Bring-up diagnostic only - see s_debug_received_checksum's own comment.
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < count; i++) {
        checksum = (uint16_t)(checksum + s_read_buf[i]);
    }
    s_debug_received_checksum = checksum;
    if (count == 256u) {
        s_debug_sticky_received_checksum = checksum;
        s_debug_sticky_rsr               = rsr;
        s_debug_sticky_session            = s_debug_session_count;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint8_t b = s_read_buf[i];

        if (s_reprogram(s_active_slot, DW_DATA_ADDR, &b, 1u, 1u) != ORA_RESULT_OK) {
            s_err_log("DriveWire: reprogram of data address failed");
            return;
        }

        // Discard whatever the CoCo's own status poll loop already captured
        // into the ring buffer while this iteration was preparing the byte
        // above - that poll loop runs continuously, completely unthrottled,
        // and every one of those reads lands in the same 64-entry ring
        // buffer drivewire_wait_for_data_read() reads from below.  Must run
        // *before* ready is set: the CoCo is polling continuously, so if it
        // reacts before this line runs, the genuine data-read capture would
        // already be sitting in the buffer and this would discard that too.
        s_read_idx = RING_BUF_CUR_WRITE_INDEX();

        uint8_t ready = DW_STATUS_READY;
        if (s_reprogram(s_active_slot, DW_STATUS_ADDR, &ready, 1u, 1u) != ORA_RESULT_OK) {
            s_err_log("DriveWire: reprogram of status address failed");
            return;
        }

        drivewire_wait_for_data_read();

        uint8_t pending = DW_STATUS_PENDING;
        s_reprogram(s_active_slot, DW_STATUS_ADDR, &pending, 1u, 1u);
    }

    // Restore the real ROM content at both channel addresses now the session
    // is over, so the served image is back to exactly what it was assembled
    // as until the next read session.
    s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u, 1u);
    s_reprogram(s_active_slot, DW_DATA_ADDR, &s_orig_data_byte, 1u, 1u);
}
#endif // !DRIVEWIRE_TEST_PATTERN

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

static void drivewire_setup(ora_lookup_fn_t ora_lookup_fn) {
    s_log             = ora_lookup_fn(ORA_ID_LOG);
    s_err_log         = ora_lookup_fn(ORA_ID_ERR_LOG);
    s_demangle        = ora_lookup_fn(ORA_ID_DEMANGLE_ADDR);
    s_reprogram       = ora_lookup_fn(ORA_ID_REPROGRAM_RAM_ROM_SLOT);
    s_read_slot       = ora_lookup_fn(ORA_ID_READ_RAM_ROM_SLOT);
    s_get_active_slot = ora_lookup_fn(ORA_ID_GET_ACTIVE_RAM_SLOT);
    s_get_sysclk_mhz  = ora_lookup_fn(ORA_ID_GET_SYSCLK_MHZ);
    s_set_status_led  = ora_lookup_fn(ORA_ID_SET_STATUS_LED);

    ora_setup_address_monitor_fn_t setup_monitor = ora_lookup_fn(ORA_ID_SETUP_ADDRESS_MONITOR);
    s_start_monitor = ora_lookup_fn(ORA_ID_START_ADDRESS_MONITOR);
    s_get_write_pos = ora_lookup_fn(ORA_ID_GET_ADDRESS_MONITOR_RING_WRITE_POS);

    // Address monitoring starts as early in setup as possible, ahead of the
    // slot calls and UART1 bring-up below.  The CoCo's own reset is not
    // gated on any of this plugin's init, so every cycle spent here before
    // ora_start_address_monitor() is a window in which its first DriveWire
    // knock can cross the ROM bus unseen and be lost with no trace (see
    // ORA_ID_START_ADDRESS_MONITOR's doc comment in api.h).  Confirmed in
    // practice: with this last, the CoCo's first (write) knock was missed on
    // every cold boot, deterministically, while the second (read) knock - by
    // which point monitoring was long since running - never was.
    //
    // An earlier attempt at this same reordering produced a reproducible
    // HardFault (INVSTATE) inside ora_read_ram_rom_slot's own call chain -
    // root-caused as the user plugin's stack genuinely overflowing into live
    // .bss, not an ordering hazard as such: pio_setup_address_monitor()'s own
    // 208-byte frame plus this file's SRAM debug instrumentation had eaten
    // the plugin's ~256-byte stack margin down to a negative number,
    // regardless of call order. Trimming that instrumentation (see the
    // deleted per-byte read-session debug vars) reopened a ~20-byte positive
    // margin, confirmed by measuring the stack pointer directly at the time.
    bool monitor_ok = false;
    ora_result_t monitor_rc = setup_monitor(
        ring_buf,
        RING_ENTRIES_LOG2,
        ORA_MONITOR_MODE_CONTROL,
        RING_DATA_SIZE,
        NULL
    );
    if (monitor_rc == ORA_RESULT_OK) {
        s_write_pos_ptr = s_get_write_pos();
        if (s_write_pos_ptr != NULL) {
            s_start_monitor();
            monitor_ok = true;
        }
    }

    ora_result_t active_slot_rc = s_get_active_slot(&s_active_slot);

    if (active_slot_rc != ORA_RESULT_OK) {
        s_err_log("DriveWire: no active RAM slot at startup");
        s_active_slot = 0u;
    }

    // Snapshot the real bytes at the channel addresses once, so sessions can
    // restore them afterwards - see the file header comment.
    ora_result_t snap1_rc = s_read_slot(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u);
    ora_result_t snap2_rc = s_read_slot(s_active_slot, DW_DATA_ADDR, &s_orig_data_byte, 1u);
    ora_result_t snap3_rc = s_read_slot(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u);

    if (snap1_rc != ORA_RESULT_OK || snap2_rc != ORA_RESULT_OK || snap3_rc != ORA_RESULT_OK) {
        s_err_log("DriveWire: failed to snapshot channel address content");
    }

    // UART1 last: no dependency on the monitor or slot calls above, and
    // gating it behind those succeeding would mean a failure there silently
    // skips UART bring-up too - from outside, indistinguishable from "setup
    // succeeded, nothing to relay yet".
    drivewire_uart_init();

#ifdef DRIVEWIRE_TEST_PATTERN
    // Marker proving execution got this far, now that UART1 exists to prove
    // it - the three-blink startup sequence's own last action is "LED off",
    // so "LED off" alone does not prove drivewire_setup() ever ran.
    for (int i = 0; i < 4; i++) drivewire_uart_putc(0x55u);
    if (monitor_ok) {
        // Reuses checkpoint 5's old value/count - monitor setup, write-pos
        // lookup and start_monitor() all succeeded.
        for (int i = 0; i < 8; i++) drivewire_uart_putc(0x99u);
    } else if (monitor_rc != ORA_RESULT_OK) {
        // Reuses checkpoint 3's old value/count/error-byte convention.
        for (int i = 0; i < 6; i++) drivewire_uart_putc(0x77u);
        drivewire_uart_putc((uint8_t)monitor_rc);
    } else {
        // Reuses checkpoint 4's old value/count - setup_monitor() succeeded
        // but the write-position lookup failed.
        for (int i = 0; i < 7; i++) drivewire_uart_putc(0x88u);
    }
    // Checkpoints 1a/1b/1c - get_active_slot()/read_slot() x2 - held since
    // they ran before UART1 existed.
    drivewire_uart_putc(0x61u);
    for (int i = 0; i < 2; i++) drivewire_uart_putc(0x62u);
    for (int i = 0; i < 3; i++) drivewire_uart_putc(0x63u);
#endif

    if (!monitor_ok) {
        if (monitor_rc != ORA_RESULT_OK) {
            s_err_log("DriveWire: address monitor setup failed %d", monitor_rc);
        } else {
            s_err_log("DriveWire: failed to get ring buffer write position");
        }
        return;
    }

    s_log("DriveWire: ready, awaiting knock");
}

// ---------------------------------------------------------------------------
// Plugin entry point
// ---------------------------------------------------------------------------

// Place the plugin's initialised data and clear its zeroed data.
//
// A plugin owns its own RAM sections (see firmware/ora/plugin.h), and the
// static RAM it is handed holds whatever the firmware last left there - so a
// static relying on zero initialisation starts with garbage in it.  This bit
// drivewire directly: s_read_idx (and friends) came up non-zero, so the very
// first ring-buffer poll indexed with an unmasked garbage value and bus
// faulted.  Matches upstream commit e549e09 ("Initialize host-control
// plugin's BSS"), which hit the same bug in a different plugin.
//
// A host test build's data belongs to the host process, and these symbols do
// not exist there, so the body is compiled out rather than skipped at run time.
static void init_data_bss(void) {
#if !defined(ORA_HOST_TEST)
    extern uint32_t __ramfunc_start;
    extern uint32_t __ramfunc_end;
    extern uint32_t __ramfunc_load;
    extern uint32_t __data_start;
    extern uint32_t __data_end;
    extern uint32_t __data_load;
    extern uint32_t __bss_start;
    extern uint32_t __bss_end;

    // Copy .ramfunc from LMA (flash) to VMA (RAM)
    uint32_t *src = &__ramfunc_load;
    uint32_t *dst = &__ramfunc_start;
    while (dst < &__ramfunc_end) {
        *dst++ = *src++;
    }

    // Copy .data from LMA (flash) to VMA (RAM)
    src = &__data_load;
    dst = &__data_start;
    while (dst < &__data_end) {
        *dst++ = *src++;
    }

    // Zero .bss
    dst = &__bss_start;
    while (dst < &__bss_end) {
        *dst++ = 0;
    }

    // ring_buf lives in its own .ring_buf linker section - placed first in
    // the data region specifically so its 256-byte alignment requirement is
    // met by DATA_BASE itself (see drivewire_plugin.ld) - not in .bss, so
    // the zeroing above never touches it.  Reproduced 3x identically
    // (DRIVE=0xCF, LSN=0x707172 relayed as a live request) before this fix:
    // fully deterministic garbage, consistent with genuinely uninitialised
    // SRAM rather than a timing race, read back out once real captures
    // start landing in slots this never cleared.
    for (unsigned i = 0; i < (1u << RING_ENTRIES_LOG2); i++) {
        ((volatile uint32_t *)ring_buf)[i] = 0;
    }
#endif // !ORA_HOST_TEST
}

void drivewire_main(
    ora_lookup_fn_t         ora_lookup_fn,
    ora_plugin_type_t       plugin_type,
    const ora_entry_args_t *entry_args
) {
    // Before anything reads a static.
    init_data_bss();

    (void)plugin_type;
    (void)entry_args;

#ifdef DRIVEWIRE_TEST_UART_ONLY
    // Isolates UART1 bring-up from everything else: no address monitor, no
    // ROM-bus signalling, no knock logic - just bring up UART1 and blast a
    // repeating incrementing pattern forever, completely independent of the
    // CoCo/hdbdos.  If nothing appears on the scope with this, the problem
    // is in UART1's own register-level bring-up, GPIO40/41 funcsel/pads, or
    // the physical wiring/probe - not the address monitor or knock-matching
    // logic, which this bypasses entirely.
    s_get_sysclk_mhz = ora_lookup_fn(ORA_ID_GET_SYSCLK_MHZ);
    drivewire_uart_init();
    uint8_t test_byte = 0;
    for (;;) {
        drivewire_uart_putc(test_byte);
        test_byte++;
    }
#endif

#ifdef DRIVEWIRE_TEST_UART_SPACED
    // Simplest possible communications test: bring up UART1 and send one
    // well-known byte (0x55 = 01010101) at a time, with a gap of roughly 8
    // bit periods between each - a separate, easily-triggered event on a
    // scope/logic analyzer instead of a dense continuous stream.  Like
    // DRIVEWIRE_TEST_UART_ONLY, this bypasses the address monitor and
    // knock logic entirely.
    //
    // Deliberately non-blocking on the TX FIFO: if UART1 stops actually
    // draining it for any reason, a blocking wait (drivewire_uart_putc's
    // normal behaviour) would leave the whole loop - and hence this LED
    // heartbeat - stuck forever, making "the CPU is wedged waiting on
    // UART1" indistinguishable from "the CPU crashed outright".  The
    // heartbeat toggles on a fixed loop-iteration count, independent of
    // whether any byte actually gets sent, so it keeps going either way.
    s_get_sysclk_mhz = ora_lookup_fn(ORA_ID_GET_SYSCLK_MHZ);
    drivewire_uart_init();
    {
        ora_set_status_led_fn_t heartbeat_led = ora_lookup_fn(ORA_ID_SET_STATUS_LED);
        uint8_t led_state = 0;
        for (;;) {
            if (!(UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_TXFF)) {
                UART_REG(DW_UART_BASE, UART_DR_OFFSET) = 0x55u;
            }
            led_state = (uint8_t)(led_state ? 0u : 1u);
            heartbeat_led(led_state);
            // Same iteration count as the three-blink startup heartbeat you
            // already confirmed was clearly visible - deliberately reused
            // rather than a new estimate, so there is no ambiguity about
            // whether this is genuinely blinking or toggling too fast to
            // perceive.  This also slows the byte rate to the same cadence,
            // which is no less useful for scope triggering than the
            // original ~8-bit-period gap was.
            for (volatile int d = 0; d < 3000000; d++) { }
        }
    }
#endif

#ifdef DRIVEWIRE_TEST_PATTERN
    // Startup heartbeat, before anything else runs: without this, "LED off"
    // while waiting for the knock is indistinguishable from "the plugin
    // never started at all" (crashed in setup, never scheduled, etc).  Three
    // quick blinks means the plugin is loaded and drivewire_main() is
    // genuinely executing.
    {
        ora_set_status_led_fn_t blink_led = ora_lookup_fn(ORA_ID_SET_STATUS_LED);
        for (int i = 0; i < 3; i++) {
            blink_led(1);
            for (volatile int d = 0; d < 3000000; d++) { }
            blink_led(0);
            for (volatile int d = 0; d < 3000000; d++) { }
        }
    }
#endif

    drivewire_setup(ora_lookup_fn);

#ifdef DRIVEWIRE_TEST_PATTERN
    // Bring-up test: confirm the knock/UART chain works end to end before
    // exercising the full protocol.  Pairs with hdbdos's ONEROM_TEST_PATTERN
    // DWWrite, which sends its own knock and then an incrementing 0x00-0xFF
    // pattern and never returns.
    //
    // Deliberately does NOT gate UART output on a successful knock match:
    // relays every captured address's low byte to UART1 TX unconditionally,
    // from the moment setup finishes.  If the 6809 code is running at all,
    // the raw byte stream is visible on a scope regardless of whether the
    // knock-matching logic below has a bug - that would otherwise be
    // indistinguishable from DWWrite never having been called at all.  The
    // status LED (forced off first - the board's boot-time default usually
    // leaves it on) lights the first time the exact "!DWSEND!" sequence is
    // seen in that stream, as a secondary, coarser signal.
    s_set_status_led(0);
    uint8_t window[KNOCK_LEN] = {0};
    bool matched = false;
    for (;;) {
        uint8_t b = (uint8_t)(drivewire_test_next_addr() & 0xFFu);
        drivewire_uart_putc(b);
        if (!matched) {
            for (unsigned i = 0; i < KNOCK_LEN - 1u; i++) {
                window[i] = window[i + 1u];
            }
            window[KNOCK_LEN - 1u] = b;
            bool m = true;
            for (unsigned i = 0; i < KNOCK_LEN; i++) {
                if (window[i] != s_knock_write[i]) { m = false; break; }
            }
            if (m) {
                matched = true;
                s_set_status_led(1);
            }
        }
    }
#else
    for (;;) {
        s_debug_main_phase = 0u;
        // Clear UART1's sticky receive-status flags (framing/parity/break/
        // overrun - see UART_RSR_OE's own comment on s_debug_sticky_rsr)
        // before this session's own knock-wait even starts, since the
        // opportunistic UART drain in drivewire_next_addr() can already be
        // pulling bytes for this session during knock-matching, before
        // drivewire_do_read() is ever called - see its own comment.  A write
        // is cleared identically here even though it doesn't drain UART1 RX,
        // since gating this on which direction gets matched would need
        // doing it twice; clearing unconditionally costs nothing.
        UART_REG(DW_UART_BASE, UART_RSR_OFFSET) = 0;
        session_dir_t dir = drivewire_wait_for_knock();
        s_debug_session_count++;

        s_debug_main_phase = 1u;
        // 16-bit count, high byte first - matches dwonewrite.asm's/
        // dwoneread.asm's identical encoding (see either's own comment).
        // No more 0-means-256 special case: 16 bits already covers the
        // dw.h-documented 0-65535 range directly.
        uint32_t count_hi = drivewire_next_addr_skip_ack() & 0xFFu;
        uint32_t count_lo = drivewire_next_addr_skip_ack() & 0xFFu;
        uint16_t count = (uint16_t)((count_hi << 8) | count_lo);
        s_debug_current_count = count;

        if (dir == SESSION_WRITE) {
            // No buffer here - drivewire_do_write() streams the ROM bus
            // straight to UART1 one byte at a time, so any count up to
            // 65535 is safe.
            s_debug_main_phase = 2u;
            drivewire_do_write(count);
            s_debug_last_count = count;
        } else if (count <= sizeof(s_read_buf)) {
            s_debug_main_phase = 3u;
            drivewire_do_read(count);
            s_debug_last_count = count;
        } else {
            // Unlike writes, drivewire_do_read() buffers the whole response
            // in s_read_buf before relaying it (see its own comment on the
            // opportunistic pre-drain), and that buffer is a fixed
            // sizeof(s_read_buf) bytes - RAM tight enough elsewhere in this
            // plugin that it cannot simply be grown to match the wire
            // format's new 65535-byte ceiling.  Nothing today asks for a
            // read this large, but the wire format now allows a future bug
            // (or a not-yet-written FUJICMD) to request one, so refuse it
            // outright rather than overrunning s_read_buf into whatever
            // static RAM happens to sit right after it.
            s_err_log("DriveWire: read count %u exceeds buffer, ignoring session", count);
        }
    }
#endif
}
