// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
// Copyright (C) 2026 William Athing
//
// MIT License
//
// DriveWire bridge plugin for One ROM.
//
// Bridges a DriveWire session between a CoCo/Dragon disk controller ROM and
// a real DriveWire server, over One ROM's RP2350 UART1 hardware (GPIO40 TX,
// GPIO41 RX - the One ROM Fire 28 Rev C SEL_C/SEL_D pads, which must be left
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
//   is refused rather than served.  The plugin collects all N bytes from
//   UART1, lays them out in a response window (DW_WINDOW_ADDR), and only
//   then sets a status byte (DW_STATUS_ADDR) to ready.  The CoCo spin-reads
//   the status until ready, copies the whole window at full bus speed, and
//   acknowledges with one more read - one wait per response, rather than a
//   handshake per byte.  See drivewire_do_read().
//
// The status and acknowledgement addresses are ordinary ROM offsets ($C100
// and $C200 in CPU space) - live hdbdos code, not spare padding.  What makes
// that safe is that the plugin reads and caches the real bytes once at
// startup and restores them the moment each session ends, so outside an
// active session the code is exactly what it was assembled as.  The window
// sits in the image's unused upper 8K instead, and needs no restore - see
// DW_WINDOW_ADDR.  The knock sequences themselves are pure reads and never
// touch served content at all.
//
// This plugin does not support yielding: a pause mid-knock or mid-session
// would be indistinguishable from the CoCo simply pausing, and there is no
// way to resynchronise cleanly, so DriveWire sessions must not be interrupted
// by another plugin's exclusive-mode request.

#include <stdint.h>
#include <stdbool.h>
#include "plugin.h"

#define RP235X
#define MCU_FLASH_SIZE_KB 2048
#define MCU_RAM_SIZE_KB 520
#define RP2350A
#include "onerom_metadata.h"
#include "reg-rp235x.h"
#include "uart-rp235x.h"

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
    // 0.7.3 for ora_address_monitor_options_t - see drivewire_main().
    .min_fw_major_version = 0,
    .min_fw_minor_version = 7,
    .min_fw_patch_version = 3,
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
static const uint8_t s_knock_write[KNOCK_LEN] = {'!','D','W','S','E','N','D','!'};  // "!DWSEND!" CoCo -> server
static const uint8_t s_knock_read[KNOCK_LEN]  = {'!','D','W','R','E','C','V','!'};  // "!DWRECV!" server -> CoCo
// CoCo -> plugin: activate an already-pushed cartridge/ROM image - see
// drivewire_do_load_cart()'s own comment for the full protocol.  Not part of
// DriveWire proper - a One-ROM-specific extension the config app's own
// knock-sending code (not fujinet-lib, not hdbdos) speaks directly.
static const uint8_t s_knock_cart[KNOCK_LEN]  = {'!','L','D','C','A','R','T','!'};  // "!LDCART!"

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

// Logical (chip) byte offset of a read session's status byte - $C100 in the
// CoCo's address space.  Live hdbdos code; see the file header comment for
// why overwriting it here is safe.
#define DW_STATUS_ADDR   0x0100u

// A read session's status values - see dwoneread.asm's "Wire protocol" for
// the CoCo side.  DW_STATUS_ADDR shows DW_STATUS_WAIT from the moment the
// session starts, alternating with DW_STATUS_BUSY as a heartbeat (see
// drivewire_heartbeat()), and DW_STATUS_READY once the whole response is in
// DW_WINDOW_ADDR.  The CoCo acknowledges the response by reading
// DW_READ_ACK_ADDR.  DW_STATUS_READY's value is dwoneread.asm's DWRREADY,
// and DW_READ_ACK_ADDR its DWRACK, so changing either breaks that ROM.
#define DW_STATUS_READY   0x02u
#define DW_STATUS_WAIT    0x03u
#define DW_STATUS_BUSY    0x01u
#define DW_READ_ACK_ADDR  (DW_STATUS_ADDR + DW_STATUS_READY)
_Static_assert(DW_STATUS_WAIT != DW_STATUS_READY && DW_STATUS_BUSY != DW_STATUS_READY,
               "the heartbeat must never show ready");

// Where a read session lays out its response for the CoCo to copy at full
// bus speed - $E000 to $E0FF, 256 bytes for the largest response
// (sizeof(s_read_buf)).  It is in the 27128's upper 8K, which the 8K HDB-DOS
// image leaves as a duplicate of itself or as padding, depending on the
// slot's size handling, and which no code runs from.  So unlike
// DW_STATUS_ADDR it needs no snapshot and no restore, and the last response
// simply stays there until the next.  The layout suits dwoneread.asm's
// stack blast (PULU/PSHS, 7 bytes at a time, PSHS filling the buffer
// downwards) - see drivewire_do_read().
#define DW_WINDOW_ADDR    0x2000u
#define DW_BLAST_CHUNK    7u

// A second live ROM byte, deliberately in a different 256-byte page from
// DW_STATUS_ADDR above.  Written to DW_KNOCK_ACK_VALUE the
// instant a write-session or "!LDCART!" knock matches (see
// drivewire_wait_for_knock()), so the CoCo's own bounded poll-and-retry -
// dwonewrite.asm's DWWrite, or a "!LDCART!" caller doing the same thing -
// can tell its knock actually landed and resend, or give up gracefully, if
// it doesn't see this in time.  The read direction's own knock does not get
// this treatment: it has never been observed to go missing, unlike the
// write knock this guards.
//
// This cannot reuse DW_STATUS_ADDR itself: the count and
// data bytes that follow a knock are sent the same way the knock is - as an
// address whose low byte IS the value, anywhere across the full $C100-$C1FF
// page (a count of 256 wire-encodes as 0, landing on $C100 exactly).  Only a
// full-address comparison against a byte outside that whole page can tell
// "the CoCo is still polling for the ack" apart from "the CoCo is now
// sending a real count/data byte that happens to have this low byte" - the
// same reason drivewire_wait_for_ack() below compares full addresses, not
// low bytes.
#define DW_KNOCK_ACK_ADDR  0x0200u
#define DW_KNOCK_ACK_VALUE 0xA5u

// The "!LDCART!" status block, in a different page from DW_KNOCK_ACK_ADDR so
// it coexists with that mechanism untouched. drivewire_do_load_cart() writes
// all five of these (into slot 0, still active at that point) before
// switching to the cart, so fujinet-config's onerom_load_cart() can confirm
// not just that the load happened but what was actually pushed -
// independently re-checksumming the activated cart's own memory afterward,
// and refusing to jump into it on a mismatch. Length/checksum are only
// meaningful when the status byte is DW_CART_STATUS_SUCCESS.
#define DW_CART_STATUS_ADDR    0x0210u
#define DW_CART_LEN_LO_ADDR    0x0211u
#define DW_CART_LEN_HI_ADDR    0x0212u
#define DW_CART_CKSUM_LO_ADDR  0x0213u
#define DW_CART_CKSUM_HI_ADDR  0x0214u
#define DW_CART_STATUS_SUCCESS 0x00u
#define DW_CART_STATUS_FAILURE 0xFFu

#define DW_UART_BASE     UART1_BASE
#define DW_UART_TX_GPIO  40u
#define DW_UART_RX_GPIO  41u
#ifndef DW_UART_BAUD
#define DW_UART_BAUD     5000000u
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

// Bound, in drivewire_poll_addr() passes with nothing to show, on how long a
// session waits for the CoCo's next ROM read - see drivewire_session_addr().
// Inside a session the CoCo reads in a tight loop with interrupts masked, so
// gaps are microseconds; the case this catches is a CoCo that has left the
// session without a trace on the bus (a CoCo 3 after RESET runs entirely
// from RAM and makes no cartridge reads at all).  Fixed rather than derived
// from s_get_sysclk_mhz() for the same reason as DW_UART_GETC_TIMEOUT: a
// pass is well under a microsecond, so this is a fraction of a second on any
// clock, orders of magnitude past any legitimate gap and still a quick
// recovery.
#define DW_SESSION_IDLE_TIMEOUT 1000000ul

// How long a read session keeps up its heartbeat (see drivewire_heartbeat())
// while waiting for one reply byte with nothing at all from the ESP32.  The
// heartbeat claims the ESP32 is working on the request, but this plugin
// can't know that: an ESP32 still booting at power-on has simply lost it,
// and beating on regardless held the CoCo for this plugin's whole UART
// timeout, tens of seconds, on every boot-probe retry.  So beat only for as
// long as a genuinely slow reply plausibly takes - a file open, say, before
// the cartridge push it triggers, whose own frames beat for themselves - and
// then go quiet, letting DWRead time out ~1.4s later and retry.  Real time,
// not a loop count: the wait loop's speed varies with what it finds.
#define DW_HEARTBEAT_IDLE_MS 1500u

// How long a reply may stall, once its first byte is in, before the rest
// counts as lost.  The ESP32 writes a reply in one go, so its bytes arrive
// back to back - a 256-byte sector in half a millisecond at 5 Mbaud - and a
// stall means bytes were lost on the way: a full RX FIFO, or a glitch on the
// line.  Without this, the plugin waited on for bytes that weren't coming,
// the ESP32 gave up on the CoCo's checksum, and the CoCo timed out with an
// ?IO ERROR.  Far longer than any pause in the ESP32's own writing, so a
// slow reply isn't cut short, and well inside the ESP32's own checksum wait
// (two 500ms reads), so the recovery in drivewire_do_read() still sees the
// status byte that wait ends with.  Real time, as DW_HEARTBEAT_IDLE_MS.
#define DW_REPLY_GAP_MS 250u

// DBC (FujiBusPacket) SLIP framing and command bytes - see the "DBC
// (FujiBusPacket)" section further down for how these are used.  Hoisted up
// here because drivewire_do_read() needs DBC_SLIP_END to tell an unprompted
// cartridge push apart from the start of an ordinary reply, before it has
// read enough to know which one it's looking at.
#define DBC_SLIP_END       0xC0u
#define DBC_SLIP_ESC       0xDBu
#define DBC_SLIP_ESC_END   0xDCu
#define DBC_SLIP_ESC_ESC   0xDDu

#define DBC_FUJI_DEVICEID  0xFFu
#define DBC_CMD_NET_OPEN   0x4Fu
#define DBC_CMD_NET_WRITE  0x57u
#define DBC_CMD_NET_CLOSE  0x43u
#define DBC_CMD_FUJI_ACK   0x06u
#define DBC_CMD_FUJI_NAK   0x15u

#define DBC_HEADER_LEN     6u   // device, command, length(lo,hi), checksum, descr

// Worst case for a header probe's raw byte capture: every one of
// DBC_HEADER_LEN decoded bytes arrived as a 2-byte SLIP escape pair.
#define DBC_PROBE_RAW_CAP (2u * DBC_HEADER_LEN)

// Ordinary (non-DBC) OP_FUJI opcode/command bytes drivewire_do_write() looks
// for as it relays a write, to arm cartridge-push detection - see that
// function's own comment.  FUJI_OP_FUJI matches dw.h's OP_FUJI; the two
// commands match fujinet-fuji.h's FUJICMD_MOUNT_IMAGE/FUJICMD_MOUNT_ALL -
// both are CoCo-client-initiated mount requests that can trigger FujiNet to
// push a ROM image (see fuji_mount_disk_image.c/fuji_mount_all.c in
// fujinet-lib).
#define FUJI_OP_FUJI         0xE2u
#define FUJI_CMD_MOUNT_IMAGE 0xF8u
#define FUJI_CMD_MOUNT_ALL   0xD7u

// How long, in drivewire_next_addr() loop iterations, to keep watching for
// a cartridge push after arming - see drivewire_do_write()'s own comment.
// Every armed window is exposed to the push-probe's own residual
// false-positive risk against ordinary disk traffic (see
// dbc_header_looks_like_push()'s own comment) - and since
// FUJICMD_MOUNT_IMAGE/MOUNT_ALL share their opcode prefix with an ordinary
// (non-ROM) disk mount, arming can't tell which one it's looking at yet, so
// this cost is paid on most mounts, not just ROM ones.  A too-generous
// budget turns a rare, tightened-gate false positive into a near-certain
// one by simply running it over far more bytes of real traffic than needed.
//
// Confirmed on real hardware (2026-10-17): the previous value (100000000)
// was still armed a full 26.5 seconds after an ordinary (non-ROM)
// FUJICMD_MOUNT_IMAGE, and a false positive on the sector-read burst that
// followed caused a real checksum error on the CoCo side - roughly 25+
// seconds of pure unnecessary exposure beyond the ~1 second mount+push
// latency this is actually sized for (FujiNet's own SD card access, plus
// the CoCo-side bus_ready()/fuji_get_error() retry dance - see the "DBC
// (FujiBusPacket)" section's own history of bugs #2 and #3).  That one data
// point upper-bounds this loop's iteration rate at 100000000/26.5s ~=
// 3.77M/s; this value targets roughly 2 seconds of coverage even if the
// real rate turns out to be that fast, a healthy margin over the observed
// latency.  Still a starting value, not a measured one - the real
// iteration rate hasn't been directly measured, only bounded from below by
// this single real-hardware data point.
#define DW_PUSH_ARM_BUDGET 10000000ul

typedef enum {
    SESSION_WRITE,
    SESSION_READ,
    SESSION_LOAD_CART,
} session_dir_t;

// Dedicated RAM slot a pushed cartridge image is received into (see
// drivewire_dbc_receive_cart()) and "!LDCART!" later activates (see
// drivewire_do_load_cart()) - deliberately never the currently-active slot
// (slot 0, hdbonerom.rom in this plugin's own build), so the transfer can
// never corrupt what the CoCo might still be executing from, and
// drivewire_dbc_receive_cart() only ever needs allow_active=0 when
// reprogramming it.  See that function's own comment for the full handshake.
#define DW_CART_RAM_SLOT 1u

// ---------------------------------------------------------------------------
// Looked-up API functions
// ---------------------------------------------------------------------------

static ora_demangle_addr_fn_t               s_demangle;
static ora_reprogram_ram_rom_slot_fn_t      s_reprogram;
static ora_read_ram_rom_slot_fn_t           s_read_slot;
static ora_get_active_ram_slot_fn_t         s_get_active_slot;
static ora_set_active_ram_slot_fn_t         s_set_active_slot;
static ora_get_ram_slot_info_fn_t           s_get_ram_slot_info;
static ora_get_sysclk_mhz_fn_t              s_get_sysclk_mhz;
static ora_start_address_monitor_fn_t       s_start_monitor;
static ora_get_address_monitor_ring_write_pos_fn_t s_get_write_pos;
static ora_set_status_led_fn_t              s_set_status_led;
static ora_get_plugin_uptime_ms_fn_t        s_uptime_ms;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t s_active_slot;
static uint8_t s_orig_status_byte;
static uint8_t s_orig_ack_byte;

// Set once drivewire_dbc_receive_cart() has written a complete cartridge
// image into DW_CART_RAM_SLOT, and cleared the instant drivewire_do_load_cart()
// consumes it.  This is what lets "!LDCART!" simply ask "is one ready" rather
// than requesting and receiving the push itself - see
// drivewire_receive_confirmed_push()'s own comment for where a push
// actually gets noticed and received.
static bool s_cart_pending;

// Bytes of DW_CART_RAM_SLOT the CoCo can actually address: the served chip
// type's own size (16K for a 27128), resolved once by drivewire_setup(), or 0
// if that lookup failed.  Not the slot size ora_get_ram_slot_info() reports -
// that is the whole ROM-table region (64K on 28-pin), and a logical offset
// past the chip's own address lines doesn't fail on reprogram, it silently
// lands on offset % chip size.  Bounding writes by the region instead let
// drivewire_dbc_receive_cart()'s mirror fill run on to 64K, overwriting the
// image it had just received with shifted copies of itself - any image size
// not dividing 16K evenly (a 16128-byte $C000-$FEFF dump being the common
// case) arrived corrupted.
static uint32_t s_cart_window_size;

// Length and plain-sum checksum of the
// cartridge image actually received, tracked alongside s_cart_pending by
// drivewire_dbc_receive_cart() and reported to the CoCo by
// drivewire_do_load_cart() - see DW_CART_LEN_LO_ADDR and friends.
static uint16_t s_cart_received_len;
static uint16_t s_cart_received_checksum;

// Holds a full read session's response, drained from UART1 before any of the
// CoCo handshake starts - see drivewire_do_read().  256 is the protocol's
// maximum single-session count (0 on the wire means 256).
static uint8_t s_read_buf[256];

// How many bytes at the front of s_read_buf are already filled in before
// drivewire_do_read()'s own per-byte collection loop starts: either
// drivewire_next_addr()'s opportunistic drain (see its own comment), or
// drivewire_push_probe_feed() replaying the raw bytes of a push attempt
// that turned out not to be one after all (see that function's own
// comment).  Reset to 0 once a read session consumes it in
// drivewire_do_read(), so stale bytes from one session's window can never
// bleed into the next.
static uint16_t s_read_buf_filled;

// Gates whether drivewire_next_addr()'s opportunistic drain engages
// drivewire_push_probe_feed() at all.  Set only by drivewire_do_write()
// recognizing a ROM-mount-triggering write (see its own comment) - content-
// based, not scoped by session phase, after confirming on real hardware
// that phase-based scoping (on by default while idly scanning between
// sessions) still exposed the bulk of every ordinary sector reply to the
// prober, since that is exactly where a fast-arriving reply gets caught
// before its own read knock is even sent.  Cleared by
// drivewire_next_addr() once s_push_probe_arm_budget runs out with nothing
// confirmed.
static bool s_push_probe_enabled;

// Countdown started by drivewire_do_write() on arming and decremented by
// drivewire_next_addr() on every pass while armed and idle (not mid-probe) -
// see DW_PUSH_ARM_BUDGET's own comment.  Reset back to that same budget by
// drivewire_receive_confirmed_push() on a successful receive, in case
// FUJICMD_MOUNT_ALL pushes more than one slot.
static uint32_t s_push_probe_arm_budget;

// drivewire_push_probe_feed()'s own state, persisted across however many
// calls it takes a candidate header to arrive one byte at a time - see its
// own comment for why this must never block waiting for more.
// PUSH_PROBE_IDLE between candidates; PUSH_PROBE_IN_HEADER while one is
// being probed.
typedef enum {
    PUSH_PROBE_IDLE,
    PUSH_PROBE_IN_HEADER,
} push_probe_state_t;
static push_probe_state_t s_push_probe_state;
static uint8_t  s_push_probe_header[DBC_HEADER_LEN];
static uint16_t s_push_probe_decoded_len;
static uint8_t  s_push_probe_chk;
static bool     s_push_probe_escape;
static uint8_t  s_push_probe_raw[DBC_PROBE_RAW_CAP];
static uint16_t s_push_probe_raw_len;

// Set by drivewire_push_probe_feed() the instant a candidate header's own
// fields check out, holding the validated header and running checksum for
// drivewire_receive_confirmed_push() to finish receiving - see both
// functions' own comments for why detecting and receiving a push are split
// across two different places.  Cleared the instant that receive starts.
static bool    s_cart_push_confirmed;
static uint8_t s_cart_push_header[DBC_HEADER_LEN];
static uint8_t s_cart_push_chk;

// Main session loop's own state, one level up from drivewire_do_read()'s own
// variables above - these cover write sessions and knock-waiting too, so a
// stall outside drivewire_do_read() entirely (e.g. waiting for whatever
// knock is supposed to follow a just-completed session) is still visible.
// phase: 0=waiting for knock, 1=reading the count bytes, 2=in a write
// session, 3=in a read session, 4=in a "!LDCART!" load, 5=a "!LDCART!" load
// succeeded and this plugin has permanently stopped monitoring the ROM bus
// (see drivewire_do_load_cart()'s own comment) - the terminal phase, since
// nothing after it changes this again short of a power cycle.  session_count
// increments once per detected knock (so once per session of any kind - a
// full sector "block" is several of these: the request, the 256-byte data,
// the 2-byte checksum, and the 1-byte ack, each a separate session).
static volatile uint8_t  s_debug_main_phase;
static volatile uint32_t s_debug_session_count;
// Increments on every pass of drivewire_poll_addr(), the most fundamental,
// frequently-hit code in the plugin - a definitive "is anything at all still
// executing" signal, independent of which higher-level function (do_read,
// do_write, wait_for_knock, wait_for_ack) is
// currently active.  If two peeks a few seconds apart show the exact same
// value here, the plugin's core (core 0) is not merely busy-looping
// somewhere - it has genuinely stopped running any code at all.
static volatile uint32_t s_debug_heartbeat;

// What went wrong last, for reading back with `onerom inspect peek memory
// --address <s_last_error's address from build/drivewire_plugin.map>
// --length 4`.  This plugin never calls the firmware's ora_log()/
// ora_err_log(): they format through the firmware's RTT printf, ~330 bytes of
// stack, and this plugin has ~350 for everything - a log call from
// drivewire_main()'s own frame overflows it and faults the plugin (the
// fault handler blinks the status LED fast).  A first version of the session
// aborts logged, and faulted exactly so on real hardware.  Recording here is
// a few stores instead.  code is the most recent DW_ERR_*, count saturates
// at 255, and arg is the detail the old log message carried, where it had
// one.  A session ending because the CoCo left it (crashed, reset, timed
// out) is ordinary recovery, not an error, and is not recorded.
enum {
    DW_ERR_NONE = 0,
    DW_ERR_REPROGRAM,          // arg: the live-ROM address that failed
    DW_ERR_PUSH_DEVICE,        // arg: the unexpected DBC device id
    DW_ERR_PUSH_SLOT,          // arg: DW_CART_RAM_SLOT
    DW_ERR_PUSH_TOO_BIG,       // arg: the chip window, in KB
    DW_ERR_PUSH_FRAME,         // arg: 1 mid-push, 2 in a read session, 3 elsewhere
    DW_ERR_CART_WINDOW,        // served chip size unknown - pushes refused
    DW_ERR_NO_ACTIVE_SLOT,     // at startup; slot 0 assumed
    DW_ERR_SNAPSHOT,           // channel bytes couldn't be read at startup
    DW_ERR_MONITOR_SETUP,      // arg: setup_monitor()'s ora_result_t
    DW_ERR_MONITOR_WRITE_POS,  // ring buffer write position unavailable
    DW_ERR_READ_COUNT,         // arg: the oversized read count, refused
    DW_ERR_STATUS_BYTE,        // the ROM's own byte at DW_STATUS_ADDR reads as ready
    DW_ERR_UART_RX,            // arg: UART1's RSR - 8 overrun, 4 break, 2 parity, 1 framing
    DW_ERR_READ_SHORT,         // arg: reply bytes received - see DW_REPLY_GAP_MS
};
static volatile struct {
    uint8_t  code;
    uint8_t  count;
    uint16_t arg;
} s_last_error;

// Records an error in s_last_error - see its comment for why this, and never
// a log call.  Out of line, as a stackless leaf: inlined into setup's error
// paths it cost drivewire_main() 16 bytes of frame, live beneath setup's
// deepest call.
__attribute__((noinline)) static void drivewire_error(uint8_t code, uint16_t arg) {
    s_last_error.code = code;
    s_last_error.arg  = arg;
    if (s_last_error.count != 0xFFu) {
        s_last_error.count++;
    }
}

// ---------------------------------------------------------------------------
// DBC (FujiBusPacket) helpers shared by both header-probing paths
// ---------------------------------------------------------------------------
//
// Hoisted up here, ahead of drivewire_next_addr() and drivewire_do_write(),
// which need them - see the "DBC (FujiBusPacket)" section further down for
// the rest of this machinery and its own fuller comment on why a cartridge
// push can arrive unprompted at all.

// Folds one more byte into a running checksum - matches FujiBusPacket's
// calcChecksum() exactly (an 8-bit add-with-end-around-carry), which stays a
// clean 0-255 value after every call since the largest possible pre-fold sum
// is 255+255.
static uint8_t dbc_checksum_add(uint8_t chk, uint8_t b) {
    uint16_t sum = (uint16_t)chk + b;
    return (uint8_t)((sum >> 8) + (sum & 0xFFu));
}

// A genuine push's first frame's own exact, fixed shape - verified against
// FujiBusPacket::serialize() (centi-fuji's lib/bus/rs232/FujiBusPacket.cpp),
// not just inferred from push_rom_image()'s own call site.
// push_rom_image() opens with `sendCommand(DBC, NET_OPEN, (uint16_t)0)` - a
// single *packed* 2-byte parameter, not a raw data payload.  serialize()
// encodes exactly one packed field as descr byte 0x05 (field count 1, plus
// FUJI_DESCR_EXCEEDS_U8 for a field wider than a byte) with no additional
// descriptor bytes, giving a fixed on-wire length of DBC_HEADER_LEN + 2 -
// the 2-byte parameter itself, carrying no actual information (always the
// literal value 0).  Notably, NET_OPEN carries no image size at all,
// contrary to what an earlier version of this code (and of
// drivewire_dbc_receive_cart()'s own NET_OPEN handling) assumed - see that
// function's own comment for the fix.
#define DBC_FIRST_FRAME_DESCR  0x05u
#define DBC_FIRST_FRAME_LEN    (DBC_HEADER_LEN + 2u)

// Checks whether a decoded 6-byte DBC header's own fields are consistent
// with a genuine cartridge push's *first* frame - rather than it merely
// starting with DBC_SLIP_END by coincidence of whatever ordinary data it
// actually is.  Used by drivewire_push_probe_feed() and
// drivewire_resolve_pushed_cart(), both of which only ever see a push's own
// opening frame (dbc_recv_frame(), fetching a second or later frame once a
// push is already confirmed under way, never needs this check at all) - so
// this requires command to be NET_OPEN specifically and descr/length to
// exactly match that frame's own fixed shape (DBC_FIRST_FRAME_DESCR/_LEN) -
// not a range, since push_rom_image()'s NET_OPEN call never varies.
//
// This matters because a coincidental match here is not a cheap mistake:
// once this gate passes, the bytes that follow are consumed directly off
// the UART stream with no way to un-consume them if the *rest* of the frame
// then fails to validate (its own checksum, in particular, which can only
// be checked once the whole claimed length has arrived) - unlike a failed
// probe during the header itself, which stays recoverable by replaying its
// raw bytes.  Confirmed on real hardware: an earlier, looser version of
// this check let the plugin start consuming a supposed push's "payload"
// that was actually the real reply to an ordinary read, with nothing left
// to serve the CoCo once that payload turned out not to validate - and
// since hdbdos's own DWRead has no timeout on the CoCo side, that reply
// never arriving hung the CoCo indefinitely with no error ever reported.
// Matching NET_OPEN's own fixed shape exactly, rather than a range, makes a
// structured (non-random) disk image's own data dramatically less likely
// to pass this specific combination by coincidence, without needing to
// solve - or afford the RAM for - replaying an arbitrarily large false
// payload.
static bool dbc_header_looks_like_push(const uint8_t header[DBC_HEADER_LEN]) {
    uint16_t claimed_len = (uint16_t)header[2] | ((uint16_t)header[3] << 8);
    return header[0] == DBC_FUJI_DEVICEID &&
           header[1] == DBC_CMD_NET_OPEN &&
           header[5] == DBC_FIRST_FRAME_DESCR &&
           claimed_len == DBC_FIRST_FRAME_LEN;
}

// ---------------------------------------------------------------------------
// UART1
// ---------------------------------------------------------------------------

// Brings up UART1 on GPIO40 (TX) / GPIO41 (RX) at DW_UART_BAUD, 8-N-1.
//
// GPIO40/41 are this board's SEL_C/SEL_D image-select pads.  The firmware's
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

    // Settle: the enable above has to actually propagate and clk_peri start
    // toggling before the RESET_DONE wait below (which synchronises through
    // that same clock domain) can ever see it complete - see this function's
    // own comment above. That propagation is a fixed, small number of clk_sys
    // cycles, not something that scales with clk_sys itself, so it went
    // unnoticed at this plugin's stock 150MHz (the intervening code executed
    // slowly enough to clear it by accident) and only surfaced as a hang here
    // once cpu-freq raised clk_sys - confirmed on real hardware at 200MHz.
    for (volatile int i = 0; i < 100; i++) { }

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

// Records, and clears, any UART1 receive error since the last call - run once
// per pass of drivewire_main()'s session loop.  The link has no flow control,
// so a byte lost to a full RX FIFO is otherwise silent: the reply just goes on
// with the next byte the ESP32 sends - its status byte, a second later - in
// the lost one's place.  RSR's flags hold until cleared, so checking once per
// session misses nothing that checking every byte would catch, and checking
// every byte grew the push receive's frames by up to 24 bytes each, inline or
// not.  Out of line, as a stackless leaf, like drivewire_error().
__attribute__((noinline)) static void drivewire_check_uart_rx(void) {
    uint32_t rsr = UART_REG(DW_UART_BASE, UART_RSR_OFFSET) &
        (UART_RSR_OE | UART_RSR_BE | UART_RSR_PE | UART_RSR_FE);
    if (rsr != 0u) {
        UART_REG(DW_UART_BASE, UART_RSR_OFFSET) = 0;
        drivewire_error(DW_ERR_UART_RX, (uint16_t)rsr);
    }
}

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

// ---------------------------------------------------------------------------
// Address bus signalling
// ---------------------------------------------------------------------------

// Defined with the rest of the DBC machinery, below - forward-declared here
// since drivewire_wait_for_knock()/drivewire_do_write() are where it gets
// called, right after whichever of them notices drivewire_push_probe_feed()
// has confirmed a push.
static void drivewire_receive_confirmed_push(void);

// Ends a probe as a false alarm: every raw byte it took - the leading
// DBC_SLIP_END included - goes to s_read_buf, in order, as the reply data it
// was.  For drivewire_push_probe_feed() when a header doesn't hold up, and
// for drivewire_do_read() when the reply ends with a probe still open.
__attribute__((always_inline)) static inline void drivewire_push_probe_replay(void) {
    if (s_read_buf_filled < sizeof(s_read_buf)) {
        s_read_buf[s_read_buf_filled++] = DBC_SLIP_END;
    }
    for (uint16_t i = 0; i < s_push_probe_raw_len && s_read_buf_filled < sizeof(s_read_buf); i++) {
        s_read_buf[s_read_buf_filled++] = s_push_probe_raw[i];
    }
    s_push_probe_state = PUSH_PROBE_IDLE;
}

// Opportunistic-drain's per-byte handler for an unprompted cartridge push -
// see the "DBC (FujiBusPacket)" section further down for the fuller
// picture.  Only ever engaged while s_push_probe_enabled is set - content-
// armed by drivewire_do_write() recognizing a ROM-mount-triggering write
// (see its own comment), not scoped to any particular caller of
// drivewire_next_addr(): once armed, this runs from inside both
// drivewire_wait_for_knock()'s knock-scan and drivewire_do_write()'s own
// relay loop, since a push can legally start arriving during either (see
// the "DBC (FujiBusPacket)" section's own history of why one of those two
// alone was not enough).  Fed exactly one UART1 RX byte per call - the same
// byte the opportunistic drain would otherwise have stashed into
// s_read_buf unconditionally - and persists its own state (s_push_probe_*)
// across calls, since a header can take several calls to arrive one byte
// at a time.  Must never itself wait for more; it only ever acts on the one
// byte it was just handed.
//
// While idle, a non-DBC_SLIP_END byte goes to s_read_buf exactly as before
// (the common case, zero added cost beyond this dispatch).  A
// DBC_SLIP_END starts a probe instead of being stashed immediately -
// dbc_header_looks_like_push()'s own comment explains why a lone marker
// byte was never trustworthy on its own.  Declining to start a new probe
// while s_cart_push_confirmed is already set (a prior one is still awaiting
// receipt) is deliberate: drivewire_wait_for_knock()/drivewire_do_write()
// each check and act on that flag immediately after the very call that can
// set it, so two pushes queuing up here at once should never happen in
// practice, but treating the byte as ordinary data in that instant is
// trivially safe, whereas overwriting s_cart_push_header before anyone has
// read it would lose the first push outright.
//
// Once DBC_HEADER_LEN logical bytes have been probed, the header's own
// fields decide: if they hold up, the raw probed bytes are discarded for
// good and s_cart_push_confirmed is set; if not, every raw byte probed -
// the leading DBC_SLIP_END included - is replayed into s_read_buf verbatim,
// so a false alarm costs nothing and loses no data.
static void drivewire_push_probe_feed(uint8_t b) {
    if (s_push_probe_state == PUSH_PROBE_IDLE) {
        if (b != DBC_SLIP_END || s_cart_push_confirmed) {
            if (s_read_buf_filled < sizeof(s_read_buf)) {
                s_read_buf[s_read_buf_filled++] = b;
            }
            return;
        }
        s_push_probe_state       = PUSH_PROBE_IN_HEADER;
        s_push_probe_escape      = false;
        s_push_probe_decoded_len = 0;
        s_push_probe_chk         = 0;
        s_push_probe_raw_len     = 0;
        return;
    }

    // PUSH_PROBE_IN_HEADER
    if (s_push_probe_raw_len < sizeof(s_push_probe_raw)) {
        s_push_probe_raw[s_push_probe_raw_len++] = b;
    }

    bool have_byte = false;
    uint8_t decoded_byte = 0;
    bool malformed = false;

    if (s_push_probe_escape) {
        s_push_probe_escape = false;
        if (b == DBC_SLIP_ESC_END) { decoded_byte = DBC_SLIP_END; have_byte = true; }
        else if (b == DBC_SLIP_ESC_ESC) { decoded_byte = DBC_SLIP_ESC; have_byte = true; }
        // else: malformed escape - abort the probe rather than leniently
        // dropping it and continuing (unlike dbc_recv_frame_payload()'s own
        // leniency, mirroring decodeSLIP(), once a push is already
        // confirmed).  A real DBC sender never emits an invalid escape
        // pair, so this is itself proof this isn't one - and critically,
        // "drop and keep going" has no bound on how many more raw bytes a
        // probe might consume before DBC_HEADER_LEN decoded bytes finally
        // accumulate, which can silently exceed s_push_probe_raw's fixed
        // capacity and truncate what gets replayed on a false alarm.
        // Confirmed on real hardware: ordinary sector data - effectively
        // random binary - produces exactly this often enough to cause
        // frequent, real checksum errors once probing runs on every byte
        // rather than once per write.
        else { malformed = true; }
    } else if (b == DBC_SLIP_END) {
        if (s_push_probe_decoded_len != 0) {
            malformed = true;  // frame ended before a full header - not real
        }
        // else: just the frame's own opening marker, repeated
    } else if (b == DBC_SLIP_ESC) {
        s_push_probe_escape = true;
    } else {
        decoded_byte = b;
        have_byte = true;
    }

    if (have_byte) {
        s_push_probe_header[s_push_probe_decoded_len] = decoded_byte;
        s_push_probe_chk = dbc_checksum_add(s_push_probe_chk,
            s_push_probe_decoded_len == 4u ? 0u : decoded_byte);
        s_push_probe_decoded_len++;
    }

    if (!malformed && s_push_probe_decoded_len < DBC_HEADER_LEN) {
        return;  // keep waiting for more header bytes, one call at a time
    }

    if (!malformed && dbc_header_looks_like_push(s_push_probe_header)) {
        for (unsigned i = 0; i < DBC_HEADER_LEN; i++) {
            s_cart_push_header[i] = s_push_probe_header[i];
        }
        s_cart_push_chk = s_push_probe_chk;
        s_cart_push_confirmed = true;
        s_push_probe_state = PUSH_PROBE_IDLE;
    } else {
        drivewire_push_probe_replay();
    }
}

// Blocks until the next CS-active address is captured, and returns its
// demangled logical (byte) address.  Discards captures that fail to
// demangle (CS glitches etc).
//
// This board only ever serves a 2764 on the 28-pin variant, which has no
// unobserved address bits, so the logical (chip-image) address space and the
// observed (bus) address space this signalling rides in are identical - see
// ora_demangle_addr_fn_t / ora_demangle_observed_addr_fn_t in api.h.  A
// 40-pin target would need to use the latter for signalling.
// An address an aborted session read but did not belong to it, handed back
// to the next drivewire_next_addr() call - see drivewire_session_addr().  A
// CoCo that gave up on a session (or was reset) and is now knocking again
// would otherwise lose that knock's first byte to the abort.  Held as the
// address plus one, so the zeroed .bss means "none" and no separate flag
// costs RAM this plugin is short of (logical addresses are far below
// UINT32_MAX, so the +1 never wraps).
static uint32_t s_pushback_addr_plus1;

// One pass of the address fetch: drains UART1 opportunistically (see below),
// then takes the next ring-buffer entry if there is one.  Returns false with
// nothing to show - an empty ring, or an entry that didn't demangle.
// always_inline keeps it out of the call depth: it runs underneath
// drivewire_do_read()'s deepest frames, where this plugin's stack margin is
// thinnest.
__attribute__((always_inline)) static inline bool drivewire_poll_addr(uint32_t *out) {
    s_debug_heartbeat++;

    // Opportunistically drain every UART1 RX byte that has already
    // arrived, on every pass through this loop - the whole FIFO, not one
    // byte a pass: at 5 Mbaud a byte lands every 2us, faster than a pass
    // that demangles a ring entry, and one byte a pass overran the FIFO
    // and dropped reply bytes on real hardware.  Emptying it each pass
    // gives a pass the FIFO's full 32 bytes of slack.  Not just while the
    // ring buffer is empty (the CoCo's own instruction fetches, e.g.
    // while copying DWRead's relocatable body onto the stack, are ROM
    // reads too, so the buffer is rarely actually empty during exactly
    // the window this matters, and gating on that meant this almost
    // never ran).  Not just once a read session officially starts in
    // drivewire_do_read() either: the DriveWire server responds to a
    // request as soon as it parses it, and at DW_UART_BAUD its entire
    // 256-byte response can arrive and overrun the 32-byte hardware RX
    // FIFO well before the CoCo even finishes sending the knock that (as
    // far as this plugin previously knew) starts a read session.  Fed
    // through drivewire_push_probe_feed() rather than stashed directly,
    // so a push starting here - mid-write-relay, or while scanning for
    // the next knock - gets the same chance to be recognized as one
    // starting right at a read session's own first byte - but only
    // while s_push_probe_enabled, i.e. only once
    // drivewire_do_write() has armed it (see that function's own
    // comment).  Falls back to the plain stash otherwise, exactly as
    // before any push support existed: confirmed on real hardware that
    // ordinary sector data can coincidentally pass the full header
    // check often enough, over a long enough session, to cause real
    // (and costly - a falsely "confirmed" push blocks real traffic for
    // seconds) failures if probed unconditionally rather than only
    // during this bounded, content-armed window.
    // Stops the moment a push is confirmed: the bytes behind its header are
    // the push's own, left in the FIFO for drivewire_receive_confirmed_push(),
    // which the caller runs as soon as this pass returns.
    while (s_read_buf_filled < sizeof(s_read_buf) && !s_cart_push_confirmed &&
           !(UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_RXFE)) {
        uint8_t b = (uint8_t)UART_REG(DW_UART_BASE, UART_DR_OFFSET);
        if (s_push_probe_enabled) {
            drivewire_push_probe_feed(b);
        } else {
            s_read_buf[s_read_buf_filled++] = b;
        }
    }

    // Disarm once the budget runs out with nothing confirmed - never
    // while mid-probe (s_push_probe_state != PUSH_PROBE_IDLE), so a
    // candidate header already in progress always gets to resolve
    // (and, on failure, replay its raw bytes) rather than being
    // abandoned mid-way with its bytes lost.  A probe is at most
    // DBC_PROBE_RAW_CAP bytes, well under a millisecond at
    // DW_UART_BAUD, so delaying disarm until it resolves costs nothing
    // measurable against DW_PUSH_ARM_BUDGET's own multi-second span.
    if (s_push_probe_enabled && s_push_probe_state == PUSH_PROBE_IDLE &&
        --s_push_probe_arm_budget == 0u) {
        s_push_probe_enabled = false;
    }

    if (RING_BUF_CUR_READ_INDEX() == RING_BUF_CUR_WRITE_INDEX()) {
        ORA_TEST_YIELD();
        return false;
    }
    uint32_t phys = (uint32_t)RING_BUF_GET_ENTRY(s_read_idx);
    RING_BUF_ADV_READ_INDEX();

    // CS inactive or demangle error: discarded, as nothing to show.
    return s_demangle(phys, out, 1) == ORA_RESULT_OK;
}

// Blocks until the CoCo's next ROM read, however long that takes - the idle
// wait for a knock.  Anything inside a session uses drivewire_session_addr()
// instead.
static uint32_t drivewire_next_addr(void) {
    uint32_t addr;
    if (s_pushback_addr_plus1 != 0u) {
        addr = s_pushback_addr_plus1 - 1u;
        s_pushback_addr_plus1 = 0u;
        return addr;
    }
    while (!drivewire_poll_addr(&addr)) { }
    return addr;
}

// Inside a session: the CoCo's next ROM read, or false if none arrives
// within DW_SESSION_IDLE_TIMEOUT passes.  Every wait on the CoCo once a
// knock has been accepted goes through here, so no session can wait on a
// CoCo that has gone away - before this existed, one that crashed or was
// reset mid-read left this plugin blocked forever waiting for the CoCo to
// collect a byte, its status/data bytes still in live ROM, until One ROM
// itself was power cycled.  Callers also abort on any address outside the
// session's own protocol set (see drivewire_session_byte() and
// drivewire_wait_for_ack()), the other way a departed CoCo shows -
// with interrupts masked and DWRead/DWWrite running from the stack, a
// session's only cartridge reads are its own signalling ones.
static bool drivewire_session_addr(uint32_t *out) {
    for (uint32_t idle = 0; idle < DW_SESSION_IDLE_TIMEOUT; idle++) {
        if (drivewire_poll_addr(out)) {
            return true;
        }
    }
    return false;
}

// Ends a session on an address that doesn't belong to it, handing that
// address to the next knock scan rather than dropping it.
static void drivewire_pushback(uint32_t addr) {
    s_pushback_addr_plus1 = addr + 1u;
}


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
    // opportunistic drain runs continuously while this loop does, so the
    // server's reply can already be sitting in s_read_buf by the time this
    // knock is even seen.  Discarding it here would lose it permanently -
    // drivewire_do_read() would then block forever on a fresh UART byte
    // that already came and went.  s_read_buf itself stays consistent
    // without help: drivewire_do_write() never touches it, and
    // drivewire_do_read() zeroes the count only once it has consumed exactly
    // that many bytes.
    //
    // Cartridge-push detection here is entirely content-armed, not scoped by
    // this loop at all - see s_push_probe_enabled's own comment for why an
    // earlier, phase-scoped design (on for the duration of this loop,
    // unconditionally) still exposed the bulk of every ordinary sector
    // reply and had to be replaced.  drivewire_next_addr() below still
    // opportunistically drains UART1 either way; it is only the decision of
    // whether to engage drivewire_push_probe_feed() on what it drains that
    // depends on whether drivewire_do_write() has armed it.
    uint8_t window[KNOCK_LEN] = {0};
    session_dir_t result;
    for (;;) {
        uint8_t b = (uint8_t)(drivewire_next_addr() & 0xFFu);
        if (s_cart_push_confirmed) {
            drivewire_receive_confirmed_push();
        }
        for (unsigned i = 0; i < KNOCK_LEN - 1u; i++) {
            window[i] = window[i + 1u];
        }
        window[KNOCK_LEN - 1u] = b;

        bool match_write = true, match_read = true, match_cart = true;
        for (unsigned i = 0; i < KNOCK_LEN; i++) {
            if (window[i] != s_knock_write[i]) match_write = false;
            if (window[i] != s_knock_read[i])  match_read = false;
            if (window[i] != s_knock_cart[i])  match_cart = false;
        }
        // match_cart acks here too, alongside match_write. A prior attempt
        // to narrow this to match_write only - on the theory that a
        // "!LDCART!" caller could mistake the knock-landed ack for the
        // load-complete one, since both are the same value at the same
        // address - was A/B tested on real hardware (2026-10-13) and
        // confirmed to cause checksum/opcode corruption on ordinary,
        // non-cart traffic, by a mechanism not fully traced. The theoretical
        // race it targeted is also not confirmed to be real: the incident
        // that motivated it turned out to be a config app that never sent
        // "!LDCART!" at all, and drivewire_do_load_cart() already clears
        // this ack again as its first action, within microseconds given the
        // ~100x clock speed gap to a 6809 - too narrow a window for that
        // caller to plausibly observe. Keep acking both here.
        if (match_write || match_cart) {
            // Acknowledge as fast as possible - dwonewrite.asm's DWWrite,
            // and a "!LDCART!" caller doing the same thing, poll
            // DW_KNOCK_ACK_ADDR for this exact value, with a timeout that
            // resends the whole knock if it never sees it (see
            // DW_KNOCK_ACK_ADDR's own comment).  drivewire_do_write()/
            // drivewire_do_load_cart() each restore the real ROM content
            // here once their own session ends, matching how
            // drivewire_do_read() already restores both channel addresses
            // at the end of a read session.
            uint8_t ack = DW_KNOCK_ACK_VALUE;
            s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &ack, 1u, 1u);
        }
        if (match_write) { result = SESSION_WRITE; break; }
        if (match_read)  { result = SESSION_READ; break; }
        if (match_cart)  { result = SESSION_LOAD_CART; break; }
    }
    return result;
}

// Whether addr is one a CoCo inside a read session can read: the status poll,
// the response window, and the acknowledgement.
__attribute__((always_inline)) static inline bool drivewire_is_read_session_addr(uint32_t addr) {
    return addr == DW_STATUS_ADDR || addr == DW_READ_ACK_ADDR ||
           addr - DW_WINDOW_ADDR < sizeof(s_read_buf);
}

// Waits for the CoCo to read DW_READ_ACK_ADDR - its acknowledgement that it
// has copied the response out of DW_WINDOW_ADDR.  Its status polls and the
// copy's own window reads are passed over; the copy runs at full bus speed
// and can lap the 64-entry ring, which is harmless, since every entry it
// overwrites is another of those.  False if the CoCo has left the session
// instead: no read at all (see drivewire_session_addr()), or one outside
// that set.
static bool drivewire_wait_for_ack(void) {
    uint32_t addr;
    for (;;) {
        if (!drivewire_session_addr(&addr)) return false;
        if (addr == DW_READ_ACK_ADDR) return true;
        if (!drivewire_is_read_session_addr(addr)) {
            drivewire_pushback(addr);
            return false;
        }
    }
}

// Nonzero only while a read session waits for the ESP32, before its
// response is published: the value DW_STATUS_ADDR is showing,
// DW_STATUS_WAIT or DW_STATUS_BUSY.  Zero everywhere else, which keeps drivewire_heartbeat()
// from ever writing DW_STATUS_ADDR outside that window - it is live HDB-DOS
// code the rest of the time.
static uint8_t s_heartbeat;

// Flips DW_STATUS_ADDR between DW_STATUS_WAIT and DW_STATUS_BUSY, if a read
// session is waiting.  dwoneread.asm's DWRead restarts its ~1.4s timeout
// whenever the status changes, so the CoCo waits as long as this keeps
// beating - through a slow reply or a whole cartridge push - and gives up
// promptly once nothing is.  A fixed timeout couldn't tell the two apart: a
// double-speed CoCo 3 timed out at 0.7s on a MOUNT IMAGE whose reply came
// ~1.0s later, after the file open and the 16K push it triggers.  Writes
// from s_heartbeat itself, so no local of its own needs a stack slot.
// Called only from depths that already make an s_reprogram() call (see
// drivewire_poll_addr() on the stack margin) - never from
// drivewire_uart_getc(), which runs deeper inside the push receive.
__attribute__((always_inline)) static inline void drivewire_heartbeat(void) {
    if (s_heartbeat == 0u) return;
    s_heartbeat ^= (uint8_t)(DW_STATUS_WAIT ^ DW_STATUS_BUSY);
    s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_heartbeat, 1u, 1u);
}

// A read session's wait for the ESP32's next reply byte, -1 if the CoCo has
// left or the ESP32 never answers, or, with in_reply - the reply has begun -
// -2 once it has stalled for DW_REPLY_GAP_MS.
// drivewire_uart_getc() with the same DW_UART_GETC_TIMEOUT, but beating the
// heartbeat (see drivewire_heartbeat()) and watching the ring meanwhile -
// giving up the moment the CoCo reads an address outside the read session's
// set, because it has left (DWRead timed out) and is likely knocking again.
// Without that, a reply that never comes - an ESP32 still booting lost the
// request - held this plugin in its UART wait for the whole timeout, several
// seconds, while the CoCo's retry knocks overran the ring unread.  Reads the
// ring directly rather than through drivewire_poll_addr(), whose
// opportunistic UART drain would race this function for the very bytes it is
// waiting on.  Returns the byte rather than writing through a pointer, which
// would pin the caller's destination to a stack slot in drivewire_main()'s
// frame - see drivewire_poll_addr() on the stack margin.
static int drivewire_read_session_getc(bool in_reply) {
    uint32_t timeout = DW_UART_GETC_TIMEOUT;
    uint32_t beat_start_ms = 0;
    bool     beat_started = false;
    while (UART_REG(DW_UART_BASE, UART_FR_OFFSET) & UART_FR_RXFE) {
        if (--timeout == 0u) {
            return -1;
        }
        // Every 4096 passes - well under a millisecond on any clock, and far
        // inside DWRead's timeout, but rare enough to cost nothing.  Beats
        // only for DW_HEARTBEAT_IDLE_MS of this wait; the clock is read
        // lazily, here, so a byte already waiting costs no call at all.
        if ((timeout & 0xFFFu) == 0u) {
            uint32_t now_ms = s_uptime_ms();
            if (!beat_started) {
                beat_started = true;
                beat_start_ms = now_ms;
            }
            if (in_reply && now_ms - beat_start_ms >= DW_REPLY_GAP_MS) {
                return -2;
            }
            if (now_ms - beat_start_ms < DW_HEARTBEAT_IDLE_MS) {
                drivewire_heartbeat();
            }
        }
        if (RING_BUF_CUR_READ_INDEX() != RING_BUF_CUR_WRITE_INDEX()) {
            uint32_t phys = (uint32_t)RING_BUF_GET_ENTRY(s_read_idx);
            RING_BUF_ADV_READ_INDEX();
            uint32_t addr;
            if (s_demangle(phys, &addr, 1) == ORA_RESULT_OK &&
                !drivewire_is_read_session_addr(addr)) {
                drivewire_pushback(addr);
                return -1;
            }
        }
    }
    return (int)(uint8_t)UART_REG(DW_UART_BASE, UART_DR_OFFSET);
}

// The next address-encoded byte of a session (count or write data), or -1
// if the CoCo has left the session.  Returned rather than written through a
// pointer, and always_inline, so neither caller's frame nor the session
// call depth grows - see drivewire_poll_addr() on the stack margin.
//
// Silently discards any address exactly equal to DW_KNOCK_ACK_ADDR - see
// that macro's own comment for why this must be a full-address compare, not
// the usual low-byte mask.  Only a write session's own knock ever writes an
// ack there, so this is a no-op for read sessions: the CoCo never generates
// that address at all outside DWWrite's own poll loop.
//
// The CoCo sends each byte as "LDA B,U" with U at DW_STATUS_ADDR, and the
// 6809's B offset is signed: $00-$7F land at DW_STATUS_ADDR up to +$7F, and
// $80-$FF at DW_STATUS_ADDR -$80 up to -$01, the page below.  The low byte
// of the address is the value either way, and anything outside that 256-byte
// window ends the session.  (A first version accepted DW_STATUS_ADDR's page
// alone, and so aborted every write carrying a byte of $80 or more.)
__attribute__((always_inline)) static inline int drivewire_session_byte(void) {
    uint32_t addr;
    do {
        if (!drivewire_session_addr(&addr)) return -1;
    } while (addr == DW_KNOCK_ACK_ADDR);
    if (addr - (DW_STATUS_ADDR - 0x80u) > 0xFFu) {
        drivewire_pushback(addr);
        return -1;
    }
    return (int)(addr & 0xFFu);
}

// ---------------------------------------------------------------------------
// Sessions
// ---------------------------------------------------------------------------

// Defined with the rest of the DBC machinery, below - forward-declared here
// since drivewire_do_read() is the one place that calls it, right at the
// point a cartridge push can legally begin.
static void drivewire_resolve_pushed_cart(void);

// Set by drivewire_do_read() when it hands the CoCo a reply that came up
// short (see DW_REPLY_GAP_MS), so the CoCo's checksum for it - the next
// session, a 2-byte write - is consumed here and never relayed.  The ESP32,
// getting no checksum, answers with E$CRC (243), which no checksum can
// match - a missing one reads as $FFFF, past any 256-byte sum - so HDB-DOS
// rereads the sector, whatever the padded reply held.  A relayed checksum
// could match a reply with a byte dropped, and the CoCo would keep it.
// Cleared by the next session whatever it is, so it can never swallow a
// later one.
static bool s_drop_checksum;

static void drivewire_do_write(uint16_t count) {
    if (count != 2u) {
        s_drop_checksum = false;
    }
    uint8_t first_byte = 0;
    for (uint16_t i = 0; i < count; i++) {
        int next = drivewire_session_byte();
        if (next < 0) {
            // The CoCo has left the session; the ESP32 has a truncated
            // command, which it times out on.  Ordinary recovery, so not
            // recorded - see s_last_error.
            s_drop_checksum = false;
            s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u, 1u);
            return;
        }
        uint8_t b = (uint8_t)next;
        if (s_drop_checksum) {
            continue;
        }
        if (i == 0) {
            first_byte = b;
        } else if (i == 1 && first_byte == FUJI_OP_FUJI &&
                   (b == FUJI_CMD_MOUNT_IMAGE || b == FUJI_CMD_MOUNT_ALL)) {
            // Arms cartridge-push detection on recognizing a ROM-mount-
            // triggering write, content-based rather than scoped by session
            // phase - see s_push_probe_enabled's own comment for why phase
            // scoping wasn't safe.  Checking only the first two relayed
            // bytes is deliberate: this is the exact wire prefix
            // fuji_mount_disk_image()/fuji_mount_all() (fujinet-lib) send,
            // and checking no more than that keeps this cheap on every
            // write, not just ones that turn out to match.  Not every mount
            // this arms for actually pushes anything (FUJICMD_MOUNT_IMAGE
            // mounts an ordinary .dsk far more often than a ROM, and
            // FUJICMD_MOUNT_ALL fires at every boot restoring whatever
            // disks were previously configured, not just when a ROM is
            // involved), so most arm windows simply expire with
            // s_push_probe_arm_budget reaching 0 and nothing confirmed - a
            // deliberate, accepted cost, far cheaper than being armed
            // unconditionally.  2026-10-10: confirmed on real hardware that
            // arming on its own was not sufficient - FUJICMD_MOUNT_ALL's
            // own every-boot firing meant the arm window routinely
            // overlapped with the heavy burst of ordinary sector reads that
            // naturally follows any mount, reopening the same false-positive
            // exposure at a lower frequency.  dbc_header_looks_like_push()
            // was tightened in response (NET_OPEN specifically, a narrow
            // claimed length) rather than narrowing which commands arm here
            // - both FUJICMD_MOUNT_IMAGE and FUJICMD_MOUNT_ALL are
            // genuine, ESP32-confirmed push triggers (see
            // fujicmd_mount_disk_image_success()/fujicmd_mount_all_success()
            // in centi-fuji's drivewireFuji.cpp), so excluding either here
            // would just trade a false positive for a missed real push.
            s_push_probe_enabled = true;
            s_push_probe_arm_budget = DW_PUSH_ARM_BUDGET;
        }
        drivewire_uart_putc(b);
    }
    s_drop_checksum = false;

    // Restore the real ROM content at the ack address now the session is
    // over - drivewire_wait_for_knock() reprogrammed it to ack this
    // session's own knock, and a write session never touches
    // DW_STATUS_ADDR at all, so only this one address needs it.
    s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u, 1u);

    // Checked only now, after this write's own bytes are all sent - not
    // per-iteration above, which would risk pausing mid-relay to receive a
    // push and starving the ring buffer of this write's own remaining,
    // non-redundant bytes.  By this point the CoCo has nothing further to
    // send for this session, so pausing here costs nothing - see
    // drivewire_receive_confirmed_push()'s own comment.  This is a flag
    // check, not a wait: drivewire_push_probe_feed() (fed from
    // drivewire_next_addr()'s own opportunistic drain, which every call
    // above already went through) is what actually notices a push, and it
    // never blocks either - see its own comment.  Relevant here because a
    // push can be confirmed mid-relay of a write that is itself unrelated
    // to any mount - e.g. the CoCo-side bus_ready()/fuji_get_error() dance
    // that follows a mount write (see the "DBC (FujiBusPacket)" section's
    // own history of this).
    if (s_cart_push_confirmed) {
        drivewire_receive_confirmed_push();
    }
}

static void drivewire_do_read(uint16_t count) {
    // The CoCo is already polling for DW_STATUS_READY, and until now
    // DW_STATUS_ADDR holds the ROM's own byte - so put DW_STATUS_WAIT there
    // before anything slower.  (Setup checks that byte isn't ready either,
    // for the moment before this lands.)  This also starts the heartbeat
    // window (see drivewire_heartbeat()), which every exit from here to the
    // publish must close.
    s_heartbeat = DW_STATUS_WAIT;
    s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_heartbeat, 1u, 1u);
    s_drop_checksum = false;

    // Settle a probe the opportunistic drain left open before collecting
    // anything: its bytes come before any still in the FIFO, and this
    // function reads the FIFO directly, past drivewire_push_probe_feed().
    // Left open, they were missing from this reply and turned up in a later
    // one - and at 5 Mbaud a whole sector lands in the drain, so a $C0
    // anywhere in it could do this.  The probe also never disarms while open
    // (see s_push_probe_arm_budget).  So feed it until it decides; and if
    // nothing more comes for DW_REPLY_GAP_MS, its bytes were this reply's
    // tail, and go to s_read_buf as such.  A push it confirms is received
    // here, as the other two callers of drivewire_receive_confirmed_push()
    // would, and the reply follows.
    while (s_push_probe_state != PUSH_PROBE_IDLE) {
        int next = drivewire_read_session_getc(true);
        if (next == -2) {
            drivewire_push_probe_replay();
        } else if (next < 0) {
            // The CoCo left; the probe's bytes belonged to its reply.  Not
            // recorded - see s_last_error.
            s_push_probe_state = PUSH_PROBE_IDLE;
            s_read_buf_filled = 0;
            s_heartbeat = 0u;
            s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u, 1u);
            return;
        } else {
            drivewire_push_probe_feed((uint8_t)next);
        }
    }
    if (s_cart_push_confirmed) {
        drivewire_receive_confirmed_push();
    }

    // Before accepting this session's own first real reply byte, repeatedly
    // watch for (and fully consume) an unprompted cartridge push, but only
    // while s_push_probe_enabled (see drivewire_do_write()'s own comment
    // for what arms it, and drivewire_resolve_pushed_cart()'s for why this
    // function needs its own check rather than relying on
    // drivewire_push_probe_feed()).  The common case - not armed - costs
    // nothing extra: the condition short-circuits before ever touching
    // UART1 here.  Skipped even while armed once s_read_buf_filled is
    // already nonzero: that only happens when drivewire_next_addr()'s own
    // opportunistic drain (see its own comment) has already pulled in real
    // reply bytes before this session's knock was even seen, which means
    // the real reply has already started - and by design (the ESP32
    // finishes pushing before it ever replies), a push never starts
    // arriving after its own reply has begun.
    while (s_push_probe_enabled && s_read_buf_filled == 0u && count > 0u) {
        int next = drivewire_read_session_getc(false);
        if (next < 0) {
            // Not recorded - see s_last_error.
            s_heartbeat = 0u;
            s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u, 1u);
            return;
        }
        uint8_t b = (uint8_t)next;
        if (b != DBC_SLIP_END) {
            s_read_buf[s_read_buf_filled++] = b;
            break;
        }
        drivewire_resolve_pushed_cart();
    }

    // Finish draining the whole response from UART1 before any of the CoCo
    // handshake below - drivewire_next_addr() may already have opportunistically
    // pre-drained some of it into s_read_buf while this session's knock was
    // still being waited for (see its own comment for why that matters).
    // Whatever it's got is already correct and in order; only read fresh
    // bytes for the rest.  The remaining reads have no handshake overhead
    // (drivewire_uart_getc() is a handful of cycles), so this drains far
    // faster than bytes can arrive even at 5 Mbps.
    uint16_t already = (s_read_buf_filled < count) ? s_read_buf_filled : count;
    for (uint16_t i = already; i < count; i++) {
        int next = drivewire_read_session_getc(i > 0u);
        if (next == -2) {
            // The reply stalled part way - bytes were lost.  Hand the CoCo
            // what there is, the rest of s_read_buf as it stands, and drop
            // the checksum it sends back, so the ESP32 answers E$CRC and
            // HDB-DOS rereads the sector - see s_drop_checksum.
            drivewire_error(DW_ERR_READ_SHORT, i);
            s_drop_checksum = true;
            break;
        }
        if (next < 0) {
            // FujiNet never sent the rest of this response, or the CoCo gave
            // up waiting for it (see drivewire_read_session_getc()) - an
            // ESP32 still booting, or one still blocked reading an earlier,
            // corrupted request (see DW_UART_GETC_TIMEOUT's own comment).
            // Bail out to the next knock rather than wedging core 1 here:
            // DW_STATUS_ADDR is put back below.  s_read_buf_filled is reset so a partially-filled
            // buffer from this abort cannot be mistaken for a genuine
            // pre-drain by whatever session starts next.
            s_read_buf_filled = 0;
            // Not recorded - see s_last_error.
            s_heartbeat = 0u;
            s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u, 1u);
            return;
        }
        s_read_buf[i] = (uint8_t)next;
    }
    s_read_buf_filled = 0;

    s_heartbeat = 0u;  // close the heartbeat window - ready goes out next

    // Lay the response out in DW_WINDOW_ADDR for dwoneread.asm's stack blast,
    // whose PSHS fills the CoCo's buffer downwards, DW_BLAST_CHUNK bytes at a
    // time, while its PULU reads the window upwards.  So the chunks go in
    // reverse: the first count % DW_BLAST_CHUNK bytes (which the CoCo copies
    // singly, upwards), then the buffer's last chunk, then the one before,
    // and so on back to the head.  The status still shows the heartbeat's
    // last value meanwhile, so the CoCo goes on waiting.
    uint16_t head = count % DW_BLAST_CHUNK;
    bool laid_out = head == 0u ||
        s_reprogram(s_active_slot, DW_WINDOW_ADDR, s_read_buf, head, 1u) == ORA_RESULT_OK;
    uint32_t dst = DW_WINDOW_ADDR + head;
    for (uint16_t end = count; laid_out && end > head; end -= DW_BLAST_CHUNK) {
        laid_out = s_reprogram(s_active_slot, dst, &s_read_buf[end - DW_BLAST_CHUNK],
                               DW_BLAST_CHUNK, 1u) == ORA_RESULT_OK;
        dst += DW_BLAST_CHUNK;
    }

    if (!laid_out) {
        // The CoCo times out, as it would on a lost reply.
        drivewire_error(DW_ERR_REPROGRAM, DW_WINDOW_ADDR);
    } else {
        // Skip the backlog of the CoCo's status polls, which ran unthrottled
        // into the 64-entry ring while the response was collected and laid
        // out.  Safe to drop: the acknowledgement can't come before ready is
        // published below.
        s_read_idx = RING_BUF_CUR_WRITE_INDEX();

        // Ready last, so the CoCo never sees it over a half-written window.
        uint8_t ready = DW_STATUS_READY;
        if (s_reprogram(s_active_slot, DW_STATUS_ADDR, &ready, 1u, 1u) != ORA_RESULT_OK) {
            drivewire_error(DW_ERR_REPROGRAM, DW_STATUS_ADDR);
        } else {
            // False if the CoCo has left the session instead - not
            // recorded, see s_last_error.
            (void)drivewire_wait_for_ack();
        }
    }

    // Restore the real ROM content at the status address now the session is
    // over, so the code there is back to exactly what it was assembled as
    // until the next read session.  DW_WINDOW_ADDR needs no restore - see
    // its own comment.
    s_reprogram(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u, 1u);
}

// ---------------------------------------------------------------------------
// DBC (FujiBusPacket) - cartridge pushes and "!LDCART!"
// ---------------------------------------------------------------------------
//
// FujiNet pushes a mounted ROM image unprompted, over UART1 - same wire as
// ordinary DriveWire traffic - the moment it has one to send, using the same
// SLIP-framed "DBC" side channel FujiNet's `fujiversal`/`COCO_HS_UART`
// firmware already speaks to a pico-class companion elsewhere (see
// lib/device/drivewire/disk.cpp's push_rom_image() and
// lib/bus/rs232/FujiBusPacket.cpp in that tree for the authoritative wire
// format this mirrors - not lib/device/rs232/disk.cpp's own, differently-named
// push_stream(), which belongs to an unrelated device).  This plugin never
// asks for it: there is no request of its own on the wire, matching
// fujiversal's own auto-push-on-mount model rather than an on-demand pull.
//
// The gap between the write session that triggers a mount and the push that
// follows is NOT a fixed, short one: on real hardware it has been observed
// to run past 450ms (FujiNet's own mount+push processing, e.g. an SD card
// file open), so there is no sensibly-bounded timeout that could watch for
// it at any one single point.  Worse, that gap is not even one uninterrupted
// wait from this plugin's own perspective: the CoCo-side client code that
// triggers a mount (traced to fujinet-lib's fuji_mount_disk_image(),
// fuji_get_error() and bus_ready()) interleaves further small write/read
// pairs of its own (a status query, an error check, each wrapped in its own
// ready-retry loop) before ever reading the data a push precedes - so a
// push's own leading DBC_SLIP_END can legally arrive while this plugin is
// in the middle of relaying a *different*, unrelated write's bytes, not
// just while idly scanning for the next knock.
//
// Two earlier designs both failed on real hardware before landing on this
// one. First, detection ran unconditionally on every byte, everywhere
// drivewire_next_addr() is called, plus a separate check at the start of
// every drivewire_do_read() session, on the theory that
// dbc_header_looks_like_push()'s own four-field check made a false positive
// on ordinary data astronomically unlikely. Confirmed wrong in practice:
// disk sector data is not random (it's a compiled program and filesystem
// structure), and a false positive is not cheap - it blocks real DriveWire
// traffic for seconds while this plugin waits on a push that was never
// coming. Narrowing detection to run only during drivewire_wait_for_knock()'s
// own scan (the second design) didn't actually reduce that exposure either:
// that scan's own opportunistic drain is *exactly* the mechanism that
// catches a fast-arriving reply before its own read knock is even sent,
// which is the common case for ordinary sector data, not a rare one - so a
// false positive there blocked drivewire_wait_for_knock() itself, the one
// function responsible for ever recognizing the next knock at all, and no
// DriveWire traffic got through from the very first session onward.
//
// Detection is therefore content-armed instead of phase-scoped:
// drivewire_do_write() arms it (sets s_push_probe_enabled, see its own
// comment) only on recognizing a write that starts with the OP_FUJI/
// FUJICMD_MOUNT_IMAGE or FUJICMD_MOUNT_ALL opcode bytes - the only writes
// that can ever cause FujiNet to push a ROM image in the first place.
// Ordinary disk I/O (the vast majority of all traffic) never arms it at
// all, so the false-positive exposure that broke the first two designs
// simply never applies to it. Once armed, drivewire_push_probe_feed() (fed
// from drivewire_next_addr()'s own opportunistic drain) runs from inside
// both drivewire_wait_for_knock()'s knock-scan and drivewire_do_write()'s
// own relay loop, and drivewire_resolve_pushed_cart() additionally covers
// drivewire_do_read()'s own drivewire_uart_getc() loop, which bypasses
// drivewire_next_addr() entirely - between the three, a push is caught
// regardless of which of those three the CoCo-side bus_ready()/
// fuji_get_error() dance (see DW_PUSH_ARM_BUDGET's own comment) happens to
// have this plugin inside of when it actually starts arriving.
// s_push_probe_arm_budget bounds how long an arm window stays open with
// nothing confirmed, extended on every successful receive in case
// FUJICMD_MOUNT_ALL pushes more than one slot.
//
// dbc_header_looks_like_push() is what tells a real push apart from an
// ordinary reply that merely happens to start with the same byte value -
// see its own comment for why a marker byte alone is not trustworthy and
// the whole header has to check out first.  A false alarm's raw bytes are
// replayed into s_read_buf verbatim rather than discarded.  Once a push is
// confirmed, the exchange is fully committed and nothing else touches
// UART1 until it ends (dbc_recv_frame()/dbc_send_frame() use
// drivewire_uart_getc()/drivewire_uart_putc() directly, never
// drivewire_next_addr()'s own opportunistic drain), so there is no live
// DriveWire transaction for these frames to arrive in the middle of, unlike
// the mid-mailbox-transaction case FujiNet's own Intellivision port has to
// handle.
//
// "!LDCART!" never triggers or receives any of this itself - it only checks
// s_cart_pending and, if a push has already landed, activates it.  See
// drivewire_do_load_cart()'s own comment.  See the "Protocol constants"
// section above for the DBC_*/FUJI_OP_FUJI/FUJI_CMD_MOUNT_* byte values and
// DW_PUSH_ARM_BUDGET used throughout, and the "DBC (FujiBusPacket) helpers"
// section (also above) for dbc_checksum_add() and dbc_header_looks_like_push(),
// hoisted up there for drivewire_next_addr()'s own use.

// Sends one bare (no params, no payload) SLIP-framed DBC header - only ever
// used here for our own ACK/NAK replies, mirroring FujiNet's own
// dbc_send_frame() in the Intellivision port's fujinet.c.
static void dbc_send_frame(uint8_t command) {
    uint8_t header[DBC_HEADER_LEN];
    header[0] = DBC_FUJI_DEVICEID;
    header[1] = command;
    header[2] = DBC_HEADER_LEN;   // length, low byte
    header[3] = 0u;               // length, high byte
    header[4] = 0u;               // checksum - filled in below
    header[5] = 0u;               // descr - no packed params, ever, on this link

    uint8_t chk = 0u;
    for (unsigned i = 0; i < DBC_HEADER_LEN; i++) {
        chk = dbc_checksum_add(chk, header[i]);
    }
    header[4] = chk;

    drivewire_uart_putc(DBC_SLIP_END);
    for (unsigned i = 0; i < DBC_HEADER_LEN; i++) {
        uint8_t b = header[i];
        if (b == DBC_SLIP_END) {
            drivewire_uart_putc(DBC_SLIP_ESC);
            drivewire_uart_putc(DBC_SLIP_ESC_END);
        } else if (b == DBC_SLIP_ESC) {
            drivewire_uart_putc(DBC_SLIP_ESC);
            drivewire_uart_putc(DBC_SLIP_ESC_ESC);
        } else {
            drivewire_uart_putc(b);
        }
    }
    drivewire_uart_putc(DBC_SLIP_END);
}

// Decodes exactly DBC_HEADER_LEN logical bytes of a SLIP-framed DBC header
// from UART1, given the caller has already consumed the frame's own leading
// DBC_SLIP_END.  Unlike dbc_recv_frame_payload() below, this always reports
// the *raw*, still-SLIP-encoded bytes it consumed getting there (into raw,
// capped at raw_cap) - drivewire_resolve_pushed_cart() needs those to
// replay verbatim into s_read_buf if they turn out not to belong to a real
// push after all (see its own comment for why a header full of plausible
// field values is still not proof enough on its own).  Pass raw=NULL to skip
// that capture, for dbc_recv_frame()'s own use below, where nothing needs
// replaying - by the time it is scanning for a second or later frame, a push
// is already confirmed under way and nothing else is sharing the wire.
//
// Returns false on a timeout or a frame that ends (another DBC_SLIP_END)
// before a full header is decoded - raw/raw_len are valid either way, since
// even a failed probe's bytes must still be replayable.  *chk_out holds the
// running checksum over the decoded header (its own checksum field
// contributing 0, matching the sender zeroing it out before computing its
// own), for dbc_recv_frame_payload() to continue accumulating.
static bool dbc_decode_header_probe(uint8_t header[DBC_HEADER_LEN], uint8_t *chk_out,
                                     uint8_t *raw, uint16_t *raw_len, uint16_t raw_cap) {
    enum { IN_FRAME, IN_ESCAPE } state = IN_FRAME;
    uint16_t decoded_len = 0;
    uint8_t chk = 0;
    *raw_len = 0;

    while (decoded_len < DBC_HEADER_LEN) {
        uint8_t b;
        if (!drivewire_uart_getc(&b)) {
            return false;
        }
        if (raw != NULL && *raw_len < raw_cap) {
            raw[(*raw_len)++] = b;
        }

        bool have_byte = false;
        uint8_t decoded_byte = 0;

        if (state == IN_ESCAPE) {
            state = IN_FRAME;
            if (b == DBC_SLIP_ESC_END) { decoded_byte = DBC_SLIP_END; have_byte = true; }
            else if (b == DBC_SLIP_ESC_ESC) { decoded_byte = DBC_SLIP_ESC; have_byte = true; }
            // else: malformed escape - abort rather than leniently dropping
            // it and continuing (unlike dbc_recv_frame_payload()'s own
            // leniency, mirroring decodeSLIP(), once a push is already
            // confirmed).  A real DBC sender never emits an invalid escape
            // pair, so this is itself proof this isn't one.
            else { return false; }
        } else if (b == DBC_SLIP_END) {
            if (decoded_len != 0) {
                return false;  // frame ended before a full header - not real
            }
            // else: this was just the frame's own opening marker, repeated
        } else if (b == DBC_SLIP_ESC) {
            state = IN_ESCAPE;
        } else {
            decoded_byte = b;
            have_byte = true;
        }

        if (have_byte) {
            header[decoded_len] = decoded_byte;
            chk = dbc_checksum_add(chk, decoded_len == 4u ? 0u : decoded_byte);
            decoded_len++;
        }
    }

    *chk_out = chk;
    return true;
}

// Reads and validates a DBC frame's payload and trailing checksum, given its
// header has already been decoded (dbc_decode_header_probe() above) and chk
// is the running checksum over that header.  Any payload streams directly
// into s_read_buf, reused here purely as scratch space - nothing else needs
// it while a cartridge push is being received, and it is already sized to
// fit push_rom_image()'s own 256-byte (MEDIA_BLOCK_SIZE) NET_WRITE chunks
// exactly.  This plugin never attempts to interpret FujiBusPacket's own
// descriptor-packed-params encoding (descr is NOT always 0 on the wire -
// push_rom_image()'s own NET_OPEN frame sends one packed 2-byte param, descr
// 0x05, not a raw payload - see dbc_header_looks_like_push()'s own comment
// for the full trace) - it doesn't need to: whatever bytes follow the
// header, packed param or raw data alike, land in the same place on the
// wire, so treating all of it uniformly as unconditional payload - no
// descriptor/param parsing to implement - works regardless of which one a
// given frame actually is.  Times out (see
// drivewire_uart_getc()) rather than blocking forever if the ESP32 stops
// responding mid-exchange.
static bool dbc_recv_frame_payload(const uint8_t header[DBC_HEADER_LEN], uint8_t chk,
                                    uint8_t *device, uint8_t *command, uint16_t *payload_len) {
    enum { IN_FRAME, IN_ESCAPE } state = IN_FRAME;
    uint16_t decoded_len = DBC_HEADER_LEN;
    bool frame_complete = false;

    while (!frame_complete) {
        uint8_t b;
        if (!drivewire_uart_getc(&b)) {
            return false;
        }

        bool have_byte = false;
        uint8_t decoded_byte = 0;

        if (state == IN_ESCAPE) {
            state = IN_FRAME;
            if (b == DBC_SLIP_ESC_END) { decoded_byte = DBC_SLIP_END; have_byte = true; }
            else if (b == DBC_SLIP_ESC_ESC) { decoded_byte = DBC_SLIP_ESC; have_byte = true; }
            // else: malformed escape - drop it, matching decodeSLIP()'s own leniency
        } else if (b == DBC_SLIP_END) {
            frame_complete = true;  // decoded_len is always != 0 here
        } else if (b == DBC_SLIP_ESC) {
            state = IN_ESCAPE;
        } else {
            decoded_byte = b;
            have_byte = true;
        }

        if (have_byte) {
            if ((decoded_len - DBC_HEADER_LEN) < sizeof(s_read_buf)) {
                s_read_buf[decoded_len - DBC_HEADER_LEN] = decoded_byte;
            }
            // else: payload longer than s_read_buf can hold - should never
            // happen (see this function's own comment); the length/checksum
            // check below catches it rather than silently accepting a
            // truncated payload.
            chk = dbc_checksum_add(chk, decoded_byte);
            decoded_len++;
        }
    }

    uint16_t claimed_len = (uint16_t)header[2] | ((uint16_t)header[3] << 8);
    if (claimed_len != decoded_len || chk != header[4]) {
        return false;
    }

    *device = header[0];
    *command = header[1];
    *payload_len = (uint16_t)(decoded_len - DBC_HEADER_LEN);
    return true;
}

// Scans for the next frame's own leading DBC_SLIP_END, then decodes it in
// full.  Only used once a push is already underway
// (drivewire_dbc_receive_cart()'s own loop, fetching its second and later
// frames) - the very first frame is always told apart from an ordinary
// reply's first byte by one of drivewire_push_probe_feed() or
// drivewire_resolve_pushed_cart() instead (see the "DBC (FujiBusPacket)"
// section header comment above for which covers what), both of which
// additionally validate the header's own fields before trusting it (see
// dbc_header_looks_like_push()'s own comment for why the marker byte alone
// is not enough).
static bool dbc_recv_frame(uint8_t *device, uint8_t *command, uint16_t *payload_len) {
    uint8_t b;
    do {
        if (!drivewire_uart_getc(&b)) {
            return false;
        }
    } while (b != DBC_SLIP_END);

    uint8_t header[DBC_HEADER_LEN];
    uint8_t chk;
    uint16_t raw_len;
    if (!dbc_decode_header_probe(header, &chk, NULL, &raw_len, 0)) {
        return false;
    }
    return dbc_recv_frame_payload(header, chk, device, command, payload_len);
}

// Runs the NET_OPEN/NET_WRITE.../NET_CLOSE loop that receives a pushed
// cartridge image into DW_CART_RAM_SLOT (never s_active_slot - see that
// macro's own comment), given the first frame's header has already been
// decoded by the caller.  Sets s_cart_pending on a complete, well-formed
// transfer; DW_CART_RAM_SLOT's contents are undefined on failure, matching
// "nothing has been activated" the same way a failed "!LDCART!" load used to
// promise.
static bool drivewire_dbc_receive_cart(uint8_t device, uint8_t command, uint16_t payload_len) {
    uint32_t slot_size = 0;
    ora_result_t size_rc = s_get_ram_slot_info(DW_CART_RAM_SLOT, NULL, &slot_size, NULL);
    // Everything below is bounded by the chip's addressable window, never
    // slot_size - see s_cart_window_size's own comment.
    uint32_t window_size = s_cart_window_size;

    uint32_t offset = 0;
    for (;;) {
        if (device != DBC_FUJI_DEVICEID) {
            drivewire_error(DW_ERR_PUSH_DEVICE, device);
            return false;
        }

        if (command == DBC_CMD_NET_OPEN) {
            // Carries no image size at all - verified against
            // FujiBusPacket::serialize(): push_rom_image()'s NET_OPEN is a
            // single packed 2-byte parameter, always the literal value 0,
            // not a raw payload (see dbc_header_looks_like_push()'s own
            // comment for the full wire-format trace).  An earlier version
            // of this code assumed a 4-byte size followed a 1-byte stream
            // id here - wrong, and the reason every push attempt failed at
            // NET_OPEN for the entire time that assumption stood: the real
            // frame's own descr/length never matched what
            // dbc_header_looks_like_push() was checking for.  With no size
            // to validate against window_size up front, this only confirms
            // the slot and window themselves are valid; DBC_CMD_NET_WRITE
            // below bounds the actual transfer as data arrives, which is the
            // only point a real size ever becomes known.
            offset = 0;
            s_cart_received_checksum = 0;
            bool slot_ok = (size_rc == ORA_RESULT_OK &&
                            window_size != 0u && window_size <= slot_size);
            dbc_send_frame(slot_ok ? DBC_CMD_FUJI_ACK : DBC_CMD_FUJI_NAK);
            drivewire_heartbeat();  // see the NET_WRITE case below
            if (!slot_ok) {
                drivewire_error(DW_ERR_PUSH_SLOT, DW_CART_RAM_SLOT);
                return false;
            }
        } else if (command == DBC_CMD_NET_WRITE) {
            if (offset + payload_len > window_size) {
                drivewire_error(DW_ERR_PUSH_TOO_BIG, (uint16_t)(window_size / 1024u));
                dbc_send_frame(DBC_CMD_FUJI_NAK);
                return false;
            }
            for (uint16_t i = 0; i < payload_len; i++, offset++) {
                s_reprogram(DW_CART_RAM_SLOT, offset, &s_read_buf[i], 1u, 0u);
                // Plain sum, matching FujiNet's own
                // drivewire_checksum() exactly (see dwload_cmoc.c's copy of
                // it), so fujinet-config can recompute the same value.
                // Capped at the absolute $C000+0x3F00 (=$FF00) boundary -
                // every CoCo decodes that as real I/O regardless of image
                // size, so bytes past it are never ROM-bus-readable and
                // would make this incomparable to the CoCo's own (equally
                // capped) checksum.
                if (offset < 0x3F00u) {
                    s_cart_received_checksum = (uint16_t)(s_cart_received_checksum + s_read_buf[i]);
                }
            }
            dbc_send_frame(DBC_CMD_FUJI_ACK);
            // One beat per frame, every few milliseconds of a push, so a
            // CoCo waiting on the reply behind it doesn't time out - see
            // drivewire_heartbeat().  A no-op for a push arriving outside a
            // read session.
            drivewire_heartbeat();
        } else if (command == DBC_CMD_NET_CLOSE) {
            // Abort-CLOSE (payload 0x01): FujiNet gave up mid-stream - unwedge
            // without activating partial data.  Bare CLOSE = a complete transfer.
            bool aborted = (payload_len > 0u && s_read_buf[0] == 0x01u);
            dbc_send_frame(DBC_CMD_FUJI_ACK);
            if (aborted) {
                return false;
            }
            s_cart_received_len = (uint16_t)offset;

            // A real EPROM smaller than this emulation's full address
            // window (e.g. a 4K/8K chip on a profile that decodes the
            // full 16K) leaves its own upper address lines unconnected,
            // so its content repeats every <image size> bytes across the
            // whole window - any access past the image's own end reads a
            // wrapped copy of the start on real hardware, not open bus or
            // whatever a previous push happened to leave there. Mirror
            // the just-received image across the rest of the slot the
            // same way. Read back from the slot itself (not s_read_buf,
            // which only ever holds the most recent network frame) in
            // chunks clipped to the image's own end, so each chunk's
            // source bytes are always ones that have already landed.
            if (offset > 0 && offset < window_size) {
                uint32_t dst = offset;
                while (dst < window_size) {
                    uint32_t src_off = dst % offset;
                    uint32_t chunk = window_size - dst;
                    uint32_t avail = offset - src_off;
                    if (chunk > avail) chunk = avail;
                    if (chunk > sizeof(s_read_buf)) chunk = sizeof(s_read_buf);
                    if (s_read_slot(DW_CART_RAM_SLOT, src_off, s_read_buf, chunk) != ORA_RESULT_OK)
                        break;
                    if (s_reprogram(DW_CART_RAM_SLOT, dst, s_read_buf, chunk, 0u) != ORA_RESULT_OK)
                        break;
                    dst += chunk;
                }
            }

            s_cart_pending = true;
            return true;
        } else {
            dbc_send_frame(DBC_CMD_FUJI_NAK);
        }

        if (!dbc_recv_frame(&device, &command, &payload_len)) {
            drivewire_error(DW_ERR_PUSH_FRAME, 1u);
            return false;
        }
    }
}

// Called from drivewire_do_read(), having already consumed a DBC_SLIP_END as
// the very first byte of what might be an unprompted cartridge push, only
// while s_push_probe_enabled (see that function's own comment and
// drivewire_do_write()'s for what arms it).  This covers the one gap
// drivewire_push_probe_feed() can't: drivewire_do_read()'s own
// drivewire_uart_getc() loop bypasses drivewire_next_addr() entirely, so a
// push arriving right as this exact read session begins - e.g. the mount's
// own "real" reply, which the CoCo-side bus_ready()/fuji_get_error() dance
// means is not necessarily the very next read after the triggering write -
// would otherwise go unnoticed here.
//
// A single DBC_SLIP_END byte is NOT enough on its own to conclude a push is
// starting, even while armed: it is the server's genuine first reply byte
// for an ordinary, non-push transaction too, and any byte value can appear
// there by sheer coincidence of whatever data the reply happens to carry
// (confirmed on real hardware, before arming existed at all: a CRC error
// reading one specific disk block, every time, traced to that block's own
// data producing a stray $C0 here).  So this goes on to probe the rest of
// the header - device, command, descr and a sane length - before ever
// treating those bytes as spent on a push rather than real reply data.
// dbc_decode_header_probe() captures every raw byte it consumes doing this;
// if the header doesn't hold up, this replays them - the leading
// DBC_SLIP_END included - into s_read_buf verbatim, exactly as
// drivewire_next_addr()'s opportunistic drain would have delivered them, so
// a false alarm costs nothing and loses no data: drivewire_do_read()'s own
// retry loop sees s_read_buf_filled become nonzero and falls through to
// treating everything from there on as the real reply.  Only once the
// header's fields all check out does this commit to receiving an actual
// push via drivewire_dbc_receive_cart(), leaving s_read_buf_filled at 0 so
// that same retry loop keeps watching - either for another push, or for the
// real reply that follows it.
static void drivewire_resolve_pushed_cart(void) {
    uint8_t header[DBC_HEADER_LEN];
    uint8_t chk;
    uint8_t raw[DBC_PROBE_RAW_CAP];
    uint16_t raw_len;
    bool header_ok = dbc_decode_header_probe(header, &chk, raw, &raw_len, sizeof(raw));

    if (!header_ok || !dbc_header_looks_like_push(header)) {
        if (s_read_buf_filled < sizeof(s_read_buf)) {
            s_read_buf[s_read_buf_filled++] = DBC_SLIP_END;
        }
        for (uint16_t i = 0; i < raw_len && s_read_buf_filled < sizeof(s_read_buf); i++) {
            s_read_buf[s_read_buf_filled++] = raw[i];
        }
        return;
    }

    uint8_t device, command;
    uint16_t payload_len;
    if (!dbc_recv_frame_payload(header, chk, &device, &command, &payload_len)) {
        drivewire_error(DW_ERR_PUSH_FRAME, 2u);
        return;  // s_read_buf_filled stays 0 - drivewire_do_read()'s retry
                 // loop will try again, treating whatever comes next as a
                 // fresh attempt.
    }
    drivewire_dbc_receive_cart(device, command, payload_len);
    s_push_probe_arm_budget = DW_PUSH_ARM_BUDGET;  // extend the window - see
                                                    // drivewire_receive_confirmed_push()'s
                                                    // own comment.
}

// Called by drivewire_wait_for_knock()/drivewire_do_write() the moment
// either notices s_cart_push_confirmed, and by drivewire_do_read() when the
// probe it settles confirms one - there the CoCo is polling for ready, kept
// waiting by the push's own heartbeats.  drivewire_push_probe_feed()
// already validated the header's own fields by the time either runs, so
// there is no "turned out not to be a push" path left to handle here.
//
// Safe to block inside dbc_recv_frame_payload()/drivewire_dbc_receive_cart()
// here specifically because of which two places call this: the CoCo is
// either redundantly spin-polling an address that has nothing to do with
// this (drivewire_wait_for_knock(), mid-knock-scan) or has already finished
// sending everything it's going to send for the current write before this
// runs (drivewire_do_write(), checked only after its own relay loop
// completes) - never mid-relay of a write's own time-sensitive bytes, which
// is the one context this plugin cannot afford to stall.
static void drivewire_receive_confirmed_push(void) {
    s_cart_push_confirmed = false;

    uint8_t device, command;
    uint16_t payload_len;
    if (!dbc_recv_frame_payload(s_cart_push_header, s_cart_push_chk,
                                 &device, &command, &payload_len)) {
        drivewire_error(DW_ERR_PUSH_FRAME, 3u);
        return;
    }
    drivewire_dbc_receive_cart(device, command, payload_len);
    // Extend the arm window rather than letting it merely run its course -
    // FUJICMD_MOUNT_ALL can push more than one slot in sequence, and each
    // push's own completion is otherwise indistinguishable from "nothing
    // more is coming" until the budget would have run out anyway.
    s_push_probe_arm_budget = DW_PUSH_ARM_BUDGET;
}

// Handles a "!LDCART!" knock: if drivewire_receive_confirmed_push()/
// drivewire_dbc_receive_cart() have already landed a complete cartridge
// image in DW_CART_RAM_SLOT, atomically activates it; otherwise fails
// exactly as "no One ROM present" always has.  Not part of DriveWire proper
// - a One-ROM-specific extension the config app speaks directly (see
// s_knock_cart's own comment) to transiently boot a user-chosen cartridge
// image.  Unlike before, this knock neither requests nor receives anything
// of its own - the push already happened, unprompted, after whatever write
// session triggered FujiNet to mount it - so there is nothing left to do
// here but check s_cart_pending and switch.  The caller is expected to jump
// to $C000 once it sees the completion ack this sets, and neither side
// expects any further DriveWire traffic afterwards - only a power cycle
// restores normal FujiNet operation, so nothing here worries about leaving
// state behind for a later session.
//
// The completion ack deliberately goes through s_active_slot's own
// DW_KNOCK_ACK_ADDR - still slot 0 at this point, still the byte this
// plugin already owns and restores - and only *then* does the switch
// happen.  It cannot instead be written into DW_CART_RAM_SLOT itself: that
// slot's contents are an arbitrary user-chosen cartridge image whose layout
// this plugin has no knowledge of, so writing a signalling byte anywhere
// within it risks corrupting the very image just loaded.  The caller is
// expected to pause briefly after seeing the ack before jumping, as cheap
// insurance against the (already tiny, given the ~100x clock difference
// between this CPU and a 6809) gap between this ack and the slot switch
// actually completing.
//
// Returns true on success - at which point the caller MUST stop calling
// drivewire_wait_for_knock()/drivewire_next_addr() (see drivewire_main()'s
// own dispatch) rather than looping back to it.  Once the CoCo jumps to
// $C000, every single instruction fetch is itself a ROM-bus read; the
// cart's own code generates these at full CPU speed, nothing like a
// DriveWire session's own deliberately paced trickle of knock/count/data
// reads, and the knock-matching loop's ring buffer (and the address
// monitor's own DMA/FIFO capture path, shared with ROM-serving - see commit
// aa765b4) was never sized or prioritised for that load.  Confirmed on real
// hardware: leaving this loop running after a successful load corrupts served
// reads within the cart's own first ~150 instruction fetches and then wedges
// the bus outright.  Returns false when nothing has been pushed yet (or a
// push failed/didn't fit) - the caller keeps monitoring normally, since
// nothing has been activated and slot 0 is still what is being served.
static bool drivewire_do_load_cart(void) {
    // Clear the knock-accept ack drivewire_wait_for_knock() just set for
    // match_cart, before it can be mistaken for the load-complete ack set
    // below - see that function's own comment for why this is safe and why
    // narrowing its ack to match_write only (removing the need for this
    // clear) is off the table.
    s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u, 1u);

    if (!s_cart_pending) {
        // Explicit failure status, so
        // onerom_load_cart() doesn't have to infer "not pending" purely
        // from its own knock-ack timing out.
        uint8_t status = DW_CART_STATUS_FAILURE;
        s_reprogram(s_active_slot, DW_CART_STATUS_ADDR, &status, 1u, 1u);
        return false;
    }
    s_cart_pending = false;

    uint8_t ack = DW_KNOCK_ACK_VALUE;
    s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &ack, 1u, 1u);

    // Report what this plugin actually received, so
    // fujinet-config can walk the activated cart's own memory afterward and
    // recompute the same checksum independently. Written into slot 0
    // (still active here) before the switch below, same as the ack above.
    uint8_t len_lo = (uint8_t)(s_cart_received_len & 0xFFu);
    uint8_t len_hi = (uint8_t)(s_cart_received_len >> 8);
    uint8_t cksum_lo = (uint8_t)(s_cart_received_checksum & 0xFFu);
    uint8_t cksum_hi = (uint8_t)(s_cart_received_checksum >> 8);
    s_reprogram(s_active_slot, DW_CART_LEN_LO_ADDR, &len_lo, 1u, 1u);
    s_reprogram(s_active_slot, DW_CART_LEN_HI_ADDR, &len_hi, 1u, 1u);
    s_reprogram(s_active_slot, DW_CART_CKSUM_LO_ADDR, &cksum_lo, 1u, 1u);
    s_reprogram(s_active_slot, DW_CART_CKSUM_HI_ADDR, &cksum_hi, 1u, 1u);
    uint8_t status = DW_CART_STATUS_SUCCESS;
    s_reprogram(s_active_slot, DW_CART_STATUS_ADDR, &status, 1u, 1u);

    // A deliberate pause before the switch below, re-added (2026-10-14) and
    // justified independently of an earlier attempt. Real-hardware testing with the pushed image being
    // byte-identical to what slot 0 already serves (hdbonerom.rom itself)
    // still hung the CoCo's poll with no recovery - which rules out any
    // explanation that depends on the *new* slot's own content differing
    // from the old one (e.g. a corrupted IRQ vector), since here it
    // doesn't. That leaves the switch itself, or reading near it, as
    // unsafe independent of what either slot contains - exactly what this
    // pause is for. Sized far more generously than the previous attempt
    // (~100x more cycles): that one was reverted after a report of
    // corruption on unrelated ordinary traffic, but causation was never
    // actually isolated - other unrelated edits were in flight in this
    // same file at the time. Treat this specific value as unverified until
    // tested in isolation again.
    for (volatile int d = 0; d < 500000; d++) { }

    s_set_active_slot(DW_CART_RAM_SLOT);
    return true;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

// Sets s_cart_window_size from the served chip type's size.  The cartridge
// slot is served with the same chip configuration as the flash slot selected
// at boot, so that slot's chip size is the window a pushed image has to fit
// and mirror within - see s_cart_window_size's own comment.  ROM_SLOT_INDEX
// counts every flash slot, plugins included, which is the indexing
// get_flash_slot_ext_info() uses with no filter flags.
//
// noinline so these locals stay out of drivewire_setup()'s own frame, which
// is live underneath pio_setup_address_monitor()'s 208-byte one - the
// plugin's tightest stack point (see the comment above that call).
__attribute__((noinline)) static void drivewire_resolve_cart_window(ora_lookup_fn_t ora_lookup_fn) {
    ora_get_metadata_uint_fn_t get_metadata_uint = ora_lookup_fn(ORA_ID_GET_METADATA_UINT);
    ora_get_flash_slot_ext_info_fn_t get_flash_slot_ext_info =
        ora_lookup_fn(ORA_ID_GET_FLASH_SLOT_EXT_INFO);
    uint32_t rom_slot_index = 0;
    uint32_t chip_size = 0;
    if (get_metadata_uint(ORA_METADATA_KEY_ROM_SLOT_INDEX, &rom_slot_index) == ORA_RESULT_OK &&
        get_flash_slot_ext_info((uint8_t)rom_slot_index, 0u, 0u,
                                NULL, NULL, &chip_size, NULL) == ORA_RESULT_OK) {
        s_cart_window_size = chip_size;
    } else {
        drivewire_error(DW_ERR_CART_WINDOW, 0u);  // cartridge pushes will be refused
    }
}

static void drivewire_setup(ora_lookup_fn_t ora_lookup_fn) {
    s_demangle        = ora_lookup_fn(ORA_ID_DEMANGLE_ADDR);
    s_reprogram       = ora_lookup_fn(ORA_ID_REPROGRAM_RAM_ROM_SLOT);
    s_read_slot       = ora_lookup_fn(ORA_ID_READ_RAM_ROM_SLOT);
    s_get_active_slot = ora_lookup_fn(ORA_ID_GET_ACTIVE_RAM_SLOT);
    s_set_active_slot = ora_lookup_fn(ORA_ID_SET_ACTIVE_RAM_SLOT);
    s_get_ram_slot_info = ora_lookup_fn(ORA_ID_GET_RAM_SLOT_INFO);
    s_get_sysclk_mhz  = ora_lookup_fn(ORA_ID_GET_SYSCLK_MHZ);
    s_set_status_led  = ora_lookup_fn(ORA_ID_SET_STATUS_LED);
    s_uptime_ms       = ora_lookup_fn(ORA_ID_GET_PLUGIN_UPTIME_MS);

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
    //
    // High priority stops captures being lost under heavy ROM read traffic.
    // At normal priority the CoCo's own boot, before it speaks this protocol,
    // dropped captures with no trace - a ring-backlog high-water-mark
    // diagnostic pegged at maximum.
    const ora_address_monitor_options_t monitor_opts = {
        .size     = sizeof(ora_address_monitor_options_t),
        .priority = ORA_ADDRESS_MONITOR_PRIORITY_HIGH,
    };
    bool monitor_ok = false;
    ora_result_t monitor_rc = setup_monitor(
        ring_buf,
        RING_ENTRIES_LOG2,
        ORA_MONITOR_MODE_CONTROL,
        RING_DATA_SIZE,
        &monitor_opts
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
        drivewire_error(DW_ERR_NO_ACTIVE_SLOT, 0u);
        s_active_slot = 0u;
    }

    drivewire_resolve_cart_window(ora_lookup_fn);

    // Snapshot the real bytes at the channel addresses once, so sessions can
    // restore them afterwards - see the file header comment.
    ora_result_t snap1_rc = s_read_slot(s_active_slot, DW_STATUS_ADDR, &s_orig_status_byte, 1u);
    ora_result_t snap2_rc = s_read_slot(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u);

    if (snap1_rc != ORA_RESULT_OK || snap2_rc != ORA_RESULT_OK) {
        drivewire_error(DW_ERR_SNAPSHOT, 0u);
    } else if (s_orig_status_byte == DW_STATUS_READY) {
        // The CoCo polls for ready from the moment it sends a read's count,
        // a little before drivewire_do_read() can put DW_STATUS_WAIT there,
        // and would copy a window that isn't laid out yet.  Can only happen
        // with an HDB-DOS build whose code at $C100 changed - recorded so
        // the corrupt reads it causes have a cause to find.
        drivewire_error(DW_ERR_STATUS_BYTE, s_orig_status_byte);
    }

    // UART1 last: no dependency on the monitor or slot calls above, and
    // gating it behind those succeeding would mean a failure there silently
    // skips UART bring-up too - from outside, indistinguishable from "setup
    // succeeded, nothing to relay yet".
    drivewire_uart_init();


    if (!monitor_ok) {
        if (monitor_rc != ORA_RESULT_OK) {
            drivewire_error(DW_ERR_MONITOR_SETUP, (uint16_t)monitor_rc);
        } else {
            drivewire_error(DW_ERR_MONITOR_WRITE_POS, 0u);
        }
        return;
    }
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


    drivewire_setup(ora_lookup_fn);

    for (;;) {
        drivewire_check_uart_rx();
        s_debug_main_phase = 0u;
        session_dir_t dir = drivewire_wait_for_knock();
        s_debug_session_count++;

        if (dir == SESSION_LOAD_CART) {
            // Unlike a write/read session, "!LDCART!" carries no count or
            // data of its own on the ROM bus at all - the whole exchange
            // happens over UART1 instead, so there is nothing more to read
            // off the address bus before dispatching.  See
            // drivewire_do_load_cart()'s own comment.
            s_debug_main_phase = 4u;
            s_drop_checksum = false;
            if (drivewire_do_load_cart()) {
                // Success: the CoCo is about to jump into the freshly-loaded
                // cart and run it at full speed.  Stop monitoring the ROM
                // bus entirely from here on - see drivewire_do_load_cart()'s
                // own comment on why continuing to do so corrupts/wedges the
                // cart's own execution.  Only a power cycle recovers normal
                // operation, matching every other comment on this knock.
                s_debug_main_phase = 5u;
                for (;;) {
                    ORA_TEST_YIELD();
                }
            }
            continue;
        }

        s_debug_main_phase = 1u;
        // 16-bit count, high byte first - matches dwonewrite.asm's/
        // dwoneread.asm's identical encoding (see either's own comment).
        // No more 0-means-256 special case: 16 bits already covers the
        // dw.h-documented 0-65535 range directly.
        int count_hi = drivewire_session_byte();
        int count_lo = (count_hi < 0) ? -1 : drivewire_session_byte();
        if (count_lo < 0) {
            // The CoCo left before sending its count - not recorded, see
            // s_last_error.
            s_drop_checksum = false;
            s_reprogram(s_active_slot, DW_KNOCK_ACK_ADDR, &s_orig_ack_byte, 1u, 1u);
            continue;
        }
        uint16_t count = (uint16_t)(((uint16_t)count_hi << 8) | count_lo);

        if (dir == SESSION_WRITE) {
            // No buffer here - drivewire_do_write() streams the ROM bus
            // straight to UART1 one byte at a time, so any count up to
            // 65535 is safe.
            s_debug_main_phase = 2u;
            drivewire_do_write(count);
        } else if (count <= sizeof(s_read_buf)) {
            s_debug_main_phase = 3u;
            drivewire_do_read(count);
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
            s_drop_checksum = false;
            drivewire_error(DW_ERR_READ_COUNT, count);
        }
    }
}
