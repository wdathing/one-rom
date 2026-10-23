# FujiNet/Centipede ROM-load protocol - handoff notes

Written from the `one-rom` session back to the `centi-fuji` session
(`~/Workspace/centi-fuji`, `FujiNetWIFI/fujinet-firmware` fork `wdathing/dragon`), continuing the
`centi-fuji` session's own handoff below. Supersedes the original version of this file, which
assumed the on-demand `FUJI_PULL_ROM` design that has since been dropped - see "Design decision"
below. Both sides have an implementation, and an end-to-end push-and-activate has been confirmed
working on real hardware - see "End-to-end success" below for that, and "Bug #8" for a regression
found and reverted afterward whose mechanism is still not understood. Still uncommitted on this
plugin's side.

## Design decision: auto-push, not pull

PR #1630 (`FUJI_PULL_ROM`, on-demand pull) was closed deliberately on 2026-10-02, in favour of
matching `fujiversal`'s own model instead: the ESP32 pushes a mounted ROM image unprompted, the
instant it has one to send, rather than waiting for a request. Reasoning, worked through across
both sessions:

- `fujiversal`'s companion (`pico/fujiversal/`) is a pure ROM-pak emulator with no DriveWire disk
  bridging sharing the wire - nothing else is ever using the link when its push happens, so there
  is no race to design around. Centipede's companion (a One ROM) *does* share the wire with
  ordinary DriveWire disk traffic, so this isn't automatically safe to copy as-is.
- Tracing `CMD::FUJI_MOUNT_IMAGE`/`FUJI_MOUNT_ALL`'s call chain in `fujiDevice.cpp`: a mount
  triggered by a DriveWire command the CoCo itself sent is synchronous end to end -
  `fujicmd_mount_disk_image_success()` -> `disk.mount()` -> `push_rom_image()` fully returns
  *before* `SYSTEM_BUS.transaction_success()` sends the CoCo its reply. So for that specific
  trigger, the CoCo is necessarily blocked waiting for the exact reply the push is a prefix of -
  no race possible.
- `fujicore_mount_all_success()` has two *other* callers - `lib/hardware/keys.cpp` (the physical
  button) and the HTTP config server (`lib/http/httpService.cpp`/`mgHttpService.cpp`) - that are
  **not** synchronised with the CoCo and could race a live transaction. Decision: restrict ROM
  mounting that pushes a cartridge image to the CoCo-initiated path only. Accepted as a reasonable
  restriction for this use case (a jumper-wire-free alternative would need extra hardware anyway).
- Resulting handshake, settled across both sessions: the One ROM plugin always listens for a push
  at the one point it can safely arrive (see below), buffers it separately from the live-served
  image, and only activates it when `!LDCART!` is knocked - decoupling "when does the ESP32
  happen to push" from "when is the CoCo actually ready to jump into it".

## One ROM side - done

`plugins/user/drivewire/src/drivewire_main.c` implements this in full:

- `drivewire_do_write()` calls `drivewire_check_for_pushed_cart()` the instant a write session's
  data has been fully relayed to UART1 TX - the only point established as safe per the race
  analysis above, and the only point a push can start arriving without being confused for an
  ordinary reply's own first byte.
- That function peeks one UART1 RX byte with a short timeout (`DW_PUSH_PEEK_TIMEOUT` - a
  placeholder value, not yet tuned against real hardware). `DBC_SLIP_END` (`$C0`) means a push is
  starting; anything else is preserved as the real reply's first byte.
- A push is received via `drivewire_dbc_receive_cart()` (the `NET_OPEN`/`NET_WRITE`.../`NET_CLOSE`
  loop) straight into a dedicated RAM slot (`DW_CART_RAM_SLOT`, never the live-served slot), and
  sets `s_cart_pending` on a complete, well-formed transfer.
- `!LDCART!` (`drivewire_do_load_cart()`) no longer requests or receives anything - it only checks
  `s_cart_pending` and, if set, atomically activates the already-buffered image. This is a
  behaviour change from the original "!LDCART!" protocol (which used to trigger the pull itself,
  via the now-removed `FUJI_PULL_ROM` opcode) - any CoCo-side or config-app code that still expects
  `!LDCART!` to *request* a load needs to change to: trigger the mount through the normal DW3 path
  first (so the ESP32 pushes), then knock `!LDCART!` once ready to jump.
- Built and flashed to a Fire 28 C board (firmware v0.7.3) on 2026-10-09; compiles clean under
  `-Wall -Wextra -Werror`. Not yet testable end to end - nothing pushes a cartridge image until
  the ESP32 side below exists.

## FujiNet/ESP32 side - done

Implemented in `~/Workspace/centi-fuji` on branch `centipede-rom-autopush` (off current
`upstream/master`, not yet pushed anywhere). Six files, ~107 lines:

- `lib/bus/drivewire/drivewire.h` / `drivewire.cpp` - the five-plus-one `#ifdef
  PINMAP_FUJIVERSAL_DRIVEWIRE` blocks (the `_dbc_pushback` deque, the `read()`/`available()`
  overrides, `readBusPacket()`/`writeBusPacket()`/`sendCommand()`) are now gated on a new
  `DRIVEWIRE_DBC_SUPPORTED` macro, `#define`d when either `PINMAP_FUJIVERSAL_DRIVEWIRE` or
  `COCO_HS_UART` is set. This plumbing compiles for Centipede now.
- The `readBusPacket()`-vs-`ESP32UARTChannel` question turned out to be a non-issue, not a porting
  task: `lib/hardware/UARTChannel.h` `using`-aliases `UARTChannel` straight to `ESP32UARTChannel`
  for every ESP32 target, unconditionally. `fujiversal` and Centipede both use the literal same
  class for `_port` on real UART hardware (as opposed to BoIP) - there was never a second code path
  to verify.
- `lib/device/drivewire/disk.cpp` - `push_rom_image()`'s body widened the same way as above, but
  its *call site* inside `mount()`'s `MEDIATYPE_ROM` case stays `PINMAP_FUJIVERSAL_DRIVEWIRE`-only.
  Widening that call site too was the obvious-looking trap: `push_rom_image()` now compiling for
  `COCO_HS_UART` means `mount()` calling it unconditionally would auto-push from *any* mount
  trigger on Centipede, including the button and HTTP ones the design decision above explicitly
  ruled unsafe. Also added `drivewireDisk::push_rom()`, a `success_is_true`-returning helper used
  by the new on-demand call site below - keeps `_media` private rather than exposing a getter
  (this codebase's reviewers have flagged that exact pattern before).
- `lib/device/drivewire/drivewireFuji.cpp` / `.h` - the actual trigger restriction. New
  `COCO_HS_UART`-gated overrides of `fujicmd_mount_disk_image_success()` and
  `fujicmd_mount_all_success()` (both already `virtual` in the shared `fujiDevice` base, so this
  needed no change there): call the base mount logic first, then push any newly-mounted ROM slot
  via `push_rom()`, before sending `transaction_success()` - so the CoCo, which these two calls
  leave blocked waiting for that exact reply, can't ask the companion to activate a cartridge the
  push hasn't actually delivered yet. The front-panel button (`lib/hardware/keys.cpp`) and the web
  config app (`lib/http/httpService*.cpp`) call `fujicore_mount_*` directly, one layer below this
  override, so they're excluded automatically - no flag, no parameter threading through the shared
  base.
- Confirmed rather than assumed: the mount commands (`CMD::FUJI_MOUNT_IMAGE`/`FUJI_MOUNT_ALL`) have
  always worked for Centipede - nothing HDBDOS/toolshed-side needed to change to trigger a mount.

Verified building (`./build.sh -cb`) for `fujinet-coco-devkitc-centipede` (full build+link),
`fujinet-coco-devkitc` (full build+link, confirms the plain board is unaffected), and
`fujiversal-drivewire` (the three touched drivewire files compile clean; the full link is blocked
by an unrelated, pre-existing vendored-component bug - reproduced identically on unmodified
`upstream/master`, not something these changes caused). Not yet flashed/tested on real Centipede
hardware against the One ROM side above.

## First end-to-end test (2026-10-09)

Both sides flashed together for the first time: `centi-fuji`'s `centipede-rom-autopush` branch
(`a8e97396f`) on the ESP32, this plugin (working tree, uncommitted) on a Fire 28 C board.

### Bug #1 (found and fixed): ordinary DriveWire reads were corrupting, 100% reproducible

Symptom: every `OP_READEX` sector read failed its checksum check on the FujiNet/ESP32 side
(`Checksum error: expected N, got 65535` - `0xFFFF`, the sentinel `_port->read()` returns on a
timeout, not a value mismatch), on literally the first read of every boot. Severe enough that CONFIG
couldn't finish loading, so the ROM mount itself was never reached. Bisected on the `centi-fuji`
side down to a build of plain `upstream/master` with zero ROM-push code - reproduced identically -
which proved the regression was entirely in this plugin, unrelated to anything the push feature
does once a push actually starts.

Root cause: `drivewire_do_write()` called the new `drivewire_check_for_pushed_cart()` *before*
restoring the real ROM content at `DW_KNOCK_ACK_ADDR`, instead of after. That peek can block for
the full `DW_PUSH_PEEK_TIMEOUT` budget whenever nothing is arriving yet (the common case - FujiNet
hasn't replied), and the CoCo resumes executing from this same ROM image the instant `DWWrite`
returns, so any fetch landing on `DW_KNOCK_ACK_ADDR` during that window saw the stub ack byte
(`0xA5`) instead of real content. Deterministic, not a race, because it happens on every single
write session, including the very first one of a boot.

Fix applied directly in this working tree (uncommitted - this session should review/commit it):
swap the two statements in `drivewire_do_write()` so the ack-address restore happens immediately
when the data phase ends, and the push-check peek runs after. The peek only touches UART1, so it
has no actual ordering dependency on the ROM-bus restore - moving it doesn't change anything about
how pushes are detected.

### Bug #2 (found, now fixed): the push itself never arrived when triggered by a mount command

With bug #1 fixed, CONFIG loads cleanly and `FUJI_MOUNT_IMAGE` reaches the ESP32. But the push
still fails every time: `push_rom_image()` sends the DBC `NET_OPEN` frame and times out waiting for
an ACK/NAK (logged on the FujiNet side as `Failed to open pico bank`). The companion never
acknowledges because it never recognizes the push as having started at all.

Timing, from the test log:

```
12:50:00.593 > Fuji cmd: MOUNT IMAGE          <- CoCo's mount command reaches the ESP32
12:50:01.055 > fopen = ...Bustout...ccc rb : ok   <- ~460ms later: file open completes
12:50:01.059 > DW ROM MOUNT ... (4096 bytes)
12:50:01.569 > push_rom_image(): Failed to open pico bank   <- NET_OPEN never ACKed
```

`drivewire_check_for_pushed_cart()` is only called once, from the tail of `drivewire_do_write()`,
right after the *one write session that carried the mount command's own parameters* - and it only
peeks for `DW_PUSH_PEEK_TIMEOUT` (tens of microseconds). That peek's own comment is explicit that
it's sized for "realistic wire turnaround, not FujiNet-side processing time." For Centipede's
mount-triggered push specifically, there's a `fujiDevice::mount()` call (which does the `fopen`
above) sitting between that write session and the point where `push_rom()` actually starts
transmitting - ~460ms in this test, SD-card/filesystem-dependent in general. The peek window is
long gone by the time the push's leading `DBC_SLIP_END` byte actually arrives.

Nothing else on this side recognizes that byte as a push marker. `drivewire_next_addr()`'s own
opportunistic drain *is* still running throughout that whole gap (it runs on every call, including
all of `drivewire_wait_for_knock()`'s polling for the CoCo's next knock) - but it has no
`DBC_SLIP_END` check at all; it just stashes whatever arrives into `s_read_buf` as if it were
ordinary reply data. So the push's opening marker gets silently absorbed as garbage, and
`drivewire_dbc_receive_cart()` is never entered.

This is a real mismatch between the two sides' designs, not a simple typo: fujiversal's original
push (no shared wire, no CoCo-synchronous command in front of it) can assume "a push starts
essentially immediately after the write that precedes it." Centipede's push is anchored behind a
mount command that needs a slow, variable-length file open first, so that assumption doesn't hold
for it.

### Bug #2 fix (2026-10-09, one-rom session)

Not the `drivewire_next_addr()` direction floated above - that function's own opportunistic drain
turned out not to be where this needed to live at all. Tracing the actual timeline: the CoCo issues
its own `!DWRECV!` knock for the mount's reply within microseconds of the triggering write (normal
request-then-read client behaviour), so by the time a slow push actually starts (450ms+ later),
`drivewire_wait_for_knock()`'s scanning phase is long over and the plugin is already deep inside
**`drivewire_do_read()`'s own direct `drivewire_uart_getc()` loop** - a separate code path from
`drivewire_next_addr()`'s opportunistic drain entirely, with zero push-awareness of its own. That's
where the push's bytes were actually landing, silently treated as literal reply data.

Fix: moved detection into `drivewire_do_read()` itself - before accepting this session's own first
real reply byte, a retry loop watches for (and fully consumes, including replaying a false alarm's
raw bytes verbatim - see below) a push, looping until a byte is confirmed real. This introduces no
new stall: `drivewire_do_read()` already blocks on UART1 for up to `DW_UART_GETC_TIMEOUT` per byte
regardless, and the ROM-bus ring buffer was never being drained during that wait either way, so
there's nothing to protect that wasn't already unprotected. The old write-tail peek
(`drivewire_check_for_pushed_cart()`, `DW_PUSH_PEEK_TIMEOUT`) is gone entirely, which also makes
bug #1's reorder fix moot - `drivewire_do_write()` is back to its pre-feature form, nothing to
reorder around.

Also hardened while here: a lone `DBC_SLIP_END` was never enough to prove a push is starting (any
byte can coincide with real reply data) - the fix validates the whole 6-byte header
(device/command/descr/length) via `dbc_decode_header_probe()` before committing, replaying the raw
bytes verbatim into `s_read_buf` on a false alarm so nothing is lost.

### Bug #3 (found and fixed): mount-triggered push can start mid-write, not just mid-read

Re-testing against `fuji_mount_disk_image()`'s actual CoCo-side client code
(`~/Workspace/fujinet-lib`) found the fix above was still incomplete: `fuji_get_error()` - called
right after the mount write, *before* ever reading its reply - does its own separate
`bus_ready()`/`dwwrite()`/`dwread()` round trip first. So the push's leading `DBC_SLIP_END` can
legally arrive while this plugin is mid-relay of a *different*, unrelated write, which
`drivewire_do_read()`'s own check (being read-only) can't see.

Fix: detection moved into `drivewire_next_addr()`'s opportunistic drain itself
(`drivewire_push_probe_feed()`), covering both `drivewire_wait_for_knock()`'s knock-scan and
`drivewire_do_write()`'s relay loop, since both call it. Being inside this hot, never-allowed-to-block
function, this is an incremental state machine (one byte per call) rather than the earlier
blocking probe, with a confirmed push signalled via a flag (`s_cart_push_confirmed`) that each
caller checks and acts on wherever safe to pause for each.

### Bug #4 (found and fixed): false positives on ordinary disk traffic, not just rare theoretical ones

With detection running on every byte everywhere, real-hardware testing on a session that **never
mounted a ROM at all** (button-triggered disk rotate, `.dsk` mount, ordinary sector reads) produced
repeated FujiNet-side checksum timeouts (`got 65535`). `dbc_header_looks_like_push()`'s combined
false-positive probability against *truly random* bytes is astronomically low, but disk sector data
isn't random - it's a compiled program and filesystem structure - and a false positive is not
cheap: it blocks real DriveWire traffic for seconds while this plugin waits on a push that was
never coming, desyncing the real exchange.

Fix (your call, not mine - flagging for your own future reference): narrowed detection to run only
while `drivewire_wait_for_knock()` is actively scanning between sessions (`s_push_probe_enabled`),
never during `drivewire_do_write()`'s relay or `drivewire_do_read()`'s own collection. This is a
known, accepted gap: a push that starts arriving while either of those is already running will be
missed. Not yet revisited with a more targeted alternative (e.g. arming detection specifically
after recognizing a ROM-mount-triggering write, rather than scoping by session phase) - may be
worth revisiting if Bug #3's scenario (push starting mid an unrelated small write) turns out to
matter in practice once this is retested end to end.

Also found and fixed in the same pass: `dbc_decode_header_probe()`/`drivewire_push_probe_feed()`'s
raw-byte replay buffer (sized for "6 header bytes, worst case all SLIP-escaped") had no bound on a
malformed SLIP escape sequence (dropped leniently, matching FujiNet's own `decodeSLIP()`, but with
no bound on how many more bytes might arrive before the header finished) - ordinary binary sector
data triggers this often enough to silently truncate a false alarm's replay, corrupting real data.
Fixed by treating a malformed escape as immediate proof a probe isn't a real push, bounding raw
consumption to the buffer's actual capacity always.

### Bug #5 (found, fully disabled push detection as the immediate fix): narrowing to knock-scan didn't actually narrow the exposure

Bug #4's fix was based on a wrong premise. `drivewire_next_addr()`'s opportunistic drain runs
during `drivewire_wait_for_knock()`'s own scan *specifically because* that's where a fast-arriving
reply gets caught before its own read knock is even sent - which is the **common** case for
ordinary sector reads, not a rare one. So leaving detection enabled there didn't reduce the
false-positive exposure at all; it just removed the two less-risky detection points
(`drivewire_do_write()`'s relay, `drivewire_do_read()`'s entry) and left the one that actually sees
the bulk of every ordinary reply. Confirmed on real hardware: a false positive now blocks
`drivewire_wait_for_knock()` itself for seconds, which is the one function responsible for ever
recognizing the next knock at all - result, no DriveWire traffic reached the ESP32, not even the
first write of a session.

Immediate fix at the time: `s_push_probe_enabled` set permanently `false`, restoring plain
DriveWire with the detection machinery present but fully dormant. Superseded by content-based
arming below.

### Content-based arming (2026-10-09, one-rom session): the actual fix

Phase-scoping (on during one function, off during another) can never separate "might be a push"
from "definitely ordinary disk data," because ordinary reply data and a push both arrive through
the exact same mechanism regardless of which function happens to be running. Only the *content* of
what triggered the write can make that distinction.

`drivewire_do_write()` now arms detection (`s_push_probe_enabled = true`, plus a budget countdown -
`s_push_probe_arm_budget`) only on recognizing a write whose first two relayed bytes are
`OP_FUJI`/`FUJICMD_MOUNT_IMAGE` or `OP_FUJI`/`FUJICMD_MOUNT_ALL` (`0xE2` + `0xF8` or `0xD7` - traced
to `fuji_mount_disk_image()`/`fuji_mount_all()` in `fujinet-lib`) - the only writes that can ever
cause FujiNet to push a ROM. Ordinary disk I/O (`OP_READEX` and friends) never arms it, so the
false-positive exposure that broke both earlier designs doesn't apply to the vast majority of
traffic at all. Once armed, detection runs in all three places a push could legally start arriving
(see the "DBC (FujiBusPacket)" section header comment for the full breakdown):
`drivewire_push_probe_feed()` (fed from `drivewire_next_addr()`'s drain) for
`drivewire_wait_for_knock()`'s scan and `drivewire_do_write()`'s own relay loop, plus
`drivewire_resolve_pushed_cart()` (restored) for `drivewire_do_read()`'s own
`drivewire_uart_getc()` loop, which bypasses `drivewire_next_addr()` entirely.

`DW_PUSH_ARM_BUDGET` (currently `100000000` `drivewire_next_addr()` iterations, generous relative
to the ~1 second mount+push latency observed, but a starting value only - needs tuning against real
hardware) bounds how long an arm window stays open with nothing confirmed; it's not every
`FUJICMD_MOUNT_IMAGE` that actually pushes (an ordinary `.dsk` mount uses the identical opcode
prefix), so most arm windows are expected to simply expire unused, which is the intended, cheap
behaviour. Extended back to the full budget on every successful receive, since
`FUJICMD_MOUNT_ALL` can push more than one slot in sequence.

### Bug #6 (found and fixed): even armed, a coincidental header match was unrecoverable past the gate

Content-based arming alone wasn't sufficient. Real-hardware testing (a fresh boot, no explicit ROM
mount at all) hung mid-config-load with **no checksum error logged at all** - different from every
earlier symptom, and the key clue: `FUJICMD_MOUNT_ALL` fires on every boot restoring whatever disks
were previously configured, not just ROM ones, and is always immediately followed by a burst of
ordinary sector reads - exactly the trigger an arm window needs to overlap with heavy disk traffic
even though no push was ever coming. Isolated by temporarily force-disabling the arm trigger: 4
boots, 0 anomalies, confirming the bug was in the detection path despite no `Fuji cmd: MOUNT`
appearing anywhere in the visible log (a `[Errno 5]`/reconnect at the very start of that capture
means the actual trigger was most likely just off-screen).

The deeper issue: unlike a failed *header* probe (which stays recoverable - raw bytes replayed
verbatim into `s_read_buf`), a header that passes `dbc_header_looks_like_push()` but then fails
full validation (its checksum, checkable only once the whole claimed payload has arrived) has
**no recovery path** - those bytes are already consumed off the UART stream with nothing to put
back, and the real reply is gone. Since `hdbdos`'s own `DWRead` has no CoCo-side timeout, the CoCo
then hangs indefinitely waiting for a reply that will never arrive, with no error ever surfacing
anywhere. The original gate's `NET_OPEN|NET_WRITE|NET_CLOSE`-with-up-to-256-byte-length check was
tight against *random* bytes but not against *structured* (non-random) disk image content - and
unlike bytes that only matter in the moment (Bug #4), this gate is the one thing standing between
"never commit" and "just lost the real reply."

Fixed by tightening `dbc_header_looks_like_push()` rather than trying to afford a large enough
replay buffer for the full payload (256+ bytes, likely not affordable in this plugin's RAM budget):
it now requires `command == DBC_CMD_NET_OPEN` specifically (not any of three commands) with a
narrow claimed length (`DBC_FIRST_FRAME_MAX_PAYLOAD`, header + up to 10 bytes, not up to a full
256-byte block) - justified because this gate only ever sees a push's *first* frame (always
`NET_OPEN`, always a short fixed 5-byte payload - subsequent frames go through `dbc_recv_frame()`
instead, which never needs this check at all since a push is already confirmed by then). Both
`FUJICMD_MOUNT_IMAGE` and `FUJICMD_MOUNT_ALL` stay as arm triggers (both are genuine,
ESP32-confirmed push sources - excluding either would trade a false positive for a missed real
push); the fix narrows what counts as a convincing header instead.

Built and flashed to the Fire 28 C (compiles clean, `-Wall -Wextra -Werror`).

### Bug #7 (found and fixed): NET_OPEN's wire format was wrong from the start - this plugin side, and its own comments, both

Bug #6's fix still didn't let a real push through - `push_rom_image(): Failed to open pico bank`
every time, identical to the very first test of this whole feature. Every prior round assumed
`NET_OPEN`'s payload was "stream id (1 byte) + 4-byte little-endian size," descr `0`. Traced
`sendCommand(DBC, NET_OPEN, (uint16_t)0)` through `FujiBusPacket`'s actual constructor and
`serialize()` (`centi-fuji`'s `lib/bus/rs232/FujiBusPacket.cpp`) to check: that call passes a
single *packed* 2-byte parameter, not a raw data payload - `processArg(std::uint16_t)` pushes a
`PacketParam`, and `serialize()`'s packed-param path computes a header `descr` of `0x05` (one
field, `FUJI_DESCR_EXCEEDS_U8` for a field wider than a byte) with total on-wire length
`DBC_HEADER_LEN + 2` - never `0` and never the length this plugin was checking for. Worse,
**`NET_OPEN` carries no image size at all** - just the literal value 0, with no size field
anywhere in the frame. Likely traced originally from `push_stream()`, a function that turns out to
belong to a completely different, unrelated device (`lib/device/rs232/disk.cpp`), not the actual
`push_rom_image()` in `lib/device/drivewire/disk.cpp` this plugin talks to - so this assumption was
never right, on either side of the two sessions working on this, for as long as this feature has
existed. It's why every single push attempt this entire session - from the very first FUJI_PULL_ROM
test through every later one - failed identically at `NET_OPEN`.

Fixed on both ends of the gate: `dbc_header_looks_like_push()` now requires the *exact* verified
shape (`descr == 0x05`, `claimed_len == DBC_HEADER_LEN + 2`) rather than the wrong `descr == 0`
with a length range - incidentally also a much tighter false-positive gate than Bug #6's fix, since
it's now an exact match on two fields instead of a range on one. `drivewire_dbc_receive_cart()`'s
`NET_OPEN` handling no longer tries to read a nonexistent size field; since no size is ever known
up front, the slot-capacity check moved to `NET_WRITE`, which now bounds the running offset against
`slot_size` explicitly and NAKs/aborts (rather than silently truncating) if a push ever exceeds the
slot. Stale comments pointing at `push_stream()` as the reference implementation, in both this
file's own history above and in the plugin's source, corrected to `push_rom_image()`.

Built and flashed to the Fire 28 C (compiles clean, `-Wall -Wextra -Werror`).

### End-to-end success (2026-10-10)

Bug #7's fix was the one that made it work. Re-tested with the user's own CoCo-side config app
(a from-scratch replacement for the stock one, which does not speak `!LDCART!` at all - see Bug #8
below): `FUJICMD_MOUNT_IMAGE` triggers the ESP32's push, `push_rom_image()` reports
`ROM transfer complete: 4096 / 4096 bytes`, and the config app's own `!LDCART!` knock switches the
CoCo over to the pushed cartridge cleanly. Ordinary DriveWire traffic unaffected throughout. Plugin
version bumped to 1.1.0 (`plugins/user/drivewire/Makefile`, `CHANGELOG.md`), still uncommitted at
the time of writing - the user's own call, per this repo's convention (GPG-signed commits, their
key).

### Bug #8 (found and fixed by the `centi-fuji` session; then found to cause a regression, reverted - 2026-10-13)

A separate agent working in the `centi-fuji` session identified a real design gap in this plugin
and patched it directly: `drivewire_wait_for_knock()` was acking *both* `match_write` and
`match_cart` the instant either knock pattern was recognized, using the identical value at the
identical address (`DW_KNOCK_ACK_ADDR` / `DW_KNOCK_ACK_VALUE`) for "knock landed" as
`drivewire_do_load_cart()` later uses for "cartridge actually activated." A `!LDCART!` caller
polling that address can't tell the two apart - if it jumped to the cartridge the moment it saw
"knock landed," it would land back in the still-served old ROM, since the slot switch hadn't
happened yet. The fix narrowed the early ack to `match_write` only, so `!LDCART!` gets no ack at
all until `drivewire_do_load_cart()` sets the real one - removing the ambiguity structurally rather
than relying on timing.

Well-reasoned, and independently confirmed plausible by static reading in the one-rom session too.
Flashed to the Fire 28 C, and real-hardware testing immediately turned up a new, different
regression: checksum errors and a new `op_unhandled(): Unhandled opcode` symptom (write-direction
corruption) on *ordinary* traffic, during nothing more than a config-app boot's own `autorun.dsk`
handling - no cartridge mount involved at all. Several hypotheses were checked and ruled out
(confirmed `autorun.dsk`'s mount is purely ESP32-internal/LittleFS and never crosses the
DriveWire/UART1 wire, so it can't be arming or tripping anything here; confirmed the changed code
path is a no-op whenever `match_cart` is false, which it always is for this traffic). With the
mechanism still unexplained by reasoning alone, a direct A/B test on real hardware (reverting just
this one change, rebuilding, reflashing) was decisive: reverted, the corruption is gone (4 boots, 0
problems); with the fix in, it reproduces. The fix's own underlying premise is also now suspect -
the original "push succeeded but didn't switch" symptom that motivated it turned out, once properly
isolated, to have a different, already-confirmed explanation (a config app that never sent
`!LDCART!` at all, not a race) - so it may have been solving a problem that wasn't real while
introducing one that is.

Reverted permanently in this plugin's working tree (`drivewire_wait_for_knock()` and
`drivewire_do_load_cart()` both back to acking/clearing on `match_cart` too, with comments
explaining why - see those two functions). The exact mechanism by which the `centi-fuji` session's
change caused this is still **not understood** - this needs attention from whoever picks this back
up, ideally with scope to actually trace it (e.g. a logic analyzer on UART1, or bisecting further
within that one change) rather than take the A/B result as the full story. Until then, treat
`match_write`-only acking for `!LDCART!` as unsafe on this plugin, confirmed on real hardware,
mechanism unknown.

### Status

- `centi-fuji`/`centipede-rom-autopush`: unchanged since "done" above - the push-sending side was
  never the problem; this plugin's own understanding of what it sends was.
- This plugin: end-to-end push-and-activate confirmed working on real hardware (see "End-to-end
  success" above). Bug #8's `centi-fuji`-side fix is reverted in this working tree pending its
  mechanism being understood - see Bug #8. Still uncommitted.

### What's left

1. Understand Bug #8's actual mechanism (why removing the `match_cart` early ack corrupts ordinary
   traffic) before trying that fix again - the current revert is a known-safe state, not a resolved
   one, and it does leave the original knock-ack ambiguity (that fix's target) back in place.
2. Tune `DW_PUSH_ARM_BUDGET` against real hardware if needed - it's a cycle-count guess, not a
   measured value.
3. Review and commit this plugin's changes (still uncommitted).
4. Propose the `centi-fuji` side upstream (new PR against `FujiNetWIFI/fujinet-firmware`, replacing
   the closed #1630) once hardware testing confirms the whole thing works.
