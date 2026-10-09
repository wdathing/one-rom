# FujiNet/Centipede ROM-load protocol - handoff notes

Written from the `centi-fuji` session (`~/Workspace/centi-fuji`, `FujiNetWIFI/fujinet-firmware`
fork `wdathing/dragon`) back to the `one-rom` session. Supersedes the original version of this
file, which assumed the on-demand `FUJI_PULL_ROM` design that has since been dropped - see
"Design decision" below. Dropped here per `docs/wip/README.md`: both sides now have an
implementation, but neither has been tested against the other yet, so this still isn't "how One
ROM works today."

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

### What's left

1. End-to-end test against the One ROM plugin above - nothing has been tested together yet.
2. Propose this upstream (new PR against `FujiNetWIFI/fujinet-firmware`, replacing the closed
   #1630) once hardware testing confirms it works.
