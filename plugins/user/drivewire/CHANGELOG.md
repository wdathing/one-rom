# Changelog

## [1.2.0] - 2026-10-25

- Supports the CoCo 3, including at double speed. Reads use a new toggle-and-acknowledge handshake, which needs the matching HDB-DOS ONEROM build.
- Recovers when the CoCo crashes, is reset or gives up partway through a transfer, instead of needing One ROM power cycled.

## [1.1.0] - 2026-10-23

The plugin now receives a cartridge image FujiNet pushes automatically when a ROM is
mounted, rather than requesting one. `!LDCART!` now activates an already-pushed image
instead of triggering the pull - a breaking change for any CoCo-side client expecting
the old request-based behaviour.
- This required a firmware update.

## [1.0.0] - 2026-08-16

Initial version.  Bridges a DriveWire session between a CoCo/Dragon disk
controller ROM and a real DriveWire server, over One ROM's RP2350 UART1
(GPIO40/41) instead of the CoCo's own bit-banger serial port.  Targets the
One ROM Fire 28 Rev C, whose SEL_C/SEL_D image-select pads sit on those
GPIOs and must be left unpopulated for this plugin to use them.

Companion hdbdos changes (`dwonewrite.asm`/`dwoneread.asm`, the `ONEROM`
build variant) live in the separate hdbdos repository, not here.
