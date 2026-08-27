# Changelog

## [0.1.0] - 2026-08-16

Initial version.  Bridges a DriveWire session between a CoCo/Dragon disk
controller ROM and a real DriveWire server, over One ROM's RP2350 UART1
(GPIO40/41) instead of the CoCo's own bit-banger serial port.  Targets the
One ROM Fire 28 Rev C, whose SEL_A/SEL_B image-select pads sit on those
GPIOs and must be left unpopulated for this plugin to use them.

Companion hdbdos changes (`dwonewrite.asm`/`dwoneread.asm`, the `ONEROM`
build variant) live in the separate hdbdos repository, not here.
