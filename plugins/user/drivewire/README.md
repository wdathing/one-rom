# DriveWire Bridge

Bridges a DriveWire session between a TRS-80 Color Computer (or Dragon
32/64) disk controller ROM and a real DriveWire server, over One ROM's
RP2350 UART1 hardware, instead of the CoCo's own bit-banger serial port.
UART1 runs at 921600 baud, well beyond what the CoCo's bit-banger port can
reach, so the ROM-bus link to the CoCo is the bottleneck rather than the
link to the server.

This plugin is one half of the system: the other half is a modified hdbdos
build (`dwonewrite.asm`/`dwoneread.asm`, the `ONEROM` variant) in the
separate hdbdos source tree, which replaces hdbdos's `DWWrite`/`DWRead`
with routines that speak this plugin's protocol instead of driving a serial
port directly.

## Hardware prerequisite

This plugin targets the **One ROM Fire 28 Rev C**, the only 28-pin variant
with GPIO40/41 (RP2354B/QFN-80). Those two GPIOs are also this board's
`SEL_A`/`SEL_B` image-select pads. **The `SEL_A`/`SEL_B` jumper must be left
unpopulated** for the plugin to use them as UART1 - the firmware's own boot
sequence stops driving pulls on them once image selection is read, but if
the jumper itself is still soldered/populated it will electrically contend
with the UART signal.

GPIO40/41 are 3.3V-only (not 5V-tolerant) ADC-capable pins. Whatever is
wired to them - the DriveWire server bridge - must be 3.3V logic, never 5V
TTL.

## Building the Plugin

```bash
make
```

This creates `build/drivewire_plugin.bin`, loaded onto One ROM as a user
plugin. A system plugin is also required (the `usb` system plugin is fine -
this plugin does not use it):

```bash
onerom program --plugin usb --plugin file=drivewire/build/drivewire_plugin.bin \
  --slot file=/path/to/hdbonerom.rom,type=2764,cs1=active-low
```

## Wire protocol

Signalling rides the ROM address bus, the same idiom the host-control
(RBCP) plugin uses to let the CPU "sneak" data out over a bus it can
otherwise only read from - see that plugin's README for the general model.
Two knock sequences (8 bytes each, matched against the observed address's
low byte) pick the session direction:

- `!DWSEND!` - write session (CoCo -> server). Followed by a count byte (0
  means 256), then that many further address-encoded reads, each forwarded
  to UART1 TX.
- `!DWRECV!` - read session (server -> CoCo). Followed by a count byte.
  For each byte: the plugin blocks on UART1 RX, writes the byte to a fixed
  logical "data" address, then flips a fixed logical "status" address to a
  ready sentinel (`0xFF`). The CoCo spin-reads status until ready, then
  reads data to collect the byte. The plugin does not prepare the next byte
  until it observes the CoCo's read of the data address.

The status/data addresses are ordinary ROM offsets (`$C100`/`$C101` in CPU
space for the 8K 2764 this plugin currently targets) - live hdbdos code, not
spare padding. Overwriting them is safe and fully reversible: the plugin
reads and caches the real bytes there once at startup, and restores them
the moment each read session ends, so outside an active session the served
image is exactly what it was assembled as. The knock sequences themselves
are pure reads and never touch served content.

## Future: multi-ROM CoCo replacement

The initial target is the 8K DOS ROM in a FujiNet-style disk controller
card. A CoCo motherboard replacement covering BASIC/Extended Color
BASIC/DOS across multiple chip-select lines is a possible future direction,
but is out of scope for this version - the status/data channel addresses
and knock sequences here are not yet parameterised per chip-select.
