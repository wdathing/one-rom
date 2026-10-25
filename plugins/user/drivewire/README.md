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
`SEL_C`/`SEL_D` image-select pads. **The `SEL_C`/`SEL_D` jumper must be left
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
  --slot file=/path/to/hdbonerom.rom,type=27128,size-handling=duplicate
```

## Wire protocol

Signalling rides the ROM address bus, the same idiom the host-control
(RBCP) plugin uses to let the CPU "sneak" data out over a bus it can
otherwise only read from - see that plugin's README for the general model.
Three knock sequences (8 bytes each, matched against the observed address's
low byte) pick the session:

- `!DWSEND!` - write session (CoCo -> server). The plugin acknowledges the
  knock at `$C200`, which the CoCo polls, resending the knock if it sees
  nothing. Two count bytes follow (high byte first, 0-65535), then that many
  data bytes, each forwarded to UART1 TX.
- `!DWRECV!` - read session (server -> CoCo). Two count bytes follow. The
  plugin then hands over each reply byte with a toggle-and-acknowledge
  handshake, described below.
- `!LDCART!` - activates a cartridge image FujiNet has already pushed (see
  the CHANGELOG for 1.1.0).

Each count or data byte is sent as a read of `$C100` plus the byte as a
signed offset (the 6809's `LDA B,U`), so the address's low byte is the
value, and the address itself falls anywhere in `$C080`-`$C17F`.

In a read session, for each byte the plugin writes it to the data address
`$C101`, then sets the status address `$C100` to that byte's toggle: `$02`
for the first byte, then alternating `$03`, `$02`... The CoCo waits for the
toggle, reads the data, and acknowledges by reading `$C100` plus the toggle
(`$C102` or `$C103`). The plugin publishes the next byte only once it has
seen that. Waiting for a value that changes, rather than for a ready flag,
means a fast CoCo - a CoCo 3 at double speed - can't collect the same byte
twice. While polling, the CoCo also re-reads the previous byte's
acknowledgement address, so an acknowledgement the plugin missed costs one
more poll rather than a hang.

The plugin ends a session early, and goes back to waiting for a knock, when
the CoCo stops reading or reads an address outside the session's own set:
it crashed, was reset, or gave up. HDB-DOS's `DWRead` gives up after about
1.4 seconds without a byte, as the stock bit-banger version does.

The handshake must match HDB-DOS's ONEROM build: this plugin needs the
`dwoneread.asm` with the same toggle protocol, and the two ship together.

The status, data and acknowledgement addresses are ordinary offsets into the
HDB-DOS image - live code, not spare padding. Overwriting them is safe and
fully reversible: the plugin reads and caches the real bytes once at
startup, and restores them the moment each read session ends, so outside an
active session the served image is exactly what it was assembled as. The
knock sequences and acknowledgements are pure reads and never touch served
content.

## Future: multi-ROM CoCo replacement

The initial target is the 8K DOS ROM in a FujiNet-style disk controller
card. A CoCo motherboard replacement covering BASIC/Extended Color
BASIC/DOS across multiple chip-select lines is a possible future direction,
but is out of scope for this version - the status/data channel addresses
and knock sequences here are not yet parameterised per chip-select.
