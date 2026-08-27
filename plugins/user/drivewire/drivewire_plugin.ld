/*
 * Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
 *
 * MIT License
 */

USER_PLUGIN_BASE = 0x10020000;
USER_PLUGIN_SIZE = 64K;
TOP_OF_SRAM = 0x20082000;

USER_PLUGIN_DATA_BASE = 0x20081C00;
/* Raised from the template's 0x20081E00 (512 bytes) - the 256-byte ring
 * buffer plus drivewire_do_read()'s own 256-byte response buffer already
 * exceed that alone.  0x20081F00 leaves 256 bytes for the stack, well above
 * drivewire_main()'s own measured ~80-byte frame plus the small core-firmware
 * helper functions it calls into. */
USER_PLUGIN_DATA_LIMIT = 0x20081F00;
USER_PLUGIN_STACK_TOP = 0x20082000;   /* 1KB shared between stack and static RAM */

PLUGIN_BASE = USER_PLUGIN_BASE;
DATA_BASE = USER_PLUGIN_DATA_BASE;
DATA_LIMIT = USER_PLUGIN_DATA_LIMIT;
STACK_TOP = USER_PLUGIN_STACK_TOP;

SECTIONS
{
    . = PLUGIN_BASE;

    .plugin_header : {
        KEEP(*(.plugin_header))
        . = ALIGN(4);
    }

    ASSERT(SIZEOF(.plugin_header) > 0, "ERROR: .plugin_header section is empty")

    .text : {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        __text_end = .;
    }

    /* Placed first in the data region so its 256-byte (64-entry * 4 bytes)
     * alignment requirement, per ora_setup_address_monitor_fn_t, is met by
     * DATA_BASE's own alignment rather than relying on the linker to find
     * room for it further in. */
    .ring_buf DATA_BASE (NOLOAD) : {
        *(.ring_buf)
        . = ALIGN(4);
    }

    .ramfunc : AT(__text_end) {
        __ramfunc_start = .;
        *(.ramfunc*)
        . = ALIGN(4);
        __ramfunc_end = .;
    }
    __ramfunc_load = LOADADDR(.ramfunc);

    .data (__ramfunc_end) : AT(__text_end + SIZEOF(.ramfunc)) {
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    }
    __data_load = LOADADDR(.data);

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        . = ALIGN(4);
        __bss_end = .;
    }
    ASSERT(. <= DATA_LIMIT, "ERROR: .ramfunc + .data + .bss exceed DATA_LIMIT")

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}
