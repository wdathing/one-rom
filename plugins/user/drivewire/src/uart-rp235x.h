// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License
//
// RP2350 registers this plugin needs that One ROM's own reg-rp235x.h does
// not define: UART1 and the clock, reset and pad bits that bring it up.

#ifndef UART_RP235X_H
#define UART_RP235X_H

#define UART0_BASE          0x40070000
#define UART1_BASE          0x40078000

#define CLOCK_PERI_CTRL         (*((volatile uint32_t *)(CLOCKS_BASE + 0x48)))
// clk_peri has no glitchless SRC mux, only AUXSRC - CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS
// (0x0) is its reset value, so enabling with no further write already selects
// clk_sys.  Its integer divider (CLOCKS_CLK_PERI_DIV, offset 0x4C) resets to
// /1 and is left untouched.
#define CLOCK_PERI_CTRL_ENABLE    (1 << 11)

// RESETS is shared chip-wide (both cores, core firmware, any plugin) - use
// the atomic SET/CLR aliases to change one bit, never a plain read-modify-
// write on RESET_RESET, which can race with something else's concurrent
// read-modify-write on the same register and clobber an unrelated bit.
#define RESET_RESET_SET (*((volatile uint32_t *)(RESETS_BASE + 0x00 + 0x2000)))
#define RESET_RESET_CLR (*((volatile uint32_t *)(RESETS_BASE + 0x00 + 0x3000)))
#define RESET_UART0         (1 << 26)
#define RESET_UART1         (1 << 27)

#define GPIO_CTRL_FUNC_UART     0x02

// Set at power-up to electrically isolate the pad from the chip's internal
// logic (part of the glitch-free power-up sequence); software must clear it
// once the pad's function select, pulls etc are configured, or the pad
// never actually drives/senses the pin regardless of what the peripheral
// behind it does.
#define PAD_ISO             (1 << PAD_ISO_BIT)

// UART Registers (PL011) - offsets are identical for UART0_BASE/UART1_BASE
#define UART_DR_OFFSET       0x00
#define UART_RSR_OFFSET      0x04  // read: receive status; write: error clear
#define UART_FR_OFFSET       0x18
#define UART_IBRD_OFFSET     0x24
#define UART_FBRD_OFFSET     0x28
#define UART_LCR_H_OFFSET    0x2C
#define UART_CR_OFFSET       0x30

#define UART_REG(base, off)  (*((volatile uint32_t *)((base) + (off))))

#define UART_FR_RXFE_BIT     4
#define UART_FR_TXFF_BIT     5
#define UART_FR_RXFE         (1 << UART_FR_RXFE_BIT)
#define UART_FR_TXFF         (1 << UART_FR_TXFF_BIT)
#define UART_RSR_FE_BIT      0  // framing error
#define UART_RSR_PE_BIT      1  // parity error
#define UART_RSR_BE_BIT      2  // break error
#define UART_RSR_OE_BIT      3  // overrun error
#define UART_RSR_FE          (1 << UART_RSR_FE_BIT)
#define UART_RSR_PE          (1 << UART_RSR_PE_BIT)
#define UART_RSR_BE          (1 << UART_RSR_BE_BIT)
#define UART_RSR_OE          (1 << UART_RSR_OE_BIT)

#define UART_LCR_H_FEN_BIT   4
#define UART_LCR_H_FEN       (1 << UART_LCR_H_FEN_BIT)
#define UART_LCR_H_WLEN_LSB  5
#define UART_LCR_H_WLEN_8    (0b11 << UART_LCR_H_WLEN_LSB)

#define UART_CR_UARTEN_BIT   0
#define UART_CR_TXE_BIT      8
#define UART_CR_RXE_BIT      9
#define UART_CR_UARTEN       (1 << UART_CR_UARTEN_BIT)
#define UART_CR_TXE          (1 << UART_CR_TXE_BIT)
#define UART_CR_RXE          (1 << UART_CR_RXE_BIT)

#endif // UART_RP235X_H
