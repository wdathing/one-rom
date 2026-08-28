// One ROM - Vector table and reset handler.

// Copyright (C) 2025 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#if !defined(TEST_BUILD)

#include "include.h"
#include "reg-rp235x.h"

// r4-r11 are callee-saved, so they are never part of the hardware-stacked
// exception frame - they keep whatever value they held at the fault instant
// until something overwrites them, so the naked trampolines below must save
// them before any C code (which is free to use r4-r11 as scratch) runs.
typedef struct {
    uint32_t r4, r5, r6, r7, r8, r9, r10, r11;
} callee_saved_t;

// ---------------------------------------------------------------------------
// Fault diagnostics - UART1 register dump
// ---------------------------------------------------------------------------
//
// Bring-up diagnostic only: the fault handlers below transmit CFSR/HFSR/
// MMFAR/BFAR out UART1 (GPIO40 TX / GPIO41 RX - the drivewire plugin's own
// pins) before they start blinking, since blink-pattern counting alone
// wasn't reliably distinguishing which exception fired.  Always reconfigures
// UART1 from scratch rather than trusting whatever a crashed plugin left
// behind - a fault that happens before a plugin finishes its own UART bring
// up must not be silent.
#define FAULT_UART_BASE    UART1_BASE
#define FAULT_UART_TX_GPIO 40u
#define FAULT_UART_RX_GPIO 41u
#define FAULT_UART_BAUD    115200u

static void fault_uart_putc(uint8_t b) {
    while (UART_REG(FAULT_UART_BASE, UART_FR_OFFSET) & UART_FR_TXFF) { }
    UART_REG(FAULT_UART_BASE, UART_DR_OFFSET) = b;
}

static void fault_uart_puts(const char *s) {
    while (*s) fault_uart_putc((uint8_t)*s++);
}

static void fault_uart_put_hex32(uint32_t v) {
    static const char digits[] = "0123456789ABCDEF";
    for (int shift = 28; shift >= 0; shift -= 4) {
        fault_uart_putc((uint8_t)digits[(v >> shift) & 0xFu]);
    }
}

static void fault_uart_init(void) {
    // clk_peri must be running before UART1's reset can complete - see
    // drivewire_uart_init()'s equivalent comment in the plugin.
    CLOCK_PERI_CTRL = CLOCK_PERI_CTRL_ENABLE;

    RESET_RESET_SET = RESET_UART1;
    RESET_RESET_CLR = RESET_UART1;
    while (!(RESET_DONE & RESET_UART1)) { }

    GPIO_CTRL(FAULT_UART_TX_GPIO) = GPIO_CTRL_FUNC_UART;
    GPIO_CTRL(FAULT_UART_RX_GPIO) = GPIO_CTRL_FUNC_UART;
    // PAD_INPUT set on TX too, not just RX - confirmed against the SDK's own
    // gpio_set_function(), which enables the pad input buffer unconditionally
    // on every pin, output-only or not.
    GPIO_PAD(FAULT_UART_TX_GPIO) = (GPIO_PAD(FAULT_UART_TX_GPIO) | PAD_INPUT) & ~(uint32_t)(PAD_PU | PAD_PD | PAD_OUTPUT_DISABLE | PAD_ISO);
    GPIO_PAD(FAULT_UART_RX_GPIO) = (GPIO_PAD(FAULT_UART_RX_GPIO) | PAD_INPUT) & ~(uint32_t)(PAD_PU | PAD_PD | PAD_OUTPUT_DISABLE | PAD_ISO);

    // Same divisor computation as the plugin, against the same runtime clock
    // reading - if this dump is also garbled, the clock assumption itself is
    // suspect, not just the plugin's own init sequence.
    uint32_t clk_peri_hz = (uint32_t)RUNTIME->sysclk_mhz * 1000000u;
    uint64_t scaled = ((uint64_t)clk_peri_hz * 4u + (FAULT_UART_BAUD / 2u)) / FAULT_UART_BAUD;
    uint32_t ibrd = (uint32_t)(scaled >> 6);
    uint32_t fbrd = (uint32_t)(scaled & 0x3Fu);
    if (ibrd == 0u) {
        ibrd = 1u;
        fbrd = 0u;
    }

    UART_REG(FAULT_UART_BASE, UART_CR_OFFSET) = 0;
    UART_REG(FAULT_UART_BASE, UART_IBRD_OFFSET) = ibrd;
    UART_REG(FAULT_UART_BASE, UART_FBRD_OFFSET) = fbrd;
    UART_REG(FAULT_UART_BASE, UART_LCR_H_OFFSET) = UART_LCR_H_WLEN_8 | UART_LCR_H_FEN;
    UART_REG(FAULT_UART_BASE, UART_CR_OFFSET) = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

// tag identifies which handler is dumping (H/B/U), e.g.
// "\r\nFAULT[H] PC=10020184 R4=... R5=20081c00 ... R11=... CFSR=00000082 HFSR=40000000 MMFAR=00000000 BFAR=00000000\r\n"
// pc is the stacked return address (0 if the caller has none, e.g. Bus/Usage
// Fault which aren't captured via the naked/stacked-frame trampoline HardFault
// uses) - for a precise fault (BFSR/MMFSR PRECISERR/IACCVIOL/DACCVIOL) this is
// the exact faulting instruction, directly matchable against a .dis listing.
// saved is NULL when the caller has no callee-saved snapshot (Bus/Usage
// Fault, which aren't captured via the naked/stacked-frame trampoline
// HardFault uses).
static void fault_uart_dump(char tag, uint32_t pc, const callee_saved_t *saved, uint32_t cfsr, uint32_t hfsr, uint32_t mmfar, uint32_t bfar) {
    fault_uart_init();
    fault_uart_puts("\r\nFAULT[");
    fault_uart_putc((uint8_t)tag);
    fault_uart_puts("] PC=");
    fault_uart_put_hex32(pc);
    if (saved != NULL) {
        fault_uart_puts(" R4=");  fault_uart_put_hex32(saved->r4);
        fault_uart_puts(" R5=");  fault_uart_put_hex32(saved->r5);
        fault_uart_puts(" R6=");  fault_uart_put_hex32(saved->r6);
        fault_uart_puts(" R7=");  fault_uart_put_hex32(saved->r7);
        fault_uart_puts(" R8=");  fault_uart_put_hex32(saved->r8);
        fault_uart_puts(" R9=");  fault_uart_put_hex32(saved->r9);
        fault_uart_puts(" R10="); fault_uart_put_hex32(saved->r10);
        fault_uart_puts(" R11="); fault_uart_put_hex32(saved->r11);
    }
    fault_uart_puts(" CFSR=");
    fault_uart_put_hex32(cfsr);
    fault_uart_puts(" HFSR=");
    fault_uart_put_hex32(hfsr);
    fault_uart_puts(" MMFAR=");
    fault_uart_put_hex32(mmfar);
    fault_uart_puts(" BFAR=");
    fault_uart_put_hex32(bfar);
    fault_uart_puts("\r\n");
}

// Forward declarations
void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

// Default exception/interrupt handlers
#define MemManage_Handler   Default_Handler
#define SVC_Handler         Default_Handler
#define DebugMon_Handler    Default_Handler
#define PendSV_Handler      Default_Handler
#define SysTick_Handler     Default_Handler

// Declare stack section
extern uint32_t _estack;

// Vector table - must be placed at the start of flash
__attribute__ ((section(".isr_vector"), used))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack,      // Initial stack pointer
    Reset_Handler,                 // Reset handler
    NMI_Handler,                   // NMI handler
    HardFault_Handler,             // Hard fault handler
    MemManage_Handler,             // MPU fault handler
    BusFault_Handler,              // Bus fault handler
    UsageFault_Handler,            // Usage fault handler
    0, 0, 0, 0,                    // Reserved
    SVC_Handler,                   // SVCall handler
    DebugMon_Handler,              // Debug monitor handler
    0,                             // Reserved
    PendSV_Handler,                // PendSV handler
    SysTick_Handler,               // SysTick handler

    // Peripheral interrupt handlers follow 0-15 above.
    // 16-19
#if defined(STM32F4)
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
#else // RP235X
    irq_handler_timer0_irq_0, Default_Handler, Default_Handler, Default_Handler,
#endif 
    // 20-23
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // 24-27
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // 28-31
#if defined(STM32F4)
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
#else // RP235X
    Default_Handler, Default_Handler, irq_handler_usbctrl_irq, Default_Handler,
#endif 
    // 32-35
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // 36-39
#if defined(STM32F4)
    Default_Handler, Default_Handler, Default_Handler, vbus_connect_handler,
#else // RP235X
    Default_Handler, vbus_connect_handler, Default_Handler, Default_Handler,
#endif 
    // 40-43
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // Different STM32F4s have different numbers of interrupts.  The maximum
    // appears to be 96 (F446), which is what's included here.  This means
    // that 0x080001C4 onwards is free, but we'll not use anything until
    // 0x08000200 to be safe.
    // Note that the RP2350 has 52, so this is sufficient.  See datasheet
    // S3.2 - Interrupts
};

//
// Variables defined by the linker.
//
// Note these are "labels" that mark memory addresses, not variables that
// store data.  The address of the label IS the address we're interested in.
// Hence we use & below to get the addresses that these labels represent.
extern uint32_t _sidata;    // Start of .data section in FLASH
extern uint32_t _sdata;     // Start of .data section in RAM
extern uint32_t _edata;     // End of .data section in RAM
extern uint32_t _sbss;      // Start of .bss section in RAM
extern uint32_t _ebss;      // End of .bss section in RAM

// Location of RAM reserved for executing the main loop function from, if
// EXECUTE_FROM_RAM is defined.
#if defined(EXECUTE_FROM_RAM)
extern uint32_t _main_loop_start;   // Start of .main_loop section in FLASH
extern uint32_t _main_loop_end;     // End of .main_loop section in FLASH
extern uint32_t _ram_func_start;    // Start of .ram_func section in RAM
extern uint32_t _ram_func_end;      // End of .ram_func section in RAM
#endif

extern uint32_t _onerom_runtime_info_ram[]; // Start of .onerom_runtime_info section in RAM
extern uint32_t _onerom_runtime_info_flash[];   // Start of .onerom_runtime_info section in flash
extern uint32_t _onerom_runtime_info_end[];   // End of .onerom_runtime_info section in RAM

void ora_launch_plugins(void);

// Reset handler
void Reset_Handler(void) {
    // Enable hard floating point support:
    // - Same on STM32F4 M4 and RP235X Cortex-M33
    // - Enable CP10 and CP11 (FP extension) in the Cortex-M33
    SCB_CPACR |= SCB_CPACR_ENABLE_FP; // Enable CP10 and CP11 full access
    __asm volatile ("dsb");
    __asm volatile ("isb");

    // We use memcpy and memset because it's likely to be faster than anything
    // we could come up with.

    // Copy onerom_runtime_info_t from flash to RAM, magic last.
    // The magic acts as a commit indicator: valid magic means the full
    // struct is visible. Skip the first 4 bytes (magic) in the main copy.
    const size_t runtime_size = (size_t)((char*)_onerom_runtime_info_end - (char*)_onerom_runtime_info_ram);

    memcpy((uint8_t*)_onerom_runtime_info_ram + 4,
        (uint8_t*)_onerom_runtime_info_flash + 4,
        runtime_size - 4);

    // Ensure all struct bytes are visible before the magic is written.
    __asm volatile ("dmb" ::: "memory");

    // Write magic last.
    memcpy(_onerom_runtime_info_ram,
        _onerom_runtime_info_flash,
        4);

    // Copy data section from flash to RAM
    memcpy(&_sdata, &_sidata, (unsigned int)((char*)&_edata - (char*)&_sdata));
    
    // Zero out bss section  
    memset(&_sbss, 0, (unsigned int)((char*)&_ebss - (char*)&_sbss));
    
    // Call the main function
    firmware_main();

    // Main has returned - which means we are doing byte serving with PIOs.
    // We can now launch plugins.
    ora_launch_plugins();

    // Belt and braces - ora_launch_plugins never returns.
    while(1) {
        __asm volatile("wfi");
    }
}

// Default handler for unhandled interrupts - fast continuous blink
void Default_Handler(void) {
    // IPSR (readable directly, no naked/stacked-frame trampoline needed)
    // gives the exact exception/IRQ number that landed here - this handler
    // covers everything not individually named in the vector table, plus
    // MemManage (aliased to it above), so knowing which one fired matters.
    uint32_t ipsr;
    __asm volatile ("mrs %0, ipsr" : "=r" (ipsr));
    fault_uart_init();
    fault_uart_puts("\r\nFAULT[D] IPSR=");
    fault_uart_put_hex32(ipsr);
    fault_uart_puts("\r\n");

    // Halt regardless of the status LED: an unhandled interrupt stays pending,
    // so returning from here just re-enters in a tight spin.  blink_pattern()
    // gates on status_led_enabled, so the LED only blinks when it is enabled.
    setup_status_led();
    while (1) {
        blink_pattern(100000, 100000, 255);
    }
}

// NMI_Handler - single blink pattern
void NMI_Handler(void) {
    // Force the status LED on so the fault is visible even if it was off.
    RUNTIME->status_led_enabled = 1;
    setup_status_led();

    while(1) {
        blink_pattern(100000, 500000, 1); // Single blink
        delay(1000000); // Long pause
    }
}

typedef struct {
    uint32_t r0, r1, r2, r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} stacked_frame_t;

// HardFault_Handler - double blink pattern
void HardFault_C(stacked_frame_t *frame, callee_saved_t *saved) {
    // Fault status registers
    volatile uint32_t cfsr  = *(volatile uint32_t *)0xE000ED28; // MMFSR+BFSR+UFSR
    volatile uint32_t hfsr  = *(volatile uint32_t *)0xE000ED2C;
    volatile uint32_t mmfar = *(volatile uint32_t *)0xE000ED34; // Valid if CFSR.MMARVALID
    volatile uint32_t bfar  = *(volatile uint32_t *)0xE000ED38; // Valid if CFSR.BFARVALID

    // Force the status LED on so the fault is visible even if it was off.
    RUNTIME->status_led_enabled = 1;
    setup_status_led();

    fault_uart_dump('H', frame->pc, saved, cfsr, hfsr, mmfar, bfar);

    while(1) {
        blink_pattern(100000, 200000, 2);
        delay(1000000);
    }
}

void __attribute__((naked)) HardFault_Handler(void) {
    __asm volatile (
        // Capture the original exception frame pointer (msp or psp) into r0
        // *before* touching the stack ourselves, so the later push below -
        // which always targets msp, since Handler mode never uses psp -
        // cannot shift it out from under us.
        "tst   lr, #4\n"
        "ite   eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "push  {r4-r11}\n"
        "mov   r1, sp\n"
        "b     HardFault_C\n"
    );
}

// BusFault_Handler - triple blink pattern
void BusFault_Handler(void) {
    volatile uint32_t cfsr  = *(volatile uint32_t *)0xE000ED28;
    volatile uint32_t hfsr  = *(volatile uint32_t *)0xE000ED2C;
    volatile uint32_t mmfar = *(volatile uint32_t *)0xE000ED34;
    volatile uint32_t bfar  = *(volatile uint32_t *)0xE000ED38;

    // Force the status LED on so the fault is visible even if it was off.
    RUNTIME->status_led_enabled = 1;
    setup_status_led();

    fault_uart_dump('B', 0, NULL, cfsr, hfsr, mmfar, bfar);

    while(1) {
        blink_pattern(100000, 200000, 3); // Triple blink
        delay(1000000); // Long pause
    }
}

// UsageFault_Handler - quadruple blink pattern
void UsageFault_Handler(void) {
    volatile uint32_t cfsr  = *(volatile uint32_t *)0xE000ED28;
    volatile uint32_t hfsr  = *(volatile uint32_t *)0xE000ED2C;
    volatile uint32_t mmfar = *(volatile uint32_t *)0xE000ED34;
    volatile uint32_t bfar  = *(volatile uint32_t *)0xE000ED38;

    // Force the status LED on so the fault is visible even if it was off.
    RUNTIME->status_led_enabled = 1;
    setup_status_led();

    fault_uart_dump('U', 0, NULL, cfsr, hfsr, mmfar, bfar);

    while(1) {
        blink_pattern(100000, 200000, 4); // Quadruple blink
        delay(1000000); // Long pause
    }
}

#endif // !TEST_BUILD
