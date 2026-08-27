# Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
#
# MIT License

TOOLCHAIN ?=
CC      := $(if $(TOOLCHAIN),$(TOOLCHAIN)/arm-none-eabi-gcc,arm-none-eabi-gcc)
OBJCOPY := $(if $(TOOLCHAIN),$(TOOLCHAIN)/arm-none-eabi-objcopy,arm-none-eabi-objcopy)
OBJDUMP := $(if $(TOOLCHAIN),$(TOOLCHAIN)/arm-none-eabi-objdump,arm-none-eabi-objdump)

MAJOR_VERSION := 0
MINOR_VERSION := 1
PATCH_VERSION := 0
BUILD_VERSION := 0

# TEST_PATTERN=1 builds the bring-up test variant (see drivewire_main.c) -
# waits for the write knock, lights the status LED, then relays an
# incrementing byte pattern to UART1 forever, instead of the real protocol.
TEST_PATTERN ?= 0

# TEST_UART_ONLY=1 builds an even more minimal bring-up test: brings up
# UART1 and blasts a repeating pattern forever, skipping the address monitor
# and knock logic entirely.  Isolates UART1's own hardware bring-up from
# everything else.  Mutually exclusive with TEST_PATTERN=1 (this one wins if
# both are set).  Most of the file's functions go genuinely unused in this
# build, so warnings are relaxed rather than threading a third set of
# #ifndef guards through otherwise-unrelated code for a throwaway test.
TEST_UART_ONLY ?= 0

# TEST_UART_SPACED=1 builds the simplest possible communications test: sends
# a single 0x55 byte at a time, with a gap of roughly 8 bit periods between
# each, instead of a dense continuous stream - meant to be easy to trigger
# on and measure a single byte's timing on a scope/logic analyzer.  Same
# relaxed-warnings rationale as TEST_UART_ONLY.  Mutually exclusive with the
# other TEST_* variants.
TEST_UART_SPACED ?= 0

# BAUD overrides UART1's baud rate (default 921600, see DW_UART_BAUD in
# drivewire_main.c) - useful for bring-up testing against equipment that
# can't reliably decode 921600, e.g. `make TEST_PATTERN=1 BAUD=115200`.
BAUD ?=

ORA_DIR := ../../../firmware/ora
ORA_INCLUDE := -I$(ORA_DIR)
SDRR_INCLUDE := -I../../../firmware/include -I../../../firmware/generated
PLUGIN_INCLUDE := -I.

BUILD_DIR := build

PLUGIN_BASE   := 0x10020000
PLUGIN_PREFIX := drivewire_plugin

ELF := $(BUILD_DIR)/$(PLUGIN_PREFIX).elf
BIN := $(BUILD_DIR)/$(PLUGIN_PREFIX).bin

SRC := src/drivewire_main.c

CFLAGS := -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 \
          -fstack-usage -fconserve-stack \
          -O2 -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections \
          -MMD -MP \
          -DMAJOR_VERSION=$(MAJOR_VERSION) -DMINOR_VERSION=$(MINOR_VERSION) \
          -DPATCH_VERSION=$(PATCH_VERSION) -DBUILD_VERSION=$(BUILD_VERSION) \
          $(SDRR_INCLUDE) $(ORA_INCLUDE) $(PLUGIN_INCLUDE) \
          $(if $(filter 1,$(TEST_PATTERN)),-DDRIVEWIRE_TEST_PATTERN) \
          $(if $(filter 1,$(TEST_UART_ONLY)),-DDRIVEWIRE_TEST_UART_ONLY -Wno-unused-function -Wno-unused-variable) \
          $(if $(filter 1,$(TEST_UART_SPACED)),-DDRIVEWIRE_TEST_UART_SPACED -Wno-unused-function -Wno-unused-variable) \
          $(if $(BAUD),-DDW_UART_BAUD=$(BAUD)u)

LDFLAGS := -nostdlib \
           -T drivewire_plugin.ld \
           -Wl,--defsym,PLUGIN_TYPE=1 \
           -Wl,--fatal-warnings \
           -Wl,--no-warn-rwx-segments \
           -Wl,-Map=$(BUILD_DIR)/$(PLUGIN_PREFIX).map

PLUGIN_OBJ := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC))

.PHONY: all clean distclean

all: $(BIN)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "Compiling $<..."
	@$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(PLUGIN_OBJ) $(ORA_DIR)/plugin.h $(ORA_DIR)/api.h $(ORA_DIR)/system.h drivewire_plugin.ld
	@echo "Linking..."
	@$(CC) $(CFLAGS) $(LDFLAGS) $(PLUGIN_OBJ) -lc -lgcc -o $@

$(BIN): $(ELF)
	@echo "Converting ELF to binary..."
	@$(OBJCOPY) -O binary $< $@
	@$(OBJCOPY) --pad-to $$(($(PLUGIN_BASE) + 0x10000)) --gap-fill 0xFF -O binary $< $@
	@$(OBJDUMP) -s -j .plugin_header $(ELF) > $(BUILD_DIR)/$(PLUGIN_PREFIX).dis
	@$(OBJDUMP) -d -S $(ELF) >> $(BUILD_DIR)/$(PLUGIN_PREFIX).dis

$(BUILD_DIR):
	@mkdir -p $@

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)

distclean: clean

-include $(PLUGIN_OBJ:.o=.d)
