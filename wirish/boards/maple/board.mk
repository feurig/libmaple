# Board specific make defines for Maple board

CPU = cortex-m3
ARCH = armv7-m
MCU_SERIES = stm32f1
MCU = MCU_STM32F103RB
MEMORY = sram_20k_flash_128k
ifeq ($(TARGET),jtag)
DFUID = 0483:df11
DFUALT = 0
endif

