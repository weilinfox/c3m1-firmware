# C3M1 Camera Firmware - Cross-platform Makefile (Linux / macOS)

TARGET    := c3m1_camera
BUILD_DIR := build

# Toolchain
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY := $(PREFIX)objcopy
OBJDUMP := $(PREFIX)objdump
SIZE    := $(PREFIX)size

# Optimization & Debug (matches original Output/ configuration: -O1 -g3)
OPT     ?= -O1
DEBUG   ?= -g3

# MCU flags
CPU     := -mcpu=cortex-m3
MCU     := $(CPU) -mthumb -mfloat-abi=soft

# Preprocessor defines
C_DEFS  := \
	-DDEBUG \
	-DUSE_HAL_DRIVER \
	-DSTM32F103xE \
	-DFOR_TEST

# Include paths
# Note: Drivers/STM32F1xx_HAL_Driver/Inc is required so "../../BSP/..." includes in main.c resolve cleanly.
C_INCLUDES := \
	-ICore/Inc \
	-IDrivers/STM32F1xx_HAL_Driver/Inc \
	-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
	-IDrivers/CMSIS/Include \
	-IFATFS/Target \
	-IFATFS/App \
	-IMiddlewares/Third_Party/FatFs/src \
	-IMiddlewares/Third_Party/FatFs/src/drivers

# C sources (47 files, matching original Output/objects.list; lcd_ex.c is #included by lcd.c)
C_SOURCES := \
	Core/Src/adc.c \
	Core/Src/fsmc.c \
	Core/Src/gpio.c \
	Core/Src/main.c \
	Core/Src/sdio.c \
	Core/Src/stm32f1xx_hal_msp.c \
	Core/Src/stm32f1xx_it.c \
	Core/Src/syscalls.c \
	Core/Src/sysmem.c \
	Core/Src/system_stm32f1xx.c \
	Core/Src/tim.c \
	Core/Src/usart.c \
	Drivers/BSP/EXTI/exti.c \
	Drivers/BSP/KEY/key.c \
	Drivers/BSP/LCD/lcd.c \
	Drivers/BSP/LED/led.c \
	Drivers/BSP/OV7725/ov7725.c \
	Drivers/BSP/OV7725/sccb.c \
	Drivers/BSP/STMFLASH/stmflash.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sd.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sram.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_fsmc.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_sdmmc.c \
	Drivers/SYSTEM/delay/delay.c \
	Drivers/SYSTEM/sys/sys.c \
	FATFS/App/fatfs.c \
	FATFS/Target/bsp_driver_sd.c \
	Middlewares/Third_Party/FatFs/src/diskio.c \
	Middlewares/Third_Party/FatFs/src/ff.c \
	Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
	Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.c \
	Middlewares/Third_Party/FatFs/src/option/syscall.c

# Assembly sources
ASM_SOURCES := \
	Core/Startup/startup_stm32f103zetx.s

# Compiler / Assembler / Linker flags
CFLAGS := $(MCU) -std=gnu11 $(DEBUG) $(OPT) $(C_DEFS) $(C_INCLUDES) \
	-Wall -ffunction-sections -fdata-sections --specs=nano.specs

ASFLAGS := $(MCU) $(DEBUG) -DDEBUG --specs=nano.specs

LDSCRIPT := STM32F103ZETX_FLASH.ld
LDFLAGS  := $(MCU) -T$(LDSCRIPT) --specs=nosys.specs --specs=nano.specs \
	-static -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--gc-sections \
	-Wl,--start-group -lc -lm -Wl,--end-group

# Object & dependency lists
OBJECTS := \
	$(patsubst %.c,$(BUILD_DIR)/obj/%.o,$(C_SOURCES)) \
	$(patsubst %.s,$(BUILD_DIR)/obj/%.o,$(ASM_SOURCES))
DEPS := $(OBJECTS:.o=.d)

.PHONY: all clean flash flash-stlink size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/obj/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:.o=.d)" $< -o $@

$(BUILD_DIR)/obj/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) -MMD -MP -MF"$(@:.o=.d)" $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f dap.cfg -c "program $< verify reset exit"

flash-stlink: $(BUILD_DIR)/$(TARGET).hex
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "reset_config none; program $< verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
