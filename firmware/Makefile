# ---------------------------------------------------------------------------
# 两轮差速底盘固件 —— Makefile（arm-none-eabi-gcc 流程）
#
# 用法：
#   make            编译，生成 build/firmware.elf / .hex / .bin
#   make flash      用 st-flash 烧录（需 ST-Link 与 st-link 工具）
#   make flash-ocd  用 OpenOCD 烧录
#   make clean      清理
#   make size       查看 Flash/RAM 占用
#
# 依赖：arm-none-eabi-gcc 在 PATH 中
#   Ubuntu: sudo apt install gcc-arm-none-eabi
#   macOS : brew install arm-none-eabi-gcc
# ---------------------------------------------------------------------------

TARGET   = firmware
BUILD    = build

PREFIX   = arm-none-eabi-
CC       = $(PREFIX)gcc
AS       = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size

# Cortex-M3，STM32F103C8T6
CPU      = -mcpu=cortex-m3
MCU      = $(CPU) -mthumb

# 源文件
C_SOURCES := $(wildcard Src/*.c)
ASM_SOURCES := Startup/startup_stm32f10x_md.s

# 头文件路径
C_INCLUDES := -IInc

# 编译选项
OPT      = -Og
DEBUG    = -g3
CFLAGS   = $(MCU) $(C_INCLUDES) $(OPT) $(DEBUG) \
           -std=c99 -Wall -Wextra \
           -ffunction-sections -fdata-sections \
           -fno-common -fstack-usage
ASFLAGS  = $(MCU) $(OPT) $(DEBUG)

# 链接：--gc-sections 把没用到的函数裁掉，蓝丸只有 64 KB Flash 很关键
LDSCRIPT = STM32F103C8Tx_FLASH.ld
LDFLAGS  = $(MCU) -T$(LDSCRIPT) \
           -specs=nano.specs -specs=nosys.specs \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
           -Wl,--print-memory-usage

# 目标文件
OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# ---- 规则 ----
.PHONY: all clean flash flash-ocd size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILD)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD):
	mkdir -p $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

flash-ocd: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	        -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
