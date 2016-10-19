PROJECT_NAME = stm32l152-disco

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT_SRC = src
STDPERIPH_SRC = Drivers/STM32L1xx_StdPeriph_Driver/src/
STMTOUCH_SRC = Drivers/STMTouch_Driver/src/
OBJ_DIR = build

vpath %.c $(PROJECT_SRC)
vpath %.c $(STDPERIPH_SRC)
vpath %.c $(STMTOUCH_SRC)

SRCS = main.c
SRCS += discover_functions.c
SRCS += icc_measure.c
SRCS += icc_measure_Ram.c
SRCS += main.c
SRCS += stm32l1xx_it.c
SRCS += stm32l_discovery_lcd.c
SRCS += system_stm32l1xx.c
SRCS += tsl_user.c

SRCS += Device/startup_stm32l1xx_md.s

# StdPeriph library
EXT_SRCS = misc.c
EXT_SRCS += stm32l1xx_adc.c
EXT_SRCS += stm32l1xx_exti.c
EXT_SRCS += stm32l1xx_flash.c
EXT_SRCS += stm32l1xx_gpio.c
EXT_SRCS += stm32l1xx_pwr.c
EXT_SRCS += stm32l1xx_rcc.c
EXT_SRCS += stm32l1xx_rtc.c
EXT_SRCS += stm32l1xx_lcd.c
EXT_SRCS += stm32l1xx_syscfg.c

# STMTouch library
EXT_SRCS += tsl.c
EXT_SRCS += tsl_acq.c
EXT_SRCS += tsl_acq_stm32l1xx_sw.c
EXT_SRCS += tsl_dxs.c
EXT_SRCS += tsl_ecs.c
EXT_SRCS += tsl_filter.c
EXT_SRCS += tsl_globals.c
EXT_SRCS += tsl_linrot.c
EXT_SRCS += tsl_object.c
EXT_SRCS += tsl_time.c
EXT_SRCS += tsl_time_stm32l1xx.c
EXT_SRCS += tsl_touchkey.c

EXT_OBJ = $(addprefix $(OBJ_DIR)/, $(EXT_SRCS:.c=.o))

INC_DIRS  = src/
INC_DIRS += Drivers/STM32L1xx_StdPeriph_Driver/inc/
INC_DIRS += Drivers/CMSIS/Device/ST/STM32L1xx/Include/
INC_DIRS += Drivers/CMSIS/Include/
INC_DIRS += Drivers/STMTouch_Driver/inc/

INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32L1XX_MD -DHSE_VALUE=1000000

CFLAGS += -ggdb -O0 -std=c99
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mthumb-interwork -Wl,--gc-sections

WFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter -Wno-unused-function
LFLAGS = -TDevice/gcc.ld -lm -lc -lnosys

# Create a directory for object files
$(shell mkdir $(OBJ_DIR) > /dev/null 2>&1)

.PHONY: all
all: $(PROJECT_NAME)

.PHONY: $(PROJECT_NAME)
$(PROJECT_NAME): $(PROJECT_NAME).elf

$(PROJECT_NAME).elf: $(SRCS) $(EXT_OBJ)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $^ $(WFLAGS) $(LFLAGS) -o $@
	$(OBJCOPY) -O ihex $(PROJECT_NAME).elf   $(PROJECT_NAME).hex
	$(OBJCOPY) -O binary $(PROJECT_NAME).elf $(PROJECT_NAME).bin

$(OBJ_DIR)/%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

clean:
	rm -rf $(OBJ_DIR) $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

flash: $(PROJECT_NAME).elf
	openocd -f interface/stlink-v2.cfg -f target/stm32lx_stlink.cfg \
	    -c "init" -c "flash probe 0" -c "program stm32l152-disco.elf 0 verify reset"
