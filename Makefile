##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.6.0-B36] date: [Sun Mar 30 06:05:42 PDT 2025] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = USB_PD_PSU_FW


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/usart.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/USBPD_CUST_NVM_API.c \
Core/Src/ina236.c \
Core/Src/USB_PD_core.c \
Core/Src/printf.c \
Core/Src/display.c \
Core/Src/dma.c \
Core/Src/adc.c \
Core/Src/sysmem.c \
Core/Src/syscalls.c \
Core/Src/system_stm32f1xx.c \
Core/Src/tim.c \
Core/Src/ee.c \
Core/Src/u8g2/mui.c \
Core/Src/u8g2/mui_u8g2.c \
Core/Src/u8g2/u8g2_arc.c \
Core/Src/u8g2/u8g2_bitmap.c \
Core/Src/u8g2/u8g2_box.c \
Core/Src/u8g2/u8g2_buffer.c \
Core/Src/u8g2/u8g2_button.c \
Core/Src/u8g2/u8g2_circle.c \
Core/Src/u8g2/u8g2_cleardisplay.c \
Core/Src/u8g2/u8g2_d_memory.c \
Core/Src/u8g2/u8g2_d_setup.c \
Core/Src/u8g2/u8g2_font.c \
Core/Src/u8g2/u8g2_fonts.c \
Core/Src/u8g2/u8g2_hvline.c \
Core/Src/u8g2/u8g2_input_value.c \
Core/Src/u8g2/u8g2_intersection.c \
Core/Src/u8g2/u8g2_kerning.c \
Core/Src/u8g2/u8g2_line.c \
Core/Src/u8g2/u8g2_ll_hvline.c \
Core/Src/u8g2/u8g2_message.c \
Core/Src/u8g2/u8g2_polygon.c \
Core/Src/u8g2/u8g2_selection_list.c \
Core/Src/u8g2/u8g2_setup.c \
Core/Src/u8g2/u8log.c \
Core/Src/u8g2/u8log_u8g2.c \
Core/Src/u8g2/u8log_u8x8.c \
Core/Src/u8g2/u8x8_8x8.c \
Core/Src/u8g2/u8x8_byte.c \
Core/Src/u8g2/u8x8_cad.c \
Core/Src/u8g2/u8x8_capture.c \
Core/Src/u8g2/u8x8_d_a2printer.c \
Core/Src/u8g2/u8x8_d_gp1247ai.c \
Core/Src/u8g2/u8x8_d_gp1287ai.c \
Core/Src/u8g2/u8x8_d_gp1294ai.c \
Core/Src/u8g2/u8x8_d_gu800.c \
Core/Src/u8g2/u8x8_d_hd44102.c \
Core/Src/u8g2/u8x8_d_il3820_296x128.c \
Core/Src/u8g2/u8x8_d_ist3020.c \
Core/Src/u8g2/u8x8_d_ist3088.c \
Core/Src/u8g2/u8x8_d_ist7920.c \
Core/Src/u8g2/u8x8_d_ks0108.c \
Core/Src/u8g2/u8x8_d_lc7981.c \
Core/Src/u8g2/u8x8_d_ld7032_60x32.c \
Core/Src/u8g2/u8x8_d_ls013b7dh03.c \
Core/Src/u8g2/u8x8_d_max7219.c \
Core/Src/u8g2/u8x8_d_pcd8544_84x48.c \
Core/Src/u8g2/u8x8_d_pcf8812.c \
Core/Src/u8g2/u8x8_d_pcf8814_hx1230.c \
Core/Src/u8g2/u8x8_d_s1d15300.c \
Core/Src/u8g2/u8x8_d_s1d15721.c \
Core/Src/u8g2/u8x8_d_s1d15e06.c \
Core/Src/u8g2/u8x8_d_sbn1661.c \
Core/Src/u8g2/u8x8_d_sed1330.c \
Core/Src/u8g2/u8x8_d_sh1106_64x32.c \
Core/Src/u8g2/u8x8_d_sh1106_72x40.c \
Core/Src/u8g2/u8x8_d_sh1107.c \
Core/Src/u8g2/u8x8_d_sh1108.c \
Core/Src/u8g2/u8x8_d_sh1122.c \
Core/Src/u8g2/u8x8_d_ssd1305.c \
Core/Src/u8g2/u8x8_d_ssd1306_128x32.c \
Core/Src/u8g2/u8x8_d_ssd1306_128x64_noname.c \
Core/Src/u8g2/u8x8_d_ssd1306_2040x16.c \
Core/Src/u8g2/u8x8_d_ssd1306_48x64.c \
Core/Src/u8g2/u8x8_d_ssd1306_64x32.c \
Core/Src/u8g2/u8x8_d_ssd1306_64x48.c \
Core/Src/u8g2/u8x8_d_ssd1306_72x40.c \
Core/Src/u8g2/u8x8_d_ssd1306_96x16.c \
Core/Src/u8g2/u8x8_d_ssd1306_96x40.c \
Core/Src/u8g2/u8x8_d_ssd1309.c \
Core/Src/u8g2/u8x8_d_ssd1316.c \
Core/Src/u8g2/u8x8_d_ssd1317.c \
Core/Src/u8g2/u8x8_d_ssd1318.c \
Core/Src/u8g2/u8x8_d_ssd1320.c \
Core/Src/u8g2/u8x8_d_ssd1322.c \
Core/Src/u8g2/u8x8_d_ssd1325.c \
Core/Src/u8g2/u8x8_d_ssd1326.c \
Core/Src/u8g2/u8x8_d_ssd1327.c \
Core/Src/u8g2/u8x8_d_ssd1329.c \
Core/Src/u8g2/u8x8_d_ssd1362.c \
Core/Src/u8g2/u8x8_d_ssd1606_172x72.c \
Core/Src/u8g2/u8x8_d_ssd1607_200x200.c \
Core/Src/u8g2/u8x8_d_st7511.c \
Core/Src/u8g2/u8x8_d_st75160.c \
Core/Src/u8g2/u8x8_d_st75256.c \
Core/Src/u8g2/u8x8_d_st7528.c \
Core/Src/u8g2/u8x8_d_st75320.c \
Core/Src/u8g2/u8x8_d_st7539.c \
Core/Src/u8g2/u8x8_d_st7565.c \
Core/Src/u8g2/u8x8_d_st7567.c \
Core/Src/u8g2/u8x8_d_st7571.c \
Core/Src/u8g2/u8x8_d_st7586s_erc240160.c \
Core/Src/u8g2/u8x8_d_st7586s_jlx320160.c \
Core/Src/u8g2/u8x8_d_st7586s_jlx384160.c \
Core/Src/u8g2/u8x8_d_st7586s_s028hn118a.c \
Core/Src/u8g2/u8x8_d_st7586s_ymc240160.c \
Core/Src/u8g2/u8x8_d_st7588.c \
Core/Src/u8g2/u8x8_d_st7920.c \
Core/Src/u8g2/u8x8_d_stdio.c \
Core/Src/u8g2/u8x8_d_t6963.c \
Core/Src/u8g2/u8x8_d_uc1601.c \
Core/Src/u8g2/u8x8_d_uc1604.c \
Core/Src/u8g2/u8x8_d_uc1608.c \
Core/Src/u8g2/u8x8_d_uc1609.c \
Core/Src/u8g2/u8x8_d_uc1610.c \
Core/Src/u8g2/u8x8_d_uc1611.c \
Core/Src/u8g2/u8x8_d_uc1617.c \
Core/Src/u8g2/u8x8_d_uc1628.c \
Core/Src/u8g2/u8x8_d_uc1638.c \
Core/Src/u8g2/u8x8_d_uc1701_dogs102.c \
Core/Src/u8g2/u8x8_d_uc1701_mini12864.c \
Core/Src/u8g2/u8x8_debounce.c \
Core/Src/u8g2/u8x8_display.c \
Core/Src/u8g2/u8x8_fonts.c \
Core/Src/u8g2/u8x8_gpio.c \
Core/Src/u8g2/u8x8_input_value.c \
Core/Src/u8g2/u8x8_message.c \
Core/Src/u8g2/u8x8_selection_list.c \
Core/Src/u8g2/u8x8_setup.c \
Core/Src/u8g2/u8x8_string.c \
Core/Src/u8g2/u8x8_u16toa.c \
Core/Src/u8g2/u8x8_u8toa.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c 




# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xB


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-ICore/Inc/u8g2


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103XX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
