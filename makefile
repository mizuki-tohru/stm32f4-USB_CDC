#################################################
# MAKEFILE For STM32F4xxx Devices 				#
# (c) 20111016 Nemui Trinomius					#
# http://nemuisan.blog.bai.ne.jp				#
# Based on Martin Thomas's makefiles thanks!	#
#################################################

# Environment Dependent!!! This environment assure under WINDOWS !!
# Throw path into YOUR environments
#export PATH = %SYSTEMROOT%;$(TOOLDIR)/arm-gcc/bin;$(TOOLDIR)/bin;$(MAKEDIR);$(OCDIR);$(DFUDIR)

# Toolchain prefix (i.e arm-elf -> arm-elf-gcc.exe)
ifeq ($(shell uname), Linux)
  TCHAIN  = arm-m4-eabi
else
  TCHAIN  = /usr/cross/bin/arm-m4-eabi
endif
#OCD		= openocd
#OCDMODE = SWD
#OCDMODE = JTAG

# Development Tools based on CodeSourceryG++
#DEVTOOL = NETX
#DEVTOOL = SOURCERY

#ifeq ($(DEVTOOL),NETX)
#TOOLDIR = C:/Devz/ARM/NetX
#REMOVAL = rm
#else
TOOLDIR = /bin
REMOVAL = /bin/rm
#endif

#MAKEDIR = C:/Cygwin/bin
#OCDIR	= C:/Devz/ARM/OCD
#ifeq ($(OCDMODE),SWD)
#OCD_CMD = -s $(OCDIR)/tcl					\
#		  -f interface/vsllink_swd.cfg		\
#          -f target/stm32f4x_flash.cfg
#else
#OCD_CMD = -s $(OCDIR)/tcl					\
#		  -f interface/jtagkey2.cfg 		\
#          -f target/stm32f4x_flash.cfg
#endif

#DFUDIR	= C:/Devz/ARM/ST/DFUse/BIN
		  
#WSHELL  = cmd
#MSGECHO = /bin/echo.exe
WSHELL  = /bin/bash
MSGECHO = /bin/echo
#GDBDIR  = C:/Devz/ARM/insight/bin
#INSIGHT = $(GDBDIR)/arm-none-eabi-insight
# Environment Dependent!!!


# OPTIMIZE Definition
OPTIMIZE		= 2
#OPTIMIZE		= 0

# FPU Definition
USING_FPU		= -mfloat-abi=softfp  -mfpu=fpv4-sp-d16
#USING_FPU		= -mfloat-abi=soft


# GCC4.5.x Specific Option
#USE_LTO			= -flto -fipa-sra
#ALIGNED_ACCESS	= -mno-unaligned-access

# Apprication Version
APP_VER = W.I.P


# Basic definition
#EVAL_BOARD    	= USE_REDBULL
#EVAL_BOARD    	= USE_STM32F4DISCOVERY

#ifeq ($(EVAL_BOARD),USE_STM32F4DISCOVERY)
SUBMODEL		= STM32F407VGT6
#else
#SUBMODEL		= STM32F407ZGT6
#endif
MPU_DENSITY		= STM32F4xx
HSE_CLOCK 		= 8000000
PERIF_DRIVER    = USE_STDPERIPH_DRIVER
#USE_EXT_SRAM    = DATA_IN_ExtSRAM
#USE_TOUCH_SENCE = USE_ADS7843

# Use Display Device?
#16bit-DataBus
#USE_DISPLAY		= USE_SSD1963_TFT

#USE_DISPLAY		= USE_ILI932x_TFT
#USE_DISPLAY		= USE_SSD1289_TFT
#USE_DISPLAY		= USE_SSD2119_TFT
#USE_DISPLAY		= USE_ILI9132_TFT
#USE_DISPLAY		= USE_ILI9327_TFT
#USE_DISPLAY		= USE_ILI9481_TFT
#USE_DISPLAY		= USE_R61509V_TFT
#USE_DISPLAY		= USE_HX8312A_TFT
#USE_DISPLAY		= USE_HX8347x_TFT
#USE_DISPLAY		= USE_HX8352A_TFT
#USE_DISPLAY		= USE_S1D19122_TFT
#USE_DISPLAY		= USE_REL225L01_TFT
#USE_DISPLAY		= USE_S6D0128_TFT
#USE_DISPLAY		= USE_LGDP4511_TFT
#USE_DISPLAY		= USE_LGDP4524_TFT
#USE_DISPLAY		= USE_S6E63D6_OLED
#USE_DISPLAY		= USE_TL1771_TFT
#USE_DISPLAY		= USE_HD66772_TFT
#USE_DISPLAY		= USE_ILI934x_TFT

#8bit-DataBus
#USE_DISPLAY		= USE_ST7735_TFT
#USE_DISPLAY		= USE_ST7735R_TFT
#USE_DISPLAY		= USE_SSD1286A_TFT
#USE_DISPLAY		= USE_ILI9225_TFT
#USE_DISPLAY		= USE_R61514S_TFT
#USE_DISPLAY		= USE_HX8340B_TFT
#USE_DISPLAY		= USE_S1D19105_TFT
#USE_DISPLAY		= USE_SPFD54124_TFT
#USE_DISPLAY		= USE_SEPS525_OLED
#USE_DISPLAY		= USE_SSD1339_OLED

#Software/Hardware SPI
#USE_DISPLAY		= USE_SSD1283A_SPI_TFT
#USE_DISPLAY		= USE_ILI932x_SPI_TFT
#USE_DISPLAY		= USE_ILI934x_SPI_TFT
#USE_DISPLAY		= USE_HX8347D_SPI_TFT
#USE_DISPLAY		= USE_ILI9163B_SPI_TFT
#USE_DISPLAY		= USE_ST7735_SPI_TFT
#USE_DISPLAY		= USE_ST7735R_SPI_TFT
#USE_DISPLAY		= USE_SPFD54124_SPI_TFT
#USE_DISPLAY		= USE_SSD2119_SPI_TFT
#USE_DISPLAY		= USE_SSD1332_SPI_OLED
#USE_DISPLAY		= USE_SSD1339_SPI_OLED
#USE_DISPLAY		= USE_SEPS525_SPI_OLED
#USE_DISPLAY		= USE_S6E63D6_SPI_OLED

# For JPEG Support
#USE_JPEG_LIB    = USE_TINYJPEG_LIB
#USE_JPEG_LIB    = USE_IJG_LIB

# Use Display Fonts? (misaki(8x8) or M+(10x10) fonts)
#USE_FONTSIZE    = FONT8x8
#USE_FONTSIZE    = FONT10x10
#USE_KANJI		= USE_KANJIFONT

# Use FreeRTOS?
#OS_SUPPORT		= BARE_METAL
OS_SUPPORT		= USE_FREERTOS

#FREERTOS_DIR = ../FreeRTOSV7.1.1
FREERTOS_DIR = ../FreeRTOSV7.2.0/FreeRTOS
FREERTOS_COM = $(FREERTOS_DIR)/Demo/Common/include

# Synthesis makefile Defines
DEFZ = $(EVAL_BOARD)  $(MPU_DENSITY)  $(PERIF_DRIVER)    $(VECTOR_START) $(ROM_START)		\
	   $(USE_DISPLAY) $(USE_FONTSIZE) $(USE_TOUCH_SENCE) $(USE_LCD_SPI)	 $(USE_JPEG_LIB)	\
	   $(OS_SUPPORT)  $(USE_SPI_DMA)  $(USE_EXT_SRAM)    $(SUBMODEL)	 $(USE_KANJI)
SYNTHESIS_DEFS	= $(addprefix -D,$(DEFZ)) 							\
				 -DPACK_STRUCT_END=__attribute\(\(packed\)\) 		\
				 -DALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
				 -DMPU_SUBMODEL=\"$(SUBMODEL)\"						\
				 -DAPP_VERSION=\"$(APP_VER)\"						\
				 -DHSE_VALUE=$(HSE_CLOCK)UL 

# TARGET definition
TARGET 		= main
TARGET_ELF  = $(TARGET).elf
TARGET_SREC = $(TARGET).s19
TARGET_HEX  = $(TARGET).hex
TARGET_BIN  = $(TARGET).bin
TARGET_LSS  = $(TARGET).lss
#TARGET_DFU  = $(TARGET).dfu
TARGET_SYM  = $(TARGET).sym

# define Cortex-M4 LIBRARY PATH
FATFS		= ../STM32F407xGT6_FatFS_DISP_20120710
FWLIB  		= $(FATFS)/lib/STM32F4xx_StdPeriph_Driver
USBLIB 		= $(FATFS)/lib/STM32_USB_Device_Library
OTGLIB		= $(FATFS)/lib/STM32_USB_OTG_Driver
CM4LIB 		= $(FATFS)/lib/CMSIS
CM4_DEVICE 	= $(CM4LIB)/Device/ST/STM32F4xx
CM4_CORE	= $(CM4LIB)/Include

# include PATH
INCPATHS	 = 	./						\
				./inc					\
				$(FATFS)/inc  			\
				$(FWLIB)/inc  			\
				$(FWLIB)/inc/device_support	\
				$(FWLIB)/inc/core_support	\
				$(USBLIB)/inc			\
				$(USBLIB)/Core/inc		\
				$(USBLIB)/Class/cdc/inc	\
				$(OTGLIB)/inc			\
				$(CM4_DEVICE)/Include	\
				$(CM4_CORE)				\
				$(FREERTOS_DIR)/Source/include	\
				$(FREERTOS_DIR)/Source/portable/GCC/ARM_CM4F \
				$(FREERTOS_COM)         \
				$(FATFSFF)		
INCLUDES     = $(addprefix -I ,$(INCPATHS))

# Set library PATH
LIBPATHS     = $(FWLIB) $(USBLIB) $(CM4LIB) $(DISPLAY_LIB)
LIBRARY_DIRS = $(addprefix -L,$(LIBPATHS))
# if you use math-library, put "-lm" 
MATH_LIB	 =	-lm

# LinkerScript PATH
LINKER_PATH =  $(FATFS)/lib/linker
LINKER_DIRS = $(addprefix -L,$(LINKER_PATH)) 

# Object definition
OBJS 	 = $(CFILES:%.c=%.o) $(CPPFILES:%.cpp=%.o) 
LIBOBJS  = $(LIBCFILES:%.c=%.o) $(SFILES:%.s=%.o)

# C code PATH
SOURCE  = ./src
CFILES = \
 $(SOURCE)/$(TARGET).c				\
 $(SOURCE)/hw_config.c				\
 $(SOURCE)/rtc_support.c			\
 $(SOURCE)/uart_support.c			\
 $(SOURCE)/i2c.c					\
 $(SOURCE)/stm32f4xx_it.c			\
 $(SOURCE)/usb_bsp.c				\
 $(SOURCE)/usbd_desc.c				\
 $(SOURCE)/usbd_usr.c				\
 $(SOURCE)/usb_core.c				\
 $(SOURCE)/usbd_cdc_vcp.c			\
 $(SOURCE)/systick.c				\
 $(SOURCE)/syscalls.c				

#CPPFILES = \
# $(SOURCE)/orbit.cpp				\
# $(SOURCE)/vector3.cpp

#/*----- Display library PATH -----*/
#DISPLAY_LIB	= $(FATFS)/lib/display
#DISPLAY_INC = $(DISPLAY_LIB)/abstract/inc
#DISPLAY_SRC = $(DISPLAY_LIB)/abstract/src
#DISPLAY_BMP = $(DISPLAY_INC)/bmps
#DISPLAY_FNT = $(DISPLAY_INC)/fonts

#DISPLAY_DRV_SRC = $(DISPLAY_LIB)/drivers/src
#DISPLAY_DRV_INC = $(DISPLAY_LIB)/drivers/inc

#DISPLAY_MCU_SRC = $(DISPLAY_LIB)/mcu_depend/src
#DISPLAY_MCU_INC = $(DISPLAY_LIB)/mcu_depend/inc

#CFILES += \
# $(DISPLAY_MCU_SRC)/display_if_basis.c	\
# $(DISPLAY_SRC)/display_if_support.c	\
# $(DISPLAY_SRC)/font_if_datatable.c		\
# $(DISPLAY_SRC)/font_if.c				\
# $(DISPLAY_SRC)/ts.c
 
# Set Abstract-layer of Display Driver
#ifneq ($(USE_DISPLAY),)
#include $(DISPLAY_LIB)/display.mk
#endif

# IJG JPEG Library
#ifeq ($(USE_JPEG_LIB),USE_IJG_LIB)
#JPEGLIB = $(DISPLAY_LIB)/jpeg
#include $(JPEGLIB)/jpeglib.mk
#endif

# Chan's TINY JPEG Library
#ifeq ($(USE_JPEG_LIB),USE_TINYJPEG_LIB)
#JPEGLIB = $(DISPLAY_LIB)/tjpgd
#CFILES += \
# $(JPEGLIB)/tjpgd.c
#endif

# Display Driver Touch Sence
#ifeq ($(USE_TOUCH_SENCE),USE_ADS7843)
#CFILES += \
# $(DISPLAY_MCU_SRC)/touch_if_basis.c	\
# $(DISPLAY_SRC)/touch_if.c
#endif

#/*----- FatFs library PATH -----*/	
FATFSFF = $(FATFS)/lib/ff
CFILES += \
 $(FATFSFF)/ff.c 							\
 $(FATFSFF)/sdio_stm32f4.c 				\
 $(FATFSFF)/diskio_sdio.c 				\
 $(FATFSFF)/option/cc932.c

#/*----- STARTUP code PATH -----*/
STARTUP_DIR = $(CM4_DEVICE)/Source/Templates/gcc_ride7
ifeq ($(OS_SUPPORT),USE_FREERTOS)
SFILES += \
	$(SOURCE)/startup_stm32f4xx_rtos.s
else
SFILES += \
	$(STARTUP_DIR)/startup_stm32f4xx.s
endif



#/*----- STM32 library PATH -----*/
LIBCFILES = \
 $(FWLIB)/src/misc.c \
 $(FWLIB)/src/stm32f4xx_syscfg.c 	\
 $(FWLIB)/src/stm32f4xx_flash.c 	\
 $(FWLIB)/src/stm32f4xx_gpio.c 		\
 $(FWLIB)/src/stm32f4xx_fsmc.c 		\
 $(FWLIB)/src/stm32f4xx_rcc.c 		\
 $(FWLIB)/src/stm32f4xx_adc.c 		\
 $(FWLIB)/src/stm32f4xx_dma.c 		\
 $(FWLIB)/src/stm32f4xx_exti.c      \
 $(FWLIB)/src/stm32f4xx_tim.c 		\
 $(FWLIB)/src/stm32f4xx_rtc.c 		\
 $(FWLIB)/src/stm32f4xx_sdio.c 		\
 $(FWLIB)/src/stm32f4xx_i2c.c 		\
 $(FWLIB)/src/stm32f4xx_spi.c 		\
 $(FWLIB)/src/stm32f4xx_usart.c 	\
 $(FWLIB)/src/stm32f4xx_pwr.c 		\
  ./src/system_stm32f4xx.c			\
 \
 $(OTGLIB)/src/usb_dcd_int.c 		\
 $(OTGLIB)/src/usb_dcd.c 			\
 $(USBLIB)/Core/src/usbd_core.c 	\
 $(USBLIB)/Core/src/usbd_req.c 		\
 $(USBLIB)/Core/src/usbd_ioreq.c 	\
 $(USBLIB)/Class/cdc/src/usbd_cdc_core.c \
 \
 $(FREERTOS_DIR)/Demo/Common/Minimal/GenQTest.c 		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/BlockQ.c 		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/blocktim.c 		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/QPeek.c 			\
 $(FREERTOS_DIR)/Demo/Common/Minimal/PollQ.c 			\
 $(FREERTOS_DIR)/Source/tasks.c 				\
 $(FREERTOS_DIR)/Source/list.c 				\
 $(FREERTOS_DIR)/Source/queue.c 				\
 $(FREERTOS_DIR)/Source/portable/GCC/ARM_CM4F/port.c		\
 $(FREERTOS_DIR)/Source/portable/MemMang/heap_2.c

# $(FATFS)/src/systick.c				\

#/*----- STM32 Debug library -----*/
ifeq ($(OPTIMIZE),0)
CFILES += \
 $(FATFS)/lib/IOView/stm32f4xx_io_view.c
else
endif


# TOOLCHAIN SETTING
CC 			= $(TCHAIN)-gcc
CPP 		= $(TCHAIN)-g++
OBJCOPY 	= $(TCHAIN)-objcopy
OBJDUMP 	= $(TCHAIN)-objdump
SIZE 		= $(TCHAIN)-size
AR 			= $(TCHAIN)-ar
LD 			= $(TCHAIN)-gcc
NM 			= $(TCHAIN)-nm
REMOVE		= $(REMOVAL) -f
REMOVEDIR 	= $(REMOVAL) -rf

# C and ASM FLAGS
CFLAGS  = -MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS)
CFLAGS += -mapcs-frame -mno-sched-prolog $(USING_FPU)
CFLAGS += -std=gnu99
CFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO)
CFLAGS += -fno-strict-aliasing -fsigned-char
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-schedule-insns2
CFLAGS += --param max-inline-insns-single=1000
CFLAGS += -fno-common -fno-hosted
CFLAGS += -Wall
#CFLAGS += -Os
#CFLAGS += -Wdouble-promotion
#CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CFLAGS += $(SYNTHESIS_DEFS)  

#usb_conf.h
CFLAGS+=-DUSE_USB_OTG_FS=1

CFLAGS+=-DUSE_STDPERIPH_DRIVER

CXXFLAGS  = -MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
CXXFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS)
CXXFLAGS += -mapcs-frame -mno-sched-prolog $(USING_FPU)
#CXXFLAGS += -std=gnu99
CXXFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO)
CXXFLAGS += -fno-strict-aliasing -fsigned-char
CXXFLAGS += -ffunction-sections -fdata-sections
CXXFLAGS += -fno-schedule-insns2
CXXFLAGS += --param max-inline-insns-single=1000
CXXFLAGS += -fno-common
CXXFLAGS += -Wall
#CXXFLAGS += -Os
#CXXFLAGS += -Wdouble-promotion
#CXXFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CXXFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CXXFLAGS += $(SYNTHESIS_DEFS)  

# Linker FLAGS -mfloat-abi=softfp -msoft-float
LDFLAGS  = -mcpu=cortex-m4 -march=armv7e-m -mthumb
LDFLAGS += -u g_pfnVectors -Wl,-static -Wl,--gc-sections -nostartfiles
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += $(LIBRARY_DIRS) $(LINKER_DIRS) $(MATH_LIB)

ifeq ($(USE_EXT_SRAM),DATA_IN_ExtSRAM)
LDFLAGS +=-T$(LINKER_PATH)/$(MPU_DENSITY)_EXTRAM.ld
else
LDFLAGS +=-T$(LINKER_PATH)/$(MPU_DENSITY).ld
endif

# Object Copy and dfu generation FLAGS
OBJCPFLAGS = -O
OBJDUMPFLAGS = -h -S -C
DFU	  = hex2dfu
DFLAGS = -w
 
all: gccversion build sizeafter

# Object Size Infomations
ELFSIZE = $(SIZE) -A -x $(TARGET).elf
sizeafter:
	@$(MSGECHO) 
	@$(MSGECHO) Size After:
	$(SIZE) $(TARGET).elf
	@$(SIZE) -A -x $(TARGET).elf
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
	@$(MSGECHO) "BUILD_TYPE = "$(OS_SUPPORT)
	@$(MSGECHO) "USING_DISPLAY = "$(USE_DISPLAY)
	@$(MSGECHO) 

# Build Object
build: $(TARGET_ELF) $(TARGET_LSS) $(TARGET_SYM) $(TARGET_HEX) $(TARGET_SREC) $(TARGET_BIN)

.SUFFIXES: .o .c .s .cpp  

$(TARGET_LSS): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Disassemble: $@
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@ 
$(TARGET_SYM): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Symbol: $@
	$(NM) -n $< > $@
$(TARGET).hex: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) ihex $^ $@    
$(TARGET).s19: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) srec $^ $@ 
$(TARGET).bin: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) binary $< $@ 
$(TARGET).dfu: $(TARGET).hex
	@$(MSGECHO)
	@$(MSGECHO) Make STM32 dfu: $@
	$(DFU) $(DFLAGS) $< $@
	@$(MSGECHO)
$(TARGET).elf: $(OBJS) stm32.a
	@$(MSGECHO) Link: $@
	$(LD) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@$(MSGECHO)

stm32.a: $(LIBOBJS)
	@$(MSGECHO) Archive: $@
	$(AR) cr $@ $(LIBOBJS)    
	@$(MSGECHO)
.c.o:
	@$(MSGECHO) Compile: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.cpp.o:
	@$(MSGECHO) Compile: $<
	$(CPP) -c $(CXXFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.s.o:
	@$(MSGECHO) Assemble: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)

# Flash and Debug Program
debug :
	$(WSHELL) /c start /B $(INSIGHT) $(TARGET).elf
	$(OCD) $(OCD_CMD) -c "soft_reset_halt"
program :
	$(OCD) $(OCD_CMD) -c "mt_flash $(TARGET).elf"
#	$(OCD) $(OCD_CMD) -c "eraser"
#	$(OCD) $(OCD_CMD) -c "mt_flash_bin $(TARGET).bin 0x08000000"


# Drop files into dust-shoot
.PHONY clean:
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).obj
	$(REMOVE) stm32.a
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).s19
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).dfu
	$(REMOVE) $(wildcard *.stackdump)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LIBOBJS)
	$(REMOVE) $(LST)
	$(REMOVE) $(CFILES:.c=.lst)
	$(REMOVE) $(CFILES:.c=.d)
	$(REMOVE) $(CPPFILES:.cpp=.lst)
	$(REMOVE) $(CPPFILES:.cpp=.d)
	$(REMOVE) $(LIBCFILES:.c=.lst)
	$(REMOVE) $(LIBCFILES:.c=.d)
	$(REMOVE) $(SFILES:.s=.lst)
	$(REMOVE) $(wildcard $(FATFS)/lib/IOView/*.d)
	$(REMOVE) $(wildcard $(FATFS)/lib/IOView/*.lst)
	$(REMOVE) $(wildcard $(FATFS)/lib/IOView/*.o)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.d)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.lst)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.o)
	$(REMOVE) $(wildcard $(CM4_DEVICE)/*.d)
	$(REMOVE) $(wildcard $(CM4_DEVICE)/*.lst)
	$(REMOVEDIR) .dep
	@$(MSGECHO)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
