#****************************************************************************************
#|  Description: Makefile for GNU ARM Embedded toolchain.
#|    File Name: makefile
#|
#|---------------------------------------------------------------------------------------
#|                          C O P Y R I G H T
#|---------------------------------------------------------------------------------------
#|   Copyright (c) 2022 by Feaser     www.feaser.com     All rights reserved
#|
#|---------------------------------------------------------------------------------------
#|                            L I C E N S E
#|---------------------------------------------------------------------------------------
#| Permission is hereby granted, free of charge, to any person obtaining a copy
#| of this software and associated documentation files (the "Software"), to deal
#| in the Software without restriction, including without limitation the rights
#| to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#| copies of the Software, and to permit persons to whom the Software is
#| furnished to do so, subject to the following conditions:
#| 
#| The above copyright notice and this permission notice shall be included in all
#| copies or substantial portions of the Software.
#| 
#| THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#| IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#| FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#| AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#| LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#| OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#| SOFTWARE.
#|
#****************************************************************************************
SHELL = sh

#|--------------------------------------------------------------------------------------|
#| Configure project name                                                               |
#|--------------------------------------------------------------------------------------|
PROJ_NAME=microtbx_demo


#|--------------------------------------------------------------------------------------|
#| Configure tool path                                                                  |
#|--------------------------------------------------------------------------------------|
# Configure the path to where the arm-none-eabi-gcc program is located. If the program
# is available on the path, then the TOOL_PATH variable can be left empty.
# Make sure to add a fordward slash at the end. 
#
# Note that on Windows it should be in the 8.3 short pathname format with forward
# slashes. Example:
#   TOOL_PATH=C:/PROGRA~2/GNUTOO~1/82018-~1/bin/
# To obtain the pathname in the 8.3 format, open the directory in the Windows command 
# prompt and run the following command: 
#   cmd /c for %A in ("%cd%") do @echo %~sA 
#
# You can also override the value directly on the command-line when calling make. For
# example to set it to empty, in case arm-none-eabi-gcc is available on the path:
#   make TOOL_PATH= clean all
TOOL_PATH=/opt/gcc-arm-none-eabi-8-2018-q4-major/bin/


#|--------------------------------------------------------------------------------------|
#| Collect project files                                                                |
#|--------------------------------------------------------------------------------------|
# Collect demo application sources
PROJ_FILES = ../../../sortlist.c
# Collect board specific sources
PROJ_FILES += $(wildcard ../../*.c)
PROJ_FILES += $(wildcard ../../*.h)
# Collect GCC specific sources
PROJ_FILES += $(wildcard ../*.c)
PROJ_FILES += $(wildcard ../*.h)
PROJ_FILES += $(wildcard ../*.s)
# Collect MicroTBX sources
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/*.c)
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/*.h)
# Collect MicroTBX port and compiler specific sources
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/port/ARM_CORTEXM/*.c)
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/port/ARM_CORTEXM/*.h)
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/port/ARM_CORTEXM/GCC/*.c)
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/port/ARM_CORTEXM/GCC/*.h)
PROJ_FILES += $(wildcard ../../../../../third_party/microtbx/source/port/ARM_CORTEXM/GCC/*.s)
# Collect CMSIS sources
PROJ_FILES += $(wildcard ../../../../../third_party/cmsis/core/CMSIS/Core/Include/*.h)
PROJ_FILES += $(wildcard ../../../../../third_party/cmsis/device/st/f0/Include/*.h)
# Collect HAL sources
PROJ_FILES += $(wildcard ../../../../../third_party/hal/st/f0/Inc/*.h)
PROJ_FILES += $(wildcard ../../../../../third_party/hal/st/f0/Src/*.c)
PROJ_FILES := $(filter-out $(wildcard ../../../../../third_party/hal/st/f0/Src/*_template*.c), $(PROJ_FILES))


#|--------------------------------------------------------------------------------------|
#| Toolchain binaries                                                                   |
#|--------------------------------------------------------------------------------------|
RM = rm
CC = $(TOOL_PATH)arm-none-eabi-gcc
LN = $(TOOL_PATH)arm-none-eabi-gcc
OC = $(TOOL_PATH)arm-none-eabi-objcopy
OD = $(TOOL_PATH)arm-none-eabi-objdump
AS = $(TOOL_PATH)arm-none-eabi-gcc
SZ = $(TOOL_PATH)arm-none-eabi-size


#|--------------------------------------------------------------------------------------|
#| Filter project files                                                                 |
#|--------------------------------------------------------------------------------------|
PROJ_ASRCS  = $(filter %.s,$(foreach file,$(PROJ_FILES),$(notdir $(file))))
PROJ_CSRCS  = $(filter %.c,$(foreach file,$(PROJ_FILES),$(notdir $(file))))


#|--------------------------------------------------------------------------------------|
#| Set important path variables                                                         |
#|--------------------------------------------------------------------------------------|
VPATH     = $(foreach path,$(sort $(foreach file,$(PROJ_FILES),$(dir $(file)))) $(subst \,/,$(OBJ_PATH)),$(path) :)
OBJ_PATH  = build
BIN_PATH  = build
INC_PATH  = $(patsubst %/,%,$(patsubst %,-I%,$(sort $(foreach file,$(filter %.h,$(PROJ_FILES)),$(dir $(file))))))
LIB_PATH  = 


#|--------------------------------------------------------------------------------------|
#| Options for toolchain binaries                                                       |
#|--------------------------------------------------------------------------------------|
STDFLAGS  = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
STDFLAGS += -fdata-sections -ffunction-sections -Wall -g3
OPTFLAGS  = -O0
DEPFLAGS  = -MT $@ -MMD -MP -MF $(OBJ_PATH)/$*.d
CFLAGS    = $(STDFLAGS) $(OPTFLAGS)
CFLAGS   += -DSTM32F091xC -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
CFLAGS   += $(INC_PATH)
AFLAGS    = $(CFLAGS)
LFLAGS    = $(STDFLAGS) $(OPTFLAGS)
LFLAGS   += -Wl,-script="../STM32F091RC_FLASH.ld" -Wl,-Map=$(BIN_PATH)/$(PROJ_NAME).map
LFLAGS   += -specs=nano.specs -Wl,--gc-sections $(LIB_PATH)
OFLAGS    = -O srec
ODFLAGS   = -x
SZFLAGS   = -B -d
RMFLAGS   = -f


#|--------------------------------------------------------------------------------------|
#| Specify library files                                                                |
#|--------------------------------------------------------------------------------------|
LIBS = 


#|--------------------------------------------------------------------------------------|
#| Define targets                                                                       |
#|--------------------------------------------------------------------------------------|
AOBJS = $(patsubst %.s,%.o,$(PROJ_ASRCS))
COBJS = $(patsubst %.c,%.o,$(PROJ_CSRCS))


#|--------------------------------------------------------------------------------------|
#| Make ALL                                                                             |
#|--------------------------------------------------------------------------------------|
.PHONY: all
all: $(BIN_PATH)/$(PROJ_NAME).srec

$(BIN_PATH)/$(PROJ_NAME).srec : $(BIN_PATH)/$(PROJ_NAME).elf
	@$(OC) $< $(OFLAGS) $@
	@$(OD) $(ODFLAGS) $< > $(BIN_PATH)/$(PROJ_NAME).map
	@echo +++ Summary of memory consumption:
	@$(SZ) $(SZFLAGS) $<
	@echo +++ Build complete [$(notdir $@)]

$(BIN_PATH)/$(PROJ_NAME).elf : $(AOBJS) $(COBJS) 
	@echo +++ Linking [$(notdir $@)]
	@$(LN) $(LFLAGS) -o $@ $(patsubst %.o,$(OBJ_PATH)/%.o,$(^F)) $(LIBS)


#|--------------------------------------------------------------------------------------|
#| Compile and assemble                                                                 |
#|--------------------------------------------------------------------------------------|
$(AOBJS): %.o: %.s
	@echo +++ Assembling [$(notdir $<)]
	@$(AS) $(AFLAGS) -c $< -o $(OBJ_PATH)/$(@F)

$(COBJS): %.o: %.c $(OBJ_PATH)/%.d
	@echo +++ Compiling [$(notdir $<)]
	@$(CC) $(DEPFLAGS) $(CFLAGS) -c $< -o $(OBJ_PATH)/$(@F)


#|--------------------------------------------------------------------------------------|
#| Make CLEAN                                                                           |
#|--------------------------------------------------------------------------------------|
.PHONY: clean
clean: 
	@echo +++ Cleaning build environment
	@$(RM) $(RMFLAGS) $(foreach file,$(AOBJS),$(OBJ_PATH)/$(file))
	@$(RM) $(RMFLAGS) $(foreach file,$(COBJS),$(OBJ_PATH)/$(file))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.lst,$(foreach file,$(AOBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.lst,$(foreach file,$(COBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.d,$(foreach file,$(COBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(BIN_PATH)/$(PROJ_NAME).elf $(BIN_PATH)/$(PROJ_NAME).map
	@$(RM) $(RMFLAGS) $(BIN_PATH)/$(PROJ_NAME).srec
	@echo +++ Clean complete
	
#|--------------------------------------------------------------------------------------|
#| Dependency generation                                                                |
#|--------------------------------------------------------------------------------------|
DEPFILES := $(PROJ_CSRCS:%.c=$(OBJ_PATH)/%.d)
$(DEPFILES):
include $(wildcard $(DEPFILES))	

