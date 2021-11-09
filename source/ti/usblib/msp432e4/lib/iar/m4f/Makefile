SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SDK_INSTALL_DIR)/imports.mak

CC  = "$(IAR_ARMCOMPILER)/bin/iccarm"
ASM = "$(IAR_ARMCOMPILER)/bin/iasmarm"
AR  = "$(IAR_ARMCOMPILER)/bin/iarchive"

NAME = msp432e4_usblib

OBJ_DIR = ../../../obj/iar/m4f/msp432e4_usblib
SRC_DIR = ../../..

CFLAGS = "-I$(IAR_ARMCOMPILER)/inc/c" "-I$(SDK_INSTALL_DIR)/source/third_party/CMSIS/Include" "-I$(SDK_INSTALL_DIR)/source" --cpu Cortex-M4F --fpu VFPv4_sp -D__MSP432E411Y__ --aeabi --guard_calls --endian=little --thumb --silent -e --use_unix_directory_separators --diag_suppress=Pa050,Go005 -On --debug
AFLAGS = 
ASMFLAGS = "-I$(IAR_ARMCOMPILER)/inc/c" 

OBJECTS = $(OBJ_DIR)/usbdaudio.o $(OBJ_DIR)/usbdbulk.o $(OBJ_DIR)/usbdcdc.o $(OBJ_DIR)/usbdcdesc.o $(OBJ_DIR)/usbdcomp.o $(OBJ_DIR)/usbdconfig.o $(OBJ_DIR)/usbddfu-rt.o $(OBJ_DIR)/usbdenum.o $(OBJ_DIR)/usbdhandler.o $(OBJ_DIR)/usbdhid.o $(OBJ_DIR)/usbdhidgamepad.o $(OBJ_DIR)/usbdhidkeyb.o $(OBJ_DIR)/usbdhidmouse.o $(OBJ_DIR)/usbdhidsensor.o $(OBJ_DIR)/usbdmsc.o $(OBJ_DIR)/usbhaudio.o $(OBJ_DIR)/usbhcdc.o $(OBJ_DIR)/usbhcdcserial.o $(OBJ_DIR)/usbhhid.o $(OBJ_DIR)/usbhhidkeyboard.o $(OBJ_DIR)/usbhhidmouse.o $(OBJ_DIR)/usbhhub.o $(OBJ_DIR)/usbhmsc.o $(OBJ_DIR)/usbhostenum.o $(OBJ_DIR)/usbhscsi.o $(OBJ_DIR)/usbbuffer.o $(OBJ_DIR)/usbdesc.o $(OBJ_DIR)/usbdma.o $(OBJ_DIR)/usbkeyboardmap.o $(OBJ_DIR)/usbmode.o $(OBJ_DIR)/usbringbuf.o $(OBJ_DIR)/usbtick.o $(OBJ_DIR)/usbulpi.o

all: $(NAME).a

$(NAME).a: $(OBJECTS)
	@ echo Archiving $@
	@ $(AR) $(AFLAGS) $@ $^

$(OBJ_DIR)/usbdaudio.o: $(SRC_DIR)/device/usbdaudio.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdbulk.o: $(SRC_DIR)/device/usbdbulk.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdcdc.o: $(SRC_DIR)/device/usbdcdc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdcdesc.o: $(SRC_DIR)/device/usbdcdesc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdcomp.o: $(SRC_DIR)/device/usbdcomp.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdconfig.o: $(SRC_DIR)/device/usbdconfig.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbddfu-rt.o: $(SRC_DIR)/device/usbddfu-rt.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdenum.o: $(SRC_DIR)/device/usbdenum.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhandler.o: $(SRC_DIR)/device/usbdhandler.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhid.o: $(SRC_DIR)/device/usbdhid.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhidgamepad.o: $(SRC_DIR)/device/usbdhidgamepad.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhidkeyb.o: $(SRC_DIR)/device/usbdhidkeyb.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhidmouse.o: $(SRC_DIR)/device/usbdhidmouse.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdhidsensor.o: $(SRC_DIR)/device/usbdhidsensor.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdmsc.o: $(SRC_DIR)/device/usbdmsc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhaudio.o: $(SRC_DIR)/host/usbhaudio.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhcdc.o: $(SRC_DIR)/host/usbhcdc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhcdcserial.o: $(SRC_DIR)/host/usbhcdcserial.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhhid.o: $(SRC_DIR)/host/usbhhid.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhhidkeyboard.o: $(SRC_DIR)/host/usbhhidkeyboard.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhhidmouse.o: $(SRC_DIR)/host/usbhhidmouse.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhhub.o: $(SRC_DIR)/host/usbhhub.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhmsc.o: $(SRC_DIR)/host/usbhmsc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhostenum.o: $(SRC_DIR)/host/usbhostenum.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbhscsi.o: $(SRC_DIR)/host/usbhscsi.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbbuffer.o: $(SRC_DIR)/usbbuffer.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdesc.o: $(SRC_DIR)/usbdesc.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbdma.o: $(SRC_DIR)/usbdma.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbkeyboardmap.o: $(SRC_DIR)/usbkeyboardmap.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbmode.o: $(SRC_DIR)/usbmode.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbringbuf.o: $(SRC_DIR)/usbringbuf.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbtick.o: $(SRC_DIR)/usbtick.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/usbulpi.o: $(SRC_DIR)/usbulpi.c
	@ echo Building $@
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $< -o $@

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).a > $(DEVNULL) 2>&1
