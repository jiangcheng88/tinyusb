include ../../../tools/top.mk
include ../../make.mk

INC += \
	src \
	$(TOP)/hw \

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

<<<<<<< HEAD
# CFLAGS += -Wno-error=cast-align
=======
# TODO: suppress warning caused by host stack
CFLAGS += -Wno-error=cast-align -Wno-error=null-dereference
>>>>>>> edd8eb3279c2440e9d4590312f2104e58beafe12

# TinyUSB Host Stack source
SRC_C += \
	src/class/cdc/cdc_host.c \
	src/class/hid/hid_host.c \
	src/class/msc/msc_host.c \
	src/host/hub.c \
	src/host/usbh.c \
	src/portable/ohci/ohci.c \
	src/portable/nxp/lpc17_40/hcd_lpc17_40.c

include ../../rules.mk
