__DEBUG = 

TARGET = nc1020
OBJS = main.o nc1020.o

INCDIR = 
BUILD_PRX = 1
ifdef __DEBUG
CFLAGS = -g
else
CFLAGS = -O2
endif
CFLAGS += -G0 -Wall
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)

LIBDIR =
LDFLAGS =

EXTRA_TARGETS = EBOOT.PBP
PSP_EBOOT_TITLE = NC1020 Emulator

PSPSDK=$(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak
