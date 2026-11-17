# The four sample programs: frc1_load puts the firmware on the chip,
# ts_grab asks it for television, arm_probe and arm_ipc talk to the ARM core.  Any C11
# compiler, plus libusb-1.0 for the parts that touch hardware.
#
# libusb is found with pkg-config.  Without it, frc1_load still builds and
# does everything that does not need a device (info / verify / fields / stub
# / plan), which is what ../tests/test_frc1.py compiles; ts_grab, arm_probe
# and arm_ipc are nothing but transport, so they are skipped.
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -pedantic
PKG     ?= pkg-config

USB_CFLAGS := $(shell $(PKG) --cflags libusb-1.0 2>/dev/null)
USB_LIBS   := $(shell $(PKG) --libs libusb-1.0 2>/dev/null)

# nanosleep, for the retry backoff the device needs.  ts_grab also needs
# -pthread: on dione it reads the stream on a second thread.
USB_DEFS := -DCHIPIO_HAVE_LIBUSB -D_POSIX_C_SOURCE=200809L

# ../build/frc1-<id>.frc is what `python ../host/build.py frc1` writes, one per
# device.  Override on the command line: make demo FRC=../build/frc1-dt295.frc
FRC ?= ../build/frc1-dt295.frc

ifeq ($(strip $(USB_LIBS)),)

all: frc1_load
	@echo
	@echo "libusb-1.0 was not found, so this is the offline build:"
	@echo "  frc1_load   info / verify / fields / stub / plan"
	@echo "  ts_grab     not built -- it is nothing but transport"
	@echo "Install libusb-1.0 (brew install libusb, apt install libusb-1.0-0-dev)"
	@echo "and run make again to get install / probe / scan / save / peek."

frc1_load: frc1_load.c frc1.h chipio.h
	$(CC) $(CFLAGS) -o $@ frc1_load.c

ts_grab arm_probe arm_ipc:
	@echo "$@ needs libusb-1.0; nothing to build without it." >&2
	@false

else

all: frc1_load ts_grab arm_probe arm_ipc

frc1_load: frc1_load.c frc1.h chipio.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -o $@ frc1_load.c $(USB_LIBS)

ts_grab: ts_grab.c frq1.h chipio.h fe_dt295.h fe_m2tv.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -pthread -o $@ ts_grab.c \
	    $(USB_LIBS)

arm_probe: arm_probe.c frq1.h chipio.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -o $@ arm_probe.c $(USB_LIBS)

# The work queue's wire is shared with the firmware: ../src/shared/armipc.h.
arm_ipc: arm_ipc.c frq1.h chipio.h ../src/shared/armipc.h ../src/shared/armcore.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -I../src/shared -o $@ arm_ipc.c $(USB_LIBS)

endif

# What the container says and what installing it would do.  Writes nothing.
.PHONY: all demo clean
demo: frc1_load
	./frc1_load info $(FRC)
	@echo
	./frc1_load plan $(FRC) 0

clean:
	rm -f frc1_load ts_grab arm_probe arm_ipc
