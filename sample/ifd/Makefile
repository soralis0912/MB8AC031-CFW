# The GV-M2TV card slot as a PC/SC reader (macOS, and Linux's pcscd).
# README.md says what each piece is and how to install the bundle.
#
#   make            the bundle, card_tool, pcsc_atr, and runs the T=1 test
#   make test       only the T=1 and presence tests (no libusb, no hardware)
#
# macOS: libusb-1.0 is linked statically into the bundle, so the installed
# driver does not depend on where Homebrew keeps its libraries.
# Linux: the bundle holds Contents/Linux/libifd_m2tv.so, linked against the
# system's libusb-1.0 (apt install libusb-1.0-0-dev libpcsclite-dev).
CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -pedantic
PKG     ?= pkg-config
UNAME   := $(shell uname -s)

ifeq ($(UNAME),Linux)

USB_CFLAGS := $(shell $(PKG) --cflags libusb-1.0 2>/dev/null)
USB_LIBS   := $(shell $(PKG) --libs libusb-1.0 2>/dev/null)
PCSC_CFLAGS := $(shell $(PKG) --cflags libpcsclite 2>/dev/null)
PCSC_LIBS  := $(shell $(PKG) --libs libpcsclite 2>/dev/null)
USB_DEFS := -DCHIPIO_HAVE_LIBUSB -D_POSIX_C_SOURCE=200809L

BUNDLE  := ifd-cfw-m2tv.bundle
SO      := $(BUNDLE)/Contents/Linux/libifd_m2tv.so
HEADERS := card_link.h t1.h presence.h ifd_api.h ../chipio.h ../frq1.h \
           ../../src/shared/armipc.h ../../src/shared/armcore.h

.PHONY: all test clean
all: $(SO) card_tool pcsc_atr test

# The same Info.plist; pcscd loads the file CFBundleExecutable names.
$(SO): ifd_m2tv.c Info.plist $(HEADERS)
	@test -n "$(USB_LIBS)" || { echo "libusb-1.0 not found (apt install libusb-1.0-0-dev)" >&2; false; }
	mkdir -p $(BUNDLE)/Contents/Linux
	sed 's/libifd_m2tv\.dylib/libifd_m2tv.so/' Info.plist > $(BUNDLE)/Contents/Info.plist
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -fPIC -shared -pthread \
	    -o $@ ifd_m2tv.c $(USB_LIBS) -ldl

card_tool: card_tool.c card_link.h t1.h ../chipio.h ../frq1.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -o $@ card_tool.c $(USB_LIBS)

pcsc_atr: pcsc_atr.c
	@test -n "$(PCSC_LIBS)" || { echo "libpcsclite not found (apt install libpcsclite-dev)" >&2; false; }
	$(CC) $(CFLAGS) $(PCSC_CFLAGS) -o $@ pcsc_atr.c $(PCSC_LIBS)

else

USB_CFLAGS := $(shell $(PKG) --cflags libusb-1.0 2>/dev/null)
USB_LIBDIR := $(shell $(PKG) --variable=libdir libusb-1.0 2>/dev/null)
USB_STATIC := $(USB_LIBDIR)/libusb-1.0.a
USB_FRAMEWORKS := -framework IOKit -framework CoreFoundation -framework Security
USB_DEFS := -DCHIPIO_HAVE_LIBUSB -D_POSIX_C_SOURCE=200809L -D_DARWIN_C_SOURCE

BUNDLE  := ifd-cfw-m2tv.bundle
DYLIB   := $(BUNDLE)/Contents/MacOS/libifd_m2tv.dylib
HEADERS := card_link.h t1.h presence.h ifd_api.h ../chipio.h ../frq1.h \
           ../../src/shared/armipc.h ../../src/shared/armcore.h

.PHONY: all test clean
all: $(DYLIB) card_tool pcsc_atr test

$(DYLIB): ifd_m2tv.c Info.plist $(HEADERS)
	@test -f "$(USB_STATIC)" || { echo "libusb-1.0.a not found (brew install libusb)" >&2; false; }
	mkdir -p $(BUNDLE)/Contents/MacOS
	cp Info.plist $(BUNDLE)/Contents/Info.plist
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -dynamiclib \
	    -install_name /usr/local/libexec/SmartCardServices/drivers/$(DYLIB) \
	    -o $@ ifd_m2tv.c $(USB_STATIC) $(USB_FRAMEWORKS)
	codesign --force --sign - $(BUNDLE)

card_tool: card_tool.c card_link.h t1.h ../chipio.h ../frq1.h
	$(CC) $(CFLAGS) $(USB_DEFS) $(USB_CFLAGS) -o $@ card_tool.c \
	    $(USB_STATIC) $(USB_FRAMEWORKS)

pcsc_atr: pcsc_atr.c
	$(CC) $(CFLAGS) -o $@ pcsc_atr.c -framework PCSC

endif

test_t1: test_t1.c t1.h
	$(CC) $(CFLAGS) -o $@ test_t1.c

test_presence: test_presence.c presence.h
	$(CC) $(CFLAGS) -o $@ test_presence.c

test: test_t1 test_presence
	./test_t1
	./test_presence

clean:
	rm -rf $(BUNDLE) card_tool pcsc_atr test_t1 test_presence
