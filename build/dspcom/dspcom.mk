USBLIB_TARGET_HOST = arm-fsl-linux-gnueabi

USB_SRC_DIR    = $(SRC_DIR)/dspcom
#WORKDIR        = $(USB_SRC_DIR)/ARM
LIBUSB_DIR      = $(USB_SRC_DIR)/libusb-1.0.19
LIBYSUSB_DIR    = $(USB_SRC_DIR)/libysusb
M_LOAD_APP_DIR  = $(USB_SRC_DIR)/mloadapp
UART_LOAD_DIR   = $(USB_SRC_DIR)/uartload

LIBUSB_CONFIGURE_OPTS = --host=$(USBLIB_TARGET_HOST) \
                        --disable-udev        \
                        CFLAGS=""             \
                        LDFLAGS=""

#ifeq ($(PRODUCT_NAME),N1)

DSP_RELEASE_DIR = /home/git/release/astcom/dsp/1.0.0/arm-fsl-linux-gnueabi

ARM_LIB_DIR = $(DSP_RELEASE_DIR)/lib
ARM_BIN_DIR = $(DSP_RELEASE_DIR)/bin

usb_all : libusb libysusb loadapp
#else
#ifeq ($(PRODUCT_NAME),N412)
#ARM_LIB_DIR = $(USB_SRC_DIR)/ARM/lib_n412
#ARM_BIN_DIR = $(USB_SRC_DIR)/ARM/bin_n412

#usb_all : libusb libysusb uartload
#endif
#endif

$(LIBUSB_DIR)/.configured:
	cd $(LIBUSB_DIR); chmod u+x ./configure; ./configure $(LIBUSB_CONFIGURE_OPTS)
	touch $(LIBUSB_DIR)/.configured

libusb: $(LIBUSB_DIR)/.configured
	$(MAKE) -C $(LIBUSB_DIR)
	@cp -dfv $(LIBUSB_DIR)/libusb/.libs/libusb-1.0.so* $(ARM_LIB_DIR)/

libusb_clean:
	cd $(LIBUSB_DIR); make clean
	rm $(LIBUSB_DIR)/.configured

#ifeq ($(PRODUCT_NAME),N1)
#libysusb:
#	@cp $(LIBUSB_DIR)/libusb/libusb.h $(WORKDIR)/include/libusb
#	@cp $(LIBYSUSB_DIR)/usbsrc.c-n1 $(LIBYSUSB_DIR)/usbsrc.c
#	cd $(LIBYSUSB_DIR); make -f Makefilelibso CC=$(USBLIB_TARGET_HOST)-gcc WORKDIR=$(WORKDIR) LIBUSB_DIR=$(LIBUSB_DIR)
#	@cp $(LIBYSUSB_DIR)/libysusb.so $(ARM_LIB_DIR)/
#else
#ifeq ($(PRODUCT_NAME),N412)
libysusb:
	@cp $(LIBUSB_DIR)/libusb/libusb.h $(DSP_RELEASE_DIR)/staging/usr/include/libusb
	@cp $(LIBYSUSB_DIR)/usbsrc.c-n412 $(LIBYSUSB_DIR)/usbsrc.c
	cd $(LIBYSUSB_DIR); make -f Makefilelibso CC=$(USBLIB_TARGET_HOST)-gcc STAGING_DIR=$(DSP_RELEASE_DIR)/staging/usr/include/libusb LIBUSB_DIR=$(LIBUSB_DIR)
	@cp $(LIBYSUSB_DIR)/libysusb.so $(ARM_LIB_DIR)/
#endif
#endif

libysusb_clean:
	cd $(LIBYSUSB_DIR); make clean -f Makefilelibso

loadapp:
	$(MAKE) -C $(M_LOAD_APP_DIR) CC=$(USBLIB_TARGET_HOST)-gcc LIBDIR=$(ARM_LIB_DIR) INCDIR=$(DSP_RELEASE_DIR)/staging/usr/include/libusb
	@cp $(M_LOAD_APP_DIR)/m-load-app $(ARM_BIN_DIR)/

loadapp_clean:
	rm $(M_LOAD_APP_DIR)/m-load-app

uartload:
	$(MAKE) -C $(UART_LOAD_DIR) CC=$(USBLIB_TARGET_HOST)-gcc LIBDIR=$(ARM_LIB_DIR) INCDIR=$(DSP_RELEASE_DIR)/staging/usr/include/libusb
	@cp $(UART_LOAD_DIR)/uartload $(ARM_BIN_DIR)/

uartload_clean:
	rm $(UART_LOAD_DIR)/uartload

#ifeq ($(PRODUCT_NAME),N1)
#usb_clean: libusb_clean libysusb_clean loadapp_clean
#	rm $(ARM_LIB_DIR)/libusb-1.0.so*
#	rm $(ARM_LIB_DIR)/libysusb.so
#	rm $(ARM_BIN_DIR)/m-load-app
#else
#ifeq ($(PRODUCT_NAME),N412)
usb_clean: libusb_clean libysusb_clean uartload_clean
	rm $(ARM_LIB_DIR)/libusb-1.0.so*
	rm $(ARM_LIB_DIR)/libysusb.so
	rm $(ARM_BIN_DIR)/uartload
#endif
#endif
