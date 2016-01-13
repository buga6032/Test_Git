KERNELDIR ?= /home/git/syscode/src/linux/linux-3-14-28-r0

LDFLAGS = -L$(LIBDIR) -lpthread -lusb-1.0 -lysusb

all: uartload

uartload : uartload.c
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

clean :
	rm uartload

