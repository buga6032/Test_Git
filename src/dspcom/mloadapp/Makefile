#KERNELDIR ?=/home/ls_syscode/src/linux/kernel3.10-sti-KERNEL_15.0_3.10.27_18.2-1
#KERNELDIR ?= /home/ls_syscode/src/linux/linux-3.2.0-psp05.06.00.00.am335x
KERNELDIR ?= /home/git/syscode/src/linux/linux-3-14-28-r0

PWD := $(shell pwd)

LDFLAGS = -L$(LIBDIR) -lusb-1.0 -lysusb -lpthread

all: m-load-app 

m-load-app : m-load-app.c
	$(CC)  -o $@ $^ -I$(INCDIR) $(LDFLAGS)

clean :
	rm m-load-app
