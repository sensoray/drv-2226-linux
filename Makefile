KERNEL_SRC= /lib/modules/$(shell uname -r)/build

ifeq ($(shell uname -m),x86_64)
LIBBITS = _64
else
LIBBITS = _32
endif

ifeq ($(KERNELRELEASE),)
SUBDIR = $(shell pwd)
else 
SUBDIR = $(SUBDIRS)
endif

driver: modules
all: modules demo demo_noovl

obj-m := s2226.o

s2226-objs = s2226drv.o h51set.o

CFLAGS_DEMO = -Wall -Wcast-align -Wcast-qual  -D_LINUX -DSENSORAY_OS_LINUX -Iovlgen
#add -g for debug binaries
#EXTRA_CFLAGS += -Wall -O2 -D_LINUX -DSENSORAY_OS_LINUX -DDRIVER_BUILD -DOS_LINUX 

# if using V4L comment out line above and uncomment line below
EXTRA_CFLAGS += -Wall -O2 -D_LINUX -DOSTYPE_LINUX -DDRIVER_BUILD -DOS_LINUX -DCONFIG_S2226_V4L
#EXTRA_CFLAGS += -fno-aggressive-loop-optimizations


modules:
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(SUBDIR) $@

modules_install: 
	mkdir -p /lib/modules/$(shell uname -r)/extra
	install s2226.ko /lib/modules/$(shell uname -r)/extra
	depmod -a

clean:
	rm -rf *.o s2226.ko

load:
	sudo /sbin/insmod s2226.ko

unload:
	sudo /sbin/rmmod s2226

ovlmid.o:	ovlmid.c
	gcc $(CFLAGS_DEMO) -c ovlmid.c        

2226demo.o:	2226demo.c
	gcc $(CFLAGS_DEMO) -c 2226demo.c        

2226demo32.o:	2226demo.c
	gcc -m32 -o 2226demo32.o $(CFLAGS_DEMO) -c 2226demo.c        

image.o:	image.c
	gcc $(CFLAGS_DEMO) -c image.c        

2226demo_noovl.o:	2226demo.c
	gcc $(CFLAGS_DEMO) -DNO_OVERLAYSNAP -c 2226demo.c -o 2226demo_noovl.o


demo:	lib2226$(LIBBITS).a 2226demo.o
	gcc -o 2226demo 2226demo.o -L./  -lpthread  -l2226$(LIBBITS) -lm

demo_noovl:	2226demo_noovl.o
	gcc -o 2226demo_noovl 2226demo_noovl.o  -lpthread -lm

ifeq ($(shell uname -m),x86_64)
demo32:	lib2226_32.a 2226demo32.o
	gcc -m32 -o 2226demo32 2226demo32.o -L./  -lpthread  -l2226_32 -lm
endif
