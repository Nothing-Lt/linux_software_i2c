
obj-m += sw_iic.o sw_iic_dev.o
sw_iic-objs := soft_iic.o lib_soft_iic.o
sw_iic_dev-objs := soft_iic_dev.o
KERNEL_MAKEFILE_PLACE = ~/ti-linux-kernel-dev/KERNEL

# KERNEL_MAKEFILE_PLACE is the place for makefile after compiling the kernel,this can be 

# The following 2 steps is for preventing the probnlem during compile the kernel and kernel module.
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

all:
	make -C $(KERNEL_MAKEFILE_PLACE) M=$(PWD) modules

clean:
	make -C $(KERNEL_MAKEFILE_PLACE) M=$(PWD) clean
