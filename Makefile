ifneq ($(KERNELRELEASE),)

obj-m := mygpio.o 

else
	
KDIR := /home/leo/labs/lab8/uClinux-dist/linux-2.6.x/

all:
	make -C $(KDIR) M=$(PWD) modules ARCH=m68knommu CROSS_COMPILE=m68k-uclinux-
	cp ./mygpio.ko /images
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers  modul*

endif
