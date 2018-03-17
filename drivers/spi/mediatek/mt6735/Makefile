subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif

obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o
obj-$(CONFIG_FINGERPRINT_SUPPORT) += fingerprint_core.o
obj-$(CONFIG_FINGERPRINT_SYNAPTICS) += synaptics/vfsspidrv.o
obj-$(CONFIG_FINGERPRINT_GF3208) += gf3208/gf-milanf.o  gf3208/gf-common.o
