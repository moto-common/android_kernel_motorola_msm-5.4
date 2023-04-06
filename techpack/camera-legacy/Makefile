# SPDX-License-Identifier: GPL-2.0-only
ifneq (,$(filter $(CONFIG_CORFUQ_DTB) $(CONFIG_CORFUR_DTB) $(CONFIG_CORFUP_DTB),y))

# auto-detect subdirs
ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_ARCH_KONA), y)
include $(srctree)/techpack/camera-legacy/config/konacamera.conf
endif

ifeq ($(CONFIG_ARCH_LITO), y)
include $(srctree)/techpack/camera-legacy/config/litocamera.conf
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
include $(srctree)/techpack/camera-legacy/config/bengalcamera.conf
endif

ifeq ($(CONFIG_ARCH_MONACO), y)
include $(srctree)/techpack/camera-legacy/config/monacocamera.conf
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
include $(srctree)/techpack/camera-legacy/config/lahainacamera.conf
endif

ifeq ($(CONFIG_ARCH_HOLI), y)
include $(srctree)/techpack/camera-legacy/config/holicamera.conf
endif

ifeq ($(CONFIG_ARCH_BLAIR), y)
include $(srctree)/techpack/camera-legacy/config/blaircamera.conf
endif

ifeq ($(CONFIG_ARCH_SHIMA), y)
include $(srctree)/techpack/camera-legacy/config/shimacamera.conf
endif

ifeq ($(CONFIG_ARCH_YUPIK), y)
include $(srctree)/techpack/camera-legacy/config/yupikcamera.conf
endif

ifeq ($(CONFIG_ARCH_KONA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/konacameraconf.h
endif

ifeq ($(CONFIG_ARCH_LITO), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/litocameraconf.h
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/bengalcameraconf.h
endif

ifeq ($(CONFIG_ARCH_MONACO), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/monacocameraconf.h
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/lahainacameraconf.h
endif

ifeq ($(CONFIG_ARCH_HOLI), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/holicameraconf.h
endif

ifeq ($(CONFIG_ARCH_BLAIR), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/blaircameraconf.h
endif

ifeq ($(CONFIG_ARCH_SHIMA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/shimacameraconf.h
endif

ifeq ($(CONFIG_ARCH_YUPIK), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera-legacy/config/yupikcameraconf.h
endif

endif

ifneq (,$(filter $(CONFIG_SPECTRA_CAMERA), y m))
# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/camera-legacy/include/uapi

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/camera-legacy/include/uapi \
                -I$(srctree)/techpack/camera-legacy/include
obj-y += drivers/
else
$(info Target not found)
endif

endif
