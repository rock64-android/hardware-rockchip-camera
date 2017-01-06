#
# RockChip Camera HAL 
#
LOCAL_PATH:= $(call my-dir)
include $(call all-subdir-makefiles)

include $(CLEAR_VARS)
LOCAL_MODULE := libMFDenoise
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \> 7.0)))
LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
else
LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
endif
else
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \> 7.0)))
LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
else
LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
endif
endif
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libcameragl
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \> 7.0)))
LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
else
LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
endif
else
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \> 7.0)))
LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)_android7.1$(LOCAL_MODULE_SUFFIX)
else
LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
endif
endif
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := libopencv_java3
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
else
LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
endif
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES:=\
	CameraHalUtil.cpp\
	MessageQueue.cpp\
	Semaphore.cpp\
	CameraHal_Module.cpp\
	CameraHal_Mem.cpp\
	CameraBuffer.cpp\
	AppMsgNotifier.cpp\
	DisplayAdapter.cpp\
	CameraAdapter.cpp\
	CameraSocAdapter.cpp\
	CameraUSBAdapter.cpp\
	CameraIspAdapter.cpp\
	CameraIspSOCAdapter.cpp\
	FakeCameraAdapter.cpp\
	CameraHal.cpp\
	CameraHal_board_xml_parse.cpp\
	CameraHal_Tracer.c\
	CameraIspTunning.cpp \
	SensorListener.cpp\

ifeq ($(strip $(BOARD_USE_DRM)), true)
LOCAL_SRC_FILES += \
	camera_mem_gralloc.cpp\
	camera_mem.cpp
endif	
	
  
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk30board)	 
LOCAL_C_INCLUDES += \
	frameworks/base/include/ui \
	frameworks/av/include \
	frameworks/native/include \
	frameworks/native/include/media/hardware \
	frameworks/native/include/media/openmax \
	external/libjpeg-turbo \
	external/jpeg \
	external/jhead \
	hardware/rockchip/hwcomposer \
	hardware/rockchip/libgralloc_ump/ump/include \
	hardware/rockchip/librkvpu \
	hardware/rockchip/libgralloc \
	hardware/libhardware/include \
	$(LOCAL_PATH)/../SiliconImage/include \
	$(LOCAL_PATH)/../SiliconImage/include/isp_cam_api \
	bionic \
	external/tinyxml2 \
	system/media/camera/include \
	system/core/libion/include/ion \
	system/core/libion/kernel-headers/linux

#has no "external/stlport" from Android 6.0 on                         
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \< 6.0)))
LOCAL_C_INCLUDES += \
    external/stlport/stlport
endif

LOCAL_C_INCLUDES += \
    external/skia/include/core \
    external/skia/include/effects \
    external/skia/include/images \
    external/skia/src/ports \
    external/skia/include/utils \
    external/expat/lib

LOCAL_SHARED_LIBRARIES:= \
    libui \
    libbinder \
    libutils \
    libcutils \
    libcamera_client \
    libgui\
    libjpeg\
    libjpeghwenc\
    libion\
    libvpu\
    libdl\
    libisp_silicomimageisp_api \
    libexpat \
    libskia \
    libhardware \
    libcameragl \
    libopencv_java3 \
    libMFDenoise
#has no "external/stlport" from Android 6.0 on                         
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \< 6.0)))
LOCAL_SHARED_LIBRARIES += \
    libstlport
endif

#LOCAL_STATIC_LIBRARIES :=  libisp_calibdb libtinyxml2 libisp_cam_calibdb libisp_ebase \
#							libisp_oslayer libisp_common libisp_hal libisp_isi\
#							libisp_cam_engine  libisp_version libisp_cameric_reg_drv  \

#LOCAL_PREBUILT_LIBS := libisp_silicomimageisp_api.so
endif
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk2928board)
LOCAL_C_INCLUDES += \
    frameworks/base/include/ui \
  external/jpeg \
  external/jhead\
  hardware/rockchip/hwcomposer_rga\
  hardware/rockchip/librkvpu\
  hardware/rockchip/libgralloc_ump/ump/include

LOCAL_SHARED_LIBRARIES:= \
    libui \
    libbinder \
    libutils \
    libcutils \
    libcamera_client \
    libgui\
    libjpeg\
    libjpeghwenc\
    libyuvtorgb\
    libion\
    libvpu\
    libdl
    

endif
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk29board)    
LOCAL_C_INCLUDES += \
	#frameworks/base/include/ui \
  frameworks/native/include/media/hardware \
  frameworks/native/include/media/openmax \
  external/jpeg \
  external/jhead	
  
LOCAL_SHARED_LIBRARIES:= \
    libui \
    libbinder \
    libutils \
    libcutils \
    libcamera_client \
    libgui\
    libjpeghwenc\
    libjpeg\
    libyuvtorgb
endif

ifeq ($(strip $(BOARD_USE_DRM)), true)
LOCAL_C_INCLUDES += hardware/rockchip/librga
LOCAL_SHARED_LIBRARIES +=librga
LOCAL_SHARED_LIBRARIES +=libdrm
endif

LOCAL_CPPFLAGS := -fpermissive
LOCAL_CFLAGS := -fno-short-enums -DCOPY_IMAGE_BUFFER
LOCAL_CFLAGS += -DLINUX  -DMIPI_USE_CAMERIC -DHAL_MOCKUP -DCAM_ENGINE_DRAW_DOM_ONLY -D_FILE_OFFSET_BITS=64 -DHAS_STDINT_H

ifeq ($(strip $(GRAPHIC_MEMORY_PROVIDER)),dma_buf)
LOCAL_CFLAGS += -DUSE_DMA_BUF
endif

ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk30board)	
LOCAL_CFLAGS += -DTARGET_RK30
LOCAL_CFLAGS += -DHAL_MOCKUP
endif

ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk2928board)
LOCAL_CFLAGS += -DTARGET_RK30
endif

ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk29board) 
LOCAL_CFLAGS += -DTARGET_RK29
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
LOCAL_CFLAGS += -DTARGET_RK32
LOCAL_CFLAGS += -DHAL_MOCKUP
LOCAL_CFLAGS += -DHAVE_ARM_NEON
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM_GPU)), mali-t720)
LOCAL_CFLAGS += -DMALI_PRODUCT_ID_T72X=1
LOCAL_CFLAGS += -DMALI_AFBC_GRALLOC=0
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM_GPU)), mali-t760)
LOCAL_CFLAGS += -DMALI_PRODUCT_ID_T76X=1
# we use mali_afbc_gralloc, only if macro MALI_AFBC_GRALLOC is 1
LOCAL_CFLAGS += -DMALI_AFBC_GRALLOC=1
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM_GPU)), mali-t860)
LOCAL_CFLAGS += -DMALI_PRODUCT_ID_T86X=1
LOCAL_CFLAGS += -DMALI_AFBC_GRALLOC=1
endif

ifneq ($(filter rk3366 rk3399, $(strip $(TARGET_BOARD_PLATFORM))), )
LOCAL_CFLAGS += -DTARGET_RK3368
ifeq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_CFLAGS += -DHAVE_ARM_NEON
endif
LOCAL_CFLAGS += -DTARGET_RK32
LOCAL_CFLAGS += -DHAL_MOCKUP
ifeq ($(strip $(TARGET_BOARD_PLATFORM_PRODUCT)), laptop)
	LOCAL_CFLAGS += -DLAPTOP
endif
endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3368)
LOCAL_CFLAGS += -DTARGET_RK3368
ifeq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_CFLAGS += -DHAVE_ARM_NEON
endif
LOCAL_CFLAGS += -DTARGET_RK32
LOCAL_CFLAGS += -DHAL_MOCKUP
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3036)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
LOCAL_CFLAGS += -DTARGET_RK32
LOCAL_CFLAGS += -DHAL_MOCKUP
endif

ifneq ($(filter rk322x rk312x , $(strip $(TARGET_BOARD_PLATFORM))), )
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
LOCAL_CFLAGS += -DTARGET_RK312x
LOCAL_CFLAGS += -DHAL_MOCKUP
LOCAL_CFLAGS += -DHAVE_ARM_NEON
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3328)
#LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
LOCAL_CFLAGS += -DTARGET_RK32
LOCAL_CFLAGS += -DTARGET_RK3328
LOCAL_CFLAGS += -DHAL_MOCKUP
endif

ifneq ($(filter rk322x rk3328 , $(strip $(TARGET_BOARD_PLATFORM))), )
LOCAL_CFLAGS += -DTARGET_RK322x
LOCAL_SRC_FILES += Jpeg_soft_encode.cpp
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3188)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
LOCAL_CFLAGS += -DTARGET_RK3188
LOCAL_CFLAGS += -DHAL_MOCKUP
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3026)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk30xx)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk319x)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
endif
ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk2928)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK2928
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk30xxb)	
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XXB
endif

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_CFLAGS += -DANDROID_5_X
endif

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 6.0)))
LOCAL_CFLAGS += -DANDROID_6_X
endif

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 7.0)))
LOCAL_CFLAGS += -DANDROID_7_X
endif

ifeq ($(strip $(BOARD_USE_DRM)), true)
LOCAL_CFLAGS +=-DRK_DRM_GRALLOC=1	
endif	


#LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH := hw
endif

LOCAL_MODULE:=camera.rk30board

LOCAL_MODULE_TAGS:= optional
include $(BUILD_SHARED_LIBRARY)


