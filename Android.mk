#
# RockChip Camera HAL 
#
LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=\
	CameraHal_Module.cpp\
	CameraHal.cpp\
	CameraHal_Utils.cpp\
	MessageQueue.cpp\
	CameraHal_Mem.cpp
  
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk30board)	 
LOCAL_C_INCLUDES += \
	frameworks/base/include/ui \
  external/jpeg \
  external/jhead\
  hardware/rk29/hwcomposer_rga\
	hardware/rk29/libgralloc_ump/ump/include	

LOCAL_SHARED_LIBRARIES:= \
    libui \
    libbinder \
    libutils \
    libcutils \
    libcamera_client \
    libgui\
    libjpeg\
    libjpeghwenc\
    libion
    #libyuvtorgb\

endif
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk2928board)
LOCAL_C_INCLUDES += \
    frameworks/base/include/ui \
  external/jpeg \
  external/jhead\
  hardware/rk29/hwcomposer_rga\
    hardware/rk29/libgralloc_ump/ump/include

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
    libion
    #    
    #libyuvtorgb\

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


LOCAL_CFLAGS := -fno-short-enums -DCOPY_IMAGE_BUFFER
ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk30board)	
LOCAL_CFLAGS += -DTARGET_RK30
endif

ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk2928board)
LOCAL_CFLAGS += -DTARGET_RK30
endif

ifeq ($(strip $(TARGET_BOARD_HARDWARE)),rk29board) 
LOCAL_CFLAGS += -DTARGET_RK29
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk30xx)	
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XX
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk2928)
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK2928
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk30xxb)	
LOCAL_CFLAGS += -DTARGET_BOARD_PLATFORM_RK30XXB
endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE:=camera.$(TARGET_BOARD_HARDWARE)

LOCAL_MODULE_TAGS:= optional
include $(BUILD_SHARED_LIBRARY)
