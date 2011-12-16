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
	

LOCAL_C_INCLUDES += \
	frameworks/base/include/ui \
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

LOCAL_CFLAGS := -fno-short-enums -DCOPY_IMAGE_BUFFER
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE:= camera.rk29board
LOCAL_MODULE_TAGS:= optional

include $(BUILD_SHARED_LIBRARY)
