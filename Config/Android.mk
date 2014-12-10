LOCAL_PATH := $(call my-dir)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)
include $(CLEAR_VARS)
LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
ifeq ($(strip $(PLATFORM_SDK_VERSION)), 21)
LOCAL_SRC_FILES := $(LOCAL_MODULE)_lollipop.so
else
LOCAL_SRC_FILES := $(LOCAL_MODULE)_kitkat.so
endif
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)


endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk312x)
include $(CLEAR_VARS)
LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
ifeq ($(strip $(PLATFORM_SDK_VERSION)), 21)
LOCAL_SRC_FILES := $(LOCAL_MODULE)_lollipop.so
else
LOCAL_SRC_FILES := $(LOCAL_MODULE)_kitkat.so
endif
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)

endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3036)
include $(CLEAR_VARS)
LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
ifeq ($(strip $(PLATFORM_SDK_VERSION)), 21)
LOCAL_SRC_FILES := $(LOCAL_MODULE)_lollipop.so
else
LOCAL_SRC_FILES := $(LOCAL_MODULE)_kitkat.so
endif
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)

endif

