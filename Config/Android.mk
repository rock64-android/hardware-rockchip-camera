LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libisp_silicomimageisp_api

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 7.0)))
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_7x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_7x_64bit.so
else
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 6.0)))
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_6x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_6x_64bit.so
else
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_5x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_5x_64bit.so
endif
endif

ifneq ($(filter rk3366 rk3399 rk3328, $(strip $(TARGET_BOARD_PLATFORM))), )
#include $(CLEAR_VARS)
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH :=
endif
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_32bit.so
#LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_64bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3368)
#include $(CLEAR_VARS)
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH :=
endif
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_32bit.so
#LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_64bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)
#include $(CLEAR_VARS)
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES := $(LOCAL_MODULE)_32bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifneq ($(filter rk322x rk312x rk3126c rk3128 px3se, $(strip $(TARGET_BOARD_PLATFORM))), )
#include $(CLEAR_VARS)
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES := $(LOCAL_MODULE)_32bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3036)
#include $(CLEAR_VARS)
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES := $(LOCAL_MODULE)_32bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3188)
#include $(CLEAR_VARS)
#LOCAL_MODULE := libisp_silicomimageisp_api
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
#LOCAL_SRC_FILES := $(LOCAL_MODULE)_32bit.so
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

