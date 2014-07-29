LOCAL_PATH := $(call my-dir)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)
include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := libisp_silicomimageisp_api.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)


endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk312x)

   include $(CLEAR_VARS)
    LOCAL_PREBUILT_LIBS := libisp_silicomimageisp_api.so
    LOCAL_MODULE_TAGS := optional
    include $(BUILD_MULTI_PREBUILT)

endif


