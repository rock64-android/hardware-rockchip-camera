LOCAL_PATH := $(call my-dir)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)

#$(shell cp -rf $(LOCAL_PATH)/cam_board.xml $(TARGET_OUT)/etc/cam_board.xml) 
#$(shell cp -rf $(LOCAL_PATH)/libisp_silicomimageisp_api.so $(TARGET_OUT)/../obj/lib/libisp_silicomimageisp_api.so) 
#$(shell cp -rf $(LOCAL_PATH)/libisp_silicomimageisp_api.so $(TARGET_OUT)/lib/libisp_silicomimageisp_api.so) 
#$(shell cp -rf $(LOCAL_PATH)/../SiliconImage/isi/drv/OV8820/calib/OV8820.xml $(TARGET_OUT)/etc/OV8820.xml) 
#$(shell cp -rf $(LOCAL_PATH)/../SiliconImage/isi/drv/OV8825/calib/OV8825.xml $(TARGET_OUT)/etc/OV8825.xml) 
#$(shell cp -rf $(LOCAL_PATH)/../SiliconImage/isi/drv/OV8858/calib/OV8858.xml $(TARGET_OUT)/etc/OV8858.xml) 
#$(shell cp -rf $(LOCAL_PATH)/../SiliconImage/isi/drv/OV13850/calib/OV13850.xml $(TARGET_OUT)/etc/OV13850.xml) 

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := libisp_silicomimageisp_api.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)


endif

