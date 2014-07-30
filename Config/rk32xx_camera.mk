ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3288)
PRODUCT_COPY_FILES += \
    hardware/rk29/camera/Config/libisp_silicomimageisp_api.so:obj/lib/libisp_silicomimageisp_api.so \
    hardware/rk29/camera/Config/libisp_silicomimageisp_api.so:system/lib/libisp_silicomimageisp_api.so \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8858/calib/OV8858.xml:system/etc/OV8858.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rk29/camera/Config/cam_board.xml:system/etc/cam_board.xml 
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk312x)
PRODUCT_COPY_FILES += \
    hardware/rk29/camera/Config/libisp_silicomimageisp_api.so:obj/lib/libisp_silicomimageisp_api.so \
    hardware/rk29/camera/Config/libisp_silicomimageisp_api.so:system/lib/libisp_silicomimageisp_api.so 
endif
