ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3288)

PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api

PRODUCT_COPY_FILES += \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A.xml:system/etc/OV8858_lens_SUNNY-3813A.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD.xml:system/etc/OV5648_lens_CHT-842B-MD.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_XY-LE001B1.xml:system/etc/OV5648_lens_XY-LE001B1.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_lens_SUNNY-3813A_OTP.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD_OTP.xml:system/etc/OV5648_lens_CHT-842B-MD_OTP.xml \
    hardware/rk29/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rk29/camera/Config/cam_board.xml:system/etc/cam_board.xml 
	
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk312x)
PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3036)
PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api
endif

#force camera API 1
PRODUCT_PROPERTY_OVERRIDES += \
    camera2.portability.force_api=1
