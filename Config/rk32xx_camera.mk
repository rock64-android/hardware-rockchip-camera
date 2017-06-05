ifeq ($(filter box vr stbvr, $(strip $(TARGET_BOARD_PLATFORM_PRODUCT))), )

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3399)

PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api

PRODUCT_COPY_FILES += \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858.xml:system/etc/OV8858.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A.xml:system/etc/OV8858_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-40108A7_OTP.xml:system/etc/OV8858_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2_OTP.xml:system/etc/OV8858_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_000V47-F415B-BW5B0.xml:system/etc/OV8858_000V47-F415B-BW5B0.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0407-FV3.xml:system/etc/OV8858_CMK-CB0407-FV3.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0692-FV1.xml:system/etc/OV8858_CMK-CB0692-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-9569A2.xml:system/etc/OV8858_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-40108A7.xml:system/etc/OV8858_LG-40108A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_OIN8A32.xml:system/etc/OV8858_OIN8A32.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_SUNNY-3813A.xml:system/etc/OV8858_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2.xml:system/etc/OV8858_R1A_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R1A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R2A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-40108A7_OTP.xml:system/etc/OV8858_R2A_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648.xml:system/etc/OV5648.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_000V30-F415B-BY0A1.xml:system/etc/OV5648_000V30-F415B-BY0A1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CMK-CW2005-FV1.xml:system/etc/OV5648_CMK-CW2005-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CHT-842B-MD.xml:system/etc/OV5648_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_XY-LE001B1.xml:system/etc/OV5648_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD.xml:system/etc/OV5648_lens_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_XY-LE001B1.xml:system/etc/OV5648_lens_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD_OTP.xml:system/etc/OV5648_lens_CHT-842B-MD_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214.xml:system/etc/IMX214.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_lens_50013A7_OTP.xml:system/etc/IMX214_lens_50013A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_LG-50013A7.xml:system/etc/IMX214_LG-50013A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_4BAD06P1.xml:system/etc/IMX214_4BAD06P1.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604.xml:system/etc/GS8604.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_9569A2.xml:system/etc/GS8604_lens_9569A2.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_40100A.xml:system/etc/GS8604_lens_40100A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV2680/calib/OV2680.xml:system/etc/OV2680.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/HM5040/calib/HM5040.xml:system/etc/HM5040.xml \
    hardware/rockchip/camera/Config/cam_board_rk3399.xml:system/etc/cam_board.xml
	
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3366)

PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api

PRODUCT_COPY_FILES += \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858.xml:system/etc/OV8858.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A.xml:system/etc/OV8858_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-40108A7_OTP.xml:system/etc/OV8858_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2_OTP.xml:system/etc/OV8858_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_000V47-F415B-BW5B0.xml:system/etc/OV8858_000V47-F415B-BW5B0.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0407-FV3.xml:system/etc/OV8858_CMK-CB0407-FV3.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0692-FV1.xml:system/etc/OV8858_CMK-CB0692-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-9569A2.xml:system/etc/OV8858_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-40108A7.xml:system/etc/OV8858_LG-40108A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_OIN8A32.xml:system/etc/OV8858_OIN8A32.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_SUNNY-3813A.xml:system/etc/OV8858_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2.xml:system/etc/OV8858_R1A_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R1A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R2A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-40108A7_OTP.xml:system/etc/OV8858_R2A_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648.xml:system/etc/OV5648.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_000V30-F415B-BY0A1.xml:system/etc/OV5648_000V30-F415B-BY0A1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CMK-CW2005-FV1.xml:system/etc/OV5648_CMK-CW2005-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CHT-842B-MD.xml:system/etc/OV5648_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_XY-LE001B1.xml:system/etc/OV5648_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD.xml:system/etc/OV5648_lens_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_XY-LE001B1.xml:system/etc/OV5648_lens_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD_OTP.xml:system/etc/OV5648_lens_CHT-842B-MD_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214.xml:system/etc/IMX214.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_lens_50013A7_OTP.xml:system/etc/IMX214_lens_50013A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_LG-50013A7.xml:system/etc/IMX214_LG-50013A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_4BAD06P1.xml:system/etc/IMX214_4BAD06P1.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604.xml:system/etc/GS8604.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_9569A2.xml:system/etc/GS8604_lens_9569A2.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_40100A.xml:system/etc/GS8604_lens_40100A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV2680/calib/OV2680.xml:system/etc/OV2680.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/HM5040/calib/HM5040.xml:system/etc/HM5040.xml \
    hardware/rockchip/camera/Config/cam_board_rk3366.xml:system/etc/cam_board.xml
	
endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3368)

PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api

PRODUCT_COPY_FILES += \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858.xml:system/etc/OV8858.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A.xml:system/etc/OV8858_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-40108A7_OTP.xml:system/etc/OV8858_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2_OTP.xml:system/etc/OV8858_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_000V47-F415B-BW5B0.xml:system/etc/OV8858_000V47-F415B-BW5B0.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0407-FV3.xml:system/etc/OV8858_CMK-CB0407-FV3.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0692-FV1.xml:system/etc/OV8858_CMK-CB0692-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-9569A2.xml:system/etc/OV8858_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-40108A7.xml:system/etc/OV8858_LG-40108A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_OIN8A32.xml:system/etc/OV8858_OIN8A32.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_SUNNY-3813A.xml:system/etc/OV8858_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2.xml:system/etc/OV8858_R1A_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R1A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R2A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-40108A7_OTP.xml:system/etc/OV8858_R2A_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648.xml:system/etc/OV5648.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_000V30-F415B-BY0A1.xml:system/etc/OV5648_000V30-F415B-BY0A1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CMK-CW2005-FV1.xml:system/etc/OV5648_CMK-CW2005-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CHT-842B-MD.xml:system/etc/OV5648_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_XY-LE001B1.xml:system/etc/OV5648_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD.xml:system/etc/OV5648_lens_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_XY-LE001B1.xml:system/etc/OV5648_lens_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD_OTP.xml:system/etc/OV5648_lens_CHT-842B-MD_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214.xml:system/etc/IMX214.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_lens_50013A7_OTP.xml:system/etc/IMX214_lens_50013A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_LG-50013A7.xml:system/etc/IMX214_LG-50013A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_4BAD06P1.xml:system/etc/IMX214_4BAD06P1.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604.xml:system/etc/GS8604.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_9569A2.xml:system/etc/GS8604_lens_9569A2.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_40100A.xml:system/etc/GS8604_lens_40100A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV2680/calib/OV2680.xml:system/etc/OV2680.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/HM5040/calib/HM5040.xml:system/etc/HM5040.xml \
    hardware/rockchip/camera/Config/cam_board_rk3368.xml:system/etc/cam_board.xml
	
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3288)

PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api

PRODUCT_COPY_FILES += \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8820/calib/OV8820.xml:system/etc/OV8820.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8825/calib/OV8825.xml:system/etc/OV8825.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858.xml:system/etc/OV8858.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2.xml:system/etc/OV8858_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A.xml:system/etc/OV8858_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-40108A7_OTP.xml:system/etc/OV8858_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_LG-9569A2_OTP.xml:system/etc/OV8858_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_000V47-F415B-BW5B0.xml:system/etc/OV8858_000V47-F415B-BW5B0.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0407-FV3.xml:system/etc/OV8858_CMK-CB0407-FV3.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_CMK-CB0692-FV1.xml:system/etc/OV8858_CMK-CB0692-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-9569A2.xml:system/etc/OV8858_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_LG-40108A7.xml:system/etc/OV8858_LG-40108A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_OIN8A32.xml:system/etc/OV8858_OIN8A32.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_SUNNY-3813A.xml:system/etc/OV8858_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2.xml:system/etc/OV8858_R1A_lens_LG-9569A2.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R1A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R1A_lens_SUNNY-3813A_OTP.xml:system/etc/OV8858_R1A_lens_SUNNY-3813A_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-9569A2_OTP.xml:system/etc/OV8858_R2A_lens_LG-9569A2_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV8858/calib/OV8858_R2A_lens_LG-40108A7_OTP.xml:system/etc/OV8858_R2A_lens_LG-40108A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648.xml:system/etc/OV5648.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_000V30-F415B-BY0A1.xml:system/etc/OV5648_000V30-F415B-BY0A1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CMK-CW2005-FV1.xml:system/etc/OV5648_CMK-CW2005-FV1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_CHT-842B-MD.xml:system/etc/OV5648_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_XY-LE001B1.xml:system/etc/OV5648_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD.xml:system/etc/OV5648_lens_CHT-842B-MD.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_XY-LE001B1.xml:system/etc/OV5648_lens_XY-LE001B1.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV5648/calib/OV5648_lens_CHT-842B-MD_OTP.xml:system/etc/OV5648_lens_CHT-842B-MD_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV13850/calib/OV13850.xml:system/etc/OV13850.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214.xml:system/etc/IMX214.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_lens_50013A7_OTP.xml:system/etc/IMX214_lens_50013A7_OTP.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_LG-50013A7.xml:system/etc/IMX214_LG-50013A7.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/IMX214/calib/IMX214_4BAD06P1.xml:system/etc/IMX214_4BAD06P1.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604.xml:system/etc/GS8604.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_9569A2.xml:system/etc/GS8604_lens_9569A2.xml \
	hardware/rockchip/camera/SiliconImage/isi/drv/GS8604/calib/GS8604_lens_40100A.xml:system/etc/GS8604_lens_40100A.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/OV2680/calib/OV2680.xml:system/etc/OV2680.xml \
    hardware/rockchip/camera/SiliconImage/isi/drv/HM5040/calib/HM5040.xml:system/etc/HM5040.xml \
    hardware/rockchip/camera/Config/cam_board_rk3288.xml:system/etc/cam_board.xml
	
endif
else
PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api
endif

ifneq ($(filter rk322x rk312x rk3126c rk3128 px3se, $(strip $(TARGET_BOARD_PLATFORM))), )
PRODUCT_PACKAGES += \
    libisp_silicomimageisp_api
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3328)
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
