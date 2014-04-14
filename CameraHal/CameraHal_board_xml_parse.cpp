
#define LOG_TAG "BoardProfiles"

#include <stdlib.h>
#include <utils/Log.h>
#include <libexpat/expat.h>
#include <dlfcn.h>
#include <isi/isi_iss.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <linux/videodev.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <signal.h>
#include <linux/ion.h>
#include <linux/android_pmem.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/types.h>
#include <linux/version.h>

#include "CameraHal_board_xml_parse.h"


void camera_board_profiles::ParserSensorInfo(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
    rk_sensor_info *pSensorInfo = &(pCamInfo->mHardInfo.mSensorInfo);
	int result;

    if (strcmp(name, "SensorName")==0) {
	    ALOGD("%s(%d): SensorName(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        strncpy(pSensorInfo->mSensorName, atts[1], strlen(atts[1]));
        ALOGD("%s(%d): SensorName(%s)\n", __FUNCTION__, __LINE__, pSensorInfo->mSensorName);
    } else if (strcmp(name, "SensorDevID")==0) {
        ALOGD("%s(%d): SensorDevID(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        if(strcmp("CAMSYS_DEVID_SENSOR_1A", atts[1])==0){
            pSensorInfo->mCamDevid = CAMSYS_DEVID_SENSOR_1A;
        }else if(strcmp("CAMSYS_DEVID_SENSOR_1B", atts[1])==0){
            pSensorInfo->mCamDevid = CAMSYS_DEVID_SENSOR_1B; 
        }else if(strcmp("CAMSYS_DEVID_SENSOR_2", atts[1])==0){
            pSensorInfo->mCamDevid = CAMSYS_DEVID_SENSOR_2; 
        }else{
            pSensorInfo->mCamDevid = 0;
            ALOGD("%s(%d): SensorDevID(%s) don't support\n", __FUNCTION__, __LINE__, atts[1]);
        }
        ALOGD("%s(%d): SensorDevID(%d)\n", __FUNCTION__, __LINE__, pSensorInfo->mCamDevid);
    } else if (strcmp(name,"SensorHostDevID")==0){
        ALOGD("%s(%d): SensorHostDevID(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        if(strcmp("CAMSYS_DEVID_MARVIN", atts[1])==0){
            pSensorInfo->mHostDevid= CAMSYS_DEVID_MARVIN;
            strncpy(pSensorInfo->mCamsysDevPath, "/dev/camsys_marvin", sizeof(pSensorInfo->mCamsysDevPath));
        }else if(strcmp("CAMSYS_DEVID_CIF_0", atts[1])==0){
            pSensorInfo->mHostDevid = CAMSYS_DEVID_CIF_0; 
            strncpy(pSensorInfo->mCamsysDevPath, "/dev/camsys_cif0", sizeof(pSensorInfo->mCamsysDevPath));
        }else if(strcmp("CAMSYS_DEVID_CIF_1", atts[1])==0){
            pSensorInfo->mHostDevid = CAMSYS_DEVID_CIF_1; 
            strncpy(pSensorInfo->mCamsysDevPath, "/dev/camsys_cif1", sizeof(pSensorInfo->mCamsysDevPath));
        }else {
            pSensorInfo->mHostDevid = 0;
            ALOGD("%s(%d): SensorDevID(%s) don't support\n", __FUNCTION__, __LINE__, atts[1]);
        }
    } else if (strcmp(name,"SensorI2cBusNum")==0){
        ALOGD("%s(%d): Sensori2cBusNum(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mSensorI2cBusNum = atoi(atts[1]);
    } else if (strcmp(name,"SensorI2cAddrByte")==0){
        ALOGD("%s(%d): SensorI2cAddrByte(%s)\n", __FUNCTION__, __LINE__, atts[1]); 
        pSensorInfo->mI2cAddrBytes = atoi(atts[1]);
    } else if (strcmp(name,"SensorI2cRate")==0){
        ALOGD("%s(%d): SensorI2cRate(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mSensorI2cRate = atoi(atts[1]);
    } else if (strcmp(name,"SensorMclk")==0){
        ALOGD("%s(%d): SensorMclk(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mMclkRate = atoi(atts[1]);
    } else if (strcmp(name,"SensorAvdd")==0){
        ALOGD("%s(%d): SensorAvdd(%s) min(%s) max(%s)\n", __FUNCTION__, __LINE__, atts[1], atts[3], atts[5]);
        strncpy(pSensorInfo->mAvdd.name, atts[1], strlen(atts[1]));
        pSensorInfo->mAvdd.min_uv = atoi(atts[3]);
        pSensorInfo->mAvdd.max_uv = atoi(atts[5]);
    } else if (strcmp(name,"SensorDovdd")==0){
        ALOGD("%s(%d): SensorDovdd(%s) min(%s) max(%s)\n", __FUNCTION__, __LINE__, atts[1], atts[3], atts[5]);
        strncpy(pSensorInfo->mDovdd.name, atts[1], strlen(atts[1]));
        pSensorInfo->mDovdd.min_uv = atoi(atts[3]);   
        pSensorInfo->mDovdd.max_uv = atoi(atts[5]);
    } else if (strcmp(name,"SensorDvdd")==0){
        ALOGD("%s(%d): SensorDvdd(%s) min(%s) max(%s)\n", __FUNCTION__, __LINE__, atts[1], atts[3], atts[5]);
        strncpy(pSensorInfo->mDvdd.name, atts[1], strlen(atts[1]));
        pSensorInfo->mDvdd.min_uv = atoi(atts[3]);  
        pSensorInfo->mDvdd.max_uv = atoi(atts[5]);
    } else if (strcmp(name,"SensorGpioPwdn")==0){
        ALOGD("%s(%d): SensorGpioPwdn(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pSensorInfo->mSensorGpioPwdn.name, atts[1], strlen(atts[1]));
        pSensorInfo->mSensorGpioPwdn.active = atoi(atts[3]);
    } else if (strcmp(name,"SensorGpioRst")==0){
        ALOGD("%s(%d): SensorGpioRst(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pSensorInfo->mSensorGpioReset.name, atts[1], strlen(atts[1]));
        pSensorInfo->mSensorGpioReset.active = atoi(atts[3]);
    } else if (strcmp(name,"SensorGpioPwen")==0){
        ALOGD("%s(%d): SensorGpioPwen(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pSensorInfo->SensorGpioPwen.name, atts[1], strlen(atts[1]));
        pSensorInfo->SensorGpioPwen.active = atoi(atts[3]);
    }else if (strcmp(name,"SensorFacing")==0){
        ALOGD("%s(%d): SensorFacing(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        if(strcmp("front", atts[1])==0){
            pSensorInfo->mFacing = RK_CAM_FACING_FRONT;
        }else if(strcmp("back", atts[1])==0){
            pSensorInfo->mFacing = RK_CAM_FACING_BACK;
        }else{
            ALOGD("%s(%d): SensorFacing(%s) is wrong \n", __FUNCTION__, __LINE__, atts[1]);
        }
    } else if (strcmp(name,"SensorMode")==0){
        ALOGD("%s(%d): SensorMode(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        if(strcmp("MIPI_2_LANE", atts[1])==0){
            pSensorInfo->mMode = MIPI_2_LANE;
        }else if(strcmp("MIPI_4_LANE", atts[1])==0){
            pSensorInfo->mMode = MIPI_4_LANE;
        }else if(strcmp("DVP", atts[1])==0){
            pSensorInfo->mMode = DVP;
        }else if(strcmp("CCIR656", atts[1])==0){
            pSensorInfo->mMode = CCIR656;
        }else{
            ALOGD("%s(%d): SensorMode(%s) don't support \n", __FUNCTION__, __LINE__, atts[1]);
        }
    } else if (strcmp(name,"SensorMirrorFlip")==0){
        ALOGD("%s(%d): SensorMirrorFlip(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mMirrorFilp = atoi(atts[1]);
    } else if (strcmp(name,"SensorPowerupSequence")==0){
        ALOGD("%s(%d): SensorPowerupSequence(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mSensorPowerupSequence = atoi(atts[1]);
    } else if(strcmp(name, "SensorOrientation")==0){
        ALOGD("%s(%d): SensorOrientation(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        pSensorInfo->mOrientation = atoi(atts[1]);
    }else if(strcmp(name, "SensorDriver")==0){
        ALOGD("%s(%d): SensorDriver(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        strncpy(pSensorInfo->mSensorDriver, atts[1], sizeof(pSensorInfo->mSensorDriver));
    }else if(strcmp(name, "SensorPhy")==0){
        ALOGD("%s(%d): SensorPhy(%s) \n", __FUNCTION__, __LINE__, atts[1]);
        if(strcmp(atts[1], "CamSys_Phy_Mipi")==0){
            pSensorInfo->mPhy.type = CamSys_Phy_Mipi;
            pSensorInfo->mPhy.info.mipi.data_en_bit = atoi(atts[3]);
        }else if(strcmp(atts[1], "CamSys_Phy_Cif")==0){
            pSensorInfo->mPhy.type = CamSys_Phy_Cif;
            pSensorInfo->mPhy.info.cif.cif_num = atoi(atts[5]);
            if(strcmp(atts[7], "CamSys_Fmt_Yuv420_8b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Yuv420_8b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Yuv420_10b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Yuv420_10b;
            }else if(strcmp(atts[7], "CamSys_Fmt_LegacyYuv420_8b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_LegacyYuv420_8b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Yuv422_8b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Yuv422_8b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Yuv422_10b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Yuv422_10b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_6b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_6b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_7b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_7b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_8b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_8b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_10b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_10b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_12b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_12b;
            }else if(strcmp(atts[7], "CamSys_Fmt_Raw_14b")==0){
                pSensorInfo->mPhy.info.cif.fmt = CamSys_Fmt_Raw_14b;
            }else {
               ALOGE("%s(%d): unknown fmt (%s) \n", __FUNCTION__, __LINE__, atts[1]); 
            }
        }else{
           ALOGE("%s(%d): unknown phy mode(%s) \n", __FUNCTION__, __LINE__, atts[1]); 
        }
        strncpy(pSensorInfo->mSensorDriver, atts[1], sizeof(pSensorInfo->mSensorDriver));
    }
}

void camera_board_profiles::ParserVCMInfo(const char *name, const char **atts, void *userData)
{
    camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_vcm_info *pVcmInfo = &(pCamInfo->mHardInfo.mVcmInfo);

    if (strcmp(name, "VCMName")==0) {
        ALOGD("%s(%d): VCMName(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        strncpy(pVcmInfo->mVcmName, atts[1], strlen(atts[1]));
    } else if (strcmp(name, "VCMI2cBusNum")==0) {
        ALOGD("%s(%d): VCMI2cBusNum(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pVcmInfo->mVcmI2cBusNum = atoi(atts[1]);
    } else if (strcmp(name,"VCMI2cAddrByte")==0){
        ALOGD("%s(%d): VCMI2cAddrByte(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pVcmInfo->mI2cAddrBytes = atoi(atts[1]);
    } else if (strcmp(name,"VCMI2cRate")==0){
        ALOGD("%s(%d): VCMI2cRate(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pVcmInfo->mVcmI2cRate = atoi(atts[1]);
    } else if (strcmp(name,"VCMGpioPwdn")==0){
        ALOGD("%s(%d): VCMGpioPwdn(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pVcmInfo->mVcmGpioPwdn.name, atts[1], strlen(atts[1]));
        pVcmInfo->mVcmGpioPwdn.active = atoi(atts[3]);
    } else if (strcmp(name,"VCMGpioPower")==0){
        ALOGD("%s(%d): VCMGpioPower(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pVcmInfo->mVcmGpioPower.name, atts[1], strlen(atts[1]));
        pVcmInfo->mVcmGpioPower.active = atoi(atts[3]);
    } else if (strcmp(name,"VCMVdd")==0){
        ALOGD("%s(%d): VCMVdd(%s) min(%s) max(%s)\n", __FUNCTION__, __LINE__, atts[1], atts[3], atts[5]);
        strncpy(pVcmInfo->mVcmVdd.name, atts[1], strlen(atts[1]));       
        pVcmInfo->mVcmVdd.min_uv= atoi(atts[3]);
        pVcmInfo->mVcmVdd.max_uv= atoi(atts[5]);
    } 
	
}

void camera_board_profiles::ParserFlashInfo(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_flash_info *pFlashInfo = &(pCamInfo->mHardInfo.mFlashInfo);
	rk_flash_config *pFlashConfig = &(pCamInfo->mSoftInfo.mFlashConfig);
	int support = 0;

    if (strcmp(name, "FlashName")==0) {
        ALOGD("%s(%d): FlashName(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        strncpy(pFlashInfo->mFlashName, atts[1], strlen(atts[1]));
    } else if (strcmp(name, "FlashI2cBusNum")==0) {
        ALOGD("%s(%d): FlashI2cBusNum(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pFlashInfo->mFlashI2cBusNum = atoi(atts[1]);
    } else if (strcmp(name,"FlashI2cAddrByte")==0){
        ALOGD("%s(%d): FlashI2cAddrByte(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pFlashInfo->mI2cAddrBytes = atoi(atts[1]);
    } else if (strcmp(name,"FlashI2cRate")==0){
        ALOGD("%s(%d): FlashI2cRate(%s)\n", __FUNCTION__, __LINE__, atts[1]);
        pFlashInfo->mFlashI2cRate = atoi(atts[1]);
    } else if (strcmp(name,"FlashGpioPwdn")==0){
        ALOGD("%s(%d): FlashGpioPwdn(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pFlashInfo->mFlashGpioPwdn.name, atts[1], strlen(atts[1]));
        pFlashInfo->mFlashGpioPwdn.active = atoi(atts[3]);
    } else if (strcmp(name,"FlashGpioPower")==0){
        ALOGD("%s(%d): FlashGpioPower(%s) active(%s) \n", __FUNCTION__, __LINE__, atts[1], atts[3]);
        strncpy(pFlashInfo->mFlashGpioPower.name, atts[1], strlen(atts[1]));
        pFlashInfo->mFlashGpioPower.active = atoi(atts[3]);
    } else if (strcmp(name,"FlashVdd")==0){
        ALOGD("%s(%d): FlashVdd(%s) min(%s) max(%s)\n", __FUNCTION__, __LINE__, atts[1], atts[3], atts[5]);
        strncpy(pFlashInfo->mFlashVdd.name, atts[1], strlen(atts[1]));       
        pFlashInfo->mFlashVdd.min_uv= atoi(atts[3]);
        pFlashInfo->mFlashVdd.max_uv= atoi(atts[5]);
    } else if (strcmp(name, "Flash_Mode_Off")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pFlashConfig->mFlashSupport |= (0x01<<FLASH_MODE_OFF_BITPOS); 
    } else if (strcmp(name, "Flash_Mode_On")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pFlashConfig->mFlashSupport |= (0x01<<FLASH_MODE_ON_BITPOS); 
    } else if (strcmp(name,"Flash_Mode_Torch")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFlashConfig->mFlashSupport |= (0x01<<FLASH_MODE_TORCH_BITPOS); 
    } else if (strcmp(name,"Flash_Mode_Auto")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFlashConfig->mFlashSupport |= (0x01<<FLASH_MODE_AUTO_BITPOS); 
    } else if (strcmp(name,"Flash_Mode_Red_Eye")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFlashConfig->mFlashSupport |= (0x01<<FLASH_MODE_RED_EYE_BITPOS); 
    }
	
}

void camera_board_profiles::ParserAwbConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_white_balance_config *pAWBConfig = &(pCamInfo->mSoftInfo.mAwbConfig);
	int support = 0;

    if (strcmp(name, "AWB_Auto")==0) {
	    support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_AUTO_BITPOS ); 
    } else if (strcmp(name, "AWB_Incandescent")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_INCANDESCENT_BITPOS); 
    } else if (strcmp(name,"AWB_Fluorescent")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_FLUORESCENT_BITPOS); 
    } else if (strcmp(name,"AWB_Warm_Fluorescent")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_WARM_FLUORESCENT_BITPOS); 
    } else if (strcmp(name,"AWB_Daylight")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_DAYLIGHT_BITPOS); 
    } else if (strcmp(name,"AWB_Cloudy_Daylight")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_CLOUDY_BITPOS); 
    } else if (strcmp(name,"AWB_Twilight")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_TWILIGHT_BITPOS); 
    } else if (strcmp(name,"AWB_Shade")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAWBConfig->mAwbSupport |= (0x01<<AWB_SHADE_BITPOS); 
    } 
	
}

void camera_board_profiles::ParserSenceConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_sence_config *pSenceConfig = &(pCamInfo->mSoftInfo.mSenceConfig);
	int support = 0;

   if (strcmp(name, "Sence_Mode_Auto")==0) {
	    support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_AUTO_BITPOS); 
    } else if (strcmp(name, "Sence_Mode_Action")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_ACTION_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Portrait")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_PORTRAIT_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Landscape")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_LANDSCAPE_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Night")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_NIGHT_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Night_Portrait")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_NIGHT_PORTRAIT_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Theatre")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_THEATRE_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Beach")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_BEACH_BITPOS); 
    } else if (strcmp(name, "Sence_Mode_Snow")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_SNOW_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Sunset")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_SUNSET_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Steayphoto")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_STEAYPHOTO_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Pireworks")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_PIREWORKS_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Sports")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_SPORTS_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Party")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport|= (0x01<<SENCE_MODE_PARTY_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Candlelight")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_CANDLELIGHT_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_Barcode")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_BARCODE_BITPOS); 
    } else if (strcmp(name,"Sence_Mode_HDR")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pSenceConfig->mSenceSupport |= (0x01<<SENCE_MODE_HDR_BITPOS); 
    }
	
}

void camera_board_profiles::ParserEffectConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_effect_config *pEffectConfig = &(pCamInfo->mSoftInfo.mEffectConfig);
	int support=0;

    if (strcmp(name, "Effect_None")==0) {
	    support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_NONE_BITPOS); 
    } else if (strcmp(name, "Effect_Mono")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_MONO_BITPOS); 
    } else if (strcmp(name,"Effect_Solarize")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_SOLARIZE_BITPOS); 
    } else if (strcmp(name,"Effect_Negative")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_NEGATIVE_BITPOS); 
    } else if (strcmp(name,"Effect_Sepia")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_SEPIA_BITPOS); 
    } else if (strcmp(name,"Effect_Posterize")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_POSTERIZE_BITPOS); 
    } else if (strcmp(name,"Effect_Whiteboard")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_WHITEBOARD_BITPOS); 
    } else if (strcmp(name,"Effect_Blackboard")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_BLACKBOARD_BITPOS); 
    } else if (strcmp(name,"Effect_Aqua")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pEffectConfig->mEffectSupport |= (0x01<<EFFECT_AQUE_BITPOS); 
    } 
	
}

void camera_board_profiles::ParserFocusConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_focus_config *pFocusConfig = &(pCamInfo->mSoftInfo.mFocusConfig);
	int support=0;

    if (strcmp(name, "Focus_Mode_Auto")==0) {
	    support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_AUTO_BITPOS); 
    } else if (strcmp(name, "Focus_Mode_Infinity")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_INFINITY_BITPOS); 
    } else if (strcmp(name,"Focus_Mode_Marco")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_MARCO_BITPOS); 
    } else if (strcmp(name,"Focus_Mode_Fixed")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_FIXED_BITPOS); 
    } else if (strcmp(name,"Focus_Mode_Edof")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_EDOF_BITPOS); 
    } else if (strcmp(name,"Focus_Mode_Continuous_Video")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_CONTINUOUS_VIDEO_BITPOS); 
    } else if (strcmp(name,"Focus_Mode_Continuous_Picture")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pFocusConfig->mFocusSupport |= (0x01<<FOCUS_CONTINUOUS_PICTURE_BITPOS); 
    } 
	
}

void camera_board_profiles::ParserAntiBandingConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_anti_banding_config *pAntiBandingConfig = &(pCamInfo->mSoftInfo.mAntiBandingConfig);
	int support = 0;

    if (strcmp(name, "Anti_Banding_Auto")==0) {
	    support = atoi(atts[1]);
	    if(support==1)
            pAntiBandingConfig->mAntiBandingSupport |= (0x01<<ANTI_BANDING_AUTO_BITPOS); 
    } else if (strcmp(name, "Anti_Banding_50HZ")==0) {
        support = atoi(atts[1]);
	    if(support==1)
            pAntiBandingConfig->mAntiBandingSupport |= (0x01<<ANTI_BANDING_50HZ_BITPOS); 
    } else if (strcmp(name,"Anti_Banding_60HZ")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAntiBandingConfig->mAntiBandingSupport |= (0x01<<ANTI_BANDING_60HZ_BITPOS); 
    } else if (strcmp(name,"Anti_Banding_Off")==0){
        support = atoi(atts[1]);
	    if(support==1)
            pAntiBandingConfig->mAntiBandingSupport |= (0x01<<ANTI_BANDING_OFF_BITPOS); 
    } 
	
}

void camera_board_profiles::ParserDVConfig(const char *name, const char **atts, void *userData)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	rk_DV_info *pDVResolution = NULL;
    
    if (strcmp(name, "DV_QCIF")==0) {
	    ALOGD("%s(%d):  DV_QCIF(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = 0x00000000;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } else if (strcmp(name, "DV_QVGA")==0) {
        ALOGD("%s(%d):  DV_QVGA(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = 0x00000000;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } else if (strcmp(name,"DV_CIF")==0){
        ALOGD("%s(%d):  DV_CIF(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = 0x00000000;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } else if (strcmp(name,"DV_VGA")==0){
        ALOGD("%s(%d):  DV_VGA(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = ISI_RES_VGA;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    }  else if (strcmp(name, "DV_480P")==0) {
        ALOGD("%s(%d):  DV_480P(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = 0x000000000;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } else if (strcmp(name,"DV_720P")==0){
        ALOGD("%s(%d):  DV_720P(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = (ISI_RES_TV720P5 | ISI_RES_TV720P15 |ISI_RES_TV720P30 |ISI_RES_TV720P60);;
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } else if (strcmp(name,"DV_1080P")==0){
        ALOGD("%s(%d):  DV_1080P(%s) resolution(%sx%s) fps(%s) support(%s)\n", __FUNCTION__, __LINE__, atts[1],atts[3], atts[5],atts[7],atts[9]);
	    pDVResolution = new rk_DV_info();
        if(pDVResolution){
            strncpy(pDVResolution->mName, atts[1], strlen(atts[1]));
    	    pDVResolution->mWidth = atoi(atts[3]);
    	    pDVResolution->mHeight = atoi(atts[5]);
    	    pDVResolution->mFps = atoi(atts[7]);
    	    pDVResolution->mIsSupport =  atoi(atts[9]);
            pDVResolution->mResolution = (ISI_RES_TV1080P5 |ISI_RES_TV1080P12 |ISI_RES_TV1080P15 |ISI_RES_TV1080P20 |ISI_RES_TV1080P24 |ISI_RES_TV1080P25 |ISI_RES_TV1080P30 |ISI_RES_TV1080P50 |ISI_RES_TV1080P60);
            pCamInfo->mSoftInfo.mDV_vector.add(pDVResolution);
        }
    } 
	
}

void camera_board_profiles::StartElementHandler(void *userData, const char *name, const char **atts)
{
	camera_board_profiles *pCamInfoProfiles = (camera_board_profiles *) userData;
	rk_cam_total_info *pCamInfo = pCamInfoProfiles->mCurDevice;
	int support = 0;

	if(strcmp(name,"CamDevie")==0){
	    rk_cam_total_info* pNewCamInfo = new rk_cam_total_info();
	    if(pNewCamInfo){
	        ALOGD("%s(%d):  camdevice malloc success! (%p) \n", __FUNCTION__,__LINE__, pNewCamInfo);
            pCamInfoProfiles->mCurDevice= pNewCamInfo;
            pCamInfoProfiles->mDevieVector.add(pNewCamInfo);
            pNewCamInfo->mDeviceIndex = (pCamInfoProfiles->mDevieVector.size()) - 1;
	    }else{
            ALOGD("%s(%d): Warnimg camdevice malloc fail! \n", __FUNCTION__,__LINE__);
	    }
	}else if (strstr(name, "Sensor")) {
        ParserSensorInfo(name, atts, userData);
    } else if (strstr(name, "VCM")) {
        ParserVCMInfo(name, atts, userData);
    } else if (strstr(name,"Flash")){
        ParserFlashInfo(name, atts, userData);
    } else if (strstr(name,"AWB")){
        ParserAwbConfig(name, atts, userData);
    } else if (strstr(name,"Sence")){
        ParserSenceConfig(name, atts, userData);
    } else if (strstr(name,"Effect")){
        ParserEffectConfig(name, atts, userData);
    } else if (strstr(name,"Focus")){
        ParserFocusConfig(name, atts, userData);
    } else if (strstr(name,"Anti_Banding")){
        ParserAntiBandingConfig(name, atts, userData);
    } else if (strstr(name,"HDR")){
        support = atoi(atts[1]);
        pCamInfo->mSoftInfo.mHDRConfig = support;
        ALOGD("%s(%d): HDR(%d)! \n", __FUNCTION__,__LINE__,support);
    } else if (strstr(name,"ZSL")){
        support = atoi(atts[1]);
        pCamInfo->mSoftInfo.mZSLConfig= support;
        ALOGD("%s(%d): ZSL(%d)! \n", __FUNCTION__,__LINE__,support);
    } else if (strstr(name,"PreviewSize")){
        pCamInfo->mSoftInfo.mPreviewWidth = atoi(atts[1]);
        pCamInfo->mSoftInfo.mPreviewHeight = atoi(atts[3]);
        ALOGD("%s(%d): PreviewSize(%dx%d)! \n", __FUNCTION__,__LINE__,pCamInfo->mSoftInfo.mPreviewWidth,pCamInfo->mSoftInfo.mPreviewHeight);
    } else if (strstr(name,"DV")){
        ParserDVConfig(name, atts, userData);
    } else if (strstr(name,"Continue_SnapShot")){
        support = atoi(atts[1]);
        pCamInfo->mSoftInfo.mContinue_snapshot_config = support;
        ALOGD("%s(%d): Continue_SnapShot(%d)! \n", __FUNCTION__,__LINE__,support);
    }    
}

camera_board_profiles* camera_board_profiles::createInstance()
{
	FILE *fp = NULL;
    
    fp = fopen(RK_BOARD_XML_PATH, "r");
    if(!fp){
  	    ALOGD("open xml file(%s) failed\n", RK_BOARD_XML_PATH);
    }
    ALOGD("open xml file(%s) success\n", RK_BOARD_XML_PATH);

    camera_board_profiles *profiles = new camera_board_profiles();
    
    XML_Parser parser = XML_ParserCreate(NULL);
    if(parser==NULL){
        ALOGD("XML_ParserCreate failed\n");
        return NULL;
    }
    
    XML_SetUserData(parser, profiles);
    XML_SetElementHandler(parser, StartElementHandler, NULL);

    const int BUFF_SIZE = 512;
    for (;;) {
        void *buff = ::XML_GetBuffer(parser, BUFF_SIZE);
        if (buff == NULL) {
            ALOGD("failed to in call to XML_GetBuffer()");
            goto exit;
        }

        int bytes_read = ::fread(buff, 1, BUFF_SIZE, fp);
        if (bytes_read < 0) {
            ALOGD("failed in call to read");
            goto exit;
        }

        int res = XML_ParseBuffer(parser, bytes_read, bytes_read == 0);
        if(res!=1){
            ALOGD("XML_ParseBuffer error or susppend (%d)\n", res);
        }

        if (bytes_read == 0) break;  // done parsing the xml file
    }

exit:
    XML_ParserFree(parser);
    fclose(fp);

    size_t nCamDev2 = profiles->mDevieVector.size();
    ALOGD("number of camdevice (%d)\n", nCamDev2);
    
    size_t nDVnum2 = profiles->mCurDevice->mSoftInfo.mDV_vector.size();
    ALOGD("now DV size(%d)\n", nDVnum2);

    return profiles;

}

camera_board_profiles* camera_board_profiles::getInstance()
{
    camera_board_profiles *profiles = createInstance();

    return profiles;
}

bool camera_board_profiles::LoadALLCalibrationData(camera_board_profiles* profiles)
{
    size_t nCamDev2 = profiles->mDevieVector.size();
    unsigned int i=0;
    char filename[50];
    
    
    if(nCamDev2>=1){
        for(i=0; i<nCamDev2; i++)
        {
            rk_sensor_info *pSensorInfo = &(profiles->mDevieVector[i]->mHardInfo.mSensorInfo);
            
            CalibDb *pcalidb = &(profiles->mDevieVector[i]->mLoadSensorInfo.calidb);
            memset(filename, 0x00, 50);
            sprintf(filename, "%s.xml", pSensorInfo->mSensorName);
            bool res = pcalidb->CreateCalibDb(filename);
            if(res){
                ALOGD("load %s success\n", filename);
            }else{
                ALOGD("load %s failed\n", filename);
            }
        }
    }

    return true;
}

void camera_board_profiles::OpenAndRegistALLSensor(camera_board_profiles* profiles)
{
    size_t nCamDev2 = profiles->mDevieVector.size();
    unsigned int i=0;
    char filename[50];
    int err;    

    ALOGD("enter OpenAndRegistALLSensor\n");
    if(nCamDev2>=1){
        for(i=0; i<nCamDev2; i++)
        {         
            OpenAndRegistOneSensor(profiles->mDevieVector[i]);
        }
    }
     ALOGD("exit OpenAndRegistALLSensor\n");
}


int camera_board_profiles::OpenAndRegistOneSensor(rk_cam_total_info *pCamInfo)
{
    rk_sensor_info *pSensorInfo = &(pCamInfo->mHardInfo.mSensorInfo);
    camsys_load_sensor_info* pLoadSensorInfo = &(pCamInfo->mLoadSensorInfo);

    if(!pCamInfo)
        return RK_RET_NULL_POINTER;

    pCamInfo->mIsConnect = 0;
    
    sprintf(pLoadSensorInfo->mSensorLibName, "%s%s.so", RK_SENSOR_LIB_PATH, pSensorInfo->mSensorName);
    ALOGD("dlopen (%s) start\n", pLoadSensorInfo->mSensorLibName);    
    
    void *hSensorLib = dlopen( pLoadSensorInfo->mSensorLibName, RTLD_NOW/*RTLD_LAZY*/ );
    if ( NULL == hSensorLib )
    {
        ALOGD( "%s can't open the specified driver(%s)\n", __FUNCTION__, pLoadSensorInfo->mSensorLibName);
        ALOGD("dlopen err:%s.\n",dlerror()); 
        pLoadSensorInfo->mhSensorLib = NULL;
        pLoadSensorInfo->pCamDrvConfig = NULL;
        return RK_RET_NULL_POINTER;
    }

    IsiCamDrvConfig_t *pIsiCamDrvConfig = (IsiCamDrvConfig_t *)dlsym( hSensorLib, "IsiCamDrvConfig" );
    if ( NULL == pIsiCamDrvConfig )
    {
        ALOGD("%s (can't load sensor driver)\n", __FUNCTION__ );
        ALOGD("dlsym err:%s.\n",dlerror()); 
        if(hSensorLib)
            dlclose( hSensorLib );
        pLoadSensorInfo->mhSensorLib = NULL;
        pLoadSensorInfo->pCamDrvConfig = NULL;
        return RK_RET_NULL_POINTER;
    }

    // initialize function pointer
    if(pIsiCamDrvConfig->pfIsiGetSensorIss){
        if ( RET_SUCCESS != pIsiCamDrvConfig->pfIsiGetSensorIss( &(pIsiCamDrvConfig->IsiSensor) ) )
        {
            ALOGD("%s (IsiGetSensorIss failed)\n", __FUNCTION__ );
            return RK_RET_FUNC_FAILED;              
        }
    }else{
        ALOGD("%s ERROR(driver(%s) don't support IsiGetSensorIss)\n", __FUNCTION__,  pSensorInfo->mSensorName);
        return RK_RET_NULL_POINTER;   
    }

    pLoadSensorInfo->mhSensorLib = hSensorLib;
    pLoadSensorInfo->pCamDrvConfig = pIsiCamDrvConfig;
    ALOGD("dlopen success  sensor name(%s)\n", pIsiCamDrvConfig->IsiSensor.pszName);

    
    if(pIsiCamDrvConfig->pfIsiGetSensorI2cInfo){
        sensor_i2c_info_t* pI2cInfo;
        if(RET_SUCCESS != pIsiCamDrvConfig->pfIsiGetSensorI2cInfo(&pI2cInfo)){
        	ALOGE("GET I2C INFO ERRO !!!!!!!!!!!!!!!!");
            return RK_RET_FUNC_FAILED;
        }
        
        pCamInfo->mLoadSensorInfo.mpI2cInfo = pI2cInfo;
        //register i2c device 
        int err = RegisterSensorDevice(pCamInfo);
        if(!err)
        {
        	if(pIsiCamDrvConfig->IsiSensor.pIsiSensorCaps->SensorOutputMode == ISI_SENSOR_OUTPUT_MODE_RAW){
	            CalibDb *pcalidb = &(pCamInfo->mLoadSensorInfo.calidb);
	            sprintf(pLoadSensorInfo->mSensorXmlFile, "%s%s.xml", RK_SENSOR_XML_PATH, pSensorInfo->mSensorName);
	            bool res = pcalidb->CreateCalibDb(pLoadSensorInfo->mSensorXmlFile);
	           	ALOGD("-----------%s--------------",pLoadSensorInfo->mSensorXmlFile);
			    if(res){
	                ALOGD("load %s success\n", pLoadSensorInfo->mSensorXmlFile);
	                pCamInfo->mIsConnect = 1;
	                return RK_RET_SUCCESS;
	            }else{
	                ALOGD("load %s failed\n", pLoadSensorInfo->mSensorXmlFile);
	                return RK_RET_FUNC_FAILED;
	            }
    		}else{
                pCamInfo->mIsConnect = 1;
                return RK_RET_SUCCESS;
			}
        }else{
			return RK_RET_NOSETUP;
    	}
    }else{
        ALOGD("sensor(%s)'s driver don't have func pfIsiGetSensorI2cInfo\n", pSensorInfo->mSensorName);
        return RK_RET_NULL_POINTER;
    }
	
	return 0;
}

int camera_board_profiles::RegisterSensorDevice(rk_cam_total_info* pCamInfo)
{
    int err = RK_RET_SUCCESS,i; 
    camsys_sysctrl_t sysctl;
    camsys_devio_name_t extdev;
    camsys_i2c_info_t i2cinfo;
    camsys_querymem_t qmem1, qmem2;
    unsigned int *regbase=MAP_FAILED, *i2cbase=MAP_FAILED;
    unsigned int i2cbytes;
    struct rk_sensor_reg *sensor_reg;
    unsigned char *i2cchar;
    int camsys_fd=-1;
    int regist_ret=-1;
	int ret = RK_RET_SUCCESS;

    //for test 
    //return RK_RET_SUCCESS;
    
    rk_sensor_info *pSensorInfo = &(pCamInfo->mHardInfo.mSensorInfo);
    rk_vcm_info *pVcmInfo = &(pCamInfo->mHardInfo.mVcmInfo);
    camsys_load_sensor_info *pLoadInfo = &(pCamInfo->mLoadSensorInfo);
    sensor_i2c_info_t *pI2cInfo = pLoadInfo->mpI2cInfo;
    
    camsys_fd = open(pSensorInfo->mCamsysDevPath, O_RDWR);
    if (camsys_fd < 0) {
        ALOGD("Open (%s) failed, error=(%s)\n", pSensorInfo->mCamsysDevPath,strerror(errno));
        err = RK_RET_NOFILE;
        goto end;
    }    

    pCamInfo->mLoadSensorInfo.mCamsysFd = camsys_fd;
    extdev.dev_id = pSensorInfo->mCamDevid;
    strlcpy((char*)extdev.avdd.name, pSensorInfo->mAvdd.name,sizeof(extdev.avdd.name));
    //strlcpy((char*)extdev.avdd.name, pSensorInfo->mAvdd.name,2);
	extdev.avdd.min_uv = pSensorInfo->mAvdd.min_uv;
    extdev.avdd.max_uv = pSensorInfo->mAvdd.max_uv;
    strlcpy((char*)extdev.dovdd.name,pSensorInfo->mDovdd.name,sizeof(extdev.dovdd.name));
    extdev.dovdd.min_uv = pSensorInfo->mDovdd.min_uv;
    extdev.dovdd.max_uv = pSensorInfo->mDovdd.max_uv;
    strlcpy((char*)extdev.dvdd.name, pSensorInfo->mDvdd.name,sizeof(extdev.dvdd.name));
    extdev.dovdd.min_uv = pSensorInfo->mDvdd.min_uv;
    extdev.dovdd.max_uv = pSensorInfo->mDvdd.max_uv;
    strlcpy((char*)extdev.afvdd.name, pVcmInfo->mVcmVdd.name,sizeof(extdev.afvdd.name));
    extdev.afvdd.min_uv = pVcmInfo->mVcmVdd.min_uv;
    extdev.afvdd.max_uv = pVcmInfo->mVcmVdd.max_uv;
    
    strlcpy((char*)extdev.pwrdn.name, pSensorInfo->mSensorGpioPwdn.name,sizeof(extdev.pwrdn.name));
    extdev.pwrdn.active = pSensorInfo->mSensorGpioPwdn.active;
    strlcpy((char*)extdev.rst.name, pSensorInfo->mSensorGpioReset.name,sizeof(extdev.rst.name));
    extdev.rst.active = pSensorInfo->mSensorGpioReset.active;
    
    strlcpy((char*)extdev.pwren.name, pSensorInfo->SensorGpioPwen.name,sizeof(extdev.pwren.name));
    extdev.pwren.active = pSensorInfo->SensorGpioPwen.active;
    
    strlcpy((char*)extdev.afpwrdn.name, pVcmInfo->mVcmGpioPwdn.name,sizeof(extdev.afpwrdn.name));
    extdev.afpwrdn.active = pVcmInfo->mVcmGpioPwdn.active;
    strlcpy((char*)extdev.afpwr.name, pVcmInfo->mVcmGpioPower.name,sizeof(extdev.afpwr.name));
    extdev.afpwr.active = pVcmInfo->mVcmGpioPower.active;

    #if 1
    if(pSensorInfo->mPhy.type == CamSys_Phy_Cif){
        extdev.phy.type = CamSys_Phy_Cif;
        extdev.phy.info.cif.fmt = pSensorInfo->mPhy.info.cif.fmt;
        extdev.phy.info.cif.cif_num = pSensorInfo->mPhy.info.cif.cif_num;
    }else if(pSensorInfo->mPhy.type == CamSys_Phy_Mipi){
        extdev.phy.type = CamSys_Phy_Mipi;
        extdev.phy.info.mipi.data_en_bit = pSensorInfo->mPhy.info.mipi.data_en_bit;
    }else{
        ALOGE("%s %d: unknow phy type(%d)\n", pSensorInfo->mPhy.type);
    }
    #else
    extdev.phy.type = CamSys_Phy_Cif;
    extdev.phy.info.cif.fmt = 0;
    extdev.phy.info.cif.cif_num = CamSys_Fmt_Raw_10b;
    #endif
    
    extdev.clk.in_rate = pSensorInfo->mMclkRate;

	//oyyf before register sensor driver, check the kernel camsys version
	err = ioctl(camsys_fd, CAMSYS_VERCHK, &(pCamInfo->mCamsysVersion));
	if(!err){
		ALOGD("------------%s.%s(%d)  get camsys head version (%x), driver version(%x), now camerahal camsys head version(%x)----------\n", 
			__FILE__, __FUNCTION__,__LINE__,pCamInfo->mCamsysVersion.head_ver,pCamInfo->mCamsysVersion.drv_ver,CAMSYS_HEAD_VERSION);
	}else{
		ALOGE("-----------%s.%s(%d) get camsys head version failed! ---------\n", __FILE__, __FUNCTION__,__LINE__);
		goto regist_err;
	}
	
    regist_ret = ioctl(camsys_fd, CAMSYS_REGISTER_DEVIO, &extdev);
    if (regist_ret<0) {
        ALOGD("CAMSYS_REGISTER_DEVIO failed\n");
        ret = RK_RET_DEVICEERR;
        goto regist_err;
    }

    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_Avdd;
    sysctl.on = 1;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_Avdd on failed!\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }
    
    sysctl.ops = CamSys_Dovdd;
    sysctl.on = 1;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_Dovdd on failed!\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }
    usleep(5000);
    sysctl.dev_mask = (pSensorInfo->mHostDevid|pSensorInfo->mCamDevid); //need modify
    sysctl.ops = CamSys_ClkIn;
    sysctl.on = 1;

    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_ClkIn on failed\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }

    //1)power en
    usleep(1000);
    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_PwrEn;
    sysctl.on = 1;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_PwrDn on failed\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }

    //2)reset 
    usleep(1000);
    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_Rst;
    sysctl.on = 0;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_PwrDn on failed\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }
    //3)power down control
    usleep(1000);
    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_PwrDn;
    sysctl.on = 0;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_PwrDn on failed\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }
    usleep(2000);
    
    i2cinfo.bus_num = pSensorInfo->mSensorI2cBusNum;
    i2cinfo.slave_addr = pLoadInfo->mpI2cInfo->i2c_addr;
    i2cinfo.reg_addr = pLoadInfo->mpI2cInfo->soft_reg_addr; 
    i2cinfo.reg_size = pLoadInfo->mpI2cInfo->reg_size;
    i2cinfo.val = pLoadInfo->mpI2cInfo->soft_reg_value;
    i2cinfo.val_size = pLoadInfo->mpI2cInfo->value_size;
    i2cinfo.i2cbuf_directly = 0;
    i2cinfo.speed = pSensorInfo->mSensorI2cRate;
    
	ALOGD("******************CAMSYS_I2CRD******************\n"); 
	#if 1
	err = ioctl(camsys_fd, CAMSYS_I2CWR, &i2cinfo);
    if(err<0) {
        ALOGD("CAMSYS_I2CWR failed, soft reset fail, reg(0x%x) vale(0x%x)\n", i2cinfo.reg_addr, i2cinfo.val); 
		ret = RK_RET_DEVICEERR;
        goto power_off;
    }
	#else
	err = ioctl(camsys_fd, CAMSYS_I2CWR, &i2cinfo);
	while(err<0) {
			ALOGD("CAMSYS_I2CWR failed, soft reset fail, reg(0x%x) vale(0x%x)\n", i2cinfo.reg_addr, i2cinfo.val);
			err = ioctl(camsys_fd, CAMSYS_I2CWR, &i2cinfo);
	}
	#endif

    if(!ListEmpty(&(pI2cInfo->chipid_info))){
        List* l = ListHead( &(pI2cInfo->chipid_info) );
        while ( l )
        {
            sensor_chipid_info_t* pChipIDInfo = (sensor_chipid_info_t *)l;
            i2cinfo.reg_addr = pChipIDInfo->chipid_reg_addr; 

            err = ioctl(camsys_fd, CAMSYS_I2CRD, &i2cinfo);
            if (err<0) {
                ALOGD("CAMSYS_I2CRD failed\n");
            } else {
                ALOGD("WARNING: I2c read: addr(0x%x) : read(0x%x) default(0x%x)\n",i2cinfo.reg_addr, i2cinfo.val, pChipIDInfo->chipid_reg_value);
                if(i2cinfo.val!=pChipIDInfo->chipid_reg_value){
                    ret = RK_RET_DEVICEERR;
                    goto power_off;
                }
            }
            
            l = l->p_next;
        }
    }else{
        ALOGD("ERROR: sensor dirver don't have chip id info\n");
        ret = RK_RET_DEVICEERR;
        goto power_off;
    }

//  power off
power_off:
    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_PwrDn;
    sysctl.on = 1;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_PwrDn off failed\n");
        ret = RK_RET_DEVICEERR;
        
    }

    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_Rst;
    sysctl.on = 1;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_Rst off failed\n");
        ret = RK_RET_DEVICEERR;
        
    }

    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_PwrEn;
    sysctl.on = 0;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_PwrEn off failed\n");
        ret = RK_RET_DEVICEERR;
       
    }
    
    usleep(1000);
    sysctl.dev_mask = (pSensorInfo->mHostDevid|pSensorInfo->mCamDevid);
    sysctl.ops = CamSys_ClkIn;
    sysctl.on = 0;

    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_ClkIn off failed\n");
        ret = RK_RET_DEVICEERR;
        
    }

    usleep(2000);
    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_Avdd;
    sysctl.on = 0;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_Avdd off failed!\n");
        ret = RK_RET_DEVICEERR;
        
    }

    sysctl.dev_mask = pSensorInfo->mCamDevid;
    sysctl.ops = CamSys_Dovdd;
    sysctl.on = 0;
    err = ioctl(camsys_fd, CAMSYS_SYSCTRL, &sysctl);
    if (err<0) {
        ALOGD("CamSys_Dovdd off failed!\n");
        ret = RK_RET_DEVICEERR;
       
    }


unmap_pos:
    
regist_err: 
    if(regist_ret==0 && ret<0){
        // unregister device  need modify
        err = ioctl(camsys_fd, CAMSYS_DEREGISTER_DEVIO, &sysctl);
        if(err<0){
            ALOGD("CamSys_Dovdd off failed!\n");
            ret = RK_RET_DEVICEERR;
        }   
    }
    
    if(camsys_fd){
        close(camsys_fd);    
        camsys_fd = 0;
    }
   
end:
	return ret;   
}

int camera_board_profiles::CheckSensorSupportDV(rk_cam_total_info* pCamInfo)
{
    size_t nDvVector = pCamInfo->mSoftInfo.mDV_vector.size();
    
    if(nDvVector>=1){
        for(int i=0; i<nDvVector; i++){
            rk_DV_info *pDVInfo = pCamInfo->mSoftInfo.mDV_vector[i];

            if((pDVInfo->mResolution & pCamInfo->mLoadSensorInfo.mpI2cInfo->resolution) 
                && pDVInfo->mIsSupport){
                pDVInfo->mAddMask = 0;
                ALOGD("(%s) resolution(%dx%d) is support by sensor \n", pCamInfo->mHardInfo.mSensorInfo.mSensorName, pDVInfo->mWidth, pDVInfo->mHeight);
            }else{
                if(pDVInfo->mIsSupport)
                    pDVInfo->mAddMask = 0;
                else
                    pDVInfo->mAddMask = 1;
                ALOGD("NOTICE: (%s)  resolution(%dx%d) is scale or crop from other resolution\n", pCamInfo->mHardInfo.mSensorInfo.mSensorName, pDVInfo->mWidth, pDVInfo->mHeight);
            }
           
        }
    }else{
        ALOGD("WARNING: sensor(%s) don't support any DV resolution\n", pCamInfo->mHardInfo.mSensorInfo.mSensorName);
    }

    return 0;
}

int camera_board_profiles::WriteDevNameTOXML(camera_board_profiles* profiles, char *SrcFile, char* DstFile)
{   
    FILE *fpsrc, *fpdst; 
	char one_line_buf[256]; 
	char *leave_line0, *leave_line1, *leave_line2;
	int isWrite=0;
	int i;

    size_t nCamNum = profiles->mDevideConnectVector.size();
    
	fpsrc = fopen(SrcFile,"r"); 
	if(fpsrc == NULL) 
	{ 
		ALOGD("%s OPEN SrcMediaProfiles '%s' FALID, mode(read only), error(%s)\n", __FUNCTION__, SrcFile, strerror(errno)); 
		return -1; 
	} 
	
	fpdst = fopen(DstFile,"w"); 
	if(fpdst == NULL) 
	{ 
		ALOGD("%s OPEN DstMediaProfiles %s TEMP FALID, mode(w), error(%s)\n",__FUNCTION__, DstFile, strerror(errno)); 
		return -2; 
	} 

	fseek(fpsrc,0,SEEK_SET); 
	fseek(fpdst,0,SEEK_SET);	
	while(fgets(one_line_buf,256,fpsrc) != NULL) 
	{ 
		
		fputs(one_line_buf, fpdst);

		if(isWrite==0){
			leave_line0 = NULL;
			leave_line0 = strstr(one_line_buf, "<?");
			if(leave_line0==NULL){
				continue;
			}

			leave_line0 = NULL;
			leave_line0 = strstr(one_line_buf, "?>");
			if(leave_line0==NULL){
				continue;
			}

			for(i=0; (i<nCamNum && i<2); i++){
				fprintf(fpdst, "<!--  videoname%d=\"%s\" index=%d facing=%d -->  \n", 
                    i, profiles->mDevideConnectVector[i]->mHardInfo.mSensorInfo.mSensorName,
                    profiles->mDevideConnectVector[i]->mDeviceIndex, profiles->mDevideConnectVector[i]->mHardInfo.mSensorInfo.mFacing);
			}
			isWrite=1;	
		}
		
		if(fgetc(fpsrc)==EOF) 
		{ 
			break; 
		} 
		fseek(fpsrc,-1,SEEK_CUR); 		 
	}

	memset(one_line_buf,0,sizeof(one_line_buf));
	fclose(fpsrc);                 
	fclose(fpdst);  

	return 0;
}

int camera_board_profiles::ReadDevNameFromXML(FILE* fp, xml_DEV_name_s* video_name)
{
    char one_line_buf[256];
	char *leave_line0, *leave_line1, *leave_line2;
	int leave_num;
	char* equal_sign = "=";
	char* mark_sign_start = "<!--";
	char* mark_sign_end = "-->";
	char* videoname_sign = "videoname";
	xml_DEV_name_s* pst_video_name = video_name;
	int count = 0;
	
	fseek(fp,0,SEEK_SET);
	
	while(fgets(one_line_buf,256,fp) != NULL) 
	{ 
		if(strlen(one_line_buf) < 3) //line is NULL
		{ 
			continue; 
		} 
		leave_line0 = NULL;
		leave_line0 = strstr(one_line_buf, mark_sign_start);
		if(leave_line0==NULL)
		{
			continue;
		}
		leave_line1 = NULL;
		leave_line1 = strstr(one_line_buf, mark_sign_end);
		if(leave_line1==NULL)
		{
			continue;
		}

		leave_line0 = NULL;
		leave_line0 = strstr(one_line_buf, videoname_sign);
		if(leave_line0==NULL)
		{
			continue;
		}	

		leave_line1 = NULL;
		leave_line1 = strstr(leave_line0, equal_sign);
		if(leave_line1==NULL)
		{
			continue;
		}	

        ALOGD("%s\n", leave_line0);
		sscanf(leave_line0, "videoname%d=\"%[^\"]\" index=%d facing=%d", 
            &(pst_video_name->camid), pst_video_name->camera_name,
            &(pst_video_name->index), &(pst_video_name->facing));
		count++;
        pst_video_name++;
        
		if(count==2){
			break;
		}
				
		if(fgetc(fp)==EOF) 
		{ 
			break; 
		} 
		fseek(fp,-1,SEEK_CUR);		 
	}

	return count;
}

int camera_board_profiles::XMLFseekCamIDPos(FILE* fp, xml_fp_pos_s* fp_pos)
{
	char one_line_buf[256];
	int find_fmt_sign=0;
	char *leave_line, *leave_line1, *leave_line2;
	char str_camId[4];
	char *equal_sign = "="; 
	int count=0;
	
	if(fp==NULL)
		return -1;
	
	if(fp_pos==NULL)
		return -2;
		
	memset(str_camId, 0x00, sizeof(str_camId));
	sprintf(str_camId, "%d", fp_pos->camid);	
	fseek(fp,0,SEEK_SET); 
	while(fgets(one_line_buf,256,fp) != NULL) 
	{
		if(strlen(one_line_buf) < 3) //line is NULL
		{ 
			continue; 
		} 
		
		if(find_fmt_sign==0)
		{
			leave_line = NULL; 
			leave_line = strstr(one_line_buf, "<CamcorderProfiles"); 
			if(leave_line == NULL) //no "<CamcorderProfiles" 
			{ 
				continue; 
			} 
			
			leave_line1 = NULL; 
			leave_line1 = strstr(leave_line,equal_sign); 
			if(leave_line1 == NULL) //no "="
			{ 
				continue; 
			}
			
			leave_line2 = NULL; 
			leave_line2 = strstr(leave_line1,str_camId); 
			if(leave_line2 == NULL) //no "0/1"
			{ 
				continue; 
			}else{
				fp_pos->camid_start = ftell(fp);
				find_fmt_sign=1;
				continue;
			}
		}else{
			leave_line = NULL; 
			leave_line = strstr(one_line_buf, "</CamcorderProfiles>"); 
			if(leave_line == NULL) //no 
			{ 
				continue; 
			}else{
				fp_pos->camid_end = ftell(fp);
				break;
			}
		}
			
		if(fgetc(fp)==EOF) 
		{ 
			break; 
		} 
		fseek(fp,-1,SEEK_CUR); 
		memset(one_line_buf,0,sizeof(one_line_buf));
	}
	
	return 0;
}

int camera_board_profiles::FindResolution(camera_board_profiles* profiles, xml_video_element_s* find_element)
{
	int ret = -1;
    unsigned int j;
	size_t nCamNum = profiles->mDevideConnectVector.size();
    size_t nDVNum = 0; 

    ALOGD("find element camid(%d) quality(%s) width(%d)\n",find_element->n_cameraId, find_element->str_quality, find_element->n_width);
    if(find_element->n_cameraId < nCamNum){
        nDVNum = profiles->mDevideConnectVector[find_element->n_cameraId]->mSoftInfo.mDV_vector.size();
        for(j=0; j<nDVNum; j++){	
            rk_DV_info* DVInfo = profiles->mDevideConnectVector[find_element->n_cameraId]->mSoftInfo.mDV_vector[j];
    		if(DVInfo->mWidth==find_element->n_width)
    		{
    			find_element->n_height = DVInfo->mHeight;
    			find_element->n_frameRate = DVInfo->mFps;
    			find_element->isAddMark = DVInfo->mAddMask;
    			break;
    		}
        }
    }else{
        return -1;
    }
	
	if( j==nDVNum)
		return -1;
	else
		return 0;
}

int camera_board_profiles::ModifyMediaProfileXML( camera_board_profiles* profiles, char* src_xml_file, char* dst_xml_file)
{
	int ret=0, err=0;
	int alter_sign = 0;
	int find_fmt_sign=0;
	int leave_num=0;
	long now_fp_pos;
	char *equal_sign = "="; 
	FILE *src_fp=NULL, *dst_fp=NULL;
	char one_line_buf[256];
	char frontpart_line[50];
	long front_fptmp,back_fptmp;
    char *leave_line, *leave_line1, *leave_line2;
    
	xml_fp_pos_s fp_pos[2];  
	xml_video_element_s find_element;

	src_fp = fopen(src_xml_file, "r");
	if(src_fp==NULL){
		err = -1;
		ALOGD("open file '%s' failed!!! (r)\n", src_xml_file);
		goto alter_exit;
	}
	
	dst_fp = fopen(dst_xml_file, "w");
	if(dst_fp==NULL){
		err = -2;
		ALOGD("open file '%s' failed!!! (r)\n", dst_xml_file);
		goto alter_exit;
	}
	
	fp_pos[0].camid = 0;
    fp_pos[0].camid_start = 0;
    fp_pos[0].camid_end = 0;
	ret = XMLFseekCamIDPos(src_fp, &fp_pos[0]);
	if(ret < 0 || fp_pos[0].camid_end <= fp_pos[0].camid_start){
		ALOGD("find camid(%d) failed\n", fp_pos[0].camid);
		err = -3;
		goto alter_exit;	
	}
	
	fp_pos[1].camid = 1;
    fp_pos[1].camid_start = 0;
    fp_pos[1].camid_end = 0;
	ret = XMLFseekCamIDPos(src_fp, &fp_pos[1]);
	if(ret < 0 || fp_pos[1].camid_end <= fp_pos[1].camid_start){
		ALOGD("find camid(%d) failed\n", fp_pos[1].camid);
		err = -3;
		goto alter_exit;	
	}

    find_element.isAddMark = 1;
    find_element.n_cameraId = -1;
    find_element.n_frameRate = 0;
    find_element.n_width = 0;
    find_element.n_height = 0;
	if(fp_pos[0].camid_end>0 && fp_pos[0].camid_start>0 && fp_pos[1].camid_end>0 && fp_pos[1].camid_start>0){
		fseek(src_fp,0,SEEK_SET); 
		fseek(dst_fp,0,SEEK_SET);
			
		while(fgets(one_line_buf,256,src_fp) != NULL) 
		{ 
			if(strlen(one_line_buf) < 3) //line is NULL
			{ 
				fputs(one_line_buf, dst_fp);
				continue; 
			} 
							
			if(find_fmt_sign==0)
			{		
				leave_line = NULL; 
				leave_line = strstr(one_line_buf,equal_sign); 
				if(leave_line == NULL) //no "="
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				} 
				leave_line1 = NULL; 
				leave_line1 = strstr(one_line_buf, "<EncoderProfile"); 
				if(leave_line1 == NULL) 
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				} 
							
				leave_line2 = NULL; 
				leave_line2 = strstr(leave_line1, "timelapse"); 
				if(leave_line2 == NULL) 
				{ 
					memset(find_element.str_quality, 0x00, sizeof(find_element.str_quality));
					sscanf(leave_line, "%*[^\"]\"%[^\"]", find_element.str_quality);
				}else{
					memset(find_element.str_quality, 0x00, sizeof(find_element.str_quality));
					sscanf(leave_line, "%*[^\"]\"timelapse%[^\"]", find_element.str_quality);
				} 
								
				//ALOGD("quality %s\n", find_element.str_quality);			
				find_fmt_sign = 1;
				front_fptmp = ftell(dst_fp);
				fprintf(dst_fp, "     \n");	
				fputs(one_line_buf, dst_fp);
				continue; 
			}
			else if(find_fmt_sign==1)
			{		
				leave_line = NULL; 
				leave_line = strstr(one_line_buf,equal_sign); 
				if(leave_line == NULL) //no "="
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				}
							
				leave_line1 = NULL; 
				leave_line1 = strstr(one_line_buf,"width"); 
				if(leave_line1 == NULL) //no "width"
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				} 
		 		sscanf(leave_line, "%*[^1-9]%d\"", &(find_element.n_width));
				//ALOGD("%d\n", find_element.n_width);
				find_fmt_sign=2;
				fputs(one_line_buf, dst_fp);
				continue; 	 
			}
			else if(find_fmt_sign==2)
			{		
				leave_line = NULL; 
				leave_line = strstr(one_line_buf,equal_sign); 
				if(leave_line == NULL) //no "="
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				}
				leave_line1 = NULL; 
				leave_line1 = strstr(one_line_buf, "frameRate"); 
				if(leave_line1 == NULL) //no "framRate"
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				} 
			
				now_fp_pos = ftell(src_fp);
				if(now_fp_pos>fp_pos[0].camid_start && now_fp_pos<fp_pos[0].camid_end)
					find_element.n_cameraId = 0;
				else if(now_fp_pos>fp_pos[1].camid_start && now_fp_pos<fp_pos[1].camid_end)
					find_element.n_cameraId = 1;
				else
					find_element.n_cameraId = -1;
				
				if(find_element.n_cameraId != -1){
					ret = FindResolution(profiles, &find_element);
					if(ret==0){
						leave_num = leave_line - one_line_buf;
						memset(frontpart_line,0,sizeof(frontpart_line)); 
						strncpy(frontpart_line,one_line_buf,leave_num);  
						fputs(frontpart_line,dst_fp);
											
						//ALOGD("new frameRate %d  isaddmark(%d)\n", find_element.n_frameRate, find_element.isAddMark);
						fprintf(dst_fp,"=\"%d\" /> \n", (find_element.n_frameRate)); 
						alter_sign++; 
						find_fmt_sign = 3;	
						ALOGD("XML modify: camID(%d) resolution:%s(%dx%d) fps(%d) isaddmark(%d)\n",find_element.n_cameraId,find_element.str_quality, 
							find_element.n_width, find_element.n_height, find_element.n_frameRate, find_element.isAddMark);
					}else{
						ALOGD("WARNING: can't find camID(%d) resolution:%s(%dx), addmark!!!\n", find_element.n_cameraId,find_element.str_quality, find_element.n_width);
						find_element.isAddMark=1;
						find_fmt_sign = 3;
						fputs(one_line_buf, dst_fp);
						//continue;
					}
				}else{
					find_fmt_sign = 3;
					fputs(one_line_buf, dst_fp);
					continue;
				}
			}else if(find_fmt_sign==3){
				leave_line = NULL; 
				leave_line = strstr(one_line_buf,"</EncoderProfile>"); 
				if(leave_line == NULL) //no "framRate"
				{ 
					fputs(one_line_buf, dst_fp);
					continue; 
				} 
				fputs(one_line_buf, dst_fp);	
				if(find_element.isAddMark){
					back_fptmp = ftell(dst_fp);
					fseek(dst_fp,front_fptmp,SEEK_SET); 
					fprintf(dst_fp, "<!--  \n");
					fseek(dst_fp,back_fptmp,SEEK_SET); 
					fprintf(dst_fp, "-->  \n");
					find_element.isAddMark=0;
				}
				find_fmt_sign=0;
			}
		
			if(fgetc(src_fp)==EOF) 
			{ 
				break; 
			} 
			fseek(src_fp,-1,SEEK_CUR); 
			memset(one_line_buf,0,sizeof(one_line_buf)); 
		} 
	}
	
alter_exit:
    if(src_fp)
	    fclose(src_fp); 

    if(dst_fp)
	    fclose(dst_fp);

	if(err==0){
		//remove(src_xml_file);    
		//chmod(src_xml_file, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
        remove(src_xml_file);
		chmod(dst_xml_file, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
	}
	return err;
}


int camera_board_profiles::ProduceNewXml(camera_board_profiles* profiles)
{
    char temp_dst_file[50];
    char dst_file[50];
    char default_file[50];
    int err=0;

    //CheckSensorSupportDV
    AddConnectSensorToVector(profiles);
    size_t nCamNum =profiles->mDevideConnectVector.size();
    
    if(nCamNum>=1){ 
        ALOGD("enter produce new xml\n");
        //new xml file name
        strncpy(default_file, RK_DEFAULT_MEDIA_PROFILES_XML_PATH, sizeof(default_file));
        strncpy(dst_file, RK_DST_MEDIA_PROFILES_XML_PATH, sizeof(dst_file));
        strncpy(temp_dst_file, RK_TMP_MEDIA_PROFILES_XML_PATH, sizeof(temp_dst_file));

        for(int i=0; i<nCamNum; i++){
            profiles->mDevideConnectVector[i]->mLoadSensorInfo.mpI2cInfo->resolution = 0;
            CheckSensorSupportDV(profiles->mDevideConnectVector[i]);
        }
        
        //write name to xml 
        err = WriteDevNameTOXML(profiles, default_file, temp_dst_file);
        if(err){
            ALOGD("write dev name to xml failed\n");
            goto end;
        }

        //modify xml
        err = ModifyMediaProfileXML( profiles, temp_dst_file, dst_file);
        if(err){
            ALOGD("modify xml failed\n");
            goto end;
        }

        ALOGD("exit produce new xml\n");
    }

end:
    return err;
    
}

int camera_board_profiles::LoadSensor(camera_board_profiles* profiles)
{    
    char dst_file[50];
    int err = RK_RET_SUCCESS;
    int count = 0;
	int result= 0;

    ALOGD("enter Load Sensor\n");
    strncpy(dst_file, RK_DST_MEDIA_PROFILES_XML_PATH, sizeof(dst_file));
    ALOGD("read cam name from xml(%s)\n",dst_file );

    FILE* fp = fopen(dst_file, "r");
    if(!fp){
        ALOGD(" is not exist, register all\n");
        goto err_end;
    }

    //read sensor name
    xml_DEV_name_s media_xml_device[2];
    memset(&media_xml_device, 0x00, sizeof(xml_DEV_name_s));
    count = ReadDevNameFromXML(fp, media_xml_device);
    if(count<1){
        ALOGD("media_profiles.xml not have any camera device\n");
        goto err_end;
    }

    ALOGD("find camera count(%d) cam1(%s)\n", count, media_xml_device[0].camera_name);
    //verrify media_xml_device is supported by board xml 
    for(int i=0; (i<count && i<2); i++)
    {
        err |= BoardFileHaveDev(profiles, (media_xml_device+i));
    }

    if(err != RK_RET_SUCCESS){
        goto err_end;
    }

    //register exist sensor
    for(int i=0; (i<count && i<2); i++){
        result = OpenAndRegistOneSensor(profiles->mDevieVector[media_xml_device[i].index]);
		if(result != 0){
			goto err_end;
		}else{
        	profiles->mDevieVector[media_xml_device[i].index]->mIsConnect = 1;
		}
       
    }

	if(profiles->mDevieVector.size()>0){
		AddConnectSensorToVector(profiles);
		return RK_RET_SUCCESS;
	}else
		return RK_RET_NOSETUP;

err_end:
    OpenAndRegistALLSensor(profiles);
    ProduceNewXml(profiles);
    ALOGD("enter Load Sensor\n");
    return err;
    
}

int camera_board_profiles::BoardFileHaveDev(camera_board_profiles* profiles, xml_DEV_name_s* media_xml_device )
{
    size_t nCamNum = profiles->mDevieVector.size();

    ALOGD("BoardFileHaveDev enter \n");
    if(media_xml_device->index < nCamNum)
    {
        rk_sensor_info *pSensorInfo =  &(profiles->mDevieVector[media_xml_device->index]->mHardInfo.mSensorInfo);
        if(!strcmp(media_xml_device->camera_name, pSensorInfo->mSensorName) 
            && (media_xml_device->facing == pSensorInfo->mFacing))
        {
            return RK_RET_SUCCESS;
        }  
    }

    ALOGD("BoardFileHaveDev exit \n");
    return RK_RET_NOSETUP;
}

void camera_board_profiles::AddConnectSensorToVector(camera_board_profiles* profiles)
{
    size_t nCamNum = profiles->mDevieVector.size();

    for(int i=0; i<nCamNum; i++)
    {
        if(profiles->mDevieVector[i]->mIsConnect == 1)
        {
            profiles->mDevideConnectVector.add(profiles->mDevieVector[i]);
        }
    }
}


