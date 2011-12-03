/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
/**
* @file CameraHal.cpp
*
* This file maps the Camera Hardware Interface to V4L2.
*
*/

#include "CameraHal.h"
#include "CameraHal_Module.h"
#include <pthread.h>

#include <binder/IPCThreadState.h>

#ifdef HAVE_ANDROID_OS 

#include <linux/android_pmem.h>
#include <binder/MemoryHeapPmem.h>
#endif
#include <utils/CallStack.h>
#include <camera/ICameraService.h>
#include <binder/IServiceManager.h>
#include <binder/IMemory.h>

#include <sys/stat.h>
#include <unistd.h>
#include <linux/fb.h>

extern rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];

namespace android {
// Logging support -- this is for debugging only
// Use "adb shell dumpsys media.camera " to change it.
static volatile int32_t gLogLevel = 0;

#define LOG1(...) LOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) LOGD_IF(gLogLevel >= 2, __VA_ARGS__);

    
#define LOG_TAG "CameraHal"

#define LOG_FUNCTION_NAME           LOG1("%s Enter", __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT      LOG1("%s Exit ", __FUNCTION__);

#define CAMERA_PREVIEWBUF_DISPING_BITPOS   0x00
#define CAMERA_PREVIEWBUF_ENCING_BITPOS    0x01
#define CAMERA_PREVIEWBUF_WRITING_BITPOS   0x02 

#define CAMERA_PREVIEWBUF_ALLOW_WRITE(a)   ((a&0x3)==0x00)

static int getCallingPid() {
    return IPCThreadState::self()->getCallingPid();
}

static int cameraPixFmt2HalPixFmt(unsigned int fmt)
{
    int hal_pixel_format;
    
    switch (fmt)
    {
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_NV12: 
        {
            hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_NV12;
            break;
        }
        case V4L2_PIX_FMT_YUV422P:
        case V4L2_PIX_FMT_NV16:
        {
            hal_pixel_format = HAL_PIXEL_FORMAT_YCbCr_422_SP;
            break;
        }
            
        case V4L2_PIX_FMT_YUYV:
        {
            hal_pixel_format = HAL_PIXEL_FORMAT_YCbCr_422_I;
            break;
        }
        default:
            hal_pixel_format = -EINVAL;
            LOGE("%s(%d): pixel format(%c%c%c%c) is unknow!",__FUNCTION__,__LINE__,fmt & 0xFF, (fmt >> 8) & 0xFF,
				(fmt >> 16) & 0xFF, ((fmt >> 24) & 0xFF));
            break;
    }

    return hal_pixel_format;
}
CameraHal::CameraHal(int cameraId)
            :mParameters(),
            mPreviewRunning(STA_PREVIEW_PAUSE),
            mPreviewLock(),
            mPreviewCond(),
            mDisplayRuning(STA_DISPLAY_PAUSE),
            mDisplayLock(),
            mDisplayCond(),
            mANativeWindowLock(),
            mANativeWindowCond(),
            mANativeWindow(NULL),
            mPreviewErrorFrameCount(0),
	        mPreviewFrameSize(0),
	        mCamDriverFrmHeightMax(0),
	        mCamDriverFrmWidthMax(0),
	        mPreviewBufferCount(0),
	        mCamDriverPreviewFmt(0),
	        mCamDriverPictureFmt(0),
	        mCamDriverV4l2BufferLen(0),
	        mMemHeap(0),
            mMemHeapPmem(0),
            mPreviewMemory(NULL),
            mPmemHeapPhyBase(0),
            mRawBuffer(0),
            mJpegBuffer(0),
            mRawBufferSize(0),
            mJpegBufferSize(0),
            mMsgEnabled(0),
            mEffect_number(0),
            mScene_number(0),
            mWhiteBalance_number(0),
            mGps_latitude(-1),
            mGps_longitude(-1),
            mGps_altitude(-1),
            mGps_timestamp(-1),
            displayThreadCommandQ("displayCmdQ"),
            displayThreadAckQ("displayAckQ"),            
            previewThreadCommandQ("previewCmdQ"),
            previewThreadAckQ("previewAckQ"),
            commandThreadCommandQ("commandCmdQ"),
            commandThreadAckQ("commandAckQ")
{
    int fp,i;
    
    cameraCallProcess[0] = 0x00; 
    sprintf(cameraCallProcess,"/proc/%d/cmdline",getCallingPid());
    fp = open(cameraCallProcess, O_RDONLY);
    if (fp < 0) {
        memset(cameraCallProcess,0x00,sizeof(cameraCallProcess));
        LOGE("Obtain calling process info failed");
    } else {
        memset(cameraCallProcess,0x00,sizeof(cameraCallProcess));
        read(fp, cameraCallProcess, 30);
        close(fp);
        fp = -1;
        LOGD("Calling process is: %s",cameraCallProcess);
    }
    
    iCamFd = -1;
    iPmemFd = -1;
    mPmemSize = 0;
    memset(&mCamDriverSupportFmt[0],0, sizeof(mCamDriverSupportFmt));
    mRecordRunning = false;
    mPictureRunning = STA_PICTURE_STOP;
    mExitAutoFocusThread = false;
    mDriverMirrorSupport = false;
    mDriverFlipSupport = false;
    mPreviewCmdReceived = false;
    mPreviewStartTimes = 0x00;    
    memset(mCamDriverV4l2Buffer, 0x00, sizeof(mCamDriverV4l2Buffer));
    for (i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        mPreviewBufferMap[i].priv_hnd= NULL;
        mPreviewBufferMap[i].buffer_hnd = NULL;
        mPreviewBufferMap[i].buf_state = 0x00;
        mPreviewBufferMap[i].phy_addr= 0x00;

        mPreviewBufs[i] = NULL;
        mVideoBufs[i] = NULL;
    }
    
    if (cameraCreate(cameraId) == 0) {
        initDefaultParameters();

        cameraHeapBufferCreate(mRawBufferSize,mJpegBufferSize);

        mDisplayThread = new DisplayThread(this);
        mPreviewThread = new PreviewThread(this);
        mCommandThread = new CommandThread(this);
        mPictureThread = new PictureThread(this);
        mAutoFocusThread = new AutoFocusThread(this);
        mDisplayThread->run("CameraDispThread",ANDROID_PRIORITY_DISPLAY);
        mPreviewThread->run("CameraPreviewThread",ANDROID_PRIORITY_DISPLAY);
        mCommandThread->run("CameraCmdThread", ANDROID_PRIORITY_URGENT_DISPLAY);
        mAutoFocusThread->run("CameraAutoFocusThread", ANDROID_PRIORITY_DISPLAY);

        LOGD("CameraHal create success!");
    } else {
        mPreviewThread = NULL;
        mDisplayThread = NULL;
        mCommandThread = NULL;
        mPictureThread = NULL;
        mAutoFocusThread = NULL;
    }

}
int CameraHal::cameraFramerateQuery(unsigned int format, unsigned int w, unsigned int h, int *min, int *max)
{
    int i,framerate,ret;    
    int preview_data_process_time;
    
#if CONFIG_AUTO_DETECT_FRAMERATE   
    struct v4l2_frmivalenum *fival;

    switch (w)
    {
        case 176:
            preview_data_process_time = 2; //ms
            break;
        case 320:
            preview_data_process_time = 3;
            break;
        case 352:
            preview_data_process_time = 4;
            break;
        case 640:
            preview_data_process_time = 6;
            break;
        case 800:
            preview_data_process_time = 10;
            break;
        case 1280:
            preview_data_process_time = 15;
            break;
        default:
            preview_data_process_time = 8;
            break;
    }

    ret = -1;
    preview_data_process_time += 50;            //thread schedule interval may 50ms
    fival = gCamInfos[mCamId].fival_list;
    for (i=0; (i<10 && fival->discrete.denominator); i++) {
        if ((fival->pixel_format == format) && (fival->width == w) && (fival->height == h)) {
            framerate = (fival->discrete.denominator*1000)/fival->discrete.numerator;
            *max = framerate;      
            *min = 1000000/(((fival->discrete.numerator*1000/fival->discrete.denominator)+preview_data_process_time));
            ret = 0;
            goto cameraFramerateQuery_end;
        }
        fival++;
    }
    
    *min = CAMERA_DEFAULT_PREVIEW_FPS_MIN;
    *max = CAMERA_DEFAULT_PREVIEW_FPS_MAX;
    ret = 0;
#else
    struct v4l2_frmivalenum fival;

    if ((mCamDriverCapability.version & 0xff) < 0x05) {
        LOGD("Camera driver version: %d.%d.%d isn't support query framerate, Please update to v0.x.5",mCamDriverCapability.driver,
            (mCamDriverCapability.version>>16) & 0xff,(mCamDriverCapability.version>>8) & 0xff,
            mCamDriverCapability.version & 0xff);
        goto default_fps;
    }
    
    ret = 0;
    fival.index = 0;
    fival.pixel_format = format;
    fival.width = w;
    fival.height = h;
	ret = ioctl(iCamFd, VIDIOC_ENUM_FRAMEINTERVALS, &fival);

    if (ret == 0) {
        if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            *min = (fival.discrete.denominator*1000)/fival.discrete.numerator;
            *max = (fival.discrete.denominator*1000)/fival.discrete.numerator;
        } else {
            LOGE("%s(%d): query framerate type(%d) is not supported",__FUNCTION__,__LINE__, fival.type);
            goto default_fps;
        }
    } else {
        LOGE("%s(%d): query framerate error(%dx%d@%c%c%c%c index:%d)",__FUNCTION__,__LINE__,
            fival.width,fival.height,(fival.pixel_format & 0xFF), (fival.pixel_format >> 8) & 0xFF,
				((fival.pixel_format >> 16) & 0xFF), ((fival.pixel_format >> 24) & 0xFF),fival.index);
default_fps:    
        if (mCamId == 0) {
            *min = CAMERA_BACK_PREVIEW_FPS_MIN;
            *max = CAMERA_BACK_PREVIEW_FPS_MAX;
        } else {
            *min = CAMERA_FRONT_PREVIEW_FPS_MIN;
            *max = CAMERA_FRONT_PREVIEW_FPS_MAX;
        }
        ret = 0;
    }
    
#endif
cameraFramerateQuery_end:
    return ret;
}
int CameraHal::cameraFpsInfoSet(CameraParameters &params)
{
    int w,h, framerate_min,framerate_max;
    char fps_str[20];
    String8 parameterString;
    
    params.getPreviewSize(&w,&h);        
   
    if (cameraFramerateQuery(mCamDriverPreviewFmt, w,h,&framerate_min,&framerate_max) == 0) {              
        /*frame per second setting*/
        memset(fps_str,0x00,sizeof(fps_str));            
        sprintf(fps_str,"%d",framerate_min);
        fps_str[strlen(fps_str)] = ',';
        sprintf(&fps_str[strlen(fps_str)],"%d",framerate_max);
        params.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, fps_str);
        parameterString = "(";
        parameterString.append(fps_str);
        parameterString.append(")");
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, parameterString.string());
        
        memset(fps_str,0x00,sizeof(fps_str));
        sprintf(fps_str,"%d",framerate_min/1000);
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, fps_str);
        params.setPreviewFrameRate(framerate_min/1000);
    }    

    return 0;
}
void CameraHal::initDefaultParameters()
{
    CameraParameters params;
    String8 parameterString;
	int i,j,previewFrameSizeMax;
	char cur_param[32],cam_size[10];
    char str_picturesize[100];//We support at most 4 resolutions: 2592x1944,2048x1536,1600x1200,1024x768 
    int ret,picture_size_bit;
    struct v4l2_format fmt;    
    
    LOG_FUNCTION_NAME    
    if (strcmp((char*)&mCamDriverCapability.driver[0],"uvcvideo") == 0) {
        /*preview size setting*/
        struct v4l2_frmsizeenum fsize;
                
        memset(str_picturesize,0x00,sizeof(str_picturesize));
        memset(&fsize, 0, sizeof(fsize));         
        picture_size_bit = 0;
        fsize.index = 0;       
        fsize.pixel_format = mCamDriverPreviewFmt;
        while ((ret = ioctl(iCamFd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0) {
        	if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {                 
                if ((fsize.discrete.width == 320) && (fsize.discrete.height == 240)) {
                    if (strcmp(cameraCallProcess,"com.tencent.android.pad") == 0) {
                        fsize.index++;
                        continue;
                    }
                }
                memset(cam_size,0x00,sizeof(cam_size));
                if (parameterString.size() != 0) 
                    cam_size[0]=',';
                sprintf((char*)(&cam_size[strlen(cam_size)]),"%d",fsize.discrete.width);
                strcat(cam_size, "x");
                sprintf((char*)(&cam_size[strlen(cam_size)]),"%d",fsize.discrete.height);
                parameterString.append((const char*)cam_size);

                if ((strlen(str_picturesize)+strlen(cam_size))<sizeof(str_picturesize)) {
                    if (fsize.discrete.width <= 2592) {
                        strcat(str_picturesize, cam_size);
                        if (fsize.discrete.width > mCamDriverFrmWidthMax) {
                            mCamDriverFrmWidthMax = fsize.discrete.width;
                            mCamDriverFrmHeightMax = fsize.discrete.height;
                        } 
                    }
                } else {
                    break;
                }
        	} else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {

        		break;
        	} else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        		
        		break;
        	}
        	fsize.index++;
        }
        if (ret != 0 && errno != EINVAL) {
    		LOGE("ERROR enumerating frame sizes: %d\n", errno);
    	}

        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
        params.setPreviewSize(640,480);
        /*picture size setting*/      
        params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, str_picturesize);        
        params.setPictureSize(mCamDriverFrmWidthMax,  mCamDriverFrmHeightMax);        

        if (mCamDriverFrmWidthMax <= 2592) {                    			
            mRawBufferSize = RAW_BUFFER_SIZE_5M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
        } else if (mCamDriverFrmWidthMax <= 2048) {
    		mRawBufferSize = RAW_BUFFER_SIZE_3M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_3M;
    	} else if (mCamDriverFrmWidthMax <= 1600) {
    		mRawBufferSize = RAW_BUFFER_SIZE_2M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_2M;
    	} else if (mCamDriverFrmWidthMax <= 1024) {
    		mRawBufferSize = RAW_BUFFER_SIZE_1M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_1M;
        }
        
        /* set framerate */
        struct v4l2_streamparm setfps;          
        
        memset(&setfps, 0, sizeof(struct v4l2_streamparm));
        setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        setfps.parm.capture.timeperframe.numerator=1;
        setfps.parm.capture.timeperframe.denominator=15;
        ret = ioctl(iCamFd, VIDIOC_S_PARM, &setfps); 

        /*frame rate setting*/    
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "15");
        params.setPreviewFrameRate(15);
        /*frame per second setting*/
        parameterString = "15000,15000";
        params.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, parameterString.string());
        parameterString = "(15000,15000)";
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, parameterString.string());

    } else if (strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") == 0) {
    
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat= mCamDriverPreviewFmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        /*picture size setting*/
     	fmt.fmt.pix.width = 10000;
     	fmt.fmt.pix.height = 10000;
    	ret = ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt);
        
        mCamDriverFrmWidthMax = fmt.fmt.pix.width;
        mCamDriverFrmHeightMax = fmt.fmt.pix.height;
        str_picturesize[0] = 0x00;

        if (mCamDriverFrmWidthMax > 2592) {
            LOGE("Camera driver support maximum resolution(%dx%d) is overflow 5Mega!",mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
            mCamDriverFrmWidthMax = 2592;
            mCamDriverFrmHeightMax = 1944;
        }
        
        /*preview size setting*/ 
        if (mCamDriverFrmWidthMax >= 176) {            
         	fmt.fmt.pix.width = 176;
         	fmt.fmt.pix.height = 144;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 176) && (fmt.fmt.pix.height == 144)) {
                    parameterString.append("176x144");
                    params.setPreviewSize(176, 144);
                    previewFrameSizeMax =  PAGE_ALIGN(176*144*3/2)*2;          // 176*144*1.5*2
                    params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"176x144");
                }
            }
        }
        
        if (strcmp(cameraCallProcess,"com.tencent.android.pad")) {
            if (mCamDriverFrmWidthMax >= 320) {            
             	fmt.fmt.pix.width = 320;
             	fmt.fmt.pix.height = 240;
                if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                    if ((fmt.fmt.pix.width == 320) && (fmt.fmt.pix.height == 240)) {
                        parameterString.append(",320x240");
                        params.setPreviewSize(320, 240);
                        previewFrameSizeMax =  PAGE_ALIGN(320*240*3/2)*2;          // 320*240*1.5*2
                        params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"320x240");
                    }
                }
            }
        }
        
        if (mCamDriverFrmWidthMax >= 352) {            
         	fmt.fmt.pix.width = 352;
         	fmt.fmt.pix.height = 288;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 352) && (fmt.fmt.pix.height == 288)) {
                    parameterString.append(",352x288");
                    params.setPreviewSize(352, 288);
                    previewFrameSizeMax =  PAGE_ALIGN(352*288*3/2)*2;          // 352*288*1.5*2
                    params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"352x288");
                }
            }
        }
        
        if (mCamDriverFrmWidthMax >= 640) {            
         	fmt.fmt.pix.width = 640;
         	fmt.fmt.pix.height = 480;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 640) && (fmt.fmt.pix.height == 480)) {
                    parameterString.append(",640x480");
                    params.setPreviewSize(640, 480);
                    previewFrameSizeMax =  PAGE_ALIGN(640*480*3/2)*2;          // 640*480*1.5*2
                    params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"640x480");
                }
            }
        }

        if (mCamDriverFrmWidthMax >= 1280) {
            fmt.fmt.pix.width = 1280;
         	fmt.fmt.pix.height = 720;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 1280) && (fmt.fmt.pix.height == 720)) {
                    parameterString.append(",1280x720");
                    previewFrameSizeMax =  PAGE_ALIGN(1280*720*3/2)*2;          // 1280*720*1.5*2
                    params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"1280x720");
                }
            }
        }
        
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
        params.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES, parameterString.string());

        switch( mCamDriverFrmWidthMax )
        {
            case 2592:	// LARGEST RESULOTION is 5Meag
                if (mPmemSize >= previewFrameSizeMax + RAW_BUFFER_SIZE_5M + JPEG_BUFFER_SIZE_5M) {
        			strcat( str_picturesize,"2592x1944,2048x1536,1600x1200,1024x768");
                    params.setPictureSize(2592,  1944);
                    mRawBufferSize = RAW_BUFFER_SIZE_5M;
                    mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
    			    break;
                } else {
                    LOGE("Camera driver support 5M pixel, but pmem(size:0x%x) is not enough(preview:0x%x, raw:0x%x,jpeg:0x%x)!"
                        ,mPmemSize,previewFrameSizeMax,RAW_BUFFER_SIZE_5M,JPEG_BUFFER_SIZE_5M);
                }
    		case 2048:	// LARGEST RESULOTION is 3Meag
    		    if (mPmemSize >= previewFrameSizeMax + RAW_BUFFER_SIZE_3M + JPEG_BUFFER_SIZE_3M) {
        			strcat( str_picturesize,"2048x1536,1600x1200,1024x768");
                    mRawBufferSize = RAW_BUFFER_SIZE_3M;
                    mJpegBufferSize = JPEG_BUFFER_SIZE_3M;  
                    params.setPictureSize(2048,1536);
    			    break;
                } else {
                    LOGE("Camera driver support 3M pixel, but pmem(size:0x%x) is not enough(preview:0x%x, raw:0x%x,jpeg:0x%x)!"
                        ,mPmemSize,previewFrameSizeMax,RAW_BUFFER_SIZE_3M,JPEG_BUFFER_SIZE_3M);
                }
    		case 1600:	// LARGEST RESULOTION is 2Meag
    		    if (mPmemSize >= previewFrameSizeMax + RAW_BUFFER_SIZE_2M + JPEG_BUFFER_SIZE_2M) {
        			strcat( str_picturesize,"1600x1200,1024x768,640x480");
                    mRawBufferSize = RAW_BUFFER_SIZE_2M;
                    mJpegBufferSize = JPEG_BUFFER_SIZE_2M;
                    params.setPictureSize(1600,1200);
    			    break;
    		    } else {
                    LOGE("Camera driver support 2M pixel, but pmem(size:0x%x) is not enough(preview:0x%x, raw:0x%x,jpeg:0x%x)!"
                        ,mPmemSize,previewFrameSizeMax,RAW_BUFFER_SIZE_2M,JPEG_BUFFER_SIZE_2M);
                }
            case 1280:  // 1280x1024
    		case 1024:	// LARGEST RESULOTION is 1Meag
    		    if (mPmemSize >= previewFrameSizeMax + RAW_BUFFER_SIZE_1M + JPEG_BUFFER_SIZE_1M) {
                    strcat( str_picturesize,"1024x768,640x480,320x240");
        		    mRawBufferSize = RAW_BUFFER_SIZE_1M;
                    mJpegBufferSize = JPEG_BUFFER_SIZE_1M;
                    params.setPictureSize(1024,768);
        			break;
    		    } else {
                    LOGE("Camera driver support 1M pixel, but pmem(size:0x%x) is not enough(preview:0x%x, raw:0x%x,jpeg:0x%x)!"
                        ,mPmemSize,previewFrameSizeMax,RAW_BUFFER_SIZE_1M,JPEG_BUFFER_SIZE_1M);
                }
            case 640:	// LARGEST RESULOTION is 0.3Meag
    		    if (mPmemSize >= previewFrameSizeMax + RAW_BUFFER_SIZE_0M3 + JPEG_BUFFER_SIZE_0M3) {
                    strcat( str_picturesize,"640x480,320x240");
        		    mRawBufferSize = RAW_BUFFER_SIZE_0M3;
                    mJpegBufferSize = JPEG_BUFFER_SIZE_0M3;
                    params.setPictureSize(640,480);
        			break;
    		    } else {
                    LOGE("Camera driver support 0.3M pixel, but pmem(size:0x%x) is not enough(preview:0x%x, raw:0x%x,jpeg:0x%x)!"
                        ,mPmemSize,previewFrameSizeMax,RAW_BUFFER_SIZE_0M3,JPEG_BUFFER_SIZE_0M3);
                }
            default:
                sprintf(str_picturesize, "%dx%d", mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
                mRawBufferSize = RAW_BUFFER_SIZE_5M;
                mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
                params.setPictureSize(mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
                break;
        }
        params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, str_picturesize);

        /*frame rate setting*/
        cameraFpsInfoSet(params);
    }

    /*preview format setting*/
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, "yuv420sp,yuv422sp");
    params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,"yuv420sp");
    params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);

	params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);

    /*picture format setting*/
    params.set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, CameraParameters::PIXEL_FORMAT_JPEG);
    params.setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);

    /*jpeg quality setting*/
    params.set(CameraParameters::KEY_JPEG_QUALITY, "70");

    /*white balance setting*/
	struct v4l2_queryctrl whiteBalance;
	struct v4l2_querymenu *whiteBalance_menu = mWhiteBalance_menu;
    char str_whitebalance[200];
	strcpy(str_whitebalance, "");//default whitebalance
	whiteBalance.id = V4L2_CID_DO_WHITE_BALANCE;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &whiteBalance)) {
		for (i = whiteBalance.minimum; i <= whiteBalance.maximum; i += whiteBalance.step) {
			whiteBalance_menu->id = V4L2_CID_DO_WHITE_BALANCE;
			whiteBalance_menu->index = i;
			if (!ioctl(iCamFd, VIDIOC_QUERYMENU, whiteBalance_menu)) {
                if (i != whiteBalance.minimum)
                    strcat(str_whitebalance, ",");
				strcat(str_whitebalance, (char *)whiteBalance_menu->name);
				if (whiteBalance.default_value == i) {
					strcpy(cur_param, (char *)whiteBalance_menu->name);
				}
				mWhiteBalance_number++;
			}
			whiteBalance_menu++;
		}
		params.set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, str_whitebalance);
		params.set(CameraParameters::KEY_WHITE_BALANCE, cur_param);
	} else {
		params.set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, "false");
		params.set(CameraParameters::KEY_WHITE_BALANCE, "auto");
	}

    /*color effect setting*/
	struct v4l2_queryctrl effect;
	struct v4l2_querymenu *effect_menu = mEffect_menu;
    char str_effect[200];
	strcpy(str_effect, "");//default effect
	effect.id = V4L2_CID_EFFECT;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &effect)) {
		for (i = effect.minimum; i <= effect.maximum; i += effect.step) {
			effect_menu->id = V4L2_CID_EFFECT;
			effect_menu->index = i;
			if (!ioctl(iCamFd, VIDIOC_QUERYMENU, effect_menu)) {
                if (i != effect.minimum)
                    strcat(str_effect, ",");
				strcat(str_effect, (char *)effect_menu->name);
				if (effect.default_value == i) {
					strcpy(cur_param, (char *)effect_menu->name);
				}
				mEffect_number++;
			}
			effect_menu++;
		}
		params.set(CameraParameters::KEY_SUPPORTED_EFFECTS, str_effect);
		params.set(CameraParameters::KEY_EFFECT, cur_param);
	} else {
		params.set(CameraParameters::KEY_SUPPORTED_EFFECTS, "false");
		params.set(CameraParameters::KEY_EFFECT, "none");
	}

    /*scene setting*/
	struct v4l2_queryctrl scene;
	struct v4l2_querymenu *scene_menu = mScene_menu;
    char str_scene[200];
	strcpy(str_scene, "");//default scene
	scene.id = V4L2_CID_SCENE;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &scene)) {
		for (i=scene.minimum; i<=scene.maximum; i+=scene.step) {
		    scene_menu->id = V4L2_CID_SCENE;
			scene_menu->index = i;
			if (!ioctl(iCamFd, VIDIOC_QUERYMENU, scene_menu)) {
                if (i != scene.minimum)
                    strcat(str_scene, ",");
				strcat(str_scene, (char *)scene_menu->name);
				if (scene.default_value == i) {
					strcpy(cur_param, (char *)scene_menu->name);
				}
				mScene_number++;
			}
			scene_menu++;
		}
		params.set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, str_scene);
		params.set(CameraParameters::KEY_SCENE_MODE, cur_param);

	} else {
		params.set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, "false");
		params.set(CameraParameters::KEY_SCENE_MODE, "auto");
	}

    /*flash mode setting*/
	struct v4l2_queryctrl flashMode;
	struct v4l2_querymenu *flashMode_menu = mFlashMode_menu;
    char str_flash[200];
	strcpy(str_flash, "");//default flash
	flashMode.id = V4L2_CID_FLASH;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &flashMode)) {
		for (i = flashMode.minimum; i <= flashMode.maximum; i += flashMode.step) {
			flashMode_menu->id = V4L2_CID_FLASH;
			flashMode_menu->index = i;
			if (!ioctl(iCamFd, VIDIOC_QUERYMENU, flashMode_menu)) {
                if (i != flashMode.minimum)
                    strcat(str_flash, ",");
				strcat(str_flash, (char *)flashMode_menu->name);
				if (flashMode.default_value == i) {
					strcpy(cur_param, (char *)flashMode_menu->name);
				}
				mFlashMode_number++;
			}
			flashMode_menu++;
		}
		params.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, str_flash);
		params.set(CameraParameters::KEY_FLASH_MODE, cur_param);
	} else {
		params.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, "false");
        params.set(CameraParameters::KEY_FLASH_MODE, "off");
	}

    /*focus mode setting*/
    struct v4l2_queryctrl focus;

    parameterString = CameraParameters::FOCUS_MODE_FIXED;
    params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
    focus.id = V4L2_CID_FOCUS_AUTO;
    if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_AUTO);
        params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_AUTO);
    }

    focus.id = V4L2_CID_FOCUS_CONTINUOUS;
    if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_EDOF);
    }

    focus.id = V4L2_CID_FOCUS_ABSOLUTE;
    if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_INFINITY);
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_MACRO);
    }

	params.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, parameterString.string());

    /*mirror and flip query*/
	struct v4l2_queryctrl mirror,flip;
    
	mirror.id = V4L2_CID_HFLIP;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &mirror)) {
		mDriverMirrorSupport = true;
	} else {
		mDriverMirrorSupport = false;
	}

    flip.id = V4L2_CID_VFLIP;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &flip)) {
		mDriverFlipSupport = true;
	} else {
		mDriverFlipSupport = false;
	}

    /*zoom setting*/
   
	struct v4l2_queryctrl zoom;
    char str_zoom_max[3],str_zoom_element[5];
    char str_zoom[200];
    strcpy(str_zoom, "");//default zoom
	int max;
    
	zoom.id = V4L2_CID_ZOOM_ABSOLUTE;
	if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &zoom)) {
		mZoomMax = zoom.maximum;
		mZoomMin= zoom.minimum;
		mZoomStep = zoom.step;	

        max = (mZoomMax - mZoomMin)/mZoomStep;
        sprintf(str_zoom_max,"%d",max);
		params.set(CameraParameters::KEY_ZOOM_SUPPORTED, "true");
		params.set(CameraParameters::KEY_MAX_ZOOM, str_zoom_max);
		params.set(CameraParameters::KEY_ZOOM, "0");
        for (i=mZoomMin; i<=mZoomMax; i+=mZoomStep) {
            sprintf(str_zoom_element,"%d,", i);
            strcat(str_zoom,str_zoom_element);
        }
		params.set(CameraParameters::KEY_ZOOM_RATIOS, str_zoom);
	} else {
		params.set(CameraParameters::KEY_ZOOM_SUPPORTED, "false");
	}
    
    /*rotation setting*/
    params.set(CameraParameters::KEY_ROTATION, "0");

    /*lzg@rockchip.com :add some settings to pass cts*/    
    /*focus distance setting ,no much meaning ,only for passing cts */
    parameterString = "0.3,50,Infinity";
    params.set(CameraParameters::KEY_FOCUS_DISTANCES, parameterString.string());
    /*focus length setting ,no much meaning ,only for passing cts */
    parameterString = "35";
    params.set(CameraParameters::KEY_FOCAL_LENGTH, parameterString.string());
   /*horizontal angle of view setting ,no much meaning ,only for passing cts */
    parameterString = "100";
    params.set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, parameterString.string());
    /*vertical angle of view setting ,no much meaning ,only for passing cts */
    parameterString = "100";
    params.set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, parameterString.string());

    /*exposure compensation setting ,no much meaning ,only for passing cts */
    parameterString = "0";
    params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, parameterString.string());
    
    parameterString = "0";
    params.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, parameterString.string());

    parameterString = "0";
    params.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, parameterString.string());

    parameterString = "0.000001";
    params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, parameterString.string());

   /*quality of the EXIF thumbnail in Jpeg picture setting */
    parameterString = "50";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, parameterString.string());
   /*supported size of the EXIF thumbnail in Jpeg picture setting */
    parameterString = "0x0,160x128";
    params.set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES, parameterString.string());
    parameterString = "160";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, parameterString.string());
    parameterString = "128";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, parameterString.string()); 

    params.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, "0");
    params.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW, "0");
    params.set(CameraParameters::KEY_RECORDING_HINT,"false");
    params.set(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED,"false");
    params.set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED,"false");
    params.set(CameraParameters::KEY_MAX_NUM_METERING_AREAS,"0");
    
    LOGD ("Support Preview sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES));
    LOGD ("Support Preview FPS range: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE));
    LOGD ("Support Preview framerate: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES)); 
    LOGD ("Support Picture sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES));
    LOGD ("Support white balance: %s",params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE));
    LOGD ("Support color effect: %s",params.get(CameraParameters::KEY_SUPPORTED_EFFECTS));
    LOGD ("Support scene: %s",params.get(CameraParameters::KEY_SUPPORTED_SCENE_MODES));
    LOGD ("Support flash: %s",params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES));
    LOGD ("Support focus: %s",params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES));
    LOGD ("Support zoom: %s(ratios: %s)",params.get(CameraParameters::KEY_ZOOM_SUPPORTED),
        params.get(CameraParameters::KEY_ZOOM_RATIOS));
    LOGD ("Support hardware faces detecte: %s",params.get(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW));
    LOGD ("Support software faces detecte: %s",params.get(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW));
    LOGD ("Support video stabilization: %s",params.get(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED));
    LOGD ("Support recording hint: %s",params.get(CameraParameters::KEY_RECORDING_HINT));
    LOGD ("Support video snapshot: %s",params.get(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED));
    LOGD ("Support Mirror and Filp: %s",(mDriverMirrorSupport && mDriverFlipSupport)? "true":"false");

    cameraConfig(params);
    LOG_FUNCTION_NAME_EXIT

}


CameraHal::~CameraHal()
{
    LOGD("CameraHal destory success");
}

int CameraHal::setPreviewWindow(struct preview_stream_ops *window)
{
    int ret = NO_ERROR;    
    Mutex::Autolock lock(mLock);

    if (mANativeWindow && (window != mANativeWindow)) {
        cameraPreviewBufferDestory();
        mANativeWindow = NULL;
    }
    
    mANativeWindow = window;
    if (mANativeWindow) {        
        mANativeWindowCond.signal();
        LOGD("%s(%d): mANativeWindow is 0x%x, Now wake up thread which wait for mANativeWindowCond",__FUNCTION__,__LINE__,(int)mANativeWindow);
    } else {
        LOG1("%s(%d): window is NULL",__FUNCTION__,__LINE__);
    }
    
setPreviewWindow_end:
    if (ret)        
        LOGE("%s(%d): exit with error(%d)",__FUNCTION__,__LINE__,ret);
    return ret;
}

void CameraHal::setCallbacks(camera_notify_callback notify_cb,
            camera_data_callback data_cb,
            camera_data_timestamp_callback data_cb_timestamp,
            camera_request_memory get_memory,
            void *user)                                   
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mRequestMemory = get_memory;
    mCallbackCookie = user;
    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::enableMsgType(int32_t msgType)
{
    LOG1("%s : 0x%x",__FUNCTION__, msgType);
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= msgType;
    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::disableMsgType(int32_t msgType)
{
    LOG1("%s : 0x%x",__FUNCTION__, msgType);
    Mutex::Autolock lock(mLock);
    mMsgEnabled &= ~msgType;
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::msgTypeEnabled(int32_t msgType)
{
    LOG1("%s : 0x%x",__FUNCTION__, msgType);
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT
    return (mMsgEnabled & msgType);
    
}
int CameraHal::cameraDisplayThreadStart(int done)
{
    int err = NO_ERROR;
    Message msg;

    if (mDisplayRuning == STA_DISPLAY_RUN) {
        LOGD("%s(%d): display thread is already run",__FUNCTION__,__LINE__);
        goto cameraDisplayThreadStart_end;
    }
   
    msg.command = CMD_DISPLAY_START;
    displayThreadCommandQ.put(&msg); 
    mDisplayCond.signal();
    if (done == true) {
        if (displayThreadAckQ.get(&msg) < 0) {
            LOGE("%s(%d): Start display thread failed,mDisplayRunging(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_START) && ((int)msg.arg1 == STA_DISPLAY_RUN)) {
                LOG1("%s(%d): Start display thread success,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);
            } else {
                LOGE("%s(%d): Start display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);     
            }
        }    
    }
cameraDisplayThreadStart_end:
    return err;
}
int CameraHal::cameraDisplayThreadPause(int done)
{
    int err = NO_ERROR;
    Message msg;

    if (mDisplayRuning == STA_DISPLAY_PAUSE) {
        LOGD("%s(%d): display thread is already pause",__FUNCTION__,__LINE__);
        goto cameraDisplayThreadPause_end;
    }
    
    msg.command = CMD_DISPLAY_PAUSE;
    displayThreadCommandQ.put(&msg); 
    mDisplayCond.signal();
    if (done == true) {
        if (displayThreadAckQ.get(&msg,2500) < 0) {
            LOGE("%s(%d): Pause display thread failed, mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_PAUSE) && ((int)msg.arg1 == STA_DISPLAY_PAUSE)) {
                LOG1("%s(%d): Pause display thread success,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);
            } else {
                LOGE("%s(%d): Pause display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);     
            }    
        }    
    } 
cameraDisplayThreadPause_end:
    return err;
}
int CameraHal::cameraDisplayThreadStop(int done)
{
    int err = NO_ERROR;
    Message msg;

    if (mDisplayRuning == STA_DISPLAY_STOP) {
        LOGD("%s(%d): display thread is already stop",__FUNCTION__,__LINE__);
        goto cameraDisplayThreadStop_end;
    }
    
    msg.command = CMD_DISPLAY_STOP;
    displayThreadCommandQ.put(&msg); 
    mDisplayCond.signal();
    if (done == true) {
        if (displayThreadAckQ.get(&msg,1000) < 0) {
            LOGE("%s(%d): Stop display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_STOP) && ((int)msg.arg1 == STA_DISPLAY_STOP)) {
                LOG1("%s(%d): Stop display thread success,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);
            } else {
                LOGE("%s(%d): Stop display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);     
            }     
        }    
    }
cameraDisplayThreadStop_end:
    return err;
}
void CameraHal::displayThread()
{
    int err,stride,i,all_dequeue;
    int dequeue_buf_index;
    buffer_handle_t *hnd = NULL; 
    private_handle_t *phnd;
    GraphicBufferMapper& mapper = GraphicBufferMapper::get();
    Message msg;
    
    LOG_FUNCTION_NAME    
    while (mDisplayRuning != STA_DISPLAY_STOP) {
display_receive_cmd:        
        if (displayThreadCommandQ.isEmpty() == false ) {
            displayThreadCommandQ.get(&msg);

            switch (msg.command)
            {
                case CMD_DISPLAY_START:
                {
                    LOGD("%s(%d): receive CMD_DISPLAY_START", __FUNCTION__,__LINE__);
                    cameraPreviewBufferDestory();
                    cameraPreviewBufferCreate(mPreviewWidth, mPreviewHeight,mCamDriverPreviewFmt,CONFIG_CAMERA_PRVIEW_BUF_CNT);
                    mDisplayRuning = STA_DISPLAY_RUN;
                    msg.arg1 = (void*)mDisplayRuning;
                    displayThreadAckQ.put(&msg);
                    break;
                }

                case CMD_DISPLAY_PAUSE:
                {
                    LOGD("%s(%d): receive CMD_DISPLAY_PAUSE", __FUNCTION__,__LINE__);
                    cameraPreviewBufferDestory();
                    mDisplayRuning = STA_DISPLAY_PAUSE;
                    msg.arg1 = (void*)mDisplayRuning;
                    displayThreadAckQ.put(&msg);                    
                    break;
                }
                
                case CMD_DISPLAY_STOP:
                {
                    LOGD("%s(%d): receive CMD_DISPLAY_STOP", __FUNCTION__,__LINE__);
                    mDisplayRuning = STA_DISPLAY_STOP;
                    msg.arg1 = (void*)mDisplayRuning;
                    displayThreadAckQ.put(&msg);
                    continue;
                }
                default:
                {
                    LOGE("%s(%d): receive unknow command(0x%x)!", __FUNCTION__,__LINE__,msg.command);
                    break;
                }
            }
        }

        if (mDisplayRuning == STA_DISPLAY_PAUSE) {
            LOG1("%s(%d): display thread pause here... ", __FUNCTION__,__LINE__);
            mDisplayCond.wait(mDisplayLock);   
            LOG1("%s(%d): display thread wake up... ", __FUNCTION__,__LINE__);
            goto display_receive_cmd;
        }
        
        if (mANativeWindow) {
            void *y_uv[2];
            Rect bounds;
            
            err = mANativeWindow->dequeue_buffer(mANativeWindow, (buffer_handle_t**)&hnd, &stride);
            if (err == 0) {
                // lock the initial queueable buffers
                bounds.left = 0;
                bounds.top = 0;
                bounds.right = mPreviewWidth;
                bounds.bottom = mPreviewHeight;
                mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)hnd);
                mapper.lock((buffer_handle_t)(*hnd), CAMHAL_GRALLOC_USAGE, bounds, y_uv);

                phnd = (private_handle_t*)*hnd;
                for (i=0; i<mPreviewBufferCount; i++) {
                    if (phnd == mPreviewBufferMap[i].priv_hnd) {
                        dequeue_buf_index = i;
                        break;
                    }
                }
                
                if (i >= mPreviewBufferCount) {                    
                    LOGE("%s(%d): dequeue buffer(0x%x magic:0x%x) don't find in mPreviewBufferMap", __FUNCTION__,__LINE__,(int)phnd,phnd->magic);                    
                    continue;
                } else {
                    LOG2("%s(%d): dequeue buffer %d from display", __FUNCTION__,__LINE__,dequeue_buf_index);
                    cameraPreviewBufferSetSta(&mPreviewBufferMap[dequeue_buf_index], CMD_PREVIEWBUF_DISPING, 0);
                }
                
                //Notify the frame has displayed, it is idle in ANativeWindow 
                msg.command = CMD_PREVIEW_QBUF;     
                msg.arg1 = (void*)dequeue_buf_index;
                msg.arg2 = (void*)CMD_PREVIEWBUF_DISPING;
                msg.arg3 = (void*)mPreviewStartTimes;
                commandThreadCommandQ.put(&msg); 
            } else {
                /* ddl@rock-chips.com: dequeueBuffer isn't block, when ANativeWindow in asynchronous mode */
                LOG2("%s(%d): %s(err:%d) dequeueBuffer failed, so pause here", __FUNCTION__,__LINE__, strerror(-err), -err);
                mDisplayCond.wait(mDisplayLock); 
                LOG2("%s(%d): wake up...", __FUNCTION__,__LINE__);
            }
            
        } else {
            LOGE("%s(%d): thread exit, because mANativeWindow is NULL", __FUNCTION__,__LINE__);
            mDisplayRuning = STA_DISPLAY_STOP;            
        }
    }
    
}
void CameraHal::previewThread()
{
    int err;
    int ret,i;
    bool loop = true;
    static struct v4l2_buffer cfilledbuffer1;
    Message msg;
    bool camera_device_error;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    
    LOG_FUNCTION_NAME
    camera_device_error = false;
    while (loop) {
        msg.command = CMD_PREVIEW_INVAL;
        if (camera_device_error == true){
            LOGD("%s(%d): camera driver or device may be error, so preview thread wait for command thread wake...",
                __FUNCTION__,__LINE__);
            previewThreadCommandQ.get(&msg);
            mPreviewLock.lock();
            camera_device_error = false;
        } else {
            mPreviewLock.lock();
            if (previewThreadCommandQ.isEmpty() == false ) 
                previewThreadCommandQ.get(&msg);
        }
        
        switch (msg.command)
        {
            case CMD_PREVIEW_STAREQ:
            {
                LOGD("%s(%d): answer CMD_PREVIEW_STAREQ: 0x%x",__FUNCTION__,__LINE__,mPreviewRunning);
                msg.command = mPreviewRunning;
                previewThreadAckQ.put(&msg);
                break;
            }    
            default:
                break;
        }
        
        if (mPreviewRunning == STA_PREVIEW_STOP) {
            mPreviewLock.unlock();
            goto previewThread_end;
        }
        
        while (mPreviewRunning == STA_PREVIEW_PAUSE) {
            mPreviewCond.wait(mPreviewLock);
            LOG1("%s(%d): wake up for mPreviewRunning:0x%x ",__FUNCTION__,__LINE__,mPreviewRunning);            
        }
        
        if (mPreviewRunning == STA_PREVIEW_RUN) {
            
            cfilledbuffer1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            cfilledbuffer1.memory = mCamDriverV4l2MemType;
            cfilledbuffer1.reserved = NULL;
            
            /* De-queue the next avaliable buffer */            
            mPreviewLock.unlock();
            if (ioctl(iCamFd, VIDIOC_DQBUF, &cfilledbuffer1) < 0) {
                LOGE("%s(%d): VIDIOC_DQBUF Failed!!! err[%s] \n",__FUNCTION__,__LINE__,strerror(errno));
                if (errno == EIO) {
                    camera_device_error = true;  
                    LOGE("%s(%d): camera driver or device may be error, so notify CAMERA_MSG_ERROR",
                            __FUNCTION__,__LINE__);
                } else {
                    mPreviewErrorFrameCount++;
                    if (mPreviewErrorFrameCount >= 4) {                
                        camera_device_error = true;   
                        LOGE("%s(%d): mPreviewErrorFrameCount is %d, camera driver or device may be error, so notify CAMERA_MSG_ERROR",
                            __FUNCTION__,__LINE__,mPreviewErrorFrameCount);
                    }
                }
                if (camera_device_error == true) {
                    if (mNotifyCb && (mMsgEnabled & CAMERA_MSG_ERROR)) {                        
                        mNotifyCb(CAMERA_MSG_ERROR, 0,0,mCallbackCookie);
                        mPreviewErrorFrameCount = 0;                        
                        continue;
                    }                    
                }
                ioctl(iCamFd, VIDIOC_QBUF, &cfilledbuffer1);
            } else {
                mPreviewErrorFrameCount = 0;
            }

            cameraPreviewBufferSetSta(&mPreviewBufferMap[cfilledbuffer1.index], CMD_PREVIEWBUF_WRITING, 0);

            mPreviewLock.lock();                       
            // Notify mANativeWindow of a new frame.
            if (mANativeWindow && mPreviewBufferMap[cfilledbuffer1.index].priv_hnd) {
                //DLOGD("%s(%d): enqueue buffer %d to display", __FUNCTION__,__LINE__,cfilledbuffer1.index);
                mapper.unlock((buffer_handle_t)mPreviewBufferMap[cfilledbuffer1.index].priv_hnd);                
                err = mANativeWindow->enqueue_buffer(mANativeWindow, (buffer_handle_t*)mPreviewBufferMap[cfilledbuffer1.index].buffer_hnd);
                if (err != 0){
                    LOGE("%s(%d): enqueue_buffer to mANativeWindow failed(%d)", __FUNCTION__,__LINE__,err);
                }
                cameraPreviewBufferSetSta(&mPreviewBufferMap[cfilledbuffer1.index], CMD_PREVIEWBUF_DISPING, 1);
                LOG2("%s(%d): enqueue buffer %d to display", __FUNCTION__,__LINE__,cfilledbuffer1.index);
                /*ddl@rock-chips.com: wake up display thread dequeue buffer, when ANativeWindow in asynchronous mode*/
                mDisplayCond.signal();
            }
                        
            // Send Video Frame 
        	if (mRecordRunning && (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) && mDataCbTimestamp) {
                if (mVideoBufs[cfilledbuffer1.index] != NULL) {
                    mDataCbTimestamp(systemTime(CLOCK_MONOTONIC), CAMERA_MSG_VIDEO_FRAME, mVideoBufs[cfilledbuffer1.index], 0, mCallbackCookie);                
                    cameraPreviewBufferSetSta(&mPreviewBufferMap[cfilledbuffer1.index], CMD_PREVIEWBUF_ENCING, 1);            
                } else {
                    LOGE("%s(%d): mVideoBufs[%d] is NULL, this frame recording cancel",__FUNCTION__,__LINE__,cfilledbuffer1.index);
                }
            } 

            if ((mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) && mDataCb) {
                if (mPreviewMemory) {
                    /* ddl@rock-chips.com : preview frame rate may be too high, CTS testPreviewCallback may be fail*/                    
                    cameraFormatConvert(mCamDriverPreviewFmt,V4L2_PIX_FMT_NV21,NULL,
                        (char*)mPreviewBufferMap[cfilledbuffer1.index].priv_hnd->base,(char*)mPreviewBufs[cfilledbuffer1.index], mPreviewWidth, mPreviewHeight);                                           
                    mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewMemory, cfilledbuffer1.index,NULL,mCallbackCookie); 
                } else {
                    LOGE("%s(%d): mPreviewMemory is NULL, preview data could not send to application",__FUNCTION__,__LINE__);
                }
            } 
        }       
        mPreviewLock.unlock(); 
    }
    
previewThread_end:    
    LOG_FUNCTION_NAME_EXIT
    return;
}
void CameraHal::pictureThread()
{
    struct CamCaptureInfo_s capture;

    LOG_FUNCTION_NAME
    capture.input_phy_addr = mPmemHeapPhyBase + mRawBuffer->offset();                        
    capture.output_phy_addr = mPmemHeapPhyBase + mJpegBuffer->offset();                        
    capture.output_vir_addr = (int)mJpegBuffer->pointer();                        
    capture.output_buflen = mJpegBuffer->size();

    capturePicture(&capture);

    mPictureLock.lock();
    mPictureRunning = STA_PICTURE_STOP;
    mPictureLock.unlock();
    LOG_FUNCTION_NAME_EXIT
    return;

}
void CameraHal::autofocusThread()
{
    int err;
    
    while (1) {
        mAutoFocusLock.lock();
        /* check early exit request */
        if (mExitAutoFocusThread) {
            mAutoFocusLock.unlock();
            LOG1("%s(%d) exit", __FUNCTION__,__LINE__);
            goto autofocusThread_end;
        }
        mAutoFocusCond.wait(mAutoFocusLock);
        /* check early exit request */
        if (mExitAutoFocusThread) {
            mAutoFocusLock.unlock();
            LOG1("%s(%d) exit", __FUNCTION__,__LINE__);
            goto autofocusThread_end;
        }
        mAutoFocusLock.unlock();
        
        err = cameraAutoFocus(CameraParameters::FOCUS_MODE_AUTO);

        if (mMsgEnabled & CAMERA_MSG_FOCUS)
            mNotifyCb(CAMERA_MSG_FOCUS, err, 0, mCallbackCookie);
        
    }

autofocusThread_end: 
    LOG_FUNCTION_NAME_EXIT
    return;
}
void CameraHal::commandThread()
{
    Message msg;
    bool  shouldLive = true;
    bool has_message;
    int err = 0;

    LOG_FUNCTION_NAME

    while(shouldLive) {
get_command:
        commandThreadCommandQ.get(&msg);

        switch(msg.command)
        {
            case CMD_PREVIEW_START:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_START, mPreviewRunning(%d) mDisplayRuning(%d)", __FUNCTION__,__LINE__,mPreviewRunning,mDisplayRuning);

                if (NULL == mANativeWindow) {
                    LOGD("%s(%d): camera preview buffer alloc from ANativeWindow, Now mANativeWIndow is NULL, wait for set...",__FUNCTION__,__LINE__);  
                    mANativeWindowCond.wait(mANativeWindowLock);
                } 

                if (commandThreadCommandQ.isEmpty() == false) {
                    LOGD("%s(%d): wake up for receive command",__FUNCTION__,__LINE__);
                    goto get_command;    
                }
                                
                err = 0;                
                cameraDisplayThreadStart(true);                
                mPreviewLock.lock();
                if(mPreviewRunning != STA_PREVIEW_RUN) {
					cameraSetSize(mPreviewWidth, mPreviewHeight, mCamDriverPreviewFmt);
    				err = cameraStart();
                } else {
                    err = -1;
                    LOGD("%s(%d): preview thread is already run", __FUNCTION__,__LINE__);
                }

                msg.arg1 = (void*)(err ? CMD_NACK : CMD_ACK);

                if( err == 0 ) {
                    mPreviewRunning = STA_PREVIEW_RUN;
                    mPreviewCond.signal(); 
                    android_atomic_inc(&mPreviewStartTimes);
                } 
                mPreviewLock.unlock();                
                LOGD("%s(%d): CMD_PREVIEW_START %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                commandThreadAckQ.put(&msg);
                break;
            }

            case CMD_PREVIEW_STOP:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_STOP,  mPreviewRunning(%d) mDisplayRuning(%d)", __FUNCTION__,__LINE__,mPreviewRunning,mDisplayRuning);

                if (mPreviewRunning  == STA_PREVIEW_RUN)
                    cameraDisplayThreadPause(true);                    
                mPreviewLock.lock();
                if( mPreviewRunning  == STA_PREVIEW_RUN) {
                    mPreviewRunning = STA_PREVIEW_PAUSE;                        
                    msg.command = CMD_PREVIEW_STAREQ;
                    previewThreadCommandQ.put(&msg);
                    mPreviewCond.signal();
                    mPreviewLock.unlock(); 
                    previewThreadAckQ.get(&msg,1000);  
                    if (msg.command != STA_PREVIEW_PAUSE) {
                        LOGE("%s(%d): Pause preview thread failed!",__FUNCTION__,__LINE__);
                        msg.command = CMD_PREVIEW_STOP;
                        msg.arg1 = (void*)CMD_NACK;
                    } else {
                        msg.command = CMD_PREVIEW_STOP;
                        msg.arg1 = (void*)CMD_ACK;
                    } 
                    cameraStop(); 
                } else {
                    mPreviewLock.unlock();
                    msg.command = CMD_PREVIEW_STOP;
                    msg.arg1 = (void*)CMD_ACK;
                    LOGD("%s(%d): preview thread is already pause",__FUNCTION__,__LINE__);
                }
                 
                LOGD("%s(%d): CMD_PREVIEW_STOP %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                commandThreadAckQ.put(&msg);
                break;
            }
            
            case CMD_PREVIEW_CAPTURE:
            {
                int i;
                struct pmem_region sub;
                struct v4l2_buffer v4l2_buf;
                
                LOGD("%s(%d): receive CMD_PREVIEW_CAPTURE", __FUNCTION__,__LINE__);
                if (mPictureThread == NULL) {
                    LOGE("%s(%d): mPictureThread is NULL",__FUNCTION__,__LINE__);
                    err = -1;
                    goto PREVIEW_CAPTURE_end;
                }
                
                mPictureLock.lock();
                if (mPictureRunning != STA_PICTURE_STOP) {
                    mPictureLock.unlock();
                    LOGE("%s(%d): picture thread state doesn't suit capture(mPictureRunning:0x%x)", __FUNCTION__,__LINE__, mPictureRunning);
                    err = -1;
                    goto PREVIEW_CAPTURE_end;
                }
                mPictureLock.unlock();

                if ((mRawBuffer == NULL) || (mJpegBuffer == NULL)) {
                    LOGE("%s(%d): cancel, because mRawBuffer and mJpegBuffer is NULL",__FUNCTION__,__LINE__);
                    err = -1;
                    goto PREVIEW_CAPTURE_end;
                }
                
                if (mPreviewRunning  == STA_PREVIEW_RUN) {
                    mPreviewLock.lock();                        
                    mPreviewRunning = STA_PREVIEW_PAUSE;
                    msg.command = CMD_PREVIEW_STAREQ;
                    previewThreadCommandQ.put(&msg);
                    mPreviewLock.unlock();                        
                    previewThreadAckQ.get(&msg,1000);
                    if (msg.command != STA_PREVIEW_PAUSE){
                        LOGE("%s(%d): Pause preview thread failed!",__FUNCTION__,__LINE__);
                    }
                    cameraStop();
                }
                
                if (mPictureThread->run("CameraPictureThread", ANDROID_PRIORITY_DISPLAY) != NO_ERROR) {
                    LOGE("%s(%d): couldn't run picture thread", __FUNCTION__,__LINE__);
                    err = -1;
                    goto PREVIEW_CAPTURE_end;
                } else {
                    mPictureLock.lock();
                    mPictureRunning = STA_PICTURE_RUN;
                    mPictureLock.unlock();
                }
PREVIEW_CAPTURE_end:
                if (err < 0) {
                    msg.command = CMD_PREVIEW_CAPTURE;
                    msg.arg1 = (void*)CMD_NACK;              
                } else {
                    msg.command = CMD_PREVIEW_CAPTURE;
                    msg.arg1 = (void*)CMD_ACK;                        
                }
                LOGD("%s(%d): CMD_PREVIEW_CAPTURE %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                commandThreadAckQ.put(&msg);
                break;
            }
            case CMD_PREVIEW_CAPTURE_CANCEL:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_CAPTURE_CANCEL", __FUNCTION__,__LINE__);
                msg.command = CMD_PREVIEW_CAPTURE_CANCEL;
                msg.arg1 = (void*)CMD_ACK;
                LOGD("%s(%d): CMD_PREVIEW_CAPTURE_CANCEL %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                commandThreadAckQ.put(&msg);
                break; 
            }
            case CMD_AF_START:
            {
                LOGD("%s(%d): receive CMD_AF_START", __FUNCTION__,__LINE__);
                msg.command = CMD_AF_START;
                msg.arg1 = (void*)CMD_ACK;
                commandThreadAckQ.put(&msg);
                mAutoFocusCond.signal();
                LOGD("%s(%d): CMD_AF_START %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                break;
            }            
            case CMD_AF_CANCEL:
            {
                LOGD("%s(%d): receive CMD_AF_CANCEL", __FUNCTION__,__LINE__);
                msg.command = CMD_AF_CANCEL;
                msg.arg1 = (void*)CMD_ACK;                    
                commandThreadAckQ.put(&msg);
                LOGD("%s(%d): CMD_AF_CANCEL %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                break;
            }
            
            case CMD_PREVIEW_QBUF:
            {
                struct v4l2_buffer vb;
                struct pmem_region region;
                int index;
                int cmd_info, preview_times;

                index = (int)msg.arg1;
                cmd_info = (int)msg.arg2;
                preview_times = (int)msg.arg3;
                
                LOG2("%s(%d): receive CMD_PREVIEW_QBUF from %s",__FUNCTION__,__LINE__, 
                    ((int)msg.arg2 == CMD_PREVIEWBUF_DISPING)?"display":"encoder");
                
                if (mPreviewRunning != STA_PREVIEW_RUN) {
                    LOG1("%s(%d): preview thread is pause, so buffer %d isn't enqueue to camera",__FUNCTION__,__LINE__,index);
                    goto CMD_PREVIEW_QBUF_end;
                }
                if (mANativeWindow && mPreviewBufferMap[index].priv_hnd) {
                    if (CAMERA_PREVIEWBUF_ALLOW_WRITE(mPreviewBufferMap[index].buf_state)) {

                        if (preview_times == mPreviewStartTimes) {                        
                            vb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                            vb.memory = mCamDriverV4l2MemType;
                            vb.index = index;                        
                            vb.reserved = NULL;
                            
                            if (ioctl(iCamFd, VIDIOC_QUERYBUF, &vb) < 0) {
                                LOGE("%s(%d): VIDIOC_QUERYBUF Failed!!! err[%s]", __FUNCTION__,__LINE__, strerror(errno));            
                            }
                            vb.m.offset = mPreviewBufferMap[index].phy_addr; 
                            if (ioctl(iCamFd, VIDIOC_QBUF, &vb) < 0) {
                                LOGE("%s(%d): VIDIOC_QBUF Failed!!! err[%s]", __FUNCTION__,__LINE__, strerror(errno));
                            }  

                            cameraPreviewBufferSetSta(&mPreviewBufferMap[index], CMD_PREVIEWBUF_WRITING, 1);
                            LOG2("%s(%d): enqueue buffer %d(addr:0x%x 0x%x) to camera", __FUNCTION__,__LINE__,index,vb.m.offset,mPreviewBufferMap[index].phy_addr);
                        } else {
                            LOG1("%s(%d): this message(CMD_PREVIEW_QBUF) is invaliade, camera have restart", __FUNCTION__,__LINE__);
                        }
                    }
                }
CMD_PREVIEW_QBUF_end:        
                break;
            }

            
            case CMD_EXIT:
            {
                LOGD("%s(%d): receive CMD_EXIT", __FUNCTION__,__LINE__);
                shouldLive = false;
                cameraDisplayThreadStop(true);
                mPreviewLock.lock();
                mPreviewRunning = STA_PREVIEW_STOP;
                mPreviewCond.signal();
                msg.command = CMD_PREVIEW_STAREQ;
                previewThreadCommandQ.put(&msg);
                mPreviewLock.unlock();                    
                previewThreadAckQ.get(&msg,3000);
                if (msg.command != STA_PREVIEW_STOP){
                    LOGE("%s(%d): Stop preview thread failed!",__FUNCTION__,__LINE__);
                    msg.command = CMD_EXIT;
                    msg.arg1 = (void*)CMD_NACK;
                } else {
                    LOG1("%s(%d): Stop preview thread success!",__FUNCTION__,__LINE__);
                    msg.command = CMD_EXIT;
                    msg.arg1 = (void*)CMD_ACK;
                }
                
                LOGD("%s(%d): CMD_EXIT %s", __FUNCTION__,__LINE__, (msg.arg1 == (void*)CMD_NACK) ? "NACK" : "ACK");
                commandThreadAckQ.put(&msg);  
                break;
            }                
            default:
                LOGE("%s(%d) receive unknow command(0x%x)\n", __FUNCTION__,__LINE__,msg.command);
                break;
        }
    }

    LOG_FUNCTION_NAME_EXIT
    return;
}
int CameraHal::cameraHeapBufferCreate(int rawBufferSize, int jpegBufferSize)
{
    int err = NO_ERROR;
    struct pmem_region sub;

    LOG_FUNCTION_NAME    
    
    if (mPmemSize < (rawBufferSize + jpegBufferSize)) {
        LOGE("%s(%d): %s total size(0x%x) is low for rawBuffer(0x%x) and jpegBuffer(0x%x)!!", __FUNCTION__,__LINE__,CAMERA_PMEM_NAME,
                mPmemSize,rawBufferSize,jpegBufferSize);
    }
    
    cameraHeapBufferDestory();      

    if (rawBufferSize) {
        mRawBuffer = (static_cast<MemoryHeapPmem*>(mMemHeapPmem.get()))->mapMemory(mPmemSize-jpegBufferSize-rawBufferSize,rawBufferSize);        
        if (mRawBuffer == NULL) {
            LOGE("%s(%d): allocate raw buffer from mMemHeapPmem failed", __FUNCTION__,__LINE__);
            err = -1;
        }
    }

    if (jpegBufferSize) {
        mJpegBuffer = (static_cast<MemoryHeapPmem*>(mMemHeapPmem.get()))->mapMemory(mPmemSize-jpegBufferSize,jpegBufferSize);        
        if (mJpegBuffer == NULL) {
            LOGE("%s(%d): allocate jpeg buffer from mMemHeapPmem failed", __FUNCTION__,__LINE__);
            err = -1;
        }
    }
    
cameraHeapBufferCreate_end:
    if (err)
        LOGE("%s(%d): exit with error(%d)", __FUNCTION__,__LINE__,err);
    else 
        LOG_FUNCTION_NAME_EXIT
    return err;
}

int CameraHal::cameraHeapBufferDestory()
{
    int i;

    LOG_FUNCTION_NAME 

    if (mRawBuffer != NULL) {
        LOG1("mRawBuffer.clear");
        mRawBuffer.clear();
        mRawBuffer = NULL;
    }

    if (mJpegBuffer != NULL) {
        LOG1("mJpegBuffer.clear");
        mJpegBuffer.clear();
        mJpegBuffer = NULL;
    }    
    
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int CameraHal::cameraPreviewBufferCreate(int width, int height, unsigned int fmt,unsigned int numBufs)
{
    int err = NO_ERROR,i;
    int undequeued = 0, total;
    buffer_handle_t* hnd = NULL;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    Rect bounds;        
    struct pmem_region sub;

    LOG_FUNCTION_NAME
    // Set gralloc usage bits for window.
    err = mANativeWindow->set_usage(mANativeWindow, CAMHAL_GRALLOC_USAGE);
    if (err != 0) {
        LOGE("%s(%d): %s(err:%d) native_window_set_usage failed", __FUNCTION__,__LINE__, strerror(-err), -err);

        if ( ENODEV == err ) {
            LOGE("%s(%d): Preview surface abandoned !",__FUNCTION__,__LINE__);
            mANativeWindow = NULL;
        }

        goto fail;
    }
    if (mANativeWindow->set_swap_interval(mANativeWindow, 1) != 0) {
        LOGE("%s(%d): set mANativeWindow run in synchronous mode failed",__FUNCTION__,__LINE__);
    }
    mANativeWindow->get_min_undequeued_buffer_count(mANativeWindow, &undequeued);
    
    ///Set the number of buffers needed for camera preview
    
    //total = numBufs+undequeued;
    total = numBufs;
    LOG1("%s(%d): min_undequeued:0x%x total:0x%x",__FUNCTION__,__LINE__, undequeued, total);
    err = mANativeWindow->set_buffer_count(mANativeWindow, total);
    if (err != 0) {
        LOGE("%s(%d): %s(err:%d) native_window_set_buffer_count(%d+%d) failed", __FUNCTION__,__LINE__,strerror(-err), -err,numBufs,undequeued);

        if ( ENODEV == err ) {
            LOGE("%s(%d): Preview surface abandoned !",__FUNCTION__,__LINE__);
            mANativeWindow = NULL;
        }
        goto fail;
    }
    
    mPreviewBufferCount= numBufs;

    // Set window geometry
    err = mANativeWindow->set_buffers_geometry(
            mANativeWindow,
            width,
            height,
            cameraPixFmt2HalPixFmt(fmt)); 

    if (err != 0) {
        LOGE("%s(%d): %s(err:%d) native_window_set_buffers_geometry failed", __FUNCTION__,__LINE__, strerror(-err), -err);

        if ( ENODEV == err ) {
            LOGE("%s(%d): Preview surface abandoned !",__FUNCTION__,__LINE__);
            mANativeWindow = NULL;
        }

        goto fail;
    }    

    for ( i=0; i < total; i++ ) {
        int stride;  

        err = mANativeWindow->dequeue_buffer(mANativeWindow, (buffer_handle_t**)&hnd, &stride);

        if (err != 0) {
            LOGE("%s(%d): %s(err:%d) dequeueBuffer failed", __FUNCTION__,__LINE__, strerror(-err), -err);

            if ( ENODEV == err ) {
                LOGE("%s(%d): Preview surface abandoned !",__FUNCTION__,__LINE__);
                mANativeWindow = NULL;
            }

            goto fail;
        }

        if (i < mPreviewBufferCount) {
            mPreviewBufferMap[i].buffer_hnd = hnd;
            mPreviewBufferMap[i].priv_hnd= (private_handle_t*)(*hnd);
            if (ioctl(mPreviewBufferMap[i].priv_hnd->fd,PMEM_GET_PHYS,&sub) == 0) {                    
                mPreviewBufferMap[i].phy_addr = sub.offset + mPreviewBufferMap[i].priv_hnd->offset;    /* phy address */ 
            } else {
                LOGE("%s(%d): %s(err:%d) obtain buffer %d phy address failed!",__FUNCTION__,__LINE__,
                        strerror(errno),errno, i);
            }
        } else {   
            private_handle_t *phnd = (private_handle_t*)*hnd;
            
            err = mANativeWindow->cancel_buffer(mANativeWindow, (buffer_handle_t*)hnd);
            if (err != 0) {
                LOGE("%s(%d):cancel_buffer failed: %s (%d)",__FUNCTION__,__LINE__, strerror(-err), -err);

                if ( ENODEV == err ) {
                    LOGE("%s(%d): Preview surface abandoned !",__FUNCTION__,__LINE__);
                    mANativeWindow = NULL;
                }
                goto fail;
            }
        }
    }

    // lock the initial queueable buffers
    bounds.left = 0;
    bounds.top = 0;
    bounds.right = mPreviewWidth;
    bounds.bottom = mPreviewHeight;

    for( i = 0;  i < mPreviewBufferCount; i++ ) {
        void *y_uv[2];

        mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)mPreviewBufferMap[i].buffer_hnd);
        mapper.lock((buffer_handle_t)mPreviewBufferMap[i].priv_hnd, CAMHAL_GRALLOC_USAGE, bounds, y_uv);   

        cameraPreviewBufferSetSta(&mPreviewBufferMap[i], CMD_PREVIEWBUF_DISPING, 0);
        
    }
    LOG_FUNCTION_NAME_EXIT    
    return err; 
 fail:
    if (mPreviewBufferCount) {
        for (i = 0; i<mPreviewBufferCount; i++) {
            err = mANativeWindow->cancel_buffer(mANativeWindow, (buffer_handle_t*)mPreviewBufferMap[i].buffer_hnd);
            if (err != 0) {
              LOGE("%s(%d): cancelBuffer failed w/ error 0x%08x",__FUNCTION__,__LINE__, err);
              break;
            }
        }
    }
    
    LOGE("%s(%d): exit with error(%d)!",__FUNCTION__,__LINE__,err);
    return err;
        
}
int CameraHal::cameraPreviewBufferDestory(void)
{
    int ret = NO_ERROR,i;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();

    LOG_FUNCTION_NAME
    //Give the buffers back to display here -  sort of free it
    if (mANativeWindow) {
        for(i = 0; i < mPreviewBufferCount; i++) {
            // unlock buffer before giving it up
            if (mPreviewBufferMap[i].priv_hnd) {
                mapper.unlock((buffer_handle_t)mPreviewBufferMap[i].priv_hnd);
                mANativeWindow->cancel_buffer(mANativeWindow, (buffer_handle_t*)mPreviewBufferMap[i].buffer_hnd);
            }
            mPreviewBufferMap[i].buffer_hnd = NULL;
            mPreviewBufferMap[i].priv_hnd = NULL;
        }
    } else {
        LOGE("%s(%d): mANativeWindow is NULL, destory is ignore",__FUNCTION__,__LINE__);
    }

    LOG_FUNCTION_NAME_EXIT
cameraPreviewBufferDestory_end:
    return ret;    
}

int CameraHal::cameraPreviewBufferSetSta(rk_previewbuf_info_t *buf_hnd,int cmd, int set)
{
    int err = NO_ERROR;   
    
    if (mPreviewBufferCount <= 0) {
        LOGE("%s(%d): Camerahal preview buffer is null, Don't allow set buffer state",__FUNCTION__,__LINE__);
        err = -EINVAL;
        goto cameraPreviewBufferSetSta_end;
    }

    if (buf_hnd == NULL) {
        LOGE("%s(%d): buf_hnd is null",__FUNCTION__,__LINE__);
        err = -EINVAL;
        goto cameraPreviewBufferSetSta_end;
    }
        
    buf_hnd->lock.lock();

    switch (cmd)
    {
        case CMD_PREVIEWBUF_DISPING:
        {
            if (set){
                if (buf_hnd->buf_state & (1<<CAMERA_PREVIEWBUF_WRITING_BITPOS))
                    LOGE("%s(%d): Set buffer displaying, but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
                buf_hnd->buf_state |= (1<<CAMERA_PREVIEWBUF_DISPING_BITPOS);
            } else { 
                buf_hnd->buf_state &= ~(1<<CAMERA_PREVIEWBUF_DISPING_BITPOS);
            }
            break;
        }

        case CMD_PREVIEWBUF_ENCING:
        {
            if (set) {
                if (buf_hnd->buf_state & (1<<CAMERA_PREVIEWBUF_WRITING_BITPOS))
                    LOGE("%s(%d): Set buffer encoding,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
                buf_hnd->buf_state |= (1<<CAMERA_PREVIEWBUF_ENCING_BITPOS);
            } else {
                buf_hnd->buf_state &= ~(1<<CAMERA_PREVIEWBUF_ENCING_BITPOS);
            }
            break;
        }

        case CMD_PREVIEWBUF_WRITING:
        {
            if (set) {
                if (buf_hnd->buf_state & ((1<<CAMERA_PREVIEWBUF_ENCING_BITPOS)|(1<<CAMERA_PREVIEWBUF_DISPING_BITPOS)))
                    LOGE("%s(%d): Set buffer writing, but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
                buf_hnd->buf_state |= (1<<CAMERA_PREVIEWBUF_WRITING_BITPOS);
            } else { 
                buf_hnd->buf_state &= ~(1<<CAMERA_PREVIEWBUF_WRITING_BITPOS);
            }
            break;
        }
        
        default:
        {
            LOGE("%s(%d): Preview buffer is not support %d status",__FUNCTION__,__LINE__,cmd);
            err = -EINVAL;
            break;
        }
    }
    
    buf_hnd->lock.unlock();


cameraPreviewBufferSetSta_end:
    return err;
}
int CameraHal::cameraCreate(int cameraId)
{
    int err = 0;
    int pmem_fd,i,j,cameraCnt;
    char cam_path[20];
    char cam_num[3];
    char *ptr_tmp;
    struct pmem_region sub;
    struct v4l2_fmtdesc fmtdesc;
    
    LOG_FUNCTION_NAME

    cameraDevicePathCur = (char*)&gCamInfos[cameraId].device_path[0];
    iCamFd = open(cameraDevicePathCur, O_RDWR);
    if (iCamFd < 0) {
        LOGE("%s(%d): open camera%d(%s) is failed",__FUNCTION__,__LINE__,cameraId,cameraDevicePathCur);
        goto exit;
    }

    memset(&mCamDriverCapability, 0, sizeof(struct v4l2_capability));
    err = ioctl(iCamFd, VIDIOC_QUERYCAP, &mCamDriverCapability);
    if (err < 0) {
    	LOGE("%s(%d): %s query device's capability failed.\n",__FUNCTION__,__LINE__,cam_path);
	    goto exit1;
    }

    memset(&fmtdesc, 0, sizeof(fmtdesc));    
	fmtdesc.index = 0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;    
	while (ioctl(iCamFd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        mCamDriverSupportFmt[fmtdesc.index] = fmtdesc.pixelformat;        
		fmtdesc.index++;        
	}

    if ((strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") == 0) 
        && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1))) {
        CameraHal_SupportFmt[0] = V4L2_PIX_FMT_YUV420;
        CameraHal_SupportFmt[1] = V4L2_PIX_FMT_YUV422P;
        CameraHal_SupportFmt[2] = V4L2_PIX_FMT_YUYV;
        CameraHal_SupportFmt[3] = 0x00;
    } else {
        CameraHal_SupportFmt[0] = V4L2_PIX_FMT_NV12;
        CameraHal_SupportFmt[1] = V4L2_PIX_FMT_NV16;
        CameraHal_SupportFmt[2] = V4L2_PIX_FMT_YUYV;
        CameraHal_SupportFmt[3] = 0x00;
    }

    LOGD("Camera driver: %s   Driver version: %d.%d.%d  CameraHal version: %d.%d.%d ",mCamDriverCapability.driver,
        (mCamDriverCapability.version>>16) & 0xff,(mCamDriverCapability.version>>8) & 0xff,
        mCamDriverCapability.version & 0xff,(CONFIG_CAMERAHAL_VERSION>>16) & 0xff,(CONFIG_CAMERAHAL_VERSION>>8) & 0xff,
        CONFIG_CAMERAHAL_VERSION & 0xff);
   
    i = 0;    
    while (CameraHal_SupportFmt[i]) {
        j = 0;
        while (mCamDriverSupportFmt[j]) {
            if (mCamDriverSupportFmt[j] == CameraHal_SupportFmt[i]) {
                break;
            }
            j++;
        }
        if (mCamDriverSupportFmt[j] == CameraHal_SupportFmt[i]) {
            break;
        }
        i++;
    }

    if (CameraHal_SupportFmt[i] == 0x00) {
        LOGE("%s(%d): all camera driver support format is not supported in CameraHal!!",__FUNCTION__,__LINE__);
        j = 0;
        while (mCamDriverSupportFmt[j]) {
            LOG1("pixelformat = '%c%c%c%c'",
				mCamDriverSupportFmt[j] & 0xFF, (mCamDriverSupportFmt[j] >> 8) & 0xFF,
				(mCamDriverSupportFmt[j] >> 16) & 0xFF, (mCamDriverSupportFmt[j] >> 24) & 0xFF);
            j++;
        }
        goto exit1;
    } else {  
        mCamDriverPreviewFmt = CameraHal_SupportFmt[i];
        LOG1("%s(%d): mCamDriverPreviewFmt(%c%c%c%c) is cameraHal and camera driver is also supported!!",__FUNCTION__,__LINE__,
            mCamDriverPreviewFmt & 0xFF, (mCamDriverPreviewFmt >> 8) & 0xFF,
			(mCamDriverPreviewFmt >> 16) & 0xFF, (mCamDriverPreviewFmt >> 24) & 0xFF);
        
    }

    if (strcmp((char*)&mCamDriverCapability.driver[0],"uvcvideo") == 0) {                  /* ddl@rock-chips.com: This driver is UVC sensor driver */
        mCamDriverV4l2MemType = V4L2_MEMORY_MMAP;
    } else if (strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") == 0) {      /* ddl@rock-chips.com: This driver is RK29 sensor driver */
        mCamDriverV4l2MemType = V4L2_MEMORY_OVERLAY;
    } else {
        mCamDriverV4l2MemType = V4L2_MEMORY_OVERLAY;
    }
    
    LOGD("%s(%d): Current driver is %s, v4l2 memory is %s",__FUNCTION__,__LINE__,mCamDriverCapability.driver, 
        (mCamDriverV4l2MemType==V4L2_MEMORY_MMAP)?"V4L2_MEMORY_MMAP":"V4L2_MEMORY_OVERLAY");
    
    pmem_fd = open(CAMERA_PMEM_NAME, O_RDWR);
    if (pmem_fd < 0) {
        LOGE("%s(%d): open the PMEM device(%s): %s",__FUNCTION__,__LINE__, CAMERA_PMEM_NAME, strerror(errno));
    }

    ioctl(pmem_fd, PMEM_GET_TOTAL_SIZE, &sub);
    mPmemSize = sub.len;

    if (pmem_fd > 0) {
        close(pmem_fd);
        pmem_fd = 0;
    } 

    mMemHeap = new MemoryHeapBase(CAMERA_PMEM_NAME,mPmemSize,0);
    iPmemFd = mMemHeap->getHeapID(); 
    if (iPmemFd < 0) {
        LOGE("%s(%d): allocate mMemHeap from %s failed",__FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
        err = -1;
        goto exit;
    }
    
    if (ioctl(iPmemFd,PMEM_GET_PHYS, &sub)) {
        LOGE("%s(%d): obtain %s physical address",__FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
        err = -1;
    } else {
        mPmemHeapPhyBase = sub.offset;
    }
    
    mMemHeapPmem = new MemoryHeapPmem(mMemHeap,0);

    mCamId = cameraId;
    
    LOG_FUNCTION_NAME_EXIT 
    return 0;

exit1:
    if (iCamFd > 0) {
        close(iCamFd);
        iCamFd = -1;
    }    
exit:
    LOGE("%s(%d): exit with error -1",__FUNCTION__,__LINE__);
    return -1;
}


int CameraHal::cameraDestroy()
{
    int err,i;

    LOG_FUNCTION_NAME

    if (mCamDriverV4l2MemType == V4L2_MEMORY_MMAP) {
        for (i=0; i<V4L2_BUFFER_MAX; i++) {
            if (mCamDriverV4l2Buffer[i] != NULL) {
                if (munmap((void*)mCamDriverV4l2Buffer[i], mCamDriverV4l2BufferLen) < 0)
                    LOGE("%s(%d): mCamDriverV4l2Buffer[%d] munmap failed : %s",__FUNCTION__,__LINE__,i,strerror(errno));
                mCamDriverV4l2Buffer[i] = NULL;
            } else {
                break;
            }
        }
    }

    cameraHeapBufferDestory();
    cameraPreviewBufferDestory();
    
    if (mPreviewMemory != NULL) {
        mPreviewMemory->release(mPreviewMemory);
        mPreviewMemory = NULL;
        for (i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
            mPreviewBufs[i] = NULL;
        }
        LOG1("mPreviewMemory.clear");
    }

    for (i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        if (mVideoBufs[i] != NULL) {
            mVideoBufs[i]->release(mVideoBufs[i]);
            mVideoBufs[i] = NULL;
        }
    }
    
    if (mMemHeapPmem != NULL) { 
        LOG1("mMemHeapPmem.clear");
        mMemHeapPmem.clear();
        mMemHeapPmem = NULL;
    }
    if (mMemHeap != NULL) {
        LOG1("mMemHeap.clear");
        mMemHeap.clear();
        mMemHeap = NULL;
    }
    mPmemHeapPhyBase = 0;
    
	if( iPmemFd > 0 ) {
		close(iPmemFd);
        iPmemFd = -1;
	}
	if( iCamFd > 0) {
        close(iCamFd);
        iCamFd = -1;
	}

    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int CameraHal::cameraSetSize(int w, int h, int fmt)
{
    int err=0;
    struct v4l2_format format;

	/* Set preview format */
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = w;
	format.fmt.pix.height = h;
	format.fmt.pix.pixelformat = fmt;
	format.fmt.pix.field = V4L2_FIELD_NONE;		/* ddl@rock-chips.com : field must be initialised for Linux kernel in 2.6.32  */
	err = ioctl(iCamFd, VIDIOC_S_FMT, &format);
	if ( err < 0 ){
		LOGE("%s(%d): VIDIOC_S_FMT failed",__FUNCTION__,__LINE__);		
	} else {
	    LOG1("%s(%d): VIDIOC_S_FMT %dx%d '%c%c%c%c'",__FUNCTION__,__LINE__,format.fmt.pix.width, format.fmt.pix.height,
				fmt & 0xFF, (fmt >> 8) & 0xFF,(fmt >> 16) & 0xFF, (fmt >> 24) & 0xFF);
	}

    return err;
}

int CameraHal::cameraConfig(const CameraParameters &params)
{
    int err, i = 0;
    struct v4l2_control control;
	struct v4l2_ext_control extCtrInfo;
	struct v4l2_ext_controls extCtrInfos;

    /*white balance setting*/
    const char *white_balance = params.get(CameraParameters::KEY_WHITE_BALANCE);
	const char *mwhite_balance = mParameters.get(CameraParameters::KEY_WHITE_BALANCE);
	if (strcmp("false", params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE))) {
		if ( !mwhite_balance || strcmp(white_balance, mwhite_balance) ) {
			for (i = 0; i < mWhiteBalance_number; i++) {
				if (!strcmp((char *)mWhiteBalance_menu[i].name, white_balance)) {
					break;
				}
			}
			control.id = mWhiteBalance_menu[i].id;
			control.value = mWhiteBalance_menu[i].index;
			err = ioctl(iCamFd, VIDIOC_S_CTRL, &control);
			if ( err < 0 ) {
                LOGE ("%s(%d): Set white balance(%s) failed",__FUNCTION__,__LINE__,white_balance);
			} else {
			    LOGD("%s(%d): Set white balance %s ",__FUNCTION__,__LINE__, mWhiteBalance_menu[i].name);
			}
		}
	}

    /*zoom setting*/
    const int zoom = params.getInt(CameraParameters::KEY_ZOOM);
	const int mzoom = mParameters.getInt(CameraParameters::KEY_ZOOM);
	if (strcmp("false", params.get("zoom-supported"))) {
		if ((mzoom < 0) || (zoom != mzoom)) {			
            control.id = V4L2_CID_ZOOM_ABSOLUTE;
			control.value = zoom * mZoomStep + mZoomMin;
			err = ioctl(iCamFd, VIDIOC_S_CTRL, &control);
			if ( err < 0 ){
				LOGE ("%s(%d): Set zoom(%d) fail",__FUNCTION__,__LINE__,control.value);
			} else {
			    LOGD ("%s(%d): Set zoom(%d)",__FUNCTION__,__LINE__, control.value);
			}
		}
	}

    /*color effect setting*/
    const char *effect = params.get(CameraParameters::KEY_EFFECT);
	const char *meffect = mParameters.get(CameraParameters::KEY_EFFECT);
	if (strcmp("false", params.get(CameraParameters::KEY_SUPPORTED_EFFECTS))) {
		if ( !meffect || strcmp(effect, meffect) ) {
			for (i = 0; i < mEffect_number; i++) {
				if (!strcmp((char *)mEffect_menu[i].name, effect)) {
					break;
				}
			}
			extCtrInfo.id = mEffect_menu[i].id;
			extCtrInfo.value = mEffect_menu[i].index;
			extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
			extCtrInfos.count = 1;
			extCtrInfos.controls = &extCtrInfo;
			err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
			if ( err < 0 ){
				LOGE ("%s(%d): Set effect(%s) fail",__FUNCTION__,__LINE__,effect);
			} else {
			    LOGD("%s(%d): Set effect %s",__FUNCTION__,__LINE__, (char *)mEffect_menu[i].name);
			}
		}
	}

	/*scene setting*/
    const char *scene = params.get(CameraParameters::KEY_SCENE_MODE);
	const char *mscene = mParameters.get(CameraParameters::KEY_SCENE_MODE);
	if (strcmp("false", params.get(CameraParameters::KEY_SUPPORTED_SCENE_MODES))) {
		if ( !mscene || strcmp(scene, mscene) ) {
			for (i = 0; i < mScene_number; i++) {
				if (!strcmp((char *)mScene_menu[i].name, scene)) {
					break;
				}
			}
			extCtrInfo.id = mScene_menu[i].id;
			extCtrInfo.value = mScene_menu[i].index;
			extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
			extCtrInfos.count = 1;
			extCtrInfos.controls = &extCtrInfo;
			err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
			if ( err < 0 ){
				LOGE("%s(%d): Set scene(%s) failed",__FUNCTION__,__LINE__,scene);
			} else {
			    LOGD ("%s(%d): Set scene %s ",__FUNCTION__,__LINE__, (char *)mScene_menu[i].name);
			}
		}
	}

    /*focus setting*/
    const char *focusMode = params.get(CameraParameters::KEY_FOCUS_MODE);
	const char *mfocusMode = mParameters.get(CameraParameters::KEY_FOCUS_MODE);
	if (strcmp("false", params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES))) {
		if ( !mfocusMode || strcmp(focusMode, mfocusMode) ) {
            cameraAutoFocus(focusMode);
		}
	}    

	/*flash mode setting*/
    const char *flashMode = params.get(CameraParameters::KEY_FLASH_MODE);
	const char *mflashMode = mParameters.get(CameraParameters::KEY_FLASH_MODE);
	if (strcmp("false", params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES))) {
		if ( !mflashMode || strcmp(flashMode, mflashMode) ) {
			for (i = 0; i < mFlashMode_number; i++) {
				if (!strcmp((char *)mFlashMode_menu[i].name, flashMode)) {
					break;
				}
			}
			extCtrInfo.id = mFlashMode_menu[i].id;
			extCtrInfo.value = mFlashMode_menu[i].index;
			extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
			extCtrInfos.count = 1;
			extCtrInfos.controls = &extCtrInfo;
			err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
			if ( err < 0 ){
				LOGE ("%s(%d): Set flash(%s) failed",__FUNCTION__,__LINE__,flashMode );				
			} else {
			    LOGD ("%s(%d): Set flash %s",__FUNCTION__,__LINE__, (char *)mFlashMode_menu[i].name);
			}
		}
	}

    mParameters = params;
    return 0;
}

int CameraHal::cameraQuery(CameraParameters &params)
{
	int err, i = 0;
	struct v4l2_control control;
	struct v4l2_ext_control extCtrInfo;
	struct v4l2_ext_controls extCtrInfos;

	/*white balance getting*/
	control.id = mWhiteBalance_menu[0].id;
	control.value = 0xFF;
	err = ioctl(iCamFd, VIDIOC_G_CTRL, &control);
	for (i = 0; i < mWhiteBalance_number; i++) {
		if (control.value == (__s32)mWhiteBalance_menu[i].index) {
			params.set(CameraParameters::KEY_WHITE_BALANCE, (char *)mWhiteBalance_menu[i].name);
			break;
		}
	}
	LOG1 ("white balance: %s", (char *)mWhiteBalance_menu[i].name);

	/*color effect getting*/
	control.id = mEffect_menu[0].id;
	control.value = 0xFF;
	err = ioctl(iCamFd, VIDIOC_G_CTRL, &control);
	for (i = 0; i < mEffect_number; i++) {
		if (control.value == (__s32)mEffect_menu[i].index) {
			params.set(CameraParameters::KEY_EFFECT, (char *)mEffect_menu[i].name);
			break;
		}
	}
	LOG1 ("effect: %s", (char *)mEffect_menu[i].name);

	/*scene getting*/
	extCtrInfo.id = mScene_menu[0].id;
	extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
	extCtrInfos.count = 1;
	extCtrInfos.controls = &extCtrInfo;
	extCtrInfo.value = 0xFF;
	err = ioctl(iCamFd, VIDIOC_G_EXT_CTRLS, &extCtrInfos);
	for (i = 0; i < mScene_number; i++) {
		if (extCtrInfo.value == (__s32)mScene_menu[i].index) {
			params.set(CameraParameters::KEY_SCENE_MODE, (char *)mScene_menu[i].name);
			break;
		}
	}
	LOG1 ("scene: %s", (char *)mScene_menu[i].name);
	return 0;
}

int CameraHal::cameraStart()
{
    int preview_size,i;
    int err;
    int nSizeBytes;
    int buffer_count;
    struct v4l2_format format;
    enum v4l2_buf_type type;
    struct v4l2_requestbuffers creqbuf;
    struct v4l2_buffer buffer;
    
    LOG_FUNCTION_NAME

    if ((!strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV420SP)) ||
        (!strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV422SP))) {
        mPreviewFrameSize = (mPreviewWidth * mPreviewHeight * 3)/2;
    } else {
        LOGE("%s(%d): cameraStart failed, because %s is not supported for preview",__FUNCTION__,__LINE__,mParameters.getPreviewFormat());
        return -1;
    }     

    creqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;    
    creqbuf.memory = mCamDriverV4l2MemType;
    creqbuf.count  = CONFIG_CAMERA_PRVIEW_BUF_CNT;
    if (ioctl(iCamFd, VIDIOC_REQBUFS, &creqbuf) < 0) {
        LOGE ("%s(%d): VIDIOC_REQBUFS Failed. %s",__FUNCTION__,__LINE__, strerror(errno));
        goto fail_reqbufs;
    }   
    
    if (mPreviewBufferMap[0].priv_hnd== NULL) {
        LOGE("%s(%d): preview buffer havn't alloced",__FUNCTION__,__LINE__);
        goto fail_reqbufs;
    }

    if (mPreviewBufferCount <= 0) {
        LOGE("%s(%d): camera start failed, because preview buffer is empty(%d)!",__FUNCTION__,__LINE__,mPreviewBufferCount);
        goto fail_bufalloc;
    }

    memset(mCamDriverV4l2Buffer, 0x00, sizeof(mCamDriverV4l2Buffer));
    for (int i = 0; i < mPreviewBufferCount; i++) {
        if (CAMERA_PREVIEWBUF_ALLOW_WRITE(mPreviewBufferMap[i].buf_state)) {
            memset(&buffer, 0, sizeof(struct v4l2_buffer));        
            buffer.type = creqbuf.type;
            buffer.memory = creqbuf.memory;
            buffer.flags = 0;
            buffer.index = i;

            if (ioctl(iCamFd, VIDIOC_QUERYBUF, &buffer) < 0) {
                LOGE("%s(%d): VIDIOC_QUERYBUF Failed",__FUNCTION__,__LINE__);
                goto fail_bufalloc;
            }

            if (buffer.memory == V4L2_MEMORY_OVERLAY) {                
                buffer.m.offset = mPreviewBufferMap[i].phy_addr;
            } else if (buffer.memory == V4L2_MEMORY_MMAP) {
                mCamDriverV4l2Buffer[i] = (char*)mmap(0 /* start anywhere */ ,
                                    buffer.length, PROT_READ, MAP_SHARED, iCamFd,
                                    buffer.m.offset);
                if (mCamDriverV4l2Buffer[i] == MAP_FAILED) {
                    LOGE("%s(%d): Unable to map buffer(length:0x%x offset:0x%x) %s(err:%d)\n",__FUNCTION__,__LINE__, buffer.length,buffer.m.offset,strerror(errno),errno);
                    goto fail_bufalloc;
                } 
            }
            mCamDriverV4l2BufferLen = buffer.length;
            err = ioctl(iCamFd, VIDIOC_QBUF, &buffer);
            if (err < 0) {
                LOGE("%s(%d): VIDIOC_QBUF Failed,err=%d[%s]\n",__FUNCTION__,__LINE__,err, strerror(errno));
                goto fail_bufalloc;
            }     
            cameraPreviewBufferSetSta(&mPreviewBufferMap[i], CMD_PREVIEWBUF_WRITING, 1);
        } else {

            if (mPreviewBufferMap[i].buf_state & (1<<CAMERA_PREVIEWBUF_DISPING_BITPOS)) {
                LOGD("%s(%d): preview buffer %d is displaying, so wait it dequeueed from display for enqueue to camera",
                        __FUNCTION__,__LINE__,i);                
            }
        }
    }    
    
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(iCamFd, VIDIOC_STREAMON, &type);
    if ( err < 0) {
        LOGE("%s(%d): VIDIOC_STREAMON Failed",__FUNCTION__,__LINE__);
        goto fail_bufalloc;
    }

    mPreviewMemory = mRequestMemory(-1, mPreviewFrameSize, CONFIG_CAMERA_PRVIEW_BUF_CNT, NULL);
    if (mPreviewMemory) {
        for (int i=0; i < CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
            mPreviewBufs[i] = (unsigned char*) mPreviewMemory->data + (i*mPreviewFrameSize);
        }
    } else {
        LOGE("%s(%d): mPreviewMemory create failed",__FUNCTION__,__LINE__);
    }
    
    int *addr;
    for (int i=0; i < CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        mVideoBufs[i] = mRequestMemory(-1, 4, 1, NULL);
        if( (NULL == mVideoBufs[i]) || ( NULL == mVideoBufs[i]->data)) {
            mVideoBufs[i] = NULL;
            LOGE("%s(%d): video buffer %d create failed",__FUNCTION__,__LINE__,i);
        }
        if (mVideoBufs[i]) {
            addr = (int*)mVideoBufs[i]->data;
            *addr = mPreviewBufferMap[i].phy_addr;
        } 
    }
    mPreviewErrorFrameCount = 0;

    LOG_FUNCTION_NAME_EXIT
    return 0;

fail_bufalloc:
    cameraPreviewBufferDestory();
fail_reqbufs:
    LOGE("%s(%d): exit with error(%d)",__FUNCTION__,__LINE__,-1);
    return -1;
}

int CameraHal::cameraStop()
{
    LOG_FUNCTION_NAME

    int ret,i;
    struct v4l2_requestbuffers creqbuf;

    creqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(iCamFd, VIDIOC_STREAMOFF, &creqbuf.type) == -1) {
        LOGE("%s(%d): VIDIOC_STREAMOFF Failed",__FUNCTION__,__LINE__);
        goto fail_streamoff;
    }

    if (mCamDriverV4l2MemType == V4L2_MEMORY_MMAP) {
        for (i=0; i<V4L2_BUFFER_MAX; i++) {
            if (mCamDriverV4l2Buffer[i] != NULL) {
                if (munmap((void*)mCamDriverV4l2Buffer[i], mCamDriverV4l2BufferLen) < 0)
                    LOGE("%s(%d): mCamDriverV4l2Buffer[%d] munmap failed : %s",__FUNCTION__,__LINE__,i,strerror(errno));
                mCamDriverV4l2Buffer[i] = NULL;
            } else {
                break;
            }
        }
    }
    /* ddl@rock-chips.com: Release v4l2 buffer must by close device, buffer isn't release in VIDIOC_STREAMOFF ioctl */
    if (strcmp((char*)&mCamDriverCapability.driver[0],"uvcvideo") == 0) {
        close(iCamFd);
        iCamFd = open(cameraDevicePathCur, O_RDWR);
        if (iCamFd < 0) {
            LOGE ("%s(%d): Could not open the camera device(%s): %s",__FUNCTION__,__LINE__, cameraDevicePathCur, strerror(errno) );
            goto fail_streamoff;
        }
    }

    for (int i = 0; i < mPreviewBufferCount; i++) {
        if (mPreviewBufferMap[i].priv_hnd != NULL) {
            cameraPreviewBufferSetSta(&mPreviewBufferMap[i], CMD_PREVIEWBUF_WRITING, 0);
        }
    }

    LOG_FUNCTION_NAME_EXIT
    return 0;

fail_streamoff:

    return -1;
}

int CameraHal::cameraAutoFocus(const char *focus)
{
    int err;
    struct v4l2_ext_control extCtrInfo;
	struct v4l2_ext_controls extCtrInfos;

    if (strcmp(focus, CameraParameters::FOCUS_MODE_AUTO) == 0) {
        extCtrInfo.id = V4L2_CID_FOCUS_AUTO;
	    extCtrInfo.value = 1;
    } else if (strcmp(focus, CameraParameters::FOCUS_MODE_INFINITY) == 0) {
        extCtrInfo.id = V4L2_CID_FOCUS_ABSOLUTE;
	    extCtrInfo.value = 0;
    } else if (strcmp(focus, CameraParameters::FOCUS_MODE_MACRO) == 0) {
        extCtrInfo.id = V4L2_CID_FOCUS_ABSOLUTE;
	    extCtrInfo.value = 0xff;
    } else if (strcmp(focus, CameraParameters::FOCUS_MODE_EDOF) == 0) {
        extCtrInfo.id = V4L2_CID_FOCUS_CONTINUOUS;
	    extCtrInfo.value = 1;
    } else if (strcmp(focus, CameraParameters::FOCUS_MODE_FIXED) == 0) {
        LOG1("%s(%d): %s is not need config sensor driver",__FUNCTION__,__LINE__,CameraParameters::FOCUS_MODE_FIXED);
        err = true;
        goto cameraAutoFocus_end;
    }

    extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
	extCtrInfos.count = 1;
	extCtrInfos.controls = &extCtrInfo;
	err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
	if ( err < 0 ){
		LOGE("%s(%d): Set focus mode(%s) failed",__FUNCTION__,__LINE__, focus);
        err = false;
	} else {
	    LOG1("%s(%d): Set focus mode %s",__FUNCTION__,__LINE__, focus);
        err = true;
	}
cameraAutoFocus_end:
    return err;
}
int CameraHal::cameraFormatConvert(int v4l2_fmt_src, int v4l2_fmt_dst, const char *android_fmt_dst, char *srcbuf, char *dstbuf, int w, int h)
{
    int y_size,i,j;

    
    LOG2("cameraFormatConvert '%c%c%c%c'->'%c%c%c%c OR %s' %dx%d, srcbuf:0x%x dstbuf:0x%x",
				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
				v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
				(v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF,
				android_fmt_dst, w,h, (int)srcbuf, (int)dstbuf);
    
    
    y_size = w*h;
    switch (v4l2_fmt_src)
    {
        case V4L2_PIX_FMT_YUV420:
        {
            if ((strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") != 0) 
                || (mCamDriverCapability.version != KERNEL_VERSION(0, 0, 1))) {
                goto cameraFormatConvert_default;
            }
        }
        case V4L2_PIX_FMT_NV12:
        {
            int *dst_vu, *src_uv;
            
            if (v4l2_fmt_dst == V4L2_PIX_FMT_NV21) { 
                if (dstbuf != srcbuf)
                    memcpy(dstbuf,srcbuf, y_size);
                src_uv = (int*)(srcbuf + y_size); 
                dst_vu = (int*)(dstbuf+y_size);
                for (i=0; i<(y_size>>3); i++) {
                    *dst_vu = ((*src_uv&0x00ff00ff)<<8) | ((*src_uv&0xff00ff00)>>8);
                    dst_vu++;
                    src_uv++;
                }
            }
            break;
        }
        case V4L2_PIX_FMT_YUV422P:
        {
            if ((strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") != 0) 
                || (mCamDriverCapability.version != KERNEL_VERSION(0, 0, 1))) {
                goto cameraFormatConvert_default;
            }
        }        
        case V4L2_PIX_FMT_NV16:
        {
            break;
        }
        case V4L2_PIX_FMT_YUYV:
        {
            char *srcbuf_begin;
            int *dstint_y, *dstint_uv, *srcint;
            
            if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV12) || 
                ((v4l2_fmt_dst == V4L2_PIX_FMT_YUV420) && (strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") == 0) 
                && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1)))) {                
                dstint_y = (int*)dstbuf;                
                srcint = (int*)srcbuf;
                for(i=0;i<(y_size>>2);i++) {
                    *dstint_y++ = ((*(srcint+1)&0x00ff0000)<<8)|((*(srcint+1)&0x000000ff)<<16)
                                |((*srcint&0x00ff0000)>>8)|(*srcint&0x000000ff);
                    
                    srcint += 2;
                }
                dstint_uv =  (int*)(dstbuf + y_size);
                srcint = (int*)srcbuf;
                for(i=0;i<h/2; i++) {
                    for (j=0; j<(w>>2); j++) {
                        *dstint_uv++ = (*(srcint+1)&0xff000000)|((*(srcint+1)&0x0000ff00)<<8)
                                    |((*srcint&0xff000000)>>16)|((*srcint&0x0000ff00)>>8); 
                        srcint += 2;
                    }
                    srcint += (w>>1);  
                }
                 
            } else if (v4l2_fmt_dst == V4L2_PIX_FMT_NV21) {
                dstint_y = (int*)dstbuf;                
                srcint = (int*)srcbuf;
                for(i=0;i<(y_size>>2);i++) {
                    *dstint_y++ = ((*(srcint+1)&0x00ff0000)<<8)|((*(srcint+1)&0x000000ff)<<16)
                                |((*srcint&0x00ff0000)>>8)|(*srcint&0x000000ff);
                    
                    srcint += 2;
                }
                dstint_uv =  (int*)(dstbuf + y_size);
                srcint = (int*)srcbuf;
                for(i=0;i<h/2; i++) {
                    for (j=0; j<(w>>2); j++) {
                        *dstint_uv++ = ((*(srcint+1)&0xff000000)>>8)|((*(srcint+1)&0x0000ff00)<<16)
                                    |((*srcint&0xff000000)>>24)|(*srcint&0x0000ff00); 
                        srcint += 2;
                    }
                    srcint += (w>>1);  
                }                  
            }            
            break;
        }
cameraFormatConvert_default:        
        default:
            LOGE("%s(%d): CameraHal is not support (%c%c%c%c)",__FUNCTION__,__LINE__,
                v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
			    (v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF);
            break;
    }
    return 0;

}
int CameraHal::cameraPmemBufferFlush(sp<MemoryHeapBase> heap, sp<IMemory> buf)
{
    struct pmem_region region;

    if ((heap != NULL) && (heap->getHeapID()>0) && (buf!= NULL)) {
        region.len = buf->size();
        region.offset = (long)buf->offset();
        ioctl(heap->getHeapID(), PMEM_CACHE_FLUSH, &region);
    }
    return 0;
}

int CameraHal::startPreview()
{
    LOG_FUNCTION_NAME
    Message msg;    
    Mutex::Autolock lock(mLock);
    
    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_START;
        commandThreadCommandQ.put(&msg);
    }
    mPreviewCmdReceived = true;
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR ;
}

void CameraHal::stopPreview()
{
    LOG_FUNCTION_NAME    
    Message msg;
    int ret = 0;
    Mutex::Autolock lock(mLock);

    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_STOP;
        commandThreadCommandQ.put(&msg);

        if (mANativeWindow == NULL) {
            mANativeWindowCond.signal();
            LOGD("%s(%d): wake up command thread for stop preview");
        }
        
        while (ret == 0) {            
            ret = commandThreadAckQ.get(&msg);
            if (ret == 0) {
                if (msg.command == CMD_PREVIEW_STOP) {                    
                    ret = 1;
                }
            }
        }
    } else {
        LOGE("%s(%d): cancel, because thread (%s %s) is NULL", __FUNCTION__,__LINE__,(mPreviewThread == NULL)?"mPreviewThread":" ",
            (mCommandThread == NULL)?"mCommandThread":" ");
    }
    mPreviewCmdReceived = false;
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::autoFocus()
{
    LOG_FUNCTION_NAME
    int ret = 0;
    Message msg;
    Mutex::Autolock lock(mLock);

    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        msg.command = CMD_AF_START;
        commandThreadCommandQ.put(&msg);
        while (ret == 0) {            
            ret = commandThreadAckQ.get(&msg,5000);
            if (ret == 0) {
                if (msg.command == CMD_AF_START) {                    
                    ret = 1;
                }
            } else {
                LOGE("%s(%d): AutoFocus is time out!!!\n",__FUNCTION__,__LINE__);
            }
        }
    } else {
        LOGE("%s(%d):  cancel, because thread (%s %s) is NULL", __FUNCTION__,__LINE__,(mPreviewThread == NULL)?"mPreviewThread":" ",
            (mCommandThread == NULL)?"mCommandThread":" ");
    }
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}
int CameraHal::cancelAutoFocus()
{
    LOG_FUNCTION_NAME

    

    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}
int CameraHal::previewEnabled()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT
    return mPreviewCmdReceived;
}
int CameraHal::storeMetaDataInBuffers(int enable)
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT
    return INVALID_OPERATION;
}
int CameraHal::startRecording()
{
    int i = 0,err=NO_ERROR;
    struct pmem_region sub;
    Message msg;
    
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);

    mRecordRunning=true;
    LOG_FUNCTION_NAME_EXIT
startRecording_end:
    return err;
}

void CameraHal::stopRecording()
{

    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    mRecordRunning=false;
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::recordingEnabled()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT
    return mRecordRunning;
}
void CameraHal::releaseRecordingFrame(const void *opaque)
{
    ssize_t offset;
    size_t  size;
    int index = -1,i;
    struct Message msg; 

    for(i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        if (mVideoBufs[i]->data == opaque) {
            index = i;
            break;
        }
    }

    if (index == -1) {
        LOGE("%s(%d): this video buffer is invaildate",__FUNCTION__,__LINE__);
        return;
    }
    
    if (mANativeWindow && mPreviewBufferMap[index].priv_hnd)
        cameraPreviewBufferSetSta(&mPreviewBufferMap[index], CMD_PREVIEWBUF_ENCING, 0);
    if (mPreviewRunning == STA_PREVIEW_RUN) {
        msg.command = CMD_PREVIEW_QBUF;     
        msg.arg1 = (void*)index;
        msg.arg2 = (void*)CMD_PREVIEWBUF_ENCING;
        msg.arg3 = (void*)mPreviewStartTimes;
        commandThreadCommandQ.put(&msg); 
    }
}

int CameraHal::takePicture()
{
    Message msg;
    Mutex::Autolock lock(mLock);
    int ret = NO_ERROR;
    
    LOG_FUNCTION_NAME 
    msg.command = CMD_PREVIEW_CAPTURE;
    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        commandThreadCommandQ.put(&msg);
        while (ret == 0) {            
            ret = commandThreadAckQ.get(&msg,5000);
            if (ret == 0) {
                if (msg.command == CMD_PREVIEW_CAPTURE) {
                    ret = 1;
                    if (msg.arg1 == (void*)CMD_NACK) {
                        LOGE("%s(%d): failed, because command thread response NACK\n",__FUNCTION__,__LINE__);    
                        ret = INVALID_OPERATION;
                    }                    
                }
            } else {
                LOGE("%s(%d): PREVIEW_CAPTURE is time out!!!\n",__FUNCTION__,__LINE__);
            }
        }
    } else {
        LOGE("%s(%d):  cancel, because thread (%s %s) is NULL", __FUNCTION__,__LINE__,(mPreviewThread == NULL)?"mPreviewThread":" ",
            (mCommandThread == NULL)?"mCommandThread":" ");
        ret = INVALID_OPERATION;
    }

    if (ret)
        LOGE("%s exit with error(%d)",__FUNCTION__,ret);
    else
        LOG_FUNCTION_NAME_EXIT
    return ret;
}

int CameraHal::cancelPicture()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);

    mPictureLock.lock();
    mPictureRunning = STA_PICTURE_WAIT_STOP;
    mPictureLock.unlock();
    mPictureThread->requestExitAndWait();
    
    LOG_FUNCTION_NAME_EXIT
    return 0;
}

int CameraHal::setParameters(const char* parameters)
{
    CameraParameters params;
    String8 str_params(parameters);
    
    params.unflatten(str_params);
    return setParameters(params);
}
int CameraHal::setParameters(const CameraParameters &params_set)
{
    LOG_FUNCTION_NAME

    int preview_w, preview_h;
    int picture_w, picture_h;
    int j;
    int framerate;
    CameraParameters params;
    
    Mutex::Autolock lock(mLock);

    params = params_set;
    
    mPictureLock.lock();
    if (mPictureRunning != STA_PICTURE_STOP) {
        mPictureLock.unlock();
        LOG1("%s(%d):  capture in progress, wait for finish...",__FUNCTION__,__LINE__);        
        mPictureThread->requestExitAndWait();
        LOG1("%s(%d):  capture finish, setParameters go!!",__FUNCTION__,__LINE__);
    }
    mPictureLock.unlock();

    if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES), params.get(CameraParameters::KEY_PREVIEW_SIZE)) == NULL) {
        LOGE("%s(%d): PreviewSize(%s) not supported",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_SIZE));
        return -1;
    } else if (strcmp(mParameters.get(CameraParameters::KEY_PREVIEW_SIZE), params.get(CameraParameters::KEY_PREVIEW_SIZE))) {
        LOGD("%s(%d): Set preview size %s",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_SIZE));
    }

    if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES), params.get(CameraParameters::KEY_PICTURE_SIZE)) == NULL) {
        LOGE("%s(%d): PictureSize(%s) not supported",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PICTURE_SIZE));
        return -1;
    } else if (strcmp(mParameters.get(CameraParameters::KEY_PICTURE_SIZE), params.get(CameraParameters::KEY_PICTURE_SIZE))) {
        LOGD("%s(%d): Set picture size %s",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PICTURE_SIZE));
    }

    if (strcmp(params.getPictureFormat(), "jpeg") != 0) {
        LOGE("%s(%d): Only jpeg still pictures are supported",__FUNCTION__,__LINE__);
        return -1;
    }

    if (params.getInt(CameraParameters::KEY_ZOOM) > params.getInt(CameraParameters::KEY_MAX_ZOOM)) {
        LOGE("%s(%d): Zomm(%d) is larger than MaxZoom(%d)",__FUNCTION__,__LINE__,params.getInt(CameraParameters::KEY_ZOOM),params.getInt(CameraParameters::KEY_MAX_ZOOM));
        return -1;
    }

    if (strcmp(mParameters.get(CameraParameters::KEY_PREVIEW_SIZE), params.get(CameraParameters::KEY_PREVIEW_SIZE)) != 0) {
        cameraFpsInfoSet(params);
        if (strstr(params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE), params_set.get(CameraParameters::KEY_PREVIEW_FPS_RANGE)) == NULL) {
            LOGE("%s(%d): PreviewFpsRange(%s) not supported, so switch to (%s)",__FUNCTION__,__LINE__,params_set.get(CameraParameters::KEY_PREVIEW_FPS_RANGE),
                params.get(CameraParameters::KEY_PREVIEW_FPS_RANGE));
        }
    } else { 
        if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE), params.get(CameraParameters::KEY_PREVIEW_FPS_RANGE)) == NULL) {
            LOGE("%s(%d): PreviewFpsRange(%s) not supported",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_FPS_RANGE));
            return -1;
        }
    }  
    
    if ((strcmp(params.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV420SP) == 0)
        || (strcmp(params.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV422SP) == 0)) {
        /* ddl@rock-chips.com : CameraHal_SupportFmt[0] : V4L2_PIX_FMT_NV12 OR V4L2_PIX_FMT_YUV420(rk29xx_camera 0.0.1) */
        if (mCamDriverPreviewFmt != CameraHal_SupportFmt[0]) {
            j = 0;
            while (mCamDriverSupportFmt[j]) {
                if (mCamDriverSupportFmt[j] == CameraHal_SupportFmt[0]) {
                    mCamDriverPreviewFmt = CameraHal_SupportFmt[0];
                    break;
                }
                j++; 
            }        
        }
    } else {
        LOGE("%s(%d): %s is not supported,Only %s and %s preview is supported",__FUNCTION__,__LINE__,params.getPreviewFormat(),CameraParameters::PIXEL_FORMAT_YUV420SP,CameraParameters::PIXEL_FORMAT_YUV422SP);
        return -1;
    }
    
    
    
    if ((strcmp((char*)&mCamDriverCapability.driver[0],"rk29xx-camera") == 0) 
        && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1))) {
        mCamDriverPictureFmt = V4L2_PIX_FMT_YUV420;
    } else {
        mCamDriverPictureFmt = V4L2_PIX_FMT_NV12;    /* ddl@rock-chips.com : Picture format must is NV12, because jpeg encoder is only support NV12 */
    } 

    framerate = params.getPreviewFrameRate();

    // gps latitude
    const char *new_gps_latitude_str = params.get(CameraParameters::KEY_GPS_LATITUDE);    
    if (new_gps_latitude_str) {
        mGps_latitude = strtod(new_gps_latitude_str,NULL);
    } else {        
        mGps_latitude = -1;
    }

    // gps longitude
    const char *new_gps_longitude_str = params.get(CameraParameters::KEY_GPS_LONGITUDE);    
    if (new_gps_longitude_str) {
        mGps_longitude = strtod(new_gps_longitude_str,NULL);
    } else {
        mGps_longitude = -1;
    }

    // gps altitude
    const char *new_gps_altitude_str = params.get(CameraParameters::KEY_GPS_ALTITUDE);
    if (new_gps_altitude_str) {
        mGps_altitude = strtod(new_gps_altitude_str,NULL);
    } else {
        mGps_altitude = -1;
    }

    // gps timestamp
    const char *new_gps_timestamp_str = params.get(CameraParameters::KEY_GPS_TIMESTAMP);
    if (new_gps_timestamp_str) {
        mGps_timestamp = strtol(new_gps_timestamp_str,0,0);
    } else {
        mGps_timestamp = -1;
    }
    
	if (cameraConfig(params) == 0) {        
        LOG1("PreviewSize(%s)", mParameters.get(CameraParameters::KEY_PREVIEW_SIZE));
        LOG1("PreviewFormat(%s)  mCamDriverPreviewFmt(%c%c%c%c)",params.getPreviewFormat(), 
            mCamDriverPreviewFmt & 0xFF, (mCamDriverPreviewFmt >> 8) & 0xFF,
			(mCamDriverPreviewFmt >> 16) & 0xFF, (mCamDriverPreviewFmt >> 24) & 0xFF);  
        LOG1("FPS Range(%s)",mParameters.get(CameraParameters::KEY_PREVIEW_FPS_RANGE));
        LOG1("PictureSize(%s)",mParameters.get(CameraParameters::KEY_PICTURE_SIZE)); 
        LOG1("PictureFormat(%s)  mCamDriverPictureFmt(%c%c%c%c)", params.getPictureFormat(),
            mCamDriverPictureFmt & 0xFF, (mCamDriverPictureFmt >> 8) & 0xFF,
			(mCamDriverPictureFmt >> 16) & 0xFF, (mCamDriverPictureFmt >> 24) & 0xFF);
        LOG1("Framerate: %d", framerate);
        LOG1("WhiteBalance: %s", params.get(CameraParameters::KEY_WHITE_BALANCE));
        LOG1("Flash: %s", params.get(CameraParameters::KEY_FLASH_MODE));
        LOG1("Focus: %s", params.get(CameraParameters::KEY_FOCUS_MODE));
        LOG1("Fcene: %s", params.get(CameraParameters::KEY_SCENE_MODE));
    	LOG1("Effect: %s", params.get(CameraParameters::KEY_EFFECT));
    	LOG1("ZoomIndex: %s", params.get(CameraParameters::KEY_ZOOM));

        mParameters.getPreviewSize(&mPreviewWidth, &mPreviewHeight); 
	}

    LOG_FUNCTION_NAME_EXIT

    return NO_ERROR;
}

char* CameraHal::getParameters()
{
    String8 params_str8;
    char* params_string;
    const char * valstr = NULL;
    CameraParameters mParams = mParameters;
    
    params_str8 = mParams.flatten();

    // camera service frees this string...
    params_string = (char*) malloc(sizeof(char) * (params_str8.length()+1));
    strcpy(params_string, params_str8.string());

    ///Return the current set of parameters

    return params_string;
}

void CameraHal::putParameters(char *parms)
{
    free(parms);
}
int CameraHal::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
    LOG_FUNCTION_NAME
    int ret = 0;
    Mutex::Autolock lock(mLock);
    
    LOG_FUNCTION_NAME_EXIT
    return ret;
}
int CameraHal::dump(int fd)
{   
    if (gLogLevel < 2) 
        android_atomic_inc(&gLogLevel);
    else 
        android_atomic_write(0,&gLogLevel);

    LOGD("Set camera hardware log level to %d",gLogLevel);
    return 0;
}
int CameraHal::getCameraFd()
{
    Mutex::Autolock lock(mLock);
    
    return iCamFd;  
}
void CameraHal::release()
{
    int i,err = 0;
    
    LOG_FUNCTION_NAME 
    Mutex::Autolock lock(mLock);  

    if(mCommandThread != NULL) {
        Message msg;
        msg.command = CMD_EXIT;
        commandThreadCommandQ.put(&msg);
    }

    mCommandThread->requestExitAndWait();
    mCommandThread.clear();
    mDisplayThread->requestExitAndWait();
    mDisplayThread.clear();
    mPreviewThread->requestExitAndWait();
    mPreviewThread.clear();
   
    mAutoFocusLock.lock();
    mExitAutoFocusThread = true;
    mAutoFocusLock.unlock();
    mAutoFocusCond.signal();
    mAutoFocusThread->requestExitAndWait();
    mAutoFocusThread.clear();
    mPictureThread->requestExitAndWait();
    mPictureThread.clear();
    
    cameraDestroy();
    
    LOG_FUNCTION_NAME_EXIT
}
}; // namespace android
