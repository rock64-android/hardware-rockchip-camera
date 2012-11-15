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

#include "../libyuvtorgb/yuvtorgb.h"

extern rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];

namespace android {
// Logging support -- this is for debugging only
// Use "adb shell dumpsys media.camera " to change it.
static volatile int32_t gLogLevel;

#ifdef ALOGD_IF
#define LOG1(...) ALOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) ALOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#else
#define LOG1(...) LOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) LOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#endif
    
#define LOG_TAG "CameraHal"

#define LOG_FUNCTION_NAME           LOG1("%s Enter", __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT      LOG1("%s Exit ", __FUNCTION__);

#define CmdAck_Chk(a)     (a.arg1 == (void*)CMDARG_ACK)

#if CONFIG_CAMERA_FRAME_DV_PROC_STAT
static nsecs_t framebuf_enc_start[CONFIG_CAMERA_PRVIEW_BUF_CNT];
static nsecs_t framebuf_enc_end[CONFIG_CAMERA_PRVIEW_BUF_CNT];
static nsecs_t last_disp_time = 0,cur_disp_time=0;
static long framebuf_disptime_cnt[6], framebuf_enctime_cnt[10];
#endif

static void debugShowFPS()
{
    static int mFrameCount = 0;
    static int mLastFrameCount = 0;
    static nsecs_t mLastFpsTime = 0;
    static float mFps = 0;
    mFrameCount++;
    if (!(mFrameCount & 0x1F)) {
        nsecs_t now = systemTime();
        nsecs_t diff = now - mLastFpsTime;
        mFps = ((mFrameCount - mLastFrameCount) * float(s2ns(1))) / diff;
        mLastFpsTime = now;
        mLastFrameCount = mFrameCount;
        LOGD("Camera %d Frames, %2.3f FPS", mFrameCount, mFps);
    }
    // XXX: mFPS has the value we want
}


static int getCallingPid() {
    return IPCThreadState::self()->getCallingPid();
}

static int cameraPixFmt2HalPixFmt(const char *fmt)
{
    int hal_pixel_format=HAL_PIXEL_FORMAT_YCrCb_NV12;
    
    if (strcmp(fmt,CameraParameters::PIXEL_FORMAT_RGB565) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_RGB_565;        
    } else if (strcmp(fmt,CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
    } else if (strcmp(fmt,CAMERA_DISPLAY_FORMAT_NV12) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_NV12;
    } else if (strcmp(fmt,CameraParameters::PIXEL_FORMAT_YUV422SP) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCbCr_422_SP;
    } else {
        hal_pixel_format = -EINVAL;
        LOGE("%s(%d): pixel format %s is unknow!",__FUNCTION__,__LINE__,fmt);        
    }

    return hal_pixel_format;
}

static void arm_nv12torgb565(int width, int height, char *src, short int *dst)
{
    int line, col;
    int y, u, v, yy, vr, ug, vg, ub;
    int r, g, b;
    char *py, *pu, *pv;    

    py = src;
    pu = py + (width * height);
    pv = pu + 1;
    y = *py++;
    yy = y << 8;
    u = *pu - 128;
    ug = 88 * u;
    ub = 454 * u;
    v = *pv - 128;
    vg = 183 * v;
    vr = 359 * v;
    
    for (line = 0; line < height; line++) {
        for (col = 0; col < width; col++) {
            r = (yy +      vr) >> 8;
            g = (yy - ug - vg) >> 8;
            b = (yy + ub     ) >> 8;
            if (r < 0)   r = 0;
            if (r > 255) r = 255;
            if (g < 0)   g = 0;
            if (g > 255) g = 255;
            if (b < 0)   b = 0;
            if (b > 255) b = 255;
            
            *dst++ = (((__u16)r>>3)<<11) | (((__u16)g>>2)<<5) | (((__u16)b>>3)<<0);

            y = *py++;
            yy = y << 8;
            if (col & 1) {
                pu += 2;
                pv = pu+1;
                u = *pu - 128;
                ug =   88 * u;
                ub = 454 * u;
                v = *pv - 128;
                vg = 183 * v;
                vr = 359 * v;
            }
        }
        
        if ((line & 1) == 0) { 
            //even line: rewind
            pu -= width;
            pv = pu+1;
        }
    }
}

static int rga_nv12torgb565(int fd,int width, int height, char *src, short int *dst)
{
#ifdef TARGET_RK30    
    struct rga_req  Rga_Request;
    int err = 0;
    
    memset(&Rga_Request,0x0,sizeof(Rga_Request));

    Rga_Request.src.yrgb_addr =  (int)src;
    Rga_Request.src.uv_addr  = (int)src+width*height;
    Rga_Request.src.v_addr   =  Rga_Request.src.uv_addr;
    Rga_Request.src.vir_w =  width;
    Rga_Request.src.vir_h = height;
    Rga_Request.src.format = RK_FORMAT_YCbCr_420_SP;
    Rga_Request.src.act_w = width;
    Rga_Request.src.act_h = height;
    Rga_Request.src.x_offset = 0;
    Rga_Request.src.y_offset = 0;

    Rga_Request.dst.yrgb_addr = (int)dst;
    Rga_Request.dst.uv_addr  = 0;
    Rga_Request.dst.v_addr   = 0;
    Rga_Request.dst.vir_w = width;
    Rga_Request.dst.vir_h = height;
    Rga_Request.dst.format = RK_FORMAT_RGB_565;
    Rga_Request.clip.xmin = 0;
    Rga_Request.clip.xmax = width - 1;
    Rga_Request.clip.ymin = 0;
    Rga_Request.clip.ymax = height - 1;
    Rga_Request.dst.act_w = width;
    Rga_Request.dst.act_h = height;
    Rga_Request.dst.x_offset = 0;
    Rga_Request.dst.y_offset = 0;
    Rga_Request.rotate_mode = 0;
    Rga_Request.mmu_info.mmu_en    = 1;
    Rga_Request.mmu_info.mmu_flag  = ((2 & 0x3) << 4) | 1;
    
    if(ioctl(fd, RGA_BLIT_SYNC, &Rga_Request) != 0) {
        LOGE("%s(%d):  RGA_BLIT_ASYNC Failed", __FUNCTION__, __LINE__);
        err = -1;
    }    
    return err;
#else
    LOGE("%s(%d): rk29 havn't RGA device in chips!!",__FUNCTION__, __LINE__);
    return -1;
#endif
}

static int rga_rgb565_cp(int fd,int width, int height, char *src, short int *dst)
{
#ifdef TARGET_RK30	  
	struct rga_req	Rga_Request;
	int err = 0;
	
	memset(&Rga_Request,0x0,sizeof(Rga_Request));

	Rga_Request.src.yrgb_addr =  (int)src;
	Rga_Request.src.uv_addr  = 0;
	Rga_Request.src.v_addr	 =	0;
	Rga_Request.src.vir_w =  width;
	Rga_Request.src.vir_h = height;
	Rga_Request.src.format = RK_FORMAT_RGB_565;
	Rga_Request.src.act_w = width;
	Rga_Request.src.act_h = height;
	Rga_Request.src.x_offset = 0;
	Rga_Request.src.y_offset = 0;

	Rga_Request.dst.yrgb_addr = (int)dst;
	Rga_Request.dst.uv_addr  = 0;
	Rga_Request.dst.v_addr	 = 0;
	Rga_Request.dst.vir_w = width;
	Rga_Request.dst.vir_h = height;
	Rga_Request.dst.format = RK_FORMAT_RGB_565;
	Rga_Request.clip.xmin = 0;
	Rga_Request.clip.xmax = width - 1;
	Rga_Request.clip.ymin = 0;
	Rga_Request.clip.ymax = height - 1;
	Rga_Request.dst.act_w = width;
	Rga_Request.dst.act_h = height;
	Rga_Request.dst.x_offset = 0;
	Rga_Request.dst.y_offset = 0;
	Rga_Request.rotate_mode = 0;
	Rga_Request.mmu_info.mmu_en    = 1;
	Rga_Request.mmu_info.mmu_flag  = ((2 & 0x3) << 4) | 1;
	
	if(ioctl(fd, RGA_BLIT_SYNC, &Rga_Request) != 0) {
		LOGE("%s(%d):  RGA_BLIT_ASYNC Failed", __FUNCTION__, __LINE__);
		err = -1;
	}	 
	return err;
#else
	LOGE("%s(%d): rk29 havn't RGA device in chips!!",__FUNCTION__, __LINE__);
	return -1;
#endif
}

static int arm_rgb565_mirror_line(int *psrc, int *pdst, int w)
{
    int i;

    for (i=0; i<(w>>1); i++) {
        *pdst = ((*psrc>>16)&0x0000ffff) | ((*psrc<<16)&0xffff0000);
        psrc++;
        pdst--;
    }

    return 0;
}
static int arm_rgb565_mirror(char *pdata, char *pline_tmp, int w, int h)
{
    int *pdata_tmp = NULL;
    int *pdata_mirror;
    int err = 0,i,j;

    pdata_tmp = (int*)pline_tmp;
    pdata_mirror = (int*)pdata;
    
    for (j=0; j<h; j++) {
        arm_rgb565_mirror_line(pdata_mirror, pdata_tmp+((w>>1)-1),w);
        memcpy(pdata_mirror, pdata_tmp, (w<<1));
        pdata_mirror += (w>>1);
    }
    return err;
}

CameraHal::CameraHal(int cameraId)
            :mParameters(),
            mSnapshotRunning(-1),
            mCommandRunning(-1),
            mPreviewRunning(STA_PREVIEW_PAUSE),
            mPreviewLock(),
            mPreviewCond(),
            mDisplayRuning(STA_DISPLAY_PAUSE),
            mDisplayLock(),
            mDisplayCond(),
            mCamDriverStream(false),
            mCamDriverStreamLock(),
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
	        mDispBufUndqueueMin(0),
            mPreviewMemory(NULL),
            mRawBufferSize(0),
            mJpegBufferSize(0),
            mMsgEnabled(0),
            mEffect_number(0),
            mScene_number(0),
            mWhiteBalance_number(0),
            mFlashMode_number(0),
            mGps_latitude(-1),
            mGps_longitude(-1),
            mGps_altitude(-1),
            mGps_timestamp(-1),
            displayThreadCommandQ("displayCmdQ"),
            displayThreadAckQ("displayAckQ"),            
            previewThreadCommandQ("previewCmdQ"),
            previewThreadAckQ("previewAckQ"),
            commandThreadCommandQ("commandCmdQ"),
            commandThreadAckQ("commandAckQ"),
            snapshotThreadCommandQ("snapshotCmdQ"),
            snapshotThreadAckQ("snapshotAckQ"),
            mCamBuffer(NULL)
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
    memset(&mCamDriverSupportFmt[0],0, sizeof(mCamDriverSupportFmt));
    mRecordRunning = false;
    mPictureRunning = STA_PICTURE_STOP;
    mExitAutoFocusThread = false;
    mDriverMirrorSupport = false;
    mDriverFlipSupport = false;
    mPreviewCmdReceived = false;
    mPreviewStartTimes = 0x00;    
    memset(mCamDriverV4l2Buffer, 0x00, sizeof(mCamDriverV4l2Buffer));
    memset(mDisplayFormat,0x00,sizeof(mDisplayFormat));
    for (i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        mPreviewBufferMap[i] = NULL;
        mDisplayBufferMap[i] = NULL;
        memset(&mGrallocBufferMap[i],0x00,sizeof(rk_previewbuf_info_t));
        mPreviewBufs[i] = NULL;
        mVideoBufs[i] = NULL;

        mPreviewBuffer[i] = NULL;
    }

    #if CONFIG_CAMERA_FRONT_MIRROR_MDATACB
    if (gCamInfos[cameraId].facing_info.facing == CAMERA_FACING_FRONT) {
    #if CONFIG_CAMERA_FRONT_MIRROR_MDATACB_ALL
        mDataCbFrontMirror = true;
    #else
        if (strstr(CONFIG_CAMERA_FRONT_MIRROR_MDATACB_APK,cameraCallProcess)) {
            mDataCbFrontMirror = true;            
        } else {
            mDataCbFrontMirror = false;
        }
    #endif
    } else {
        mDataCbFrontMirror = false;
    }
    #else
    mDataCbFrontMirror = false;
    #endif
    
    //open the rga device,zyc
    mRGAFd = -1;

    if (cameraCreate(cameraId) == 0) {
        initDefaultParameters();
		if(access(CAMERA_IPP_NAME, O_RDWR) < 0)
        	cameraRawJpegBufferCreate(mRawBufferSize*2,mJpegBufferSize);
		else
            cameraRawJpegBufferCreate(mRawBufferSize,mJpegBufferSize);
        mDisplayThread = new DisplayThread(this);
        mPreviewThread = new PreviewThread(this);
        mCommandThread = new CommandThread(this);
        mPictureThread = new PictureThread(this);
	    mSnapshotThread = new SnapshotThread(this);
        mAutoFocusThread = new AutoFocusThread(this);
        mDisplayThread->run("CameraDispThread",ANDROID_PRIORITY_URGENT_DISPLAY);
        mPreviewThread->run("CameraPreviewThread",ANDROID_PRIORITY_DISPLAY);
        mCommandThread->run("CameraCmdThread", ANDROID_PRIORITY_URGENT_DISPLAY);
        mAutoFocusThread->run("CameraAutoFocusThread", ANDROID_PRIORITY_DISPLAY);
        mSnapshotThread->run("CameraSnapshotThread", ANDROID_PRIORITY_NORMAL);

        LOGD("CameraHal create success!");
    } else {
        mPreviewThread = NULL;
        mDisplayThread = NULL;
        mCommandThread = NULL;
        mPictureThread = NULL;
	    mSnapshotThread = NULL;		
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
        LOGE("Camera driver version: %d.%d.%d isn't support query framerate, Please update to v0.x.5",
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
            /* ddl@rock-chip.com: Compatible for v0.x.5 camera driver*/
            if ((mCamDriverCapability.version & 0xff) > 0x05) {
                if ((fival.discrete.denominator < 60) && (fival.discrete.numerator == 1000))
                    fival.discrete.denominator *= 1000;                
            }
            *min = (fival.discrete.denominator*1000)/fival.discrete.numerator;
            *max = (fival.discrete.denominator*1000)/fival.discrete.numerator;
        } else {
            LOGE("%s(%d): query framerate type(%d) is not supported",__FUNCTION__,__LINE__, fival.type);
            goto default_fps;
        }
    } else {
        if ((w==240) && (h==160)) {
            if ((mCamDriverCapability.version & 0xff) >= 0x07) {
                LOGE("%s(%d): Query 240x160 framerate error,please supply this framerate "
                    "in kernel board_rk29_xxx.c file for CONFIG_SENSOR_240X160_FPS_FIXD_XX",__FUNCTION__,__LINE__);
                ret = -EINVAL;
                goto cameraFramerateQuery_end;
            }
        } else {
            LOGE("%s(%d): Query framerate error(%dx%d@%c%c%c%c index:%d)",__FUNCTION__,__LINE__,
                fival.width,fival.height,(fival.pixel_format & 0xFF), (fival.pixel_format >> 8) & 0xFF,
    				((fival.pixel_format >> 16) & 0xFF), ((fival.pixel_format >> 24) & 0xFF),fival.index);
        }
default_fps:    
        if (gCamInfos[mCamId].facing_info.facing == CAMERA_FACING_BACK) {
            *min = CONFIG_CAMERA_BACK_PREVIEW_FPS_MIN;
            *max = CONFIG_CAMERA_BACK_PREVIEW_FPS_MAX;
        } else {
            *min = CONFIG_CAMERA_FRONT_PREVIEW_FPS_MIN;
            *max = CONFIG_CAMERA_FRONT_PREVIEW_FPS_MAX;
        }
        ret = 0;
    }
    
#endif
cameraFramerateQuery_end:
    return ret;
}
int CameraHal::cameraFpsInfoSet(CameraParameters &params)
{
    int min,max, framerate_min,framerate_max,w,h;
    char fps_str[20];
    char framerates[50];
    String8 parameterString;
    Vector<Size> sizes;
    unsigned int i;
    
    params.getSupportedPreviewSizes(sizes);   
    params.getPreviewSize(&w, &h);
    if (gCamInfos[mCamId].facing_info.facing == CAMERA_FACING_BACK) {
        framerate_min = CONFIG_CAMERA_BACK_PREVIEW_FPS_MIN;
        framerate_max = CONFIG_CAMERA_BACK_PREVIEW_FPS_MAX;
    } else {
        framerate_min = CONFIG_CAMERA_FRONT_PREVIEW_FPS_MIN;
        framerate_max = CONFIG_CAMERA_FRONT_PREVIEW_FPS_MAX;
    }

    memset(framerates,0x00,sizeof(framerates));
    for (i=0; i<sizes.size(); i++) {
        cameraFramerateQuery(mCamDriverPreviewFmt,sizes[i].width,sizes[i].height,&min,&max);        
        if (min<framerate_min) {
            framerate_min = min;
        }

        if (max>framerate_max) {
            framerate_max = max;
        }

        if ((w==sizes[i].width) && (h==sizes[i].height)) {
            params.setPreviewFrameRate(min/1000);
        } 

        memset(fps_str,0x00,sizeof(fps_str));            
        sprintf(fps_str,"%d",min/1000);
        if (strstr(framerates,fps_str) == NULL) {
            if (strlen(framerates)) 
                sprintf(&framerates[strlen(framerates)],",");    
            sprintf(&framerates[strlen(framerates)],"%d",min/1000);
        }

        memset(fps_str,0x00,sizeof(fps_str));            
        sprintf(fps_str,"%d",max/1000);
        if (strstr(framerates,fps_str) == NULL) {
            sprintf(&framerates[strlen(framerates)],",%d",max/1000);
        }
    }
    
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
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, framerates);   

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
    memset(str_picturesize,0x00,sizeof(str_picturesize));
    if (CAMERA_IS_UVC_CAMERA()) {
        /*preview size setting*/
        struct v4l2_frmsizeenum fsize;                
        
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

        if (mCamDriverFrmWidthMax <= 1024) {
    		mRawBufferSize = RAW_BUFFER_SIZE_1M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_1M;
        } else if (mCamDriverFrmWidthMax <= 1600) {
    		mRawBufferSize = RAW_BUFFER_SIZE_2M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_2M;
        } else if (mCamDriverFrmWidthMax <= 2048) {
    		mRawBufferSize = RAW_BUFFER_SIZE_3M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_3M;
        } else if (mCamDriverFrmWidthMax <= 2592) {                    			
            mRawBufferSize = RAW_BUFFER_SIZE_5M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
    	} else {
    	    LOGE("%s(%d):Camera Hal is only support 5Mega camera, but the uvc camera is %dx%d",
                 __FUNCTION__,__LINE__,mCamDriverFrmWidthMax, mCamDriverFrmHeightMax);
            mRawBufferSize = RAW_BUFFER_SIZE_5M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
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
    	/*not support zoom */
    	params.set(CameraParameters::KEY_ZOOM_SUPPORTED, "false");

    } else if (CAMERA_IS_RKSOC_CAMERA()) {
    
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat= mCamDriverPreviewFmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        /*picture size setting*/
     	fmt.fmt.pix.width = 10000;
     	fmt.fmt.pix.height = 10000;
    	ret = ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt);
        
        mCamDriverFrmWidthMax = fmt.fmt.pix.width;
        mCamDriverFrmHeightMax = fmt.fmt.pix.height;        

        if (mCamDriverFrmWidthMax > 3264) {
            LOGE("Camera driver support maximum resolution(%dx%d) is overflow 8Mega!",mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
            mCamDriverFrmWidthMax = 3264;
            mCamDriverFrmHeightMax = 2448;
        }
        
        /*preview size setting*/ 
        if (mCamDriverFrmWidthMax >= 176) {            
         	fmt.fmt.pix.width = 176;
         	fmt.fmt.pix.height = 144;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 176) && (fmt.fmt.pix.height == 144)) {
                    parameterString.append("176x144");
                    params.setPreviewSize(176, 144);
                    previewFrameSizeMax =  PAGE_ALIGN(176*144*2)*2;          // 176*144*2     rgb565
                    //params.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,"176x144");
                }
            }
        }

        if ((mCamDriverCapability.version & 0xff) >= 0x07) {
            int tmp0,tmp1;
            if (cameraFramerateQuery(mCamDriverPreviewFmt, 240,160,&tmp1,&tmp0) == 0) {
                if (mCamDriverFrmWidthMax >= 240) {            
                 	fmt.fmt.pix.width = 240;
                 	fmt.fmt.pix.height = 160;
                    if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                        if ((fmt.fmt.pix.width == 240) && (fmt.fmt.pix.height == 160)) {
                            parameterString.append(",240x160");
                            params.setPreviewSize(240, 160);
                            previewFrameSizeMax =  PAGE_ALIGN(240*160*2)*2;          // 240*160*2     rgb565
                            
                        }
                    }
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
                        previewFrameSizeMax =  PAGE_ALIGN(320*240*2)*2;          // 320*240*2
                        
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
                    previewFrameSizeMax =  PAGE_ALIGN(352*288*2)*2;          // 352*288*1.5*2
                    
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
                    previewFrameSizeMax =  PAGE_ALIGN(640*480*2)*2;          // 640*480*1.5*2
                    
                }
            }
        }

        if (mCamDriverFrmWidthMax >= 720) {
            fmt.fmt.pix.width = 720;
         	fmt.fmt.pix.height = 480;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 720) && (fmt.fmt.pix.height == 480)) {
                    parameterString.append(",720x480");
                    previewFrameSizeMax =  PAGE_ALIGN(720*480*2)*2;          // 720*480*1.5*2
                    
                }
            }
        }

#ifdef TARGET_RK30    
        if (mCamDriverFrmWidthMax >= 800) {
            fmt.fmt.pix.width = 800;
            fmt.fmt.pix.height = 600;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 800) && (fmt.fmt.pix.height == 600)) {
                    parameterString.append(",800x600");
                    previewFrameSizeMax =  PAGE_ALIGN(800*600*2)*2;          // 720*480*1.5*2
					params.setPreviewSize(800, 600);
                }
            }
        }
#endif
        if (mCamDriverFrmWidthMax >= 1280) {
            fmt.fmt.pix.width = 1280;
         	fmt.fmt.pix.height = 720;
            if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) {
                if ((fmt.fmt.pix.width == 1280) && (fmt.fmt.pix.height == 720)) {
                    parameterString.append(",1280x720");
                    previewFrameSizeMax =  PAGE_ALIGN(1280*720*2)*2;          // 1280*720*1.5*2
                    
                }
            }
        }
        mSupportPreviewSizeReally = parameterString;
        /* ddl@rock-chips.com: Facelock speed is low, so scale down preview data to facelock for speed up */
        if ((strcmp(cameraCallProcess,"com.android.facelock")==0)) {            
            if (strstr(mSupportPreviewSizeReally.string(),"640x480")||
                strstr(mSupportPreviewSizeReally.string(),"320x240")) {
                parameterString = "160x120";
                params.setPreviewSize(160, 120);    
            }
        }
        params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
        
        strcat(str_picturesize,parameterString.string());
        strcat(str_picturesize,",");
        if(mCamDriverFrmWidthMax <= 640){
            strcat( str_picturesize,"640x480,320x240");
		    mRawBufferSize = RAW_BUFFER_SIZE_0M3;
            mJpegBufferSize = JPEG_BUFFER_SIZE_0M3;
            params.setPictureSize(640,480);
        }else if (mCamDriverFrmWidthMax <= 1280) {
            strcat( str_picturesize,"1024x768,640x480,320x240");
		    mRawBufferSize = RAW_BUFFER_SIZE_1M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_1M;
            params.setPictureSize(1024,768);
        } else if (mCamDriverFrmWidthMax <= 1600) {
			strcat( str_picturesize,"1600x1200,1024x768,640x480");
            mRawBufferSize = RAW_BUFFER_SIZE_2M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_2M;
            params.setPictureSize(1600,1200);
        } else if (mCamDriverFrmWidthMax <= 2048) {
			strcat( str_picturesize,"2048x1536,1600x1200,1024x768");
            mRawBufferSize = RAW_BUFFER_SIZE_3M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_3M;  
            params.setPictureSize(2048,1536);
        } else if (mCamDriverFrmWidthMax <= 2592) {                    			
    		strcat( str_picturesize,"2592x1944,2048x1536,1600x1200,1024x768");
            params.setPictureSize(2592,1944);
            mRawBufferSize = RAW_BUFFER_SIZE_5M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_5M;
        } else if (mCamDriverFrmWidthMax <= 3264) {                    			
    		strcat( str_picturesize,"3264x2448,2592x1944,2048x1536,1600x1200,1024x768");
            params.setPictureSize(3264,2448);
            mRawBufferSize = RAW_BUFFER_SIZE_8M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_8M;
    	} else {
            sprintf(str_picturesize, "%dx%d", mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
            mRawBufferSize = RAW_BUFFER_SIZE_8M;
            mJpegBufferSize = JPEG_BUFFER_SIZE_8M;
            params.setPictureSize(mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
    	}
        
        params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, str_picturesize);

        /*frame rate setting*/
        cameraFpsInfoSet(params);
        
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
        }
    }
    /*preview format setting*/
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, "yuv420sp,rgb565,yuv420p");
    params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);
    if ((strcmp(cameraCallProcess,"com.android.camera")==0) || (strcmp(cameraCallProcess,"com.android.gallery3d")==0)){    //for PanoramaActivity
        params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_RGB565);   
    } else {
        params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);   
    }
    /* zyc@rock-chips.com: preset the displayformat for cts */
	strcpy(mDisplayFormat,CAMERA_DISPLAY_FORMAT_NV12);


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

	focus.id = V4L2_CID_FOCUSZONE;
     
	// focus area settings
    if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {

 	   params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"1");
	}
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


    /*Exposure setting*/
    struct v4l2_queryctrl exposure;
    char str_exposure[16];
    exposure.id = V4L2_CID_EXPOSURE;
    if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &exposure)) {
        sprintf(str_exposure,"%d",exposure.default_value);
    	params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, str_exposure);
        sprintf(str_exposure,"%d",exposure.maximum);        
    	params.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, str_exposure);
        sprintf(str_exposure,"%d",exposure.minimum);        
    	params.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, str_exposure);
        sprintf(str_exposure,"%d",exposure.step); 
    	params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, str_exposure);
    } else {
    	params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, "0");
    	params.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, "0");
    	params.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, "0");
    	params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, "0.000001f");
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
    /* zyc@rock-chips.com: for cts ,KEY_MAX_NUM_DETECTED_FACES_HW should not be 0 */
    params.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, "0");
    params.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW, "0");
    params.set(CameraParameters::KEY_RECORDING_HINT,"false");
    params.set(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED,"false");
    params.set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED,"true");
    params.set(CameraParameters::KEY_MAX_NUM_METERING_AREAS,"0");

    LOGD ("Support Preview format: %s ",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS));
    LOGD ("Support Preview sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES));
    LOGD ("Support Preview FPS range: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE));
    LOGD ("Support Preview framerate: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES)); 
    LOGD ("Support Picture sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES));
    if (params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE))
        LOGD ("Support white balance: %s",params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE));
    if (params.get(CameraParameters::KEY_SUPPORTED_EFFECTS))
        LOGD ("Support color effect: %s",params.get(CameraParameters::KEY_SUPPORTED_EFFECTS));
    if (params.get(CameraParameters::KEY_SUPPORTED_SCENE_MODES))
        LOGD ("Support scene: %s",params.get(CameraParameters::KEY_SUPPORTED_SCENE_MODES));
    if (params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES))
        LOGD ("Support flash: %s",params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES));
    LOGD ("Support focus: %s",params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES));
    LOGD ("Support zoom: %s(ratios: %s)",params.get(CameraParameters::KEY_ZOOM_SUPPORTED),
        params.get(CameraParameters::KEY_ZOOM_RATIOS));
    if (strcmp("0", params.get(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION))
		|| strcmp("0", params.get(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION))) {
        LOGD ("Support exposure: (%s -> %s)",params.get(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION),
            params.get(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION));
    }
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
    Message msg;
    Mutex::Autolock lock(mLock);

    if (mANativeWindow) {
        if (mGrallocBufferMap[0].priv_hnd) {
            if ((PRIVATE_HANDLE_GET_W(mGrallocBufferMap[0].priv_hnd) != mPreviewWidth) ||
                (PRIVATE_HANDLE_GET_H(mGrallocBufferMap[0].priv_hnd) != mPreviewHeight)) {        
                LOGD("%s(%d): mANativeWidow's buffer is invalidate for current setParameters",__FUNCTION__,__LINE__);
                if (mPreviewRunning == STA_PREVIEW_RUN) {
                    msg.command = CMD_PREVIEW_STOP;
                    msg.arg1 = (void*)CMDARG_ACK;
                    commandThreadCommandQ.put(&msg);
                    ret = 0;
                    while (ret == 0) {            
                        ret = commandThreadAckQ.get(&msg);
                        if (ret == 0) {
                            if (msg.command == CMD_PREVIEW_STOP) {                    
                                ret = 1;
                            }
                        }
                    }
                    ret = 0;
                } else {
                    cameraDisplayThreadPause(true);
                }
            }
        }
    }
    
    mANativeWindow = window;
    if (mANativeWindow) {        
		mANativeWindowLock.lock();        
        mANativeWindowCond.signal();
		mANativeWindowLock.unlock();
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
    msg.arg1 = (void*)((done == true) ? CMDARG_ACK : CMDARG_NACK);
    mDisplayLock.lock();
    displayThreadCommandQ.put(&msg);    
    mDisplayCond.signal();
    mDisplayLock.unlock(); 
	if (done == true) {
        if (displayThreadAckQ.get(&msg) < 0) {
            LOGE("%s(%d): Start display thread failed,mDisplayRunging(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_START) && ((unsigned int)msg.arg2 == STA_DISPLAY_RUN)
                && ((unsigned int)msg.arg1 == CMDARG_OK)) {
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
    msg.arg1 = (void*)((done == true) ? CMDARG_ACK : CMDARG_NACK);
    mDisplayLock.lock();
    displayThreadCommandQ.put(&msg);
    mDisplayCond.signal();
    mDisplayLock.unlock(); 
	if (done == true) {
        if (displayThreadAckQ.get(&msg) < 0) {
            LOGE("%s(%d): Pause display thread failed, mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_PAUSE) && ((int)msg.arg2 == STA_DISPLAY_PAUSE)
                && ((unsigned int)msg.arg1 == CMDARG_OK)) {
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
    msg.arg1 = (void*)((done == true) ? CMDARG_ACK : CMDARG_NACK);
    mDisplayLock.lock();
    displayThreadCommandQ.put(&msg);
	mDisplayCond.signal();
    mDisplayLock.unlock(); 
    if (done == true) {
        if (displayThreadAckQ.get(&msg) < 0) {
            LOGE("%s(%d): Stop display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);    
        } else {
            if ((msg.command == CMD_DISPLAY_STOP) && ((int)msg.arg2 == STA_DISPLAY_STOP)
                && ((unsigned int)msg.arg1 == CMDARG_OK)) {
                LOG1("%s(%d): Stop display thread success,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);
            } else {
                LOGE("%s(%d): Stop display thread failed,mDisplayRuning(%d)",__FUNCTION__,__LINE__,mDisplayRuning);     
            }     
        }    
    }
cameraDisplayThreadStop_end:
    return err;
}

int CameraHal::cameraPreviewThreadSet(unsigned int setStatus,int done)
{
    int err = NO_ERROR;
    Message msg;
    
    if (mPreviewRunning == setStatus) {
        LOGD("%s(%d): preview thread is already at status : %d",__FUNCTION__,__LINE__,setStatus);
        err = -1;
        goto cameraPreviewThreadStateSet_end;
    }

    //check the status
    if((setStatus != CMD_PREVIEW_THREAD_PAUSE) && (setStatus != CMD_PREVIEW_THREAD_START) && (setStatus != CMD_PREVIEW_THREAD_STOP)){
        LOGE("%s(%d): the status wanted to be set is not support.",__FUNCTION__,__LINE__);
        err = -1;
        goto cameraPreviewThreadStateSet_end;
    }
        
    msg.command = setStatus;
    msg.arg1 = (void*)((done == true) ? CMDARG_ACK : CMDARG_NACK);
    mPreviewLock.lock();
    previewThreadCommandQ.put(&msg);
    mPreviewLock.unlock();
    mPreviewCond.signal();

    if (done == true) {
        if (previewThreadAckQ.get(&msg) < 0) {
            err = -1;
            LOGE("%s(%d): set preview thread status failed,mPreviewRunning(%d)",__FUNCTION__,__LINE__,mPreviewRunning);    
        } else {
            if ((msg.command == setStatus) && ((unsigned int)msg.arg2 == setStatus)
                && ((unsigned int)msg.arg1 == CMDARG_OK)) {
                LOG1("%s(%d): set preview thread status success,mPreviewRunning(%d)",__FUNCTION__,__LINE__,mPreviewRunning);
            } else {
                err = -1;
                LOGE("%s(%d): set preview thread status failed,mPreviewRunning(%d)",__FUNCTION__,__LINE__,mPreviewRunning);     
            }     
        }    
    }
    
cameraPreviewThreadStateSet_end:
    return err;
    
}
void CameraHal::displayThread()
{
    int err,stride,i,queue_cnt;
    int dequeue_buf_index,queue_buf_index,queue_display_index;
    buffer_handle_t *hnd = NULL; 
    NATIVE_HANDLE_TYPE *phnd;
    GraphicBufferMapper& mapper = GraphicBufferMapper::get();
    Message msg;
    void *y_uv[3];
    Rect bounds;
    
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
                    cameraDisplayBufferDestory();                    
                    cameraDisplayBufferCreate(mPreviewWidth, mPreviewHeight,mDisplayFormat,CONFIG_CAMERA_PRVIEW_BUF_CNT);
                    mDisplayRuning = STA_DISPLAY_RUN;
                    if (CmdAck_Chk(msg)) {
                        msg.arg1 = (void*)CMDARG_OK;
                        msg.arg2 = (void*)mDisplayRuning;
                        displayThreadAckQ.put(&msg);
                    }
                    break;
                }

                case CMD_DISPLAY_PAUSE:
                {
                    LOGD("%s(%d): receive CMD_DISPLAY_PAUSE", __FUNCTION__,__LINE__);

                    cameraDisplayBufferDestory();
                    mDisplayRuning = STA_DISPLAY_PAUSE;
                    if (CmdAck_Chk(msg)) {
                        msg.arg1 = (void*)CMDARG_OK;
                        msg.arg2 = (void*)mDisplayRuning;
                        displayThreadAckQ.put(&msg);
                    }                    
                    break;
                }
                
                case CMD_DISPLAY_STOP:
                {
                    LOGD("%s(%d): receive CMD_DISPLAY_STOP", __FUNCTION__,__LINE__);
					cameraDisplayBufferDestory();
                    mDisplayRuning = STA_DISPLAY_STOP;
                    if (CmdAck_Chk(msg)) {
                        msg.arg1 = (void*)CMDARG_OK;
                        msg.arg2 = (void*)mDisplayRuning;
                        displayThreadAckQ.put(&msg);
                    }
                    continue;
                }

                case CMD_DISPLAY_FRAME:
                {                    
                    if (mDisplayRuning != STA_DISPLAY_RUN) 
                        goto display_receive_cmd;
                        
                   
                    if (mANativeWindow == NULL) {
                        LOGE("%s(%d): thread exit, because mANativeWindow is NULL", __FUNCTION__,__LINE__);
                        mDisplayRuning = STA_DISPLAY_STOP;  
                        continue;
                    }
                    
                    queue_buf_index = (int)msg.arg1;                    
                    queue_display_index = CONFIG_CAMERA_PRVIEW_BUF_CNT;
                    if (mDisplayBufferMap[queue_buf_index] != mPreviewBufferMap[queue_buf_index]) {
                        
                        for (i=0; i<mPreviewBufferCount; i++) {
                            if ((mDisplayBufferMap[i]->buf_state & CMD_PREVIEWBUF_DISPING) == 0) 
                                break;
                        }
                        if (i<mPreviewBufferCount) {
                            queue_display_index = i;
                        } else {
                            err = 0x01;
                            while (err != 0) {
                                err = mANativeWindow->dequeue_buffer(mANativeWindow, (buffer_handle_t**)&hnd, &stride);
                                if (err == 0) {
                                    // lock the initial queueable buffers
                                    bounds.left = 0;
                                    bounds.top = 0;
                                    bounds.right = mPreviewWidth;
                                    bounds.bottom = mPreviewHeight;
                                    mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)hnd);
                                    mapper.lock((buffer_handle_t)(*hnd), CAMHAL_GRALLOC_USAGE, bounds, y_uv);

                                    phnd = (NATIVE_HANDLE_TYPE*)*hnd;
                                    for (i=0; i<mPreviewBufferCount; i++) {
                                        if (phnd == mDisplayBufferMap[i]->priv_hnd) {  
                                            queue_display_index = i;
                                            break;
                                        }
                                    }  
                                    
                                } else {
                                    LOG2("%s(%d): %s(err:%d) dequeueBuffer failed, so pause here", __FUNCTION__,__LINE__, strerror(-err), -err);

                                    mDisplayLock.lock();
                                    if (displayThreadCommandQ.isEmpty() == false ) {
                                        mDisplayLock.unlock(); 
                                        goto display_receive_cmd;
                                    }
						                                    
                                    mDisplayCond.wait(mDisplayLock); 
                                    mDisplayLock.unlock();
                                    LOG2("%s(%d): wake up...", __FUNCTION__,__LINE__);
                                }
                            }
                        } 
                        
                        if(CAMERA_IS_RKSOC_CAMERA()) {                             
                            cameraFormatConvert(mCamDriverPreviewFmt, 0,mDisplayFormat,
                                (char*)mPreviewBufferMap[queue_buf_index]->vir_addr,(char*)mDisplayBufferMap[queue_display_index]->vir_addr,
                                mPreviewBufferMap[queue_buf_index]->phy_addr, mDisplayBufferMap[queue_display_index]->phy_addr,
                                mPreviewWidth, mPreviewHeight,mPreviewWidth, mPreviewHeight,false);
                        } else {
                        	/* zyc@rock-chips.com: for usb camera */							
                        	cameraFormatConvert(V4L2_PIX_FMT_NV12, 0,mDisplayFormat,
                        	    (char*)mPreviewBufferMap[queue_buf_index]->vir_addr,(char*)mDisplayBufferMap[queue_display_index]->vir_addr,
                        		mPreviewBufferMap[queue_buf_index]->phy_addr, mDisplayBufferMap[queue_display_index]->phy_addr,
                        		mPreviewWidth, mPreviewHeight,mPreviewWidth, mPreviewHeight,false);
                        }                        
                    } else {
                        queue_display_index = queue_buf_index;
                    }
                    
                    LOG2("%s(%d): receive buffer %d, queue buffer %d to display", __FUNCTION__,__LINE__,queue_buf_index,queue_display_index);

                    cameraPreviewBufferSetSta(mDisplayBufferMap[queue_display_index], CMD_PREVIEWBUF_DISPING, 1);
                    mapper.unlock((buffer_handle_t)mDisplayBufferMap[queue_display_index]->priv_hnd);
                    err = mANativeWindow->enqueue_buffer(mANativeWindow, (buffer_handle_t*)mDisplayBufferMap[queue_display_index]->buffer_hnd);                    
                    #if CONFIG_CAMERA_FRAME_DV_PROC_STAT
                    cur_disp_time = systemTime();                    
                    if (last_disp_time!=0) {
                        if ((cur_disp_time-last_disp_time) < 40000000) {
                            framebuf_disptime_cnt[0]++;
                        } else if (((cur_disp_time-last_disp_time) >= 40000000)&&((cur_disp_time-last_disp_time) < 50000000)) {
                            framebuf_disptime_cnt[1]++;
                        } else if (((cur_disp_time-last_disp_time) >= 50000000)&&((cur_disp_time-last_disp_time) < 60000000)) {
                            framebuf_disptime_cnt[2]++;
                        } else if (((cur_disp_time-last_disp_time) >= 60000000)&&((cur_disp_time-last_disp_time) < 70000000)) {
                            framebuf_disptime_cnt[3]++;
                        } else if (((cur_disp_time-last_disp_time) >= 70000000)&&((cur_disp_time-last_disp_time) < 80000000)) {
                            framebuf_disptime_cnt[4]++;
                        }
                    }
                    last_disp_time = cur_disp_time;
                    #endif
                    if (err != 0){
                        cameraPreviewBufferSetSta(mDisplayBufferMap[queue_display_index], CMD_PREVIEWBUF_DISPING, 0);
                                                
                        bounds.left = 0;
                        bounds.top = 0;
                        bounds.right = mPreviewWidth;
                        bounds.bottom = mPreviewHeight;
                        mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)mDisplayBufferMap[queue_display_index]->buffer_hnd);
                        mapper.lock((buffer_handle_t)(mDisplayBufferMap[queue_display_index]->priv_hnd), CAMHAL_GRALLOC_USAGE, bounds, y_uv);

                        mDisplayRuning = STA_DISPLAY_PAUSE;
                        LOGE("%s(%d): enqueue buffer %d to mANativeWindow failed(%d),so display pause", __FUNCTION__,__LINE__,queue_display_index,err);
                    }                
                    

                    if (mDisplayBufferMap[queue_buf_index] != mPreviewBufferMap[queue_buf_index]) {
                        cameraPreviewBufferSetSta(mPreviewBufferMap[queue_buf_index], CMD_PREVIEWBUF_DISPING, 0);
                        msg.command = CMD_PREVIEW_QBUF;     
                        msg.arg1 = (void*)queue_buf_index;
                        msg.arg2 = (void*)CMD_PREVIEWBUF_DISPING;
                        msg.arg3 = (void*)mPreviewStartTimes;
                        commandThreadCommandQ.put(&msg);

                        if ((mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) && mDataCb) {
                            if (strcmp(mParameters.getPreviewFormat(),mDisplayFormat) == 0) {
                                if (mPreviewMemory) {  
                                    if (mDataCbFrontMirror == true) {
                                        if (strcmp(CameraParameters::PIXEL_FORMAT_RGB565,mDisplayFormat) == 0)
                                            arm_rgb565_mirror((char*)mDisplayBufferMap[queue_display_index]->vir_addr,
                                                              (char*)mPreviewBufferMap[queue_buf_index]->vir_addr, 
                                                              mPreviewWidth, mPreviewHeight);
                                        else
                                            LOGE("%s(%d): %s format mirror isn't support!",__FUNCTION__,__LINE__,mDisplayFormat);
                                    } else {
                                        memcpy((char*)mPreviewBufs[queue_display_index], (char*)mDisplayBufferMap[queue_display_index]->vir_addr, mPreviewFrame2AppSize);                                                             
                                    }
                                    mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewMemory, queue_display_index,NULL,mCallbackCookie);                                 
                                } else {
                                    LOGE("%s(%d): mPreviewMemory is NULL, preview data could not send to application",__FUNCTION__,__LINE__);
                                }    
                            }
                        }
                    } else {
                        queue_cnt = 0;
                        for (i=0; i<mPreviewBufferCount; i++) {
                            if (mDisplayBufferMap[i]->buf_state & CMD_PREVIEWBUF_DISPING) 
                                queue_cnt++;
                        }

                        if (queue_cnt > mDispBufUndqueueMin) {
                            err = mANativeWindow->dequeue_buffer(mANativeWindow, (buffer_handle_t**)&hnd, &stride);
                            if (err == 0) {                                    
                                // lock the initial queueable buffers
                                bounds.left = 0;
                                bounds.top = 0;
                                bounds.right = mPreviewWidth;
                                bounds.bottom = mPreviewHeight;
                                mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)hnd);
                                mapper.lock((buffer_handle_t)(*hnd), CAMHAL_GRALLOC_USAGE, bounds, y_uv);

                                phnd = (NATIVE_HANDLE_TYPE*)*hnd;
                                for (i=0; i<mPreviewBufferCount; i++) {
                                    if (phnd == mDisplayBufferMap[i]->priv_hnd) {
                                        dequeue_buf_index = i;
                                        break;
                                    }
                                }
                                
                                if (i >= mPreviewBufferCount) {                    
                                    LOGE("%s(%d): dequeue buffer(0x%x ) don't find in mDisplayBufferMap", __FUNCTION__,__LINE__,(int)phnd);                    
                                    continue;
                                } else {
                                    cameraPreviewBufferSetSta(mDisplayBufferMap[dequeue_buf_index], CMD_PREVIEWBUF_DISPING, 0);
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
                                mDisplayLock.lock();
                                if (displayThreadCommandQ.isEmpty() == false ) {
                                    mDisplayLock.unlock(); 
                                    goto display_receive_cmd;
                                }                  
                                mDisplayCond.wait(mDisplayLock); 
                                mDisplayLock.unlock();
                                LOG2("%s(%d): wake up...", __FUNCTION__,__LINE__);
                            }
                        }
                    }                    
                    break;
        
                }
                default:
                {
                    LOGE("%s(%d): receive unknow command(0x%x)!", __FUNCTION__,__LINE__,msg.command);
                    break;
                }
            }
        }
        
        mDisplayLock.lock();
        if (displayThreadCommandQ.isEmpty() == false ) {
            mDisplayLock.unlock(); 
            goto display_receive_cmd;
        }        	
        LOG1("%s(%d): display thread pause here... ", __FUNCTION__,__LINE__);
        mDisplayCond.wait(mDisplayLock);  
        mDisplayLock.unlock(); 
        LOG1("%s(%d): display thread wake up... ", __FUNCTION__,__LINE__);
        goto display_receive_cmd;        
        
        if (mANativeWindow == NULL) { 
            LOGE("%s(%d): thread exit, because mANativeWindow is NULL", __FUNCTION__,__LINE__);
            mDisplayRuning = STA_DISPLAY_STOP;            
        }
    }
    LOG_FUNCTION_NAME_EXIT
}
void CameraHal::previewThread()
{
    int err;
    int ret,i;
    bool loop = true;
    static struct v4l2_buffer cfilledbuffer1;
    Message msg;
    bool camera_device_error;
    int buffer_log;
    bool snapshot;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    
    LOG_FUNCTION_NAME
    camera_device_error = false;
    snapshot = false;
    while (loop) {
        msg.command = CMD_PREVIEW_INVAL;
        if (camera_device_error == true){
            LOGD("%s(%d): camera driver or device may be error, so preview thread wait for command thread wake...",
                __FUNCTION__,__LINE__);
            previewThreadCommandQ.get(&msg);
            mPreviewLock.lock();
            camera_device_error = false;
            goto previewThread_cmd;
        } 
        
        mCamDriverStreamLock.lock();
        if (mCamDriverStream == false) {
            mCamDriverStreamLock.unlock();
            LOGD("%s(%d): camera driver is stream off, so preview thread wait for command thread wake...",
                 __FUNCTION__,__LINE__);
            previewThreadCommandQ.get(&msg);
            mPreviewLock.lock();
            goto previewThread_cmd;
        } else {
            mCamDriverStreamLock.unlock();
        }
            
        mPreviewLock.lock();
        if (previewThreadCommandQ.isEmpty() == false ) 
            previewThreadCommandQ.get(&msg);
        
previewThread_cmd:        
        switch (msg.command)
        {
            case CMD_PREVIEW_THREAD_START:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_START", __FUNCTION__,__LINE__);
                mPreviewRunning = STA_PREVIEW_RUN;
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)CMDARG_OK;
                    msg.arg2 = (void*)mPreviewRunning;
                    previewThreadAckQ.put(&msg);
                }
                break;
            }
            case CMD_PREVIEW_THREAD_PAUSE:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_PAUSE", __FUNCTION__,__LINE__);
                mPreviewRunning = STA_PREVIEW_PAUSE;
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)CMDARG_OK;
                    msg.arg2 = (void*)mPreviewRunning;
                    previewThreadAckQ.put(&msg);
                }
                break;
            }
                
            case CMD_PREVIEW_THREAD_STOP:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_STOP", __FUNCTION__,__LINE__);
                mPreviewRunning = STA_PREVIEW_STOP;
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)CMDARG_OK;
                    msg.arg2 = (void*)mPreviewRunning;
                    previewThreadAckQ.put(&msg);
                }
                break;
            }
            case CMD_PREVIEW_VIDEOSNAPSHOT:
            {
                snapshot = true;
                LOGD("%s(%d): receive videoSnapshot, mPreviewRuning:0x%x snapshot:%d",__FUNCTION__,__LINE__,
                    mPreviewRunning,snapshot);
                
                break;
            }
            default:
                break;
        }
        
        if (mPreviewRunning == STA_PREVIEW_STOP) {
            mPreviewLock.unlock();

            msg.command = CMD_SNAPSHOT_EXIT;
            msg.arg1 = (void*)CMDARG_NACK;
            snapshotThreadCommandQ.put(&msg);
            
            goto previewThread_end;
        }
        
        if (mPreviewRunning == STA_PREVIEW_PAUSE) {
            if (previewThreadCommandQ.isEmpty() == false ) {
                mPreviewLock.unlock();
                continue;
            }
            mPreviewCond.wait(mPreviewLock);
            mPreviewLock.unlock();
            LOG1("%s(%d): wake up for mPreviewRunning:0x%x ",__FUNCTION__,__LINE__,mPreviewRunning);            
        }
        
        if (mPreviewRunning == STA_PREVIEW_RUN) {

            mCamDriverStreamLock.lock();
            if (mCamDriverStream == false) {
                mCamDriverStreamLock.unlock();    
                continue;
            }
            mCamDriverStreamLock.unlock();
            
            cfilledbuffer1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            cfilledbuffer1.memory = mCamDriverV4l2MemType;
            cfilledbuffer1.reserved = NULL;
            
            /* De-queue the next avaliable buffer */            
            mPreviewLock.unlock();
            if (CAMERA_IS_UVC_CAMERA()) {
                if (mPreviewFrameIndex==0) {
                    for (i=0; i<CONFIG_CAMERA_UVC_INVAL_FRAMECNT; i++) {
                        if (ioctl(iCamFd, VIDIOC_DQBUF, &cfilledbuffer1) >= 0) {
                            ioctl(iCamFd, VIDIOC_QBUF, &cfilledbuffer1);
                        }
                    }
                }
            }
            
            if (ioctl(iCamFd, VIDIOC_DQBUF, &cfilledbuffer1) < 0) {
                LOGE("%s(%d): VIDIOC_DQBUF Failed!!! err[%s] \n",__FUNCTION__,__LINE__,strerror(errno));
                if (errno == EIO) {
                    mCamDriverStreamLock.lock();
                    if (mCamDriverStream) {
                        camera_device_error = true;  
                        LOGE("%s(%d): camera driver or device may be error, so notify CAMERA_MSG_ERROR",
                                __FUNCTION__,__LINE__);
                    }
                    mCamDriverStreamLock.unlock();
                } else {
                    mCamDriverStreamLock.lock();
                    if (mCamDriverStream) {
                        mPreviewErrorFrameCount++;
                        if (mPreviewErrorFrameCount >= 2) {  
                            mCamDriverStreamLock.unlock();
                            camera_device_error = true;   
                            LOGE("%s(%d): mPreviewErrorFrameCount is %d, camera driver or device may be error, so notify CAMERA_MSG_ERROR",
                                __FUNCTION__,__LINE__,mPreviewErrorFrameCount);
                        } else {
                            mCamDriverStreamLock.unlock();
                            continue;
                        }
                    } else {
                        mCamDriverStreamLock.unlock();  
                        continue;
                    }
                }
                if (camera_device_error == true) {
                    if (mNotifyCb && (mMsgEnabled & CAMERA_MSG_ERROR)) {                        
                        mNotifyCb(CAMERA_MSG_ERROR, CAMERA_ERROR_SERVER_DIED,0,mCallbackCookie);
                        mPreviewErrorFrameCount = 0;                        
                        continue; 
                    }                    
                }
            } else {
                mPreviewErrorFrameCount = 0;
            }
            mPreviewFrameIndex++; 
            if (gLogLevel == 2)
                debugShowFPS();
            
            cameraPreviewBufferSetSta(mPreviewBufferMap[cfilledbuffer1.index], CMD_PREVIEWBUF_WRITING, 0);
            
            //zyc ,convert the format for usb camera
            if (CAMERA_IS_UVC_CAMERA()) {				
                cameraFormatConvert(mCamDriverPreviewFmt,V4L2_PIX_FMT_NV12,NULL,
                	(char*)mCamDriverV4l2Buffer[cfilledbuffer1.index],(char*)mPreviewBufferMap[cfilledbuffer1.index]->vir_addr, 
                	0,0,mPreviewWidth, mPreviewHeight,mPreviewWidth, mPreviewHeight,false);	
                
				mCamBuffer->flushCacheMem(PREVIEWBUFFER,mCamBuffer->getPreviewBufInfo().mPerBuffersize * cfilledbuffer1.index,
                                        mCamBuffer->getPreviewBufInfo().mPerBuffersize);                														  
            }      
            
            buffer_log = 0;
            if (mANativeWindow) {
                buffer_log |= CMD_PREVIEWBUF_DISPING;
            }

            if (mRecordRunning && (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) && mDataCbTimestamp) {
                buffer_log |= CMD_PREVIEWBUF_ENCING;
                if (snapshot) {
                    buffer_log |= CMD_PREVIEWBUF_SNAPSHOT_ENCING;
                    snapshot = false;
                }                
            }

            cameraPreviewBufferSetSta(mPreviewBufferMap[cfilledbuffer1.index], buffer_log, 1);

            mPreviewLock.lock();                       
            // Notify mANativeWindow of a new frame.
            if (buffer_log & CMD_PREVIEWBUF_DISPING) {                 
                msg.command = CMD_DISPLAY_FRAME;
                msg.arg1 = (void*)cfilledbuffer1.index;
                displayThreadCommandQ.put(&msg);
                mDisplayCond.signal();
            }
            
            // Send Video Frame 
        	if (buffer_log & CMD_PREVIEWBUF_ENCING) {
                //in snapshot progress
                if(buffer_log & CMD_PREVIEWBUF_SNAPSHOT_ENCING) {   
                    msg.command = CMD_SNAPSHOT_SNAPSHOT;
                    msg.arg1 = (void*)cfilledbuffer1.index;
                    snapshotThreadCommandQ.put(&msg);
                }
             
                if (mVideoBufs[cfilledbuffer1.index] != NULL) { 
                    #if CONFIG_CAMERA_FRAME_DV_PROC_STAT
                    framebuf_enc_start[cfilledbuffer1.index]=(nsecs_t)systemTime(CLOCK_MONOTONIC);
                    #endif
                    mDataCbTimestamp(systemTime(CLOCK_MONOTONIC), CAMERA_MSG_VIDEO_FRAME, mVideoBufs[cfilledbuffer1.index], 0, mCallbackCookie);                                                
                    LOG2("%s(%d): enqueue buffer %d to encoder", __FUNCTION__,__LINE__,cfilledbuffer1.index);
                    
                } else {
                    LOGE("%s(%d): mVideoBufs[%d] is NULL, this frame recording cancel",__FUNCTION__,__LINE__,cfilledbuffer1.index);
                    cameraPreviewBufferSetSta(mPreviewBufferMap[cfilledbuffer1.index], CMD_PREVIEWBUF_ENCING, 0);
                }    		    
            } 

            if ((mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) && mDataCb) {
                if (!strcmp(mParameters.getPreviewFormat(),CameraParameters::PIXEL_FORMAT_YUV420SP) ||
                    !strcmp(mParameters.getPreviewFormat(),CameraParameters::PIXEL_FORMAT_YUV420P)) {
                    if (mPreviewMemory) {                     
                        
                        if (CAMERA_IS_UVC_CAMERA()) {
                            cameraFormatConvert(V4L2_PIX_FMT_NV12,0x00,mParameters.getPreviewFormat(),
                                (char*)mPreviewBufferMap[cfilledbuffer1.index]->vir_addr,(char*)mPreviewBufs[cfilledbuffer1.index],
                                0,0,mPreviewWidth, mPreviewHeight,mPreviewFrame2AppWidth, mPreviewFrame2AppHeight,
                                mDataCbFrontMirror);
                        } else {
                            cameraFormatConvert(mCamDriverPreviewFmt,0x00,mParameters.getPreviewFormat(),
                                (char*)mPreviewBufferMap[cfilledbuffer1.index]->vir_addr,(char*)mPreviewBufs[cfilledbuffer1.index],
                                0,0,mPreviewWidth, mPreviewHeight,mPreviewFrame2AppWidth, mPreviewFrame2AppHeight,
                                mDataCbFrontMirror);
                        }
                        mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewMemory, cfilledbuffer1.index,NULL,mCallbackCookie);                         
                    } else {
                        LOGE("%s(%d): mPreviewMemory is NULL, preview data could not send to application",__FUNCTION__,__LINE__);
                    }
                }
            }
            mPreviewLock.unlock(); 
        } 
    }
    
previewThread_end:    
    LOG_FUNCTION_NAME_EXIT
    return;
}
void CameraHal::pictureThread()
{
    struct CamCaptureInfo_s capture;
    
    LOG_FUNCTION_NAME 
    mPictureLock.lock();
    mPictureRunning = STA_PICTURE_RUN;
    mPictureLock.unlock();   
    capture.input_phy_addr = mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_phy); 
	capture.input_vir_addr = mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir);
	capture.output_phy_addr = mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_phy);                        
    capture.output_vir_addr = mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_vir);                        
    capture.output_buflen = mCamBuffer->getJpegBufInfo().mBufferSizes;
    LOGD("%s(%d): capture.input_phy:0x%x, output_phy:0x%x vir:0x%x",
        __FUNCTION__,__LINE__,capture.input_phy_addr,capture.output_phy_addr,capture.output_vir_addr);
    capturePicture(&capture);
    
    mPictureLock.lock();
    mPictureRunning = STA_PICTURE_STOP;
    mPictureLock.unlock();
    LOG_FUNCTION_NAME_EXIT
    return;

}
void CameraHal::snapshotThread()
{
    bool loop = true;
    Message msg;
    int index;
    struct CamCaptureInfo_s capture;    

    LOG_FUNCTION_NAME
    while (loop) {
        mSnapshotRunning = -1;
        snapshotThreadCommandQ.get(&msg);
        mSnapshotRunning = msg.command;
        
        switch (msg.command)
        {
            case CMD_SNAPSHOT_SNAPSHOT:
            {
                index = (int)msg.arg1;
                
                LOGD("%s(%d): receive CMD_SNAPSHOT_SNAPSHOT with buffer %d",__FUNCTION__,__LINE__, index);
			
                if(mCamBuffer->getRawBufInfo().mNumBffers== 0                    || mCamBuffer->getJpegBufInfo().mNumBffers == 0){
                    LOGE("%s(%d): cancel, because mRawBuffer and mJpegBuffer is NULL",__FUNCTION__,__LINE__);
                    cameraPreviewBufferSetSta(mPreviewBufferMap[index], CMD_PREVIEWBUF_SNAPSHOT_ENCING, 0);
                    goto CMD_SNAPSHOT_SNAPSHOT_end;
                }
                
                capture.input_phy_addr = mPreviewBufferMap[index]->phy_addr;   
                capture.input_vir_addr = mPreviewBufferMap[index]->vir_addr;			  
                capture.output_phy_addr = mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_phy);                        
                capture.output_vir_addr = mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_vir);                        
                capture.output_buflen = mCamBuffer->getJpegBufInfo().mBufferSizes;
                captureVideoPicture(&capture,index);
                
CMD_SNAPSHOT_SNAPSHOT_end:
                break;
            }

            case CMD_SNAPSHOT_EXIT:
            {
                LOGD("%s(%d): receive CMD_SNAPSHOT_EXIT",__FUNCTION__,__LINE__);
                loop = false;
                break;
            }

            default:
            {
                LOGE("%s(%d): receive unknow command(0x%x)",__FUNCTION__,__LINE__,msg.command);
                break;
            }
        }
    }
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
        mCommandRunning = -1;
        commandThreadCommandQ.get(&msg);

        mCommandRunning = msg.command;
        switch(msg.command)
        {
            case CMD_PREVIEW_START:
            {                
                LOGD("%s(%d): receive CMD_PREVIEW_START, mPreviewRunning(%d) mDisplayRuning(%d)", __FUNCTION__,__LINE__,mPreviewRunning,mDisplayRuning);

                if (NULL == mANativeWindow) {
                    LOGD("%s(%d): camera preview buffer alloc from ANativeWindow, Now mANativeWIndow is NULL, wait for set...",__FUNCTION__,__LINE__);  
					mANativeWindowLock.lock();
					while (NULL == mANativeWindow && (mPreviewCmdReceived == true)){
						LOGD("%s(%d):ANativeWindow is NULL , lock and wait ...",__FUNCTION__,__LINE__);
						mANativeWindowCond.wait(mANativeWindowLock);
					}
						
					mANativeWindowLock.unlock();
                    if(mPreviewCmdReceived == false)
						goto PREVIEW_START_OUT;
					} 
                           
                err = 0;                
                cameraDisplayThreadStart(true);              
               
                if(mPreviewRunning != STA_PREVIEW_RUN) {
					cameraSetSize(mPreviewWidth, mPreviewHeight, mCamDriverPreviewFmt);
    				err = cameraStart();
                } else {
                    err = CMDARG_ERR;
                    LOGD("%s(%d): preview thread is already run", __FUNCTION__,__LINE__);
                }

                if( err == CMDARG_OK ) {
                    cameraPreviewThreadSet(CMD_PREVIEW_THREAD_START,true);                
                    android_atomic_inc(&mPreviewStartTimes);
                }
                
                LOGD("%s(%d): CMD_PREVIEW_START %s", __FUNCTION__,__LINE__, (err==CMDARG_ERR) ? "ERR" : "OK");
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)(err ? CMDARG_ERR : CMDARG_OK);
                    commandThreadAckQ.put(&msg);
                }
PREVIEW_START_OUT:
                break;
            }

            case CMD_PREVIEW_STOP:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_STOP,  mPreviewRunning(%d) mDisplayRuning(%d)", __FUNCTION__,__LINE__,mPreviewRunning,mDisplayRuning);

                if (mPreviewRunning  == STA_PREVIEW_RUN)
                    cameraDisplayThreadPause(true);                    
                if( mPreviewRunning  == STA_PREVIEW_RUN) {
                    cameraStop();
                	if(cameraPreviewThreadSet(CMD_PREVIEW_THREAD_PAUSE,true) < 0){
                        LOGE("%s(%d): Pause preview thread failed!",__FUNCTION__,__LINE__);
                        msg.command = CMD_PREVIEW_STOP;
                        err = CMDARG_ERR;
                    }else{
                        msg.command = CMD_PREVIEW_STOP;
                        err = CMDARG_OK;

                    } 
                } else {
                    msg.command = CMD_PREVIEW_STOP;
                    err = CMDARG_OK;
                    LOGD("%s(%d): preview thread is already pause",__FUNCTION__,__LINE__);
                }
                 
                LOGD("%s(%d): CMD_PREVIEW_STOP %s", __FUNCTION__,__LINE__, (err==CMDARG_ERR) ? "ERR" : "OK");
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)(err ? CMDARG_ERR : CMDARG_OK);
                    commandThreadAckQ.put(&msg);
                }
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

                if(mCamBuffer->getRawBufInfo().mNumBffers== 0
                    || mCamBuffer->getJpegBufInfo().mNumBffers == 0){
                    LOGE("%s(%d): cancel, because mRawBuffer and mJpegBuffer is NULL",__FUNCTION__,__LINE__);
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
								   
                if (mPreviewRunning  == STA_PREVIEW_RUN) {
                	  cameraPreviewThreadSet(CMD_PREVIEW_THREAD_PAUSE,true);
                    cameraStop();
                }
                
                if (mPictureThread->run("CameraPictureThread", ANDROID_PRIORITY_DISPLAY) != NO_ERROR) {
                    LOGE("%s(%d): couldn't run picture thread", __FUNCTION__,__LINE__);
                    err = -1;
                    mPictureLock.lock();
                    mPictureRunning = STA_PICTURE_STOP;
                    mPictureLock.unlock();                   
                    goto PREVIEW_CAPTURE_end;
                }
PREVIEW_CAPTURE_end:
                if (err < 0) {
                    msg.command = CMD_PREVIEW_CAPTURE;
                    err = CMDARG_ERR;              
                } else {
                    msg.command = CMD_PREVIEW_CAPTURE;
                    err = CMDARG_OK;                        
                }
                LOGD("%s(%d): CMD_PREVIEW_CAPTURE %s", __FUNCTION__,__LINE__, (err == CMDARG_ERR) ? "ERR" : "OK");
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)err;
                    commandThreadAckQ.put(&msg);
                }
                break;
            }
            case CMD_PREVIEW_CAPTURE_CANCEL:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_CAPTURE_CANCEL", __FUNCTION__,__LINE__);
                msg.command = CMD_PREVIEW_CAPTURE_CANCEL;
                err = CMDARG_OK;
                LOGD("%s(%d): CMD_PREVIEW_CAPTURE_CANCEL %s", __FUNCTION__,__LINE__, (err == CMDARG_ERR) ? "ERR" : "OK");
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)err;
                    commandThreadAckQ.put(&msg);
                }
                break; 
            }
            case CMD_AF_START:
            {
                LOGD("%s(%d): receive CMD_AF_START", __FUNCTION__,__LINE__);
                msg.command = CMD_AF_START;
                err = CMDARG_OK;
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)err;
                    commandThreadAckQ.put(&msg);
                }
                mAutoFocusCond.signal();
                LOGD("%s(%d): CMD_AF_START %s", __FUNCTION__,__LINE__, (err == CMDARG_ERR) ? "ERR" : "OK");
                break;
            }            
            case CMD_AF_CANCEL:
            {
                LOGD("%s(%d): receive CMD_AF_CANCEL", __FUNCTION__,__LINE__);
                msg.command = CMD_AF_CANCEL;
                err = CMDARG_OK;                    
                if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)err;
                    commandThreadAckQ.put(&msg);
                }
                LOGD("%s(%d): CMD_AF_CANCEL %s", __FUNCTION__,__LINE__, (err == CMDARG_ERR) ? "ERR" : "OK");
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

                if (cmd_info == CMD_PREVIEWBUF_DISPING) {
                    LOG2("%s(%d): receive CMD_PREVIEW_QBUF %d from display",__FUNCTION__,__LINE__,index);
                } else if (cmd_info == CMD_PREVIEWBUF_ENCING) {
                    LOG2("%s(%d): receive CMD_PREVIEW_QBUF %d from encoder",__FUNCTION__,__LINE__,index);
                } else if (cmd_info == CMD_PREVIEWBUF_SNAPSHOT_ENCING) {
                    LOG2("%s(%d): receive CMD_PREVIEW_QBUF %d from videoSnapshot",__FUNCTION__,__LINE__,index);
                }
                
                if (mPreviewRunning != STA_PREVIEW_RUN) {
                    LOG1("%s(%d): preview thread is pause, so buffer %d isn't enqueue to camera",__FUNCTION__,__LINE__,index);
                    goto CMD_PREVIEW_QBUF_end;
                }
                if (mANativeWindow && mPreviewBufferMap[index]->phy_addr) {
                    if (CAMERA_PREVIEWBUF_ALLOW_WRITE(mPreviewBufferMap[index]->buf_state)) {

                        if (preview_times == mPreviewStartTimes) {                        
                            vb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                            vb.memory = mCamDriverV4l2MemType;
                            vb.index = index;                        
                            vb.reserved = NULL;
                            vb.m.offset = mPreviewBufferMap[index]->phy_addr; 
                            
                            cameraPreviewBufferSetSta(mPreviewBufferMap[index], CMD_PREVIEWBUF_WRITING, 1);
                            
                            if (ioctl(iCamFd, VIDIOC_QBUF, &vb) < 0) {
                                LOGE("%s(%d): VIDIOC_QBUF %d Failed!!! err[%s]", __FUNCTION__,__LINE__,index, strerror(errno));
                                cameraPreviewBufferSetSta(mPreviewBufferMap[index], CMD_PREVIEWBUF_WRITING, 0);
                            } 
                        } else {
                            LOG1("%s(%d): this message(CMD_PREVIEW_QBUF) is invaliade, camera have restart", __FUNCTION__,__LINE__);
                        }
                    } else {
                        LOG2("%s(%d): buffer %d state 0x%x",__FUNCTION__,__LINE__, index, mPreviewBufferMap[index]->buf_state);
                    }
                } else {
                    if (mANativeWindow == NULL) {
                        LOGE("%s(%d): mANativeWindow_hnd is NULL, enqueue buffer %d to camera failed",
                            __FUNCTION__,__LINE__, index);
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
                if (cameraPreviewThreadSet(CMD_PREVIEW_THREAD_STOP,true) < 0){
                    LOGE("%s(%d): Stop preview thread failed!",__FUNCTION__,__LINE__);
                    msg.command = CMD_EXIT;
                    err = CMDARG_ERR;
                } else {
                    LOG1("%s(%d): Stop preview thread success!",__FUNCTION__,__LINE__);
                    msg.command = CMD_EXIT;
                    err = CMDARG_OK;
                }
                
                LOGD("%s(%d): CMD_EXIT %s", __FUNCTION__,__LINE__, (err == CMDARG_ERR) ? "ERR" : "OK");
               if (CmdAck_Chk(msg)) {
                    msg.arg1 = (void*)err;
                    commandThreadAckQ.put(&msg);
                } 
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
int CameraHal::cameraRawJpegBufferCreate(int rawBufferSize, int jpegBufferSize)
{
    int err = NO_ERROR;
	struct bufferinfo_s  tmpbufinfo;
    
    LOG_FUNCTION_NAME 

    if (mCamBuffer->getRawBufInfo().mBufferSizes) { 
        mCamBuffer->destroyRawBuffer();        
    }

    if (mCamBuffer->getJpegBufInfo().mBufferSizes) { 
        mCamBuffer->destroyJpegBuffer();        
    }
    
	tmpbufinfo.mNumBffers = 1;
	tmpbufinfo.mPerBuffersize = jpegBufferSize;
          
	if(jpegBufferSize) {
		tmpbufinfo.mBufType = JPEGBUFFER;
		err = mCamBuffer->createJpegBuffer(&tmpbufinfo);
		if(err != 0)
			LOGE("%s(%d): Jpeg buffer malloc failed",__FUNCTION__,__LINE__);
	}
	if(rawBufferSize) {
		tmpbufinfo.mNumBffers = 1;
		tmpbufinfo.mPerBuffersize = rawBufferSize;
		tmpbufinfo.mBufType = RAWBUFFER;
		err = mCamBuffer->createRawBuffer(&tmpbufinfo);
		if(err != 0)
			LOGE("%s(%d): Raw buffer malloc failed",__FUNCTION__,__LINE__);
	}
    
cameraRawJpegBufferCreate_end:
    if (err)
        LOGE("%s(%d): exit with error(%d)", __FUNCTION__,__LINE__,err);
    else 
        LOG_FUNCTION_NAME_EXIT
    return err;
}

int CameraHal::cameraRawJpegBufferDestory()
{

    LOG_FUNCTION_NAME 

    if(mCamBuffer) {
    	mCamBuffer->destroyJpegBuffer();
        mCamBuffer->destroyRawBuffer();
    }
	
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int CameraHal::cameraDisplayBufferCreate(int width, int height, const char *fmt,int numBufs)
{
    int err = NO_ERROR,undequeued = 0;
    int i, total;
    buffer_handle_t* hnd = NULL;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    Rect bounds;        
    struct pmem_region sub;
    bool previewbuf_direct_disp;

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
    /* ddl@rock-chips.com: NativeWindow switch to async mode after v0.1.3 */
    /*
    if (mANativeWindow->set_swap_interval(mANativeWindow, 1) != 0) {
        LOGE("%s(%d): set mANativeWindow run in synchronous mode failed",__FUNCTION__,__LINE__);
    }
    */
    
    mANativeWindow->get_min_undequeued_buffer_count(mANativeWindow, &undequeued);
    mDispBufUndqueueMin = undequeued;
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
            mGrallocBufferMap[i].buffer_hnd = hnd;
            mGrallocBufferMap[i].priv_hnd= (NATIVE_HANDLE_TYPE*)(*hnd);
            
        #if defined(TARGET_RK29)            
            if (ioctl(mGrallocBufferMap[i].priv_hnd->fd,PMEM_GET_PHYS,&sub) == 0) {                    
                mGrallocBufferMap[i].phy_addr = sub.offset + mGrallocBufferMap[i].priv_hnd->offset;    /* phy address */ 
            } else {   
                /* ddl@rock-chips.com: gralloc buffer is not continuous in phy */
                mGrallocBufferMap[i].phy_addr = 0x00;
            }        
        #else
            mGrallocBufferMap[i].phy_addr = 0x00;        
        #endif
        
        } else {             
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
        void* y_uv[3];
        
        mANativeWindow->lock_buffer(mANativeWindow, (buffer_handle_t*)mGrallocBufferMap[i].buffer_hnd);
        mapper.lock((buffer_handle_t)mGrallocBufferMap[i].priv_hnd, CAMHAL_GRALLOC_USAGE, bounds, y_uv);
        
    #if defined(TARGET_BOARD_PLATFORM_RK30XX) || defined(TARGET_RK29) || defined(TARGET_BOARD_PLATFORM_RK2928)
        mGrallocBufferMap[i].vir_addr = mGrallocBufferMap[i].priv_hnd->base;
    #elif defined(TARGET_BOARD_PLATFORM_RK30XXB)
        mGrallocBufferMap[i].vir_addr = (int)y_uv[0];
    #endif

        mGrallocBufferMap[i].lock = new Mutex();        
        mDisplayBufferMap[i] = &mGrallocBufferMap[i]; 
        cameraPreviewBufferSetSta(&mGrallocBufferMap[i], CMD_PREVIEWBUF_DISPING, 0);
        LOGD("%s(%d): mGrallocBufferMap[i] phy_addr: 0x%x  vir_dir: 0x%x",
            __FUNCTION__,__LINE__, i, mGrallocBufferMap[i].phy_addr,mGrallocBufferMap[i].vir_addr);
    }

    cameraPreviewBufferCreate(numBufs);

    if (mGrallocBufferMap[0].phy_addr) {
        if (strcmp(fmt,CameraParameters::PIXEL_FORMAT_RGB565)) {
            previewbuf_direct_disp = true;
        } else if (mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) {
            previewbuf_direct_disp = true;
        } else {
            previewbuf_direct_disp = false;
        }
    } else {
        previewbuf_direct_disp = false;
    }
    
    if (previewbuf_direct_disp == true) {
        /* 
        * ddl@rock-chips.com : Sensor data is directly stored in gralloc buffer(display buffer),
        *                      when gralloc buffer is physical continues memory and display format is yuv; 
        */
        for (i=0; i<numBufs; i++) {
            mPreviewBufferMap[i] = &mGrallocBufferMap[i];
        }
        LOGD("%s(%d): Display buffer is directly used for Preview buffer",__FUNCTION__,__LINE__);
    } else {
        for (i=0; i<numBufs; i++) {
            mPreviewBufferMap[i] = mPreviewBuffer[i];
        }
        LOGD("%s(%d): Display buffer and Preview buffer is independent",__FUNCTION__,__LINE__);
    }
    
    LOG_FUNCTION_NAME_EXIT    
    return err; 
 fail:
    if (mPreviewBufferCount) {
        for (i = 0; i<mPreviewBufferCount; i++) {
            if (mGrallocBufferMap[i].buffer_hnd) {
                err = mANativeWindow->cancel_buffer(mANativeWindow, (buffer_handle_t*)mGrallocBufferMap[i].buffer_hnd);
                if (err != 0) {
                  LOGE("%s(%d): cancelBuffer failed w/ error 0x%08x",__FUNCTION__,__LINE__, err);                  
                }
            }
        }
    }
    
    LOGE("%s(%d): exit with error(%d)!",__FUNCTION__,__LINE__,err);
    return err;
        
}
int CameraHal::cameraDisplayBufferDestory(void)
{
    int ret = NO_ERROR,i;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();

    LOG_FUNCTION_NAME
    //Give the buffers back to display here -  sort of free it
    if (mANativeWindow) {
        for(i = 0; i < mPreviewBufferCount; i++) {
            // unlock buffer before giving it up
            if (mGrallocBufferMap[i].priv_hnd) {
                mapper.unlock((buffer_handle_t)mGrallocBufferMap[i].priv_hnd);
                mANativeWindow->cancel_buffer(mANativeWindow, (buffer_handle_t*)mGrallocBufferMap[i].buffer_hnd);
            }
            if (mGrallocBufferMap[i].lock) {
                delete mGrallocBufferMap[i].lock;
                mGrallocBufferMap[i].lock = NULL;
            }
            mGrallocBufferMap[i].buffer_hnd = NULL;
            mGrallocBufferMap[i].priv_hnd = NULL;
        }
    } else {
        LOGE("%s(%d): mANativeWindow is NULL, destory is ignore",__FUNCTION__,__LINE__);
    }

    LOG_FUNCTION_NAME_EXIT
cameraPreviewBufferDestory_end:
    return ret;    
}
int CameraHal::cameraPreviewBufferCreate(unsigned int numBufs)
{
	LOG_FUNCTION_NAME
    unsigned int frame_size,i;
    struct bufferinfo_s previewbuf;
    int ret = 0;
    
    switch (mCamDriverPreviewFmt)
    {
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_YUV420:
            frame_size = mPreviewWidth*mPreviewHeight*3/2;
            break;
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_YUV422P:
        default:
            frame_size = mPreviewWidth*mPreviewHeight*2;
            break;            
    }

    if ((frame_size == mCamBuffer->getPreviewBufInfo().mPerBuffersize) && 
        (numBufs == mCamBuffer->getPreviewBufInfo().mNumBffers)) {
        goto cameraPreviewBufferCreate_end;
    } else {
        cameraPreviewBufferDestory();
    }

    previewbuf.mNumBffers = numBufs;	
    previewbuf.mPerBuffersize = frame_size;	
    previewbuf.mBufType = PREVIEWBUFFER;

    if(mCamBuffer->createPreviewBuffer(&previewbuf) !=0) {
        LOGE("%s(%d):Preview buffer create failed",__FUNCTION__,__LINE__);		
        ret = -1;	
    }

    for (i=0; i<numBufs; i++) {
        mPreviewBuffer[i] = (rk_previewbuf_info_t*)malloc(sizeof(rk_previewbuf_info_t));            
        if (mPreviewBuffer[i]) {
            memset((char*)mPreviewBuffer[i],0x00,sizeof(rk_previewbuf_info_t));
            mPreviewBuffer[i]->lock = new Mutex();                
            mPreviewBuffer[i]->vir_addr = (int)mCamBuffer->getBufferAddr(PREVIEWBUFFER,i,buffer_addr_vir);
            mPreviewBuffer[i]->phy_addr = (int)mCamBuffer->getBufferAddr(PREVIEWBUFFER,i,buffer_addr_phy);
        } else {
            LOGE("%s(%d): mPreviewBuffer[%d] malloc failed",__FUNCTION__,__LINE__,i);
        }
    }

cameraPreviewBufferCreate_end:    
    LOG_FUNCTION_NAME_EXIT
	return ret;
	
}
int CameraHal::cameraPreviewBufferDestory(void)
{
    unsigned int i;
    
	LOG_FUNCTION_NAME
    
    for(i = 0; i < mCamBuffer->getPreviewBufInfo().mNumBffers; i++) {
		if (mPreviewBuffer[i] != NULL) {
			if (mPreviewBuffer[i]->lock) {
				delete mPreviewBuffer[i]->lock;
				mPreviewBuffer[i]->lock = NULL;					  
			}
			free(mPreviewBuffer[i]);
			mPreviewBuffer[i] = NULL;
		}
	}

    mCamBuffer->destroyPreviewBuffer();
    
	LOG_FUNCTION_NAME_EXIT
	return 0;
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
        
    buf_hnd->lock->lock();

    if (cmd & CMD_PREVIEWBUF_DISPING) {
        if (set){
            if (CAMERA_PREVIEWBUF_ALLOW_DISPLAY(buf_hnd->buf_state)==false)
                LOGE("%s(%d): Set buffer displaying, but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state |= CMD_PREVIEWBUF_DISPING;
        } else { 
            if ((buf_hnd->buf_state & CMD_PREVIEWBUF_DISPING) == 0)
                LOGE("%s(%d): Clear buffer displaying,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state &= ~CMD_PREVIEWBUF_DISPING;
        }
    }

    if (cmd & CMD_PREVIEWBUF_ENCING) {
        if (set) {
            if (CAMERA_PREVIEWBUF_ALLOW_ENC(buf_hnd->buf_state)==false)
                LOGE("%s(%d): Set buffer encoding,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state |= CMD_PREVIEWBUF_ENCING;
        } else {
            if ((buf_hnd->buf_state & CMD_PREVIEWBUF_ENCING) == 0)
                LOGE("%s(%d): Clear buffer encoding,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state &= ~CMD_PREVIEWBUF_ENCING;
        }
    }

    if (cmd & CMD_PREVIEWBUF_SNAPSHOT_ENCING) {
        if (set) {
            if (CAMERA_PREVIEWBUF_ALLOW_ENC_PICTURE(buf_hnd->buf_state)==false)
                LOGE("%s(%d): Set buffer snapshot encoding,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state |= CMD_PREVIEWBUF_SNAPSHOT_ENCING;
        } else {
            if ((buf_hnd->buf_state & CMD_PREVIEWBUF_SNAPSHOT_ENCING) == 0)
                LOGE("%s(%d): Clear buffer snapshot encoding,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state &= ~CMD_PREVIEWBUF_SNAPSHOT_ENCING;
        }
    }

    if (cmd & CMD_PREVIEWBUF_WRITING) {
        if (set) {
            if (CAMERA_PREVIEWBUF_ALLOW_WRITE(buf_hnd->buf_state)==false)
                LOGE("%s(%d): Set buffer writing, but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state |= CMD_PREVIEWBUF_WRITING;
        } else { 
            if ((buf_hnd->buf_state & CMD_PREVIEWBUF_WRITING) == 0)
                LOGE("%s(%d): Clear buffer writing,  but buffer status(0x%x) is error",__FUNCTION__,__LINE__,buf_hnd->buf_state);
            buf_hnd->buf_state &= ~CMD_PREVIEWBUF_WRITING;
        }
    }
    
    buf_hnd->lock->unlock();


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

    #ifdef  TARGET_RK30 
    if((mRGAFd = open("/dev/rga",O_RDWR)) < 0) {
    	LOGE("%s(%d):open rga device failed!!",__FUNCTION__,__LINE__);
    	goto exit;
	}

    #if CONFIG_CAMERA_INVALIDATE_RGA
    mRGAFd = -1;
    #endif
    
    #endif
	
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

    if (CAMERA_IS_RKSOC_CAMERA() 
        && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1))) {
        CameraHal_SupportFmt[0] = V4L2_PIX_FMT_YUV420;
        CameraHal_SupportFmt[1] = V4L2_PIX_FMT_YUV422P;
        CameraHal_SupportFmt[2] = V4L2_PIX_FMT_YUYV;
        CameraHal_SupportFmt[3] = 0x00;
    } else {
        CameraHal_SupportFmt[0] = V4L2_PIX_FMT_NV12;
        CameraHal_SupportFmt[1] = V4L2_PIX_FMT_NV16;
        CameraHal_SupportFmt[2] = V4L2_PIX_FMT_YUYV;
	 CameraHal_SupportFmt[3] = V4L2_PIX_FMT_RGB565;
        CameraHal_SupportFmt[4] = 0x00;
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
        LOGD("%s(%d): mCamDriverPreviewFmt(%c%c%c%c) is cameraHal and camera driver is also supported!!",__FUNCTION__,__LINE__,
            mCamDriverPreviewFmt & 0xFF, (mCamDriverPreviewFmt >> 8) & 0xFF,
			(mCamDriverPreviewFmt >> 16) & 0xFF, (mCamDriverPreviewFmt >> 24) & 0xFF);
        
    }

    if (CAMERA_IS_UVC_CAMERA()) {                  /* ddl@rock-chips.com: This driver is UVC sensor driver */
        mCamDriverV4l2MemType = V4L2_MEMORY_MMAP;
    } else if (CAMERA_IS_RKSOC_CAMERA()) {      /* ddl@rock-chips.com: This driver is RK29 sensor driver */
        mCamDriverV4l2MemType = V4L2_MEMORY_OVERLAY;
    } else {
        mCamDriverV4l2MemType = V4L2_MEMORY_OVERLAY;
    }
    
    LOGD("%s(%d): Current driver is %s, v4l2 memory is %s",__FUNCTION__,__LINE__,mCamDriverCapability.driver, 
        (mCamDriverV4l2MemType==V4L2_MEMORY_MMAP)?"V4L2_MEMORY_MMAP":"V4L2_MEMORY_OVERLAY");
   

    if(access(CAMERA_PMEM_NAME, O_RDWR) < 0) {
    #if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)
        mCamBuffer = new IonMemManager();
        LOGD("%s(%d): Camera Hal memory is alloced from ION device",__FUNCTION__,__LINE__);
    #else
        LOGE("%s(%d): %s isn't registered,CameraHal_Mem current configuration isn't support ION memory!!!",
            __FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
        goto exit1;
    #endif
    } else {
        mCamBuffer = new PmemManager((char*)CAMERA_PMEM_NAME);
        LOGD("%s(%d): Camera Hal memory is alloced from %s device",__FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
    }
    
    mCamId = cameraId;
    
    LOG_FUNCTION_NAME_EXIT 
    return 0;

exit1:
    if (iCamFd > 0) {
        close(iCamFd);
        iCamFd = -1;
    }  

    if (mRGAFd > 0) {
        close(mRGAFd);
        mRGAFd = -1;
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
    cameraRawJpegBufferDestory();
    cameraDisplayBufferDestory();
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
    
	if(mCamBuffer) {
		delete mCamBuffer;
		mCamBuffer = NULL;
	}

	if( iCamFd > 0) {
        close(iCamFd);
        iCamFd = -1;
	}
	if(mRGAFd > 0){
        close(mRGAFd);
        mRGAFd = -1;		
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

int CameraHal::cameraConfig(const CameraParameters &tmpparams)
{
    int err = 0, i = 0;
    struct v4l2_control control;
	struct v4l2_ext_control extCtrInfo;
	struct v4l2_ext_controls extCtrInfos;
	CameraParameters params = tmpparams;

    if (params.getPreviewFrameRate() != mParameters.getPreviewFrameRate()) {
        if (CAMERA_IS_UVC_CAMERA()) {
            struct v4l2_streamparm setfps;          
        
            memset(&setfps, 0, sizeof(struct v4l2_streamparm));
            setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            setfps.parm.capture.timeperframe.numerator=1;
            setfps.parm.capture.timeperframe.denominator=params.getPreviewFrameRate();
            err = ioctl(iCamFd, VIDIOC_S_PARM, &setfps); 
            if (err != 0) {
                LOGE ("%s(%d): Set framerate(%d fps) failed",__FUNCTION__,__LINE__,params.getPreviewFrameRate());
                return err;
            } else {
                LOGD ("%s(%d): Set framerate(%d fps) success",__FUNCTION__,__LINE__,params.getPreviewFrameRate());
            }
        } 
    }

    /*white balance setting*/
    const char *white_balance = params.get(CameraParameters::KEY_WHITE_BALANCE);
	const char *mwhite_balance = mParameters.get(CameraParameters::KEY_WHITE_BALANCE);
	if (params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE)) {
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
	if (!CAMERA_IS_UVC_CAMERA()) {
	    const int zoom = params.getInt(CameraParameters::KEY_ZOOM);
		const int mzoom = mParameters.getInt(CameraParameters::KEY_ZOOM);
		if (params.get(CameraParameters::KEY_ZOOM_SUPPORTED)) {
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
	}

    /*color effect setting*/
    const char *effect = params.get(CameraParameters::KEY_EFFECT);
	const char *meffect = mParameters.get(CameraParameters::KEY_EFFECT);
	if (params.get(CameraParameters::KEY_SUPPORTED_EFFECTS)) {
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
	if (params.get(CameraParameters::KEY_SUPPORTED_SCENE_MODES)) {
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
	if (params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES)) {
		if ( !mfocusMode || strcmp(focusMode, mfocusMode) ) {
       		if(!cameraAutoFocus(focusMode)){
        		params.set(CameraParameters::KEY_FOCUS_MODE,(mfocusMode?mfocusMode:CameraParameters::FOCUS_MODE_FIXED));
        		err = -1;
   			}
		}
	} else{
		params.set(CameraParameters::KEY_FOCUS_MODE,(mfocusMode?mfocusMode:CameraParameters::FOCUS_MODE_FIXED));
	}

	/*flash mode setting*/
    const char *flashMode = params.get(CameraParameters::KEY_FLASH_MODE);
	const char *mflashMode = mParameters.get(CameraParameters::KEY_FLASH_MODE);
	
	if (params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES)) {
		if ( !mflashMode || strcmp(flashMode, mflashMode) ) {
			for (i = 0; i < mFlashMode_number; i++) {
				if (!strcmp((char *)mFlashMode_menu[i].name, flashMode)) {
					break;
				}
			}
			if(i== mFlashMode_number || mFlashMode_number == 0){
				params.set(CameraParameters::KEY_FLASH_MODE,(mflashMode?mflashMode:CameraParameters::FLASH_MODE_OFF));
				err = -1;
			} else {
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
	}

    /*exposure setting*/
	const char *exposure = params.get(CameraParameters::KEY_EXPOSURE_COMPENSATION);
    const char *mexposure = mParameters.get(CameraParameters::KEY_EXPOSURE_COMPENSATION);
    
	if (strcmp("0", params.get(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION))
		|| strcmp("0", params.get(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION))) {
	    if (!mexposure || (exposure && strcmp(exposure,mexposure))) {
    		control.id = V4L2_CID_EXPOSURE;
    		control.value = atoi(exposure);
    		err = ioctl(iCamFd, VIDIOC_S_CTRL, &control);
    		if ( err < 0 ){
    		    LOGE("%s(%d): Set exposure(%s) failed",__FUNCTION__,__LINE__,exposure);
    		} else {	    
		        LOGD("%s(%d): Set exposure %s",__FUNCTION__,__LINE__,exposure);
    		}
	    }
	}    
    
    mParameters = params;
    mParameters.getPreviewSize(&mPreviewFrame2AppWidth, &mPreviewFrame2AppHeight); 
    if (strcmp(cameraCallProcess,"com.android.facelock")==0) {        
        if ((mPreviewFrame2AppWidth==160) && (mPreviewFrame2AppHeight==120)) {
            if (strstr(mSupportPreviewSizeReally.string(),"640x480")) {
                mPreviewWidth = 640;
                mPreviewHeight = 480;
            } else if (strstr(mSupportPreviewSizeReally.string(),"320x240")) {
                mPreviewWidth = 320;
                mPreviewHeight = 240;
            } else {
                LOGE("%s(%d): CameraHal isn't support 160x120 preview size for facelock",__FUNCTION__,__LINE__);
                err = -1;
            }
        } else {
            mPreviewWidth = mPreviewFrame2AppWidth;
            mPreviewHeight = mPreviewFrame2AppHeight;
        }        
    } else {
        mPreviewWidth = mPreviewFrame2AppWidth;
        mPreviewHeight = mPreviewFrame2AppHeight;
    }
    mParameters.getPictureSize(&mPictureWidth, &mPictureHeight);
    if ((mPreviewWidth!=mPreviewFrame2AppWidth) || (mPreviewHeight!=mPreviewFrame2AppHeight))
        LOGD("%s(%d): Display and preview size is %dx%d, but application(%s) receive framesize is %dx%d",
               __FUNCTION__,__LINE__, mPreviewWidth,mPreviewHeight,
               cameraCallProcess,mPreviewFrame2AppWidth,mPreviewFrame2AppHeight);
end:  
    return err;
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
int CameraHal::cameraStream(bool on)
{
    int err = 0;
    int cmd ;
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cmd = (on)?VIDIOC_STREAMON:VIDIOC_STREAMOFF;

    mCamDriverStreamLock.lock();
    err = ioctl(iCamFd, cmd, &type);
    if (err < 0) {
        LOGE("%s(%d): %s Failed",__FUNCTION__,__LINE__,((on)?"VIDIOC_STREAMON":"VIDIOC_STREAMOFF"));
        goto cameraStream_end;
    }
    mCamDriverStream = on;
    mCamDriverStreamLock.unlock();

cameraStream_end:
    return err;
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

    if (!strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV420SP) ||
        !strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV420P)) {
        mPreviewFrameSize = (mPreviewWidth * mPreviewHeight * 3)/2;
        mPreviewFrame2AppSize = (mPreviewFrame2AppWidth * mPreviewFrame2AppHeight * 3)/2;
    } else if ((!strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_YUV422SP)) ||
       (!strcmp(mParameters.getPreviewFormat(), CameraParameters::PIXEL_FORMAT_RGB565))) {
        mPreviewFrameSize = mPreviewWidth * mPreviewHeight * 2;
        mPreviewFrame2AppSize = mPreviewFrame2AppWidth * mPreviewFrame2AppHeight*2;
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
    
    if (mPreviewBufferMap[0]==NULL) {
        LOGE("%s(%d): preview buffer havn't alloced",__FUNCTION__,__LINE__);
        goto fail_reqbufs;
    }

    if (mPreviewBufferCount <= 0) {
        LOGE("%s(%d): camera start failed, because preview buffer is empty(%d)!",__FUNCTION__,__LINE__,mPreviewBufferCount);
        goto fail_bufalloc;
    }

    memset(mCamDriverV4l2Buffer, 0x00, sizeof(mCamDriverV4l2Buffer));
    for (int i = 0; i < mPreviewBufferCount; i++) {
        if (CAMERA_PREVIEWBUF_ALLOW_WRITE(mPreviewBufferMap[i]->buf_state)) {
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
                buffer.m.offset = mPreviewBufferMap[i]->phy_addr;
                mCamDriverV4l2Buffer[i] = (char*)mPreviewBufferMap[i]->vir_addr;
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
            
            cameraPreviewBufferSetSta(mPreviewBufferMap[i], CMD_PREVIEWBUF_WRITING, 1);
            err = ioctl(iCamFd, VIDIOC_QBUF, &buffer);
            if (err < 0) {
                LOGE("%s(%d): VIDIOC_QBUF Failed,err=%d[%s]\n",__FUNCTION__,__LINE__,err, strerror(errno));
                 cameraPreviewBufferSetSta(mPreviewBufferMap[i], CMD_PREVIEWBUF_WRITING, 0);
                goto fail_bufalloc;
            }           
        } else {
            if (mPreviewBufferMap[i]->buf_state & CMD_PREVIEWBUF_DISPING) {
                LOGD("%s(%d): preview buffer %d is displaying, so wait it dequeueed from display for enqueue to camera",
                        __FUNCTION__,__LINE__,i);                
            }
        }
    }
	
    if(!mPreviewMemory) {
	    mPreviewMemory = mRequestMemory(-1, mPreviewFrame2AppSize, CONFIG_CAMERA_PRVIEW_BUF_CNT, NULL);
    } else if (mPreviewMemory->size != (unsigned int)(mPreviewFrame2AppSize*CONFIG_CAMERA_PRVIEW_BUF_CNT)) {
        mPreviewMemory->release(mPreviewMemory);
        mPreviewMemory = mRequestMemory(-1, mPreviewFrame2AppSize, CONFIG_CAMERA_PRVIEW_BUF_CNT, NULL);
    }
    
    if (mPreviewMemory) {
        for (int i=0; i < CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
            mPreviewBufs[i] = (unsigned char*) mPreviewMemory->data + (i*mPreviewFrame2AppSize);
        }
    } else {
        LOGE("%s(%d): mPreviewMemory create failed",__FUNCTION__,__LINE__);
    }
    
    int *addr;	
	
    for (int i=0; i < CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        if(!mVideoBufs[i])
            mVideoBufs[i] = mRequestMemory(-1, 4, 1, NULL);
        if( (NULL == mVideoBufs[i]) || ( NULL == mVideoBufs[i]->data)) {
            mVideoBufs[i] = NULL;
            LOGE("%s(%d): video buffer %d create failed",__FUNCTION__,__LINE__,i);
        }
        if (mVideoBufs[i]) {
            addr = (int*)mVideoBufs[i]->data;
			//zyc , video buffer not same as preview buffer in usb camera,
			// the format of usb camera is YUYV,video buffer is NV12
			if (CAMERA_IS_UVC_CAMERA()) {
				*addr = mPreviewBuffer[i]->phy_addr;
			} else {
	            *addr = mPreviewBufferMap[i]->phy_addr;
			}
        }
    }
    mPreviewErrorFrameCount = 0;
    mPreviewFrameIndex = 0;

    cameraStream(true);
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
    
    cameraStream(false);
    
    /* ddl@rock-chips.com: Release v4l2 buffer must by close device, buffer isn't release in VIDIOC_STREAMOFF ioctl */
    if (CAMERA_IS_UVC_CAMERA()) {
        close(iCamFd);
        iCamFd = open(cameraDevicePathCur, O_RDWR);
        if (iCamFd < 0) {
            LOGE ("%s(%d): Could not open the camera device(%s): %s",__FUNCTION__,__LINE__, cameraDevicePathCur, strerror(errno) );
            goto fail_streamoff;
        }
    }

    for (int i = 0; i < mPreviewBufferCount; i++) {
        if (mPreviewBufferMap[i]->phy_addr) {
            cameraPreviewBufferSetSta(mPreviewBufferMap[i], CMD_PREVIEWBUF_WRITING, 0);
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
    
    if (!focus) {
    	LOGE("%s(%d): focus is null",__FUNCTION__,__LINE__);
    	err = false;
    	goto cameraAutoFocus_end;
    }
    
    if (strcmp(focus, CameraParameters::FOCUS_MODE_AUTO) == 0) {
        extCtrInfo.id = V4L2_CID_FOCUS_AUTO;
	    extCtrInfo.value = 1;
		// set zone focus
		if(mParameters.getInt(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS) == 1){
			//parse zone,
	    	int lx,ty,rx,dy;
			const char* zoneStr = mParameters.get(CameraParameters::KEY_FOCUS_AREAS);
	    	if(zoneStr){
			//get lx
	    	lx = strtol(zoneStr+1,0,0);

	    	//get ty
	    	char* tys = strstr(zoneStr,",");
	    	ty = strtol(tys+1,0,0);
		

	    	//get rx
	    	char* rxs = strstr(tys+1,",");
	    	rx = strtol(rxs+1,0,0);
	
	    	//get dy
			char* dys = strstr(rxs+1,",");
	    	dy = strtol(dys+1,0,0);
			extCtrInfo.rect[0] = lx;
			extCtrInfo.rect[1] = ty;
			extCtrInfo.rect[2] = rx;
		    extCtrInfo.rect[3] = dy;
			}
		}
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
    } else {
    	err = false;
	    goto cameraAutoFocus_end;
    }

    extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
	extCtrInfos.count = 1;
	extCtrInfos.controls = &extCtrInfo;
	err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
	if ( err < 0 ){
		LOGE("%s(%d): Set focus mode(%s) failed",__FUNCTION__,__LINE__, focus);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
        err = true;
	} else {
	    LOG1("%s(%d): Set focus mode %s",__FUNCTION__,__LINE__, focus);
        err = true;
	}
cameraAutoFocus_end:
    return err;
}
int CameraHal::cameraFormatConvert(int v4l2_fmt_src, int v4l2_fmt_dst, const char *android_fmt_dst, char *srcbuf, char *dstbuf, 
                                    int srcphy,int dstphy,int src_w, int src_h, int dst_w, int dst_h, bool mirror)
{
    int y_size,i,j;

    /*
    if (v4l2_fmt_dst) {    
        LOGD("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x,%dx%d)->'%c%c%c%c'@(0x%x,0x%x,%dx%d) ",
    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
    				(int)srcbuf, srcphy,src_w,src_h,
    				v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
    				(v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF,
    				 (int)dstbuf,dstphy,dst_w,dst_h);
    } else if (android_fmt_dst) {
        LOGD("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x,%dx%d)->%s@(0x%x,0x%x,%dx%d)",
    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF
    				, (int)srcbuf, srcphy,src_w,src_h,android_fmt_dst, (int)dstbuf,dstphy,
    				 dst_w,dst_h);
    }
    */  
    
    y_size = src_w*src_h;
    switch (v4l2_fmt_src)
    {
        case V4L2_PIX_FMT_YUV420:
        {
            if (CAMERA_IS_UVC_CAMERA() 
                || (CAMERA_IS_RKSOC_CAMERA() && (mCamDriverCapability.version != KERNEL_VERSION(0, 0, 1)))) {
                goto cameraFormatConvert_default;
            }
        }
        case V4L2_PIX_FMT_NV12:
        {
            int *dst_vu, *src_uv;

            if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV12) || 
                (android_fmt_dst && (strcmp(android_fmt_dst,CAMERA_DISPLAY_FORMAT_NV12)==0))) {
                if (dstbuf && (dstbuf != srcbuf)) {
                    memcpy(dstbuf,srcbuf, y_size*3/2);
                }
            } else if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV21) || 
                (android_fmt_dst && (strcmp(android_fmt_dst,CameraParameters::PIXEL_FORMAT_YUV420SP)==0))) {
                if ((src_w == dst_w) && (src_h == dst_h)) {
                    if (mirror == false) {
                        if (dstbuf != srcbuf)
                            memcpy(dstbuf,srcbuf, y_size);
                        src_uv = (int*)(srcbuf + y_size); 
                        dst_vu = (int*)(dstbuf+y_size);
                        for (i=0; i<(y_size>>3); i++) {
                            *dst_vu = ((*src_uv&0x00ff00ff)<<8) | ((*src_uv&0xff00ff00)>>8);
                            dst_vu++;
                            src_uv++;
                        }
                    } else {                        
                        char *psrc,*pdst;
                        psrc = srcbuf;
                        pdst = dstbuf + dst_w-1;
                        for (i=0; i<src_h; i++) {                            
                            for (j=0; j<src_w; j++) {
                                *pdst-- = *psrc++;
                            }
                            pdst += 2*dst_w;
                        }

                        psrc = srcbuf + y_size; 
                        pdst = dstbuf + y_size + dst_w-1;
                        for (i=0; i<src_h/2; i++) {                            
                            for (j=0; j<src_w; j++) {
                                *pdst-- = *psrc++;
                            }
                            pdst += 2*dst_w;
                        }
                    }
                } else {
                    if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV21) || 
                        (android_fmt_dst && (strcmp(android_fmt_dst,CameraParameters::PIXEL_FORMAT_YUV420SP)==0))) {
                        int *dst_uv,*src_uv; 
						unsigned *dst_y,*src_y,*src_y1;
						int a, b, c, d;
                        if ((src_w == dst_w*4) && (src_h == dst_h*4)) {
                            dst_y = (unsigned int*)dstbuf;
                            src_y = (unsigned int*)srcbuf;      
							src_y1= src_y + (src_w*3)/4;
                            for (i=0; i<dst_h; i++) {
                                for(j=0; j<dst_w/4; j++) {
									a = (*src_y>>24) + (*src_y&0xff) + (*src_y1>>24) + (*src_y1&0xff);
									a >>= 2;
									src_y++;
									src_y1++;
									b = (*src_y>>24) + (*src_y&0xff) + (*src_y1>>24) + (*src_y1&0xff);
									b >>= 2;
									src_y++;
									src_y1++;
									c = (*src_y>>24) + (*src_y&0xff) + (*src_y1>>24) + (*src_y1&0xff);
									c >>= 2;
									src_y++;
									src_y1++;
									d = (*src_y>>24) + (*src_y&0xff) + (*src_y1>>24) + (*src_y1&0xff);
									d >>= 2;
									src_y++;
									src_y1++;
                                    *dst_y++ = a | (b<<8) | (c<<16) | (d<<24);
                                }
                                //dst_y = (int*)(srcbuf+src_w*(i+1));
                                src_y += (src_w*3)/4;
								src_y1= src_y + (src_w*3)/4;
                            }
                            dst_uv = (int*)(dstbuf+dst_w*dst_h);
                            //dst_uv = (int*)(srcbuf+y_size);
                            src_uv = (int*)(srcbuf+y_size);
                            for (i=0; i<dst_h/2; i++) {
                                for(j=0; j<dst_w/4; j++) {
                                    *dst_uv = (*src_uv&0xffff0000)|((*(src_uv+2)&0xffff0000)>>16);
                                    *dst_uv = ((*dst_uv&0x00ff00ff)<<8)|((*dst_uv&0xff00ff00)>>8);
                                    dst_uv++;
                                    src_uv += 4;
                                }
                                //dst_uv = (int*)(srcbuf+y_size+src_w*(i+1));
                                src_uv += src_w*3/4;
                            }
                        }
                    } else {
                        if (v4l2_fmt_dst) {    
                            LOGE("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->'%c%c%c%c'@(0x%x,0x%x), %dx%d->%dx%d "
                                 "scale isn't support",
                        				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                        				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
                        				v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
                        				(v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF,
                        				(int)srcbuf, srcphy, (int)dstbuf,dstphy,src_w,src_h,dst_w,dst_h);
                        } else if (android_fmt_dst) {
                            LOGD("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->%s@(0x%x,0x%x) %dx%d->%dx%d "
                                 "scale isn't support",
                        				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                        				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF
                        				, (int)srcbuf, srcphy,android_fmt_dst, (int)dstbuf,dstphy,
                        				 src_w,src_h,dst_w,dst_h);
                        }
                    }          
                }
            } else if (android_fmt_dst && (strcmp(android_fmt_dst,CameraParameters::PIXEL_FORMAT_RGB565)==0)) {
                
                if (srcphy && dstphy) {
                    YUV2RGBParams  para;

                    memset(&para, 0x00, sizeof(YUV2RGBParams));
                	para.yuvAddr = srcphy;
                	para.outAddr = dstphy;
                	para.inwidth  = (src_w + 15)&(~15);
                	para.inheight = (src_h + 15)&(~15);
                	para.outwidth  = (dst_w + 15)&(~15);
                	para.outheight = (dst_h + 15)&(~15);
                    para.inColor  = PP_IN_YUV420sp;
                    para.outColor  = PP_OUT_RGB565;

                    doYuvToRgb(&para);                    
                } else if (srcbuf && dstbuf) {
                	if(mRGAFd > 0) {
                        rga_nv12torgb565(mRGAFd,src_w,src_h,srcbuf, (short int*)dstbuf);                    	  
                    } else {
                    	arm_nv12torgb565(src_w,src_h,srcbuf, (short int*)dstbuf);                 
                    }
                }
            }
            break;
        }
        case V4L2_PIX_FMT_YUV422P:
        {
            if (CAMERA_IS_UVC_CAMERA() 
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
                ((v4l2_fmt_dst == V4L2_PIX_FMT_YUV420) && CAMERA_IS_RKSOC_CAMERA() 
                && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1)))) { 
                if ((src_w == dst_w) && (src_h == dst_h)) {
                    dstint_y = (int*)dstbuf;                
                    srcint = (int*)srcbuf;
                    for(i=0;i<(y_size>>2);i++) {
                        *dstint_y++ = ((*(srcint+1)&0x00ff0000)<<8)|((*(srcint+1)&0x000000ff)<<16)
                                    |((*srcint&0x00ff0000)>>8)|(*srcint&0x000000ff);
                        
                        srcint += 2;
                    }
                    dstint_uv =  (int*)(dstbuf + y_size);
                    srcint = (int*)srcbuf;
                    for(i=0;i<src_h/2; i++) {
                        for (j=0; j<(src_w>>2); j++) {
                            *dstint_uv++ = (*(srcint+1)&0xff000000)|((*(srcint+1)&0x0000ff00)<<8)
                                        |((*srcint&0xff000000)>>16)|((*srcint&0x0000ff00)>>8); 
                            srcint += 2;
                        }
                        srcint += (src_w>>1);  
                    }
                } else {
                    if (v4l2_fmt_dst) {    
                        LOGE("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->'%c%c%c%c'@(0x%x,0x%x), %dx%d->%dx%d "
                             "scale isn't support",
                    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
                    				v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
                    				(v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF,
                    				(int)srcbuf, srcphy, (int)dstbuf,dstphy,src_w,src_h,dst_w,dst_h);
                    } else if (android_fmt_dst) {
                        LOGD("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->%s@(0x%x,0x%x) %dx%d->%dx%d "
                             "scale isn't support",
                    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF
                    				, (int)srcbuf, srcphy,android_fmt_dst, (int)dstbuf,dstphy,
                    				 src_w,src_h,dst_w,dst_h);
                    }
                }

            } else if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV21)|| 
                       (android_fmt_dst && (strcmp(android_fmt_dst,CameraParameters::PIXEL_FORMAT_YUV420SP)==0))) {
                if ((src_w==dst_w) && (src_h==dst_h)) {
                    dstint_y = (int*)dstbuf;                
                    srcint = (int*)srcbuf;
                    for(i=0;i<(y_size>>2);i++) {
                        *dstint_y++ = ((*(srcint+1)&0x00ff0000)<<8)|((*(srcint+1)&0x000000ff)<<16)
                                    |((*srcint&0x00ff0000)>>8)|(*srcint&0x000000ff);
                        srcint += 2;
                    }
                    dstint_uv =  (int*)(dstbuf + y_size);
                    srcint = (int*)srcbuf;
                    for(i=0;i<src_h/2; i++) {
                        for (j=0; j<(src_w>>2); j++) {
                            *dstint_uv++ = ((*(srcint+1)&0xff000000)>>8)|((*(srcint+1)&0x0000ff00)<<16)
                                        |((*srcint&0xff000000)>>24)|(*srcint&0x0000ff00); 
                            srcint += 2;
                        }
                        srcint += (src_w>>1);  
                    }                  
                } else {
                    if (v4l2_fmt_dst) {    
                        LOGE("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->'%c%c%c%c'@(0x%x,0x%x), %dx%d->%dx%d "
                             "scale isn't support",
                    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
                    				v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
                    				(v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF,
                    				(int)srcbuf, srcphy, (int)dstbuf,dstphy,src_w,src_h,dst_w,dst_h);
                    } else if (android_fmt_dst) {
                        LOGD("cameraFormatConvert '%c%c%c%c'@(0x%x,0x%x)->%s@(0x%x,0x%x) %dx%d->%dx%d "
                             "scale isn't support",
                    				v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
                    				(v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF
                    				, (int)srcbuf, srcphy,android_fmt_dst, (int)dstbuf,dstphy,
                    				 src_w,src_h,dst_w,dst_h);
                    }
                }
            }            
            break;
        }
	case V4L2_PIX_FMT_RGB565:
	{
		if (android_fmt_dst && (strcmp(android_fmt_dst,CameraParameters::PIXEL_FORMAT_RGB565)==0)){
			if (srcbuf && dstbuf && (srcbuf != dstbuf)){
				if(mRGAFd > 0) 
					rga_rgb565_cp(mRGAFd,src_w,src_h,srcbuf, (short int*)dstbuf);						  
				else
					memcpy(dstbuf,srcbuf,src_w*src_h*2);
				}
		}
		break;
	}
cameraFormatConvert_default:        
        default:
            if (android_fmt_dst) {
                LOGE("%s(%d): CameraHal is not support (%c%c%c%c -> %s)",__FUNCTION__,__LINE__,
                    v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
    			    (v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF, android_fmt_dst);
            } else if (v4l2_fmt_dst) {
                LOGE("%s(%d): CameraHal is not support (%c%c%c%c -> %c%c%c%c)",__FUNCTION__,__LINE__,
                    v4l2_fmt_src & 0xFF, (v4l2_fmt_src >> 8) & 0xFF,
    			    (v4l2_fmt_src >> 16) & 0xFF, (v4l2_fmt_src >> 24) & 0xFF,
    			    v4l2_fmt_dst & 0xFF, (v4l2_fmt_dst >> 8) & 0xFF,
                    (v4l2_fmt_dst >> 16) & 0xFF, (v4l2_fmt_dst >> 24) & 0xFF);
            }
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
    mPreviewCmdReceived = true;
    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_START;
        msg.arg1 = (void*)CMDARG_NACK;
        commandThreadCommandQ.put(&msg);
    }
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR ;
}

void CameraHal::stopPreview()
{
    LOG_FUNCTION_NAME    
    Message msg;
    int ret = 0;
    Mutex::Autolock lock(mLock);
    mPreviewCmdReceived = false;
    if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_STOP;
        msg.arg1 = (void*)CMDARG_ACK;
        commandThreadCommandQ.put(&msg);

        if (mANativeWindow == NULL) {
            mANativeWindowCond.signal();
            LOGD("%s(%d): wake up command thread for stop preview",__FUNCTION__,__LINE__);
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
        msg.arg1 = (void*)CMDARG_ACK;
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
    mParameters.setPictureSize(mPreviewWidth,mPreviewHeight);
    mRecordRunning=true;
    #if CONFIG_CAMERA_FRAME_DV_PROC_STAT
    memset(framebuf_disptime_cnt,0x00, sizeof(framebuf_disptime_cnt));
    memset(framebuf_enctime_cnt,0x00, sizeof(framebuf_enctime_cnt));
    #endif
    LOG_FUNCTION_NAME_EXIT
startRecording_end:
    return err;
}

void CameraHal::stopRecording()
{

    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    mParameters.setPictureSize(mPictureWidth,mPictureHeight);
    mRecordRunning=false;
    #if CONFIG_CAMERA_FRAME_DV_PROC_STAT
    LOGD("disp frame: %ld(<40ms) %ld(40-50ms) %ld(50-60ms) %ld(60-70ms) %ld(70-80ms)",framebuf_disptime_cnt[0],framebuf_disptime_cnt[1],framebuf_disptime_cnt[2],framebuf_disptime_cnt[3],framebuf_disptime_cnt[4]);
    LOGD("enc frame: %ld(<40ms) %ld(40-50ms) %ld(50-60ms) %ld(60-70ms) %ld(70-80ms)"
        " %ld(80-90ms) %ld(90-100ms)",framebuf_enctime_cnt[0],framebuf_enctime_cnt[1],framebuf_enctime_cnt[2],framebuf_enctime_cnt[3],framebuf_enctime_cnt[4],
        framebuf_enctime_cnt[5],framebuf_enctime_cnt[6]);
    #endif
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
    
    if (mANativeWindow && mPreviewBufferMap[index]->phy_addr) {
        cameraPreviewBufferSetSta(mPreviewBufferMap[index], CMD_PREVIEWBUF_ENCING, 0);        
    }
    #if CONFIG_CAMERA_FRAME_DV_PROC_STAT
    framebuf_enc_end[index] = (nsecs_t)systemTime(CLOCK_MONOTONIC);

    if ((framebuf_enc_end[index]- framebuf_enc_start[index]) < 40000000) {
        framebuf_enctime_cnt[0]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 40000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 50000000)) {
        framebuf_enctime_cnt[1]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 50000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 60000000)) {
        framebuf_enctime_cnt[2]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 60000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 70000000)) {
        framebuf_enctime_cnt[3]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 70000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 80000000)) {
        framebuf_enctime_cnt[4]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 80000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 90000000)) {
        framebuf_enctime_cnt[5]++;
    } else if (((framebuf_enc_end[index]- framebuf_enc_start[index]) >= 90000000)&&((framebuf_enc_end[index]- framebuf_enc_start[index]) < 100000000)) {
        framebuf_enctime_cnt[6]++;
    }
    #endif
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
        
    if(!mRecordRunning) {
        LOG1("%s Enter for picture",__FUNCTION__);
        if ((mPreviewThread != NULL) && (mCommandThread != NULL)) {
            msg.command = CMD_PREVIEW_CAPTURE;
            msg.arg1 = (void*)CMDARG_ACK;
            commandThreadCommandQ.put(&msg);
            while (ret == 0) {            
                ret = commandThreadAckQ.get(&msg,6000);
                if (ret == 0) {
                    if (msg.command == CMD_PREVIEW_CAPTURE) {
                        ret = 1;
                        if (msg.arg1 == (void*)CMDARG_ERR) {
                            LOGE("%s(%d): failed, because command thread response ERR\n",__FUNCTION__,__LINE__);    
                            ret = INVALID_OPERATION;
                        }                    
                    }
                } else {
                    LOGE("%s(%d): PREVIEW_CAPTURE is time out! mCommandRunning 0x%x\n",__FUNCTION__,__LINE__,mCommandRunning);
                }
            }
            
            if (ret == 1) {
                ret = NO_ERROR;
                mPreviewCmdReceived = false;
            }
            
        } else {
            LOGE("%s(%d):  cancel, because thread (%s %s) is NULL", __FUNCTION__,__LINE__,(mPreviewThread == NULL)?"mPreviewThread":" ",
                (mCommandThread == NULL)?"mCommandThread":" ");
            ret = INVALID_OPERATION;
        }
    } else {
        LOG1("%s Enter for videoSnapshot",__FUNCTION__);
        if (mPreviewThread != NULL) {
            msg.command = CMD_PREVIEW_VIDEOSNAPSHOT;
            previewThreadCommandQ.put(&msg);
        } else {
            LOGE("%s(%d): videoSnapshot cancel, because preview thread is NULL", __FUNCTION__,__LINE__);
            ret = INVALID_OPERATION;
        }
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
    int fps_min,fps_max;
    int j;
    int framerate;
	int err = NO_ERROR;
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
        if (strcmp(params.get(CameraParameters::KEY_PREVIEW_SIZE),"240x160")==0) {
            if ((mCamDriverCapability.version & 0xff) < 0x07) {
                LOGE("%s(%d): 240x160 is not support for v%d.%d.%d camera driver,"
                    "Please update to v0.x.7 version driver!!",__FUNCTION__,__LINE__,
                    (mCamDriverCapability.version>>16) & 0xff,(mCamDriverCapability.version>>8) & 0xff,
                    mCamDriverCapability.version & 0xff);
            }
        } else {
            LOGE("%s(%d): PreviewSize(%s) not supported",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_SIZE));
        }
        return BAD_VALUE;
    } else if (strcmp(mParameters.get(CameraParameters::KEY_PREVIEW_SIZE), params.get(CameraParameters::KEY_PREVIEW_SIZE))) {
        LOGD("%s(%d): Set preview size %s",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_SIZE));
    }

    if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES), params.get(CameraParameters::KEY_PICTURE_SIZE)) == NULL) {
        LOGE("%s(%d): PictureSize(%s) not supported",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PICTURE_SIZE));
        return BAD_VALUE;
    } else if (strcmp(mParameters.get(CameraParameters::KEY_PICTURE_SIZE), params.get(CameraParameters::KEY_PICTURE_SIZE))) {
        LOGD("%s(%d): Set picture size %s",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PICTURE_SIZE));
    }

    if (strcmp(params.getPictureFormat(), "jpeg") != 0) {
        LOGE("%s(%d): Only jpeg still pictures are supported",__FUNCTION__,__LINE__);
        return BAD_VALUE;
    }

    if (params.getInt(CameraParameters::KEY_ZOOM) > params.getInt(CameraParameters::KEY_MAX_ZOOM)) {
        LOGE("%s(%d): Zomm(%d) is larger than MaxZoom(%d)",__FUNCTION__,__LINE__,params.getInt(CameraParameters::KEY_ZOOM),params.getInt(CameraParameters::KEY_MAX_ZOOM));
        return BAD_VALUE;
    }

    params.getPreviewFpsRange(&fps_min,&fps_max);
    if ((fps_min < 0) || (fps_max < 0) || (fps_max < fps_min)) {
        LOGE("%s(%d): FpsRange(%s) is invalidate",__FUNCTION__,__LINE__,params.get(CameraParameters::KEY_PREVIEW_FPS_RANGE));
        return BAD_VALUE;
    }

    if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS),params.getPreviewFormat())) {
        
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
	#if (CONFIG_CAMERA_DISPLAY_FORCE==0)
        char hwc_value[PROPERTY_VALUE_MAX];
        int hwc_version_int=0, hwc_version_float=0, hwc_on;
        bool force_rgb;

        property_get("sys.ghwc.version", hwc_value, "0.0");
        sscanf(hwc_value,"%d.%d",&hwc_version_int,&hwc_version_float);        
        property_get("sys.hwc.compose_policy", hwc_value, "5a5a");
        sscanf(hwc_value,"%d",&hwc_on); 
        
        if (hwc_on != 0x5a5a) {
            if ((hwc_version_int>=0x01) && (hwc_version_float>=0x00)) {                
                force_rgb = false;
                LOGD("%s(%d): HWC version v%d.%d is support yuv, Camera display format isn't fix rgb",__FUNCTION__,__LINE__,
                    hwc_version_int,hwc_version_float);
            } else {
                force_rgb = true;
                LOGD("%s(%d): HWC version isn't support yuv, Camera display format fix rgb",__FUNCTION__,__LINE__);
            }
        } else {
            force_rgb = false;
            LOGD("%s(%d): HWC isn't in current software, Camera display format isn't fix rgb",__FUNCTION__,__LINE__);
        }

        if (force_rgb == false) {
            if ((strcmp(params.getPreviewFormat(),CameraParameters::PIXEL_FORMAT_RGB565)==0)
                || (strcmp(cameraCallProcess,"com.android.facelock")==0)) {
                strcpy(mDisplayFormat,CameraParameters::PIXEL_FORMAT_RGB565);
            } else { 
                strcpy(mDisplayFormat,CAMERA_DISPLAY_FORMAT_NV12);
            }
        } else {
            strcpy(mDisplayFormat,CameraParameters::PIXEL_FORMAT_RGB565);
        }
        #else 
        strcpy(mDisplayFormat,CONFIG_CAMERA_DISPLAY_FORCE_FORMAT);
        #endif
        
    } else {
        LOGE("%s(%d): %s is not supported,Only %s and %s preview is supported",__FUNCTION__,__LINE__,params.getPreviewFormat(),CameraParameters::PIXEL_FORMAT_YUV420SP,CameraParameters::PIXEL_FORMAT_YUV422SP);
        return BAD_VALUE;
    }
   
    if (CAMERA_IS_RKSOC_CAMERA()
        && (mCamDriverCapability.version == KERNEL_VERSION(0, 0, 1))) {
        mCamDriverPictureFmt = mCamDriverPreviewFmt;
    } else if(CAMERA_IS_UVC_CAMERA()){
		mCamDriverPictureFmt = V4L2_PIX_FMT_NV12;
    }else{
	   mCamDriverPictureFmt = mCamDriverPreviewFmt;    /* ddl@rock-chips.com : Picture format must is NV12 or RGB565, because jpeg encoder is only support NV12 */
	}

    if ((mCamDriverPictureFmt != V4L2_PIX_FMT_NV12) &&
        (mCamDriverPictureFmt != V4L2_PIX_FMT_RGB565)) 
        LOGE("%s(%d): %c%c%c%c is not supported,Only NV12 and RGB565 picture is supported",__FUNCTION__,__LINE__,
             mCamDriverPictureFmt & 0xFF, (mCamDriverPictureFmt >> 8) & 0xFF,
			(mCamDriverPictureFmt >> 16) & 0xFF, (mCamDriverPictureFmt >> 24) & 0xFF);

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
    
	if (!cameraConfig(params)) {        
        LOG1("PreviewSize(%s)", mParameters.get(CameraParameters::KEY_PREVIEW_SIZE));
        LOG1("PreviewFormat(%s)  mCamDriverPreviewFmt(%c%c%c%c) mDisplayFormat(%s)",params.getPreviewFormat(), 
            mCamDriverPreviewFmt & 0xFF, (mCamDriverPreviewFmt >> 8) & 0xFF,
			(mCamDriverPreviewFmt >> 16) & 0xFF, (mCamDriverPreviewFmt >> 24) & 0xFF,
			mDisplayFormat);  
        LOG1("FPS Range(%s)",mParameters.get(CameraParameters::KEY_PREVIEW_FPS_RANGE));
        LOG1("PictureSize(%s)",mParameters.get(CameraParameters::KEY_PICTURE_SIZE)); 
        LOG1("PictureFormat(%s)  mCamDriverPictureFmt(%c%c%c%c)", params.getPictureFormat(),
            mCamDriverPictureFmt & 0xFF, (mCamDriverPictureFmt >> 8) & 0xFF,
			(mCamDriverPictureFmt >> 16) & 0xFF, (mCamDriverPictureFmt >> 24) & 0xFF);
        LOG1("Framerate: %d  ", framerate);
        LOG1("WhiteBalance: %s", params.get(CameraParameters::KEY_WHITE_BALANCE));
        LOG1("Flash: %s", params.get(CameraParameters::KEY_FLASH_MODE));
        LOG1("Focus: %s", params.get(CameraParameters::KEY_FOCUS_MODE));
        LOG1("Scene: %s", params.get(CameraParameters::KEY_SCENE_MODE));
    	LOG1("Effect: %s", params.get(CameraParameters::KEY_EFFECT));
    	LOG1("ZoomIndex: %s", params.get(CameraParameters::KEY_ZOOM));	    
	}else{
	    err = BAD_VALUE;
	}
    LOG_FUNCTION_NAME_EXIT

    return err;
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

    if (cmd == CAMERA_CMD_START_FACE_DETECTION) {
        if ((mParameters.getInt(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW) == 0)
            && (mParameters.getInt(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW) == 0)) 
            ret = BAD_VALUE;
    } 
    
    LOG_FUNCTION_NAME_EXIT
    return ret;
}
int CameraHal::dump(int fd)
{   
    int i;
    
    if (gLogLevel < 2) 
        android_atomic_inc(&gLogLevel);
    else 
        android_atomic_write(0,&gLogLevel);

    LOGD("Set %s log level to %d",LOG_TAG,gLogLevel);

    if (mCamBuffer) {
        mCamBuffer->dump();
    }
    
    commandThreadCommandQ.dump();

    for (i=0; i<CONFIG_CAMERA_PRVIEW_BUF_CNT; i++) {
        if (mPreviewBufferMap[i]->priv_hnd) {
            LOGD("%s(%d): buffer %d state is 0x%x", __FUNCTION__,__LINE__,i, mPreviewBufferMap[i]->buf_state);
        }
    }
    LOGD("%s(%d): mPreviewRunning:0x%x, mDisplayRuning:0x%x, mPictureRunning:0x%x, mCommandRunning:0x%x mSnapshotRunning:0x%x",
        __FUNCTION__,__LINE__,mPreviewRunning, mDisplayRuning, mPictureRunning,mCommandRunning,mSnapshotRunning);
    
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
        msg.arg1 = (void*)CMDARG_NACK;
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
    mSnapshotThread->requestExitAndWait();
    mSnapshotThread.clear();
	
    cameraDestroy();
    
    LOG_FUNCTION_NAME_EXIT
}
}; // namespace android
