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
#ifndef ANDROID_HARDWARE_CAMERA_HARDWARE_H
#define ANDROID_HARDWARE_CAMERA_HARDWARE_H

#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <utils/Log.h>
#include <utils/threads.h>
#include <cutils/properties.h>
#include <cutils/atomic.h>
#include <linux/version.h>
#include <linux/videodev2.h> 
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <utils/threads.h>
#include <ui/GraphicBufferAllocator.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/GraphicBuffer.h>
#include <system/window.h>
#include <camera/Camera.h>
#include <hardware/camera.h>
#include <camera/CameraParameters.h>
#include "Semaphore.h"
#if defined(TARGET_RK29)
#include <linux/android_pmem.h>
#endif
#include "MessageQueue.h"
#include "../jpeghw/release/encode_release/hw_jpegenc.h"
#include "../jpeghw/release/encode_release/rk29-ipp.h"
#include "../libon2/vpu_global.h"

#include "CameraHal_Module.h"
#include "common_type.h"

#include "vpu_global.h"
#include "vpu_mem_pool.h"

/* 
*NOTE: 
*       CONFIG_CAMERA_INVALIDATE_RGA is debug macro, 
*    CONFIG_CAMERA_INVALIDATE_RGA must equal to 0 in official version.     
*/
#define CONFIG_CAMERA_INVALIDATE_RGA    0


#if defined(TARGET_RK30) && (defined(TARGET_BOARD_PLATFORM_RK30XX) || (defined(TARGET_BOARD_PLATFORM_RK2928)))
#include "../libgralloc_ump/gralloc_priv.h"
#if (CONFIG_CAMERA_INVALIDATE_RGA==0)
#include <hardware/rga.h>
#endif
#elif defined(TARGET_RK30) && defined(TARGET_BOARD_PLATFORM_RK30XXB)
#include <hardware/hal_public.h>
#include <hardware/rga.h>
#elif defined(TARGET_RK29)
#include "../libgralloc/gralloc_priv.h"
#endif


#include "CameraHal_Mem.h"
#include "CameraHal_Tracer.h"

extern "C" int getCallingPid();
extern "C" void callStack();
extern "C" int cameraPixFmt2HalPixFmt(const char *fmt);
extern "C" void arm_nv12torgb565(int width, int height, char *src, short int *dst,int dstbuf_w);
extern "C" int rga_nv12torgb565(int src_width, int src_height, char *src, short int *dst, 
                                int dstbuf_width,int dst_width,int dst_height);
extern "C" int rk_camera_yuv_scale_crop_ipp(int v4l2_fmt_src, int v4l2_fmt_dst, 
	            int srcbuf, int dstbuf,int src_w, int src_h,int dst_w, int dst_h,bool rotation_180);
extern "C"  int YData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w);
extern "C"  int UVData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w);
extern "C"  int YuvData_Mirror_Flip(int v4l2_fmt_src, char *pdata, char *pline_tmp, int w, int h);
extern "C" int YUV420_rotate(const unsigned char* srcy, int src_stride,  unsigned char* srcuv,
                   unsigned char* dsty, int dst_stride, unsigned char* dstuv,
                   int width, int height,int rotate_angle);
extern "C"  int arm_camera_yuv420_scale_arm(int v4l2_fmt_src, int v4l2_fmt_dst, 
									char *srcbuf, char *dstbuf,int src_w, int src_h,int dst_w, int dst_h,bool mirror,int zoom_value);
extern "C" char* getCallingProcess();

extern "C" void arm_yuyv_to_nv12(int src_w, int src_h,char *srcbuf, char *dstbuf);

extern "C" int cameraFormatConvert(int v4l2_fmt_src, int v4l2_fmt_dst, const char *android_fmt_dst, 
							char *srcbuf, char *dstbuf,int srcphy,int dstphy,int src_size,
							int src_w, int src_h, int srcbuf_w,
							int dst_w, int dst_h, int dstbuf_w,
							bool mirror);
							
extern "C" int rga_nv12_scale_crop(int src_width, int src_height, char *src, short int *dst, int dstbuf_width,int dst_width,int dst_height,int zoom_val,bool mirror,bool isNeedCrop);

extern rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];

namespace android {

/*
*       CAMERA HAL VERSION NOTE
*
*v0.2.0x00: add continues focus/auto focus, log level;
*v0.3.0x00: add usb camera support;
*v0.4.0x00: add flash;
*v0.5.0x00: check whether focus and flash is available
*v0.6.0x00: sync camerahal
*v0.7.0x00: 
		(1)add lock to protect mFrameInfoArray ,fix mFrameInfoArray corrupt
		(2)remove compile warnings
		(3)sync mid v0.6.01
*v0.8.0x00:
		support fake camera
*v0.9.0x00
		do nothing if mFrameInfoArray has been cleared when return frame.
*v0.0x0a.0x00:
*       1) support no cam_board.xml;
*v0.0x0b.0x00:
*       1) 	support setVideoSize function;
*       2) add cameraConfig function;
*       3) sync mid v0.0x0a.01
*v0.0x0c.0x00
        1) support mi output yuv420 when sensor output yuv422
*v0.0x0d.0x00:
*       1) add continues video focus, but is fixed focus for video recording;
*       2) stop continus af before af oneShot, marvin isp af afOnshot and afProcessFrame may be conflict;
*v0.0x0e.0x00:
*       1) support digital zoom
*v0.f.0:
*       this version sync below version:
*    v0.d.1:
*       1) add continues video focus, but is fixed focus for video recording;
*    v0.d.2:
*       1) invalidate <= 1080p picture size, because the flash feature is not yet perfect;
*    v0.d.3:
*       1) stop focus and set max focus in disconnect camera for ov8825 vcm noise;
*       2) set max focus in continues video focus;
*v0.10.0:
        1) optimize flash unit
        2) fix zoom erro in cameraConfig function
  v0.0x11.0
  	 support ov8858 ov13850 sensor dirver for preview
*v0.0x12.0
	    1) add flash trig pol control
*v0.0x13.0:
        merge source code frome mid,include following version:
        v0.d.0x04:
               1) support mirror frame which sended by mDataCb and from front camera,
                    config by CONFIG_CAMERA_FRONT_MIRROR_MDATACB;
        v0.d.0x05:
               1) add wechat in CONFIG_CAMERA_FRONT_MIRROR_MDATACB_APK 
        v0.d.0x06:
               1) add support picture size to increase the speed of taking capture in DV 
        v0.d.0x07:
               1) support fake camera 
        v0.d.0x08:
               1) fill the flash info into exif 
*v0.0x14.0:
        1) zoom feature can config in board xml
*v0.0x15.0:
        1) support flash auto mode
        2) uvc support iommu 
        3) judge whether need to enable flash in preview status.
        4) flash pol must be initialized in setparameter,fix it
*v0.0x16.0:
	  1)add interface of getting awb param from isp.
	  2)use makernote tag as customer infomation.
*v0.0x17.0:
	  1) modify the timing of getting awb param from isp, move it to CameraIspAdapter::bufferCb.
*v0.0x18.0:
*     1) match for Libisp v0.0x15.0(af speed up and add vcm current setting);
*v0.0x19.0:
      1) substitute rga for arm to do nv12 scale and crop to improve performance
      2) support continuos shot feature
      3) digital zoom by rga,prevent from decresing framerate,so front cam can support digital zoom well now.
*v0.0x20.0:
      1) add version info of getting from sensor XML file to makernote.
*v0.0x21.0:
      1) strcmp func fatal erro in funtion CameraIspAdapter::isNeedToEnableFlash if no KEY_SUPPORTED_FLASH_MODES supported,fix it
*v0.0x22.0:
*     1) autofocus and afListenThread and afProcessFrame(in Libisp) may be deadlock;
*     2) startAfContinous and stopAf may be frequently, af cmd queue is full. Main thread hold;
*     3) fix digital zoom by isp, must by rga;
*     4) encProcessThread / bufferCb / setParameters(changeVideoSize->StopPreview)maybe deadlock;
*     5) digital zoom by rga, needn't reconfig size;
*v0.0x23.0:
      1) needn't disable CAMERA_MSG_SHUTTER during continuos pic taken.
*v0.0x24.0:
      1) fix compile warnning due to  last commit
*v0.0x25.0:
      1) support isp tunning 
*v0.0x26.0:
      1) mIsSendToTunningTh dosen't initialize in last commit,fix it
*v0.0x27.0
      1) merge source code frome mid,include following version:
         v0.0x22.2:
              1) add support 500w ;
      2) autofocus and take picture failed , because fix startAfOneShot failed;
*v0.0x28.0
*     1) flash mode can config in board xml.
*     2) support MWB/ME.
*     3) add func getFlashStatus, fix wrong flash status in jpeg exif info.
*v0.0x29.0
*     1) modify isp tunning func
*     2) zoom is not available when pic taken if pic size less than 5M,fix it  
*v0.0x2a.0
*     1) fix CTS failed items as follows :
        testPreviewCallback,testPreviewFormats,testPreviewFpsRange, testPreviewPictureSizesCombination
*v0.0x2b.0
*     1) fix CTS failed items as follows :
        testInvalidParameters,testParameters,testSceneMode,testImmediateZoom
*v0.0x2c.0
*     1) fix CTS failed items as follows :
        testFaceDetection,testFocusAreas
*v0.0x2d.0
*     1) fix CTS failed items as follows :
        testJpegExif,testVideoSnapshot
*v0.0x2e.0
*     1) when preview stopped ,preview cb should be stopped ,or may cause CTS faile  
*v0.0x2f.0
*     1) add sensor drv version
*v0.0x30.0
*     1) check illumination is support or not by chkAwbIllumination for MWB;
*v0.0x31.0
*     1) invalidate ME for soc sensor;
*v0.0x32.0
*     1) lock aec when take picture;
*     2) add check sharpness for low illumnation;
*v0.0x33.0
*     1) support iommu;
      2) merge source code frome mid,include following version:
	    *v0.0x2d.1
		*	  1) fix rk312x compile warning
		*v0.0x2d.2
		*	  1) move camera config file from device to hardware 
		*v0.0x2d.3
		*	  1) fix last commit bug 
		        testJpegExif,testVideoSnapshot
		*v0.0x2e.3
		*     1) when preview stopped ,preview cb should be stopped ,or may cause CTS faile  
*v0.0x34.0
*	  1)add awb stable
*     2)add fov parameters
*     3)modiy to fit more than 2 camera sensor board info
*v0.0x35.0
*     1)  file is opened in func ispTuneStoreBuffer  but not been closed,fix it;
*v0.0x36.0
*     1) modify fov format from int to float
*v0.0x37.0
*     1) set mwb when capture picture with changing resolution.
*v0.0x38.0
*     1) merge source code frome mid,include following version:
*       *v0.0x36.1
        *     1) modify to pass cts verifier FOV 
        *v0.0x36.2
        *     1) modify to pass cts verifier orientation 
        *     2) use arm scale when display,because VOIP need NV21 
        *v0.0x36.3
        *     1) support new jpeg vpumalloc,size is set by cameraHal 
*     2) fix initDefaultParameters for previewsize/picturesize;
*v0.0x39.0:
*     1) setMe is invalidate when soc sensor;
*v0.0x3a.0:
*     1) enum sensor resolution and check for DV media_profiles.xml in CheckSensorSupportDV;
*v0.0x3a.1:
	  1) support rk312x preview and picture taken .
*/
#define CONFIG_CAMERAHAL_VERSION KERNEL_VERSION(0, 0x3a, 0x01)

/*  */
#define CAMERA_DISPLAY_FORMAT_YUV420P   CameraParameters::PIXEL_FORMAT_YUV420P
#define CAMERA_DISPLAY_FORMAT_YUV420SP   CameraParameters::PIXEL_FORMAT_YUV420SP
#define CAMERA_DISPLAY_FORMAT_RGB565     CameraParameters::PIXEL_FORMAT_RGB565
#define CAMERA_DISPLAY_FORMAT_NV12       "nv12"
/* 
*NOTE: 
*       CONFIG_CAMERA_DISPLAY_FORCE and CONFIG_CAMERA_DISPLAY_FORCE_FORMAT is debug macro, 
*    CONFIG_CAMERA_DISPLAY_FORCE must equal to 0 in official version.     
*/
#define CONFIG_CAMERA_DISPLAY_FORCE     1
#define CONFIG_CAMERA_DISPLAY_FORCE_FORMAT CAMERA_DISPLAY_FORMAT_RGB565

#define CONFIG_CAMERA_SINGLE_SENSOR_FORCE_BACK_FOR_CTS   0
#define CONFIG_CAMERA_FRAME_DV_PROC_STAT    0
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB  1
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB_ALL  0
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB_APK  "<com.skype.raider>,<com.yahoo.mobile.client.andro>,<com.tencent.mm>"
#define CONFIG_CAMERA_SETVIDEOSIZE	0

#define CONFIG_CAMERA_PREVIEW_BUF_CNT 4
#define CONFIG_CAMERA_DISPLAY_BUF_CNT		4
#define CONFIG_CAMERA_VIDEO_BUF_CNT 4
#define CONFIG_CAMERA_VIDEOENC_BUF_CNT		4

#define CONFIG_CAMERA_UVC_INVAL_FRAMECNT    5
#define CONFIG_CAMERA_ORIENTATION_SKYPE     0
#define CONFIG_CAMERA_FRONT_ORIENTATION_SKYPE     0
#define CONFIG_CAMERA_BACK_ORIENTATION_SKYPE      0

#define CONFIG_CAMERA_FRONT_PREVIEW_FPS_MIN    3000        // 3fps
#define CONFIG_CAMERA_FRONT_PREVIEW_FPS_MAX    40000        //30fps
#define CONFIG_CAMERA_BACK_PREVIEW_FPS_MIN     3000        
#define CONFIG_CAMERA_BACK_PREVIEW_FPS_MAX     40000

#define CAMERAHAL_VERSION_PROPERTY_KEY                  "sys_graphic.cam_hal.ver"
#define CAMERAHAL_CAMSYS_VERSION_PROPERTY_KEY           "sys_graphic.cam_drv_camsys.ver"
#define CAMERAHAL_V4L2_VERSION_PROPERTY_KEY             "sys_graphic.cam_dri_v4l2.ver"
#define CAMERAHAL_LIBISP_PROPERTY_KEY                   "sys_graphic.cam_libisp.ver"
#define CAMERAHAL_ISI_PROPERTY_KEY                      "sys_graphic.cam_isi.ver"
#define CAMERAHAL_CAMBOARDXML_PARSER_PROPERTY_KEY       "sys_graphic.cam_camboard.ver"
#define CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY              "sys_graphic.cam_trace"
#define CAMERAHAL_USER_PREVIEW_REQUEST_KEY              "sys_graphic.cam_preview_req"


#define CAMERA_PMEM_NAME                     "/dev/pmem_cam"
#define CAMERA_DRIVER_SUPPORT_FORMAT_MAX   32

#define RAW_BUFFER_SIZE_8M         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ? 0xF40000:0xB70000)
#define RAW_BUFFER_SIZE_5M         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ? 0x9A0000:0x740000)
#define RAW_BUFFER_SIZE_3M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ?0x600000 :0x480000)
#define RAW_BUFFER_SIZE_2M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ?0x3A0000 :0x2c0000)
#define RAW_BUFFER_SIZE_1M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565)? 0x180000 :0x1d0000)
#define RAW_BUFFER_SIZE_0M3         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565)?0x150000 :0x100000)

#define JPEG_BUFFER_SIZE_8M          0x700000
#define JPEG_BUFFER_SIZE_5M          0x400000
#define JPEG_BUFFER_SIZE_3M          0x300000
#define JPEG_BUFFER_SIZE_2M          0x300000
#define JPEG_BUFFER_SIZE_1M          0x200000
#define JPEG_BUFFER_SIZE_0M3         0x100000

#define OPTIMIZE_MEMORY_USE
#define VIDEO_ENC_BUFFER            0x151800 
#define FILTER_FRAME_NUMBER (3)
#define IOMMU_ENABLED   (0)

#define V4L2_BUFFER_MAX             32
#define V4L2_BUFFER_MMAP_MAX        16
#define PAGE_ALIGN(x)   (((x) + 0xFFF) & (~0xFFF)) // Set as multiple of 4K

#define KEY_CONTINUOUS_PIC_NUM  "rk-continous-pic-num"
#define KEY_CONTINUOUS_PIC_INTERVAL_TIME "rk-continous-pic-interval-time"
#define KEY_CONTINUOUS_SUPPORTED    "rk-continous-supported"
#define KEY_PREVIEW_W_FORCE  "rk-previwe-w-force"
#define KEY_PREVIEW_H_FORCE  "rk-previwe-h-force"


#define CAMHAL_GRALLOC_USAGE GRALLOC_USAGE_HW_TEXTURE | \
                             GRALLOC_USAGE_HW_RENDER | \
                             GRALLOC_USAGE_SW_WRITE_OFTEN | \
                             GRALLOC_USAGE_SW_READ_OFTEN /*| \
                             GRALLOC_USAGE_SW_WRITE_MASK| \
                             GRALLOC_USAGE_SW_READ_RARELY*/ 
#define CAMERA_IPP_NAME                  "/dev/rk29-ipp"




#if defined(TARGET_BOARD_PLATFORM_RK30XX) || defined(TARGET_RK29) || defined(TARGET_BOARD_PLATFORM_RK2928)                         
    #define NATIVE_HANDLE_TYPE             private_handle_t
    #define PRIVATE_HANDLE_GET_W(hd)       (hd->width)    
    #define PRIVATE_HANDLE_GET_H(hd)       (hd->height)    
#elif defined(TARGET_BOARD_PLATFORM_RK30XXB)               
    #define NATIVE_HANDLE_TYPE             IMG_native_handle_t
    #define PRIVATE_HANDLE_GET_W(hd)       (hd->iWidth)    
    #define PRIVATE_HANDLE_GET_H(hd)       (hd->iHeight)    
#endif

#define RK_VIDEOBUF_CODE_CHK(rk_code)		((rk_code&(('R'<<24)|('K'<<16)))==(('R'<<24)|('K'<<16)))


class FrameProvider
{
public:
   virtual int returnFrame(int index,int cmd)=0;
   virtual ~FrameProvider(){};
   FrameProvider(){};
   FramInfo_s mPreviewFrameInfos[CONFIG_CAMERA_PREVIEW_BUF_CNT];
};

typedef struct rk_buffer_info {
    Mutex* lock;
    int phy_addr;
    int vir_addr;
    int buf_state;
} rk_buffer_info_t;

class BufferProvider{
public:
    int createBuffer(int count,int perbufsize,buffer_type_enum buftype);
    int freeBuffer();
    virtual int setBufferStatus(int bufindex,int status,int cmd=0);
    virtual int getOneAvailableBuffer(int *buf_phy,int *buf_vir);
    int getBufferStatus(int bufindex);
    int getBufCount();
    int getBufPhyAddr(int bufindex);
    int getBufVirAddr(int bufindex);
    int flushBuffer(int bufindex);
    BufferProvider(MemManagerBase* memManager):mBufInfo(NULL),mCamBuffer(memManager){}
    virtual ~BufferProvider(){mCamBuffer = NULL;mBufInfo = NULL;}
protected:
    rk_buffer_info_t* mBufInfo;
    int mBufCount;
    buffer_type_enum mBufType;
    MemManagerBase* mCamBuffer;
};

//preview buffer 管理
class PreviewBufferProvider:public BufferProvider
{
public:

    enum PREVIEWBUFSTATUS {
        CMD_PREVIEWBUF_DISPING = 0x01,
        CMD_PREVIEWBUF_VIDEO_ENCING = 0x02,
        CMD_PREVIEWBUF_SNAPSHOT_ENCING = 0x04,
        CMD_PREVIEWBUF_DATACB = 0x08,
        CMD_PREVIEWBUF_WRITING = 0x10,
    };
    
#define CAMERA_PREVIEWBUF_ALLOW_DISPLAY(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_ENC(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_ENC_PICTURE(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_DATA_CB(a)  ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_WRITE(a)   ((a&(CMD_PREVIEWBUF_DISPING|CMD_PREVIEWBUF_VIDEO_ENCING|CMD_PREVIEWBUF_SNAPSHOT_ENCING|CMD_PREVIEWBUF_DATACB|CMD_PREVIEWBUF_WRITING))==0x00)

    virtual int setBufferStatus(int bufindex,int status,int cmd);
    
    PreviewBufferProvider(MemManagerBase* memManager):BufferProvider(memManager){}
    ~PreviewBufferProvider(){}
};


class DisplayAdapter;
class AppMsgNotifier;
typedef struct cameraparam_info cameraparam_info_s;
//diplay buffer 由display adapter类自行管理。

/* mjpeg decoder interface in libvpu.*/
typedef void* (*getMjpegDecoderFun)(void);
typedef void (*destroyMjpegDecoderFun)(void* jpegDecoder);

typedef int (*initMjpegDecoderFun)(void* jpegDecoder);
typedef int (*deInitMjpegDecoderFun)(void* jpegDecoder);

typedef int (*mjpegDecodeOneFrameFun)(void * jpegDecoder,uint8_t* aOutBuffer, uint32_t *aOutputLength,
        uint8_t* aInputBuf, uint32_t* aInBufSize, uint32_t out_phyaddr);

typedef struct mjpeg_interface {
    void*                       decoder;
    int                         state;
    
    getMjpegDecoderFun          get;
    destroyMjpegDecoderFun      destroy;
    initMjpegDecoderFun         init;
    deInitMjpegDecoderFun       deInit;
    mjpegDecodeOneFrameFun      decode;
} mjpeg_interface_t;

/*************************
CameraAdapter 负责与驱动通信，且为帧数据的提供者，为display及msgcallback提供数据。
***************************/
class CameraAdapter:public FrameProvider
{
public:
    CameraAdapter(int cameraId);
    virtual ~CameraAdapter();

    void setImageAllFov(bool sw){mImgAllFovReq=sw;}
    DisplayAdapter* getDisplayAdapterRef(){return mRefDisplayAdapter;}
    void setDisplayAdapterRef(DisplayAdapter& refDisplayAdap);
    void setEventNotifierRef(AppMsgNotifier& refEventNotify);
    void setPreviewBufProvider(BufferProvider* bufprovider);
    CameraParameters & getParameters();
    virtual int getCurPreviewState(int *drv_w,int *drv_h);
	virtual int getCurVideoSize(int *video_w, int *video_h);
    virtual bool isNeedToRestartPreview();
    int getCameraFd();
   int initialize();
    
    virtual status_t startPreview(int preview_w,int preview_h,int w, int h, int fmt,bool is_capture);
    virtual status_t stopPreview();
   // virtual int initialize() = 0;
    virtual int returnFrame(int index,int cmd);
    virtual int setParameters(const CameraParameters &params_set,bool &isRestartValue);
    virtual void initDefaultParameters(int camFd);
    virtual status_t autoFocus();
    virtual status_t cancelAutoFocus();

    virtual void dump(int cameraId);
	virtual void getCameraParamInfo(cameraparam_info_s &paraminfo);
	virtual bool getFlashStatus();
    virtual int selectPreferedDrvSize(int *width,int * height,bool is_capture){ return 0;}
protected:
    //talk to driver
    virtual int cameraCreate(int cameraId);
    virtual int cameraDestroy();
    
    virtual int cameraSetSize(int w, int h, int fmt, bool is_capture); 
    virtual int cameraStream(bool on);
    virtual int cameraStart();
    virtual int cameraStop();
    //dqbuf
    virtual int getFrame(FramInfo_s** frame); 
    virtual int cameraAutoFocus(bool auto_trig_only);

    //qbuf
 //   virtual status_t fillThisBuffer();

    //define  the frame info ,such as w, h ,fmt ,dealflag(preview callback ? display ? video enc ? picture?)
    virtual int reprocessFrame(FramInfo_s* frame);
    virtual int adapterReturnFrame(int index,int cmd);

private:
    class CameraPreviewThread :public Thread
    {
        //deque 到帧后根据需要分发给DisplayAdapter类及EventNotifier类。
        CameraAdapter* mPreivewCameraAdapter;
    public:
        CameraPreviewThread(CameraAdapter* adapter)
            : Thread(false), mPreivewCameraAdapter(adapter) { }

        virtual bool threadLoop() {
            mPreivewCameraAdapter->previewThread();

            return false;
        }
    };


protected:    
    virtual void previewThread();
protected:
    DisplayAdapter* mRefDisplayAdapter;
    AppMsgNotifier* mRefEventNotifier;
    sp<CameraPreviewThread> mCameraPreviewThread;
    int mPreviewRunning;
	int mPictureRunning;
    BufferProvider* mPreviewBufProvider;
    int mCamDrvWidth;
    int mCamDrvHeight;
    int mCamPreviewH ;
    int mCamPreviewW ;
    int mVideoWidth;
    int mVideoHeight;
    bool mImgAllFovReq;

    unsigned int mCamDriverSupportFmt[CAMERA_DRIVER_SUPPORT_FORMAT_MAX];
    enum v4l2_memory mCamDriverV4l2MemType;

    unsigned int mCamDriverPreviewFmt;
    struct v4l2_capability mCamDriverCapability;

    int mPreviewErrorFrameCount;
    int mPreviewFrameIndex;

    Mutex mCamDriverStreamLock;
    bool mCamDriverStream;

    bool camera_device_error;

    CameraParameters mParameters;
    unsigned int CameraHal_SupportFmt[6];

    char *mCamDriverV4l2Buffer[V4L2_BUFFER_MAX];
    unsigned int mCamDriverV4l2BufferLen;

    mjpeg_interface_t mMjpegDecoder;
    void* mLibstageLibHandle;

    int mZoomVal;
    int mZoomMin;
    int mZoomMax;
    int mZoomStep;
    
    int mCamFd;
    int mCamId;
};

//soc camera adapter
class CameraSOCAdapter: public CameraAdapter
{
public:
    CameraSOCAdapter(int cameraId);
    virtual ~CameraSOCAdapter();

    /*********************
    talk to driver 
    **********************/
    //parameters
    virtual int setParameters(const CameraParameters &params_set,bool &isRestartValue);
    virtual void initDefaultParameters(int camFd);
    virtual int cameraAutoFocus(bool auto_trig_only);
    virtual int selectPreferedDrvSize(int *width,int * height,bool is_capture);
    
private:   
    static unsigned int mFrameSizesEnumTable[][2];
    
    typedef struct frameSize_s{
        unsigned int width;
        unsigned int height;
        unsigned int fmt;
        int framerate;
    }frameSize_t;
    Vector<frameSize_t> mFrameSizeVector;
    
    int cameraFramerateQuery(unsigned int format, unsigned int w, unsigned int h, int *min, int *max);
    int cameraFpsInfoSet(CameraParameters &params);
    int cameraConfig(const CameraParameters &tmpparams,bool isInit,bool &isRestartValue);


    unsigned int mCamDriverFrmWidthMax;
    unsigned int mCamDriverFrmHeightMax;
    String8 mSupportPreviewSizeReally;
 
    struct v4l2_querymenu mWhiteBalance_menu[20];
    int mWhiteBalance_number;
    
    struct v4l2_querymenu mEffect_menu[20];
    int mEffect_number;
    
    struct v4l2_querymenu mScene_menu[20];
    int mScene_number;
    
    struct v4l2_querymenu mAntibanding_menu[20];
    int mAntibanding_number;
    
    
    struct v4l2_querymenu mFlashMode_menu[20];
    int mFlashMode_number;

	//TAE & TAF
	__u32 m_focus_mode;
	static const __u32 focus_fixed = 0xFFFFFFFF;
	__u32 m_focus_value;
	__s32 m_taf_roi[4];

	void GetAFParameters(const char* focusmode = NULL);
	int AndroidZoneMapping(
			const char* tag,
			__s32 pre_w,
			__s32 pre_h,
			__s32 drv_w,
			__s32 drv_h,
			bool bPre2Drv,
			__s32 *zone);

};

//usb camera adapter



class CameraUSBAdapter: public CameraAdapter
{
public:
    CameraUSBAdapter(int cameraId);
    virtual ~CameraUSBAdapter();
    virtual int cameraStop();
    virtual int setParameters(const CameraParameters &params_set,bool &isRestartValue);
    virtual void initDefaultParameters(int camFd);
    virtual int reprocessFrame(FramInfo_s* frame);


private:
    int cameraConfig(const CameraParameters &tmpparams,bool isInit,bool &isRestartValue);
    
    int mCamDriverFrmWidthMax;
    int mCamDriverFrmHeightMax;
    String8 mSupportPreviewSizeReally;
 
    struct v4l2_querymenu mWhiteBalance_menu[20];
    int mWhiteBalance_number;
    
    struct v4l2_querymenu mEffect_menu[20];
    int mEffect_number;
    
    struct v4l2_querymenu mScene_menu[20];
    int mScene_number;
    
    struct v4l2_querymenu mAntibanding_menu[20];
    int mAntibanding_number;
    
    int mZoomMin;
    int mZoomMax;
    int mZoomStep;
    
    struct v4l2_querymenu mFlashMode_menu[20];
    int mFlashMode_number;

};
/*************************
DisplayAdapter 为帧数据消费者，从CameraAdapter接收帧数据并显示
***************************/
class DisplayAdapter//:public CameraHal_Tracer
{
    class DisplayThread : public Thread {
        DisplayAdapter* mDisplayAdapter;        
    public:
        DisplayThread(DisplayAdapter* disadap)
            : Thread(false), mDisplayAdapter(disadap){}

        virtual bool threadLoop() {
            mDisplayAdapter->displayThread();

            return false;
        }
    };

public:
    enum DisplayThreadCommands {
		// Comands
		CMD_DISPLAY_PAUSE,        
        CMD_DISPLAY_START,
        CMD_DISPLAY_STOP,
        CMD_DISPLAY_FRAME,
        CMD_DISPLAY_INVAL
    };

    enum DISPLAY_STATUS{
        STA_DISPLAY_RUNNING,
        STA_DISPLAY_PAUSE,
        STA_DISPLAY_STOP,
    };
	enum DisplayCommandStatus{
		CMD_DISPLAY_START_PREPARE = 1,
		CMD_DISPLAY_START_DONE,
		CMD_DISPLAY_PAUSE_PREPARE,
		CMD_DISPLAY_PAUSE_DONE,
		CMD_DISPLAY_STOP_PREPARE,
		CMD_DISPLAY_STOP_DONE,
		CMD_DISPLAY_FRAME_PREPARE,
		CMD_DISPLAY_FRAME_DONE,
	};
	
    typedef struct rk_displaybuf_info {
        Mutex* lock;
        buffer_handle_t* buffer_hnd;
        NATIVE_HANDLE_TYPE *priv_hnd;
        int phy_addr;
        int vir_addr;
        int buf_state;
        int stride;
    } rk_displaybuf_info_t;

    bool isNeedSendToDisplay();
    void notifyNewFrame(FramInfo_s* frame);
    
    int startDisplay(int width, int height);
    int stopDisplay();
    int pauseDisplay();

    int setPreviewWindow(struct preview_stream_ops* window);
    int getDisplayStatus(void);
    void setFrameProvider(FrameProvider* framePro);
    void dump();

    DisplayAdapter();
    ~DisplayAdapter();

    struct preview_stream_ops* getPreviewWindow();
private:
    int cameraDisplayBufferCreate(int width, int height, const char *fmt,int numBufs);
    int cameraDisplayBufferDestory(void);
    void displayThread();
    void setBufferState(int index,int status);
    void setDisplayState(int state);

    rk_displaybuf_info_t* mDisplayBufInfo;
    int mDislayBufNum;
    int mDisplayWidth;
    int mDisplayHeight;
    int mDisplayRuning;
    char mDisplayFormat[30];

    int mDispBufUndqueueMin;
    preview_stream_ops_t* mANativeWindow;
    FrameProvider* mFrameProvider;

    Mutex mDisplayLock;
    Condition mDisplayCond;
	int mDisplayState;

    MessageQueue    displayThreadCommandQ;
    sp<DisplayThread> mDisplayThread;
};

typedef struct cameraparam_info{
	float ExposureTime;
	float ISOSpeedRatings;
	float f_RgProj;
	float f_s;
	float f_s_Max1;
	float f_s_Max2;
	float f_Bg1;
	float f_Rg1;
	float f_Bg2;
	float f_Rg2;
	float expPriorOut;
	float expPriorIn;
	int   illuIdx;
	int   region;
	
	float likehood[32];
	float wight[32];	
	char  illuName[32][20];
	int   count;

	char XMLVersion[50];
}cameraparam_info_s;

//takepicture used
 typedef struct picture_info{
    int num;
    int w;
    int h;
    int fmt;
	int quality ;
	int rotation;
	int thumbquality;
	int thumbwidth;
	int thumbheight;

    //gps info
	double	altitude;
	double	latitude;
	double	longtitude;
	long	timestamp;    
	char	getMethod[33];//getMethod : len <= 32

	int  focalen;
	int  isovalue;
	int  flash;
	int  whiteBalance;

	cameraparam_info_s cameraparam; 
 }picture_info_s;


//picture encode used
struct CamCaptureInfo_s
{
    int input_phy_addr;
    int input_vir_addr;
    int output_phy_addr;
    int output_vir_addr;
    int output_buflen;
};
/**************************************
EventNotifier   负责处理msg 的回调，拍照或者录影时也作为帧数据的消费者。
**************************************/
class AppMsgNotifier
{
private:
	enum CameraStatus{
		CMD_EVENT_PREVIEW_DATA_CB_PREPARE 	= 0x01,
		CMD_EVENT_PREVIEW_DATA_CB_DONE 		= 0x02,	
		CMD_EVENT_PREVIEW_DATA_CB_MASK 		= 0x03,	

		CMD_EVENT_VIDEO_ENCING_PREPARE		= 0x04,
		CMD_EVENT_VIDEO_ENCING_DONE			= 0x08,	
		CMD_EVENT_VIDEO_ENCING_MASK			= 0x0C,	

		CMD_EVENT_PAUSE_PREPARE				= 0x10,
		CMD_EVENT_PAUSE_DONE				= 0x20,	
		CMD_EVENT_PAUSE_MASK				= 0x30,	

		CMD_EVENT_EXIT_PREPARE				= 0x40,
		CMD_EVENT_EXIT_DONE					= 0x80,
		CMD_EVENT_EXIT_MASK					= 0xC0,	

		CMD_ENCPROCESS_SNAPSHOT_PREPARE		= 0x100,
		CMD_ENCPROCESS_SNAPSHOT_DONE		= 0x200,
		CMD_ENCPROCESS_SNAPSHOT_MASK		= 0x300,

		CMD_ENCPROCESS_PAUSE_PREPARE		= 0x400,
		CMD_ENCPROCESS_PAUSE_DONE			= 0x800,
		CMD_ENCPROCESS_PAUSE_MASK			= 0xC00,

		CMD_ENCPROCESS_EXIT_PREPARE			= 0x1000,
		CMD_ENCPROCESS_EXIT_DONE			= 0x2000,
		CMD_ENCPROCESS_EXIT_MASK			= 0x3000,

		STA_RECORD_RUNNING				= 0x4000,
		STA_RECEIVE_PIC_FRAME				= 0x8000,
		STA_RECEIVE_PREVIEWCB_FRAME         = 0x10000,
	};
	
    //处理preview data cb及video enc
    class CameraAppMsgThread :public Thread
    {
    public:
        enum EVENT_THREAD_CMD{
            CMD_EVENT_PREVIEW_DATA_CB,
            CMD_EVENT_VIDEO_ENCING,
            CMD_EVENT_PAUSE,
            CMD_EVENT_EXIT
        };
	protected:
		AppMsgNotifier* mAppMsgNotifier;
	public:
		CameraAppMsgThread(AppMsgNotifier* hw)
			: Thread(false),mAppMsgNotifier(hw) { }
	
		virtual bool threadLoop() {
			mAppMsgNotifier->eventThread();
	
			return false;
		}
    };

    //处理picture
	class EncProcessThread : public Thread {
	public:
	    enum ENC_THREAD_CMD{
            CMD_ENCPROCESS_SNAPSHOT,
            CMD_ENCPROCESS_PAUSE,
            CMD_ENCPROCESS_EXIT,
	    };
	protected:
		AppMsgNotifier* mAppMsgNotifier;
	public:
		EncProcessThread(AppMsgNotifier* hw)
			: Thread(false),mAppMsgNotifier(hw) { }
	
		virtual bool threadLoop() {
			mAppMsgNotifier->encProcessThread();
	
			return false;
		}
	};

	friend class EncProcessThread;
public:
    AppMsgNotifier();
    ~AppMsgNotifier();

    void setPictureRawBufProvider(BufferProvider* bufprovider);
    void setPictureJpegBufProvider(BufferProvider* bufprovider);
    
    int takePicture(picture_info_s picinfo);
    int flushPicture();
    int pausePreviewCBFrameProcess();

    void setVideoBufProvider(BufferProvider* bufprovider);
    int startRecording(int w,int h);
    int stopRecording();
    void releaseRecordingFrame(const void *opaque);
    int msgEnabled(int32_t msg_type);
    void notifyCbMsg(int msg,int ret);
    
    bool isNeedSendToVideo();
    bool isNeedSendToPicture();
    bool isNeedSendToDataCB();
    bool isNeedToDisableCaptureMsg();
    void notifyNewPicFrame(FramInfo_s* frame);
    void notifyNewPreviewCbFrame(FramInfo_s* frame);
    void notifyNewVideoFrame(FramInfo_s* frame);
    int enableMsgType(int32_t msgtype);
    int disableMsgType(int32_t msgtype);
    void setCallbacks(camera_notify_callback notify_cb,
            camera_data_callback data_cb,
            camera_data_timestamp_callback data_cb_timestamp,
            camera_request_memory get_memory,
            void *user);
    void setFrameProvider(FrameProvider * framepro);
    int  setPreviewDataCbRes(int w,int h, const char *fmt);
    
    void stopReceiveFrame();
    void startReceiveFrame();
    void dump();
	void setDatacbFrontMirrorState(bool mirror);
	picture_info_s&  getPictureInfoRef();
	vpu_display_mem_pool *pool;
private:

   void encProcessThread();
   void eventThread();

	int captureEncProcessPicture(FramInfo_s* frame);
    int processPreviewDataCb(FramInfo_s* frame);
    int processVideoCb(FramInfo_s* frame);
    
    int copyAndSendRawImage(void *raw_image, int size);
    int copyAndSendCompressedImage(void *compressed_image, int size);
    int Jpegfillexifinfo(RkExifInfo *exifInfo,picture_info_s &params);
    int Jpegfillgpsinfo(RkGPSInfo *gpsInfo,picture_info_s &params);
    
    int32_t mMsgTypeEnabled;
    
    picture_info_s mPictureInfo;
   // bool mReceivePictureFrame;
    int mEncPictureNum;
    Mutex mPictureLock;

    Mutex mRecordingLock;
//    bool mRecordingRunning;
    int mRecordW;
    int mRecordH;

    Mutex mDataCbLock;


    sp<CameraAppMsgThread> mCameraAppMsgThread;
    sp<EncProcessThread> mEncProcessThread;
    BufferProvider* mRawBufferProvider;
    BufferProvider* mJpegBufferProvider;
    BufferProvider* mVideoBufferProvider;
    FrameProvider* mFrameProvider;

    camera_notify_callback mNotifyCb;
    camera_data_callback mDataCb;    
    camera_data_timestamp_callback mDataCbTimestamp;
    camera_request_memory mRequestMemory;
    void  *mCallbackCookie;

    MessageQueue encProcessThreadCommandQ;
    MessageQueue eventThreadCommandQ;

    camera_memory_t* mVideoBufs[CONFIG_CAMERA_VIDEO_BUF_CNT];

    char mPreviewDataFmt[30];
    int mPreviewDataW;
    int mPreviewDataH;
	int mRunningState;
    bool mDataCbFrontMirror;
    int mPicSize;
    camera_memory_t* mPicture;
};


/***********************
CameraHal类负责与cameraservice联系，实现
cameraservice要求实现的接口。此类只负责公共资源的申请，以及任务的分发。
***********************/
class CameraHal
{
public:  
/*--------------------Interface Methods---------------------------------*/
    /** Set the ANativeWindow to which preview frames are sent */
    int setPreviewWindow(struct preview_stream_ops *window);

    /** Set the notification and data callbacks */
    void setCallbacks(camera_notify_callback notify_cb,
            camera_data_callback data_cb,
            camera_data_timestamp_callback data_cb_timestamp,
            camera_request_memory get_memory,
            void *user);

    /**
     * The following three functions all take a msg_type, which is a bitmask of
     * the messages defined in include/ui/Camera.h
     */

    /**
     * Enable a message, or set of messages.
     */
    void enableMsgType(int32_t msg_type);

    /**
     * Disable a message, or a set of messages.
     *
     * Once received a call to disableMsgType(CAMERA_MSG_VIDEO_FRAME), camera
     * HAL should not rely on its client to call releaseRecordingFrame() to
     * release video recording frames sent out by the cameral HAL before and
     * after the disableMsgType(CAMERA_MSG_VIDEO_FRAME) call. Camera HAL
     * clients must not modify/access any video recording frame after calling
     * disableMsgType(CAMERA_MSG_VIDEO_FRAME).
     */
    void disableMsgType(int32_t msg_type);

    /**
     * Query whether a message, or a set of messages, is enabled.  Note that
     * this is operates as an AND, if any of the messages queried are off, this
     * will return false.
     */
    int msgTypeEnabled(int32_t msg_type);

    /**
     * Start preview mode.
     */
    int startPreview();

    /**
     * Stop a previously started preview.
     */
    void stopPreview();

    /**
     * Returns true if preview is enabled.
     */
    int previewEnabled();

    /**
     * Request the camera HAL to store meta data or real YUV data in the video
     * buffers sent out via CAMERA_MSG_VIDEO_FRAME for a recording session. If
     * it is not called, the default camera HAL behavior is to store real YUV
     * data in the video buffers.
     *
     * This method should be called before startRecording() in order to be
     * effective.
     *
     * If meta data is stored in the video buffers, it is up to the receiver of
     * the video buffers to interpret the contents and to find the actual frame
     * data with the help of the meta data in the buffer. How this is done is
     * outside of the scope of this method.
     *
     * Some camera HALs may not support storing meta data in the video buffers,
     * but all camera HALs should support storing real YUV data in the video
     * buffers. If the camera HAL does not support storing the meta data in the
     * video buffers when it is requested to do do, INVALID_OPERATION must be
     * returned. It is very useful for the camera HAL to pass meta data rather
     * than the actual frame data directly to the video encoder, since the
     * amount of the uncompressed frame data can be very large if video size is
     * large.
     *
     * @param enable if true to instruct the camera HAL to store
     *        meta data in the video buffers; false to instruct
     *        the camera HAL to store real YUV data in the video
     *        buffers.
     *
     * @return OK on success.
     */
    int storeMetaDataInBuffers(int enable);

    /**
     * Start record mode. When a record image is available, a
     * CAMERA_MSG_VIDEO_FRAME message is sent with the corresponding
     * frame. Every record frame must be released by a camera HAL client via
     * releaseRecordingFrame() before the client calls
     * disableMsgType(CAMERA_MSG_VIDEO_FRAME). After the client calls
     * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
     * responsibility to manage the life-cycle of the video recording frames,
     * and the client must not modify/access any video recording frames.
     */
    int startRecording();

    /**
     * Stop a previously started recording.
     */
    void stopRecording();

    /**
     * Returns true if recording is enabled.
     */
    int recordingEnabled();

    /**
     * Release a record frame previously returned by CAMERA_MSG_VIDEO_FRAME.
     *
     * It is camera HAL client's responsibility to release video recording
     * frames sent out by the camera HAL before the camera HAL receives a call
     * to disableMsgType(CAMERA_MSG_VIDEO_FRAME). After it receives the call to
     * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
     * responsibility to manage the life-cycle of the video recording frames.
     */
    void releaseRecordingFrame(const void *opaque);

    /**
     * Start auto focus, the notification callback routine is called with
     * CAMERA_MSG_FOCUS once when focusing is complete. autoFocus() will be
     * called again if another auto focus is needed.
     */
    int autoFocus();

    /**
     * Cancels auto-focus function. If the auto-focus is still in progress,
     * this function will cancel it. Whether the auto-focus is in progress or
     * not, this function will return the focus position to the default.  If
     * the camera does not support auto-focus, this is a no-op.
     */
    int cancelAutoFocus();

    /**
     * Take a picture.
     */
    int takePicture();

    /**
     * Cancel a picture that was started with takePicture. Calling this method
     * when no picture is being taken is a no-op.
     */
    int cancelPicture();

    /**
     * Set the camera parameters. This returns BAD_VALUE if any parameter is
     * invalid or not supported.
     */
    int setParameters(const char *parms);
    int setParameters(const CameraParameters &params_set);
	int setParametersUnlock(const CameraParameters &params_set);

    /** Retrieve the camera parameters.  The buffer returned by the camera HAL
        must be returned back to it with put_parameters, if put_parameters
        is not NULL.
     */
    char* getParameters();

    /** The camera HAL uses its own memory to pass us the parameters when we
        call get_parameters.  Use this function to return the memory back to
        the camera HAL, if put_parameters is not NULL.  If put_parameters
        is NULL, then you have to use free() to release the memory.
    */
    void putParameters(char *);

    /**
     * Send command to camera driver.
     */
    int sendCommand(int32_t cmd, int32_t arg1, int32_t arg2);

    /**
     * Release the hardware resources owned by this object.  Note that this is
     * *not* done in the destructor.
     */
    void release();

    /**
     * Dump state of the camera hardware
     */
    int dump(int fd);
    void dump_mem(__u16 addr_s, __u16 len, __u8* content);
/*--------------------Internal Member functions - Public---------------------------------*/
    /** Constructor of CameraHal */
    CameraHal(int cameraId);

    // Destructor of CameraHal
    virtual ~CameraHal();    

    int getCameraFd();
	bool isNeedToDisableCaptureMsg();

public:
    CameraAdapter* mCameraAdapter;
    DisplayAdapter* mDisplayAdapter;
    AppMsgNotifier*   mEventNotifier;
    MemManagerBase* mMemoryManager;
    BufferProvider* mPreviewBuf;
    BufferProvider* mVideoBuf;
    BufferProvider* mRawBuf;
    BufferProvider* mJpegBuf;
    MemManagerBase* mCamMemManager;


private:
    int selectPreferedDrvSize(int *width,int * height,bool is_capture);
    int fillPicturInfo(picture_info_s& picinfo);
	void setCamStatus(int status, int type);
	int checkCamStatus(int cmd);
    enum CommandThreadCommands { 
		// Comands
        CMD_PREVIEW_START = 1,
        CMD_PREVIEW_STOP,
        CMD_PREVIEW_CAPTURE_CANCEL,
        CMD_CONTINUOS_PICTURE,
        
        CMD_AF_START,
        CMD_AF_CANCEL,

        CMD_SET_PREVIEW_WINDOW,
        CMD_SET_PARAMETERS,
        CMD_EXIT,

    };

    enum CommandThreadStatus{
       CMD_STATUS_RUN ,
       CMD_STATUS_STOP ,

    };
	
	enum CommandStatus{
		CMD_PREVIEW_START_PREPARE 			= 0x01,
		CMD_PREVIEW_START_DONE				= 0x02,	
		CMD_PREVIEW_START_MASK				= 0x11,		
		
		CMD_PREVIEW_STOP_PREPARE			= 0x04,
		CMD_PREVIEW_STOP_DONE				= 0x08,
		CMD_PREVIEW_STOP_MASK				= 0x0C,
		
		CMD_CONTINUOS_PICTURE_PREPARE		= 0x10,
		CMD_CONTINUOS_PICTURE_DONE			= 0x20,
		CMD_CONTINUOS_PICTURE_MASK			= 0x30,		
		
		CMD_PREVIEW_CAPTURE_CANCEL_PREPARE 	= 0x40,
		CMD_PREVIEW_CAPTURE_CANCEL_DONE 	= 0x80,
		CMD_PREVIEW_CAPTURE_CANCEL_MASK 	= 0xC0,		
		
		CMD_AF_START_PREPARE				= 0x100,
		CMD_AF_START_DONE					= 0x200,
		CMD_AF_START_MASK					= 0x300,		
		
		CMD_AF_CANCEL_PREPARE				= 0x400,
		CMD_AF_CANCEL_DONE					= 0x800,
		CMD_AF_CANCEL_MASK					= 0xC00,		
		
		CMD_SET_PREVIEW_WINDOW_PREPARE		= 0x1000,
		CMD_SET_PREVIEW_WINDOW_DONE			= 0x2000,
		CMD_SET_PREVIEW_WINDOW_MASK			= 0x3000,		
		
		CMD_SET_PARAMETERS_PREPARE			= 0x4000,
		CMD_SET_PARAMETERS_DONE				= 0x8000,
		CMD_SET_PARAMETERS_MASK				= 0xC000,		
		
		CMD_EXIT_PREPARE					= 0x10000,
		CMD_EXIT_DONE						= 0x20000,
		CMD_EXIT_MASK						= 0x30000,

		STA_RECORD_RUNNING					= 0x40000,
		STA_PREVIEW_CMD_RECEIVED			= 0x80000,
		
		STA_DISPLAY_RUNNING					= 0x100000,
        STA_DISPLAY_PAUSE					= 0x200000,
        STA_DISPLAY_STOP					= 0x400000,	
		STA_DISPLAY_MASK					= 0x700000,	       
	};

	class CommandThread : public Thread {
        CameraHal* mHardware;
    public:
        CommandThread(CameraHal* hw)
            : Thread(false), mHardware(hw) { }

        virtual bool threadLoop() {
            mHardware->commandThread();

            return false;
        }
    };

   friend class CommandThread;
   void commandThread();
   sp<CommandThread> mCommandThread;
   MessageQueue commandThreadCommandQ;
   int mCommandRunning;

   void updateParameters(CameraParameters & tmpPara);
   CameraParameters mParameters;


   mutable Mutex mLock;        // API lock -- all public methods
  // bool mRecordRunning;
  // bool mPreviewCmdReceived;
   int mCamFd;
   unsigned int mCameraStatus;
   int mCamId;
};

}

#endif


