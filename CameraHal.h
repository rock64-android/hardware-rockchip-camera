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


#include "MessageQueue.h"
#include "../jpeghw/release/encode_release/hw_jpegenc.h"


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
namespace android {

/*
*v0.1.0 : CameraHal support for android 4.0(ICS);
*v0.1.1 : CameraHal support query framerate from driver, display thread support NativeWindow sync and asyc mode;
*v0.1.2 : CameraHal support video snap;
*v0.1.3 : CameraHal display NativeWindow in async mode;
*v0.1.5 : CameraHal support send rgb565 to NativeWindow for display, facelock activity fix rgb565;
*v0.1.6 : Camera and Facelock activity fix rgb565 display, but send yuv420 to JAVA for Facelock;
*v0.2.0 : Camera CTS test PASS(android.hardware.cts.CameraTest) and support usb camera(UVC);
*v0.2.1 : Camera CTS test PASS(android.hardware.cts.CameraGLTest);
*v0.2.2 : 
*         1) CameraHal fix send yuv data to app error when attach usb camera(UVC);
*         2) CameraHal add support exposure,but sensor driver must support it;
*         3) CameraHal fix send 160x120 resolution to facelock for speed up facelock;
*v0.2.3 : 
*         1) CameraHal support 240x160 for fring;
*         2) CameraHal support query version by getprop sys_graphic.camerahal.version;
*         3) CameraHal display format add check hwc module version;
*v0.2.4 : CameraHal support obtain necessary memory(preview/raw/jpeg) from pmem or ion device;
*v0.2.5 : 
*         1) CameraHal compatible for camera driver v0.x.9
*         2) CameraHal_Utils don't call msgTypeEnabled,may be deadlock;
*v0.2.6 :
*         1) CameraHal compatible for RK30XX and RK29XX;
*         2) CameraHal support mirror frame which sended by mDataCb and from front camera,
*            config by CONFIG_CAMERA_FRONT_MIRROR_MDATACB;
*v0.2.7 :
*         1) CameraHal support CONFIG_CAMERA_SINGLE_SENSOR_FORCE_BACK_FOR_CTS
*v0.2.8 :
*         1) CameraHal support CONFIG_CAMERA_XXX_PREVIEW_FPS_XXX for cts 
*            android.hardware.cts.CameraGLTest#testCameraToSurfaceTextureMetadata
*            android.hardware.cts.CameraTest#testPreviewFpsRange
*         2) CameraHal support nv12->rgb565 by rga in rk30xx
*v0.2.9 : modify the condition of raw and jpeg size , avoid the failure of unstandard mCamDriverFrmWidthMax value
          from driver.
*v0.2.a : 
*         1) Print all resolution framerate in KEY_SUPPORTED_PREVIEW_FRAME_RATES;
*         2) CONFIG_CAMERA_XXX_PREVIEW_FPS_XXX direct validate;
*v0.2.b : 
*         1) fix preview thread may be asynchronous with command thread when take picture frequently in rk30;
*         2) improve messagequeue between thread communication;
*         3) fix KEY_ZOOM_SUPPORTED config error in initDefaultParameters when digital zoom isn't supported;
*         4) fix mDisplayFormat is NV21, but mNativeWindow buffer format is NV12;
*         5) add CONFIG_CAMERA_UVC_INVAL_FRAMECNT config invaildate first some frame in uvc camera;

*v0.2.c:
*		  1) add lock when display and preview thread receive pause message,ensure that the thread is indeed in pause
            status before the command thread send start message.
		  2)  the second parameter of erro callback message is wrong ,fix it
          3) status of pic thread setting is moved to pic thread before capture, and unset after capture , to avoid the thread 
			 asynchronous.
*v0.2.d:
	      1) adjust the timeout interval of takepic , display thread pause and preview thread pause.
		  2) update the version to odd ,from this version on ,we use the odd number to indicate that this is not sure the stable.
		  3) fix uvc camera convert display format is error in rk30;

*v0.2.e:  
*         1) display thread must check whether message queue is empty;
*v0.3.0: just update the version num
*v0.3.1:  
*         1) Only one usb sensor in board config as back;
*         2) add configuration sensor orientation in skype app;
*v0.3.3:
*         1) add CONFIG_CAMERA_ORIENTATION_SKYPE
*v0.3.5:
*         1) Stop camera driver before stop preview thread;
*         2) Camera command thread is scheduled before wait for NativeWindow, so main thread can't wake up command thread;
*v0.3.7:
*         1) Display thread support ANativeWindow dequeue buffer operation is block, and is sync mode;
*
*v0.3.9:
*         1) fix mCamDriverStreamLock may be lost unlock in preview thread;
*v0.3.b:
	      1) support driver preview  format rgb565
*v0.3.d:
*         1) fix preview data callback mirror local value have not been init before used;
*         2) fix preview lock may be unlock, and display thread api lock and signal order;
*v0.3.f:
*         1) use arm to do rotation when taking pic if no ipp supported;
*v0.3.11:
*         1) add support rk3066b;
		  2) command thread may trap when native window is null , fix it. 
*v0.3.13:
		  1) default preview size is setted to svga if driver supported for RK30;
		  2) fix uvc camera may be panic when close;

*0.3.15:  
*         1) if there is no ipp, do picture rotation of 90 an 180 degree  by arm 
*v0.3.17:
*         1) Support config whether mirror the preview data which send to apk by apk name;
*v0.3.19: 
*         1) fix operate mCamId before initiation in v0.3.17 version;
*         2) add support 8Mega picture; 
*v0.3.21: 1) add focus zone support
*v0.3.23:
*         1)fix uvc camera erro when taking pic,must unmap buffer
*         2)throw erro exception if failure to allocate preview memory
*         3)add mirror preview data which send to yahoo messager apk
*v0.3.25:
*         1)fix some print error and picturesize array size no enough in initDefaultParameters;
*         2)pmem code invalidate by CONFIG_CAMERA_MEM;
*         3)rga code invalidate by CONFIG_CAMERA_INVALIDATE_RGA;
*         4)support android-4.2 directly;
*v0.3.27:
*         1)support FOCUS_MODE_CONTINUOUS_PICTURE;
*         2)fix falsh menu error in initDefaultParameters;
*
*v0.3.29:
*         1)add support preview format yuv420p(yv12) for CtsVerifter, and delete rgb565 in preview support format;
*v0.3.2b: 1)add face detection support
*		  2)stop camera stream befor set previewthread status  when taking pic 
*v0.3.2d: 
*         1)fix panorama preview error after take picture twice when in uvc camera; because some preview buffer state(displaying)
*           is invalidate in camera start. the state is preview thread set, but display thread has been pause.
*         2)fix mCamDriverStreamLock isn't unlock when streamoff error in cameraStream function;
*v0.3.2f:
*         1)fix preview thread and command thread may dead lock in mPreviewLock when fast take picture frequently;
*v0.3.31:
*         1)fix testFocusAreas faild in CTS;
*v0.3.33:
*         1)fix v0.3.33 version zoneStr haven't check is NULL;
*/
#define CONFIG_CAMERAHAL_VERSION KERNEL_VERSION(0, 3, 0x33) 

/*  */
#define CAMERA_DISPLAY_FORMAT_YUV420SP   CameraParameters::PIXEL_FORMAT_YUV420SP
#define CAMERA_DISPLAY_FORMAT_RGB565     CameraParameters::PIXEL_FORMAT_RGB565
#define CAMERA_DISPLAY_FORMAT_NV12       "nv12"
/* 
*NOTE: 
*       CONFIG_CAMERA_DISPLAY_FORCE and CONFIG_CAMERA_DISPLAY_FORCE_FORMAT is debug macro, 
*    CONFIG_CAMERA_DISPLAY_FORCE must equal to 0 in official version.     
*/
#define CONFIG_CAMERA_DISPLAY_FORCE     0
#define CONFIG_CAMERA_DISPLAY_FORCE_FORMAT CAMERA_DISPLAY_FORMAT_RGB565

#define CONFIG_CAMERA_SINGLE_SENSOR_FORCE_BACK_FOR_CTS   0
#define CONFIG_CAMERA_FRAME_DV_PROC_STAT    0
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB  1
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB_ALL  0
#define CONFIG_CAMERA_FRONT_MIRROR_MDATACB_APK  "<com.skype.raider>,<com.yahoo.mobile.client.andro>"
#define CONFIG_CAMERA_PRVIEW_BUF_CNT        4
#define CONFIG_CAMERA_UVC_INVAL_FRAMECNT    5
#define CONFIG_CAMERA_ORIENTATION_SKYPE     0
#define CONFIG_CAMERA_FRONT_ORIENTATION_SKYPE     0
#define CONFIG_CAMERA_BACK_ORIENTATION_SKYPE      0

#define CONFIG_CAMERA_FRONT_PREVIEW_FPS_MIN    3000        // 3fps
#define CONFIG_CAMERA_FRONT_PREVIEW_FPS_MAX    30000        //30fps
#define CONFIG_CAMERA_BACK_PREVIEW_FPS_MIN     3000        
#define CONFIG_CAMERA_BACK_PREVIEW_FPS_MAX     30000

#define CAMERAHAL_VERSION_PROPERTY_KEY       "sys_graphic.cam_hal.ver"
#define CAMERADRIVER_VERSION_PROPERTY_KEY    "sys_graphic.cam_driver.ver"
#define CAMERA_PMEM_NAME                     "/dev/pmem_cam"
#define CAMERA_DRIVER_SUPPORT_FORMAT_MAX   32

#define RAW_BUFFER_SIZE_8M         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ? 0xF40000:0xB70000)
#define RAW_BUFFER_SIZE_5M         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ? 0x9A0000:0x740000)
#define RAW_BUFFER_SIZE_3M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ?0x600000 :0x480000)
#define RAW_BUFFER_SIZE_2M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565) ?0x3A0000 :0x2c0000)
#define RAW_BUFFER_SIZE_1M          (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565)? 0x180000 :0x120000)
#define RAW_BUFFER_SIZE_0M3         (( mCamDriverPreviewFmt == V4L2_PIX_FMT_RGB565)?0x150000 :0x100000)

#define JPEG_BUFFER_SIZE_8M          0x700000
#define JPEG_BUFFER_SIZE_5M          0x400000
#define JPEG_BUFFER_SIZE_3M          0x300000
#define JPEG_BUFFER_SIZE_2M          0x300000
#define JPEG_BUFFER_SIZE_1M          0x200000
#define JPEG_BUFFER_SIZE_0M3         0x100000

#define V4L2_BUFFER_MAX             32
#define V4L2_BUFFER_MMAP_MAX        16
#define PAGE_ALIGN(x)   (((x) + 0xFFF) & (~0xFFF)) // Set as multiple of 4K

#define CAMHAL_GRALLOC_USAGE GRALLOC_USAGE_HW_TEXTURE | \
                             GRALLOC_USAGE_HW_RENDER | \
                             GRALLOC_USAGE_SW_WRITE_OFTEN | \
                             GRALLOC_USAGE_SW_READ_OFTEN /*| \
                             GRALLOC_USAGE_SW_WRITE_MASK| \
                             GRALLOC_USAGE_SW_READ_RARELY*/ 
#define CAMERA_IPP_NAME                  "/dev/rk29-ipp"
#ifdef ALOGD
#define LOGD      ALOGD
#endif
#ifdef ALOGV
#define LOGV      ALOGV
#endif
#ifdef ALOGE
#define LOGE      ALOGE
#endif
#ifdef ALOGI
#define LOGI      ALOGI
#endif

#if defined(TARGET_BOARD_PLATFORM_RK30XX) || defined(TARGET_RK29) || defined(TARGET_BOARD_PLATFORM_RK2928)                         
    #define NATIVE_HANDLE_TYPE             private_handle_t
    #define PRIVATE_HANDLE_GET_W(hd)       (hd->width)    
    #define PRIVATE_HANDLE_GET_H(hd)       (hd->height)    
#elif defined(TARGET_BOARD_PLATFORM_RK30XXB)               
    #define NATIVE_HANDLE_TYPE             IMG_native_handle_t
    #define PRIVATE_HANDLE_GET_W(hd)       (hd->iWidth)    
    #define PRIVATE_HANDLE_GET_H(hd)       (hd->iHeight)    
#endif

struct CamCaptureInfo_s
{
	int input_phy_addr;
	int input_vir_addr;
	int output_phy_addr;
	int output_vir_addr;
	int output_buflen;
};

struct CamMemHeapInfo_s
{
	sp<MemoryHeapBase> heap;
    sp<IMemory> buffer; 
};

typedef struct rk_previewbuf_info {
    Mutex *lock;
    buffer_handle_t* buffer_hnd;
    NATIVE_HANDLE_TYPE *priv_hnd;
    camera_memory_t* video_buf;
    int phy_addr;
    int vir_addr;
    int buf_state;
} rk_previewbuf_info_t;

enum PreviewBufStatus {
    CMD_PREVIEWBUF_DISPING = 0x01,
    CMD_PREVIEWBUF_ENCING = 0x02,
    CMD_PREVIEWBUF_SNAPSHOT_ENCING = 0x04,
    CMD_PREVIEWBUF_WRITING = 0x08,
};

#define CAMERA_PREVIEWBUF_ALLOW_DISPLAY(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_ENC(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_ENC_PICTURE(a) ((a&CMD_PREVIEWBUF_WRITING)==0x00)
#define CAMERA_PREVIEWBUF_ALLOW_WRITE(a)   ((a&(CMD_PREVIEWBUF_DISPING|CMD_PREVIEWBUF_ENCING|CMD_PREVIEWBUF_SNAPSHOT_ENCING|CMD_PREVIEWBUF_WRITING))==0x00)


#define CAMERA_IS_UVC_CAMERA()  (strcmp((char*)&mCamDriverCapability.driver[0],"uvcvideo") == 0)
#define CAMERA_IS_RKSOC_CAMERA()  ((strstr((char*)&mCamDriverCapability.driver[0],"rk") != NULL)\
                                    && (strstr((char*)&mCamDriverCapability.driver[0],"-camera") != NULL))


class CameraHal {
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

/*--------------------Internal Member functions - Public---------------------------------*/
    /** Constructor of CameraHal */
    CameraHal(int cameraId);

    // Destructor of CameraHal
    ~CameraHal();    

    int getCameraFd();
    
private:

    class PreviewThread : public Thread {
        CameraHal* mHardware;
    public:
        PreviewThread(CameraHal* hw)
            : Thread(false), mHardware(hw) { }

        virtual bool threadLoop() {
            mHardware->previewThread();

            return false;
        }
    };

    class DisplayThread : public Thread {
        CameraHal* mHardware;        
    public:
        DisplayThread(CameraHal* hw)
            : Thread(false), mHardware(hw){}

        virtual bool threadLoop() {
            mHardware->displayThread();

            return false;
        }
    };

    class PictureThread : public Thread {
    protected:
        CameraHal* mHardware;
    public:
        PictureThread(CameraHal* hw)
            : Thread(false), mHardware(hw) { }

        virtual bool threadLoop() {
            mHardware->pictureThread();

            return false;
        }
    };

    class SnapshotThread : public Thread {
    protected:
        CameraHal* mHardware;
    public:
        SnapshotThread(CameraHal* hw)
            : Thread(false),mHardware(hw) { }

        virtual bool threadLoop() {
            mHardware->snapshotThread();

            return false;
        }
    };

    class AutoFocusThread : public Thread {
        CameraHal* mHardware;
    public:
        AutoFocusThread(CameraHal* hw)
            : Thread(false), mHardware(hw) { }

        virtual bool threadLoop() {
            mHardware->autofocusThread();

            return false;
        }
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

    void displayThread();
    void previewThread();
	void commandThread();
    void pictureThread();
    void snapshotThread();
    void autofocusThread();
    void initDefaultParameters();
    int capturePicture(struct CamCaptureInfo_s *capture);
    int captureVideoPicture(struct CamCaptureInfo_s *capture, int index);
	int capturePicturePmemInfoSet(int pmem_fd, int input_offset, int out_offset);
    int cameraCreate(int cameraId);
    int cameraDestroy();
    int cameraConfig(const CameraParameters &params);
    int cameraQuery(CameraParameters &params);
    int cameraSetSize(int w, int h,  int fmt);
    int cameraStart();
    int cameraStop();
    int cameraStream(bool on);
	int cameraAutoFocus(const char *focus, bool auto_trig_only);
    int Jpegfillgpsinfo(RkGPSInfo *gpsInfo);
    int Jpegfillexifinfo(RkExifInfo *exifInfo);
    int copyAndSendRawImage(void *raw_image, int size);
    int copyAndSendCompressedImage(void *compressed_image, int size);
        
    int cameraRawJpegBufferCreate(int rawBufferSize, int jpegBufferSize);
    int cameraRawJpegBufferDestory();
    int cameraFormatConvert(int v4l2_fmt_src, int v4l2_fmt_dst, const char *android_fmt_dst, char *srcbuf, char *dstbuf, 
                            int srcphy,int dstphy,int src_w, int src_h,int dst_w, int dst_h, bool mirror);
     int cameraDisplayBufferCreate(int width, int height, const char *fmt,int numBufs);
     int cameraDisplayBufferDestory(void);
     int cameraPreviewBufferCreate(unsigned int numBufs);
    int cameraPreviewBufferDestory();
    int cameraPreviewBufferSetSta(rk_previewbuf_info_t *buf_hnd,int cmd, int set);
    int cameraFramerateQuery(unsigned int format, unsigned int w, unsigned int h, int *min, int *max);
    int cameraFpsInfoSet(CameraParameters &params);

    int cameraDisplayThreadStart(int done);
    int cameraDisplayThreadPause(int done);
    int cameraDisplayThreadStop(int done);

    int cameraPreviewThreadSet(unsigned int setStatus,int done);

	int cameraSetFaceDetect(bool window,bool on);   
    char *cameraDevicePathCur;    
    char cameraCallProcess[30];
    struct v4l2_capability mCamDriverCapability;
    unsigned int mCamDriverFrmWidthMax;
    unsigned int mCamDriverFrmHeightMax;
    unsigned int mCamDriverPreviewFmt;
    unsigned int mCamDriverPictureFmt;
    unsigned int mCamDriverSupportFmt[CAMERA_DRIVER_SUPPORT_FORMAT_MAX];
    enum v4l2_memory mCamDriverV4l2MemType;
    char *mCamDriverV4l2Buffer[V4L2_BUFFER_MAX];
    unsigned int mCamDriverV4l2BufferLen;

    mutable Mutex mLock;        // API lock -- all public methods
    CameraParameters mParameters;
    Mutex mANativeWindowLock;
    Condition mANativeWindowCond;
    preview_stream_ops_t *mANativeWindow;
    String8 mSupportPreviewSizeReally;
    int mMsgEnabled;
    int mPreviewErrorFrameCount;
	int mPreviewBufferCount;
    int mPreviewWidth;
    int mPreviewHeight;
    int mPreviewFrame2AppWidth;
    int mPreviewFrame2AppHeight;
    int mPictureWidth;
    int mPictureHeight;
    int mJpegBufferSize;
    int mRawBufferSize;
    int mPreviewFrameSize;
    int mPreviewFrame2AppSize;
    int mDispBufUndqueueMin;
    volatile int32_t mPreviewStartTimes;  
    int mPreviewFrameIndex;

    rk_previewbuf_info_t *mPreviewBuffer[CONFIG_CAMERA_PRVIEW_BUF_CNT];
    rk_previewbuf_info_t *mPreviewBufferMap[CONFIG_CAMERA_PRVIEW_BUF_CNT];
    rk_previewbuf_info_t *mDisplayBufferMap[CONFIG_CAMERA_PRVIEW_BUF_CNT];
    rk_previewbuf_info_t mGrallocBufferMap[CONFIG_CAMERA_PRVIEW_BUF_CNT];

    camera_memory_t* mPreviewMemory;
    unsigned char* mPreviewBufs[CONFIG_CAMERA_PRVIEW_BUF_CNT];
    camera_memory_t* mVideoBufs[CONFIG_CAMERA_PRVIEW_BUF_CNT];
    
    unsigned int CameraHal_SupportFmt[5];
    char mDisplayFormat[30];
    
    sp<DisplayThread>  mDisplayThread;
    sp<PreviewThread>  mPreviewThread;
	sp<CommandThread>  mCommandThread;
    sp<PictureThread>  mPictureThread;
    sp<SnapshotThread>  mSnapshotThread;
    sp<AutoFocusThread>  mAutoFocusThread;
    int mSnapshotRunning;
    int mCommandRunning;
    unsigned int mPreviewRunning;
    Mutex mPreviewLock;
    Condition mPreviewCond;
    bool mPreviewCmdReceived;
    int mDisplayRuning;
    Mutex mDisplayLock;
    Condition mDisplayCond;    
    bool mRecordRunning; 
    Mutex mPictureLock;
    int mPictureRunning;
    Mutex mAutoFocusLock;
    Condition mAutoFocusCond;
    bool mExitAutoFocusThread; 
    Mutex mCamDriverStreamLock;
    bool mCamDriverStream;
		
    int iCamFd;
    int mCamId;
    int mRGAFd;
    
    bool mDriverMirrorSupport;
    bool mDriverFlipSupport;
    bool mDataCbFrontMirror;
    
    struct v4l2_querymenu mWhiteBalance_menu[20];
    int mWhiteBalance_number;

    struct v4l2_querymenu mEffect_menu[20];
    int mEffect_number;

    struct v4l2_querymenu mScene_menu[20];
    int mScene_number;
    
    int mZoomMin;
    int mZoomMax;
    int mZoomStep;

    struct v4l2_querymenu mFlashMode_menu[20];
    int mFlashMode_number;

    double mGps_latitude;
    double mGps_longitude;
    double mGps_altitude;
    long mGps_timestamp;

    enum DisplayRunStatus {
        STA_DISPLAY_PAUSE,
        STA_DISPLAY_RUN,
        STA_DISPLAY_STOP
    };

    enum PreviewRunStatus {
        STA_PREVIEW_PAUSE, // equal to CMD_PREVIEW_PAUSE 
        STA_PREVIEW_RUN,  // equal to CMD_PREVIEW_START 
        STA_PREVIEW_STOP, // equal to CMD_PREVIEW_STOP
        STA_PREVIEW_INVAL = 4
    };

    enum PictureRunStatus {
        STA_PICTURE_STOP,
        STA_PICTURE_RUN,
        STA_PICTURE_WAIT_STOP,
    };
    enum SnapshotThreadCommands {
        CMD_SNAPSHOT_SNAPSHOT,
        CMD_SNAPSHOT_EXIT
    };
    enum DisplayThreadCommands {
		// Comands
		CMD_DISPLAY_PAUSE,        
        CMD_DISPLAY_START,
        CMD_DISPLAY_STOP,
        CMD_DISPLAY_FRAME,
        CMD_DISPLAY_INVAL
    };
    enum PreviewThreadCommands {
		// Comands       
		CMD_PREVIEW_THREAD_PAUSE,        
        CMD_PREVIEW_THREAD_START,
        CMD_PREVIEW_THREAD_STOP,
        CMD_PREVIEW_VIDEOSNAPSHOT,
        CMD_PREVIEW_INVAL
    };
    enum CommandThreadCommands { 
		// Comands
        CMD_PREVIEW_START,
        CMD_PREVIEW_STOP,
        CMD_PREVIEW_CAPTURE,        
        CMD_PREVIEW_CAPTURE_CANCEL,
        CMD_PREVIEW_QBUF,        
        
        CMD_AF_START,
        CMD_AF_CANCEL,
        
        CMD_EXIT,

    };

    enum ThreadCmdArgs {
        CMDARG_ERR = -1,
        CMDARG_OK = 0,        
        CMDARG_ACK = 1,
        CMDARG_NACK = 2,        
    };
    
    MessageQueue    displayThreadCommandQ;
    MessageQueue    displayThreadAckQ;
    MessageQueue    previewThreadCommandQ;
    MessageQueue    snapshotThreadAckQ;
    MessageQueue    snapshotThreadCommandQ;
    MessageQueue    previewThreadAckQ;
    MessageQueue    commandThreadCommandQ;
    MessageQueue    commandThreadAckQ;
    
    camera_notify_callback mNotifyCb;
    camera_data_callback mDataCb;    
    camera_data_timestamp_callback mDataCbTimestamp;
    camera_request_memory mRequestMemory;
    void  *mCallbackCookie;
    MemManagerBase* mCamBuffer;
};

}; // namespace android
#endif
