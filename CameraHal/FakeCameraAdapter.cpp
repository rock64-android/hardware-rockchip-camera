#include "FakeCameraAdapter.h"

namespace android{
CameraFakeAdapter::CameraFakeAdapter(int cameraId)
                   :CameraAdapter(cameraId)
{
    mCamDriverV4l2MemType = V4L2_MEMORY_OVERLAY;

}

CameraFakeAdapter::~CameraFakeAdapter()
{
}


int CameraFakeAdapter::setParameters(const CameraParameters &params_set)
{
    mParameters = params_set;
    return 0;
}

void CameraFakeAdapter::initDefaultParameters(int camFd)
{
    CameraParameters params;
    String8 parameterString;
    /*preview size setting*/
    parameterString.append("176x144,320x240,352x288,640x480,800x600,1280x720,1920x1080,1600x1200,2592x1944");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
    params.setPreviewSize(640,480);
    /*picture size setting*/      
    params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, parameterString.string());        
    params.setPictureSize(640,480); 

    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, "yuv420sp,yuv420p");
    params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);
    params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
    params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);

 	params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"0");
    parameterString = CameraParameters::FOCUS_MODE_FIXED;
    params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
	params.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, parameterString.string());
    
    /*picture format setting*/
    params.set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, CameraParameters::PIXEL_FORMAT_JPEG);
    params.setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);
    /*jpeg quality setting*/
    params.set(CameraParameters::KEY_JPEG_QUALITY, "70");
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
    parameterString = "60";
    params.set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, parameterString.string());
    /*vertical angle of view setting ,no much meaning ,only for passing cts */
    parameterString = "28.9";
    params.set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, parameterString.string());

    /*quality of the EXIF thumbnail in Jpeg picture setting */
    parameterString = "50";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, parameterString.string());
    /*supported size of the EXIF thumbnail in Jpeg picture setting */
    parameterString = "0x0,160x128,160x96";
    params.set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES, parameterString.string());
    parameterString = "160";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, parameterString.string());
    parameterString = "128";
    params.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, parameterString.string()); 
    /* zyc@rock-chips.com: for cts ,KEY_MAX_NUM_DETECTED_FACES_HW should not be 0 */
    params.set(CameraParameters::KEY_RECORDING_HINT,"false");
    params.set(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED,"false");
    params.set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED,"true");
    params.set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK_SUPPORTED, "false");
    params.set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK_SUPPORTED, "false");
    //params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
    //params.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES,CameraParameters::FOCUS_MODE_FIXED);

    // params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"0");
    params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, "0");
    params.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, "1");
    params.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, "0");
    params.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, "1");
    params.set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, "false");
    params.set(CameraParameters::KEY_SUPPORTED_EFFECTS, "false");
    params.set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, "false");
    params.set(CameraParameters::KEY_SUPPORTED_ANTIBANDING, "false");

    //for video test
    params.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "3000,30000");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(3000,30000)");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "10,15,30");  
	
    mParameters = params;
    LOGD("%s",__func__);
    
}


int CameraFakeAdapter::getFrame(FramInfo_s** tmpFrame)
{
    int buf_phy, buf_vir;
    int index = -1;
    LOG1("GET ONE FRAME ");
    index = mPreviewBufProvider->getOneAvailableBuffer(&buf_phy,&buf_vir);
    if(index < 0){
        LOGE("%s:no available buffer found",__FUNCTION__);
        return -1;
    }
 //   mPreviewBufProvider->setBufferStatus(index, 1, (PreviewBufferProvider::CMD_PREVIEWBUF_WRITING));
    // fill frame info:w,h,phy,vir
    mPreviewFrameInfos[index].frame_fmt=  mCamDriverPreviewFmt;
    mPreviewFrameInfos[index].frame_height = mCamPreviewH;
    mPreviewFrameInfos[index].frame_width = mCamPreviewW;
    mPreviewFrameInfos[index].frame_index = index;
    mPreviewFrameInfos[index].phy_addr = mPreviewBufProvider->getBufPhyAddr(index);
    mPreviewFrameInfos[index].vir_addr = (int)mCamDriverV4l2Buffer[index];
    //get zoom_value
    mPreviewFrameInfos[index].zoom_value = 100;
    mPreviewFrameInfos[index].used_flag = 0;
    mPreviewFrameInfos[index].frame_size = 0;

    /*
    //FAKE CAMERA TODO:
    //copy image data to preview buffer
    buffer info:
    output frame format: yuv420sp(NV12)
    output frame res   :mCamPreviewW x mCamPreviewH
    outpurt frame buffer: buf_vir
    
    */

    *tmpFrame = &(mPreviewFrameInfos[index]);
    mPreviewFrameIndex++;
    return 0;
}

int CameraFakeAdapter::adapterReturnFrame(int index,int cmd)
{
    mCamDriverStreamLock.lock();
    LOG1("RETURN FRAME ");
    if (!mCamDriverStream) {
        LOGD("%s(%d): preview thread is pause, so buffer %d isn't enqueue to camera",__FUNCTION__,__LINE__,index);
        mCamDriverStreamLock.unlock();
        return 0;
    }
	mPreviewBufProvider->setBufferStatus(index,0, cmd); 
    mCamDriverStreamLock.unlock();
    return 0;
}


int CameraFakeAdapter::cameraStream(bool on)
{
    mCamDriverStreamLock.lock();
    mCamDriverStream = on;

	mCamDriverStreamLock.unlock();
    return 0;
}

int CameraFakeAdapter::cameraStart()
{
    int buffer_count;
    int previewBufStatus = ((PreviewBufferProvider::CMD_PREVIEWBUF_WRITING) | (PreviewBufferProvider::CMD_PREVIEWBUF_DISPING)
                            |(PreviewBufferProvider::CMD_PREVIEWBUF_VIDEO_ENCING) |(PreviewBufferProvider::CMD_PREVIEWBUF_SNAPSHOT_ENCING)
                            | (PreviewBufferProvider::CMD_PREVIEWBUF_DATACB));
    buffer_count = mPreviewBufProvider->getBufCount();
    for (int i = 0; i < buffer_count; i++) {
        mCamDriverV4l2Buffer[i] = (char*)mPreviewBufProvider->getBufVirAddr(i);
        mPreviewBufProvider->setBufferStatus(i, 0,previewBufStatus);
    }
    
    mPreviewErrorFrameCount = 0;
    mPreviewFrameIndex = 0;
    cameraStream(true);

    return 0;
}

int CameraFakeAdapter::cameraSetSize(int w, int h, int fmt, bool is_capture)
{
    return 0;
}
int CameraFakeAdapter::cameraStop()
{
    return 0;
}


void CameraFakeAdapter::dump(int cameraId)
{
    return;
}

int CameraFakeAdapter::cameraCreate(int cameraId)
{
    mCamDriverPreviewFmt = V4L2_PIX_FMT_NV12;
    return 0;
}

int CameraFakeAdapter::cameraDestroy()
{
    return 0;
}


}
