#include "CameraHal.h"

namespace android {


#define EXIF_DEF_MAKER          "rockchip"
#define EXIF_DEF_MODEL          "rk29sdk"


static char ExifMaker[32];
static char ExifModel[32];


AppMsgNotifier::AppMsgNotifier()
               :encProcessThreadCommandQ("pictureEncThreadQ"),
                eventThreadCommandQ("eventThreadQ")

{
    LOGD("%s(%d):IN",__FUNCTION__,__LINE__);

    mMsgTypeEnabled = 0;
//    mReceivePictureFrame = false;
	mRunningState = 0;
    mEncPictureNum = 0;
//    mRecordingRunning = 0;
    mRecordW = 0;
    mRecordH = 0;
    mRawBufferProvider = NULL;
    mJpegBufferProvider = NULL;
    mVideoBufferProvider =NULL;
    mFrameProvider = NULL;
    mNotifyCb = NULL;
    mDataCb = NULL;
    mDataCbTimestamp = NULL;
    mRequestMemory = NULL;
    mCallbackCookie = NULL;

    mPreviewDataW = 0;
    mPreviewDataH = 0;
    int i ;
    //request mVideoBufs
	for (i=0; i<CONFIG_CAMERA_VIDEOENC_BUF_CNT; i++) {
		mVideoBufs[i] = NULL;
	}
    
    //create thread 
    mCameraAppMsgThread = new CameraAppMsgThread(this);
    mCameraAppMsgThread->run("AppMsgThread",ANDROID_PRIORITY_DISPLAY);
    mEncProcessThread = new EncProcessThread(this);
    mEncProcessThread->run("EncProcessThread",ANDROID_PRIORITY_NORMAL);
    LOGD("%s(%d):OUT",__FUNCTION__,__LINE__);

}
AppMsgNotifier::~AppMsgNotifier()
{
    LOGD("%s(%d):IN",__FUNCTION__,__LINE__);
    //stop thread
    Message msg;
    Semaphore sem,sem1;
    if(mCameraAppMsgThread != NULL){
        msg.command = CameraAppMsgThread::CMD_EVENT_EXIT;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        eventThreadCommandQ.put(&msg);
        if(msg.arg1){
            sem.Wait();
        }
        mCameraAppMsgThread->requestExitAndWait();
        mCameraAppMsgThread.clear();
    }

    if(mEncProcessThread != NULL){
        msg.command = EncProcessThread::CMD_ENCPROCESS_EXIT;
        sem1.Create();
        msg.arg1 = (void*)(&sem1);
        encProcessThreadCommandQ.put(&msg);
        if(msg.arg1){
            sem1.Wait();
        }
        mEncProcessThread->requestExitAndWait();
        mEncProcessThread.clear();
    }

    int i = 0;
    //release mVideoBufs
    for (int i=0; i < CONFIG_CAMERA_VIDEOENC_BUF_CNT; i++) {

    	if(mVideoBufs[i]!= NULL){
    		//free(mVideoBufs[i]);
    		mVideoBufs[i]->release(mVideoBufs[i]);
    		mVideoBufs[i] = NULL;
    	}
    }

    //destroy buffer
    if(mRawBufferProvider)
        mRawBufferProvider->freeBuffer();
    if(mJpegBufferProvider)
        mJpegBufferProvider->freeBuffer();
    if(mVideoBufferProvider)
        mVideoBufferProvider->freeBuffer();
    LOGD("%s(%d):OUT",__FUNCTION__,__LINE__);
}

void AppMsgNotifier::setPictureRawBufProvider(BufferProvider* bufprovider)
{
    mRawBufferProvider = bufprovider;
 }
void AppMsgNotifier::setPictureJpegBufProvider(BufferProvider* bufprovider)
{
    mJpegBufferProvider = bufprovider;
}
void AppMsgNotifier::setFrameProvider(FrameProvider * framepro)
{
    mFrameProvider = framepro;
}
void AppMsgNotifier::setVideoBufProvider(BufferProvider* bufprovider)
{
    mVideoBufferProvider = bufprovider;
}


int AppMsgNotifier::takePicture(picture_info_s picinfo)
{
   LOG_FUNCTION_NAME
    Mutex::Autolock lock(mPictureLock); 
    //if(mReceivePictureFrame){
    if(mRunningState&STA_RECEIVE_PIC_FRAME){
        LOGE("%s(%d): picture taken process is running now !",__FUNCTION__,__LINE__);
        return -1;
    }
    memset(&mPictureInfo,0,sizeof(picture_info_s));
    memcpy(&mPictureInfo,&picinfo,sizeof(picture_info_s));
    mEncPictureNum = mPictureInfo.num;
    //mReceivePictureFrame = true;
	mRunningState |= STA_RECEIVE_PIC_FRAME;
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int AppMsgNotifier::flushPicture()
{
    Message msg;
    Semaphore sem;
   LOG_FUNCTION_NAME
    {
	    Mutex::Autolock lock(mPictureLock); 

	    //mReceivePictureFrame = false;
		mRunningState &= ~STA_RECEIVE_PIC_FRAME;
    }

    //send a msg to cancel pic
    msg.command = EncProcessThread::CMD_ENCPROCESS_PAUSE;
    sem.Create();
    msg.arg1 = (void*)(&sem);
    encProcessThreadCommandQ.put(&msg);
    if(msg.arg1){
        sem.Wait();
    }
    LOG_FUNCTION_NAME_EXIT
    return 0;
}

bool AppMsgNotifier::isNeedSendToPicture()
{
    Mutex::Autolock lock(mPictureLock); 
    //if((mReceivePictureFrame) && (mPictureInfo.num > 0)){
    if((mRunningState&STA_RECEIVE_PIC_FRAME) && (mPictureInfo.num > 0)){
        LOGD("%s%d:need to encode picture this frame",__FUNCTION__,__LINE__);
        return true;
    }else
        return false;
}
int AppMsgNotifier::startRecording(int w,int h)
{
   LOG_FUNCTION_NAME
    int i = 0,frame_size = 0;
	int *addr;
	struct bufferinfo_s videoencbuf;
    
    Mutex::Autolock lock(mRecordingLock);
    //create video buffer
    //video enc just support yuv420 format
    frame_size = PAGE_ALIGN(w*h*3/2);
    //release video buffer
    mVideoBufferProvider->freeBuffer();
    mVideoBufferProvider->createBuffer(CONFIG_CAMERA_VIDEOENC_BUF_CNT, frame_size, VIDEOENCBUFFER);
	for (int i=0; i < CONFIG_CAMERA_VIDEOENC_BUF_CNT; i++) {
    	if(!mVideoBufs[i])
    		mVideoBufs[i] = mRequestMemory(-1, 4, 1, NULL);
    	if( (NULL == mVideoBufs[i]) || ( NULL == mVideoBufs[i]->data)) {
    		mVideoBufs[i] = NULL;
    		LOGE("%s(%d): video buffer %d create failed",__FUNCTION__,__LINE__,i);
    	}
    	if (mVideoBufs[i]) {
    		addr = (int*)mVideoBufs[i]->data;
    		*addr =  (int)mVideoBufferProvider->getBufPhyAddr(i);
    	}
	}

   
    mRecordW = w;
    mRecordH = h;
//    mRecordingRunning = true;
	mRunningState |= STA_RECORD_RUNNING;

    LOG_FUNCTION_NAME_EXIT

    return 0;
}
int AppMsgNotifier::stopRecording()
{
    Message msg;
    Semaphore sem;
   LOG_FUNCTION_NAME

    {
        Mutex::Autolock lock(mRecordingLock);
        //mRecordingRunning = false;
        mRunningState &= ~STA_RECORD_RUNNING;
    }
    //send msg to stop recording
    msg.command = CameraAppMsgThread::CMD_EVENT_PAUSE;
    sem.Create();
    msg.arg1 = (void*)(&sem);
    eventThreadCommandQ.put(&msg);
    if(msg.arg1){
        sem.Wait();
    }
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
bool AppMsgNotifier::isNeedSendToVideo()
{
  // LOG_FUNCTION_NAME
    Mutex::Autolock lock(mRecordingLock);
   // if(mRecordingRunning == false)
   if(!(mRunningState & STA_RECORD_RUNNING))
        return false;
    else{
        LOG2("%s%d:need to encode video this frame",__FUNCTION__,__LINE__);

        return true;
    }
}
void AppMsgNotifier::releaseRecordingFrame(const void *opaque)
{
    ssize_t offset;
    size_t  size;
    int index = -1,i;
//   LOG_FUNCTION_NAME
 
	for(i=0; i<mVideoBufferProvider->getBufCount(); i++) {
		if (mVideoBufs[i]->data == opaque) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		LOGE("%s(%d): this video buffer is invaildate",__FUNCTION__,__LINE__);
		return;
	}
	mVideoBufferProvider->setBufferStatus(index, 0, 0);		
//    LOG_FUNCTION_NAME_EXIT

}
//must call this when PREVIEW DATACB MSG is disabled,and before enableMsgType
int AppMsgNotifier::setPreviewDataCbRes(int w,int h, const char *fmt)
{
    LOG_FUNCTION_NAME
    memset(mPreviewDataFmt,0,sizeof(mPreviewDataFmt));
    strcpy(mPreviewDataFmt,fmt);
    mPreviewDataW = w;
    mPreviewDataH = h;
    LOG_FUNCTION_NAME_EXIT
    return 0;
    
}
int AppMsgNotifier::enableMsgType(int32_t msgtype)
{
    LOG_FUNCTION_NAME
    if(msgtype & (CAMERA_MSG_PREVIEW_FRAME)){
        Mutex::Autolock lock(mDataCbLock);
        mMsgTypeEnabled |= msgtype;
		//LOGE("%s(%d): this video buffer is invaildate",__FUNCTION__,__LINE__);
    }else
        mMsgTypeEnabled |= msgtype;
    LOG_FUNCTION_NAME_EXIT

    return 0;
}
int AppMsgNotifier::msgEnabled(int32_t msg_type)
{
    return (mMsgTypeEnabled & msg_type);
}
int AppMsgNotifier::disableMsgType(int32_t msgtype)
{
    LOG_FUNCTION_NAME

    if(msgtype & (CAMERA_MSG_POSTVIEW_FRAME | CAMERA_MSG_RAW_IMAGE|CAMERA_MSG_COMPRESSED_IMAGE)){
//        if((mEncPictureNum <= 0) || (mReceivePictureFrame == false))
    if((mEncPictureNum <= 0) || ((mRunningState&STA_RECEIVE_PIC_FRAME) == 0x0))			
            mMsgTypeEnabled &= ~msgtype;
        else
            LOGD("%s%d:no need to disable picure msgtype.",__FUNCTION__,__LINE__);

    }else if(msgtype & (CAMERA_MSG_PREVIEW_FRAME)){
            
            {
                LOG1("%s%d: get mDataCbLock",__FUNCTION__,__LINE__);
                Mutex::Autolock lock(mDataCbLock);
                mMsgTypeEnabled &= ~msgtype;
                LOG1("%s%d: release mDataCbLock",__FUNCTION__,__LINE__);

            }
            //send a msg to disable preview frame cb 
            Message msg;
			msg.command = CameraAppMsgThread::CMD_EVENT_PAUSE;
			msg.arg1 = NULL;
			eventThreadCommandQ.put(&msg);
			LOG1("%s%d: disable CAMERA_MSG_PREVIEW_FRAME success",__FUNCTION__,__LINE__);			
    }
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
void AppMsgNotifier::setCallbacks(camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    LOG_FUNCTION_NAME
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mRequestMemory = get_memory;
    mCallbackCookie = user;
    LOG_FUNCTION_NAME_EXIT
}
void AppMsgNotifier::notifyCbMsg(int msg,int ret)
{
    if(mMsgTypeEnabled & msg)
        mNotifyCb(msg, ret, 0, mCallbackCookie);
}
// need sync with enable/disable msg ?
bool AppMsgNotifier::isNeedSendToDataCB()
{
    Mutex::Autolock lock(mDataCbLock);
    
    return ((mMsgTypeEnabled & CAMERA_MSG_PREVIEW_FRAME) &&(mDataCb));
}

void AppMsgNotifier::notifyNewPicFrame(FramInfo_s* frame)
{
    //send to enc thread;
   // encProcessThreadCommandQ
               //send a msg to disable preview frame cb
    Message msg;
    Mutex::Autolock lock(mPictureLock); 
    mPictureInfo.num--;
    msg.command = EncProcessThread::CMD_ENCPROCESS_SNAPSHOT;
    msg.arg2 = (void*)(frame);
    msg.arg3 = (void*)(frame->used_flag);
    encProcessThreadCommandQ.put(&msg);
}
void AppMsgNotifier::notifyNewPreviewCbFrame(FramInfo_s* frame)
{
    //send to app msg thread
    Message msg;
    Mutex::Autolock lock(mDataCbLock);
    msg.command = CameraAppMsgThread::CMD_EVENT_PREVIEW_DATA_CB;
    msg.arg2 = (void*)(frame);
    msg.arg3 = (void*)(frame->used_flag);
    eventThreadCommandQ.put(&msg);

}
void AppMsgNotifier::notifyNewVideoFrame(FramInfo_s* frame)
{
    //send to app msg thread
    Message msg;
    Mutex::Autolock lock(mRecordingLock);
    msg.command = CameraAppMsgThread::CMD_EVENT_VIDEO_ENCING ;
    msg.arg2 = (void*)(frame);
    msg.arg3 = (void*)(frame->used_flag);
    LOG2("%s(%d):notify new frame,index(%d)",__FUNCTION__,__LINE__,frame->frame_index);
    eventThreadCommandQ.put(&msg);
}

static BufferProvider* g_rawbufProvider = NULL;
static BufferProvider* g_jpegbufProvider = NULL;

extern "C" int jpegEncFlushBufferCb(int buf_type, int offset, int len)
{
    int ret = 0;

    /* ddl@rock-chips.com notes: 
     *                     0 : input buffer index for jpeg encoder
     *                     1 : output buffer index for jpeg encoder
     */

    if (buf_type == 0) {
        g_rawbufProvider->flushBuffer(0);
    } else if (buf_type == 1) {
        g_jpegbufProvider->flushBuffer(0);
    }

    return ret;
}


int AppMsgNotifier::copyAndSendRawImage(void *raw_image, int size)
{
    camera_memory_t* picture = NULL;
    void *dest = NULL, *src = NULL;

    if(mMsgTypeEnabled & CAMERA_MSG_RAW_IMAGE) {
        picture = mRequestMemory(-1, size, 1, NULL);
        if (NULL != picture) {
            dest = picture->data;
            if (NULL != dest) {
                memcpy(dest, raw_image, size);
                mDataCb(CAMERA_MSG_RAW_IMAGE, picture, 0, NULL, mCallbackCookie);
            }
            picture->release(picture);
        } else if (mMsgTypeEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY) {
            mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
        }
    } else if (mMsgTypeEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY) {
        mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
    }
    return 0;
}

int AppMsgNotifier::copyAndSendCompressedImage(void *compressed_image, int size)
{
    camera_memory_t* picture = NULL;
    void *dest = NULL, *src = NULL;

    if(mMsgTypeEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
        picture = mRequestMemory(-1, size, 1, NULL);
        if (NULL != picture) {
            dest = picture->data;
            if (NULL != dest) {
                memcpy(dest, compressed_image, size);
                mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, picture, 0, NULL, mCallbackCookie);
            }
            picture->release(picture);
        }
    }
    return 0;
}

int AppMsgNotifier::Jpegfillexifinfo(RkExifInfo *exifInfo,picture_info_s &params)
{
	char property[PROPERTY_VALUE_MAX];
	
	if(exifInfo==NULL){
		LOGE( "..%s..%d..argument error ! ",__FUNCTION__,__LINE__);
		return 0;
	}
	
	/*fill in jpeg exif tag*/ 
	property_get("ro.product.brand", property, EXIF_DEF_MAKER);
	strncpy((char *)ExifMaker, property,sizeof(ExifMaker) - 1);
	ExifMaker[sizeof(ExifMaker) - 1] = '\0';
	exifInfo->maker = ExifMaker;
	exifInfo->makerchars = strlen(ExifMaker)+1;
	
	property_get("ro.product.model", property, EXIF_DEF_MODEL);
	strncpy((char *)ExifModel, property,sizeof(ExifModel) - 1);
	ExifModel[sizeof(ExifModel) - 1] = '\0';
	exifInfo->modelstr = ExifModel;
	exifInfo->modelchars = strlen(ExifModel)+1;  
	
	exifInfo->Orientation = 1;

	// Date time
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime((char *)exifInfo->DateTime, 20, "%Y:%m:%d %H:%M:%S", timeinfo);
	
	exifInfo->ExposureTime.num = 1;
	exifInfo->ExposureTime.denom = 100;
	exifInfo->ApertureFNumber.num = 0x118;
	exifInfo->ApertureFNumber.denom = 0x64;
	exifInfo->ISOSpeedRatings = 0x59;
	exifInfo->CompressedBitsPerPixel.num = 0x4;
	exifInfo->CompressedBitsPerPixel.denom = 0x1;
	exifInfo->ShutterSpeedValue.num = 0x452;
	exifInfo->ShutterSpeedValue.denom = 0x100;
	exifInfo->ApertureValue.num = 0x2f8;
	exifInfo->ApertureValue.denom = 0x100;
	exifInfo->ExposureBiasValue.num = 0;
	exifInfo->ExposureBiasValue.denom = 0x100;
	exifInfo->MaxApertureValue.num = 0x02f8;
	exifInfo->MaxApertureValue.denom = 0x100;
	exifInfo->MeteringMode = 02;

	exifInfo->Flash = params.flash;	
	exifInfo->FocalLength.num = (uint32_t)params.focalen;
	exifInfo->FocalLength.denom = 0x1;
	
	exifInfo->FocalPlaneXResolution.num = 0x8383;
	exifInfo->FocalPlaneXResolution.denom = 0x67;
	exifInfo->FocalPlaneYResolution.num = 0x7878;
	exifInfo->FocalPlaneYResolution.denom = 0x76;
	exifInfo->SensingMethod = 2;
	exifInfo->FileSource = 3;
	exifInfo->CustomRendered = 1;
	exifInfo->ExposureMode = 0;

	exifInfo->WhiteBalance = params.whiteBalance;
	exifInfo->DigitalZoomRatio.num = params.w;
	exifInfo->DigitalZoomRatio.denom = params.w;
	exifInfo->SceneCaptureType = 0x01;	 
	return 0;
}


/*fill in jpeg gps information*/
int AppMsgNotifier::Jpegfillgpsinfo(RkGPSInfo *gpsInfo,picture_info_s &params)
{
	char* gpsprocessmethod = NULL;
	double latitude,longtitude,altitude;
	double deg,min,sec;
	double fract;
	long timestamp,num; 
	int year,month,day,hour_t,min_t,sec_t;
	char date[12];
	
	if(gpsInfo==NULL) {    
		LOGE( "%s(%d): gpsInfo is NULL ",__FUNCTION__,__LINE__);
		return 0;
	}

	altitude = params.altitude;
	latitude = params.latitude;
	longtitude = params.longtitude;
	timestamp = params.timestamp; 
//	gpsprocessmethod = (char*)params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);
	
	if(latitude >= 0){
		gpsInfo->GPSLatitudeRef[0] = 'N';
		gpsInfo->GPSLatitudeRef[1] = '\0';
	}else if((latitude <0)&&(latitude!=-1)){
		gpsInfo->GPSLatitudeRef[0] = 'S';
		gpsInfo->GPSLatitudeRef[1] = '\0';
	}else{
		gpsInfo->GPSLatitudeRef[0] = '\0';
		gpsInfo->GPSLatitudeRef[1] = '\0';
	}
	
   if(latitude!= -1)
   {
		latitude = fabs(latitude);
		fract = modf(latitude,&deg);
		fract = modf(fract*60,&min);
		fract = modf(fract*60,&sec);
		if(fract >= 0.5)sec+=1;
		
		//LOGD("latitude: deg = %f;min = %f;sec =%f",deg,min,sec);

		gpsInfo->GPSLatitude[0].num = (uint32_t)deg;
		gpsInfo->GPSLatitude[0].denom = 1;
		gpsInfo->GPSLatitude[1].num =  (uint32_t)min;
		gpsInfo->GPSLatitude[1].denom = 1;
		gpsInfo->GPSLatitude[2].num =  (uint32_t)sec;
		gpsInfo->GPSLatitude[2].denom = 1;
   }
  
   if(longtitude >= 0){
		gpsInfo->GPSLongitudeRef[0] = 'E';
		gpsInfo->GPSLongitudeRef[1] = '\0';
	}else if((longtitude < 0)&&(longtitude!=-1)){
		gpsInfo->GPSLongitudeRef[0] = 'W';
		gpsInfo->GPSLongitudeRef[1] = '\0';
	}else{
		gpsInfo->GPSLongitudeRef[0] = '\0';
		gpsInfo->GPSLongitudeRef[1] = '\0';
	}

	if(longtitude!=-1)
	{
		longtitude = fabs(longtitude);
		fract = modf(longtitude,&deg);
		fract = modf(fract*60,&min);
		modf(fract*60,&sec);
		
		//LOGD("longtitude: deg = %f;min = %f;sec =%f",deg,min,sec);
		gpsInfo->GPSLongitude[0].num = (uint32_t)deg;
		gpsInfo->GPSLongitude[0].denom = 1;
		gpsInfo->GPSLongitude[1].num = (uint32_t)min;
		gpsInfo->GPSLongitude[1].denom = 1;
		gpsInfo->GPSLongitude[2].num = (uint32_t)sec;
		gpsInfo->GPSLongitude[2].denom = 1;
	}
	
	if(altitude >= 0){
		gpsInfo->GPSAltitudeRef = 0;
	}else if((altitude <0 )&&(altitude!=-1)) {
		gpsInfo->GPSAltitudeRef = 1;
	} 
	
	if(altitude!=-1)
	{
		altitude = fabs(altitude);
		gpsInfo->GPSAltitude.num =(uint32_t)altitude;
		gpsInfo->GPSAltitude.denom = 0x1;
		//LOGD("altitude =%f  GPSAltitudeRef: %d",altitude,gpsInfo->GPSAltitudeRef);		
	}
	
	if(timestamp!=-1)
	{
	   /*timestamp,has no meaning,only for passing cts*/
		//LOGD("timestamp =%d",timestamp);
		gpsInfo->GpsTimeStamp[0].num =0;
		gpsInfo->GpsTimeStamp[0].denom = 1;
		gpsInfo->GpsTimeStamp[1].num = 0;
		gpsInfo->GpsTimeStamp[1].denom = 1;
		gpsInfo->GpsTimeStamp[2].num = timestamp&0x03;
		gpsInfo->GpsTimeStamp[2].denom = 1; 		
		memcpy(gpsInfo->GpsDateStamp,"2008:01:01\0",11);//"YYYY:MM:DD\0"
	}	 
	return 0;
}


int AppMsgNotifier::captureEncProcessPicture(FramInfo_s* frame){
    int ret = 0;
	int jpeg_w,jpeg_h,i;
	unsigned int pictureSize;
	int jpegSize;
	int quality;
	int thumbquality = 0;
	int thumbwidth	= 0;
	int thumbheight = 0;
	int err = 0;
	int rotation = 0;
	JpegEncInInfo JpegInInfo;
	JpegEncOutInfo JpegOutInfo;  
	RkExifInfo exifInfo;
	RkGPSInfo gpsInfo;
	char ExifAsciiPrefix[8] = {'A', 'S', 'C', 'I', 'I', '\0', '\0', '\0'};
	char gpsprocessmethod[45];
	char *getMethod = NULL;
	double latitude,longtitude,altitude;
	long timestamp;
	JpegEncType encodetype;
    int picfmt;
    int rawbuf_phy;
    int rawbuf_vir;
    int jpegbuf_phy;
    int jpegbuf_vir;
    int input_phy_addr,input_vir_addr;
    int output_phy_addr,output_vir_addr;
    int jpegbuf_size;
	int bufindex;

	memset(&JpegInInfo,0x00,sizeof(JpegEncInInfo));
	memset(&JpegOutInfo,0x00,sizeof(JpegEncOutInfo));
	memset(&exifInfo,0x00,sizeof(exifInfo));

	quality = mPictureInfo.quality;
	thumbquality = mPictureInfo.thumbquality;
	thumbwidth	= mPictureInfo.thumbwidth;
	thumbheight = mPictureInfo.thumbheight;
	rotation = mPictureInfo.rotation;
    
	jpeg_w = mPictureInfo.w;
    jpeg_h = mPictureInfo.h;
	/*get gps information*/
	altitude = mPictureInfo.altitude;
	latitude = mPictureInfo.latitude;
	longtitude = mPictureInfo.longtitude;
	timestamp = mPictureInfo.timestamp;    
	getMethod = mPictureInfo.getMethod;//getMethod : len <= 32

    picfmt = mPictureInfo.fmt;
	
	
	if(picfmt ==V4L2_PIX_FMT_RGB565){
		encodetype = HWJPEGENC_RGB565;
		pictureSize = jpeg_w * jpeg_h *2;
	}
	else{
		encodetype = JPEGENC_YUV420_SP;
		pictureSize = jpeg_w * jpeg_h * 3/2;
	}
	if (pictureSize & 0xfff) {
		pictureSize = (pictureSize & 0xfffff000) + 0x1000;
	}

    jpegbuf_size = 0x700000; //pictureSize;
    //create raw & jpeg buffer
    ret = mRawBufferProvider->createBuffer(1, pictureSize, RAWBUFFER);
    if(ret < 0){
        LOGE("mRawBufferProvider->createBuffer FAILED");
        goto 	captureEncProcessPicture_exit;
    }
    ret =mJpegBufferProvider->createBuffer(1, jpegbuf_size,JPEGBUFFER);
    if(ret < 0){
        LOGE("mJpegBufferProvider->createBuffer FAILED");
        goto 	captureEncProcessPicture_exit;
    }

    bufindex=mRawBufferProvider->getOneAvailableBuffer(&rawbuf_phy, &rawbuf_vir);
    if(bufindex < 0){
        LOGE("mRawBufferProvider->getOneAvailableBuffer FAILED");
        goto 	captureEncProcessPicture_exit;
    }
	mRawBufferProvider->setBufferStatus(bufindex, 1);		
    bufindex=mJpegBufferProvider->getOneAvailableBuffer(&jpegbuf_phy, &jpegbuf_vir);
    if(bufindex < 0){
        LOGE("mJpegBufferProvider->getOneAvailableBuffer FAILED");
        goto 	captureEncProcessPicture_exit;
    }
    
    g_rawbufProvider = mRawBufferProvider;
    g_jpegbufProvider = mJpegBufferProvider;


    input_phy_addr = frame->phy_addr;
    input_vir_addr = frame->vir_addr;
    output_phy_addr = jpegbuf_phy;
    output_vir_addr = jpegbuf_vir;
	LOGD("rawbuf_phy:%x,rawbuf_vir:%x;jpegbuf_phy = %x,jpegbuf_vir = %x",rawbuf_phy,rawbuf_vir,jpegbuf_phy,jpegbuf_vir);
	
	if (mMsgTypeEnabled & CAMERA_MSG_SHUTTER)
		mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
	LOGD("captureEncProcessPicture,rotation = %d,jpeg_w = %d,jpeg_h = %d",rotation,jpeg_w,jpeg_h);
    //2. copy to output buffer for mirro and flip
	/*ddl@rock-chips.com: v0.4.7*/
    // bool rotat_180 = false; //used by ipp
    if((frame->frame_fmt == V4L2_PIX_FMT_NV12)){
        output_phy_addr = rawbuf_phy;
        output_vir_addr = rawbuf_vir;
        arm_camera_yuv420_scale_arm(V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV12, (char*)(frame->vir_addr),
            (char*)rawbuf_vir,frame->frame_width, frame->frame_height,
             jpeg_w, jpeg_h,false);
        input_phy_addr = output_phy_addr;
        input_vir_addr = output_vir_addr;
        mRawBufferProvider->flushBuffer(0);
        LOG1("EncPicture:V4L2_PIX_FMT_NV12,arm_camera_yuv420_scale_arm");
    }
	/*if ((frame->frame_fmt != picfmt) || (frame->frame_width!= jpeg_w) || (frame->frame_height != jpeg_h) 
    	|| (frame->zoom_value != 100)) {

        output_phy_addr = rawbuf_phy;
        output_vir_addr = rawbuf_vir;
        //do rotation,scale,zoom,fmt convert.   
		if(cameraFormatConvert(frame->frame_fmt, picfmt, NULL,
        (char*)input_vir_addr,(char*)output_vir_addr,0,0,jpeg_w*jpeg_h*2,
        jpeg_w, jpeg_h,jpeg_w,jpeg_w, jpeg_h,jpeg_w,false)==0)
      // arm_yuyv_to_nv12(frame->frame_width, frame->frame_height,(char*)input_vir_addr, (char*)output_vir_addr);
        {
            //change input addr
			input_phy_addr = output_phy_addr;
			input_vir_addr = output_vir_addr;
			mRawBufferProvider->flushBuffer(0);
		}
	}*/
	
	if((mMsgTypeEnabled & (CAMERA_MSG_RAW_IMAGE))|| (mMsgTypeEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY)) {
		copyAndSendRawImage((void*)input_vir_addr, pictureSize);
    }
    
    output_phy_addr = jpegbuf_phy;
    output_vir_addr = jpegbuf_vir;

    //3. src data will be changed by mirror and flip algorithm
    //use jpeg buffer as line buffer

	if(rotation == 0)
	{
		JpegInInfo.rotateDegree = DEGREE_0; 
	}
	else if(rotation == 180)
	{
		YuvData_Mirror_Flip(V4L2_PIX_FMT_NV12, (char*)input_vir_addr,
		(char*) jpegbuf_vir, jpeg_w, jpeg_h);
		mRawBufferProvider->flushBuffer(0);
		JpegInInfo.rotateDegree = DEGREE_0; 
	}
	else if(rotation == 90)
	{
		YuvData_Mirror_Flip(V4L2_PIX_FMT_NV12, (char*)input_vir_addr,
		(char*) jpegbuf_vir, jpeg_w, jpeg_h);
		mRawBufferProvider->flushBuffer(0);	
		JpegInInfo.rotateDegree = DEGREE_270;
	}
	else if(rotation == 270)
	{
		JpegInInfo.rotateDegree = DEGREE_270; 		
	}

	JpegInInfo.frameHeader = 1;
	JpegInInfo.yuvaddrfor180 = (int)NULL;
	JpegInInfo.type = encodetype;
	JpegInInfo.y_rgb_addr = input_phy_addr;
	JpegInInfo.uv_addr = input_phy_addr + jpeg_w*jpeg_h;	 
	//JpegInInfo.y_vir_addr = input_vir_addr;
	//JpegInInfo.uv_vir_addr = input_vir_addr + jpeg_w*jpeg_h;
	JpegInInfo.inputW = jpeg_w;
	JpegInInfo.inputH = jpeg_h;

	JpegInInfo.qLvl = quality/10;
	if (JpegInInfo.qLvl < 5) {
		JpegInInfo.qLvl = 5;
	}
	JpegInInfo.thumbqLvl = thumbquality /10;
	if (JpegInInfo.thumbqLvl < 5) {
		JpegInInfo.thumbqLvl = 5;
	}
	if(JpegInInfo.thumbqLvl  >10) {
		JpegInInfo.thumbqLvl = 9;
	}

	if(thumbwidth !=0 && thumbheight !=0) {
		JpegInInfo.doThumbNail = 1; 		 //insert thumbnail at APP0 extension
		JpegInInfo.thumbData = NULL;		 //if thumbData is NULL, do scale, the type above can not be 420_P or 422_UYVY
		JpegInInfo.thumbDataLen = -1;
		JpegInInfo.thumbW = thumbwidth;
		JpegInInfo.thumbH = thumbheight;
		JpegInInfo.y_vir_addr = (unsigned char*)input_vir_addr;
		JpegInInfo.uv_vir_addr = (unsigned char*)input_vir_addr+jpeg_w*jpeg_h;
	}else{	  
		JpegInInfo.doThumbNail = 0; 		 //insert thumbnail at APP0 extension	
	}

	Jpegfillexifinfo(&exifInfo,mPictureInfo);
	JpegInInfo.exifInfo =&exifInfo;

	if((longtitude!=-1)&& (latitude!=-1)&&(timestamp!=-1)&&(getMethod!=NULL)) {    
		Jpegfillgpsinfo(&gpsInfo,mPictureInfo);  
		memset(gpsprocessmethod,0,45);	 
		memcpy(gpsprocessmethod,ExifAsciiPrefix,8);   
		memcpy(gpsprocessmethod+8,getMethod,strlen(getMethod)+1);		   
		gpsInfo.GpsProcessingMethodchars = strlen(getMethod)+1+8;
		gpsInfo.GPSProcessingMethod  = gpsprocessmethod;
		LOGD("\nGpsProcessingMethodchars =%d",gpsInfo.GpsProcessingMethodchars);
		JpegInInfo.gpsInfo = &gpsInfo;
	} else {
		JpegInInfo.gpsInfo = NULL;
	}

	JpegOutInfo.outBufPhyAddr = output_phy_addr;
	JpegOutInfo.outBufVirAddr = (unsigned char*)output_vir_addr;
	JpegOutInfo.outBuflen = jpegbuf_size;
	JpegOutInfo.jpegFileLen = 0x00;
	JpegOutInfo.cacheflush= jpegEncFlushBufferCb;
	LOG1("JpegOutInfo.outBufPhyAddr:%x,JpegOutInfo.outBufVirAddr:%p,jpegbuf_size:%d",JpegOutInfo.outBufPhyAddr,JpegOutInfo.outBufVirAddr,jpegbuf_size);

	err = hw_jpeg_encode(&JpegInInfo, &JpegOutInfo);
	
	if ((err < 0) || (JpegOutInfo.jpegFileLen <=0x00)) {
		LOGE("%s(%d): hw_jpeg_encode Failed, err: %d  JpegOutInfo.jpegFileLen:0x%x\n",__FUNCTION__,__LINE__,
			err, JpegOutInfo.jpegFileLen);
		goto captureEncProcessPicture_exit;
	} else { 
		copyAndSendCompressedImage((void*)JpegOutInfo.outBufVirAddr,JpegOutInfo.jpegFileLen);
	}
captureEncProcessPicture_exit: 
 //destroy raw and jpeg buffer
    mRawBufferProvider->freeBuffer();
    mJpegBufferProvider->freeBuffer();
	if(err < 0) {
		LOGE("%s(%d) take picture erro!!!,",__FUNCTION__,__LINE__);
		if (mNotifyCb && (mMsgTypeEnabled & CAMERA_MSG_ERROR)) {						
			mNotifyCb(CAMERA_MSG_ERROR, CAMERA_ERROR_SERVER_DIED,0,mCallbackCookie);
		}
	} 


return ret;
}

int AppMsgNotifier::processPreviewDataCb(FramInfo_s* frame){
    int ret = 0;
	mDataCbLock.lock();
    if ((mMsgTypeEnabled & CAMERA_MSG_PREVIEW_FRAME) && mDataCb) {
        //compute request mem size
        int tempMemSize = 0;
        //request bufer
        camera_memory_t* tmpPreviewMemory = NULL;

        if (strcmp(mPreviewDataFmt,android::CameraParameters::PIXEL_FORMAT_RGB565) == 0) {
            tempMemSize = mPreviewDataW*mPreviewDataH*2;        
        } else if (strcmp(mPreviewDataFmt,android::CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
            tempMemSize = mPreviewDataW*mPreviewDataH*3/2;        
        } else if (strcmp(mPreviewDataFmt,android::CameraParameters::PIXEL_FORMAT_YUV422SP) == 0) {
            tempMemSize = mPreviewDataW*mPreviewDataH*2;        
        } else {
            LOGE("%s(%d): pixel format %s is unknow!",__FUNCTION__,__LINE__,mPreviewDataFmt);        
        }
        mDataCbLock.unlock();
	    tmpPreviewMemory = mRequestMemory(-1, tempMemSize, 1, NULL);
        if (tmpPreviewMemory) {
            //fill the tmpPreviewMemory
            arm_camera_yuv420_scale_arm(V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, (char*)(frame->vir_addr),
                (char*)tmpPreviewMemory->data,frame->frame_width, frame->frame_height,mPreviewDataW, mPreviewDataH,false);
            //if(cameraFormatConvert(frame->frame_fmt, V4L2_PIX_FMT_NV12, NULL,
            //		(char*)frame->vir_addr,(char*)tmpPreviewMemory->data,0,0,tempMemSize,
            //		frame->frame_width, frame->frame_height,frame->frame_width,mPreviewDataW, mPreviewDataH, mPreviewDataW,false)==0)
            //arm_yuyv_to_nv12(frame->frame_width, frame->frame_height,(char*)(frame->vir_addr), (char*)buf_vir);
            //fill the tmpPreviewMemory
            //callback
            mDataCb(CAMERA_MSG_PREVIEW_FRAME, tmpPreviewMemory, 0,NULL,mCallbackCookie);  
            //release buffer
            tmpPreviewMemory->release(tmpPreviewMemory);
        } else {
            LOGE("%s(%d): mPreviewMemory create failed",__FUNCTION__,__LINE__);
        }
    }
	else
	{
		mDataCbLock.unlock();
		LOG1("%s(%d): needn't to send preview datacb",__FUNCTION__,__LINE__);
	}
    return ret;
}
int AppMsgNotifier::processVideoCb(FramInfo_s* frame){
    int ret = 0,buf_phy = 0,buf_vir = 0,buf_index = -1;
    //get one available buffer
    if((buf_index = mVideoBufferProvider->getOneAvailableBuffer(&buf_phy,&buf_vir)) == -1){
        ret = -1;
        LOGE("%s(%d):no available buffer",__FUNCTION__,__LINE__);
        return ret;
    }

    mVideoBufferProvider->setBufferStatus(buf_index, 1);
    if((frame->frame_fmt == V4L2_PIX_FMT_NV12)){
        arm_camera_yuv420_scale_arm(V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV12, (char*)(frame->vir_addr),
            (char*)buf_vir,frame->frame_width, frame->frame_height,
            mRecordW, mRecordH,false);

        mVideoBufferProvider->flushBuffer(buf_index);
        mDataCbTimestamp(systemTime(CLOCK_MONOTONIC), CAMERA_MSG_VIDEO_FRAME, mVideoBufs[buf_index], 0, mCallbackCookie);
        LOG1("EncPicture:V4L2_PIX_FMT_NV12,arm_camera_yuv420_scale_arm");
    }
	/*//fill video buffer
	if(cameraFormatConvert(frame->frame_fmt, V4L2_PIX_FMT_NV12, NULL,
    (char*)frame->vir_addr,(char*)buf_vir,0,0,frame->frame_width*frame->frame_height*2,
    frame->frame_width, frame->frame_height,frame->frame_width,frame->frame_width, frame->frame_height,frame->frame_width,false)==0)
//	arm_yuyv_to_nv12(frame->frame_width, frame->frame_height,(char*)(frame->vir_addr), (char*)buf_vir);

    {
		//LOGD("%s(%d):send frame to video encode,index(%d)",__FUNCTION__,__LINE__,buf_index);
		mVideoBufferProvider->flushBuffer(buf_index);

		mDataCbTimestamp(systemTime(CLOCK_MONOTONIC), CAMERA_MSG_VIDEO_FRAME, mVideoBufs[buf_index], 0, mCallbackCookie);												    	
	}*/
	
    return ret;
}
void AppMsgNotifier::stopReceiveFrame()
{
    //pause event and enc thread
    flushPicture();
    //disable messeage receive
}

void AppMsgNotifier::dump()
{

}

void AppMsgNotifier::encProcessThread()
{
		bool loop = true;
		Message msg;
		int err = 0;
        int frame_used_flag = -1;
	
		LOG_FUNCTION_NAME
		while (loop) {
            memset(&msg,0,sizeof(msg));
			encProcessThreadCommandQ.get(&msg);
			
			switch (msg.command)
			{
				case EncProcessThread::CMD_ENCPROCESS_SNAPSHOT:
				{
					FramInfo_s *frame = (FramInfo_s*)msg.arg2;
					
					LOGD("%s(%d): receive CMD_SNAPSHOT_SNAPSHOT with buffer %d,mEncPictureNum=%d",__FUNCTION__,__LINE__, frame->frame_index,mEncPictureNum);

                    //set picture encode info
                    mEncPictureNum--;
					captureEncProcessPicture(frame);
					if(!mEncPictureNum){
						Mutex::Autolock lock(mPictureLock); 
						//mReceivePictureFrame = false;
						mRunningState &= ~STA_RECEIVE_PIC_FRAME;
					}
                    //return frame
                    frame_used_flag = (int)msg.arg3;
                    mFrameProvider->returnFrame(frame->frame_index,frame_used_flag);
					
					break;
				}
				case EncProcessThread::CMD_ENCPROCESS_PAUSE:

				{
                    if(msg.arg1)
                        ((Semaphore*)(msg.arg1))->Signal();
                   //wake up waiter
					break; 
				}
				case EncProcessThread::CMD_ENCPROCESS_EXIT:
				{
					LOGD("%s(%d): receive CMD_ENCPROCESS_EXIT",__FUNCTION__,__LINE__);
					loop = false;
                    if(msg.arg1)
                        ((Semaphore*)(msg.arg1))->Signal();

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

void AppMsgNotifier::eventThread()
{
	bool loop = true;
	Message msg;
	int index,err = 0;
    FramInfo_s *frame = NULL;
    int frame_used_flag = -1;
	LOG_FUNCTION_NAME
	while (loop) {
        memset(&msg,0,sizeof(msg));
		eventThreadCommandQ.get(&msg);
		switch (msg.command)
		{
          case CameraAppMsgThread::CMD_EVENT_PREVIEW_DATA_CB:
				frame = (FramInfo_s*)msg.arg2;
                processPreviewDataCb(frame);
                //return frame
                frame_used_flag = (int)msg.arg3;
                mFrameProvider->returnFrame(frame->frame_index,frame_used_flag);
                break;
          case CameraAppMsgThread::CMD_EVENT_VIDEO_ENCING:
                frame_used_flag = (int)msg.arg3;
				frame = (FramInfo_s*)msg.arg2;				
                LOG2("%s(%d):get new frame , index(%d),useflag(%d)",__FUNCTION__,__LINE__,frame->frame_index,frame_used_flag);

                processVideoCb(frame);
                //return frame
                mFrameProvider->returnFrame(frame->frame_index,frame_used_flag);
                break;
          case CameraAppMsgThread::CMD_EVENT_PAUSE:
				{
                    LOG1("%s(%d),receive CameraAppMsgThread::CMD_EVENT_PAUSE",__FUNCTION__,__LINE__);
                    if(msg.arg1)
						((Semaphore*)(msg.arg1))->Signal();
					//wake up waiter					
					break; 
				}
          case CameraAppMsgThread::CMD_EVENT_EXIT:
                {
        		loop = false;
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                break;
                }
          default:
                break;
		}
    }
	LOG_FUNCTION_NAME_EXIT
    return;

 }
}

