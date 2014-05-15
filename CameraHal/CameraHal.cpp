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

#include <utils/CallStack.h>
#include <camera/ICameraService.h>
#include <binder/IServiceManager.h>
#include <binder/IMemory.h>

#include <sys/stat.h>
#include <unistd.h>
#include <linux/fb.h>

#include "CameraIspAdapter.h"

#ifdef TARGET_RK29
#include "../libyuvtorgb/yuvtorgb.h"
#endif
extern rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];

namespace android {

/************************
接口实现有两种方式
1。接口由command线程负责具体实现
    有两种方式，一种异步，一种同步
    1.1  同步
        通过Semaphore
    1.2   异步
        无Semaphore
2. 接口由在接口函数本身中实现，不发送给command线程。

接口间的同步异步关系。
1.1 和2 型 为 同步关系
1.2 和2 型 为 异步关系
1.1 和1.2型 为 同步关系 ，同步由线程队列实现。

1.2型接口对于cameraservice来说是异步接口。
1.2 和 1.2 为同步关系，对于cameraservice来说是异步。

*********************/
CameraHal::CameraHal(int cameraId)
          :commandThreadCommandQ("commandCmdQ")
{
	LOG_FUNCTION_NAME
    
	{
        char trace_level[PROPERTY_VALUE_MAX];
        int level;

        property_get(CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY, trace_level, "0");

        sscanf(trace_level,"%d",&level);
        
        setTracerLevel(level);

	}

    mCamId = cameraId;
    mCamFd = -1;
    mCommandRunning = -1;
	mCameraStatus = 0;
    mDisplayAdapter = new DisplayAdapter();
    mEventNotifier = new AppMsgNotifier();

    #if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)
        mCamMemManager = new IonMemManager();
        LOGD("%s(%d): Camera Hal memory is alloced from ION device",__FUNCTION__,__LINE__);
	#elif(CONFIG_CAMERA_MEM == CAMERA_MEM_IONDMA)
		mCamMemManager = new IonDmaMemManager();
        LOGD("%s(%d): Camera Hal memory is alloced from ION device",__FUNCTION__,__LINE__);
    #elif(CONFIG_CAMERA_MEM == CAMERA_MEM_PMEM)
        if(access(CAMERA_PMEM_NAME, O_RDWR) < 0) {
            LOGE("%s(%d): %s isn't registered, CameraHal_Mem current configuration isn't support ION memory!!!",
                __FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
        } else {
            mCamMemManager = new PmemManager((char*)CAMERA_PMEM_NAME);
            LOGD("%s(%d): Camera Hal memory is alloced from %s device",__FUNCTION__,__LINE__,CAMERA_PMEM_NAME);
        }
    #endif
    
    mPreviewBuf = new PreviewBufferProvider(mCamMemManager);
    mVideoBuf = new BufferProvider(mCamMemManager);
    mRawBuf = new BufferProvider(mCamMemManager);
    mJpegBuf = new BufferProvider(mCamMemManager);


    if((strcmp(gCamInfos[cameraId].driver,"uvcvideo") == 0)) {
        LOGD("it is a uvc camera!");
        mCameraAdapter = new CameraUSBAdapter(cameraId);
    }
    else if(gCamInfos[cameraId].pcam_total_info->mHardInfo.mSensorInfo.mPhy.type == CamSys_Phy_Cif){
        LOGD("it is a isp soc camera");
        if(gCamInfos[cameraId].pcam_total_info->mHardInfo.mSensorInfo.mPhy.info.cif.fmt == CamSys_Fmt_Raw_10b)
            mCameraAdapter = new CameraIspSOCAdapter(cameraId);
        else
            mCameraAdapter = new CameraIspAdapter(cameraId);
    }
    else if(gCamInfos[cameraId].pcam_total_info->mHardInfo.mSensorInfo.mPhy.type == CamSys_Phy_Mipi){
        LOGD("it is a isp  camera");
        mCameraAdapter = new CameraIspAdapter(cameraId);
    }
    else{
        LOGD("it is a soc camera!");
        mCameraAdapter = new CameraSOCAdapter(cameraId);
    }

    //initialize
    mCameraAdapter->initialize();
    updateParameters(mParameters);
    mCameraAdapter->setPreviewBufProvider(mPreviewBuf);
    mCameraAdapter->setDisplayAdapterRef(*mDisplayAdapter);
    mCameraAdapter->setEventNotifierRef(*mEventNotifier);

    mDisplayAdapter->setFrameProvider(mCameraAdapter);
    
    mEventNotifier->setPictureRawBufProvider(mRawBuf);
    mEventNotifier->setPictureJpegBufProvider(mJpegBuf);
    mEventNotifier->setVideoBufProvider(mVideoBuf);
    mEventNotifier->setFrameProvider(mCameraAdapter);
    //command thread
    mCommandThread = new CommandThread(this);
	mCommandThread->run("CameraCmdThread", ANDROID_PRIORITY_URGENT_DISPLAY);
    
	LOG_FUNCTION_NAME_EXIT

    
}


void CameraHal::updateParameters(CameraParameters & tmpPara)
{
	LOG_FUNCTION_NAME

    tmpPara = mCameraAdapter->getParameters();

	LOG_FUNCTION_NAME_EXIT
}

//release resouse
CameraHal::~CameraHal()
{
	LOG_FUNCTION_NAME
    if(mDisplayAdapter){
        delete mDisplayAdapter;
        mDisplayAdapter = NULL;
    }
    if(mEventNotifier){
        delete mEventNotifier;
        mEventNotifier = NULL;
    }
    if(mCameraAdapter){
        delete mCameraAdapter;
        mCameraAdapter = NULL;
    }
    if(mPreviewBuf){
        delete mPreviewBuf;
        mPreviewBuf = NULL;
    }
    if(mVideoBuf){
        delete mVideoBuf;
        mVideoBuf = NULL;
    }
    if(mRawBuf){
        delete mRawBuf;
        mRawBuf = NULL; 
    }
    if(mJpegBuf){
        delete mJpegBuf;
        mJpegBuf = NULL;
    }
    if(mCamMemManager){
        delete mCamMemManager;
        mCamMemManager =NULL;
    }
	if(mCommandThread != NULL){
        Message msg;  
        Semaphore sem;
        msg.command = CMD_EXIT;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_EXIT_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_EXIT_DONE)
			LOG1("exit command OK.");
   		mCommandThread->requestExitAndWait();
    	mCommandThread.clear();
	}
    
    LOGD("CameraHal destory success");
    LOG_FUNCTION_NAME_EXIT
}
void CameraHal::release()
{
	LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);  
	LOG_FUNCTION_NAME_EXIT
}


int CameraHal::setPreviewWindow(struct preview_stream_ops *window)
{
    LOG_FUNCTION_NAME  
    Message msg;  
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_SET_PREVIEW_WINDOW;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        msg.arg2 = (void*)window;
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_SET_PREVIEW_WINDOW_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_SET_PREVIEW_WINDOW_DONE)
			LOGD("set preview window OK.");		
    }
	LOG_FUNCTION_NAME_EXIT
    return 0;
}

//async
int CameraHal::startPreview()
{
    LOG_FUNCTION_NAME
    Message msg;    
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_START;
        msg.arg1  = NULL;
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_PREVIEW_START_PREPARE, 1);
    }
   // mPreviewCmdReceived = true;
   setCamStatus(STA_PREVIEW_CMD_RECEIVED, 1);
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR ;
}

//MUST SYNC
void CameraHal::stopPreview()
{
    LOG_FUNCTION_NAME  
    Message msg; 
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_STOP;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_PREVIEW_STOP_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_PREVIEW_STOP_DONE)
			LOGD("stop preview OK.");
    }
    //mPreviewCmdReceived = false;
	setCamStatus(STA_PREVIEW_CMD_RECEIVED, 0);
    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::setCallbacks(camera_notify_callback notify_cb,
            camera_data_callback data_cb,
            camera_data_timestamp_callback data_cb_timestamp,
            camera_request_memory get_memory,
            void *user)                                   
{
    LOG_FUNCTION_NAME  
    Mutex::Autolock lock(mLock);
    mEventNotifier->setCallbacks(notify_cb, data_cb,data_cb_timestamp,get_memory,user);
    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::enableMsgType(int32_t msgType)
{

    LOG_FUNCTION_NAME
    if(msgType & (CAMERA_MSG_PREVIEW_FRAME)){
        //mParameters is sync
        int w,h;
        const char * fmt=  mParameters.getPreviewFormat();
		mParameters.getPreviewSize(&w, &h); 
        mEventNotifier->setPreviewDataCbRes(w, h, fmt);
    }
    mEventNotifier->enableMsgType(msgType);
    LOG_FUNCTION_NAME_EXIT
}

void CameraHal::disableMsgType(int32_t msgType)
{
    LOG_FUNCTION_NAME
    mEventNotifier->disableMsgType(msgType);
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::msgTypeEnabled(int32_t msgType)
{
    LOG_FUNCTION_NAME
    return  mEventNotifier->msgEnabled(msgType);
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::autoFocus()
{
    LOG_FUNCTION_NAME
    Message msg; 
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    
    if ((mCommandThread != NULL)) {
        msg.command = CMD_AF_START;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_AF_START_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_AF_START_DONE)
			LOG1("AF command OK.");
    }
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}

int CameraHal::cancelAutoFocus()
{
    LOG_FUNCTION_NAME
    Message msg; 
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_AF_CANCEL;
        sem.Create();
        msg.arg1 = NULL;
        commandThreadCommandQ.put(&msg);
     
    }

    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}
int CameraHal::previewEnabled()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    LOG_FUNCTION_NAME_EXIT
    return (mCameraStatus&STA_PREVIEW_CMD_RECEIVED);
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
    int err=NO_ERROR,prevStatus = -1;
    CameraParameters params;
    int recordW,recordH;
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    //get preview status
    //if(mPreviewCmdReceived)
    if(mCameraStatus&STA_PREVIEW_CMD_RECEIVED)
       // prevStatus = mCameraAdapter->getCurPreviewState(&recordW, &recordH);
        prevStatus = mCameraAdapter->getCurVideoSize(&recordW, &recordH);
    if(prevStatus == -1){
        err = -1;
        return err;
    }
    #if 1
    //set picture size
    mParameters.setPictureSize(recordW,recordH);
    setParametersUnlock(mParameters);
    updateParameters(mParameters);
    #endif
    
    //notify event
    mEventNotifier->startRecording(recordW,recordH);
    //mRecordRunning = true;
	setCamStatus(STA_RECORD_RUNNING, 1);
    LOG_FUNCTION_NAME_EXIT
    return err;
}

void CameraHal::stopRecording()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    mEventNotifier->stopRecording();
    //mRecordRunning = false;
	setCamStatus(STA_RECORD_RUNNING, 0);
    LOG_FUNCTION_NAME_EXIT
}

int CameraHal::recordingEnabled()
{
    LOG_FUNCTION_NAME
    LOG_FUNCTION_NAME_EXIT
    Mutex::Autolock lock(mLock);
    //return mRecordRunning;
	return (mCameraStatus&STA_RECORD_RUNNING);
}
void CameraHal::releaseRecordingFrame(const void *opaque)
{
   // LOG_FUNCTION_NAME
    mEventNotifier->releaseRecordingFrame(opaque);
  //  LOG_FUNCTION_NAME_EXIT
}

int CameraHal::takePicture()
{
    LOG_FUNCTION_NAME  
    Message msg;    
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_CONTINUOS_PICTURE;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_CONTINUOS_PICTURE_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_CONTINUOS_PICTURE_DONE)
			LOGD("take picture command OK.");
    }
    //when back to preview status,cameraservice will call startpreview.
    //mPreviewCmdReceived = false;
	setCamStatus(STA_PREVIEW_CMD_RECEIVED, 0);
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}

int CameraHal::cancelPicture()
{
    LOG_FUNCTION_NAME  
    Message msg;    
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_PREVIEW_CAPTURE_CANCEL;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_PREVIEW_CAPTURE_CANCEL_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_PREVIEW_CAPTURE_CANCEL_DONE)
			LOGD("cancel picture OK.");
    }
    LOG_FUNCTION_NAME_EXIT
    return NO_ERROR;
}

int CameraHal::setParameters(const char* parameters)
{
    CameraParameters params;
    String8 str_params(parameters);
    
    params.unflatten(str_params);
    return setParameters(params);
}

//sync
int CameraHal::setParameters(const CameraParameters &params_set)
{
    int err = 0;
    LOG_FUNCTION_NAME
    //get preview status
    //if(mPreviewCmdReceived){
    if(mCameraStatus&STA_PREVIEW_CMD_RECEIVED){
        //preview has been started,some parameter can't change

    }

    Message msg;    
    Semaphore sem;
    Mutex::Autolock lock(mLock);
    if ((mCommandThread != NULL)) {
        msg.command = CMD_SET_PARAMETERS;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        msg.arg2 = (void*)(&params_set);
        commandThreadCommandQ.put(&msg);
		setCamStatus(CMD_SET_PARAMETERS_PREPARE, 1);
        if(msg.arg1){
            sem.Wait();
        }
		if(mCameraStatus&CMD_SET_PARAMETERS_DONE)
			LOG1("set parameters OK.");
    }
    LOG_FUNCTION_NAME_EXIT
    return err;
}


int CameraHal::setParametersUnlock(const CameraParameters &params_set)
{
    int err = 0;
    LOG_FUNCTION_NAME
    //get preview status
    //if(mPreviewCmdReceived){
    if(mCameraStatus&STA_PREVIEW_CMD_RECEIVED){
        //preview has been started,some parameter can't change

    }

    Message msg;    
    Semaphore sem;
    if ((mCommandThread != NULL)) {
        msg.command = CMD_SET_PARAMETERS;
        sem.Create();
        msg.arg1 = (void*)(&sem);
        msg.arg2 = (void*)(&params_set);
        commandThreadCommandQ.put(&msg);
        if(msg.arg1){
            sem.Wait();
        }
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

    return ret;
}


int CameraHal::dump(int fd)
{   
    int i;
    char trace_level[PROPERTY_VALUE_MAX];
    int level;

    property_get(CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY, trace_level, "0");

    sscanf(trace_level,"%d",&level);
    
    setTracerLevel(level);
    
    commandThreadCommandQ.dump();
    if(mCameraAdapter)
        mCameraAdapter->dump(mCamId);
    if(mDisplayAdapter)
        mDisplayAdapter->dump();
    if(mEventNotifier)
        mEventNotifier->dump();

    
    return 0;
}
int CameraHal::getCameraFd()
{
    Mutex::Autolock lock(mLock);
    
    return mCameraAdapter->getCameraFd();  
}


int CameraHal::selectPreferedDrvSize(int *width,int * height,bool is_capture)
{
    return 0;
}
int CameraHal::fillPicturInfo(picture_info_s& picinfo)
{
	mParameters.getPictureSize(&picinfo.w, &picinfo.h);				 
	picinfo.quality= mParameters.getInt("jpeg-quality");
    picinfo.rotation = strtol(mParameters.get(CameraParameters::KEY_ROTATION),0,0);
	picinfo.thumbquality= strtol(mParameters.get(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY),0,0);
	picinfo.thumbwidth= strtol(mParameters.get(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH),0,0);
	picinfo.thumbheight= strtol(mParameters.get(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT),0,0);

    // gps longitude
    const char *new_gps_longitude_str = mParameters.get(CameraParameters::KEY_GPS_LONGITUDE);    
    if (new_gps_longitude_str) {
        picinfo.longtitude= strtod(new_gps_longitude_str,NULL);
    } else {
        picinfo.longtitude = -1;
    }

    // gps altitude
    const char *new_gps_altitude_str = mParameters.get(CameraParameters::KEY_GPS_ALTITUDE);
    if (new_gps_altitude_str) {
        picinfo.latitude= strtod(new_gps_altitude_str,NULL);
    } else {
        picinfo.latitude = -1;
    }

    // gps timestamp
    const char *new_gps_timestamp_str = mParameters.get(CameraParameters::KEY_GPS_TIMESTAMP);
    if (new_gps_timestamp_str) {
        picinfo.timestamp= strtol(new_gps_timestamp_str,0,0);
    } else {
        picinfo.timestamp = -1;
    }

	char* getMethod = (char*)mParameters.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);//getMethod : len <= 32
	if(getMethod)
        strcpy(picinfo.getMethod,getMethod);
    //focus length
    picinfo.focalen = strtol(mParameters.get(CameraParameters::KEY_FOCAL_LENGTH),0,0);
	//flash 
	if (mParameters.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES) && mParameters.get(CameraParameters::KEY_FLASH_MODE)) {
		if (!strcmp(CameraParameters::FLASH_MODE_OFF, mParameters.get(CameraParameters::KEY_FLASH_MODE))) {
			picinfo.flash = 0;
		} else {
			picinfo.flash = 0;
		}	 
	} else {
		picinfo.flash = 0;
	}
	//white balance 
	if (mParameters.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE) && mParameters.get(CameraParameters::KEY_WHITE_BALANCE)) {
		if (!strcmp(CameraParameters::WHITE_BALANCE_AUTO, mParameters.get(CameraParameters::KEY_WHITE_BALANCE))) {
			picinfo.whiteBalance = 0;
		} else {
			picinfo.whiteBalance = 1;
		}	 
	} else {
		picinfo.whiteBalance = 0;
	}
    picinfo.num = 1;
	picinfo.fmt = V4L2_PIX_FMT_NV12;
    return 0;
}
void CameraHal::commandThread()
{
    Message msg;
    bool  shouldLive = true;
    bool has_message;
    int err = 0,tmp_arg1;
   struct v4l2_control control;
   struct v4l2_queryctrl hdr;
    picture_info_s picinfo;
    int pic_num = 1;
    int prevStatus = -1,drv_w,drv_h,picture_w,picture_h;
    int app_previw_w = 0,app_preview_h = 0;
    LOG_FUNCTION_NAME

    while(shouldLive) {
get_command:
        //mCommandRunning = -1;
        memset(&msg,0,sizeof(msg));
        commandThreadCommandQ.get(&msg);

        switch(msg.command)
        {
            case CMD_PREVIEW_START:
            { 
                LOGD("%s(%d):receive CMD_PREVIEW_START",__FUNCTION__,__LINE__);

                //1, need to stop or pause display ?
                if(mDisplayAdapter->getDisplayStatus() == DisplayAdapter::STA_DISPLAY_RUNNING){
                    err=mDisplayAdapter->pauseDisplay();
					if(err != -1)
						setCamStatus(STA_DISPLAY_PAUSE, 1);
                }
                //2. current preview status ?
                mParameters.getPreviewSize(&app_previw_w,&app_preview_h);
                prevStatus = mCameraAdapter->getCurPreviewState(&drv_w,&drv_h);
                int prefered_w = app_previw_w, prefered_h = app_preview_h;
                selectPreferedDrvSize(&prefered_w,&prefered_h,false);
                if(prevStatus){
                    //get preview size
                    if((prefered_w != drv_w) || (prefered_h != drv_h)){
                        //need to stop preview.
                        err=mCameraAdapter->stopPreview();
						if(err != 0)
							goto PREVIEW_START_OUT;
                        //set new drv size;
                        drv_w = prefered_w;
                        drv_h = prefered_h;
                        err=mCameraAdapter->startPreview(app_previw_w,app_preview_h,drv_w, drv_h, 0, false);
						if(err != 0)
							goto PREVIEW_START_OUT;
                    }
                        
                }else{
                    drv_w = prefered_w;
                    drv_h = prefered_h;
                    //selet a proper preview size.
                    err=mCameraAdapter->startPreview(app_previw_w,app_preview_h,drv_w, drv_h, 0, false);
					if(err != 0)
						goto PREVIEW_START_OUT;
                }
                if(mDisplayAdapter->getPreviewWindow())
                {
                 	err = mDisplayAdapter->startDisplay(app_previw_w, app_preview_h);
					if(err != -1)
						setCamStatus(STA_DISPLAY_RUNNING, 1);
					else
						goto PREVIEW_START_OUT;
            	}
				
				setCamStatus(CMD_PREVIEW_START_DONE, 1);
				PREVIEW_START_OUT:	
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOGD("%s(%d): CMD_PREVIEW_START out",__FUNCTION__,__LINE__);
                break;
            }

            case CMD_PREVIEW_STOP:
            {
                LOGD("%s(%d):receive CMD_PREVIEW_STOP",__FUNCTION__,__LINE__);
                //pause display
                if(mDisplayAdapter->getDisplayStatus() == DisplayAdapter::STA_DISPLAY_RUNNING){
                    err =mDisplayAdapter->pauseDisplay();
					if(err != -1)
						setCamStatus(STA_DISPLAY_PAUSE, 1);
					else
						goto CMD_PREVIEW_STOP_OUT;
                }
                //stop eventnotify
                mEventNotifier->stopReceiveFrame();
                //stop preview
                err=mCameraAdapter->stopPreview();
				if(err != 0)
					goto CMD_PREVIEW_STOP_OUT;
                //wake up wait thread.
                
				setCamStatus(CMD_PREVIEW_STOP_DONE, 1);  
		CMD_PREVIEW_STOP_OUT:			
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOGD("%s(%d): CMD_PREVIEW_STOP out",__FUNCTION__,__LINE__);
                break;
            }
            case CMD_SET_PREVIEW_WINDOW:
            {
                LOGD("%s(%d):receive CMD_SET_PREVIEW_WINDOW",__FUNCTION__,__LINE__);
                mParameters.getPreviewSize(&app_previw_w,&app_preview_h);
                mDisplayAdapter->setPreviewWindow((struct preview_stream_ops *)msg.arg2);
                prevStatus = mCameraAdapter->getCurPreviewState(&drv_w,&drv_h);

                if ((mDisplayAdapter->getPreviewWindow()) && prevStatus) {
                    err=mDisplayAdapter->startDisplay(app_previw_w, app_preview_h);
					if(err != -1)
						setCamStatus(STA_DISPLAY_RUNNING, 1);
                } else {
                    LOG1("%s(%d): not start display now",__FUNCTION__,__LINE__);
                }
				
				setCamStatus(CMD_SET_PREVIEW_WINDOW_DONE, 1);				
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOGD("%s(%d): CMD_SET_PREVIEW_WINDOW out",__FUNCTION__,__LINE__);
                break;
            }
            case CMD_SET_PARAMETERS:
            {
                LOG1("%s(%d): receive CMD_SET_PARAMETERS", __FUNCTION__,__LINE__);
                //set parameters
                CameraParameters* para = (CameraParameters*)msg.arg2;
                mCameraAdapter->setParameters(*para);
                //update parameters
                updateParameters(mParameters);
				
				setCamStatus(CMD_SET_PARAMETERS_DONE, 1);					
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOG1("%s(%d): CMD_SET_PARAMETERS out",__FUNCTION__,__LINE__);
                break;
                    
            }
            case CMD_PREVIEW_CAPTURE_CANCEL:
            {
                LOGD("%s(%d): receive CMD_PREVIEW_CAPTURE_CANCEL", __FUNCTION__,__LINE__);
                mEventNotifier->flushPicture();
				setCamStatus(CMD_PREVIEW_CAPTURE_CANCEL_DONE, 1);			
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOGD("%s(%d): CMD_PREVIEW_CAPTURE_CANCEL out",__FUNCTION__,__LINE__);
                break; 
            }
    	    case CMD_CONTINUOS_PICTURE:
             {
    			LOGD("%s(%d): receive CMD_CONTINUOS_PICTURE", __FUNCTION__,__LINE__);
				mParameters.getPictureSize(&picture_w, &picture_h);
                //need to pause display(is recording? is continuous picture ?) 
                //if(!mRecordRunning && (pic_num == 1)){
                if(!(mCameraStatus&STA_RECORD_RUNNING) && (pic_num == 1)){
                    err=mDisplayAdapter->pauseDisplay();
					if(err != -1)
						setCamStatus(STA_DISPLAY_PAUSE, 1);
                }
                //need to resize preview size ?
                int prefered_w = picture_w, prefered_h = picture_h;
                selectPreferedDrvSize(&prefered_w,&prefered_h,true);
                prevStatus = mCameraAdapter->getCurPreviewState(&drv_w,&drv_h);
                if(prevStatus){
				    //get preview size
				    //if(mRecordRunning){
                    if(mCameraStatus&STA_RECORD_RUNNING){
                       LOGE("%s(%d):not support set picture size when recording.",__FUNCTION__,__LINE__);
                	} else if((prefered_w != drv_w) || (prefered_h != drv_h)){
                        //need to stop preview.
                        err=mDisplayAdapter->pauseDisplay();
						if(err != -1)
							setCamStatus(STA_DISPLAY_PAUSE, 1);

						err =mCameraAdapter->stopPreview();
						if(err != 0)
							goto CMD_CONTINUOS_PICTURE_OUT;
						
                        //set new drv size;
                        drv_w = prefered_w;
                        drv_h = prefered_h;
                        err=mCameraAdapter->startPreview(picture_w,picture_h,drv_w, drv_h, 0, true);
						if(err != 0)
							goto CMD_CONTINUOS_PICTURE_OUT;
                        if(pic_num > 1)
                        {
                        	err=mDisplayAdapter->startDisplay(picture_w,picture_h);
							if(err != -1)
								setCamStatus(STA_DISPLAY_RUNNING, 1);
							else
								goto CMD_CONTINUOS_PICTURE_OUT;
                    	}
                    }
                        
                }else{
                    //selet a proper preview size.
                    LOGD("%s(%d):WARNING,take pic before start preview!",__FUNCTION__,__LINE__);
                    err=mCameraAdapter->startPreview(picture_w,picture_h,drv_w, drv_h, 0, true);
					if(err != 0)
						goto CMD_CONTINUOS_PICTURE_OUT;
                }
                //take picture
                fillPicturInfo(picinfo);
                mEventNotifier->takePicture(picinfo);

				setCamStatus(CMD_CONTINUOS_PICTURE_DONE, 1);
			CMD_CONTINUOS_PICTURE_OUT:				
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                LOGD("%s(%d): CMD_CONTINUOS_PICTURE out",__FUNCTION__,__LINE__);
    			break;
    	    }
            case CMD_AF_START:
            {
                LOGD("%s(%d): receive CMD_AF_START", __FUNCTION__,__LINE__);
                mCameraAdapter->autoFocus();
                
				setCamStatus(CMD_AF_START_DONE, 1);
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();				
				LOGD("%s(%d): exit receive CMD_AF_START", __FUNCTION__,__LINE__);
                break;
            }            
            case CMD_AF_CANCEL:
            {
                LOGD("%s(%d): receive CMD_AF_CANCEL", __FUNCTION__,__LINE__);
				
				setCamStatus(CMD_AF_CANCEL_DONE, 1);					
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
                break;
            }
            
            case CMD_EXIT:
            {
                LOGD("%s(%d): receive CMD_EXIT", __FUNCTION__,__LINE__);
                shouldLive = false;
				
				setCamStatus(CMD_EXIT_DONE, 1);			
                if(msg.arg1)
                    ((Semaphore*)(msg.arg1))->Signal();
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

int CameraHal::checkCamStatus(int cmd)
{
	int err = 0;
	switch(cmd)
	{
		case CMD_PREVIEW_START:
		{
			break;
		}
		case CMD_PREVIEW_STOP:
		{
			if((mCameraStatus&STA_DISPLAY_STOP))
				err = -1;			
			break;
		}
		case CMD_SET_PREVIEW_WINDOW:
		{
			if((mCameraStatus&STA_DISPLAY_RUNNING) || (mCameraStatus&STA_RECORD_RUNNING))
				err = -1;
			break;
		}		
		case CMD_SET_PARAMETERS:
		{
			break;			
		}	
		case CMD_PREVIEW_CAPTURE_CANCEL:
		{
			break;			
		}
		case CMD_CONTINUOS_PICTURE:
		{
			if((mCameraStatus&STA_DISPLAY_RUNNING)==0x0)
				err = -1;			
			break;			
		}	
		case CMD_AF_START:
		{
			break;			
		}	
		case CMD_AF_CANCEL:
		{
			break;			
		}			
	}


	return err;
}
//type:1-set, 0-clear
void CameraHal::setCamStatus(int status, int type)
{
	if(type)
	{
		if(status&CMD_PREVIEW_START_PREPARE)
		{
			mCameraStatus &= ~CMD_PREVIEW_START_MASK;
			mCameraStatus |= CMD_PREVIEW_START_PREPARE;
		}
		if(status&CMD_PREVIEW_START_DONE)
		{
			mCameraStatus &= ~CMD_PREVIEW_START_MASK;
			mCameraStatus |= CMD_PREVIEW_START_DONE;
		}
		if(status&CMD_PREVIEW_STOP_PREPARE)
		{
			mCameraStatus &= ~CMD_PREVIEW_STOP_MASK;
			mCameraStatus |= CMD_PREVIEW_STOP_PREPARE;
		}
		if(status&CMD_PREVIEW_STOP_DONE)
		{
			mCameraStatus &= ~CMD_PREVIEW_STOP_MASK;
			mCameraStatus |= CMD_PREVIEW_STOP_DONE;
		}
		if(status&CMD_SET_PREVIEW_WINDOW_PREPARE)
		{
			mCameraStatus &= ~CMD_SET_PREVIEW_WINDOW_MASK;
			mCameraStatus |= CMD_SET_PREVIEW_WINDOW_PREPARE;
		}
		if(status&CMD_SET_PREVIEW_WINDOW_DONE)
		{
			mCameraStatus &= ~CMD_SET_PREVIEW_WINDOW_MASK;
			mCameraStatus |= CMD_SET_PREVIEW_WINDOW_DONE;
		}

		if(status&CMD_SET_PARAMETERS_PREPARE)
		{
			mCameraStatus &= ~CMD_SET_PARAMETERS_MASK;
			mCameraStatus |= CMD_SET_PARAMETERS_PREPARE;
		}
		if(status&CMD_SET_PARAMETERS_DONE)
		{
			mCameraStatus &= ~CMD_SET_PARAMETERS_MASK;
			mCameraStatus |= CMD_SET_PARAMETERS_DONE;
		}

		if(status&CMD_PREVIEW_CAPTURE_CANCEL_PREPARE)
		{
			mCameraStatus &= ~CMD_PREVIEW_CAPTURE_CANCEL_MASK;
			mCameraStatus |= CMD_PREVIEW_CAPTURE_CANCEL_PREPARE;
		}
		if(status&CMD_PREVIEW_CAPTURE_CANCEL_DONE)
		{
			mCameraStatus &= ~CMD_PREVIEW_CAPTURE_CANCEL_MASK;
			mCameraStatus |= CMD_PREVIEW_CAPTURE_CANCEL_DONE;
		}

		if(status&CMD_CONTINUOS_PICTURE_PREPARE)
		{
			mCameraStatus &= ~CMD_CONTINUOS_PICTURE_MASK;
			mCameraStatus |= CMD_CONTINUOS_PICTURE_PREPARE;
		}
		if(status&CMD_CONTINUOS_PICTURE_DONE)
		{
			mCameraStatus &= ~CMD_CONTINUOS_PICTURE_MASK;
			mCameraStatus |= CMD_CONTINUOS_PICTURE_DONE;
		}

		if(status&CMD_AF_START_PREPARE)
		{
			mCameraStatus &= ~CMD_AF_START_MASK;
			mCameraStatus |= CMD_AF_START_PREPARE;
		}
		if(status&CMD_AF_START_DONE)
		{
			mCameraStatus &= ~CMD_AF_START_MASK;
			mCameraStatus |= CMD_AF_START_DONE;
		}

		if(status&CMD_AF_CANCEL_PREPARE)
		{
			mCameraStatus &= ~CMD_AF_CANCEL_MASK;
			mCameraStatus |= CMD_AF_CANCEL_PREPARE;
		}
		if(status&CMD_AF_CANCEL_DONE)
		{
			mCameraStatus &= ~CMD_AF_CANCEL_MASK;
			mCameraStatus |= CMD_AF_CANCEL_DONE;
		}

		if(status&CMD_EXIT_PREPARE)
		{
			mCameraStatus &= ~CMD_EXIT_MASK;
			mCameraStatus |= CMD_EXIT_PREPARE;
		}
		if(status&CMD_EXIT_DONE)
		{
			mCameraStatus &= ~CMD_EXIT_MASK;
			mCameraStatus |= CMD_EXIT_DONE;
		}

		if(status&STA_RECORD_RUNNING)
		{
			mCameraStatus |= STA_RECORD_RUNNING;
		}

		if(status&STA_PREVIEW_CMD_RECEIVED)
		{
			mCameraStatus |= STA_PREVIEW_CMD_RECEIVED;
		}

		if(status&STA_DISPLAY_RUNNING)
		{
			mCameraStatus &= ~STA_DISPLAY_MASK;
			mCameraStatus |= STA_DISPLAY_RUNNING;
		}
		if(status&STA_DISPLAY_PAUSE)
		{
			mCameraStatus &= ~STA_DISPLAY_MASK;
			mCameraStatus |= STA_DISPLAY_PAUSE;
		}
		if(status&STA_DISPLAY_STOP)
		{
			mCameraStatus &= ~STA_DISPLAY_MASK;
			mCameraStatus |= STA_DISPLAY_STOP;
		}
	}
	else
	{
		if(status&STA_RECORD_RUNNING)
		{
			mCameraStatus &= ~STA_RECORD_RUNNING;
		}

		if(status&STA_PREVIEW_CMD_RECEIVED)
		{
			mCameraStatus &= ~STA_PREVIEW_CMD_RECEIVED;
		}		
	}
}
}; // namespace android

