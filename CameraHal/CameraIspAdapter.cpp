#include "CameraIspAdapter.h"
#include "cam_api/halholder.h"

namespace android{
#define LOG_TAG "CameraHal_CameraIspAdapter"
static volatile int32_t gLogLevel = 2;

#ifdef ALOGD_IF
#define LOG1(...) ALOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) ALOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#else
#define LOG1(...) LOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) LOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#endif

#define LOG_FUNCTION_NAME           LOG1("%s Enter", __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT      LOG1("%s Exit ", __FUNCTION__);


/******************************************************************************
 * MainWindow_AfpsResChangeCb
 *****************************************************************************/
void CameraIspAdapter_AfpsResChangeCb( void *ctx )
{
    CameraIspAdapter *mainwindow = (CameraIspAdapter*)ctx;

    if (mainwindow)
    {
        mainwindow->AfpsResChangeCb();
    }
}



/******************************************************************************
 * MainWindow::AfpsResChangeCb
 *****************************************************************************/
void CameraIspAdapter::AfpsResChangeCb()
{
   // emit resChanged();
   LOGD("%s(%d): success!",__func__,__LINE__);
}


int CameraIspAdapter::DEFAULTPREVIEWWIDTH = 800;
int CameraIspAdapter::DEFAULTPREVIEWHEIGHT = 600;
int CameraIspAdapter::preview_frame_inval = 1;





CameraIspAdapter::CameraIspAdapter(int cameraId)
                    :CameraAdapter(cameraId),
                    m_camDevice(NULL),
                    mSensorItfCur(0)
{
    LOG_FUNCTION_NAME
}
CameraIspAdapter::~CameraIspAdapter()
{
    LOG_FUNCTION_NAME
	preview_frame_inval = 0;
    if(m_camDevice){
        disconnectCamera();
        delete m_camDevice;
        m_camDevice = NULL;
        if(HalHolder::m_halHolder){
            delete HalHolder::m_halHolder;
            HalHolder::m_halHolder = NULL;
        }
    }
}
int CameraIspAdapter::cameraCreate(int cameraId)
{
    LOG_FUNCTION_NAME
	rk_cam_total_info *pCamInfo = gCamInfos[cameraId].pcam_total_info;
	char* dev_filename = pCamInfo->mHardInfo.mSensorInfo.mCamsysDevPath;
	int mipiLaneNum = 0;
	int i =0;
	unsigned int ispVersion;
	
	for(i=0; i<4; i++){
		mipiLaneNum += (pCamInfo->mHardInfo.mSensorInfo.mPhy.info.mipi.data_en_bit>>i)&0x01;
	}
	
    m_camDevice = new CamDevice( HalHolder::handle(dev_filename), CameraIspAdapter_AfpsResChangeCb, (void*)this ,NULL, mipiLaneNum);
	if(m_camDevice){
		m_camDevice->getIspVersion(&ispVersion);
		LOGD("%s(%d)  camerahal.ispversion(%x) libisp.ispversion(%x)\n", __FUNCTION__,__LINE__,CONFIG_SILICONIMAGE_LIBISP_VERSION,ispVersion);
		if(ispVersion != CONFIG_SILICONIMAGE_LIBISP_VERSION){
			;
		}
	}
	//load sensor
    loadSensor( cameraId);
    //open image
    //openImage("/system/lib/libisp_isi_drv_OV8825.so");
    return 0;

}
int CameraIspAdapter::cameraDestroy()
{
    LOG_FUNCTION_NAME
    if(m_camDevice){
        disconnectCamera();
        delete m_camDevice;
        m_camDevice = NULL;
        if(HalHolder::m_halHolder){
            delete HalHolder::m_halHolder;
            HalHolder::m_halHolder = NULL;
        }
    }
    LOG_FUNCTION_NAME_EXIT
    return 0;
}

void CameraIspAdapter::setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h)
{
    CamEngineWindow_t dcWin;
    if(((width_sensor*10/height_sensor) != (preview_w*10/preview_h))){
        int ratio = ((width_sensor*10/preview_w) >= (height_sensor*10/preview_h))?(height_sensor*10/preview_h):(width_sensor*10/preview_w);
        dcWin.width = ((ratio*preview_w/10) & ~0x1);
        dcWin.height = ((ratio*preview_h/10) & ~0x1);
        dcWin.hOffset =(ABS(width_sensor-dcWin.width ))>>1;
        dcWin.vOffset = (ABS(height_sensor-dcWin.height))>>1;

        //dcWin.width = 1440;
        //dcWin.height = 1080;
       // dcWin.hOffset = 240;
       // dcWin.vOffset = 0;
        
    }else{
        dcWin.width = width_sensor;
        dcWin.height = height_sensor;
        dcWin.hOffset = 0;
        dcWin.vOffset = 0;
    }
    m_camDevice->previewSetup_ex( dcWin, preview_w, preview_h,
                                CAMERIC_MI_DATAMODE_YUV422,CAMERIC_MI_DATASTORAGE_INTERLEAVED,(bool_t)true);


}
status_t CameraIspAdapter::startPreview(int preview_w,int preview_h,int w, int h, int fmt,bool is_capture)
{
    LOG_FUNCTION_NAME
	bool err_af;
	bool err;
	bool avail = false;
	
    if ( ( !m_camDevice->hasSensor() ) &&
         ( !m_camDevice->hasImage()  ) ){
          goto startPreview_end;
    	}

    //need to change resolution ?
    if((preview_w != mCamPreviewW) ||(preview_h != mCamPreviewH)
        || (w != mCamDrvWidth) || (h != mCamDrvHeight)){

        //change resolution
        //get sensor res
        int width_sensor = 0,height_sensor = 0;
        uint32_t resMask;
        CamEnginePathConfig_t mainPathConfig ,selfPathConfig;

        m_camDevice->getPreferedSensorRes(preview_w, preview_h, &width_sensor, &height_sensor,&resMask);
		LOGD("-------width_sensor=%d,height_sensor=%d---------",width_sensor,height_sensor);
        //stop streaming
        if(-1 == stop())
			goto startPreview_end;

        //need to change sensor resolution ?
        if((width_sensor != mCamDrvWidth) || (height_sensor != mCamDrvHeight)){
            m_camDevice->changeResolution(resMask,false);
        }
        //reset dcWin,output width(data path)
            //get dcWin
        setupPreview(width_sensor,height_sensor,preview_w,preview_h);
		
        m_camDevice->getPathConfig(CHAIN_MASTER,CAM_ENGINE_PATH_MAIN,mainPathConfig);
        m_camDevice->getPathConfig(CHAIN_MASTER,CAM_ENGINE_PATH_SELF,selfPathConfig);
        m_camDevice->setPathConfig( CHAIN_MASTER, mainPathConfig, selfPathConfig  );

		//start streaming
        if(-1 == start())
			goto startPreview_end;

        //
        mCamDrvWidth = width_sensor;
        mCamDrvHeight = height_sensor;
        mCamPreviewH = preview_h;
        mCamPreviewW = preview_w;
        LOGD("%s:sensor(%dx%d),user(%dx%d))",__func__,mCamDrvWidth,mCamDrvHeight,
                                                        mCamPreviewW,mCamPreviewH);
        
    }else{
    
        if(mPreviewRunning == 0){
			if(-1 == start())
				goto startPreview_end;
        }
    }
    mPreviewRunning = 1;

	#if 0
	err_af = m_camDevice->isAfAvailable(avail);
	if ( err_af = false){
		LOGE("%s(%d):AF is not AfAvailable ",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
        //err = true;
	}else{
		LOGD("%s(%d): AF is  AfAvailable!!!\n",__FUNCTION__,__LINE__);
		} 
	err_af = m_camDevice->startAfContinous(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
	if ( err_af = false ){
		LOGE("%s(%d): Set startAfContinous failed",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
	} else if( err_af = true ){
	    LOGD("%s(%d): Set startAfContinous success;err_af = %d",__FUNCTION__,__LINE__,err_af);
	}
	#endif
	
    LOG_FUNCTION_NAME_EXIT
    return 0;
startPreview_end:
	LOGD("%s:start preview error.",__func__);
	return -1;
}
status_t CameraIspAdapter::stopPreview()
{
    LOG_FUNCTION_NAME
	int err;
    if(mPreviewRunning){
        if(-1 == stop())
			return -1;
        clearFrameArray();
    }
    mPreviewRunning = 0;
	#if 0
	err = m_camDevice->stopAf();
	if ( err = 0 ){
		LOGE("%s(%d):stop AF failed ",__FUNCTION__,__LINE__);
	}else{
		LOGD("%s(%d): stop AF  success!!!\n",__FUNCTION__,__LINE__);
		} 
	#endif
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int CameraIspAdapter::setParameters(const CameraParameters &params_set)
{
    mParameters = params_set;
    return 0;
}
void CameraIspAdapter::initDefaultParameters(int camFd)
{
    CameraParameters params;
	String8 parameterString;
	LOG_FUNCTION_NAME
	LOGD("--------------enter initDefaultParameters-----------------");
	#if 1
	bool err_af;
	bool avail = false;
	err_af = m_camDevice->isAfAvailable(avail);
	if ( err_af = false){
		LOGE("%s(%d):AF is not AfAvailable ",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
        //err = true;
	}else{
		LOGD("%s(%d): AF is  AfAvailable!!!\n",__FUNCTION__,__LINE__);
		} 
	err_af = m_camDevice->startAfContinous(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
	if ( err_af = false ){
		LOGE("%s(%d): Set startAfContinous failed",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
	} else if( err_af = true ){
	    LOGD("%s(%d): Set startAfContinous success;err_af = %d",__FUNCTION__,__LINE__,err_af);
	}
	#endif
	#if 1
	IsiSensorCaps_t     pCaps;	
	m_camDevice->getSensorCaps(pCaps);	//setFocus
	LOGD("  ===pCaps.Resolution:0x%x;pCaps.AfpsResolutions:0x%x====\n",pCaps.Resolution,pCaps.AfpsResolutions);
	/*preview size setting*/	
	if((pCaps.Resolution & ISI_RES_TV1080P15) || (pCaps.Resolution & ISI_RES_TV1080P30)) //.....	
	  {		
		parameterString.append("1920x1080,1280x720,800x600,640x480");    	
		params.setPreviewSize(1920, 1080);	

		params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, "1280x720,1024x768,800x600,640x480,352x288,320x240,176x144,1600x1200,2592x1944,3264x2448");
		params.setPictureSize(2592,1944);
	  }	else if((pCaps.Resolution & ISI_RES_1600_1200)|| (pCaps.Resolution & ISI_RES_SVGA30))	
	  {		
	  	parameterString.append("800x600");
		params.setPreviewSize(800, 600);	

		params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, "1600x1200,1024x768,640x480,352x288,320x240,176x144");
		params.setPictureSize(1600,1200);
	  }	
	params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
	#endif
	rk_cam_total_info *pCamInfo = gCamInfos[camFd].pcam_total_info;
	
	#if 1
	struct v4l2_queryctrl focus;

    parameterString = CameraParameters::FOCUS_MODE_FIXED;
    params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
    focus.id = V4L2_CID_FOCUS_AUTO;
    //if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
      if (false/*(pCamInfo->mSoftInfo.mFocusConfig.mFocusSupport & (0x01<<FOCUS_AUTO_BITPOS))*/){
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_AUTO);
        params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_AUTO);
		params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"1");
    }else{
     	params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"0");
		LOGD("%s(%d):AF is not AfAvailable when init ",__FUNCTION__,__LINE__);
	}

    focus.id = V4L2_CID_FOCUS_CONTINUOUS;
    //if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
    if(0){
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);
    }

    focus.id = V4L2_CID_FOCUS_ABSOLUTE;
    //if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
    if(0){
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_INFINITY);
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_MACRO);
    }

	params.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, parameterString.string());

	focus.id = V4L2_CID_FOCUSZONE;
     
	// focus area settings
    //if (!ioctl(iCamFd, VIDIOC_QUERYCTRL, &focus)) {
    /*if(1){
 	   params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"1");
	}else{
	   params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"0");
	}*/
	#endif
	
	//params.setPreviewSize(1280, 720);	
	//params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, "1280x720");
	//params.setPreviewSize(1920,1080);
	//params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, "1920x1080,1280x720,800x600,640x480");
	//params.setPreviewSize(3264,2448);
    //params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, "3264x2448");
	
	//params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, "1280x720,1024x768,800x600,640x480,352x288,320x240,176x144,1600x1200,2592x1944,3264x2448");
	//params.setPictureSize(2592,1944);
	/*preview format setting*/
	params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, "yuv420sp,yuv420p");
	params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);
	params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
	params.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,CameraParameters::PIXEL_FORMAT_YUV420SP);

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
	params.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, "false");

	//for video test
	params.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "3000,30000");
	params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(3000,30000)");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "10,15,30");  
	
    mParameters = params;

    LOGD ("Support Preview format: %s .. %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS), params.get(CameraParameters::KEY_PREVIEW_FORMAT)); 
	LOGD ("Support Preview sizes: %s     %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES), params.get(CameraParameters::KEY_PREVIEW_SIZE));
	
	LOGD ("Support Preview FPS range: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE));   
	LOGD ("Support Preview framerate: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES));   
	LOGD ("Support Picture sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES));
	
	LOGD ("Support focus: %s  focus zone: %s",params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES),
        params.get(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS));
    LOG_FUNCTION_NAME_EXIT
}
status_t CameraIspAdapter::autoFocus()
{
	LOGD("%s(%d): enter autoFocus()!",__func__,__LINE__);
	int err = 0,ret;
	bool avail = false;
	bool enable = false;
	bool err_af;
	CamEngineAfSearchAlgorithm_t CamEngineAfSearchAlgorithm;
    struct v4l2_ext_control extCtrInfo;
	struct v4l2_ext_controls extCtrInfos;
	extCtrInfo.rect[0] = 0;
    extCtrInfo.rect[1] = 0;
    extCtrInfo.rect[2] = 0;
    extCtrInfo.rect[3] = 0;   
	//if (strcmp(focus, CameraParameters::FOCUS_MODE_AUTO) == 0) {
	if(1){
    LOGD("%s(%d): enter FOCUS_MODE_AUTO!!!\n",__FUNCTION__,__LINE__);
        extCtrInfo.id = V4L2_CID_FOCUS_AUTO;
        //if (auto_trig_only)
        if(0)
            extCtrInfo.value = 2;
        else
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
	}
	extCtrInfos.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
	extCtrInfos.count = 1;
	extCtrInfos.controls = &extCtrInfo;
	//err = ioctl(iCamFd, VIDIOC_S_EXT_CTRLS, &extCtrInfos);
	//m_camDevice->setFocus(0);
	//err = m_camDevice->setFocus(64);	//
	err = m_camDevice->isAfAvailable(avail);
	if ( err = 0 ){
		LOGE("%s(%d):AF is not AfAvailable ",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
        //err = true;
	}else{
		LOGD("%s(%d): AF is  AfAvailable!!!\n",__FUNCTION__,__LINE__);
		} 
	err = m_camDevice->startAfOneShot(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
	if ( err = 0 ){
		LOGE("%s(%d): startAfOneShot() failed",__FUNCTION__,__LINE__);
        err = true;
	} else{
	    LOGD("%s(%d): startAfOneShot() success;err = %d",__FUNCTION__,__LINE__,err);
        err = false;
	}
	sleep(2);
	err = m_camDevice->stopAf();
	LOGD("%s(%d):::Stop AF focus ;err = %d",__FUNCTION__,__LINE__,err);
	err_af = m_camDevice->startAfContinous(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
	if ( err_af = false ){
		LOGE("%s(%d): Set startAfContinous failed",__FUNCTION__,__LINE__);
		//do not return false temporary ,driver may be somthing wrong,zyc@rock-chips.com
	} else if( err_af = true ){
	    LOGD("%s(%d):::Set startAfContinous success;err_af = %d",__FUNCTION__,__LINE__,err_af);
	}
	/*while( true == m_camDevice->getAfStatus(enable,CamEngineAfSearchAlgorithm)){
		LOGD("%s(%d): AF is running",__FUNCTION__,__LINE__);
	}*/

	LOGD("%s(%d): AF has completed",__FUNCTION__,__LINE__);
		
    return 0;
}

void CameraIspAdapter::setScenarioMode(CamEngineModeType_t newScenarioMode)
{
    CamEngineModeType_t oldScenarioMode = CAM_ENGINE_MODE_INVALID;
    m_camDevice->getScenarioMode( oldScenarioMode );


    if ( oldScenarioMode != newScenarioMode )
    {
        // disconnect
        disconnectCamera();

        m_camDevice->setScenarioMode( newScenarioMode );
        m_camDevice->clearSensorDriverFile();
    }
}
//sensor interface :mipi ? parallel?
void CameraIspAdapter::setSensorItf(int newSensorItf)
{
    if ( CamEngineItf::Sensor == m_camDevice->configType() )
    {
        int oldSensorItf = m_camDevice->getSensorItf();
        if ( oldSensorItf != newSensorItf )
        {
            m_camDevice->setSensorItf( newSensorItf );
             mSensorItfCur = newSensorItf;
            loadSensor( mCamId);
           
        }
        
    }
}
void CameraIspAdapter::enableAfps( bool enable )
{
    if ( CamEngineItf::Sensor == m_camDevice->configType() )
    {
        if ( enable != m_camDevice->isAfpsEnabled() )
        {
            m_camDevice->enableAfps( enable );
            m_camDevice->changeEcm( true );
        }
    }
}

void CameraIspAdapter::openImage( const char* fileName)
{
    // open sensor
    if ( NULL != fileName )
    {
        disconnectCamera();
        PicBufMetaData_t        image;
        image.Type = PIC_BUF_TYPE_RAW8;
        image.Layout =PIC_BUF_LAYOUT_BAYER_RGRGGBGB;
        image.Data.raw.PicWidthPixel= 1920;
        image.Data.raw.PicHeightPixel = 1080;
        image.Data.raw.PicWidthBytes = 1920;

        image.Data.raw.pBuffer = (uint8_t *)malloc( image.Data.raw.PicWidthBytes * image.Data.raw.PicHeightPixel );
        if (true == m_camDevice->openImage(fileName,image,NULL,NULL,NULL,NULL,1.0,1.0))
        {
            //set scenario mode
            CamEngineModeType_t mode1 = CAM_ENGINE_MODE_IMAGE_PROCESSING; 

            m_camDevice->setScenarioMode( mode1 );
            // connect
            connectCamera();


        }else{
            LOGE("%s(%d):failed!",__func__,__LINE__);
        }





        if(image.Data.raw.pBuffer)
            free(image.Data.raw.pBuffer);


        
    }

    
}
void CameraIspAdapter::loadSensor( const int cameraId)
{
    // open sensor
    if(cameraId>=0)
    {
        disconnectCamera();
        LOGD("cameraId : %d ===============",cameraId);

        rk_cam_total_info *pCamInfo = gCamInfos[cameraId].pcam_total_info;
        if ( true == m_camDevice->openSensor( pCamInfo, mSensorItfCur ) )
        {
        	bool res = m_camDevice->checkVersion(pCamInfo);
			if(res!=true)
			return;
#if 1
            // connect
            uint32_t resMask;
            CamEngineWindow_t dcWin;

            m_camDevice->getPreferedSensorRes(DEFAULTPREVIEWWIDTH, DEFAULTPREVIEWHEIGHT, 
                                        &mCamDrvWidth, &mCamDrvHeight,&resMask);
            m_camDevice->setSensorResConfig(resMask);
            setupPreview(mCamDrvWidth,mCamDrvHeight,DEFAULTPREVIEWWIDTH,DEFAULTPREVIEWHEIGHT);

            LOGD("%s:sensor(%dx%d),user(%dx%d)",__func__,mCamDrvWidth,mCamDrvHeight,DEFAULTPREVIEWWIDTH,DEFAULTPREVIEWHEIGHT);

            connectCamera();
            mCamPreviewH = DEFAULTPREVIEWHEIGHT;
            mCamPreviewW = DEFAULTPREVIEWWIDTH;
            //mSensorDriverFile[mSensorItfCur] = fileName;
#endif
        }
        else
        {
            LOGE("%s(%d):failed!",__func__,__LINE__);
        }
    }
}

void CameraIspAdapter::loadCalibData(const char* fileName )
{
    if ( NULL != fileName )
    {
        disconnectCamera();
        if ( true == m_camDevice->loadCalibrationData( fileName ) )
        {
            // connect
            connectCamera();
        }
        else
        {
            LOGE("%s(%d):failed!",__func__,__LINE__);
        }
    }
}

bool CameraIspAdapter::connectCamera(){
    bool result = false;
    result = m_camDevice->connectCamera( true, this );
    if ( true != result)
    {
        LOGE("%s(%d):failed!",__func__,__LINE__);
    }
    return result;
}

void CameraIspAdapter::disconnectCamera()
{
    m_camDevice->disconnectCamera();
}

int CameraIspAdapter::start()
{
    if ( ( !m_camDevice->hasSensor() ) &&
         ( !m_camDevice->hasImage()  ) )
    {
        LOGE("%s(%d):failed!",__func__,__LINE__);
        return -1;
    }

    if ( true == m_camDevice->startPreview() )
    {

        m_camDevice->isPictureOrientationAllowed( CAM_ENGINE_MI_ORIENTATION_ORIGINAL );
		return 0;
    }
	else
		return -1;
}
int CameraIspAdapter::pause()
{
    if ( ( !m_camDevice->hasSensor() ) &&
         ( !m_camDevice->hasImage()  ) )
    {
        LOGE("%s(%d):failed!",__func__,__LINE__);
       return -1;
    }

    if ( true == m_camDevice->pausePreview() )
    {
        LOGD("%s(%d):success!",__func__,__LINE__);
		return 0;
    }
	else
		return -1;
}


/******************************************************************************
 * stop
 *****************************************************************************/
int CameraIspAdapter::stop()
{
    if ( ( !m_camDevice->hasSensor() ) &&
         ( !m_camDevice->hasImage()  ) )
    {
        LOGE("%s(%d):failed!",__func__,__LINE__);
        return -1;
    }

    if ( true == m_camDevice->stopPreview() )
    {
    
        LOGD("%s(%d):success!",__func__,__LINE__);
		return 0;
    }
	else
	{
		LOGD("%s(%d):fail!",__func__,__LINE__);
		return -1;
	}
}

void CameraIspAdapter::clearFrameArray(){
    MediaBuffer_t *pMediaBuffer = NULL;
    FramInfo_s *tmpFrame = NULL;
    int num = mFrameInfoArray.size();
    while(--num >= 0){
        tmpFrame = ( FramInfo_s *)mFrameInfoArray.keyAt(num );
        pMediaBuffer = (MediaBuffer_t *)mFrameInfoArray.valueAt(num);
        //remove item
        mFrameInfoArray.removeItem((void*)tmpFrame);
        free(tmpFrame);
        //unlock
        MediaBufUnlockBuffer( pMediaBuffer );
    }
    LOGD("%s(%d):out!!!",__FUNCTION__,__LINE__);
}
int CameraIspAdapter::adapterReturnFrame(int index,int cmd){
	#if 1
    FramInfo_s* tmpFrame = ( FramInfo_s *)index;
    //LOGD("%s(%d):freeframe,pMediaBuffer(%p)!!!",__FUNCTION__,__LINE__,pMediaBuffer);
    MediaBuffer_t *pMediaBuffer = (MediaBuffer_t *)mFrameInfoArray.valueFor((void*)tmpFrame);
    {	
        //remove item
        mFrameInfoArray.removeItem((void*)tmpFrame);
        free(tmpFrame);

        //unlock
        MediaBufUnlockBuffer( pMediaBuffer );
    }
    #endif
	#if 0
	MediaBuffer_t *pMediaBuffer = (MediaBuffer_t *)index;
    //unlock
    MediaBufUnlockBuffer( pMediaBuffer );
	#endif
    return 0;
}

int CameraIspAdapter::getCurPreviewState(int *drv_w,int *drv_h)
{
	*drv_w = mCamPreviewW;
	*drv_h = mCamPreviewH;
    return mPreviewRunning;
}

void CameraIspAdapter::bufferCb( MediaBuffer_t* pMediaBuffer )
{
    static int writeoneframe = 0;
    uint32_t y_addr,uv_addr;
    void* y_addr_vir = NULL,*uv_addr_vir = NULL ;
    int width,height;
    int fmt = 0;
	int tem_val;
	

	//LOG_FUNCTION_NAME
	
	
	Mutex::Autolock lock(mLock);
    // get & check buffer meta data
    PicBufMetaData_t *pPicBufMetaData = (PicBufMetaData_t *)(pMediaBuffer->pMetaData);
    HalHandle_t  tmpHandle = m_camDevice->getHalHandle();
    
    if(pPicBufMetaData->Type == PIC_BUF_TYPE_YCbCr420 || pPicBufMetaData->Type == PIC_BUF_TYPE_YCbCr422){
            if(pPicBufMetaData->Type == PIC_BUF_TYPE_YCbCr420){
                fmt = V4L2_PIX_FMT_NV12;
                }else{
                fmt = V4L2_PIX_FMT_YUYV;
                }
        
            if(pPicBufMetaData->Layout == PIC_BUF_LAYOUT_SEMIPLANAR ){
                y_addr = (uint32_t)(pPicBufMetaData->Data.YCbCr.semiplanar.Y.pBuffer);
                //now gap of y and uv buffer is 0. so uv addr could be calc from y addr.
                uv_addr = (uint32_t)(pPicBufMetaData->Data.YCbCr.semiplanar.CbCr.pBuffer);
                width = pPicBufMetaData->Data.YCbCr.semiplanar.Y.PicWidthPixel;
                height = pPicBufMetaData->Data.YCbCr.semiplanar.Y.PicHeightPixel;
                //get vir addr
                HalMapMemory( tmpHandle, y_addr, 100, HAL_MAPMEM_READWRITE, &y_addr_vir );
                HalMapMemory( tmpHandle, uv_addr, 100, HAL_MAPMEM_READWRITE, &uv_addr_vir );
                #if 0
                if(writeoneframe++ == 10){
                    //write file
                    	FILE* fp =NULL;
                	char filename[40];

                	filename[0] = 0x00;
                	sprintf(filename, "/data/yuv420_%dx%d.bin",width,height);
                	fp = fopen(filename, "wb+");
                	if (fp > 0) {
                		fwrite((char*)y_addr_vir, 1,width*height,fp);
                		fwrite((char*)uv_addr_vir, 1,width*height/2,fp); //yuv422

                		fclose(fp);
                		LOGD("Write success yuv data to %s",filename);
                	} else {
                		LOGE("Create %s failed(%d, %s)",filename,fp, strerror(errno));
                	}
                }
                #endif

                
            }else if(pPicBufMetaData->Layout == PIC_BUF_LAYOUT_COMBINED){
                y_addr = (uint32_t)(pPicBufMetaData->Data.YCbCr.combined.pBuffer );
                width = pPicBufMetaData->Data.YCbCr.combined.PicWidthPixel>>1;
                height = pPicBufMetaData->Data.YCbCr.combined.PicHeightPixel;
                HalMapMemory( tmpHandle, y_addr, 100, HAL_MAPMEM_READWRITE, &y_addr_vir );

            }
     //      LOGD("user got frame,res(%dx%d),pMediaBuffer(%p)",width,height,pMediaBuffer);

        }else{


                y_addr = (uint32_t)(pPicBufMetaData->Data.raw.pBuffer );
                width = pPicBufMetaData->Data.raw.PicWidthPixel;
                height = pPicBufMetaData->Data.raw.PicHeightPixel;
                HalMapMemory( tmpHandle, y_addr, 100, HAL_MAPMEM_READWRITE, &y_addr_vir );
				#if 0
                if(writeoneframe++ == 10){
                    //write file
                    	FILE* fp =NULL;
                	char filename[40];

                	filename[0] = 0x00;
                	sprintf(filename, "/data/raw8_%dx%d.raw",width,height);
                	fp = fopen(filename, "wb+");
                	if (fp > 0) {
                		fwrite((char*)y_addr_vir, 1,width*height,fp);
                	//	fwrite((char*)uv_addr_vir, 1,width*height*3/2,fp); //yuv422

                		fclose(fp);
                		LOGD("Write success yuv data to %s",filename);
                	} else {
                		LOGE("Create %s failed(%d, %s)",filename,fp, strerror(errno));
                	}
                }
				#endif
            LOGE("not support this type(%dx%d)  ,just support  yuv20 now",width,height);
            return;
    }
    

    if ( pMediaBuffer->pNext != NULL )
    {
        MediaBufLockBuffer( (MediaBuffer_t*)pMediaBuffer->pNext );
    }
#if 1
	
	if(preview_frame_inval > 0){
	  	 preview_frame_inval--;
		 LOGD("frame_inval:%d\n",preview_frame_inval);
		 goto end;
	  	}
	//need to display ?
	if(mRefDisplayAdapter->isNeedSendToDisplay()){  
	    MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
      }
      //add to vector
      tmpFrame->frame_index = (int)tmpFrame; 
      tmpFrame->phy_addr = (int)y_addr;
      tmpFrame->frame_width = width;
      tmpFrame->frame_height= height;
      tmpFrame->vir_addr = y_addr_vir;
      tmpFrame->frame_fmt = fmt;
      mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      mRefDisplayAdapter->notifyNewFrame(tmpFrame);
    }

	//video enc ?
	if(mRefEventNotifier->isNeedSendToVideo()){
	    MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
      }
      //add to vector
      tmpFrame->frame_index = (int)tmpFrame; 
      tmpFrame->phy_addr = (int)y_addr;
      tmpFrame->frame_width = width;
      tmpFrame->frame_height= height;
      tmpFrame->vir_addr = y_addr_vir;
      tmpFrame->frame_fmt = fmt;
      mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      mRefEventNotifier->notifyNewVideoFrame(tmpFrame);		
	}
	//picture ?
	if(mRefEventNotifier->isNeedSendToPicture()){
		MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
		}
	  //add to vector
	  tmpFrame->frame_index = (int)tmpFrame; 
	  tmpFrame->phy_addr = (int)y_addr;
	  tmpFrame->frame_width = width;
	  tmpFrame->frame_height= height;
	  tmpFrame->vir_addr = y_addr_vir;
	  tmpFrame->frame_fmt = fmt;
	  mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
	  mRefEventNotifier->notifyNewPicFrame(tmpFrame);	
	}

	//preview data callback ?
	if(mRefEventNotifier->isNeedSendToDataCB()){
	}
	#endif

	end:
		//frame_inval--;
		//LOG2("222frame_inval:%d\n",preview_frame_inval);
		tem_val =0 ;
}

}

