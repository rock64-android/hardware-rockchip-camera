#include "CameraIspAdapter.h"
#include "cam_api/halholder.h"

namespace android{

#define ISP_OUT_YUV420SP   0
#define ISP_OUT_YUV422_INTERLEAVED  1
#define ISP_OUT_FORMAT  ISP_OUT_YUV420SP //ISP_OUT_YUV422_INTERLEAVED

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
    LOG_FUNCTION_NAME

    

    LOG_FUNCTION_NAME_EXIT
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

	LOG_FUNCTION_NAME_EXIT
}
CameraIspAdapter::~CameraIspAdapter()
{
    cameraDestroy();
	
}
int CameraIspAdapter::cameraCreate(int cameraId)
{
    LOG_FUNCTION_NAME
	rk_cam_total_info *pCamInfo = gCamInfos[cameraId].pcam_total_info;
	char* dev_filename = pCamInfo->mHardInfo.mSensorInfo.mCamsysDevPath;
	int mipiLaneNum = 0;
	int i =0;

    pCamInfo->mLibIspVersion = CONFIG_SILICONIMAGE_LIBISP_VERSION;
	
	for(i=0; i<4; i++){
		mipiLaneNum += (pCamInfo->mHardInfo.mSensorInfo.mPhy.info.mipi.data_en_bit>>i)&0x01;
	}
	
    m_camDevice = new CamDevice( HalHolder::handle(dev_filename), CameraIspAdapter_AfpsResChangeCb, (void*)this ,NULL, mipiLaneNum);
	
	//load sensor
    loadSensor( cameraId);
    //open image
    //openImage("/system/lib/libisp_isi_drv_OV8825.so");   

    {
        if (OSLAYER_OK != osQueueInit(&mAfListenerQue.queue,10, sizeof(CamEngineAfEvt_t)))    
        {
            TRACE_E("create af listener queue failed!");
        }

        ListInit(&mAfListenerQue.list);

        mAfListenerThread = new CameraAfThread(this);
        mAfListenerThread->run("CamAfLisThread",ANDROID_PRIORITY_NORMAL);

        m_camDevice->resetAf(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
        m_camDevice->registerAfEvtQue(&mAfListenerQue);
    }

    LOG_FUNCTION_NAME_EXIT
    return 0;

}
int CameraIspAdapter::cameraDestroy()
{
    LOG_FUNCTION_NAME

    {
        CamEngineAfEvt_t cmd;
        int ret;

        cmd.evnt_id = (CamEngineAfEvtId_t)0xfefe5aa5;
        
        osQueueWrite(&mAfListenerQue.queue, &cmd);
        
        if(mAfListenerThread!= NULL){
        	mAfListenerThread->requestExitAndWait();
        	mAfListenerThread.clear();
    	}

        osQueueDestroy(&mAfListenerQue.queue);
        
    }

    
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
#if (ISP_OUT_FORMAT == ISP_OUT_YUV422_INTERLEAVED)
    m_camDevice->previewSetup_ex( dcWin, preview_w, preview_h,
                                CAMERIC_MI_DATAMODE_YUV422,CAMERIC_MI_DATASTORAGE_INTERLEAVED,(bool_t)true);
    LOGD("%s:isp out put format is YUV422 interleaved.",__func__);
#elif(ISP_OUT_FORMAT == ISP_OUT_YUV420SP)

    m_camDevice->previewSetup_ex( dcWin, preview_w, preview_h,
                                CAMERIC_MI_DATAMODE_YUV420,CAMERIC_MI_DATASTORAGE_SEMIPLANAR,(bool_t)true);
    LOGD("%s:isp out put format is YUV420SP.",__func__);
#else
    LOGE("%s:isp don't support this format %s now",__func__,ISP_OUT_FORMAT);
#endif

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


    if (is_capture) {
        m_camDevice->startFlash(true);
    } else {
        m_camDevice->stopFlash(true);
    }

    
    mPreviewRunning = 1;
#if 0
	{
        bool err_af = false;
        
        // Continues picture focus
        if (strcmp(mParameters.get(CameraParameters::KEY_FOCUS_MODE),CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE)==0) {
            err_af = m_camDevice->startAfContinous(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
        	if ( err_af = false ){
        		TRACE_E("Set startAfContinous failed");        		
        	} 
        } 

    }
#endif	
    LOG_FUNCTION_NAME_EXIT
    return 0;
startPreview_end:
	LOG_FUNCTION_NAME_EXIT
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
	
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int CameraIspAdapter::setParameters(const CameraParameters &params_set)
{

    {
        bool err_af = false;
        
        if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES),params_set.get(CameraParameters::KEY_FOCUS_MODE))) {
            if (strcmp(mParameters.get(CameraParameters::KEY_FOCUS_MODE),params_set.get(CameraParameters::KEY_FOCUS_MODE))) {
                // Continues picture focus
                if (strcmp(params_set.get(CameraParameters::KEY_FOCUS_MODE),CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE)==0) {
                    err_af = m_camDevice->startAfContinous(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
                	if ( err_af == false ){
                		TRACE_E("Set startAfContinous failed");        		
                	} 
                }  

                if( err_af == true )
                    TRACE_D(1,"Set focus mode: %s success",params_set.get(CameraParameters::KEY_FOCUS_MODE));
            }
        } else {
            TRACE_E("%s(%d): %s isn't supported for this camera, support focus: %s",__FUNCTION__,__LINE__,
                params_set.get(CameraParameters::KEY_FOCUS_MODE),
                mParameters.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES));

        }

    }


    {
        CamEngineFlashCfg_t flash_cfg;

        if (mParameters.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES) && mParameters.get(CameraParameters::KEY_FLASH_MODE)) {
		
            if (strstr(mParameters.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES),params_set.get(CameraParameters::KEY_FLASH_MODE))) {
                if (strcmp(mParameters.get(CameraParameters::KEY_FLASH_MODE),params_set.get(CameraParameters::KEY_FLASH_MODE))) {
                    if (strcmp(params_set.get(CameraParameters::KEY_FLASH_MODE),CameraParameters::FLASH_MODE_ON)==0) {
                        flash_cfg.mode = CAM_ENGINE_FLASH_ON;
                        m_camDevice->configureFlash(&flash_cfg);
            			LOG1("Set flash on success!");
                    } else if (strcmp(params_set.get(CameraParameters::KEY_FLASH_MODE),CameraParameters::FLASH_MODE_OFF)==0) {
                        flash_cfg.mode = CAM_ENGINE_FLASH_OFF;
                        m_camDevice->configureFlash(&flash_cfg);
            			LOG1("Set flash off success!");
                    }
                }
            } else {
                TRACE_E("%s isn't supported for this camera, support flash: %s",
                    params_set.get(CameraParameters::KEY_FLASH_MODE),
                    mParameters.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES));

            }

        }

    }
    
    mParameters = params_set;  
    
    return 0;
}
void CameraIspAdapter::initDefaultParameters(int camFd)
{
    CameraParameters params;
	String8 parameterString;
    rk_cam_total_info *pCamInfo = gCamInfos[camFd].pcam_total_info;
	LOG_FUNCTION_NAME
    
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
	  	parameterString.append("1280x720,800x600,720x480,640x480");    	
		params.setPreviewSize(800, 600);	

		params.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, "1600x1200,1024x768,640x480,352x288,320x240,176x144");
		params.setPictureSize(1600,1200);
	  }	
	params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, parameterString.string());
	#endif	

    //auto focus parameters
	{
        bool err_af;
    	bool avail = false;

        parameterString = CameraParameters::FOCUS_MODE_FIXED;
        params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);

		if ((strcmp(pCamInfo->mHardInfo.mFlashInfo.mFlashName,"Fixed")!=0)) {

            err_af = m_camDevice->isAfAvailable(avail);
            if ((err_af == true) && (avail == true)) {
                parameterString.append(",");
                parameterString.append(CameraParameters::FOCUS_MODE_AUTO);
                params.set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_AUTO);
        		params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"1");

                parameterString.append(",");
                parameterString.append(CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);

                if(0){
                    parameterString.append(",");
                    parameterString.append(CameraParameters::FOCUS_MODE_INFINITY);
                    parameterString.append(",");
                    parameterString.append(CameraParameters::FOCUS_MODE_MACRO);
                }

            } else {
             	params.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,"0");
        	}

		}
    	params.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, parameterString.string());
	}

    //flash parameters
    {
        CamEngineFlashCfg_t flash_cfg;
        
        if ((strcmp(pCamInfo->mHardInfo.mFlashInfo.mFlashName,"Internal")==0)) {
            parameterString = CameraParameters::FLASH_MODE_OFF;

            parameterString.append(",");
            parameterString.append(CameraParameters::FLASH_MODE_ON);

            params.set(CameraParameters::KEY_FLASH_MODE,CameraParameters::FLASH_MODE_ON);
            params.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES,parameterString.string());


            flash_cfg.mode = CAM_ENGINE_FLASH_ON;
            m_camDevice->configureFlash(&flash_cfg);
        }
    }
	
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

    //for video test
    params.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "3000,30000");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(3000,30000)");
    params.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "10,15,30");  
	
    mParameters = params;

    LOG1 ("Support Preview format: %s .. %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS), params.get(CameraParameters::KEY_PREVIEW_FORMAT)); 
	LOG1 ("Support Preview sizes: %s     %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES), params.get(CameraParameters::KEY_PREVIEW_SIZE));
	
	LOG1 ("Support Preview FPS range: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE));   
	LOG1 ("Support Preview framerate: %s",params.get(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES));   
	LOG1 ("Support Picture sizes: %s ",params.get(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES));
	
	LOG1 ("Support focus: %s  focus zone: %s",params.get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES),
        params.get(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS));
	if (params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES) && params.get(CameraParameters::KEY_FLASH_MODE))
    	LOG1 ("Support flash: %s  flash: %s",params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES),
        	params.get(CameraParameters::KEY_FLASH_MODE));
    LOG_FUNCTION_NAME_EXIT
}

status_t CameraIspAdapter::autoFocus()
{
    bool shot,err_af = false;

    if (strcmp(mParameters.get(CameraParameters::KEY_FOCUS_MODE), CameraParameters::FOCUS_MODE_AUTO) == 0) {
        shot = true;
    } else if (strcmp(mParameters.get(CameraParameters::KEY_FOCUS_MODE), CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE) == 0) {

        if (m_camDevice->checkAfShot(&shot) == false) {
            shot = true;    
        }
    }

    if (shot == true) {
        TRACE_D(1, "Single auto focus must be trigger");
        err_af = m_camDevice->startAfOneShot(CAM_ENGINE_AUTOFOCUS_SEARCH_ALGORITHM_ADAPTIVE_RANGE);
    	if ( err_af == false ){
    		TRACE_E("Trigger a single auto focus failed!");        		
    	} else {
            TRACE_D(1,"Trigger a single auto focus success");
    	}
    } else {
        TRACE_D(1, "It has been focused!");

        CamEngineAfEvt_t evnt;

        evnt.evnt_id = CAM_ENGINE_AUTOFOCUS_FINISHED;
        evnt.info.fshEvt.focus = BOOL_TRUE;

        osQueueWrite(&mAfListenerQue.queue, &evnt);
    }
	 
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
            LOGE("failed!");
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
    LOG_FUNCTION_NAME
    MediaBuffer_t *pMediaBuffer = NULL;
    FramInfo_s *tmpFrame = NULL;
    Mutex::Autolock lock(mFrameArrayLock);

    int num = mFrameInfoArray.size();
    while(--num >= 0){
        tmpFrame = (FramInfo_s *)mFrameInfoArray.keyAt(num);
        if(mFrameInfoArray.indexOfKey((void*)tmpFrame) < 0){
            LOGE("%s:this frame is not in frame array,used_flag is %d!",__func__,tmpFrame->used_flag);
        }else{
            pMediaBuffer = (MediaBuffer_t *)mFrameInfoArray.valueAt(num);
            //remove item
            mFrameInfoArray.removeItem((void*)tmpFrame);
            free(tmpFrame);
            //unlock
            MediaBufUnlockBuffer( pMediaBuffer );
        }
    }
    mFrameInfoArray.clear();
    LOG_FUNCTION_NAME_EXIT
}
int CameraIspAdapter::adapterReturnFrame(int index,int cmd){
	#if 1
    FramInfo_s* tmpFrame = ( FramInfo_s *)index;
    Mutex::Autolock lock(mFrameArrayLock);
    if(mFrameInfoArray.size() > 0){
        if(mFrameInfoArray.indexOfKey((void*)tmpFrame) < 0){
            LOGE("%s:this frame is not in frame array,used_flag is %d!",__func__,tmpFrame->used_flag);
        }else{
            MediaBuffer_t *pMediaBuffer = (MediaBuffer_t *)mFrameInfoArray.valueFor((void*)tmpFrame);
            {	
                //remove item
                mFrameInfoArray.removeItem((void*)tmpFrame);
                free(tmpFrame);

                //unlock
                MediaBufUnlockBuffer( pMediaBuffer );
            }
        }
    }else{
        LOGD("%s:frame array has been cleard!",__func__);
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
    uint32_t y_addr = 0,uv_addr = 0;
    void* y_addr_vir = NULL,*uv_addr_vir = NULL ;
    int width = 0,height = 0;
    int fmt = 0;
	int tem_val;
	
	Mutex::Autolock lock(mLock);
    // get & check buffer meta data
    PicBufMetaData_t *pPicBufMetaData = (PicBufMetaData_t *)(pMediaBuffer->pMetaData);
    HalHandle_t  tmpHandle = m_camDevice->getHalHandle();
    //
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
                		fwrite((char*)y_addr_vir, 1,width*height*3/2,fp);
                //		fwrite((char*)uv_addr_vir, 1,width*height/2,fp); //yuv422

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
      tmpFrame->vir_addr = (int)y_addr_vir;
      tmpFrame->frame_fmt = fmt;
	  
      tmpFrame->used_flag = 0;
      {
        Mutex::Autolock lock(mFrameArrayLock);
        mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      }
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
      tmpFrame->vir_addr = (int)y_addr_vir;
      tmpFrame->frame_fmt = fmt;
      tmpFrame->used_flag = 1;
      {
        Mutex::Autolock lock(mFrameArrayLock);
        mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      }
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
	  tmpFrame->vir_addr = (int)y_addr_vir;
	  tmpFrame->frame_fmt = fmt;
      tmpFrame->used_flag = 2;
      {
        Mutex::Autolock lock(mFrameArrayLock);
        mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      }
	  mRefEventNotifier->notifyNewPicFrame(tmpFrame);	
	}

	//preview data callback ?
	if(mRefEventNotifier->isNeedSendToDataCB()){
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
	  tmpFrame->vir_addr = (int)y_addr_vir;
	  tmpFrame->frame_fmt = fmt;
      tmpFrame->used_flag = 3;
      {
        Mutex::Autolock lock(mFrameArrayLock);
        mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      }
	  mRefEventNotifier->notifyNewPreviewCbFrame(tmpFrame);			
	}
	#endif

end:
	
	tem_val =0 ;
}

void CameraIspAdapter::dump(int cameraId)
{
	rk_cam_total_info *pCamInfo = gCamInfos[cameraId].pcam_total_info;
	m_camDevice->checkVersion(pCamInfo);
}

int CameraIspAdapter::afListenerThread(void)
{

    LOG_FUNCTION_NAME

    bool bExit = false;
    int evnt_id ;

    while (bExit == false)
    {
        CamEngineAfEvt_t afEvt;        
        
        OSLAYER_STATUS osStatus = (OSLAYER_STATUS)osQueueRead(&mAfListenerQue.queue, &afEvt); 
        if (OSLAYER_OK != osStatus)
        {
            TRACE_E( "receiving af event failed -> OSLAYER_RESULT=%d\n", osStatus );
            continue; /* for now we simply try again */
        }

        evnt_id = (int)afEvt.evnt_id;
        switch (evnt_id)
        {
            case CAM_ENGINE_AUTOFOCUS_MOVE:
            {
                TRACE_D(2,"CAMERA_MSG_FOCUS_MOVE: %d",afEvt.info.mveEvt.start);
                mRefEventNotifier->notifyCbMsg(CAMERA_MSG_FOCUS_MOVE, afEvt.info.mveEvt.start);
                break;
            }

            case CAM_ENGINE_AUTOFOCUS_FINISHED:
            {
                TRACE_D(2,"CAMERA_MSG_FOCUS: %d",afEvt.info.fshEvt.focus);
                mRefEventNotifier->notifyCbMsg(CAMERA_MSG_FOCUS, afEvt.info.fshEvt.focus);
                break;
            }

            case 0xfefe5aa5:
            {
                TRACE_D(1, "receive exit command for af thread handle!");
                bExit = true;
                break;
            }
            default:
            {                    
                TRACE_E("afEvt.evnt_id: 0x%x is invalidate!",afEvt.evnt_id);
                break;
            }
        }
    }

    LOG_FUNCTION_NAME_EXIT

    return 0;
}

}

