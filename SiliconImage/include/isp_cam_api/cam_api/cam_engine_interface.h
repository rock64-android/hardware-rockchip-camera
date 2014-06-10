/******************************************************************************
 *
 * Copyright 2010, Dream Chip Technologies GmbH. All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of:
 * Dream Chip Technologies GmbH, Steinriede 10, 30827 Garbsen / Berenbostel,
 * Germany
 *
 *****************************************************************************/
/**
 * @file cam_engine_interface.h
 *
 * @brief
 *   Cam Engine C++ API.
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup module_name Module Name
 * @{
 *
 */

#ifndef __CAM_ENGINE_ITF_H__
#define __CAM_ENGINE_ITF_H__

#include <isi/isi.h>
#include <isi/isi_iss.h>

// calibration database
#include <cam_calibdb/cam_calibdb_api.h>

// cam-engine
#include <cam_engine/cam_engine_api.h>
#include <cam_engine/cam_engine_drv_api.h>
#include <cam_engine/cam_engine_aaa_api.h>
#include <cam_engine/cam_engine_jpe_api.h>
#include <cam_engine/cam_engine_mi_api.h>
#include <cam_engine/cam_engine_isp_api.h>
#include <cam_engine/cam_engine_imgeffects_api.h>
#include <cam_engine/cam_engine_cproc_api.h>
#include <cam_engine/cam_engine_simp_api.h>

#include <vector>

#include <CameraHal_board_xml_parse.h>
#include <linux/version.h>

/*
*              SILICONIMAGE LIBISP VERSION NOTE
*
*v0.2.0x00 : add checkAfShot and registerAfEvtQue api, log level;
*v0.3.0x00 : add configFlash/startFlash/stopFlash
*v0.4.0x00 : invalidate DominateIlluProfileIdx init in AwbIlluEstProcessFrame, because AwbReConfigure maybe failed
*            if awb fLikeHoodSum near zero;
*v0.5.0x00 : 1). if f_s < 0.25 and d65 weight < 0.75, illuminate d65 -> cwf; 
*            2). f_RgProj min is 1.3, if illuminate is no A; 
*v0.6.0x00 :
             1) support mi output yuv420 when sensor output yuv422
             2) merge cameric_mi.c from isp vendor version delivery_sw_M14_v2_rel_1_2_20140424
*v0.7.0:
*            1). AfCntMax 120 -> 10 in  CamEngineCamerIcDrvMeasureCbRestart, because af validate delay too long after changeResoultion
*            2). flash delay val 100000 -> 700000;
*v0.8.0:     1). can change mp config when streaming , could do digital zoom use this feature.
*v0.9.0:     1). soc sensor buswidth depend on boad XML
*v0.a.0:
*            this version sync v0.7.1 version;
*            1). flash delay val 70000 -> 50000; 
*v0.b.0:     1). optimize flash unit
*v0.c.0:   1) fix i2c read error
*
*v0.d.0:  support ov8858 and ov13850 sensor dirver
*v0.e.0:   1) add flash trig pol control
*v0.f.0:   1) iomux trigger pin as flash_trigger_out when do flash,or as gpio.
*v0.g.0:   1) increase fl_time
*v0.0x10.0:
*          1) isp_flash_prediv is wrong ,fix it
*v0.0x11.0:
*          1) marvin reset(ircl bit7) may be hold ahb when isp_clk:isp_hclk > 2:1, so used hard reset (cru reset);
*/


#define CONFIG_SILICONIMAGE_LIBISP_VERSION KERNEL_VERSION(0, 0x11, 0x00)

class CamEngineItf;
typedef void (AfpsResChangeCb_t)(void *ctx);

typedef struct CamEngVer_s {
    unsigned int libisp_ver;
    unsigned int isi_ver;
} CamEngVer_t;

class CamEngineVersionItf
{
public:
    CamEngineVersionItf(){};
    ~CamEngineVersionItf(){};
    
    bool getVersion(CamEngVer_t *ver);
};

/**
 * @brief CamEngineItf class declaration.
 */

class CamEngineItf
{
public:
    /**
     * @brief Standard constructor for the CamEngineItf object.
     */
    CamEngineItf( HalHandle_t hHal, AfpsResChangeCb_t *pcbResChange = NULL, void *ctxCbResChange = NULL, int mipiLaneNum=1);
    ~CamEngineItf();

public:
     enum State
    {
        Invalid = 0,
        Idle,
        Running,
        Paused
    };

    enum ConfigType
    {
        None = 0,
        Sensor,
        Image
    };

    enum SnapshotType
    {
        RGB         = 0,
        RAW8        = 1,
        RAW12       = 2,
        JPEG        = 3,    /**< jpeg with exif header */
        JPEG_JFIF   = 4,    /**< jpeg with hardware generated jfif header */
        DPCC        = 5
    };

private:
    CamEngineItf (const CamEngineItf& other);
    CamEngineItf& operator = (const CamEngineItf& other);

public:
    HalHandle_t   getHalHandle() const{return m_hHal;} // zyc add
    State       state() const;
    ConfigType  configType() const;

    CamEngineModeType_t configMode() const;

    const char *softwareVersion() const;
    uint32_t    bitstreamId() const;
    uint32_t    camerIcMasterId() const;
    uint32_t    camerIcSlaveId() const;
    bool        isBitstream3D() const;
	//oyyf add
	void getIspVersion(unsigned int* version);
	//oyyf add
	bool getSensorIsiVersion(unsigned int* pVersion);
	//oyyf add
	bool getSensorTuningXmlVersion(char** pTuningXmlVersion);
	//oyyf adds
	bool getLibispIsiVersion(unsigned int* pVersion) ;
	//oyyf add
	bool checkVersion(rk_cam_total_info *pCamInfo);
    //zyc add
    bool getPreferedSensorRes(int width_in ,int height_in ,int *width_out,int *height_out,uint32_t* mask);
    bool previewSetup_ex(CamEngineWindow_t dcWin,int usr_w,int usr_h,CamerIcMiDataMode_t mode,CamerIcMiDataLayout_t layout,bool_t dcEnable);

    // open sensor driver
    bool        openSensor( rk_cam_total_info *pCamInfo, int sensorItfIdx );
    bool        openSensor( const char *pFileName, int sensorItfIdx = 0 );

    // setup sensor for a priview resolution
    bool        previewSensor();
    // open image file
    bool        openImage( const char               *pFileName,
                           PicBufMetaData_t         image,
                           CamEngineWbGains_t *wbGains = NULL,
                           CamEngineCcMatrix_t *ccMatrix = NULL,
                           CamEngineCcOffset_t *ccOffset = NULL,
                           CamEngineBlackLevel_t *blvl = NULL,
                           float gain = 1.0f, float itime = 0.01f );

    bool        previewSetup();

    bool        hasImage();
    bool        hasSensor();

    const char *sensorDriverFile() const;
    void        clearSensorDriverFile() const;

    bool        loadCalibrationData( camsys_load_sensor_info *pLoadInfo );
    bool        loadCalibrationData( const char *pFileName );
    const char *calibrationDataFile() const;
    CamCalibDbHandle_t calibrationDbHandle() const;

    const char *sensorName() const;
    uint32_t    sensorRevision() const;
    bool        isSensorConnected() const;
    uint32_t    subSensorRevision() const;
    bool        isSubSensorConnected() const;
    const IsiRegDescription_t*
                sensorRegisterTable() const;
    void        getSensorCaps( IsiSensorCaps_t &sensorCaps ) const;
    void        getSensorConfig( IsiSensorConfig_t &sensorConfig ) const;
    void        setSensorConfig( const IsiSensorConfig_t &sensorConfig );
    bool        getIlluminationProfiles( std::vector<CamIlluProfile_t *> &profiles ) const;
    bool        dumpSensorRegister( bool main_notsub, const char* pFileName ) const;
    bool        getSensorRegisterDescription( uint32_t addr, IsiRegDescription_t &description ) const;
    bool        readSensorRegister( bool main_notsub, uint32_t addr, uint32_t &value ) const;
    bool        writeSensorRegister( bool main_notsub, uint32_t addr, uint32_t value ) const;

    void        getScenarioMode( CamEngineModeType_t &mode ) const;
    void        setScenarioMode( CamEngineModeType_t &mode ) const;
    int         getSensorItf() const;
    void        setSensorItf( int &idx ) const;
    bool        changeSensorItf( bool restoreState = false );

    bool        is3DEnabled() const;
    void        enableTestpattern( bool enable = true );
    bool        isTestpatternEnabled() const;
    void        enableAfps( bool enable = true );
    bool        isAfpsEnabled() const;

    bool        getFlickerPeriod( float &flickerPeriod ) const;
    void        getFlickerPeriod( CamEngineFlickerPeriod_t &flickerPeriod ) const;
    void        setFlickerPeriod( CamEngineFlickerPeriod_t &flickerPeriod ) const;

    bool        registerBufferCb( CamEngineBufferCb_t fpCallback, void* BufferCbCtx );
    bool        deRegisterBufferCb();
    bool        getBufferCb( CamEngineBufferCb_t* fpCallback, void** BufferCbCtx );

    bool        mapBuffer( const MediaBuffer_t *pSrcBuffer, PicBufMetaData_t *pDstBuffer );
    bool        unmapBuffer( PicBufMetaData_t* pBuffer );

    bool reset();

    bool connect();
    void disconnect();
    bool changeResolution( uint32_t resolution, bool restoreState = false );
    bool changeEcm( bool restoreState = false );

    bool start( uint32_t frames = 0 );
    bool pause();
    bool stop();

    bool getResolution( int &width, int &height ) const;
    bool getResolution( uint32_t &resolution ) const;

    bool getGain( float &gain ) const;
    bool getGainLimits( float &minGain, float &maxGain, float &step ) const;
    bool setGain( float newGain, float &setGain );

    bool getIntegrationTime( float &time ) const;
    bool getIntegrationTimeLimits( float &minTime, float &maxTime, float &step ) const;
    bool setIntegrationTime( float newTime, float &setTime );

    bool setExposure( float newExp, float &setExp );

    bool getFocus( uint32_t &focus ) const;
    bool getFocusLimits( uint32_t &minFocus, uint32_t &maxFocus ) const;
    bool setFocus( uint32_t focus );

    bool getPathConfig( const CamEngineChainIdx_t idx, CamEnginePathType_t path, CamEnginePathConfig_t &pathConfig ) const;
    bool setPathConfig( const CamEngineChainIdx_t idx, const CamEnginePathConfig_t &mpConfig, const CamEnginePathConfig_t &spConfig );

    bool searchAndLock( CamEngineLockType_t locks );
    bool unlock( CamEngineLockType_t locks );

    // auto exposure functions
    bool isAecEnabled();
    bool startAec();
    bool stopAec();
    bool resetAec();
    bool configureAec( const CamEngineAecSemMode_t &mode, const float setPoint, const float clmTolerance, const float dampOver, const float dampUnder );
    bool getAecStatus( bool &enabled, CamEngineAecSemMode_t &mode, float &setPoint, float &clmTolerance, float &dampOver, float &dampUnder );
    bool getAecHistogram( CamEngineAecHistBins_t &histogram ) const;
    bool getAecLuminance( CamEngineAecMeanLuma_t &luma ) const;
    bool getAecObjectRegion( CamEngineAecMeanLuma_t &objectRegion ) const;

    // auto focus functions
    bool isAfAvailable( bool &available );
    bool resetAf( const CamEngineAfSearchAlgorithm_t &searchAlgorithm );
    bool startAfContinous( const CamEngineAfSearchAlgorithm_t &searchAlgorithm );
    bool startAfOneShot( const CamEngineAfSearchAlgorithm_t &searchAlgorithm );
    bool stopAf();
    bool getAfStatus( bool &enabled, CamEngineAfSearchAlgorithm_t &seachAlgorithm );
    bool checkAfShot( bool *shot );  /* ddl@rock-chips.com */
    bool registerAfEvtQue( CamEngineAfEvtQue_t *evtQue );   /* ddl@rock-chips.com */

    // flash   ddl@rock-chips.com
    bool configureFlash( CamEngineFlashCfg_t *cfgFsh );
    bool startFlash ( bool operate_now );
    bool stopFlash ( bool operate_now );
    
    // auto white balance functions
    bool isAwbEnabled();
    bool startAwb( const CamEngineAwbMode_t &mode, const uint32_t idx, const bool_t damp );
    bool stopAwb();
    bool resetAwb();
    bool getAwbStatus( bool &enabled, CamEngineAwbMode_t &mode, uint32_t &idx, CamEngineAwbRgProj_t &RgProj, bool &damping );

    bool startAdpf();
    bool stopAdpf();
    bool configureAdpf( const float gradient, const float offset, const float min, const float div, const uint8_t sigmaGreen, const uint8_t sigmaRedBlue );
    bool getAdpfStatus( bool &enabled, float &gradient, float &offset, float &min, float &div, uint8_t &sigmaGreen, uint8_t &sigmaRedBlue );

    bool startAdpcc();
    bool stopAdpcc();
    bool getAdpccStatus( bool &enabled );

    bool startAvs();
    bool stopAvs();
    bool configureAvs( const bool useParams, const unsigned short numItpPoints, const float theta, const float baseGain, const float fallOff, const float acceleration );
    bool getAvsConfig( bool &usingParams, unsigned short &numItpPoints, float &theta, float &baseGain, 
                       float &fallOff, float &acceleration, int &numDampData, double **ppDampXData, double **ppDampYData );
    bool getAvsStatus( bool &running, CamEngineVector_t &displVec, CamEngineVector_t &offsVec );

    bool enableJpe( int width, int height );
    bool disableJpe();

    // memory infterface module
    bool isPictureOrientationAllowed( CamEngineMiOrientation_t orientation );
    bool setPictureOrientation( CamEngineMiOrientation_t orientation );
    //bool getPictureOrientation( CamerIcMiOrientation_t &orientation );

    // image effects module 
    void imgEffectsEnable( CamerIcIeConfig_t *pCamerIcIeConfig );
    void imgEffectsDisable();
    
    void imgEffectsSetTintCb( uint8_t tint );
    void imgEffectsSetTintCr( uint8_t tint );
    void imgEffectsSetColorSelection( CamerIcIeColorSelection_t color, uint8_t threshold );
    void imgEffectsSetSharpen( uint8_t factor, uint8_t threshold );

    // color processing module 
    bool cProcIsAvailable();
    void cProcEnable( CamEngineCprocConfig_t *config );
    void cProcDisable();
    void cProcStatus( bool& running, CamEngineCprocConfig_t& config );

    void cProcSetContrast( float contrast );
    void cProcSetBrightness( float brightness );
    void cProcSetSaturation( float saturation );
    void cProcSetHue( float hue );

    // super impose module 
    void sImpEnable( CamEngineSimpConfig_t *config );
    void sImpDisable();

    // black level substraction module
    void blsGet(uint16_t *Red, uint16_t *GreenR, uint16_t *GreenB, uint16_t *Blue);
    void blsSet(const uint16_t Red, const uint16_t GreenR, const uint16_t GreenB, const uint16_t Blue);

    // white balance module
    void wbGainGet(float *Red, float *GreenR, float *GreenB, float *Blue);
    void wbGainSet(const float Red, const float GreenR, const float GreenB, const float Blue);

    void wbCcMatrixGet( float *Red1, float *Red2, float *Red3,
                        float *Green1, float *Green2, float *Green3,
                        float *Blue1, float *Blue2, float *Blue3 );

    void wbCcMatrixSet( const float Red1, const float Red2, const float Red3,
                        const float Green1, const float Green2, const float Green3,
                        const float Blue1, const float Blue2, const float Blue3 );

    void wbCcOffsetGet( int16_t *Red, int16_t *Green, int16_t *Blue );
    void wbCcOffsetSet( const int16_t Red, const int16_t Green, const int16_t Blue );

    // lense shade correction module
    void lscStatus( bool& running, CamEngineLscConfig_t& config );
    void lscEnable();
    void lscDisable();

    // chromatic abberation correction module
    void cacStatus( bool& running, CamEngineCacConfig_t& config );
    void cacEnable();
    void cacDisable();

    // wide dynamic range mode
    void wdrEnable();
    void wdrDisable();
    void wdrSetCurve( CamEngineWdrCurve_t WdrCurve );

    // gamma out correction module
    void gamCorrectStatus( bool& running );
    void gamCorrectEnable();
    void gamCorrectDisable();
    void gamCorrectSetCurve( CamEngineGammaOutCurve_t GammaCurve );

    // isp ilter
    void demosaicGet( bool& bypass, uint8_t& threshold ) const;
    void demosaicSet( bool bypass, uint8_t threshold );
    void filterStatus( bool& running, uint8_t& denoiseLevel, uint8_t& sharpenLevel );
    void filterEnable();
    void filterDisable();
    void filterSetLevel( uint8_t denoiseLevel, uint8_t sharpenLevel );

    // color noise reduction
    bool cnrIsAvailable();
    void cnrStatus( bool& running, uint32_t& TC1, uint32_t& TC2 );
    void cnrEnable();
    void cnrDisable();
    void cnrSetThresholds( uint32_t TC1, uint32_t TC2 );
    
    // memory interface 
    void miSwapUV( bool swap );
    
    // dual cropping unit 
    bool dcropIsAvailable();
    void dcropMpWindowGet( bool& enabled, uint16_t& x, uint16_t& y, uint16_t& width, uint16_t& height );
    void dcropMpWindowSet( bool  enable , uint16_t  x,  uint16_t y, uint16_t  width, uint16_t  height );
    void dcropSpWindowGet( bool& enabled, uint16_t& x, uint16_t& y, uint16_t& width, uint16_t& height );
    void dcropSpWindowSet( bool  enable , uint16_t  x,  uint16_t y, uint16_t  width, uint16_t  height );


    bool isSOCSensor(); // zyc add
    uint32_t getYCSequence(); //zyc add
    uint32_t getBusWidth(); //zyc add
    uint32_t reSetMainPathWhileStreaming
                (
                    CamEngineWindow_t* pWin,
                    uint32_t outWidth, 
                    uint32_t outHeight
                    
                );

private:
    void closeSensor();
    void closeImage();

    bool setupSensor();
    void doneSensor();
    void startSensor();
    void stopSensor();

    bool setupCamerIC();
    void doneCamerIC();
    bool restartMipiDrv(); //zyc add


public:
    class SensorHolder;
    class CamEngineHolder;
private:
    HalHandle_t         m_hHal;
    AfpsResChangeCb_t   *m_pcbItfAfpsResChange;
    void                *m_ctxCbResChange;

    SensorHolder        *m_pSensor;

    CamEngineHolder     *m_pCamEngine;
    int    mSensorBusWidthBoardSet;
};


/* @} module_name_api*/

#endif /*__CAM_ENGINE_ITF_H__*/
