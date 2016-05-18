//sensor the same with ov14825

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
 * @file sensor.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>
#include <common/misc.h>


#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"

#include "sensor_MIPI_priv.h"

#define  sensor_NEWEST_TUNING_XML "18-7-2014_oyyf-hkw_sensor_CMK-CB0407-FV1_v0.1.1"

//hkw no use;
#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( sensor_INFO , "sensor: ", INFO,    1U );
CREATE_TRACER( sensor_WARN , "sensor: ", WARNING, 1U );
CREATE_TRACER( sensor_ERROR, "sensor: ", ERROR,   1U );

CREATE_TRACER( sensor_DEBUG, "sensor: ", INFO,    1U );

CREATE_TRACER( sensor_NOTICE0, "sensor: ", INFO,  1U );
CREATE_TRACER( sensor_NOTICE1, "sensor: ", INFO,  1U );


#define sensor_SLAVE_ADDR       0x36U                           /**< i2c slave address of the sensor camera sensor */
#define sensor_SLAVE_ADDR2      0x36U
#define sensor_SLAVE_AF_ADDR    0x18U                           /**< i2c slave address of the sensor integrated AD5820 */

#define sensor_MAXN_GAIN 		(128.0f)
#define sensor_MIN_GAIN_STEP   ( 0.001f); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define sensor_MAX_GAIN_AEC    (16.0f )  /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


/*!<
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG   64U
#define MAX_REG 1023U

#define MAX_VCMDRV_CURRENT      100U
#define MAX_VCMDRV_REG          1023U




/*!<
 * Lens movement is triggered every 133ms (VGA, 7.5fps processed frames
 * worst case assumed, usually even much slower, see OV5630 driver for
 * details). Thus the lens has to reach the requested position after
 * max. 133ms. Minimum mechanical ringing is expected with mode 1 ,
 * 100us per step. A movement over the full range needs max. 102.3ms
 * (see table 9 AD5820 datasheet).
 */
#define MDI_SLEW_RATE_CTRL 5U /* S3..0 for MOTOR hkw*/



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char sensor_g_acName[] = "sensor_MIPI";
//extern const IsiRegDescription_t sensor_g_aRegDescription[];
extern const IsiRegDescription_t sensor_g_aRegDescription_onelane[];
extern const IsiRegDescription_t sensor_g_aRegDescription_twolane[];
extern const IsiRegDescription_t sensor_g_binning_onelane[];
extern const IsiRegDescription_t sensor_g_binning_twolane[];
extern const IsiRegDescription_t sensor_g_fullres_onelane[];
extern const IsiRegDescription_t sensor_g_fullres_twolane[];

extern const sensor_gain_table_t sensor_A_gain_table[];
extern const sensor_gain_table_t sensor_D_gain_table[];

extern const IsiRegDescription_t sensor_g_2592x1944P30_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P25_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P20_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P15_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P10_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P7_twolane_fpschg[];
extern const IsiRegDescription_t sensor_g_2592x1944P5_twolane_fpschg[];






const IsiSensorCaps_t sensor_g_IsiSensorDefaultConfig;



#define sensor_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define sensor_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define sensor_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers


static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_TWO_LANE;//SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT sensor_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT sensor_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT sensor_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT sensor_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT sensor_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT sensor_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT sensor_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT sensor_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT sensor_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT sensor_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT sensor_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT sensor_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT sensor_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT sensor_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT sensor_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT sensor_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT sensor_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT sensor_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT sensor_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT sensor_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT sensor_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT sensor_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT sensor_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT sensor_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT sensor_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT sensor_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT sensor_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT sensor_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT sensor_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT sensor_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT sensor_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT sensor_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT sensor_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT sensor_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT sensor_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT sensor_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT sensor_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT sensor_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


static float dctfloor( const float f )
{
    if ( f < 0 )
    {
        return ( (float)((int32_t)f - 1L) );
    }
    else
    {
        return ( (float)((uint32_t)f) );
    }
}



/*****************************************************************************/
/**
 *          sensor_IsiCreateSensorIss
 *
 * @brief   This function creates a new sensor sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT sensor_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    sensor_Context_t *psensorCtx;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    psensorCtx = ( sensor_Context_t * )malloc ( sizeof (sensor_Context_t) );
    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR,  "%s: Can't allocate sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( psensorCtx, 0, sizeof( sensor_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( psensorCtx );
        return ( result );
    }
    
    psensorCtx->IsiCtx.HalHandle              = pConfig->HalHandle;
    psensorCtx->IsiCtx.HalDevID               = pConfig->HalDevID;
    psensorCtx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    psensorCtx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? sensor_SLAVE_ADDR : pConfig->SlaveAddr;
    psensorCtx->IsiCtx.NrOfAddressBytes       = 2U;

    psensorCtx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    psensorCtx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? sensor_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    psensorCtx->IsiCtx.NrOfAfAddressBytes     = 0U;

    psensorCtx->IsiCtx.pSensor                = pConfig->pSensor;

    psensorCtx->Configured             = BOOL_FALSE;
    psensorCtx->Streaming              = BOOL_FALSE;
    psensorCtx->TestPattern            = BOOL_FALSE;
    psensorCtx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    psensorCtx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    psensorCtx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    psensorCtx->VcmInfo.RatedCurrent   = psensorCtx->VcmInfo.StartCurrent + MAX_LOG*psensorCtx->VcmInfo.Step;
    psensorCtx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	psensorCtx->IsiSensorMipiInfo.sensorHalDevID = psensorCtx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        psensorCtx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( sensor_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        psensorCtx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )psensorCtx;

    result = HalSetCamConfig( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an sensor sensor instance.
 *
 * @param   handle      sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT sensor_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)sensor_IsiSensorSetStreamingIss( psensorCtx, BOOL_FALSE );
    (void)sensor_IsiSensorSetPowerIss( psensorCtx, BOOL_FALSE );

    (void)HalDelRef( psensorCtx->IsiCtx.HalHandle );

    MEMSET( psensorCtx, 0, sizeof( sensor_Context_t ) );
    free ( psensorCtx );

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCapsIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor capabilities structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT sensor_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t  mipi_lanes
)
{
    RESULT result = RET_SUCCESS;
    
    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        if(mipi_lanes == SUPPORT_MIPI_TWO_LANE) {
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P30;
                    break;
                }
				case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P25;
                    break;
                }
				case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P20;
                    break;
                }
				case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P15;
                    break;
                }
				case 5:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P10;
                    break;
                }
				case 6:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P7;
                    break;
                }
				case 7:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P5;
                    break;
                }
                /*case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1296_972P30;
                    break;
                }*/
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }
        }  else if(mipi_lanes == SUPPORT_MIPI_ONE_LANE) {
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P7;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1296_972P15;
                    break;
                }
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }
        }              
    
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT; //
        pIsiSensorCaps->Mode            = ISI_MODE_MIPI;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_GRGRBGBG;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS; //hsync?
        pIsiSensorCaps->VPol            = ISI_VPOL_NEG; //VPolarity
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING; //?
        pIsiSensorCaps->Bls             = ISI_BLS_OFF; //close;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;//close;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;//close;<
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );//close;
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_OFF );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL; //;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A  //光源；
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10; 
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP ); //跳帧;没用
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;//
    }
end:
    return result;
}
 
static RESULT sensor_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = sensor_IsiGetCapsIssInternal(pIsiSensorCaps,psensorCtx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t sensor_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_GRGRBGBG,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_NEG,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_2592_1944P30, 
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_D65,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_RAW_10,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          sensor_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT sensor_SetupOutputFormat
(
    sensor_Context_t       *psensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s%s (enter)\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* mode */
    switch ( pConfig->Mode )            /* only ISI_MODE_BAYER supported, no configuration needed here */
    {
        case( ISI_MODE_MIPI ):
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: mode not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* field-selection */
    switch ( pConfig->FieldSelection )  /* only ISI_FIELDSEL_BOTH supported, no configuration needed */
    {
        case ISI_FIELDSEL_BOTH:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by sensor sensor, so the YCSequence parameter is not checked */
    switch ( pConfig->YCSequence )
    {
        default:
        {
            break;
        }
    }

    /* 422 conversion */
    switch ( pConfig->Conv422 )         /* only ISI_CONV422_NOCOSITED supported, no configuration needed */
    {
        case ISI_CONV422_NOCOSITED:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_GRGRBGBG supported, no configuration needed */
    {
        case ISI_BPAT_GRGRBGBG:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* horizontal polarity */
    switch ( pConfig->HPol )            /* only ISI_HPOL_REFPOS supported, no configuration needed */
    {
        case ISI_HPOL_REFPOS:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* vertical polarity */
    switch ( pConfig->VPol )            /* only ISI_VPOL_NEG supported, no configuration needed */
    {
        case ISI_VPOL_NEG:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }


    /* edge */
    switch ( pConfig->Edge )            /* only ISI_EDGE_RISING supported, no configuration needed */
    {
        case ISI_EDGE_RISING:
        {
            break;
        }

        case ISI_EDGE_FALLING:          /*TODO for MIPI debug*/
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* gamma */
    switch ( pConfig->Gamma )           /* only ISI_GAMMA_OFF supported, no configuration needed */
    {
        case ISI_GAMMA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* color conversion */
    switch ( pConfig->CConv )           /* only ISI_CCONV_OFF supported, no configuration needed */
    {
        case ISI_CCONV_OFF:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->SmiaMode )        /* only ISI_SMIA_OFF supported, no configuration needed */
    {
        case ISI_SMIA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->MipiMode )        /* only ISI_MIPI_MODE_RAW_12 supported, no configuration needed */
    {
        case ISI_MIPI_MODE_RAW_10:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->AfpsResolutions ) /* no configuration needed */
    {
        case ISI_AFPS_NOTSUPP:
        {
            break;
        }
        default:
        {
            // don't care about what comes in here
            //TRACE( sensor_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( sensor_INFO, "%s%s (exit)\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int sensor_get_PCLK( sensor_Context_t *psensorCtx, int XVCLK)
{
    // calculate PCLK
    /*uint32_t SCLK, temp1, temp2, temp3;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_sys_pre_div, Pll2_sys_divider2x, Sys_pre_div, sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_pre_div_map[] = {1, 2, 4, 1};

    
    //temp1 = ReadSCCB(0x6c, 0x3007);
    sensor_IsiRegReadIss(  psensorCtx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	sensor_IsiRegReadIss(  psensorCtx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	sensor_IsiRegReadIss(  psensorCtx, 0x030c, &temp1 );
	sensor_IsiRegReadIss(  psensorCtx, 0x030d, &temp3 );
	temp1 = temp1 & 0x03;
	temp2 = (temp1<<8) + temp3;
	if(!temp2) {
 		Pll2_multiplier = 1;
 	}
	else {
 	Pll2_multiplier = temp2;
	}
	
    sensor_IsiRegReadIss(  psensorCtx, 0x030f, &temp1 );
	temp1 = temp1 & 0x0f;
	Pll2_sys_pre_div = temp1 + 1;
	sensor_IsiRegReadIss(  psensorCtx, 0x030e, &temp1 );
	temp1 = temp1 & 0x07;
	Pll2_sys_divider2x = Pll2_sys_divider2x_map[temp1];

	sensor_IsiRegReadIss(  psensorCtx, 0x3106, &temp1 );
	temp2 = (temp1>>2) & 0x03;
	Sys_pre_div = Sys_pre_div_map[temp2];
    
    temp2 = (temp1>>4) & 0x0f;
	 if(!temp2) {
 		sclk_pdiv = 1;
 	}
	 else {
 		sclk_pdiv = temp2;
 	}
  	 SCLK = XVCLK * 4 / Pll2_prediv0 / Pll2_prediv2x * Pll2_multiplier /Pll2_sys_pre_div / Pll2_sys_divider2x / Sys_pre_div / sclk_pdiv;
	
	temp1 = SCLK>>8;
	sensor_IsiRegWriteIss(psensorCtx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	sensor_IsiRegWriteIss(psensorCtx, 0x350a, temp1);
	TRACE( sensor_INFO, "===============%s get_PCLK:%d==============\n", __FUNCTION__,SCLK);
	return SCLK*10000;*/
	return 168000000;
}

/*****************************************************************************/
/**
 *          sensor_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT sensor_SetupOutputWindowInternal
(
    sensor_Context_t        *psensorCtx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
	uint16_t usTimeHts;
	uint16_t usTimeVts;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 2400;
	TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(psensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	
		psensorCtx->IsiSensorMipiInfo.ulMipiFreq = 720;
		switch ( pConfig->Resolution )
		{
			case ISI_RES_1296_972P15:
			{
				TRACE( sensor_NOTICE1, "%s(%d): Resolution binning\n", __FUNCTION__,__LINE__ );
				result = IsiRegDefaultsApply( psensorCtx, sensor_g_binning_onelane);
				if ( result != RET_SUCCESS )
				{
					return ( result );
				}
				usTimeHts = 0x0518; //hkw
				usTimeVts = 0x03d0;
				/* sleep a while, that sensor can take over new default values */
				osSleep( 10 );
				break;
				
			}
			#if 0
			case ISI_RES_2596_1944P7:
			{
				TRACE( sensor_NOTICE1, "%s(%d): Resolution fullres\n", __FUNCTION__,__LINE__ );
				result = IsiRegDefaultsApply( psensorCtx, sensor_g_fullres_onelane);
				if ( result != RET_SUCCESS )
				{
					return ( result );
				}
				usTimeHts = 0x0f28;
				usTimeVts = 0x09aa;
				/* sleep a while, that sensor can take over new default values */
				osSleep( 10 );
				break;
				
			}
			#endif
	
			default:
			{
				TRACE( sensor_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
				return ( RET_NOTSUPP );
			}
		}
	} else if(psensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

        psensorCtx->IsiSensorMipiInfo.ulMipiFreq = 720;
    	switch ( pConfig->Resolution )
        {
            /*case ISI_RES_1296_972P30:
            {
                TRACE( sensor_NOTICE1, "%s(%d): Resolution binning\n", __FUNCTION__,__LINE__ );
    			result = IsiRegDefaultsApply( psensorCtx, sensor_g_binning_twolane);
    		    if ( result != RET_SUCCESS )
    		    {
    		        return ( result );
    		    }
    			usTimeHts = 0x0abe; 
                usTimeVts = 0x07c4;
    		    // sleep a while, that sensor can take over new default values 
    		    osSleep( 50 );
    			break;
                
            }*/
            
            case ISI_RES_2592_1944P15:
			case ISI_RES_2592_1944P30:
			case ISI_RES_2592_1944P25:
			case ISI_RES_2592_1944P20:
			case ISI_RES_2592_1944P10:
			case ISI_RES_2592_1944P7:
			case ISI_RES_2592_1944P5:
            {
                TRACE( sensor_NOTICE1, "%s(%d): Resolution fullres\n", __FUNCTION__,__LINE__ );
    		    if (set2Sensor == BOOL_TRUE) {
				//if(1){
					if (res_no_chg == BOOL_FALSE) {
						if((result = IsiRegDefaultsApply((IsiSensorHandle_t)psensorCtx, sensor_g_fullres_twolane)) != RET_SUCCESS){
							result = RET_FAILURE;
							TRACE( sensor_ERROR, "%s: failed to set two lane ISI_RES_4208_3120 \n", __FUNCTION__ );
			            }
					}
					if (pConfig->Resolution == ISI_RES_2592_1944P30) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P30_twolane_fpschg);
                	} else if (pConfig->Resolution == ISI_RES_2592_1944P25) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P25----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P25_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2592_1944P20) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P20----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P20_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2592_1944P15) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P15----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2592_1944P10) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P10----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P10_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2592_1944P7) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P7----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P7_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2592_1944P5) {
                    	TRACE( sensor_ERROR, "%s: ------ISI_RES_2592_1944P5----- \n", __FUNCTION__ );
                        result = IsiRegDefaultsApply( (IsiSensorHandle_t)psensorCtx, sensor_g_2592x1944P5_twolane_fpschg);
                    } 
				}
	            usTimeHts = 0x0abe;
				if (pConfig->Resolution == ISI_RES_2592_1944P30) {
	            	usFrameLengthLines = 0x7c4;//0x7c4,0x174c; //zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P25) {
	            	usFrameLengthLines = 0xa13;//0xa13,0x1bf3; //zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P20) {
	            	usFrameLengthLines = 0xba6; //0xba6,zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P15) {
	            	usFrameLengthLines = 0x11c0; //0x11c0,zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P10) {
	            	usFrameLengthLines = 0x174c; //0x174c,zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P7) {
	            	usFrameLengthLines = 0x3540; //0x3540,zyh
				}else if(pConfig->Resolution == ISI_RES_2592_1944P5) {
	            	usFrameLengthLines = 0x4a8c; //0x4a8czyh
				}
				psensorCtx->IsiSensorMipiInfo.ulMipiFreq = 640;
	            break;
	            
                
            }

            default:
            {
                TRACE( sensor_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
    } 	
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    
	usLineLengthPck = 0x0abe;//usTimeHts;
    usFrameLengthLines = usFrameLengthLines*1.5;//*2;
	rVtPixClkFreq = sensor_get_PCLK(psensorCtx, xclk);
    
    // store frame timing for later use in AEC module
    psensorCtx->VtPixClkFreq     = rVtPixClkFreq;
    psensorCtx->LineLengthPck    = usLineLengthPck;
    psensorCtx->FrameLengthLines = usFrameLengthLines;
	psensorCtx->AecMaxIntegrationTime = ( ((float)(psensorCtx->FrameLengthLines - 4)) * ((float)psensorCtx->LineLengthPck) ) / psensorCtx->VtPixClkFreq;


    TRACE( sensor_NOTICE1, "%s  Resolution: %dx%d@%dfps(exit)\n", __FUNCTION__, 
        ISI_RES_W_GET(pConfig->Resolution),
        ISI_RES_H_GET(pConfig->Resolution),
        ISI_FPS_GET(pConfig->Resolution));

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT sensor_SetupImageControl
(
    sensor_Context_t        *psensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = sensor_IsiRegReadIss(  psensorCtx, sensor_BLC_CTRL00, &RegValue );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = sensor_IsiRegReadIss(  psensorCtx, sensor_BLC_CTRL00, &RegValue );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = sensor_IsiRegReadIss(  psensorCtx, sensor_AEC_MANUAL, &RegValue );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = sensor_IsiRegReadIss(  psensorCtx, sensor_ISP_CTRL01, &RegValue );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = sensor_IsiRegReadIss(  psensorCtx, sensor_AEC_MANUAL, &RegValue );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = sensor_IsiRegReadIss( psensorCtx, sensor_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = sensor_IsiRegWriteIss( psensorCtx, sensor_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( sensor_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT sensor_SetupOutputWindow
(
    sensor_Context_t        *psensorCtx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(psensorCtx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(psensorCtx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return sensor_SetupOutputWindowInternal(psensorCtx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          sensor_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in sensor-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      sensor context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT sensor_AecSetModeParameters
(
    sensor_Context_t       *psensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s%s (enter)\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"");
#if 1
    if ( (psensorCtx->VtPixClkFreq == 0.0f) )
    {
        TRACE( sensor_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    psensorCtx->AecMaxIntegrationTime = ( ((float)(psensorCtx->FrameLengthLines-4)) * ((float)psensorCtx->LineLengthPck) ) / psensorCtx->VtPixClkFreq;
    psensorCtx->AecMinIntegrationTime = 0.0001f;

    TRACE( sensor_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"", psensorCtx->AecMaxIntegrationTime  );

    psensorCtx->AecMaxGain = sensor_MAX_GAIN_AEC;
    psensorCtx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    psensorCtx->AecIntegrationTimeIncrement = ((float)psensorCtx->LineLengthPck) / psensorCtx->VtPixClkFreq;
    psensorCtx->AecGainIncrement = sensor_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    psensorCtx->AecCurGain               = psensorCtx->AecMinGain;
    psensorCtx->AecCurIntegrationTime    = 0.0f;
    psensorCtx->OldCoarseIntegrationTime = 0;
    psensorCtx->OldFineIntegrationTime   = 0;
    //psensorCtx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time
#endif
    TRACE( sensor_INFO, "%s%s (exit)\n", __FUNCTION__, psensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}


/*****************************************************************************/
/**
 *          sensor_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT sensor_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( psensorCtx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &psensorCtx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = sensor_IsiRegWriteIss ( psensorCtx, sensor_SOFTWARE_RST, sensor_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( sensor_DEBUG, "%s: sensor System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = sensor_IsiRegWriteIss( psensorCtx, sensor_MODE_SELECT, sensor_MODE_SELECT_OFF );//sensor_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: Can't write sensor Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( psensorCtx, sensor_g_aRegDescription );
    if(psensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE)
	{
		//IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );
		//IsiWriteRegister(0x1b,,);
		 TRACE( sensor_ERROR, "%s: Derek 1111\n", __FUNCTION__ );
		IsiWriteRegister( handle, 0x0103, 0x01);
		 TRACE( sensor_ERROR, "%s: Derek 2222\n", __FUNCTION__ );
		osSleep( 100);//derek
	 TRACE( sensor_ERROR, "%s: Derek 3333\n", __FUNCTION__ );
        result = IsiRegDefaultsApply( psensorCtx, sensor_g_aRegDescription_twolane);
	}
	else if(psensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE)
        result = IsiRegDefaultsApply( psensorCtx, sensor_g_aRegDescription_onelane);
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 50 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( psensorCtx, sensor_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = sensor_IsiRegWriteIss( psensorCtx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = sensor_SetupOutputFormat( psensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = sensor_SetupOutputWindow( psensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = sensor_SetupImageControl( psensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = sensor_AecSetModeParameters( psensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        psensorCtx->Configured = BOOL_TRUE;
    }

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiChangeSensorResolutionIss
 *
 * @brief   Change image sensor resolution while keeping all other static settings.
 *          Dynamic settings like current gain & integration time are kept as
 *          close as possible. Sensor needs 2 frames to engage (first 2 frames
 *          are not correctly exposed!).
 *
 * @note    Re-read current & min/max values as they will probably have changed!
 *
 * @param   handle                  Sensor instance handle
 * @param   Resolution              new resolution ID
 * @param   pNumberOfFramesToSkip   reference to storage for number of frames to skip
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 * @retval  RET_OUTOFRANGE
 * 不用改
 *****************************************************************************/
static RESULT sensor_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (psensorCtx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    #if 0
    result = sensor_IsiGetCapsIss( handle, &Caps);
    if (RET_SUCCESS != result)
    {
        return result;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }
    #else
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (sensor_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }
    #endif

    if ( Resolution == psensorCtx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(psensorCtx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(psensorCtx->Config.Resolution))) ) {

            if (psensorCtx->Streaming != BOOL_FALSE) {
                TRACE( sensor_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( sensor_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        psensorCtx->Config.Resolution = Resolution;
#if 1 
        // tell sensor about that
        result = sensor_SetupOutputWindowInternal( psensorCtx, &psensorCtx->Config,BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( sensor_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = psensorCtx->AecCurGain;
        float OldIntegrationTime = psensorCtx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = sensor_AecSetModeParameters( psensorCtx, &psensorCtx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( sensor_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = sensor_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( sensor_ERROR, "%s: sensor_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }
        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
#endif
    }

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiSensorSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT sensor_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (psensorCtx->Configured != BOOL_TRUE) || (psensorCtx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = sensor_IsiRegReadIss ( psensorCtx, sensor_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = sensor_IsiRegWriteIss ( psensorCtx, sensor_MODE_SELECT, sensor_MODE_SELECT_ON );//sensor_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = sensor_IsiRegReadIss ( psensorCtx, sensor_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = sensor_IsiRegWriteIss ( psensorCtx, sensor_MODE_SELECT, sensor_MODE_SELECT_OFF );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        TRACE(sensor_INFO," STREAM OFF ++++++++++++++");
    }

    if (result == RET_SUCCESS)
    {
        psensorCtx->Streaming = on;
    }
	osSleep(50);
    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      sensor sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT sensor_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    psensorCtx->Configured = BOOL_FALSE;
    psensorCtx->Streaming  = BOOL_FALSE;

    //TRACE( sensor_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( sensor_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( sensor_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( sensor_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( sensor_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( sensor_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( psensorCtx->IsiCtx.HalHandle, psensorCtx->IsiCtx.HalDevID, false );

        osSleep( 50 );

	}

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT sensor_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = sensor_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 16U) | (sensor_CHIP_ID_MIDDLE_BYTE_DEFAULT<<8U);
    RevId = RevId | sensor_CHIP_ID_LOW_BYTE_DEFAULT;

    result = sensor_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( sensor_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( sensor_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetSensorRevisionIss
 *
 * @brief   reads the sensor revision register and returns this value
 *
 * @param   handle      pointer to sensor description struct
 * @param   p_value     pointer to storage value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT sensor_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = sensor_IsiRegReadIss ( handle, sensor_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 16U );
    result = sensor_IsiRegReadIss ( handle, sensor_CHIP_ID_MIDDLE_BYTE, &data );
    *p_value |= ( (data & 0xFF) << 8U );
    result = sensor_IsiRegReadIss ( handle, sensor_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiRegReadIss
 *
 * @brief   grants user read access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   p_value     pointer to value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT sensor_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, sensor_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( sensor_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiRegWriteIss
 *
 * @brief   grants user write access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   value       value to write
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * 不用改
 *****************************************************************************/
static RESULT sensor_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, sensor_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( sensor_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          sensor instance
 *
 * @param   handle       sensor sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT sensor_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( sensor_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = psensorCtx->AecMinGain;
    *pMaxGain = psensorCtx->AecMaxGain;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          sensor instance
 *
 * @param   handle       sensor sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT sensor_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( sensor_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = psensorCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = psensorCtx->AecMaxIntegrationTime;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          sensor_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT sensor_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t reg_gain= 0;
	float gain_tmp= 0; 
	float gain_result = 0;	
	sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

	//get coarse gain
	result = sensor_IsiRegReadIss ( psensorCtx, sensor_A_GAIN_ADDR_L, &reg_gain);
	TRACE( sensor_INFO, " -------reg0205:%x-------\n",reg_gain );
	switch(reg_gain){
		case 0x00:
			gain_tmp = 1.00;
			break;
		case 0x10:
			gain_tmp = 1.06;//1.07;
			break;
		case 0x20:
			gain_tmp = 1.09;// 1.10;
			break;
		case 0x30:
			gain_tmp = 1.20;
			break;	
		case 0x40:
			gain_tmp = 1.30;
			break;	
		case 0x50:
			gain_tmp = 1.50;
			break;	
		case 0x60:
			gain_tmp = 1.60;
			break;
		case 0x70:
			gain_tmp = 1.80;
			break;
		case 0x80:
			gain_tmp = 2.00;
			break;
		case 0x90:
			gain_tmp = 2.30;
			break;
		case 0xa0:
			gain_tmp = 2.70;
			break;
		case 0xb0:
			gain_tmp = 3.20;
			break;
		case 0xc0:
			gain_tmp = 4.00;
			break;
		case 0xd0:
			gain_tmp = 5.33;
			break;
		case 0xe0:
			gain_tmp = 8.00;
			break;
		case 0xe4:
			gain_tmp = 9.10;
			break;
		case 0xe8:
			gain_tmp = 10.70;
			break;
		case 0xec:
			gain_tmp = 12.80;
			break;
		case 0xf0:
			gain_tmp = 16.00;
			break;
		default:
			gain_tmp = 0;
			break;
	}
	gain_result = gain_tmp;
	
	//get fine gain
	result = sensor_IsiRegReadIss ( psensorCtx, sensor_D_GAIN_RED_ADDR_L, &reg_gain);
	TRACE( sensor_INFO, " -------reg0205:%x-------\n",reg_gain );
	switch(reg_gain){
		case 0x00:
			gain_tmp = 1.00;
			break;
		case 0x08:
			gain_tmp = 1.03;//1.07;
			break;
		case 0x10:
			gain_tmp = 1.06;// 1.10;
			break;
		case 0x18:
			gain_tmp = 1.09;
			break;	
		case 0x20:
			gain_tmp = 1.13;
			break;	
		case 0x28:
			gain_tmp = 1.16;
			break;	
		case 0x30:
			gain_tmp = 1.19;
			break;
		case 0x38:
			gain_tmp = 1.22;
			break;
		case 0x40:
			gain_tmp = 1.25;
			break;
		case 0x48:
			gain_tmp = 1.28;
			break;
		case 0x50:
			gain_tmp = 1.31;
			break;
		case 0x58:
			gain_tmp = 1.34;
			break;
		case 0x60:
			gain_tmp = 1.38;
			break;
		case 0x68:
			gain_tmp = 1.41;
			break;
		case 0x70:
			gain_tmp = 1.44;
			break;
		case 0x78:
			gain_tmp = 1.47;
			break;
		case 0x80:
			gain_tmp = 1.50;
			break;
		case 0x88:
			gain_tmp = 1.53;
			break;
		case 0x90:
			gain_tmp = 1.56;
			break;
		case 0x98:
			gain_tmp = 1.59;
			break;
		case 0xa0:
			gain_tmp = 1.63;
			break;
		case 0xa8:
			gain_tmp = 1.66;
			break;
		case 0xb0:
			gain_tmp = 1.69;
			break;
		case 0xb8:
			gain_tmp = 1.72;
			break;
		case 0xc0:
			gain_tmp = 1.75;
			break;
		case 0xc8:
			gain_tmp = 1.78;
			break;
		case 0xd0:
			gain_tmp = 1.81;
			break;
		case 0xd8:
			gain_tmp = 1.84;
			break;
		case 0xe0:
			gain_tmp = 1.88;
			break;
		case 0xe8:
			gain_tmp = 1.91;
			break;
		case 0xf0:
			gain_tmp = 1.94;
			break;
		case 0xf8:
			gain_tmp = 1.97;
			break;
		default:
			gain_tmp = 1;
			break;
			
	}
	gain_result *= gain_tmp;
	*pSetGain = ( (float)gain_result );
    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT sensor_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = psensorCtx->AecGainIncrement;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * 不用改；设置gain值
 *****************************************************************************/
RESULT sensor_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    float Gain_tmp = 0;
    uint16_t Reg_A_Gain = 0;
    uint16_t Reg_D_Gain = 0;
    uint16_t i = 0;
    uint32_t data= 0;
    float gain_result = 0.0;
    TRACE( sensor_INFO, "%s: (enter) psensorCtx->AecMaxGain(%f) \n", __FUNCTION__,psensorCtx->AecMaxGain);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < psensorCtx->AecMinGain ) NewGain = psensorCtx->AecMinGain;
    if( NewGain > psensorCtx->AecMaxGain ) NewGain = psensorCtx->AecMaxGain;


    //To find the approximate coarse gain
    Gain_tmp = (NewGain*100);
    do {
        if(Gain_tmp < sensor_A_gain_table[i].gain_value)
            break;
        i++;
    } while(sensor_A_gain_table[i].gain_value != 0);
    if( i == 0 ) {
        Reg_A_Gain  = 0;
        gain_result = 0;
    } else {
	    Reg_A_Gain = sensor_A_gain_table[i-1].reg_gain;
	    gain_result = sensor_A_gain_table[i-1].gain_value/100.0;
	}
    Gain_tmp = (Gain_tmp*100.0)/(float)sensor_A_gain_table[i-1].gain_value;
    TRACE( sensor_DEBUG,"%s============Gain_tmp:%lf===============1", __FUNCTION__,Gain_tmp);
    
	
    //use the fine gain to fix   
    i = 0;
    do {
        if(Gain_tmp < sensor_D_gain_table[i].gain_value)
            break;
        i++;
    } while(sensor_D_gain_table[i].gain_value != 0);
    if( i == 0) {
	    Reg_D_Gain = 0;
    } else {
	    Reg_D_Gain = sensor_D_gain_table[i-1].reg_gain;
    }
    TRACE( sensor_DEBUG,"%s============Reg_A_Gain:%d Reg_D_Gain:%d===============2", __FUNCTION__,Reg_A_Gain,Reg_D_Gain);
	
    if( Reg_A_Gain != 0 && Reg_D_Gain != 0) {
        gain_result = gain_result * sensor_D_gain_table[i-1].gain_value / 100.0;
    } else if (Reg_D_Gain == 0) {
        gain_result = gain_result;
    } else if (Reg_A_Gain == 0){
        gain_result = sensor_D_gain_table[i-1].gain_value/100.0;
    } else {
        gain_result = 0 ; 
    }
    TRACE( sensor_DEBUG,"%s============i:%d gain_result:%lf===============2", __FUNCTION__,i,gain_result);
	
    if( ((gain_result) != psensorCtx->OldGain) )
     {
        result = sensor_IsiRegWriteIss( psensorCtx, 0x0104, 0x01 );
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_A_GAIN_ADDR_L,        Reg_A_Gain & 0xff);
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_D_GAIN_GREENR_ADDR_L, Reg_D_Gain & 0xff);
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_D_GAIN_RED_ADDR_L,    Reg_D_Gain & 0xff);
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_D_GAIN_BLUE_ADDR_L,   Reg_D_Gain & 0xff);
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_D_GAIN_GREENB_ADDR_L, Reg_D_Gain & 0xff);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        psensorCtx->OldGain = gain_result;
        result = sensor_IsiRegWriteIss( psensorCtx, 0x0104, 0x00 );
    }

    //calculate gain actually set
    psensorCtx->AecCurGain = gain_result;
    //return current state
    *pSetGain = psensorCtx->AecCurGain;
    TRACE( sensor_DEBUG,"-----------%s: psetgain=%f, NewGain=%f\n", __FUNCTION__, *pSetGain, NewGain);

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          sensor_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT sensor_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = psensorCtx->AecCurIntegrationTime;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT sensor_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = psensorCtx->AecIntegrationTimeIncrement;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   NewIntegrationTime      integration time to be set
 * @param   pSetIntegrationTime     set integration time
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 *设置曝光时间；根据应用手册修改寄存器宏
 *****************************************************************************/
RESULT sensor_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;
	uint32_t result_intertime= 0;
	
    //uint32_t FineIntegrationTime   = 0; //not supported by sensor

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > psensorCtx->AecMaxIntegrationTime ) NewIntegrationTime = psensorCtx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < psensorCtx->AecMinIntegrationTime ) NewIntegrationTime = psensorCtx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by sensor
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)psensorCtx->VtPixClkFreq );

    // avoid division by zero
    if ( psensorCtx->LineLengthPck == 0 )
    {
        TRACE( sensor_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)psensorCtx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * psensorCtx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)psensorCtx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != psensorCtx->OldCoarseIntegrationTime )
    {//

        result = sensor_IsiRegWriteIss( psensorCtx, 0x0104, 0x01 );
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_AEC_FINE_EXPO_L, 0xae );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_AEC_COARSE_EXPO_H, (CoarseIntegrationTime & 0x0000FF00U)>> 8 );
		TRACE( sensor_DEBUG, " -------reg0x0202:%x-------\n",(CoarseIntegrationTime & 0x0000FF00U) >> 8);

		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = sensor_IsiRegWriteIss( psensorCtx, sensor_AEC_COARSE_EXPO_L, (CoarseIntegrationTime & 0x000000FFU) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( sensor_DEBUG, " -------reg0x0203:%x-------\n",(CoarseIntegrationTime & 0x000000FFU)  );

        result = sensor_IsiRegWriteIss( psensorCtx, 0x0104, 0x00 );
        psensorCtx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
#if 1//Eric added for test
		CoarseIntegrationTime =CoarseIntegrationTime+10;
		
		if (CoarseIntegrationTime<0x07C4)
		{
		  sensor_IsiRegWriteIss(psensorCtx,0x0340,	0x07);
		  sensor_IsiRegWriteIss(psensorCtx,0x0341,	0xC4);
		}	
		else
		{
		  sensor_IsiRegWriteIss(psensorCtx,0x0340,	(CoarseIntegrationTime >> 8) & 0xFF);
		  sensor_IsiRegWriteIss(psensorCtx,0x0341,	CoarseIntegrationTime	& 0xFF);
		
		}	
#endif
       /* 
		osSleep(30);
		result = sensor_IsiRegReadIss ( psensorCtx, sensor_AEC_FINE_EXPO_L, &data);
		TRACE( sensor_ERROR, " -------reg0201:%x-------\n",data );
		result_intertime = (data & 0x0f) << 8;
		result = sensor_IsiRegReadIss ( psensorCtx, sensor_AEC_COARSE_EXPO_H, &data);
		TRACE( sensor_ERROR, " -------reg0202:%x-------\n",data );
		result_intertime = result_intertime + data;
		result = sensor_IsiRegReadIss ( psensorCtx, sensor_AEC_COARSE_EXPO_L, &data);
		TRACE( sensor_ERROR, " -------reg0203:%x-------\n",data );
		//result_intertime = (result_intertime << 4) + (data >> 4);
	    */	
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }
    psensorCtx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)psensorCtx->LineLengthPck) / psensorCtx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = psensorCtx->AecCurIntegrationTime;

    TRACE( sensor_DEBUG, "---------%s:psensorCtx->VtPixClkFreq:%f;psensorCtx->LineLengthPck:%x\n-------SetTi=%f NewTi=%f  CoarseIntegrationTime=%x,result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, psensorCtx->VtPixClkFreq,psensorCtx->LineLengthPck,*pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,result_intertime,(CoarseIntegrationTime & 0x0000F000U) >> 12U ,(CoarseIntegrationTime & 0x00000FF0U) >> 4U,(CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          sensor_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  sensor sensor instance handle
 * @param   NewGain                 newly calculated gain to be set
 * @param   NewIntegrationTime      newly calculated integration time to be set
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 * 不用改，设置整个曝光；
 *****************************************************************************/
RESULT sensor_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( sensor_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = sensor_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = sensor_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( sensor_DEBUG, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT sensor_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = psensorCtx->AecCurGain;
    *pSetIntegrationTime = psensorCtx->AecCurIntegrationTime;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetResolutionIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSettResolution         set resolution
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
RESULT sensor_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = psensorCtx->Config.Resolution;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   psensorCtx             sensor sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT sensor_IsiGetAfpsInfoHelperIss(
    sensor_Context_t   *psensorCtx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(psensorCtx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    psensorCtx->Config.Resolution = Resolution;

    // tell sensor about that
    result = sensor_SetupOutputWindowInternal( psensorCtx, &psensorCtx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = sensor_AecSetModeParameters( psensorCtx, &psensorCtx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( sensor_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = psensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = psensorCtx->AecMinGain;
    pAfpsInfo->AecMaxGain           = psensorCtx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = psensorCtx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = psensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          sensor_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  sensor sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 * 不用改；没用；
 *****************************************************************************/
RESULT sensor_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        TRACE( sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = psensorCtx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = psensorCtx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = psensorCtx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = psensorCtx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    sensor_Context_t *pDummyCtx = (sensor_Context_t*) malloc( sizeof(sensor_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( sensor_ERROR,  "%s: Can't allocate dummy sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *psensorCtx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = sensor_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
        if ( lres == RET_SUCCESS ) \
        { \
            ++idx; \
        } \
        else \
        { \
            UPDATE_RESULT( result, lres ); \
        } \
    }

    // check which AFPS series is requested and build its params list for the enabled AFPS resolutions


				switch(Resolution)
			    {
			        default:
			            TRACE( sensor_ERROR,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
			            result = RET_NOTSUPP;
			            break;
					#if 0
					case ISI_RES_1296_972P30:
					case ISI_RES_1296_972P25:
					case ISI_RES_1296_972P20:
					case ISI_RES_1296_972P15:
					case ISI_RES_1296_972P10:
						//TRACE( sensor_ERROR, "%s: (99999exit)\n", __FUNCTION__);
						AFPSCHECKANDADD( ISI_RES_1296_972P30);
						AFPSCHECKANDADD( ISI_RES_1296_972P25);
						AFPSCHECKANDADD( ISI_RES_1296_972P20);
						AFPSCHECKANDADD( ISI_RES_1296_972P15);
						AFPSCHECKANDADD( ISI_RES_1296_972P10);
						break;
					#endif
					case ISI_RES_2592_1944P30:
					case ISI_RES_2592_1944P25:
					case ISI_RES_2592_1944P20:
					case ISI_RES_2592_1944P15:
					case ISI_RES_2592_1944P10:
					case ISI_RES_2592_1944P7:
					case ISI_RES_2592_1944P5:
						//TRACE( sensor_ERROR, "%s: (88888exit)\n", __FUNCTION__);
						AFPSCHECKANDADD( ISI_RES_2592_1944P30);
						AFPSCHECKANDADD( ISI_RES_2592_1944P25);
						AFPSCHECKANDADD( ISI_RES_2592_1944P20);
						AFPSCHECKANDADD( ISI_RES_2592_1944P15);
						AFPSCHECKANDADD( ISI_RES_2592_1944P10);
						AFPSCHECKANDADD( ISI_RES_2592_1944P7);
						AFPSCHECKANDADD( ISI_RES_2592_1944P5);
						break;
						// check next series here...
			        #if 0
			        // 1080p15 series in ascending integration time order (most probably the same as descending frame rate order)
			        case ISI_RES_TV1080P15:
			        case ISI_RES_TV1080P10:
			        case ISI_RES_TV1080P5:
			            AFPSCHECKANDADD( ISI_RES_TV1080P15 );
			            AFPSCHECKANDADD( ISI_RES_TV1080P10 );
			            AFPSCHECKANDADD( ISI_RES_TV1080P5  );
			            break;
			        #endif
				}
            

    // release dummy context again
    free(pDummyCtx);

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibKFactor
 *
 * @brief   Returns the sensor specific K-Factor
 *
 * @param   handle       sensor sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT sensor_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&sensor_KFactor;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          sensor_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the sensor specific PCA-Matrix
 *
 * @param   handle          sensor sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT sensor_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&sensor_PCAMatrix;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&sensor_SVDMeanValue;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&sensor_CenterLine;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&sensor_AwbClipParm;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&sensor_AwbGlobalFadeParm;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&sensor_AwbFade2Parm;

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          sensor_IsiGetIlluProfile
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	//return ( result );
	//#if 0
    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    	#if 0
        uint16_t i;

        *ptIsiIlluProfile = NULL;

        /* check if we've a default profile */
        for ( i=0U; i<sensor_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( sensor_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &sensor_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiGetLscMatrixTable
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT sensor_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    	#if 0
        uint16_t i;


        switch ( CieProfile )
        {
            case ISI_CIEPROF_A:
            {
                if ( ( psensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( psensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( psensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( psensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( psensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( psensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( psensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( psensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( psensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( psensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &sensor_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( sensor_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          sensor_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT sensor_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          sensor sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT sensor_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    TRACE( sensor_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
 if ((psensorCtx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(psensorCtx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((psensorCtx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( sensor_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = sensor_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( sensor_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          sensor sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT sensor_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
	sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( sensor_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( sensor_ERROR, "%s: psensorCtx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = psensorCtx->VcmInfo.StartCurrent;
    else 
        nPosition = psensorCtx->VcmInfo.StartCurrent + (psensorCtx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( sensor_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | psensorCtx->VcmInfo.StepMode );
	
    //TRACE( sensor_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( psensorCtx->IsiCtx.HalHandle,
                             psensorCtx->IsiCtx.I2cAfBusNum,
                             psensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             psensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( sensor_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          sensor sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT sensor_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( sensor_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( psensorCtx->IsiCtx.HalHandle,
                            psensorCtx->IsiCtx.I2cAfBusNum,
                            psensorCtx->IsiCtx.SlaveAfAddress,
                            0,
                            psensorCtx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( sensor_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
	#if 1
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

    /*  //map 0 to 64 -> infinity 
    if( *pAbsStep == 0 )
    {
        *pAbsStep = MAX_LOG;
    }
    else
    {
        *pAbsStep = ( MAX_REG - *pAbsStep ) / 16U;
    }*/
	if( *pAbsStep <= psensorCtx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>psensorCtx->VcmInfo.StartCurrent) && (*pAbsStep<=psensorCtx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (psensorCtx->VcmInfo.RatedCurrent - *pAbsStep ) / psensorCtx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( sensor_ERROR, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT sensor_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          sensor_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT sensor_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = sensor_IsiRegReadIss( psensorCtx, sensor_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = sensor_IsiRegWriteIss( psensorCtx, sensor_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = sensor_IsiRegReadIss( psensorCtx, sensor_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = sensor_IsiRegWriteIss( psensorCtx, sensor_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     psensorCtx->TestPattern = enable;
    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          sensor_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT sensor_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = psensorCtx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= psensorCtx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = psensorCtx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT sensor_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
    	TRACE( sensor_ERROR, "%s: psensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( sensor_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT sensor_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    sensor_Context_t *psensorCtx = (sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( psensorCtx == NULL )
    {
    	TRACE( sensor_ERROR, "%s: psensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( sensor_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = sensor_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          sensor_IsiGetSensorIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor description struct
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT sensor_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = sensor_g_acName;
        pIsiSensor->pRegisterTable                      = sensor_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &sensor_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= sensor_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= sensor_IsiGetSensorTuningXmlVersion;//oyyf
        pIsiSensor->pIsiCreateSensorIss                 = sensor_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = sensor_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = sensor_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = sensor_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = sensor_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = sensor_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = sensor_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = sensor_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = sensor_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = sensor_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = sensor_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = sensor_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = sensor_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = sensor_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = sensor_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = sensor_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = sensor_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = sensor_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = sensor_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = sensor_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = sensor_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = sensor_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = sensor_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = sensor_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = sensor_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = sensor_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = sensor_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = sensor_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = sensor_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = sensor_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = sensor_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = sensor_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = sensor_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = sensor_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = sensor_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = sensor_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = sensor_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = sensor_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = sensor_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT sensor_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( sensor_ERROR,  "%s: Can't allocate sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = sensor_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = sensor_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = sensor_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = sensor_SOFTWARE_RST_VALUE;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps;
        uint32_t lanes,i;        

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(sensor_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
                    pCaps = malloc(sizeof(sensor_caps_t));
                    if (pCaps != NULL) {
                        memcpy(&pCaps->caps,&Caps,sizeof(IsiSensorCaps_t));
                        ListPrepareItem(pCaps);
                        ListAddTail(&pSensorI2cInfo->lane_res[i], pCaps);
                    }
                    Caps.Index++;
                }
            }
        }
    }
    
    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = sensor_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = sensor_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = sensor_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = sensor_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
    return RET_SUCCESS;
}

/******************************************************************************
 * See header file for detailed comment.
 *****************************************************************************/


/*****************************************************************************/
/**
 */
/*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig =
{
    0,
    sensor_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,						/**< IsiSensor_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiSensor_t.pIsiCheckOTPInfo>*/  //zyc
        0,						/**< IsiSensor_t.pIsiSetSensorOTPInfo>*/  //zyl
        0,						/**< IsiSensor_t.pIsiEnableSensorOTP>*/  //zyl
        0,                      /**< IsiSensor_t.pIsiCreateSensorIss */
        0,                      /**< IsiSensor_t.pIsiReleaseSensorIss */
        0,                      /**< IsiSensor_t.pIsiGetCapsIss */
        0,                      /**< IsiSensor_t.pIsiSetupSensorIss */
        0,                      /**< IsiSensor_t.pIsiChangeSensorResolutionIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetStreamingIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetPowerIss */
        0,                      /**< IsiSensor_t.pIsiCheckSensorConnectionIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorRevisionIss */
        0,                      /**< IsiSensor_t.pIsiRegisterReadIss */
        0,                      /**< IsiSensor_t.pIsiRegisterWriteIss */

        0,                      /**< IsiSensor_t.pIsiExposureControlIss */
        0,                      /**< IsiSensor_t.pIsiGetGainLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetCurrentExposureIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetResolutionIss */
        0,                      /**< IsiSensor_t.pIsiGetAfpsInfoIss */

        0,                      /**< IsiSensor_t.pIsiGetCalibKFactor */
        0,                      /**< IsiSensor_t.pIsiGetCalibPcaMatrix */
        0,                      /**< IsiSensor_t.pIsiGetCalibSvdMeanValue */
        0,                      /**< IsiSensor_t.pIsiGetCalibCenterLine */
        0,                      /**< IsiSensor_t.pIsiGetCalibClipParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibGlobalFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetIlluProfile */
        0,                      /**< IsiSensor_t.pIsiGetLscMatrixTable */

        0,                      /**< IsiSensor_t.pIsiMdiInitMotoDriveMds */
        0,                      /**< IsiSensor_t.pIsiMdiSetupMotoDrive */
        0,                      /**< IsiSensor_t.pIsiMdiFocusSet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusGet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusCalibrate */

        0,                      /**< IsiSensor_t.pIsiGetSensorMipiInfoIss */

        0,                      /**< IsiSensor_t.pIsiActivateTestPattern */
    },
    sensor_IsiGetSensorI2cInfo,
};


