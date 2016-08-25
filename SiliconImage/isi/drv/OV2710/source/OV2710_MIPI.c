
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
 * @file OV2710.c
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

#include "OV2710_MIPI_priv.h"

#define  OV2710_NEWEST_TUNING_XML "18-7-2014_oyyf-hkw_OV2710_CMK-CB0407-FV1_v0.1.2"

//hkw no use;
#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER(OV2710_INFO, "OV2710: ", INFO,      0U);
CREATE_TRACER(OV2710_WARN, "OV2710: ", WARNING,   1U);
CREATE_TRACER( OV2710_ERROR, "OV2710: ", ERROR,   1U);

CREATE_TRACER(OV2710_DEBUG, "OV2710: ", INFO, 0U);

CREATE_TRACER( OV2710_NOTICE0 , "OV2710: ", TRACE_NOTICE0, 1);
CREATE_TRACER( OV2710_NOTICE1, "OV2710: ", TRACE_NOTICE1, 1U );


#define OV2710_SLAVE_ADDR       0x6cU                           /**< i2c slave address of the OV2710 camera sensor */
#define OV2710_SLAVE_ADDR2      0x20U
#define OV2710_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the OV2710 integrated AD5820 */

#define OV2710_MIN_GAIN_STEP   (1.0f / 16.0f); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define OV2710_MAX_GAIN_AEC    (10.0f)            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */

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
#define MDI_SLEW_RATE_CTRL 6U /* S3..0 */



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV2710_g_acName[] = "OV2710_MIPI";

extern const IsiRegDescription_t OV2710_g_aRegDescription_onelane[];

extern const IsiRegDescription_t OV2710_g_1920x1080_onelane[];

const IsiSensorCaps_t OV2710_g_IsiSensorDefaultConfig;



#define OV2710_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define OV2710_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define OV2710_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_ONE_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV2710_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV2710_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV2710_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV2710_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV2710_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV2710_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV2710_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV2710_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV2710_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV2710_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV2710_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV2710_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV2710_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT OV2710_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV2710_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV2710_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV2710_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV2710_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV2710_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV2710_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT OV2710_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV2710_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV2710_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV2710_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV2710_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV2710_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV2710_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV2710_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV2710_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV2710_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV2710_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV2710_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV2710_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV2710_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV2710_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT OV2710_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT OV2710_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT OV2710_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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
 *          OV2710_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV2710 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV2710_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    OV2710_Context_t *pOV2710Ctx;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV2710Ctx = ( OV2710_Context_t * )malloc ( sizeof (OV2710_Context_t) );
    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR,  "%s: Can't allocate OV2710 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV2710Ctx, 0, sizeof( OV2710_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV2710Ctx );
        return ( result );
    }
    
    pOV2710Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pOV2710Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pOV2710Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pOV2710Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? OV2710_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV2710Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pOV2710Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pOV2710Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? OV2710_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV2710Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pOV2710Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pOV2710Ctx->Configured             = BOOL_FALSE;
    pOV2710Ctx->Streaming              = BOOL_FALSE;
    pOV2710Ctx->TestPattern            = BOOL_FALSE;
    pOV2710Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV2710Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pOV2710Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV2710Ctx->VcmInfo.RatedCurrent   = pOV2710Ctx->VcmInfo.StartCurrent + MAX_LOG*pOV2710Ctx->VcmInfo.Step;
    pOV2710Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pOV2710Ctx->IsiSensorMipiInfo.sensorHalDevID = pOV2710Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( OV2710_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pOV2710Ctx;

    result = HalSetCamConfig( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, false, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV2710 sensor instance.
 *
 * @param   handle      OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV2710_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV2710_IsiSensorSetStreamingIss( pOV2710Ctx, BOOL_FALSE );
    (void)OV2710_IsiSensorSetPowerIss( pOV2710Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV2710Ctx->IsiCtx.HalHandle );

    MEMSET( pOV2710Ctx, 0, sizeof( OV2710_Context_t ) );
    free ( pOV2710Ctx );

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCapsIss
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
static RESULT OV2710_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t  mipi_lanes
)
{
    RESULT result = RET_SUCCESS;
    TRACE( OV2710_INFO, "%s (enter pIsiSensorCaps:%x)\n", __FUNCTION__, pIsiSensorCaps);
    
    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        if(mipi_lanes == SUPPORT_MIPI_ONE_LANE) {
            TRACE( OV2710_INFO, "%s (ONE LANE Index:%d)\n", __FUNCTION__, pIsiSensorCaps->Index);
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
                    TRACE( OV2710_INFO, "%s (resolution 10809)\n", __FUNCTION__);
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
        pIsiSensorCaps->BPat            = ISI_BPAT_BGBGGRGR;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_NEG;
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL; 
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10; 
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP );
	pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }
end:
    TRACE( OV2710_INFO, "%s (result %x)\n", __FUNCTION__, result);
    return result;
}
 
static RESULT OV2710_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = OV2710_IsiGetCapsIssInternal(pIsiSensorCaps,pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV2710_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_NEG,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_TV1080P30, 
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
 *          OV2710_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2710 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_SetupOutputFormat
(
    OV2710_Context_t       *pOV2710Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s%s (enter)\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV2710 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV2710_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
        case ISI_BPAT_BGBGGRGR:
        {
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2710_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( OV2710_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV2710_INFO, "%s%s (exit)\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int OV2710_get_PCLK( OV2710_Context_t *pOV2710Ctx, int XVCLK)
{
    // calculate PCLK
    uint32_t SCLK, temp1, temp2, temp3;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_sys_pre_div, Pll2_sys_divider2x, Sys_pre_div, sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_pre_div_map[] = {1, 2, 4, 1};
    
    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);
    //temp1 = ReadSCCB(0x6c, 0x3007);
    OV2710_IsiRegReadIss(  pOV2710Ctx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	OV2710_IsiRegReadIss(  pOV2710Ctx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	OV2710_IsiRegReadIss(  pOV2710Ctx, 0x030c, &temp1 );
	OV2710_IsiRegReadIss(  pOV2710Ctx, 0x030d, &temp3 );
	temp1 = temp1 & 0x03;
	temp2 = (temp1<<8) + temp3;
	if(!temp2) {
 		Pll2_multiplier = 1;
 	}
	else {
 	Pll2_multiplier = temp2;
	}
	
    OV2710_IsiRegReadIss(  pOV2710Ctx, 0x030f, &temp1 );
	temp1 = temp1 & 0x0f;
	Pll2_sys_pre_div = temp1 + 1;
	OV2710_IsiRegReadIss(  pOV2710Ctx, 0x030e, &temp1 );
	temp1 = temp1 & 0x07;
	Pll2_sys_divider2x = Pll2_sys_divider2x_map[temp1];

	OV2710_IsiRegReadIss(  pOV2710Ctx, 0x3106, &temp1 );
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
	OV2710_IsiRegWriteIss(pOV2710Ctx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	OV2710_IsiRegWriteIss(pOV2710Ctx, 0x350a, temp1);
	return SCLK*10000;
}

/*****************************************************************************/
/**
 *          OV2710_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2710 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/

static RESULT OV2710_SetupOutputWindowInternal
(
    OV2710_Context_t        *pOV2710Ctx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 2400;
    uint32_t RegValue;

	if(pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	
		switch ( pConfig->Resolution )
		{
			case ISI_RES_TV1080P30:
			{	
				if (set2Sensor == BOOL_TRUE) {
					TRACE(OV2710_INFO, "%s(%d): Resolution 1920x1080\n", __FUNCTION__, __LINE__);
					result = IsiRegDefaultsApply(pOV2710Ctx, OV2710_g_1920x1080_onelane);
					if (result != RET_SUCCESS)
					{
						return (result);
					}
				}
				//HTS
				RegValue = 0;
				OV2710_IsiRegReadIss(  pOV2710Ctx, 0x380C, &RegValue );
				usLineLengthPck = (RegValue & 0x0f) << 8;
				OV2710_IsiRegReadIss(  pOV2710Ctx, 0x380D, &RegValue );
				usLineLengthPck |= RegValue & 0x0ff;
				
				//VTS
				RegValue = 0;
				OV2710_IsiRegReadIss(  pOV2710Ctx, 0x380E, &RegValue );
				usFrameLengthLines = (RegValue & 0x0f) << 8;
				OV2710_IsiRegReadIss(  pOV2710Ctx, 0x380E, &RegValue );
				usFrameLengthLines |= RegValue & 0x0ff; 
				pOV2710Ctx->IsiSensorMipiInfo.ulMipiFreq = 800;
				TRACE( OV2710_ERROR, "HTS:0x%x VTS:0x%x\n", usLineLengthPck, usFrameLengthLines);
				osSleep(10);		
				break;
			}
	
			default:
			{
				TRACE( OV2710_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
				return ( RET_NOTSUPP );
			}
		}
	}
    
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
   
    //rVtPixClkFreq = OV2710_get_PCLK(pOV2710Ctx, xclk);
    rVtPixClkFreq = 80000000;
    // store frame timing for later use in AEC module
    pOV2710Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV2710Ctx->LineLengthPck    = usLineLengthPck;
    pOV2710Ctx->FrameLengthLines = usFrameLengthLines;
    pOV2710Ctx->AecMaxIntegrationTime = (((float)(pOV2710Ctx->FrameLengthLines - 4)) * ((float)pOV2710Ctx->LineLengthPck)) / pOV2710Ctx->VtPixClkFreq;
	
    TRACE( OV2710_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV2710 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_SetupImageControl
(
    OV2710_Context_t        *pOV2710Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = OV2710_IsiRegReadIss(  pOV2710Ctx, OV2710_BLC_CTRL00, &RegValue );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = OV2710_IsiRegReadIss(  pOV2710Ctx, OV2710_BLC_CTRL00, &RegValue );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = OV2710_IsiRegReadIss(  pOV2710Ctx, OV2710_AEC_MANUAL, &RegValue );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = OV2710_IsiRegReadIss(  pOV2710Ctx, OV2710_ISP_CTRL01, &RegValue );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = OV2710_IsiRegReadIss(  pOV2710Ctx, OV2710_AEC_MANUAL, &RegValue );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = OV2710_IsiRegReadIss( pOV2710Ctx, OV2710_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = OV2710_IsiRegReadIss( pOV2710Ctx, OV2710_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV2710_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT OV2710_SetupOutputWindow
(
    OV2710_Context_t        *pOV2710Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV2710Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV2710Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return OV2710_SetupOutputWindowInternal(pOV2710Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          OV2710_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV2710-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV2710 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_AecSetModeParameters
(
    OV2710_Context_t       *pOV2710Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"",
        pOV2710Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pOV2710Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( OV2710_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pOV2710Ctx->AecMaxIntegrationTime = ( ((float)(pOV2710Ctx->FrameLengthLines - 4)) * ((float)pOV2710Ctx->LineLengthPck) ) / pOV2710Ctx->VtPixClkFreq;
    pOV2710Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( OV2710_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"", pOV2710Ctx->AecMaxIntegrationTime  );

    pOV2710Ctx->AecMaxGain = OV2710_MAX_GAIN_AEC;
    pOV2710Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pOV2710Ctx->AecIntegrationTimeIncrement = ((float)pOV2710Ctx->LineLengthPck) / pOV2710Ctx->VtPixClkFreq;
    pOV2710Ctx->AecGainIncrement = OV2710_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pOV2710Ctx->AecCurGain               = pOV2710Ctx->AecMinGain;
    pOV2710Ctx->AecCurIntegrationTime    = 0.0f;
    pOV2710Ctx->OldCoarseIntegrationTime = 0;
    pOV2710Ctx->OldFineIntegrationTime   = 0;
    //pOV2710Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( OV2710_INFO, "%s%s (exit)\n", __FUNCTION__, pOV2710Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2710_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2710 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV2710Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV2710Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV2710_IsiRegWriteIss ( pOV2710Ctx, OV2710_SOFTWARE_RST, OV2710_SOFTWARE_RST_VALUE );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( OV2710_DEBUG, "%s: OV2710 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_MODE_SELECT, OV2710_MODE_SELECT_OFF );//OV2710_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: Can't write OV2710 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    if(pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
		result = IsiRegDefaultsApply( pOV2710Ctx, OV2710_g_aRegDescription_onelane);
    }
    
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }    

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );

    #if 0
    /* 3.) verify default values to make sure everything has been written correctly as expected */
    result = IsiRegDefaultsVerify( pOV2710Ctx, OV2710_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = OV2710_SetupOutputFormat( pOV2710Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = OV2710_SetupOutputWindow( pOV2710Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV2710_SetupImageControl( pOV2710Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV2710_AecSetModeParameters( pOV2710Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }

    if (result == RET_SUCCESS)
    {
        pOV2710Ctx->Configured = BOOL_TRUE;
    }
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiChangeSensorResolutionIss
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
 *
 *****************************************************************************/
static RESULT OV2710_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV2710Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (OV2710_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV2710Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV2710Ctx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV2710Ctx->Config.Resolution))) ) {

            if (pOV2710Ctx->Streaming != BOOL_FALSE) {
                TRACE( OV2710_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV2710_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pOV2710Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV2710_SetupOutputWindowInternal( pOV2710Ctx, &pOV2710Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2710_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV2710Ctx->AecCurGain;
        float OldIntegrationTime = pOV2710Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV2710_AecSetModeParameters( pOV2710Ctx, &pOV2710Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2710_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV2710_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2710_ERROR, "%s: OV2710_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( OV2710_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2710_IsiSensorSetStreamingIss
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
static RESULT OV2710_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV2710Ctx->Configured != BOOL_TRUE) || (pOV2710Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, OV2710_MODE_SELECT, OV2710_MODE_SELECT_ON);
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, 0x4201, 0x00);
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, 0x4202, 0x00);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, OV2710_MODE_SELECT, OV2710_MODE_SELECT_OFF);
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, 0x4201, 0x00);
	result = OV2710_IsiRegWriteIss(pOV2710Ctx, 0x4202, 0x0f);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV2710Ctx->Streaming = on;
    }

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV2710 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV2710Ctx->Configured = BOOL_FALSE;
    pOV2710Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV2710_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV2710_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        TRACE( OV2710_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2710_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2710_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2710_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV2710Ctx->IsiCtx.HalHandle, pOV2710Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV2710_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | OV2710_CHIP_ID_LOW_BYTE_DEFAULT;

    result = OV2710_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV2710_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( OV2710_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetSensorRevisionIss
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
static RESULT OV2710_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV2710_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV2710_IsiRegReadIss ( handle, OV2710_CHIP_ID_HIGH_BYTE, &data );
    *p_value |= ( (data & 0xFF) << 8U );
    result = OV2710_IsiRegReadIss ( handle, OV2710_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiRegReadIss
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
 *
 *****************************************************************************/
static RESULT OV2710_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV2710_g_aRegDescription_onelane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;    
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( OV2710_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiRegWriteIss
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
 *
 *****************************************************************************/
static RESULT OV2710_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV2710_g_aRegDescription_onelane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( OV2710_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV2710 instance
 *
 * @param   handle       OV2710 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV2710_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pOV2710Ctx->AecMinGain;
    *pMaxGain = pOV2710Ctx->AecMaxGain;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV2710 instance
 *
 * @param   handle       OV2710 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV2710_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pOV2710Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pOV2710Ctx->AecMaxIntegrationTime;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2710_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }
	
    *pSetGain = pOV2710Ctx->AecCurGain;
	TRACE( OV2710_ERROR, "Get Gain:%f\n", *pSetGain);
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV2710Ctx->AecGainIncrement;
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT OV2710_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t gainlow = 0, gainlow_14b = 0;
	uint16_t tmp_gain_val = 0, usGain = 0;
    TRACE( OV2710_INFO, "%s: (enter) pOV2710Ctx->AecMaxGain(%f) \n", __FUNCTION__,pOV2710Ctx->AecMaxGain);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( OV2710_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pOV2710Ctx->AecMinGain ) NewGain = pOV2710Ctx->AecMinGain;
    if( NewGain > pOV2710Ctx->AecMaxGain ) NewGain = pOV2710Ctx->AecMaxGain;

	#if 0
	usGain = (uint16_t)(NewGain *16.0f + 0.5);
	tmp_gain_val = usGain;

	// min gain
	if (tmp_gain_val < 16)
		tmp_gain_val = 16;
	
	// max gain
	if (tmp_gain_val > 496)
		tmp_gain_val = 512;

	if (tmp_gain_val > 31)
	{
		gainlow |= 0x10;
		tmp_gain_val = tmp_gain_val >> 1;
	}

	if (tmp_gain_val > 31)
	{
		gainlow |= 0x20;
		tmp_gain_val = tmp_gain_val >> 1;
	}

	if (tmp_gain_val > 31)
	{
		gainlow |= 0x40;
		tmp_gain_val = tmp_gain_val >> 1;
	}

	if (tmp_gain_val > 31)
	{
		gainlow |= 0x80;
		tmp_gain_val = tmp_gain_val >> 1;
	}

	if (tmp_gain_val >= 16)
		gainlow_14b = ((tmp_gain_val - 16) & 0x0f);

	gainlow = gainlow | gainlow_14b;
	#else
	if(NewGain>=1.0 && NewGain<2.0)
	{
		gainlow = (uint16_t)(16.0*NewGain - 16.0 );
	}else if(NewGain>=2.0 && NewGain<4.0)
	{
		gainlow = (uint16_t)(8.0*NewGain);
	}else if(NewGain>=4.0 && NewGain<8.0)
	{
		gainlow = (uint16_t)(4.0*NewGain + 32.0 );
	}else if(NewGain>=8.0 && NewGain<16.0)
	{
		gainlow = (uint16_t)(2.0*NewGain + 92.0 );
	}else if(NewGain>=16.0 && NewGain<32.0)
	{
		gainlow = (uint16_t)(NewGain + 224.0 );
	}
	#endif

    result = OV2710_IsiRegWriteIss(pOV2710Ctx, OV2710_AEC_AGC_ADJ_H, 0);
    RETURN_RESULT_IF_DIFFERENT(RET_SUCCESS, result);
    result = OV2710_IsiRegWriteIss(pOV2710Ctx, OV2710_AEC_AGC_ADJ_L, (gainlow & 0xff));
    RETURN_RESULT_IF_DIFFERENT(RET_SUCCESS, result);

    //calculate gain actually set
    pOV2710Ctx->AecCurGain = (NewGain);

    //return current state
    *pSetGain = pOV2710Ctx->AecCurGain;
    TRACE( OV2710_DEBUG, "usGain:%d psetgain=%f, NewGain=%f,result_gain=%x\n", usGain, *pSetGain, NewGain, gainlow);
	
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2710_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV2710Ctx->AecCurIntegrationTime;
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV2710Ctx->AecIntegrationTimeIncrement;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  OV2710 sensor instance handle
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
 *
 *****************************************************************************/
RESULT OV2710_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
    uint32_t data= 0;
    uint32_t result_intertime= 0;

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( OV2710_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pOV2710Ctx->AecMinIntegrationTime,
        pOV2710Ctx->AecMaxIntegrationTime);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( OV2710_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pOV2710Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV2710Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pOV2710Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV2710Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by OV2710
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pOV2710Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pOV2710Ctx->LineLengthPck == 0 )
    {
        TRACE( OV2710_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV2710Ctx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pOV2710Ctx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV2710Ctx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pOV2710Ctx->OldCoarseIntegrationTime )
    {
       result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_AEC_EXPO_H, (CoarseIntegrationTime & 0x0000F000U) >> 12U );
       RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_AEC_EXPO_M, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
       RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_AEC_EXPO_L, (CoarseIntegrationTime & 0x0000000FU) << 4U );
       RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

       pOV2710Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
       *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //calculate integration time actually set
    pOV2710Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pOV2710Ctx->LineLengthPck) / pOV2710Ctx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pOV2710Ctx->AecCurIntegrationTime;

    TRACE( OV2710_DEBUG, "%s:\n"
         "pOV2710Ctx->VtPixClkFreq:%f pOV2710Ctx->LineLengthPck:%x \n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, 
         pOV2710Ctx->VtPixClkFreq,pOV2710Ctx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x0000F000U) >> 12U ,
         (CoarseIntegrationTime & 0x00000FF0U) >> 4U,
         (CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          OV2710_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV2710 sensor instance handle
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
 *
 *****************************************************************************/
RESULT OV2710_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV2710_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV2710_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = OV2710_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = OV2710_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( OV2710_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2710_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV2710Ctx->AecCurGain;
    *pSetIntegrationTime = pOV2710Ctx->AecCurIntegrationTime;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetResolutionIss
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
 *
 *****************************************************************************/
RESULT OV2710_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV2710Ctx->Config.Resolution;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV2710Ctx             OV2710 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetAfpsInfoHelperIss(
    OV2710_Context_t   *pOV2710Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV2710Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV2710Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV2710_SetupOutputWindowInternal( pOV2710Ctx, &pOV2710Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV2710_AecSetModeParameters( pOV2710Ctx, &pOV2710Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2710_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV2710Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV2710Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV2710Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV2710Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV2710Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2710_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV2710 sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
RESULT OV2710_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        TRACE( OV2710_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV2710Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pOV2710Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pOV2710Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pOV2710Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    OV2710_Context_t *pDummyCtx = (OV2710_Context_t*) malloc( sizeof(OV2710_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( OV2710_ERROR,  "%s: Can't allocate dummy OV2710 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pOV2710Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = OV2710_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes)
    {
        case SUPPORT_MIPI_ONE_LANE:
        {
	    switch (Resolution){
		case ISI_RES_TV1080P30:
		    AFPSCHECKANDADD(ISI_RES_TV1080P30);
		    break;
		}
            break;
        }

        default:
            TRACE( OV2710_ERROR,  "%s: pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibKFactor
 *
 * @brief   Returns the OV2710 specific K-Factor
 *
 * @param   handle       OV2710 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV2710_KFactor;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV2710 specific PCA-Matrix
 *
 * @param   handle          OV2710 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV2710_PCAMatrix;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV2710 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV2710_SVDMeanValue;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV2710 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&OV2710_CenterLine;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV2710 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&OV2710_AwbClipParm;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV2710 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV2710_AwbGlobalFadeParm;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV2710 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV2710_AwbFade2Parm;

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2710_IsiGetIlluProfile
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
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);
    RESULT result = RET_SUCCESS;

    if ( pOV2710Ctx == NULL )
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
        for ( i=0U; i<OV2710_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV2710_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV2710_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
#endif
    }

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetLscMatrixTable
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
 *
 *****************************************************************************/
static RESULT OV2710_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

	RESULT result = RET_SUCCESS;
	return (result);
#if 0
    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {

        uint16_t i;

        switch ( CieProfile )
        {
            case ISI_CIEPROF_A:
            {
                if ( ( pOV2710Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pOV2710Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV2710_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pOV2710Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pOV2710Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV2710_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pOV2710Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pOV2710Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV2710_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pOV2710Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pOV2710Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV2710_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pOV2710Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pOV2710Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV2710_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV2710_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV2710_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;

    }

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
#endif
}


/*****************************************************************************/
/**
 *          OV2710_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV2710 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    TRACE( OV2710_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
	if ((pOV2710Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 		vcm_movefull_t = 64* (1<<(pOV2710Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pOV2710Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
	}else{
 		vcm_movefull_t =64*1023/1000;
		TRACE( OV2710_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
	}
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = OV2710_IsiMdiFocusSet( handle, MAX_LOG );

    //TRACE( OV2710_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV2710 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 
 *****************************************************************************/
static RESULT OV2710_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( OV2710_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( OV2710_ERROR, "%s: pOV2710Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pOV2710Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pOV2710Ctx->VcmInfo.StartCurrent + (pOV2710Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( OV2710_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pOV2710Ctx->VcmInfo.StepMode );
	
    //TRACE( OV2710_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pOV2710Ctx->IsiCtx.HalHandle,
                             pOV2710Ctx->IsiCtx.I2cAfBusNum,
                             pOV2710Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pOV2710Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( OV2710_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV2710 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2710_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    TRACE( OV2710_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV2710Ctx->IsiCtx.HalHandle,
                            pOV2710Ctx->IsiCtx.I2cAfBusNum,
                            pOV2710Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pOV2710Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

	if( *pAbsStep <= pOV2710Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pOV2710Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pOV2710Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pOV2710Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pOV2710Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( OV2710_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV2710_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2710_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT OV2710_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;


    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);
    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV2710_IsiRegReadIss( pOV2710Ctx, OV2710_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV2710_IsiRegReadIss( pOV2710Ctx, OV2710_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV2710_IsiRegWriteIss( pOV2710Ctx, OV2710_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pOV2710Ctx->TestPattern = enable;
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          OV2710_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV2710 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT OV2710_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pOV2710Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pOV2710Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pOV2710Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( OV2710_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV2710_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
    	TRACE( OV2710_ERROR, "%s: pOV2710Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV2710_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV2710_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV2710_Context_t *pOV2710Ctx = (OV2710_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV2710_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2710Ctx == NULL )
    {
    	TRACE( OV2710_ERROR, "%s: pOV2710Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV2710_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV2710_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          OV2710_IsiGetSensorIss
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
RESULT OV2710_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2710_INFO, "%s (enter pIsiSensor:%x)\n", __FUNCTION__, pIsiSensor);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = OV2710_g_acName;
        pIsiSensor->pRegisterTable                      = OV2710_g_aRegDescription_onelane;
        pIsiSensor->pIsiSensorCaps                      = &OV2710_g_IsiSensorDefaultConfig;
	pIsiSensor->pIsiGetSensorIsiVer			= OV2710_IsiGetSensorIsiVersion;
	pIsiSensor->pIsiGetSensorTuningXmlVersion	= OV2710_IsiGetSensorTuningXmlVersion;
	//pIsiSensor->pIsiCheckOTPInfo                  = check_read_otp;
        pIsiSensor->pIsiCreateSensorIss                 = OV2710_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = OV2710_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = OV2710_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = OV2710_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = OV2710_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = OV2710_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = OV2710_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = OV2710_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = OV2710_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = OV2710_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = OV2710_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = OV2710_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = OV2710_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = OV2710_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = OV2710_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV2710_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV2710_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV2710_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV2710_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV2710_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV2710_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV2710_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV2710_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = OV2710_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = OV2710_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = OV2710_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = OV2710_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = OV2710_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = OV2710_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = OV2710_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = OV2710_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = OV2710_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = OV2710_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = OV2710_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = OV2710_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = OV2710_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = OV2710_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = OV2710_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = OV2710_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV2710_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT OV2710_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV2710_ERROR,  "%s: Can't allocate OV2710 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV2710_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = OV2710_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = OV2710_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = OV2710_SOFTWARE_RST_VALUE;
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
                while(OV2710_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = OV2710_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = OV2710_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV2710_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = OV2710_CHIP_ID_LOW_BYTE_DEFAULT;
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
    OV2710_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,						/**< IsiSensor_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiSensor_t.pIsiCheckOTPInfo>*/  //zyc 
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
    OV2710_IsiGetSensorI2cInfo,
};


