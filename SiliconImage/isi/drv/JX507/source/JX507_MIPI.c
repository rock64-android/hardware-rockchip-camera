

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
 * @file JX507.c
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

#include "JX507_MIPI_priv.h"

#define  JX507_NEWEST_TUNING_XML "18-7-2014_oyyf-hkw_JX507_CMK-CB0407-FV1_v0.1.1"

//hkw no use;
#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( JX507_INFO , "JX507: ", ERROR,    1U );
CREATE_TRACER( JX507_WARN , "JX507: ", ERROR, 1U );
CREATE_TRACER( JX507_ERROR, "JX507: ", ERROR,   1U );

CREATE_TRACER( JX507_DEBUG, "JX507: ", ERROR,     1U );

CREATE_TRACER( JX507_REG_INFO , "JX507: ", ERROR, 1);
CREATE_TRACER( JX507_REG_DEBUG, "JX507: ", INFO, 1U );

#define JX507_SLAVE_ADDR       0x60U                          /**< i2c slave address of the JX507 camera sensor */
#define JX507_SLAVE_ADDR2       0x60U

#define JX507_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the JX507 integrated AD5820 */

#define JX507_MAXN_GAIN 		(128.0f)
#define JX507_MIN_GAIN_STEP   ( 1.0f / JX507_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define JX507_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
const char JX507_g_acName[] = "JX507_MIPI";
//extern const IsiRegDescription_t OV8858_g_aRegDescription[];
extern const IsiRegDescription_t JX507_g_aRegDescription_twolane[];
extern const IsiRegDescription_t JX507_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t JX507_g_1296x972_twolane[];
extern const IsiRegDescription_t JX507_g_1296x972_fourlane[];
extern const IsiRegDescription_t JX507_g_1920x1080_twolane[];
extern const IsiRegDescription_t JX507_g_1920x1080_fourlane[];
extern const IsiRegDescription_t JX507_g_2592x1944_twolane[];
extern const IsiRegDescription_t JX507_g_2592x1944_fourlane[];

const IsiSensorCaps_t JX507_g_IsiSensorDefaultConfig;



#define JX507_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define JX507_I2C_NR_ADR_BYTES     (1U)                        // 1 byte base address and 2 bytes sub address
#define JX507_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_TWO_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT JX507_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT JX507_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT JX507_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT JX507_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT JX507_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT JX507_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT JX507_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT JX507_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT JX507_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT JX507_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT JX507_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT JX507_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT JX507_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT JX507_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT JX507_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT JX507_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT JX507_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT JX507_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT JX507_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT JX507_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT JX507_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT JX507_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT JX507_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT JX507_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT JX507_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT JX507_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT JX507_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT JX507_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT JX507_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT JX507_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT JX507_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT JX507_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT JX507_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT JX507_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT JX507_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT JX507_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT JX507_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT JX507_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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
 *          JX507_IsiCreateSensorIss
 *
 * @brief   This function creates a new JX507 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT JX507_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    JX507_Context_t *pJX507Ctx;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pJX507Ctx = ( JX507_Context_t * )malloc ( sizeof (JX507_Context_t) );
    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR,  "%s: Can't allocate JX507 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pJX507Ctx, 0, sizeof( JX507_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pJX507Ctx );
        return ( result );
    }else{
		TRACE( JX507_ERROR, "%s(%d): 8888888888888 HalAddRef 8888888888 \n",  __FUNCTION__,__LINE__);
	}

    pJX507Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
	TRACE( JX507_ERROR, "%s(%d): 8888888888888 HalAddRef 8888888888 pJX507Ctx->IsiCtx.HalHandl(0x%x) pConfig->HalHandle(0x%x)\n",  __FUNCTION__,__LINE__,pJX507Ctx->IsiCtx.HalHandle,pConfig->HalHandle);
    pJX507Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pJX507Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pJX507Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? JX507_SLAVE_ADDR : pConfig->SlaveAddr;
    pJX507Ctx->IsiCtx.NrOfAddressBytes       = 1U;

    pJX507Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pJX507Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? JX507_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pJX507Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pJX507Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pJX507Ctx->Configured             = BOOL_FALSE;
    pJX507Ctx->Streaming              = BOOL_FALSE;
    pJX507Ctx->TestPattern            = BOOL_FALSE;
    pJX507Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pJX507Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pJX507Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pJX507Ctx->VcmInfo.RatedCurrent   = pJX507Ctx->VcmInfo.StartCurrent + MAX_LOG*pJX507Ctx->VcmInfo.Step;
    pJX507Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pJX507Ctx->IsiSensorMipiInfo.sensorHalDevID = pJX507Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( JX507_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pJX507Ctx;

    result = HalSetCamConfig( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, true, false, false ); //zyl
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV8858 sensor instance.
 *
 * @param   handle      JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT JX507_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)JX507_IsiSensorSetStreamingIss( pJX507Ctx, BOOL_FALSE );
    (void)JX507_IsiSensorSetPowerIss( pJX507Ctx, BOOL_FALSE );

    (void)HalDelRef( pJX507Ctx->IsiCtx.HalHandle );
	TRACE( JX507_ERROR, "%s(%d): 777777777 HalDelRef 8888888888 handle(0x%x)\n",  __FUNCTION__,__LINE__, pJX507Ctx->IsiCtx.HalHandle);

    MEMSET( pJX507Ctx, 0, sizeof( JX507_Context_t ) );
    free ( pJX507Ctx );

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *         JX507_IsiGetCapsIss
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
static RESULT JX507_IsiGetCapsIssInternal
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
        if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE){            
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P15;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1296_972P30;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
                    break;
                }
	           // case 3:
               // {
               //     pIsiSensorCaps->Resolution = ISI_RES_SXGAP30;
               //     break;
               // }			
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }            
        } else if(mipi_lanes == SUPPORT_MIPI_TWO_LANE) {
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P15;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1296_972P30;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
                    break;
                }	
	           // case 3:
               // {
               //     pIsiSensorCaps->Resolution = ISI_RES_SXGAP30;
               //     break;
               // }				
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
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P15;
                    break;
                }	
                //case 3:
               // {
               //     pIsiSensorCaps->Resolution = ISI_RES_SXGAP30;
               //     break;
               // }				
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
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS; //hsync?
        pIsiSensorCaps->VPol            = ISI_VPOL_NEG; //VPolarity
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING; //?
        pIsiSensorCaps->Bls             = ISI_BLS_OFF; //close;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;//close;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;//close;
        pIsiSensorCaps->BLC             = ( ISI_BLC_OFF);
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
    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT JX507_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = JX507_IsiGetCapsIssInternal(pIsiSensorCaps,pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t JX507_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_NEG,               // VPol
    ISI_EDGE_FALLING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_2592_1944P15, //ISI_RES_TV1080P15,          // Res
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
    0
};



/*****************************************************************************/
/**
 *          JX507_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      JX507 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT JX507_SetupOutputFormat
(
    JX507_Context_t       *pJX507Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s%s (enter)\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV8858 sensor, so the YCSequence parameter is not checked */
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
            TRACE( JX507_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( JX507_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( JX507_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( JX507_INFO, "%s%s (exit)\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int JX507_get_PCLK( JX507_Context_t *pJX507Ctx, int XVCLK)
{
/*    // calculate PCLK
    uint32_t SCLK, temp1, temp2, temp3;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_sys_pre_div, Pll2_sys_divider2x, Sys_pre_div, sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_pre_div_map[] = {1, 2, 4, 1};

    
    //temp1 = ReadSCCB(0x6c, 0x3007);
    JX507_IsiRegReadIss(  pJX507Ctx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	JX507_IsiRegReadIss(  pJX507Ctx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	JX507_IsiRegReadIss(  pJX507Ctx, 0x030c, &temp1 );
	JX507_IsiRegReadIss(  pJX507Ctx, 0x030d, &temp3 );
	temp1 = temp1 & 0x03;
	temp2 = (temp1<<8) + temp3;
	if(!temp2) {
 		Pll2_multiplier = 1;
 	}
	else {
 	Pll2_multiplier = temp2;
	}
	
    JX507_IsiRegReadIss(  pJX507Ctx, 0x030f, &temp1 );
	temp1 = temp1 & 0x0f;
	Pll2_sys_pre_div = temp1 + 1;
	JX507_IsiRegReadIss(  pJX507Ctx, 0x030e, &temp1 );
	temp1 = temp1 & 0x07;
	Pll2_sys_divider2x = Pll2_sys_divider2x_map[temp1];

	JX507_IsiRegReadIss(  pJX507Ctx, 0x3106, &temp1 );
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
	JX507_IsiRegWriteIss(pJX507Ctx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	JX507_IsiRegWriteIss(pJX507Ctx, 0x350a, temp1);
	return SCLK*10000;
	*/
	return 912*10000;
}

/*****************************************************************************/
/**
 *          JX507_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      JX507 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT JX507_SetupOutputWindow
(
    JX507_Context_t        *pJX507Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
	uint16_t usTimeHts;
	uint16_t usTimeVts;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 2400;
	TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

if(pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

    pJX507Ctx->IsiSensorMipiInfo.ulMipiFreq = 720;
	switch ( pConfig->Resolution )
    {
        case ISI_RES_1296_972P30:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 1296x972\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_1296x972_twolane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0788; //hkw
            usTimeVts = 0x062a;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        } 
		#if 0
        case ISI_RES_SXGAP30:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 1280x960\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_1296x972_twolane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0788; //hkw
            usTimeVts = 0x04dc;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }	
		#endif
        case ISI_RES_TV1080P30:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 1920x1080\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_1920x1080_twolane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0908; //hkw
            usTimeVts = 0x0524;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }
        
        case ISI_RES_2592_1944P15:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 2592x1944\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_2592x1944_twolane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0ba8;
            usTimeVts = 0x07f6;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
	}
}
else if(pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
    	pJX507Ctx->IsiSensorMipiInfo.ulMipiFreq = 720;
		switch ( pConfig->Resolution )
		{
        case ISI_RES_1296_972P30:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 1296x972\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_1296x972_fourlane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0788; //hkw
            usTimeVts = 0x062a;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }
        case ISI_RES_TV1080P30:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 1920x1080\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_1920x1080_fourlane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0908; //hkw
            usTimeVts = 0x0524;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }        
        case ISI_RES_2592_1944P15:
        {
            TRACE( JX507_ERROR, "%s(%d): Resolution 2592x1944\n", __FUNCTION__,__LINE__ );
			result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_2592x1944_fourlane);
		    if ( result != RET_SUCCESS )
		    {
		        return ( result );
		    }
			usTimeHts = 0x0ba8;
            usTimeVts = 0x07f6;
		    /* sleep a while, that sensor can take over new default values */
		    osSleep( 10 );
			break;
            
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
	}

}
    
	
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	rVtPixClkFreq = JX507_get_PCLK(pJX507Ctx, xclk);
    
    // store frame timing for later use in AEC module
    pJX507Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pJX507Ctx->LineLengthPck    = usLineLengthPck;
    pJX507Ctx->FrameLengthLines = usFrameLengthLines;



    TRACE( JX507_ERROR, "%s  resolution(0x%x) freq(%f)(exit)\n", __FUNCTION__, pConfig->Resolution,rVtPixClkFreq);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      JX507 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT JX507_SetupImageControl
(
    JX507_Context_t        *pJX507Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = JX507_IsiRegReadIss(  pJX507Ctx, JX507_BLC_CTRL00, &RegValue );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = JX507_IsiRegReadIss(  pJX507Ctx, JX507_BLC_CTRL00, &RegValue );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = JX507_IsiRegReadIss(  pJX507Ctx, JX507_AEC_MANUAL, &RegValue );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = JX507_IsiRegReadIss(  pJX507Ctx, JX507_ISP_CTRL01, &RegValue );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = JX507_IsiRegReadIss(  pJX507Ctx, JX507_AEC_MANUAL, &RegValue );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = JX507_IsiRegReadIss( pJX507Ctx, JX507_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = JX507_IsiRegReadIss( pJX507Ctx, JX507_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( JX507_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in JX507-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      JX507 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT JX507_AecSetModeParameters
(
    JX507_Context_t       *pJX507Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s%s (enter)\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"");

    if ( (pJX507Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( JX507_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pJX507Ctx->AecMaxIntegrationTime = ( ((float)(pJX507Ctx->FrameLengthLines - 4)) * ((float)pJX507Ctx->LineLengthPck) ) / pJX507Ctx->VtPixClkFreq;
    pJX507Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( JX507_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"", pJX507Ctx->AecMaxIntegrationTime  );

    pJX507Ctx->AecMaxGain = JX507_MAX_GAIN_AEC;
    pJX507Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pJX507Ctx->AecIntegrationTimeIncrement = ((float)pJX507Ctx->LineLengthPck) / pJX507Ctx->VtPixClkFreq;
    pJX507Ctx->AecGainIncrement = JX507_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pJX507Ctx->AecCurGain               = pJX507Ctx->AecMinGain;
    pJX507Ctx->AecCurIntegrationTime    = 0.0f;
    pJX507Ctx->OldCoarseIntegrationTime = 0;
    pJX507Ctx->OldFineIntegrationTime   = 0;
    //pJX507Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( JX507_INFO, "%s%s (exit)\n", __FUNCTION__, pJX507Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      JX507 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT JX507_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pJX507Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pJX507Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
	uint32_t temp;
	JX507_IsiRegReadIss( pJX507Ctx, JX507_SOFTWARE_RST, &temp );
    result = JX507_IsiRegWriteIss ( pJX507Ctx, JX507_SOFTWARE_RST, temp|JX507_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( JX507_DEBUG, "%s: JX507 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    JX507_IsiRegReadIss( pJX507Ctx, JX507_MODE_SELECT, &temp );
    result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_MODE_SELECT, temp|JX507_MODE_SELECT_OFF );//JX507_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: Can't write JX507 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_aRegDescription );
    if(pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
        result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_aRegDescription_fourlane);
        }
	else if(pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE)
        result = IsiRegDefaultsApply( pJX507Ctx, JX507_g_aRegDescription_twolane);
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pJX507Ctx, JX507_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = JX507_IsiRegWriteIss( pJX507Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = JX507_SetupOutputFormat( pJX507Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = JX507_SetupOutputWindow( pJX507Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = JX507_SetupImageControl( pJX507Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = JX507_AecSetModeParameters( pJX507Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pJX507Ctx->Configured = BOOL_TRUE;
    }

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiChangeSensorResolutionIss
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
static RESULT JX507_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pJX507Ctx->Configured != BOOL_TRUE) || (pJX507Ctx->Streaming != BOOL_FALSE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    #if 0
    result = JX507_IsiGetCapsIss( handle, &Caps);
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
    while (JX507_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }
    #endif
    if ( Resolution == pJX507Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( JX507_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pJX507Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = JX507_SetupOutputWindow( pJX507Ctx, &pJX507Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( JX507_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pJX507Ctx->AecCurGain;
        float OldIntegrationTime = pJX507Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = JX507_AecSetModeParameters( pJX507Ctx, &pJX507Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( JX507_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = JX507_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( JX507_ERROR, "%s: JX507_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
    }

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiSensorSetStreamingIss
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
static RESULT JX507_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pJX507Ctx->Configured != BOOL_TRUE) || (pJX507Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = JX507_IsiRegWriteIss ( pJX507Ctx, JX507_MODE_SELECT, (RegValue & ~JX507_MODE_SELECT_ON) );//JX507_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = JX507_IsiRegWriteIss ( pJX507Ctx, JX507_MODE_SELECT, (RegValue | JX507_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        TRACE(JX507_INFO," STREAM OFF ++++++++++++++");
    }

    if (result == RET_SUCCESS)
    {
        pJX507Ctx->Streaming = on;
    }
osSleep(50);
    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      JX507 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT JX507_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pJX507Ctx->Configured = BOOL_FALSE;
    pJX507Ctx->Streaming  = BOOL_FALSE;

    TRACE( JX507_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( JX507_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( JX507_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( JX507_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( JX507_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( JX507_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pJX507Ctx->IsiCtx.HalHandle, pJX507Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT JX507_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = JX507_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (JX507_CHIP_ID_LOW_BYTE_DEFAULT);

    result = JX507_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( JX507_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( JX507_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetSensorRevisionIss
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
static RESULT JX507_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = JX507_IsiRegReadIss ( handle, JX507_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = JX507_IsiRegReadIss ( handle, JX507_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiRegReadIss
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
static RESULT JX507_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, JX507_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
 //       TRACE( JX507_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x)\n", __FUNCTION__, NrOfBytes, address);

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( JX507_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiRegWriteIss
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
static RESULT JX507_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, JX507_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
//    TRACE( JX507_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x 0x%08x)\n", __FUNCTION__, NrOfBytes, address, value);

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( JX507_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          JX507 instance
 *
 * @param   handle       JX507 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT JX507_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( JX507_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pJX507Ctx->AecMinGain;
    *pMaxGain = pJX507Ctx->AecMaxGain;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          JX507 instance
 *
 * @param   handle       JX507 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT JX507_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( JX507_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pJX507Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pJX507Ctx->AecMaxIntegrationTime;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  JX507 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT JX507_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

	
	result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_AGC_ADJ_H, &data);
	TRACE( JX507_INFO, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_AGC_ADJ_L, &data);
	TRACE( JX507_INFO, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / JX507_MAXN_GAIN;
	
    //*pSetGain = pJX507Ctx->AecCurGain;
    

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  JX507 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT JX507_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pJX507Ctx->AecGainIncrement;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  JX507 sensor instance handle
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
RESULT JX507_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
	uint32_t data= 0;
	uint32_t result_gain= 0;

    TRACE( JX507_INFO, "%s: (enter) pJX507Ctx->AecMaxGain(%f) \n", __FUNCTION__,pJX507Ctx->AecMaxGain);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( JX507_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pJX507Ctx->AecMinGain ) NewGain = pJX507Ctx->AecMinGain;
    if( NewGain > pJX507Ctx->AecMaxGain ) NewGain = pJX507Ctx->AecMaxGain;

    usGain = (uint16_t)(NewGain * JX507_MAXN_GAIN+0.5); //大概加0.5 hkw

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pJX507Ctx->OldGain) )
    {
        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_AGC_ADJ_H, (usGain>>8)&0x07); //fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_AGC_ADJ_L, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pJX507Ctx->OldGain = usGain;

		/*osSleep(30);
		result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_AGC_ADJ_H, &data);
		TRACE( JX507_ERROR, " -------reg35088888888:%x-------\n",data );
		result_gain = (data & 0x07) ;
		result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_AGC_ADJ_L, &data);
		TRACE( JX507_ERROR, " -------reg35099999999:%x-------\n",data );
		result_gain = (result_gain<<8) + data;*/
		
    }

    //calculate gain actually set
    pJX507Ctx->AecCurGain = ( (float)usGain ) / JX507_MAXN_GAIN;

    //return current state
    *pSetGain = pJX507Ctx->AecCurGain;
    TRACE( JX507_INFO, "-----------%s: psetgain=%f, NewGain=%f,result_gain=%x\n", __FUNCTION__, *pSetGain, NewGain,result_gain);

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  JX507 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT JX507_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pJX507Ctx->AecCurIntegrationTime;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  JX507 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT JX507_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pJX507Ctx->AecIntegrationTimeIncrement;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  JX507 sensor instance handle
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
RESULT JX507_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;
	uint32_t result_intertime= 0;
	
    //uint32_t FineIntegrationTime   = 0; //not supported by JX507

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( JX507_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pJX507Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pJX507Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pJX507Ctx->AecMinIntegrationTime ) NewIntegrationTime = pJX507Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by JX507
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pJX507Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pJX507Ctx->LineLengthPck == 0 )
    {
        TRACE( JX507_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pJX507Ctx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pJX507Ctx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pJX507Ctx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pJX507Ctx->OldCoarseIntegrationTime )
    {//
        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_EXPO_H, (CoarseIntegrationTime & 0x0000F000U) >> 12U );//fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_EXPO_M, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_AEC_EXPO_L, (CoarseIntegrationTime & 0x0000000FU) << 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pJX507Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
        
		/*osSleep(30);
		result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_EXPO_H, &data);
		TRACE( JX507_ERROR, " -------reg3500:%x-------\n",data );
		result_intertime = (data & 0x0f) << 8;
		result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_EXPO_M, &data);
		TRACE( JX507_ERROR, " -------reg3501:%x-------\n",data );
		result_intertime = result_intertime + data;
		result = JX507_IsiRegReadIss ( pJX507Ctx, JX507_AEC_EXPO_L, &data);
		TRACE( JX507_ERROR, " -------reg3502:%x-------\n",data );
		result_intertime = (result_intertime << 4) + (data >> 4);*/
		
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //if( FineIntegrationTime != pJX507Ctx->OldFineIntegrationTime )
    //{
    //    result = JX507_IsiRegWriteIss( pJX507Ctx, ... , FineIntegrationTime );
    //    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    //    pJX507Ctx->OldFineIntegrationTime = FineIntegrationTime; //remember current integration time
    //    *pNumberOfFramesToSkip = 1U; //skip 1 frame
    //}

    //calculate integration time actually set
    //pJX507Ctx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pJX507Ctx->LineLengthPck) + ((float)FineIntegrationTime) ) / pJX507Ctx->VtPixClkFreq;
    pJX507Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pJX507Ctx->LineLengthPck) / pJX507Ctx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pJX507Ctx->AecCurIntegrationTime;

    TRACE( JX507_INFO, "---------%s:pJX507Ctx->VtPixClkFreq:%f;pJX507Ctx->LineLengthPck:%x\n-------SetTi=%f NewTi=%f  CoarseIntegrationTime=%x,result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, pJX507Ctx->VtPixClkFreq,pJX507Ctx->LineLengthPck,*pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,result_intertime,(CoarseIntegrationTime & 0x0000F000U) >> 12U ,(CoarseIntegrationTime & 0x00000FF0U) >> 4U,(CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          JX507_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  JX507 sensor instance handle
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
RESULT JX507_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( JX507_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( JX507_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = JX507_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = JX507_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( JX507_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT JX507_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pJX507Ctx->AecCurGain;
    *pSetIntegrationTime = pJX507Ctx->AecCurIntegrationTime;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetResolutionIss
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
RESULT JX507_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pJX507Ctx->Config.Resolution;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pJX507Ctx             JX507 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT JX507_IsiGetAfpsInfoHelperIss(
    JX507_Context_t   *pJX507Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pJX507Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pJX507Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = JX507_SetupOutputWindow( pJX507Ctx, &pJX507Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = JX507_AecSetModeParameters( pJX507Ctx, &pJX507Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( JX507_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pJX507Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pJX507Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pJX507Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pJX507Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pJX507Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          JX507_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  JX507 sensor instance handle
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
RESULT JX507_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        TRACE( JX507_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pJX507Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pJX507Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pJX507Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pJX507Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    JX507_Context_t *pDummyCtx = (JX507_Context_t*) malloc( sizeof(JX507_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( JX507_ERROR,  "%s: Can't allocate dummy JX507 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pJX507Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    if ( (pJX507Ctx->Config.AfpsResolutions & (_res_)) != 0 ) \
    { \
        RESULT lres = JX507_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
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
            TRACE( JX507_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
            result = RET_NOTSUPP;
            break;

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

        // check next series here...
    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibKFactor
 *
 * @brief   Returns the JX507 specific K-Factor
 *
 * @param   handle       JX507 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT JX507_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&JX507_KFactor;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          JX507_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the JX507 specific PCA-Matrix
 *
 * @param   handle          JX507 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT JX507_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&JX507_PCAMatrix;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              JX507 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT JX507_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&JX507_SVDMeanValue;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              JX507 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT JX507_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&JX507_CenterLine;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              JX507 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT JX507_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&JX507_AwbClipParm;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              JX507 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT JX507_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&JX507_AwbGlobalFadeParm;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              JX507 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT JX507_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&JX507_AwbFade2Parm;

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          JX507_IsiGetIlluProfile
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
static RESULT JX507_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint16_t i;

        *ptIsiIlluProfile = NULL;

        /* check if we've a default profile */
        for ( i=0U; i<JX507_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( JX507_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &JX507_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          JX507_IsiGetLscMatrixTable
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
static RESULT JX507_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
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
                if ( ( pJX507Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pJX507Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( JX507_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pJX507Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pJX507Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( JX507_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pJX507Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pJX507Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( JX507_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pJX507Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pJX507Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( JX507_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pJX507Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pJX507Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &JX507_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( JX507_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( JX507_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          JX507_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT JX507_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( JX507_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          JX507 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT JX507_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    //TRACE( JX507_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
 if ((pJX507Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pJX507Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pJX507Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( JX507_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = JX507_IsiMdiFocusSet( handle, MAX_LOG );

    //TRACE( JX507_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          JX507 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT JX507_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( JX507_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( JX507_ERROR, "%s: pJX507Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pJX507Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pJX507Ctx->VcmInfo.StartCurrent + (pJX507Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( JX507_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pJX507Ctx->VcmInfo.StepMode );
	
    //TRACE( JX507_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pJX507Ctx->IsiCtx.HalHandle,
                             pJX507Ctx->IsiCtx.I2cAfBusNum,
                             pJX507Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pJX507Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( JX507_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          JX507 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT JX507_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( JX507_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pJX507Ctx->IsiCtx.HalHandle,
                            pJX507Ctx->IsiCtx.I2cAfBusNum,
                            pJX507Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pJX507Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( JX507_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
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
	if( *pAbsStep <= pJX507Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pJX507Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pJX507Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pJX507Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pJX507Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( JX507_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT JX507_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          JX507_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT JX507_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = JX507_IsiRegReadIss( pJX507Ctx, JX507_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = JX507_IsiRegReadIss( pJX507Ctx, JX507_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = JX507_IsiRegWriteIss( pJX507Ctx, JX507_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pJX507Ctx->TestPattern = enable;
    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          JX507_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          JX507 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT JX507_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pJX507Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pJX507Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pJX507Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( JX507_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT JX507_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
    	TRACE( JX507_ERROR, "%s: pJX507Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( JX507_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT JX507_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    JX507_Context_t *pJX507Ctx = (JX507_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( JX507_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pJX507Ctx == NULL )
    {
    	TRACE( JX507_ERROR, "%s: pJX507Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( JX507_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = JX507_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          JX507_IsiGetSensorIss
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
RESULT JX507_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( JX507_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = JX507_g_acName;
        pIsiSensor->pRegisterTable                      = JX507_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &JX507_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= JX507_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= JX507_IsiGetSensorTuningXmlVersion;//oyyf
        pIsiSensor->pIsiCreateSensorIss                 = JX507_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = JX507_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = JX507_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = JX507_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = JX507_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = JX507_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = JX507_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = JX507_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = JX507_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = JX507_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = JX507_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = JX507_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = JX507_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = JX507_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = JX507_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = JX507_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = JX507_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = JX507_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = JX507_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = JX507_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = JX507_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = JX507_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = JX507_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = JX507_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = JX507_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = JX507_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = JX507_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = JX507_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = JX507_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = JX507_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = JX507_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = JX507_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = JX507_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = JX507_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = JX507_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = JX507_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = JX507_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = JX507_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = JX507_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( JX507_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT JX507_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( JX507_ERROR,  "%s: Can't allocate JX507 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = JX507_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = JX507_SLAVE_ADDR2;	
    pSensorI2cInfo->soft_reg_addr = JX507_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = JX507_SOFTWARE_RST_VALUE;
    pSensorI2cInfo->reg_size = 1;
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
                while(JX507_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = JX507_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = JX507_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
#if 0
    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = JX507_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = JX507_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
#endif    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = JX507_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = JX507_CHIP_ID_LOW_BYTE_DEFAULT;
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
    JX507_IsiGetSensorIss,
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
        0,
    },
    JX507_IsiGetSensorI2cInfo,
};


