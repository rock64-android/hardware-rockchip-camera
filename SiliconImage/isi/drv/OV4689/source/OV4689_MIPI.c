//OV4689 the same with ov14825

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
 * @file OV4689.c
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

#include "OV4689_MIPI_priv.h"

 #define OV4689_AF_ENABLE
 #define OV4689_CALIB_ENABLE

#define  OV4689_NEWEST_TUNING_XML "20-Apr-2016_YANGHUA_OV4689_v1.0"


#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U


/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV4689_INFO , "OV4689: ", INFO,    0U );
CREATE_TRACER( OV4689_WARN , "OV4689: ", WARNING, 1U );
CREATE_TRACER( OV4689_ERROR, "OV4689: ", ERROR,   1U );

CREATE_TRACER( OV4689_DEBUG, "OV4689: ", INFO,     0U );

CREATE_TRACER( OV4689_REG_INFO , "OV4689: ", INFO, 0);
CREATE_TRACER( OV4689_REG_DEBUG, "OV4689: ", INFO, 0U );


#define OV4689_SLAVE_ADDR       0x20U                           /**< i2c slave address of the OV4689 camera sensor */ //* if SID is high, the sensor's I2C address comes from reg.0X3012 which has a default value of 0x20 */
#define OV4689_SLAVE_ADDR2      0x6cU					  // if SID is low, the sensor's I2C address comes from reg.0X3004 which has a default value of 0x6C 
#ifdef OV4689_AF_ENABLE
#define OV4689_SLAVE_AF_ADDR    0x18U                           /**< i2c slave address of the OV4689 integrated AD5820 */
#endif

#define OV4689_MIN_GAIN_STEP   ( 1.0f / 128.0f); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define OV4689_MAX_GAIN_AEC    ( 16.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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

#define COLOUR 			0
#define WHITE_BLACK 	1


/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV4689_g_acName[] = "OV4689_MIPI";

extern const IsiRegDescription_t OV4689_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t OV4689_g_fourlane_resolution_2688_1520[];
extern const IsiRegDescription_t OV4689_g_fourlane_resolution_1280_720[];


extern const IsiRegDescription_t OV4689_g_aRegDescription_twolane[];

extern const IsiRegDescription_t OV4689_g_twolane_resolution_2688_1520[];
extern const IsiRegDescription_t OV4689_g_2688x1520P30_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_2688x1520P25_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_2688x1520P20_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_2688x1520P15_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_2688x1520P10_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_2688x1520P7_twolane_fpschg[];

extern const IsiRegDescription_t OV4689_g_twolane_resolution_1920_1080[];
extern const IsiRegDescription_t OV4689_g_1920x1080P30_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1920x1080P25_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1920x1080P20_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1920x1080P15_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1920x1080P10_twolane_fpschg[];

extern const IsiRegDescription_t OV4689_g_twolane_resolution_1280_720[];
extern const IsiRegDescription_t OV4689_g_1280x720P30_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1280x720P25_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1280x720P20_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1280x720P15_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_1280x720P10_twolane_fpschg[];

extern const IsiRegDescription_t OV4689_g_twolane_resolution_672_376[];
extern const IsiRegDescription_t OV4689_g_672x376P60_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_672x376P30_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_672x376P25_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_672x376P20_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_672x376P15_twolane_fpschg[];
extern const IsiRegDescription_t OV4689_g_672x376P10_twolane_fpschg[];


const IsiSensorCaps_t OV4689_g_IsiSensorDefaultConfig;


#define OV4689_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define OV4689_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define OV4689_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE


/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV4689_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV4689_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV4689_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV4689_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV4689_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV4689_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV4689_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV4689_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV4689_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV4689_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV4689_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV4689_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV4689_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT OV4689_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV4689_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV4689_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV4689_IsiGetColorIss( IsiSensorHandle_t handle, char *pGetColor);
static RESULT OV4689_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV4689_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV4689_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV4689_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT OV4689_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV4689_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

#ifdef OV4689_CALIB_ENABLE
static RESULT OV4689_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV4689_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV4689_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV4689_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV4689_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV4689_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV4689_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV4689_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );
#endif

#ifdef OV4689_AF_ENABLE
static RESULT OV4689_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV4689_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV4689_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV4689_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV4689_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );
#endif

static RESULT OV4689_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT OV4689_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);

static RESULT OV4689_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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
 *          OV4689_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV4689 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV4689_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    OV4689_Context_t *pOV4689Ctx;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV4689Ctx = ( OV4689_Context_t * )malloc ( sizeof (OV4689_Context_t) );
    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV4689Ctx, 0, sizeof( OV4689_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV4689Ctx );
        return ( result );
    }

    pOV4689Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pOV4689Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pOV4689Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pOV4689Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? OV4689_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV4689Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pOV4689Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
#ifdef OV4689_AF_ENABLE
    pOV4689Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? OV4689_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
#else
    pOV4689Ctx->IsiCtx.SlaveAfAddress         = pConfig->SlaveAfAddr;
#endif
    pOV4689Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pOV4689Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pOV4689Ctx->Configured             = BOOL_FALSE;
    pOV4689Ctx->Streaming              = BOOL_FALSE;
    pOV4689Ctx->TestPattern            = BOOL_FALSE;
    pOV4689Ctx->isAfpsRun              = BOOL_FALSE;
	
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV4689Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pOV4689Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV4689Ctx->VcmInfo.RatedCurrent   = pOV4689Ctx->VcmInfo.StartCurrent + MAX_LOG*pOV4689Ctx->VcmInfo.Step;
    pOV4689Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;  

    pOV4689Ctx->IsiSensorMipiInfo.sensorHalDevID = pOV4689Ctx->IsiCtx.HalDevID;
    if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }

    pConfig->hSensor = ( IsiSensorHandle_t )pOV4689Ctx;

    result = HalSetCamConfig( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, false, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV4689 sensor instance.
 *
 * @param   handle      OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV4689_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV4689_IsiSensorSetStreamingIss( pOV4689Ctx, BOOL_FALSE );
    (void)OV4689_IsiSensorSetPowerIss( pOV4689Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV4689Ctx->IsiCtx.HalHandle );

    MEMSET( pOV4689Ctx, 0, sizeof( OV4689_Context_t ) );
    free ( pOV4689Ctx );

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCapsIss
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
static RESULT OV4689_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t mipi_lanes
)
{

    RESULT result = RET_SUCCESS;

    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
		if(mipi_lanes == SUPPORT_MIPI_ONE_LANE){
			result = RET_OUTOFRANGE;
		} 
		else if(mipi_lanes == SUPPORT_MIPI_TWO_LANE){
	            switch (pIsiSensorCaps->Index) 
			{
#if 0
					case 0:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_2688_1520P30;
	                    break;
				}
					case 1:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_2688_1520P25;
	                    break;
				}
	                case 2:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_2688_1520P20;
	                    break;
				}
	            	case 3:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_2688_1520P15;
	                    break;
				}
	                case 4:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_2688_1520P10;
	                    break;
				}
	        

				#if 1
	                case 5:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
	                    break;
				}
	                case 6:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P25;
	                    break;
				}
	                case 7:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P20;
	                    break;
				}
	                case 8:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P15;
	                    break;
				}
				    case 9:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_TV1080P10;
	                    break;
				}
	                case 10:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P60;
	                    break;
				}
	                case 11:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P30;
	                    break;
				}
	                case 12:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P25;
	                    break;
				}
	                case 13:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P20;
	                    break;
				}
				    case 14:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P15;
	                    break;
				}
	                case 15:
				{
	                    pIsiSensorCaps->Resolution = ISI_RES_672_376P10;
	                    break;
				}
				#endif
#else
				case 0:
				{
					 pIsiSensorCaps->Resolution = ISI_RES_2688_1520P30;
					 break;
				}
				#if 0
				case 1:
				{
					pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
					break;
				}
				#endif

#endif
	                default:
				{
	                    result = RET_OUTOFRANGE;
	                    goto end;
	                	}
			}            
		}		 
		else if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE){
	            switch (pIsiSensorCaps->Index) 
	            {
	                case 0:
	                {
	                    pIsiSensorCaps->Resolution = ISI_RES_TV720P30;
	                    break;
	                }
	                default:
	                {
	                    result = RET_OUTOFRANGE;
	                    goto end;
	                }
	            }            
		}
	
        TRACE( OV4689_INFO, "res:%d    %s (exit)\n", pIsiSensorCaps->Index, __FUNCTION__);
    
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT;
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
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_OFF );

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

    return ( result );
}

static RESULT OV4689_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    result = OV4689_IsiGetCapsIssInternal(pIsiSensorCaps, pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes);
    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV4689_g_IsiSensorDefaultConfig =
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
    ISI_RES_TV1080P30,          // Res	//////////////////////////	
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
 *          OV4689_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV4689 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_SetupOutputFormat
(
    OV4689_Context_t       *pOV4689Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s%s (enter)\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV4689 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV4689_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV4689_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( OV4689_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV4689_INFO, "%s%s (exit)\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

int OV4689_get_PCLK( OV4689_Context_t *pOV4689Ctx, int XVCLK)
{
	 // calculate PCLK
	 uint32_t temp1, temp2;
	 int Pll2_predivp, Pll2_prediv2x, Pll2_mult, Pll2_divsp, Pll2_divs2x;
	 long SCLK;
	 int Pll2_predivp_map[] = {1, 2};
	 int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};	// pll2_prediv *2
	 int Pll2_divsp_map[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	 int Pll2_divs2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};		// pll2_divs * 2
	 
	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x0311, &temp1 );
	 temp2 = temp1 & 0x01;
	 Pll2_predivp = Pll2_predivp_map[temp2];
	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x030B, &temp1 );
	 temp2 = temp1 & 0x07;
	 Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x30C, &temp1 );
	 temp2 = temp1 & 0x03;
	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x030D, &temp1 );
	 Pll2_mult = (temp2<<8) + temp1;
	 
	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x030F, &temp1 );
	 temp2 = temp1 & 0x0f;
	 Pll2_divsp = Pll2_divsp_map[temp2];
	 
	 OV4689_IsiRegReadIss(  pOV4689Ctx, 0x030E, &temp1 );
	 temp2 = (temp1) & 0x07;
	 Pll2_divs2x = Pll2_divs2x_map[temp2];
	 
	// SCLK = XVCLK /Pll2_predivp * 2 / Pll2_prediv2x * Pll2_mult / Pll2_divsp * 2 /Pll2_divs2x * 4;  // *4 ?
	 SCLK = XVCLK /Pll2_predivp * 2 / Pll2_prediv2x * Pll2_mult / Pll2_divsp * 2 /Pll2_divs2x;  // *4 ?  oyyf
	 
	 return SCLK;
 }

/*****************************************************************************/
/**
 *          OV4689_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV4689 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_SetupOutputWindowInternal
(
    OV4689_Context_t        *pOV4689Ctx,
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
    int xclk = 24000000;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

	if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	    /* resolution */
	    switch ( pConfig->Resolution )
	    {
	        default:
	        {
	            TRACE( OV4689_ERROR, "%s: one lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }		
	}
	else if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
	    switch ( pConfig->Resolution )
	    {
	     case ISI_RES_2688_1520P30:
		 case ISI_RES_2688_1520P15:
		 case ISI_RES_2688_1520P10:
		 case ISI_RES_2688_1520P7:
	        {
			if (set2Sensor == BOOL_TRUE) {                    
	                    if (res_no_chg == BOOL_FALSE) {
	                        result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV4689Ctx, OV4689_g_twolane_resolution_2688_1520);
	                    }
	                    if (pConfig->Resolution == ISI_RES_2688_1520P30) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P30_twolane_fpschg);
	                    } else if (pConfig->Resolution == ISI_RES_2688_1520P25) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P25_twolane_fpschg);
	                    } else if (pConfig->Resolution == ISI_RES_2688_1520P20) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P20_twolane_fpschg);
	                    } else if (pConfig->Resolution == ISI_RES_2688_1520P15) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P15_twolane_fpschg);
	                    } else  if (pConfig->Resolution == ISI_RES_2688_1520P10) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P10_twolane_fpschg);
	                    } else  if (pConfig->Resolution == ISI_RES_2688_1520P7) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_2688x1520P7_twolane_fpschg);
	                    } 
			      TRACE( OV4689_INFO, "%s  (setting flag): Resolution %dx%d@%dfps  \n", __FUNCTION__, ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution), ISI_FPS_GET(pConfig->Resolution));
			}
			
                     usTimeHts= 0x0a0a;
				   if (pConfig->Resolution == ISI_RES_2688_1520P30) {
	                    usTimeVts = 0x614;
	               }else if (pConfig->Resolution == ISI_RES_2688_1520P25) {
	                    usTimeVts = 0x74a;
	               }else if (pConfig->Resolution == ISI_RES_2688_1520P20) {
	                    usTimeVts = 0x91c;
	               }else if (pConfig->Resolution == ISI_RES_2688_1520P15) {
	                    usTimeVts = 0xc2c;
	               } else if (pConfig->Resolution == ISI_RES_2688_1520P10) {
	                    usTimeVts = 0x1234;
	               } else if (pConfig->Resolution == ISI_RES_2688_1520P7) {
	                    usTimeVts = 0x19ff;
	               }
                
			/* sleep a while, that sensor can take over new default values */
    		        osSleep( 10 );

		        pOV4689Ctx->IsiSensorMipiInfo.ulMipiFreq = 672;

			 break;	            
	        }
		 case ISI_RES_TV1080P30:
		 case ISI_RES_TV1080P25:
		 case ISI_RES_TV1080P15:
		 case ISI_RES_TV1080P10:
	        {
			if (set2Sensor == BOOL_TRUE) {                    
	                    if (res_no_chg == BOOL_FALSE) {
	                        result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV4689Ctx, OV4689_g_twolane_resolution_1920_1080);
	                    }
	                    if (pConfig->Resolution == ISI_RES_TV1080P30) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_1920x1080P30_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_TV1080P25) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_1920x1080P25_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_TV1080P20) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_1920x1080P20_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_TV1080P15) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_1920x1080P15_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_TV1080P10) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_1920x1080P10_twolane_fpschg);
	                    } 
			      TRACE( OV4689_INFO, "%s  (setting flag): Resolution %dx%d@%dfps  \n", __FUNCTION__, ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution), ISI_FPS_GET(pConfig->Resolution));
			}
			
                     //usTimeHts = 0x0a0a;
				   usTimeHts = 0x0a18;
				   if (pConfig->Resolution == ISI_RES_TV1080P30) {
	                    usTimeVts = 0x0614;
	               } else if (pConfig->Resolution == ISI_RES_TV1080P25) {
	                    usTimeVts = 0x74a;
	               } else if (pConfig->Resolution == ISI_RES_TV1080P20) {
	                    usTimeVts = 0x91c;
	               } else if (pConfig->Resolution == ISI_RES_TV1080P15) {
	                    usTimeVts = 0xc2c;
	               } else if (pConfig->Resolution == ISI_RES_TV1080P10) {
	                    usTimeVts = 0x1234;
	               }
                
			/* sleep a while, that sensor can take over new default values */
    		        osSleep( 10 );

		        pOV4689Ctx->IsiSensorMipiInfo.ulMipiFreq = 672;

			 break;	            
	        }
		 case ISI_RES_672_376P60:
		 case ISI_RES_672_376P30:
		 case ISI_RES_672_376P25:
		 case ISI_RES_672_376P15:
	        {
			if (set2Sensor == BOOL_TRUE) {                    
	                    if (res_no_chg == BOOL_FALSE) {
	                        result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV4689Ctx, OV4689_g_twolane_resolution_672_376);
	                    }
	                    if (pConfig->Resolution == ISI_RES_672_376P60) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P60_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_672_376P30) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P30_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_672_376P25) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P25_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_672_376P20) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P20_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_672_376P15) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P15_twolane_fpschg);
	                    }  else  if (pConfig->Resolution == ISI_RES_672_376P10) {                        
	                        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_672x376P10_twolane_fpschg);
	                    }
						
			      TRACE( OV4689_INFO, "%s  (setting flag): Resolution %dx%d@%dfps  \n", __FUNCTION__, ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution), ISI_FPS_GET(pConfig->Resolution));
			}
			
                   usTimeHts = 0x0505;
				   if (pConfig->Resolution == ISI_RES_672_376P60) {
	                    usTimeVts= 0x0614;
	               } else if (pConfig->Resolution == ISI_RES_672_376P30) {
	                    usTimeVts = 0x074a;
	               } else if (pConfig->Resolution == ISI_RES_672_376P25) {
	                    usTimeVts = 0x0c2c;
	               } else if (pConfig->Resolution == ISI_RES_672_376P20) {
	                    usTimeVts = 0x1234;
	               } else if (pConfig->Resolution == ISI_RES_672_376P15) {
	                    usTimeVts = 0x1844;
	               } else if (pConfig->Resolution == ISI_RES_672_376P10) {
	                    usTimeVts = 0x2464;
	               }
                
			/* sleep a while, that sensor can take over new default values */
    		        osSleep( 10 );

		        pOV4689Ctx->IsiSensorMipiInfo.ulMipiFreq = 672;

			 break;	            
	        }
	       default:
	        {
	            TRACE( OV4689_ERROR, "%s: two lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
	}
	else if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
	    switch ( pConfig->Resolution )
	    {
		 case ISI_RES_TV720P30:
	        {
			if (set2Sensor == BOOL_TRUE) {                    
                    		if (res_no_chg == BOOL_FALSE) {
                        		result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV4689Ctx, OV4689_g_fourlane_resolution_1280_720);
                    		}
                    		if (pConfig->Resolution == ISI_RES_TV720P30) {                        
                        		result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_fourlane_resolution_1280_720);
                    		} 
			      TRACE( OV4689_INFO, "%s  (setting flag): Resolution %dx%d@%dfps  \n", __FUNCTION__, ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution), ISI_FPS_GET(pConfig->Resolution));		
			}
			
    			usTimeHts = 0x035c; 
                usTimeVts = 0x0305;
                
    		    	/* sleep a while, that sensor can take over new default values */
    		    	osSleep( 10 );

			pOV4689Ctx->IsiSensorMipiInfo.ulMipiFreq = 600;
	            	break;
	            
	        }
	        default:
	        {
	            TRACE( OV4689_ERROR, "%s: four lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
	}


/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */

	usLineLengthPck = usTimeHts;
       usFrameLengthLines = usTimeVts;

	// store frame timing for later use in AEC module
	rVtPixClkFreq = OV4689_get_PCLK(pOV4689Ctx, xclk);    
       pOV4689Ctx->VtPixClkFreq     = rVtPixClkFreq;
       pOV4689Ctx->LineLengthPck    = usLineLengthPck;
       pOV4689Ctx->FrameLengthLines = usFrameLengthLines;	

	//have to reset mipi freq here,zyc

       TRACE( OV4689_ERROR, "%s  oyyf (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   usLineLengthPck:%d   usFrameLengthLines:%d    rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,usLineLengthPck,usFrameLengthLines,rVtPixClkFreq);

       return ( result );
}

static RESULT OV4689_SetupOutputWindow
(
    OV4689_Context_t        *pOV4689Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV4689Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV4689Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return OV4689_SetupOutputWindowInternal(pOV4689Ctx, pConfig, BOOL_TRUE, BOOL_FALSE);
}



/*****************************************************************************/
/**
 *          OV4689_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV4689 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_SetupImageControl
(
    OV4689_Context_t        *pOV4689Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = OV4689_IsiRegReadIss(  pOV4689Ctx, OV4689_BLC_CTRL00, &RegValue );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = OV4689_IsiRegReadIss(  pOV4689Ctx, OV4689_BLC_CTRL00, &RegValue );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = OV4689_IsiRegReadIss(  pOV4689Ctx, OV4689_AEC_MANUAL, &RegValue );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = OV4689_IsiRegReadIss(  pOV4689Ctx, OV4689_ISP_CTRL01, &RegValue );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = OV4689_IsiRegReadIss(  pOV4689Ctx, OV4689_AEC_MANUAL, &RegValue );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = OV4689_IsiRegReadIss( pOV4689Ctx, OV4689_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV4689_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV4689-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV4689 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_AecSetModeParameters
(
    OV4689_Context_t       *pOV4689Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;


    if ( (pOV4689Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( OV4689_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pOV4689Ctx->AecMaxIntegrationTime = ( ((float)(pOV4689Ctx->FrameLengthLines - 4)) * ((float)pOV4689Ctx->LineLengthPck) ) / pOV4689Ctx->VtPixClkFreq;
    pOV4689Ctx->AecMinIntegrationTime = 0.0001f;    

    pOV4689Ctx->AecMaxGain = OV4689_MAX_GAIN_AEC;
    pOV4689Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pOV4689Ctx->AecIntegrationTimeIncrement = ((float)pOV4689Ctx->LineLengthPck) / pOV4689Ctx->VtPixClkFreq;
    pOV4689Ctx->AecGainIncrement = OV4689_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pOV4689Ctx->AecCurGain               = pOV4689Ctx->AecMinGain;
    pOV4689Ctx->AecCurIntegrationTime    = 0.0f;
    pOV4689Ctx->OldCoarseIntegrationTime = 0;
    pOV4689Ctx->OldFineIntegrationTime   = 0;
    //pOV4689Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time


	
       TRACE( OV4689_ERROR, "%s%s (enter)  Res: 0x%x  0x%x maxT(%f) MaxGain(%f) hts(%d) vts(%d) pclk(%d) \n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"",
        pOV4689Ctx->Config.Resolution, pConfig->Resolution,  pOV4689Ctx->AecMaxIntegrationTime,  pOV4689Ctx->AecMaxGain, pOV4689Ctx->LineLengthPck,pOV4689Ctx->FrameLengthLines,pOV4689Ctx->VtPixClkFreq);

    TRACE( OV4689_INFO, "%s%s (exit)\n", __FUNCTION__, pOV4689Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          OV4689_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV4689 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV4689Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV4689Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV4689_IsiRegWriteIss ( pOV4689Ctx, OV4689_SOFTWARE_RST, 0x01U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = OV4689_IsiRegWriteIss( pOV4689Ctx, OV4689_MODE_SELECT, 0x00 );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: Can't write OV4689 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
	if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
//		result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_aRegDescription_onelane);
	}
	else if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
	    result = IsiRegDefaultsApply( pOV4689Ctx,OV4689_g_fourlane_resolution_1280_720);
    }
	else if(pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
        result = IsiRegDefaultsApply( pOV4689Ctx, OV4689_g_aRegDescription_twolane);
	}
    
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pOV4689Ctx, OV4689_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = OV4689_SetupOutputFormat( pOV4689Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = OV4689_SetupOutputWindow( pOV4689Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV4689_SetupImageControl( pOV4689Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV4689_AecSetModeParameters( pOV4689Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pOV4689Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiChangeSensorResolutionIss
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
static RESULT OV4689_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));


    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV4689Ctx->Configured != BOOL_TRUE))
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (OV4689_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV4689Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;

        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV4689Ctx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV4689Ctx->Config.Resolution))) ) {

            if (pOV4689Ctx->Streaming != BOOL_FALSE) {
                TRACE( OV4689_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        		
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV4689_INFO, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pOV4689Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV4689_SetupOutputWindowInternal( pOV4689Ctx, &pOV4689Ctx->Config,BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( OV4689_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV4689Ctx->AecCurGain;
        float OldIntegrationTime = pOV4689Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV4689_AecSetModeParameters( pOV4689Ctx, &pOV4689Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV4689_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV4689_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV4689_ERROR, "%s: OV4689_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else         
        	*pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
    }

    TRACE( OV4689_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiSensorSetStreamingIss
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
static RESULT OV4689_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;
	uint32_t RegValue2 = 0;

    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	
    TRACE( OV4689_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV4689Ctx->Configured != BOOL_TRUE) || (pOV4689Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = OV4689_IsiRegReadIss ( pOV4689Ctx, OV4689_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV4689_IsiRegWriteIss ( pOV4689Ctx, OV4689_MODE_SELECT, (RegValue | 0x01U) );
//	    result = OV4689_IsiRegWriteIss ( pOV4689Ctx, 0x5040,0x80);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		
        TRACE(OV4689_INFO," STREAM ON ++++++++++++++");
    }
    else
    {   
        /* disable streaming */
        result = OV4689_IsiRegReadIss ( pOV4689Ctx, OV4689_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV4689_IsiRegWriteIss ( pOV4689Ctx, OV4689_MODE_SELECT, (RegValue & ~0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        TRACE(OV4689_ERROR," STREAM OFF ++++++++++++++");
    }

    if (result == RET_SUCCESS)
    {
        pOV4689Ctx->Streaming = on;
    }

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV4689 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV4689Ctx->Configured = BOOL_FALSE;
    pOV4689Ctx->Streaming  = BOOL_FALSE;

    result = HalSetPower( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetReset( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        result = HalSetPower( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pOV4689Ctx->IsiCtx.HalHandle, pOV4689Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV4689_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (OV4689_CHIP_ID_LOW_BYTE_DEFAULT);

    result = OV4689_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV4689_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }


    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetSensorRevisionIss
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
static RESULT OV4689_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;
	uint32_t vcm_pos = MAX_LOG;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV4689_IsiRegReadIss ( handle, OV4689_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = OV4689_IsiRegReadIss ( handle, OV4689_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF) );

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV4689_IsiRegReadIss
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
static RESULT OV4689_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV4689_g_aRegDescription_fourlane);
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
 //       TRACE( OV4689_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x)\n", __FUNCTION__, NrOfBytes, address);

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( OV4689_ERROR, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiRegWriteIss
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
static RESULT OV4689_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV4689_g_aRegDescription_fourlane);
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
//    TRACE( OV4689_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x 0x%08x)\n", __FUNCTION__, NrOfBytes, address, value);

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( OV4689_ERROR, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV4689 instance
 *
 * @param   handle       OV4689 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV4689_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pOV4689Ctx->AecMinGain;
    *pMaxGain = pOV4689Ctx->AecMaxGain;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV4689 instance
 *
 * @param   handle       OV4689 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV4689_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pOV4689Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pOV4689Ctx->AecMaxIntegrationTime;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV4689 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pOV4689Ctx->AecCurGain;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV4689_IsiGetColorIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV4689 sensor instance handle
 * @param   pSetGain                get color
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetColorIss
(
    IsiSensorHandle_t   handle,
    char				*pGetColor
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pGetColor == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pGetColor = COLOUR;


    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV4689 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV4689Ctx->AecGainIncrement;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  OV4689 sensor instance handle
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
RESULT OV4689_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( OV4689_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pOV4689Ctx->AecMinGain ) NewGain = pOV4689Ctx->AecMinGain;
    if( NewGain > pOV4689Ctx->AecMaxGain ) NewGain = pOV4689Ctx->AecMaxGain;

    //usGain = (uint16_t)(NewGain * 128.0f+0.5);
  
    if (NewGain>=1.0 && NewGain < 2.0) {
		usGain = (uint16_t)(NewGain * 128.0f);
    	}
	else if (NewGain >=2.0 && NewGain < 4.0) {
		usGain = 0x0100 + ((uint16_t)(NewGain * 64.0f) - 8);
    	}
	else if (NewGain >=4.0 && NewGain < 8.0) {
		usGain = 0x0300 + ((uint16_t)(NewGain * 32.0f) - 12);
    	}
	else if(NewGain >=8.0 && NewGain <= 16.0) {
		usGain = 0x0700 + ((uint16_t)(NewGain * 16.0f) - 8);
		}

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pOV4689Ctx->OldGain) )
    {
    	// TODO:  check Gain reg
        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3508, (usGain>>8));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    	// TODO: check Gain reg
        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3509, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pOV4689Ctx->OldGain = usGain;
    }

    //calculate gain actually set
    //pOV4689Ctx->AecCurGain = ( (float)usGain ) / 128.0f;

	 if ((usGain>>8) == 0) {
		pOV4689Ctx->AecCurGain = ( (float)(usGain&0xff) ) / 128.0f;
    	}
	else if ((usGain>>8) == 1) {
		pOV4689Ctx->AecCurGain = ( (float)(usGain&0xff) + 8.0f) / 64.0f;
    	}
	else if ((usGain>>8) == 3) {
		pOV4689Ctx->AecCurGain = ( (float)(usGain&0xff) + 12.0f ) / 32.0f;
    	}
	else if ((usGain>>8) == 7){
		pOV4689Ctx->AecCurGain = ( (float)(usGain&0xff) + 8.0f ) / 16.0f;
	}

    //return current state
    *pSetGain = pOV4689Ctx->AecCurGain;
    TRACE( OV4689_ERROR, "-----------%s: psetgain=%f, NewGain=%f curGain(%f)\n", __FUNCTION__, *pSetGain, NewGain,pOV4689Ctx->AecCurGain);

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

     

/*****************************************************************************/
/**
 *          OV4689_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV4689 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV4689Ctx->AecCurIntegrationTime;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV4689 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV4689Ctx->AecIntegrationTimeIncrement;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  OV4689 sensor instance handle
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
RESULT OV4689_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
    //uint32_t FineIntegrationTime   = 0; //not supported by OV4689

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( OV4689_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pOV4689Ctx->AecMinIntegrationTime,
        pOV4689Ctx->AecMaxIntegrationTime);


    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( OV4689_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pOV4689Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV4689Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pOV4689Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV4689Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by OV4689
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pOV4689Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pOV4689Ctx->LineLengthPck == 0 )
    {
        TRACE( OV4689_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV4689Ctx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pOV4689Ctx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV4689Ctx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pOV4689Ctx->OldCoarseIntegrationTime )
    {
    		// TODO: check integration time register
        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3500, (CoarseIntegrationTime & 0x0000F000U) >> 12U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		// TODO: check integration time register
        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3501, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		// TODO: check integration time register
        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3502, (CoarseIntegrationTime & 0x0000000FU) << 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pOV4689Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //if( FineIntegrationTime != pOV4689Ctx->OldFineIntegrationTime )
    //{
    //    result = OV4689_IsiRegWriteIss( pOV4689Ctx, ... , FineIntegrationTime );
    //    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    //    pOV4689Ctx->OldFineIntegrationTime = FineIntegrationTime; //remember current integration time
    //    *pNumberOfFramesToSkip = 1U; //skip 1 frame
    //}

    //calculate integration time actually set
    //pOV4689Ctx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pOV4689Ctx->LineLengthPck) + ((float)FineIntegrationTime) ) / pOV4689Ctx->VtPixClkFreq;
    pOV4689Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pOV4689Ctx->LineLengthPck) / pOV4689Ctx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pOV4689Ctx->AecCurIntegrationTime;

    TRACE( OV4689_INFO, "%s: SetTi=%f NewTi=%f,CoarseIntegrationTime=0x%x\n", __FUNCTION__, *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime);   
    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          OV4689_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV4689 sensor instance handle
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
RESULT OV4689_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV4689_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV4689_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );

	
	result |= OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3208, 0x00);//6c 3208 00	;group 0 hold start
	
    result |= OV4689_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result |= OV4689_IsiSetGainIss( handle, NewGain, pSetGain );

	result |= OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3208, 0x10);//6c 3208 10  ;group 0 hold end
	result |= OV4689_IsiRegWriteIss( pOV4689Ctx, 0x320b, 0x00);//6c 320b 00
	result |= OV4689_IsiRegWriteIss( pOV4689Ctx, 0x3208, 0xe0);//6c 3208 e0  ;quick launch group 0



    TRACE( OV4689_ERROR, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV4689_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV4689_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV4689Ctx->AecCurGain;
    *pSetIntegrationTime = pOV4689Ctx->AecCurIntegrationTime;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetResolutionIss
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
RESULT OV4689_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV4689Ctx->Config.Resolution;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV4689Ctx             OV4689 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetAfpsInfoHelperIss(
    OV4689_Context_t   *pOV4689Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV4689Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV4689Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV4689_SetupOutputWindowInternal( pOV4689Ctx, &pOV4689Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV4689_AecSetModeParameters( pOV4689Ctx, &pOV4689Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV4689_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV4689Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV4689Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV4689Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV4689Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV4689Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV4689_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV4689 sensor instance handle
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
RESULT OV4689_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        TRACE( OV4689_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV4689Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pOV4689Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pOV4689Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pOV4689Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    OV4689_Context_t *pDummyCtx = (OV4689_Context_t*) malloc( sizeof(OV4689_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( OV4689_ERROR,  "%s: Can't allocate dummy ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pOV4689Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = OV4689_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
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
    switch (pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes)
    {
        case SUPPORT_MIPI_ONE_LANE:
        {

            break;
        }

        case SUPPORT_MIPI_TWO_LANE:
        {
    
		switch(Resolution)
		{
		       default:
		            TRACE( OV4689_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
		            result = RET_NOTSUPP;
		            break;	        
			case ISI_RES_2688_1520P30:
			case ISI_RES_2688_1520P25:
			case ISI_RES_2688_1520P20:
		 	case ISI_RES_2688_1520P15:
			case ISI_RES_2688_1520P10:
		 	case ISI_RES_2688_1520P7:
				AFPSCHECKANDADD( ISI_RES_2688_1520P30 );
				AFPSCHECKANDADD( ISI_RES_2688_1520P25 );
				AFPSCHECKANDADD( ISI_RES_2688_1520P20 );
				AFPSCHECKANDADD( ISI_RES_2688_1520P15 );
				AFPSCHECKANDADD( ISI_RES_2688_1520P10 );
				AFPSCHECKANDADD( ISI_RES_2688_1520P7 );
				break;

			#if 1
			case ISI_RES_TV1080P30:
		 	case ISI_RES_TV1080P25:
			case ISI_RES_TV1080P20:
		 	case ISI_RES_TV1080P15:
		 	case ISI_RES_TV1080P10:
				AFPSCHECKANDADD( ISI_RES_TV1080P30 );
				AFPSCHECKANDADD( ISI_RES_TV1080P25 );
				AFPSCHECKANDADD( ISI_RES_TV1080P20 );
				AFPSCHECKANDADD( ISI_RES_TV1080P15 );
				AFPSCHECKANDADD( ISI_RES_TV1080P10 );
				break;		 
			case ISI_RES_672_376P60:
		 	case ISI_RES_672_376P30:
		 	case ISI_RES_672_376P25:
			case ISI_RES_672_376P20:
		 	case ISI_RES_672_376P15:
			case ISI_RES_672_376P10:
				AFPSCHECKANDADD( ISI_RES_672_376P60 );
				AFPSCHECKANDADD( ISI_RES_672_376P30 );
				AFPSCHECKANDADD( ISI_RES_672_376P25 );
				AFPSCHECKANDADD( ISI_RES_672_376P20 );
				AFPSCHECKANDADD( ISI_RES_672_376P15 );
				AFPSCHECKANDADD( ISI_RES_672_376P10 );
				break;
		        // check next series here...
			#endif
		}
		break;
        }
        case SUPPORT_MIPI_FOUR_LANE:
        {
		    switch(Resolution)
		    {
		      default:
		            TRACE( OV4689_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
		            result = RET_NOTSUPP;
		            break;
			case ISI_RES_TV720P30:
				AFPSCHECKANDADD( ISI_RES_TV720P30 ); 	
				break;

		        // check next series here...
		    }			
			break;
        }
        default:
        {
            TRACE( OV4689_ERROR,  "%s: pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;
	}
    }
	
    // release dummy context again
    free(pDummyCtx);

	

    TRACE( OV4689_INFO, "%s: enable res:%d (exit)\n", __FUNCTION__, idx);

    return ( result );
}

#ifdef OV4689_CALIB_ENABLE
/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibKFactor
 *
 * @brief   Returns the OV4689 specific K-Factor
 *
 * @param   handle       OV4689 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV4689_KFactor;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV4689 specific PCA-Matrix
 *
 * @param   handle          OV4689 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV4689_PCAMatrix;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV4689 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV4689_SVDMeanValue;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV4689 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiCenterLine = (IsiLine_t*)&OV4689_CenterLine;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV4689 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&OV4689_AwbClipParm;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV4689 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV4689_AwbGlobalFadeParm;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV4689 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV4689_AwbFade2Parm;

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV4689_IsiGetIlluProfile
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
static RESULT OV4689_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
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
        for ( i=0U; i<OV4689_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV4689_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV4689_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV4689_IsiGetLscMatrixTable
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
static RESULT OV4689_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
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
                if ( ( pOV4689Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pOV4689Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV4689_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pOV4689Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pOV4689Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV4689_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pOV4689Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pOV4689Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV4689_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pOV4689Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pOV4689Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV4689_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pOV4689Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pOV4689Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV4689_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV4689_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV4689_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}
#endif

#ifdef OV4689_AF_ENABLE

/*****************************************************************************/
/**
 *          OV4689_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV4689 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    /* ddl@rock-chips.com: v0.3.0 */
    if (pOV4689Ctx->VcmInfo.StepMode <= 7) {
        vcm_movefull_t = 52*(1<<(pOV4689Ctx->VcmInfo.StepMode-1));
    } else if ((pOV4689Ctx->VcmInfo.StepMode>=9) && (pOV4689Ctx->VcmInfo.StepMode<=15)) {
        vcm_movefull_t = 2*(1<<(pOV4689Ctx->VcmInfo.StepMode-9));
    } else {
        TRACE( OV4689_ERROR, "%s: pOV8825Ctx->VcmInfo.StepMode: %d is invalidate!\n",__FUNCTION__, pOV4689Ctx->VcmInfo.StepMode);
        DCT_ASSERT(0);
    }

    *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));

//    *pMaxStep = MAX_LOG;

    result = OV4689_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV4689 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
	OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
    	TRACE( OV4689_ERROR, "%s: pOV4689Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( OV4689_ERROR, "%s: pOV4689Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}
	/* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pOV4689Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pOV4689Ctx->VcmInfo.StartCurrent + (pOV4689Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( OV4689_DEBUG, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);

    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0

    TRACE( OV4689_DEBUG, "%s: value = %d, 0x%02x 0x%02x af_addr(0x%x) bus(%d)\n", __FUNCTION__, nPosition, data[0], data[1],pOV4689Ctx->IsiCtx.SlaveAfAddress,pOV4689Ctx->IsiCtx.I2cAfBusNum );

    result = HalWriteI2CMem( pOV4689Ctx->IsiCtx.HalHandle,
                             pOV4689Ctx->IsiCtx.I2cAfBusNum,
                             pOV4689Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pOV4689Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV4689 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };


    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV4689Ctx->IsiCtx.HalHandle,
                            pOV4689Ctx->IsiCtx.I2cAfBusNum,
                            pOV4689Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pOV4689Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );

    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV4689_DEBUG, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see AD5820 datasheet */
    /* Data[1] = D3..D0, S3..S0 */
	#if 0
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

    /* map 0 to 64 -> infinity */
    if( *pAbsStep == 0 )
    {
        *pAbsStep = MAX_LOG;
    }
    else
    {
        *pAbsStep = ( MAX_REG - *pAbsStep ) / 16U;
    }
	#endif
	*pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );


    /* map 0 to 64 -> infinity */   /* ddl@rock-chips.com: v0.3.0 */
    if( *pAbsStep <= pOV4689Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pOV4689Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pOV4689Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pOV4689Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pOV4689Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
	

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);


    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV4689_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}
#endif


/*****************************************************************************/
/**
 *          OV4689_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT OV4689_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t ulRegValue = 0UL;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV4689_IsiRegReadIss( pOV4689Ctx, 0x5040, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x5040, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV4689_IsiRegReadIss( pOV4689Ctx, 0x5040, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV4689_IsiRegWriteIss( pOV4689Ctx, 0x5040, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pOV4689Ctx->TestPattern = enable;
    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV4689_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV4689 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT OV4689_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

    ptIsiSensorMipiInfo->ucMipiLanes = pOV4689Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pOV4689Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pOV4689Ctx->IsiSensorMipiInfo.sensorHalDevID;


    TRACE( OV4689_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV4689_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
    	TRACE( OV4689_ERROR, "%s: pOV4689Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV4689_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV4689_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV4689_Context_t *pOV4689Ctx = (OV4689_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV4689Ctx == NULL )
    {
    	TRACE( OV4689_ERROR, "%s: pOV4689Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV4689_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV4689_NEWEST_TUNING_XML;
	return result;
}

/*****************************************************************************/
/**
 *          OV4689_IsiGetSensorIss
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
RESULT OV4689_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV4689_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = OV4689_g_acName;
        pIsiSensor->pRegisterTable                      = OV4689_g_aRegDescription_twolane;/*OV4689_g_fourlane_resolution_1280_720;*/
        pIsiSensor->pIsiSensorCaps                      = &OV4689_g_IsiSensorDefaultConfig;
        pIsiSensor->pIsiGetSensorIsiVer					= OV4689_IsiGetSensorIsiVersion;//oyyf

        pIsiSensor->pIsiGetSensorTuningXmlVersion		= OV4689_IsiGetSensorTuningXmlVersion;//oyyf

        pIsiSensor->pIsiCreateSensorIss                 = OV4689_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = OV4689_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = OV4689_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = OV4689_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = OV4689_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = OV4689_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = OV4689_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = OV4689_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = OV4689_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = OV4689_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = OV4689_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = OV4689_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = OV4689_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = OV4689_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = OV4689_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV4689_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV4689_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV4689_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV4689_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV4689_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV4689_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV4689_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV4689_IsiGetAfpsInfoIss;


        /* AWB specific functions */
#ifdef OV4689_CALIB_ENABLE
        pIsiSensor->pIsiGetCalibKFactor                 = OV4689_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = OV4689_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = OV4689_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = OV4689_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = OV4689_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = OV4689_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = OV4689_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = OV4689_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = OV4689_IsiGetLscMatrixTable;
#endif

        /* AF functions */
#ifdef OV4689_AF_ENABLE
        pIsiSensor->pIsiMdiInitMotoDriveMds             = OV4689_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = OV4689_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = OV4689_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = OV4689_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = OV4689_IsiMdiFocusCalibrate;
#endif

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = OV4689_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = OV4689_IsiActivateTestPattern;
		pIsiSensor->pIsiGetColorIss						= OV4689_IsiGetColorIss;

    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV4689_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV4689_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV4689_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV4689_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = OV4689_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = OV4689_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = 0x01;
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
                while(OV4689_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = OV4689_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = OV4689_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV4689_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = OV4689_CHIP_ID_LOW_BYTE_DEFAULT;
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
    OV4689_IsiGetSensorIss,
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
		0,						/**< IsiSensor_t.pIsiGetColorIss */

    },
    OV4689_IsiGetSensorI2cInfo,
};


