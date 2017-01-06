//ov8825 the same with ov14825

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
 * @file OV5645.c
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

#include "OV5645_priv.h"


#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV5645_INFO , "OV5645: ", ERROR,    1U );
CREATE_TRACER( OV5645_WARN , "OV5645: ", ERROR, 1U );
CREATE_TRACER( OV5645_ERROR, "OV5645: ", ERROR,   1U );

CREATE_TRACER( OV5645_DEBUG, "OV5645: ", ERROR,     1U );

CREATE_TRACER( OV5645_REG_INFO , "OV5645: ", ERROR, 1);
CREATE_TRACER( OV5645_REG_DEBUG, "OV5645: ", INFO, 0U );

#define OV5645_SLAVE_ADDR       0x78U                         /**< i2c slave address of the OV5645 camera sensor */
#define OV5645_SLAVE_ADDR2       0x78U
#define OV5645_SLAVE_AF_ADDR    0x00U                           /**< i2c slave address of the OV5645 integrated AD5820 */



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV5645_g_acName[] = "OV5645_SOC_PARREL";
extern const IsiRegDescription_t OV5645_g_aRegDescription[];
extern const IsiRegDescription_t OV5645_g_twolane_resolution_1280_960[];
extern const IsiRegDescription_t OV5645_g_twolane_resolution_2592_1944[];


const IsiSensorCaps_t OV5645_g_IsiSensorDefaultConfig;


#define OV5645_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define OV5645_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define OV5645_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_ONE_LANE


/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV5645_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV5645_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV5645_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV5645_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV5645_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV5645_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV5645_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV5645_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV5645_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV5645_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV5645_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV5645_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV5645_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT OV5645_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV5645_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV5645_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV5645_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV5645_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV5645_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV5645_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT OV5645_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV5645_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV5645_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV5645_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV5645_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV5645_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV5645_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV5645_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV5645_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV5645_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV5645_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV5645_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV5645_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV5645_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV5645_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT OV5645_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT OV5645_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);



/*****************************************************************************/
/**
 *          OV5645_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV5645 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV5645_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    OV5645_Context_t *pOV5645Ctx;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV5645Ctx = ( OV5645_Context_t * )malloc ( sizeof (OV5645_Context_t) );
    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV5645Ctx, 0, sizeof( OV5645_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV5645Ctx );
        return ( result );
    }

    pOV5645Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pOV5645Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pOV5645Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pOV5645Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? OV5645_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV5645Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pOV5645Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pOV5645Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? OV5645_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV5645Ctx->IsiCtx.NrOfAfAddressBytes     = 0;

    pOV5645Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pOV5645Ctx->Configured             = BOOL_FALSE;
    pOV5645Ctx->Streaming              = BOOL_FALSE;
    pOV5645Ctx->TestPattern            = BOOL_FALSE;
    pOV5645Ctx->isAfpsRun              = BOOL_FALSE;

    pOV5645Ctx->IsiSensorMipiInfo.sensorHalDevID = pOV5645Ctx->IsiCtx.HalDevID;
    if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type){
        pOV5645Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    }else{
        TRACE( OV5645_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pOV5645Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }

    pConfig->hSensor = ( IsiSensorHandle_t )pOV5645Ctx;

    result = HalSetCamConfig( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, true, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, 10000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV5645 sensor instance.
 *
 * @param   handle      OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV5645_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV5645_IsiSensorSetStreamingIss( pOV5645Ctx, BOOL_FALSE );
    (void)OV5645_IsiSensorSetPowerIss( pOV5645Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV5645Ctx->IsiCtx.HalHandle );

    MEMSET( pOV5645Ctx, 0, sizeof( OV5645_Context_t ) );
    free ( pOV5645Ctx );

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCapsIss
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
static RESULT OV5645_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t  mipi_lanes
)
{

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);


    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {

        if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE){            
            switch (pIsiSensorCaps->Index) 
            {
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
                    pIsiSensorCaps->Resolution = ISI_RES_1280_960P30;			
                    break;
                }    
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }
        }  else if(mipi_lanes == SUPPORT_MIPI_ONE_LANE) {
            
            switch (pIsiSensorCaps->Index) 
            {
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }
        } 
        TRACE(OV5645_DEBUG,"resolution_mipi_lanes = %x,  Index = %x",mipi_lanes,pIsiSensorCaps->Index);
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_8BIT_ZZ;
        pIsiSensorCaps->Mode            = ISI_MODE_BT601;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR|ISI_YCSEQ_CBYCRY;           
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_RGRGGBGB ;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_POS;
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_ON;
        pIsiSensorCaps->CConv           = ISI_CCONV_ON;
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO );
        pIsiSensorCaps->AGC             = ( ISI_AGC_AUTO );
        pIsiSensorCaps->AWB             = ( ISI_AWB_AUTO );
        pIsiSensorCaps->AEC             = ( ISI_AEC_AUTO );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL;
        pIsiSensorCaps->CieProfile      = 0;
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_YUV422_8;
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP );
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_YUV;
    }
end:
    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


static RESULT OV5645_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    result = OV5645_IsiGetCapsIssInternal(pIsiSensorCaps, pOV5645Ctx->IsiSensorMipiInfo.ucMipiLanes);

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5645_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV5645_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_8BIT_ZZ,         // BusWidth
    ISI_MODE_BT601, //ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_RGRGGBGB,//ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_ON,              // Gamma
    ISI_CCONV_ON,              // CConv
    ISI_RES_1280_960P30,          // Res
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_AUTO,                // AGC
    ISI_AWB_AUTO,                // AWB
    ISI_AEC_AUTO,                // AEC
    ISI_DPCC_AUTO,               // DPCC
    0,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_YUV422_8,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_YUV,
    0,
};



/*****************************************************************************/
/**
 *          OV5645_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5645 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_SetupOutputFormat
(
    OV5645_Context_t       *pOV5645Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
    return result;

    TRACE( OV5645_INFO, "%s%s (enter)\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        //case ISI_BUSWIDTH_12BIT:
        case ISI_BUSWIDTH_8BIT_ZZ:
        //case ISI_BUSWIDTH_12BIT:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* mode */
    switch ( pConfig->Mode )            /* only ISI_MODE_BAYER supported, no configuration needed here */
    {
        case( ISI_MODE_MIPI ):
        case ISI_MODE_BAYER:
        case ISI_MODE_BT601:
        case ISI_MODE_PICT:
        case  ISI_MODE_DATA:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV5645_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

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
        case ISI_CONV422_COSITED:
        case ISI_CONV422_INTER:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
        case ISI_BPAT_BGBGGRGR:
        case ISI_BPAT_RGRGGBGB:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV5645_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* vertical polarity */
    switch ( pConfig->VPol )            /*no configuration needed */
    {
        case ISI_VPOL_NEG:
        {
            break;
        }
        case ISI_VPOL_POS:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV5645_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* gamma */
    switch ( pConfig->Gamma )           /* only ISI_GAMMA_OFF supported, no configuration needed */
    {
        case ISI_GAMMA_ON:
        case ISI_GAMMA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* color conversion */
    switch ( pConfig->CConv )           /* only ISI_CCONV_OFF supported, no configuration needed */
    {
        case ISI_CCONV_OFF:
        case ISI_CCONV_ON:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV5645_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->MipiMode )        /* only ISI_MIPI_MODE_RAW_12 supported, no configuration needed */
    {
        case ISI_MIPI_MODE_RAW_10:
		case ISI_MIPI_MODE_YUV422_8:
        case ISI_MIPI_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV5645_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( OV5645_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV5645_INFO, "%s%s (exit)\n", __FUNCTION__, pOV5645Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

int OV5645_get_PCLK( OV5645_Context_t *pOV5645Ctx, int XVCLK)
{
    // calculate PCLK
    
    return 0;
}

/*****************************************************************************/
/**
 *          OV5645_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5645 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_SetupOutputWindow
(
    OV5645_Context_t        *pOV5645Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result     = RET_SUCCESS;

	TRACE( OV5645_ERROR, "%s (enter.....)\n", __FUNCTION__);
	pOV5645Ctx->IsiSensorMipiInfo.ulMipiFreq = 420; //680; //620; //480; //580; //528; //328;
        /* resolution */
    switch ( pConfig->Resolution )
    {
        case ISI_RES_1280_960P30:
        {
            if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV5645Ctx,OV5645_g_twolane_resolution_1280_960)) != RET_SUCCESS){
                TRACE( OV5645_ERROR, "%s: failed to set  OV5645_g_twolane_resolution_1280_960 \n", __FUNCTION__ );
            }else{

                TRACE( OV5645_ERROR, "%s: success to set  OV5645_g_twolane_resolution_1280_960 \n", __FUNCTION__ );
            }
            break;
        }
        case ISI_RES_2592_1944P15:
        {
            if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pOV5645Ctx,OV5645_g_twolane_resolution_2592_1944)) != RET_SUCCESS){
                TRACE( OV5645_ERROR, "%s: failed to set  OV5645_g_twolane_resolution_2592_1944 \n", __FUNCTION__ );
            }else{

                TRACE( OV5645_INFO, "%s: success to set  OV5645_g_twolane_resolution_2592_1944  \n", __FUNCTION__ );
            }
            break;
        }
        default:
        {
            TRACE( OV5645_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

 TRACE( OV5645_ERROR, "@@@@%s, pConfig->Resolution = %x\n", __FUNCTION__,pConfig->Resolution );
    return ( result );
}




/*****************************************************************************/
/**
 *          OV5645_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV5645 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_SetupImageControl
(
    OV5645_Context_t        *pOV5645Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);


    return ( result );
}


/*****************************************************************************/
/**
 *          OV5645_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV5645-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV5645 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_AecSetModeParameters
(
    OV5645_Context_t       *pOV5645Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    return ( result );
}
//static int int11;
/*****************************************************************************/
/**
 *          OV5645_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5645 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);
	
    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV5645Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV5645Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );
	TRACE( OV5645_ERROR, "%s (enter)\n", __FUNCTION__);

	
		TRACE( OV5645_ERROR, "%s (enter).........\n", __FUNCTION__);
	    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
	    result = IsiRegDefaultsApply( pOV5645Ctx, OV5645_g_aRegDescription );
	    if ( result != RET_SUCCESS )
	    {
	        return ( result );
	    }

	    /* sleep a while, that sensor can take over new default values */
	    osSleep( 10 );
	    #if 0


	    /* 3.) verify default values to make sure everything has been written correctly as expected */
	    result = IsiRegDefaultsVerify( pOV5645Ctx, OV5645_g_aRegDescription );
	    if ( result != RET_SUCCESS )
	    {
	        return ( result );
	    }
	    // output of pclk for measurement (only debugging)
	    result = OV5645_IsiRegWriteIss( pOV5645Ctx, 0x3009U, 0x10U );
	    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	    #endif

	    /* 4.) setup output format (RAW10|RAW12) */
	    result = OV5645_SetupOutputFormat( pOV5645Ctx, pConfig );
	    if ( result != RET_SUCCESS )
	    {
	        TRACE( OV5645_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
	        return ( result );
	    }

	    /* 5.) setup output window */
	    result = OV5645_SetupOutputWindow( pOV5645Ctx, pConfig );
	    if ( result != RET_SUCCESS )
	    {
	        TRACE( OV5645_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
	        return ( result );
	    }

	    result = OV5645_SetupImageControl( pOV5645Ctx, pConfig );
	    if ( result != RET_SUCCESS )
	    {
	        TRACE( OV5645_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
	        return ( result );
	    }

	    result = OV5645_AecSetModeParameters( pOV5645Ctx, pConfig );
	    if ( result != RET_SUCCESS )
	    {
	        TRACE( OV5645_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
	        return ( result );
	    }
		

    if (result == RET_SUCCESS)
    {
        pOV5645Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiChangeSensorResolutionIss
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
static RESULT OV5645_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV5645Ctx->Configured != BOOL_TRUE) || (pOV5645Ctx->Streaming != BOOL_FALSE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (OV5645_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV5645Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV5645_ERROR, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pOV5645Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV5645_SetupOutputWindow( pOV5645Ctx, &pOV5645Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV5645_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV5645Ctx->AecCurGain;
        float OldIntegrationTime = pOV5645Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV5645_AecSetModeParameters( pOV5645Ctx, &pOV5645Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV5645_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip = 2;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV5645_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV5645_ERROR, "%s: OV5645_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
    }

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiSensorSetStreamingIss
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
static RESULT OV5645_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV5645Ctx->Configured != BOOL_TRUE) || (pOV5645Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
		result = OV5645_IsiRegWriteIss( handle, 0x3008, 0x02);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       // result = OV5645_IsiRegWriteIss( handle, 0x4202, 0x0);
    }
    else
    {
        /* disable streaming */	
		result = OV5645_IsiRegWriteIss( handle, 0x3008, 0x42);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
      //  result = OV5645_IsiRegWriteIss( handle, 0x4202, 0x03);
    }

    if (result == RET_SUCCESS)
    {
        pOV5645Ctx->Streaming = on;
    }

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV5645 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV5645Ctx->Configured = BOOL_FALSE;
    pOV5645Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV5645_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5645_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        TRACE( OV5645_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5645_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5645_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5645_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV5645Ctx->IsiCtx.HalHandle, pOV5645Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV5645_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (OV5645_CHIP_ID_LOW_BYTE_DEFAULT);
    //RevId = RevId | OV5645_CHIP_ID_LOW_BYTE_DEFAULT;

    result = OV5645_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV5645_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( OV5645_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
    TRACE(OV5645_DEBUG,"@@@hw RevId = 0x%08x, value = 0x%08x\n",RevId,value); //hw++
    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetSensorRevisionIss
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
static RESULT OV5645_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV5645_IsiRegReadIss ( handle, OV5645_CHIP_ID_HIGH_BYTE, &data );
	TRACE(OV5645_DEBUG,"@@@hw Sensor_CHIP_ID_HIGH_BYTE = %x\n",data); //hw++
    *p_value = ( (data & 0xFF) << 8U );
    //result = OV5645_IsiRegReadIss ( handle, OV5645_CHIP_ID_MIDDLE_BYTE, &data );
    //*p_value |= ( (data & 0xFF) << 8U );
    result = OV5645_IsiRegReadIss ( handle, OV5645_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));
     TRACE(OV5645_DEBUG,"@@@hw Sensor_CHIP_ID_LOW_BYTE = %x\n",data); //hw++
    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiRegReadIss
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
static RESULT OV5645_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV5645_g_aRegDescription );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
 //       TRACE( OV5645_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x)\n", __FUNCTION__, NrOfBytes, address);

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( OV5645_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiRegWriteIss
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
static RESULT OV5645_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV5645_g_aRegDescription );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
//    TRACE( OV5645_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x 0x%08x)\n", __FUNCTION__, NrOfBytes, address, value);

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( OV5645_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV5645 instance
 *
 * @param   handle       OV5645 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV5645_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pOV5645Ctx->AecMinGain;
    *pMaxGain = pOV5645Ctx->AecMaxGain;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV5645 instance
 *
 * @param   handle       OV5645 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV5645_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pOV5645Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pOV5645Ctx->AecMaxIntegrationTime;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5645_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV5645 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pOV5645Ctx->AecCurGain;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV5645 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV5645Ctx->AecGainIncrement;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  OV5645 sensor instance handle
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
RESULT OV5645_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( OV5645_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pOV5645Ctx->AecMinGain ) NewGain = pOV5645Ctx->AecMinGain;
    if( NewGain > pOV5645Ctx->AecMaxGain ) NewGain = pOV5645Ctx->AecMaxGain;

    usGain = (uint16_t)NewGain;

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pOV5645Ctx->OldGain) )
    {

        pOV5645Ctx->OldGain = usGain;
    }

    //calculate gain actually set
    pOV5645Ctx->AecCurGain = NewGain;

    //return current state
    *pSetGain = pOV5645Ctx->AecCurGain;
    TRACE( OV5645_DEBUG, "%s: g=%f\n", __FUNCTION__, *pSetGain );

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV5645 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV5645Ctx->AecCurIntegrationTime;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV5645 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV5645Ctx->AecIntegrationTimeIncrement;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  OV5645 sensor instance handle
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
RESULT OV5645_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV5645_DEBUG, "%s: Ti=%f\n", __FUNCTION__, *pSetIntegrationTime );
    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          OV5645_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV5645 sensor instance handle
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
RESULT OV5645_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV5645_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV5645_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = OV5645_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = OV5645_IsiSetGainIss( handle, NewGain, pSetGain );

	//*pNumberOfFramesToSkip = 2;

    TRACE( OV5645_DEBUG, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5645_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV5645Ctx->AecCurGain;
    *pSetIntegrationTime = pOV5645Ctx->AecCurIntegrationTime;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetResolutionIss
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
RESULT OV5645_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV5645Ctx->Config.Resolution;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV5645Ctx             OV5645 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetAfpsInfoHelperIss(
    OV5645_Context_t   *pOV5645Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV5645Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV5645Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV5645_SetupOutputWindow( pOV5645Ctx, &pOV5645Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5645_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV5645_AecSetModeParameters( pOV5645Ctx, &pOV5645Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5645_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV5645Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV5645Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV5645Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV5645Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV5645Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV5645_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV5645 sensor instance handle
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
RESULT OV5645_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        TRACE( OV5645_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV5645Ctx->Config.Resolution;
    }

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibKFactor
 *
 * @brief   Returns the OV5645 specific K-Factor
 *
 * @param   handle       OV5645 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiKFactor = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV5645 specific PCA-Matrix
 *
 * @param   handle          OV5645 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiPcaMatrix = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV5645 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiSvdMeanValue = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV5645 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiCenterLine = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV5645 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiClipParam = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV5645 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiGlobalFadeParam = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV5645 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiFadeParam = NULL;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV5645_IsiGetIlluProfile
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
static RESULT OV5645_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetLscMatrixTable
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
static RESULT OV5645_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5645_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV5645 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }


    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV5645 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV5645 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5645_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT OV5645_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5645_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV5645 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT OV5645_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }
	
    ptIsiSensorMipiInfo->ucMipiLanes = pOV5645Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pOV5645Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pOV5645Ctx->IsiSensorMipiInfo.sensorHalDevID;

    TRACE( OV5645_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV5645_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV5645_Context_t *pOV5645Ctx = (OV5645_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV5645_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5645Ctx == NULL )
    {
    	TRACE( OV5645_ERROR, "%s: pOV5645Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV5645_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}


/*****************************************************************************/
/**
 *          OV5645_IsiGetSensorIss
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
RESULT OV5645_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5645_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = OV5645_g_acName;
        pIsiSensor->pRegisterTable                      = OV5645_g_aRegDescription;
        pIsiSensor->pIsiSensorCaps                      = &OV5645_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= OV5645_IsiGetSensorIsiVersion;

        pIsiSensor->pIsiCreateSensorIss                 = OV5645_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = OV5645_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = OV5645_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = OV5645_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = OV5645_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = OV5645_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = OV5645_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = OV5645_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = OV5645_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = OV5645_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = OV5645_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = OV5645_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = OV5645_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = OV5645_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = OV5645_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV5645_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV5645_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV5645_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV5645_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV5645_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV5645_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV5645_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV5645_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = OV5645_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = OV5645_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = OV5645_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = OV5645_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = OV5645_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = OV5645_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = OV5645_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = OV5645_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = OV5645_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = OV5645_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = OV5645_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = OV5645_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = OV5645_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = OV5645_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = OV5645_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = OV5645_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV5645_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV5645_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV5645_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV5645_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = OV5645_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = OV5645_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = 0x82;
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
                while(OV5645_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = OV5645_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = OV5645_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
#if 0
    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = OV5645_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = OV5645_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
 #endif   
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV5645_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = OV5645_CHIP_ID_LOW_BYTE_DEFAULT;
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
    OV5645_IsiGetSensorIss,
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
        0,						/**< IsiSensor_t.pIsiGetColorIss */
    },
    OV5645_IsiGetSensorI2cInfo,
};



