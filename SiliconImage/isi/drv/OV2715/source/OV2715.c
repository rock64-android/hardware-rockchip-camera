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
 * @file OV2715.c
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

#include "OV2715_priv.h"

#define  OV2715_NEWEST_TUNING_XML "16-Nov-2011_AN_OV2715_pentax_v1.0"

#define CC_OFFSET_SCALING  2.0f

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV2715_INFO , "OV2715: ", INFO,    0);
CREATE_TRACER( OV2715_WARN , "OV2715: ", WARNING, 1);
CREATE_TRACER( OV2715_ERROR, "OV2715: ", ERROR,   1);

CREATE_TRACER( OV2715_DEBUG,     "OV2715: ", INFO, 0);

CREATE_TRACER( OV2715_REG_INFO , "OV2715: ", INFO, 0);
CREATE_TRACER( OV2715_REG_DEBUG, "OV2715: ", INFO, 0);

#define OV2715_SLAVE_ADDR       0x36            /**< i2c slave address of the OV2715 camera sensor */


#define OV2715_MIN_GAIN_STEP    ( 1.0f/16.0f )  /**< min gain step size used by GUI (hardware min = 1/16; 1/16..32/16 depending on actual gain ) */
#define OV2715_MAX_GAIN_AEC     ( 16.0f )       /**< max. gain used by the AEC (arbitrarily chosen, hardware limit = 62.0, driver limit = 32.0 ) */



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV2715_g_acName[] = "OV2715";
extern const IsiRegDescription_t OV2715_g_aRegDescription[];
const IsiSensorCaps_t OV2715_g_IsiSensorDefaultConfig;

/* AWB specific value (from OV2715_tables.c) */
extern const Isi1x1FloatMatrix_t    OV2715_KFactor;
extern const Isi3x2FloatMatrix_t    OV2715_PCAMatrix;
extern const Isi3x1FloatMatrix_t    OV2715_SVDMeanValue;
extern const IsiLine_t              OV2715_CenterLine;
extern const IsiAwbClipParm_t       OV2715_AwbClipParm;
extern const IsiAwbGlobalFadeParm_t OV2715_AwbGlobalFadeParm;
extern const IsiAwbFade2Parm_t      OV2715_AwbFade2Parm;

/* illumination profiles */
#include "OV2715_a.h"       /* CIE A - default profile */
#include "OV2715_f2.h"      /* CIE F2 (cool white flourescent CWF) */
#include "OV2715_d50.h"     /* CIE D50 (D50 lightbox) */
#include "OV2715_d65.h"     /* CIE D65 (D65) note: indoor because of our lightbox */
#include "OV2715_d75.h"     /* CIE D75 (D75) overcast daylight, 7500K */
#include "OV2715_f11.h"     /* CIE F11 (TL84) */

#define OV2715_ISIILLUPROFILES_DEFAULT  6
static IsiIlluProfile_t OV2715_IlluProfileDefault[OV2715_ISIILLUPROFILES_DEFAULT] =
{
    {
        .p_next             = NULL,

        .name               = "A",
        .id                 = ISI_CIEPROF_A,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_CIE_A,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_CIE_A,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_CIE_A,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_CIE_A,
        .pGaussFactor       = &OV2715_GaussFactor_CIE_A,
        .pThreshold         = &OV2715_Threshold_CIE_A,
        .pComponentGain     = &OV2715_CompGain_CIE_A,

        .pSaturationCurve   = &OV2715_SaturationCurve_CIE_A,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_CIE_A,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_CIE_A,

        .pVignettingCurve   = &OV2715_VignettingCurve_CIE_A,
    },
    {
        .p_next             = NULL,

        .name               = "F2",
        .id                 = ISI_CIEPROF_F2,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_F2,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_F2,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_F2,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_F2,
        .pGaussFactor       = &OV2715_GaussFactor_F2,
        .pThreshold         = &OV2715_Threshold_F2,
        .pComponentGain     = &OV2715_CompGain_F2,

        .pSaturationCurve   = &OV2715_SaturationCurve_F2,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_F2,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_F2,

        .pVignettingCurve   = &OV2715_VignettingCurve_F2,
    },
    {
        .p_next             = NULL,

        .name               = "D50",
        .id                 = ISI_CIEPROF_D50,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,             /* from lightbox */
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_TRUE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_D50,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_D50,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_D50,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_D50,
        .pGaussFactor       = &OV2715_GaussFactor_D50,
        .pThreshold         = &OV2715_Threshold_D50,
        .pComponentGain     = &OV2715_CompGain_D50,

        .pSaturationCurve   = &OV2715_SaturationCurve_D50,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_D50,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_D50,

        .pVignettingCurve   = &OV2715_VignettingCurve_D50,
    },
    {
        .p_next             = NULL,

        .name               = "D65",
        .id                 = ISI_CIEPROF_D65,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_D65,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_D65,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_D65,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_D65,
        .pGaussFactor       = &OV2715_GaussFactor_D65,
        .pThreshold         = &OV2715_Threshold_D65,
        .pComponentGain     = &OV2715_CompGain_D65,

        .pSaturationCurve   = &OV2715_SaturationCurve_D65,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_D65,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_D65,

        .pVignettingCurve   = &OV2715_VignettingCurve_D65,
    },
    {
        .p_next             = NULL,

        .name               = "D75",
        .id                 = ISI_CIEPROF_D75,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_D75,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_D75,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_D75,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_D75,
        .pGaussFactor       = &OV2715_GaussFactor_D75,
        .pThreshold         = &OV2715_Threshold_D75,
        .pComponentGain     = &OV2715_CompGain_D75,

        .pSaturationCurve   = &OV2715_SaturationCurve_D75,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_D75,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_D75,

        .pVignettingCurve   = &OV2715_VignettingCurve_D75,
    },
    {
        .p_next             = NULL,

        .name               = "F11",
        .id                 = ISI_CIEPROF_F11,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV2715_XTalkCoeff_F11,
        .pCrossTalkOffset   = &OV2715_XTalkOffset_F11,

        .pGaussMeanValue    = &OV2715_GaussMeanValue_F11,
        .pCovarianceMatrix  = &OV2715_CovarianceMatrix_F11,
        .pGaussFactor       = &OV2715_GaussFactor_F11,
        .pThreshold         = &OV2715_Threshold_F11,
        .pComponentGain     = &OV2715_CompGain_F11,

        .pSaturationCurve   = &OV2715_SaturationCurve_F11,
        .pCcMatrixTable     = &OV2715_CcMatrixTable_F11,
        .pCcOffsetTable     = &OV2715_CcOffsetTable_F11,

        .pVignettingCurve   = &OV2715_VignettingCurve_F11,
    }
};




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV2715_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV2715_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV2715_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV2715_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV2715_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV2715_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV2715_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV2715_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV2715_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV2715_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV2715_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV2715_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV2715_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV2715_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV2715_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV2715_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV2715_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV2715_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );

static RESULT OV2715_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV2715_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV2715_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV2715_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV2715_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV2715_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV2715_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV2715_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV2715_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV2715_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );
static RESULT OV2715_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT OV2715_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t handle, char** pTuningXmlVersion);



static double floor( const double f )
{
    if ( f < 0 )
    {
        return ( (double)((int32_t)f - 1L) );
    }
    else
    {
        return ( (double)((uint32_t)f) );
    }
}


/*****************************************************************************/
/**
 *          OV2715_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV2715 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV2715_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t   *pConfig
)
{
    RESULT result = RET_SUCCESS;

    OV2715_Context_t *pOV2715Ctx;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV2715Ctx = ( OV2715_Context_t * )malloc ( sizeof (OV2715_Context_t) );
    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR,  "%s: Can't allocate ov2715 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV2715Ctx, 0, sizeof( OV2715_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV2715Ctx );
        return ( result );
    }

    pOV2715Ctx->IsiCtx.HalHandle           = pConfig->HalHandle;
    pOV2715Ctx->IsiCtx.HalDevID            = pConfig->HalDevID;
    pOV2715Ctx->IsiCtx.I2cBusNum           = pConfig->I2cBusNum;
    pOV2715Ctx->IsiCtx.SlaveAddress        = ( pConfig->SlaveAddr == 0 ) ? OV2715_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV2715Ctx->IsiCtx.NrOfAddressBytes    = 2U;

    pOV2715Ctx->IsiCtx.I2cAfBusNum         = pConfig->I2cAfBusNum;
    pOV2715Ctx->IsiCtx.SlaveAfAddress      = ( pConfig->SlaveAfAddr == 0U ) ? 0U : pConfig->SlaveAfAddr;
    pOV2715Ctx->IsiCtx.NrOfAfAddressBytes  = 0U;

    pOV2715Ctx->IsiCtx.pSensor             = pConfig->pSensor;

    pOV2715Ctx->GroupHold           = BOOL_FALSE;
    pOV2715Ctx->OldGain             = 0;
    pOV2715Ctx->OldIntegrationTime  = 0;

    pOV2715Ctx->Configured          = BOOL_FALSE;
    pOV2715Ctx->Streaming           = BOOL_FALSE;
    pOV2715Ctx->TestPattern         = BOOL_FALSE;
    pOV2715Ctx->isAfpsRun           = BOOL_FALSE;

    pConfig->hSensor = ( IsiSensorHandle_t )pOV2715Ctx;

    result = HalSetCamConfig( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, true, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, 10000000UL);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV2715 sensor instance.
 *
 * @param   handle      OV2715 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV2715_IsiReleaseSensorIss
(
    IsiSensorHandle_t   handle
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV2715_IsiSensorSetStreamingIss( pOV2715Ctx, BOOL_FALSE );
    (void)OV2715_IsiSensorSetPowerIss( pOV2715Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV2715Ctx->IsiCtx.HalHandle );

    MEMSET( pOV2715Ctx, 0, sizeof( OV2715_Context_t ) );
    free ( pOV2715Ctx );

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2715_IsiGetCapsIss
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
static RESULT OV2715_IsiGetCapsIss
(
    IsiSensorHandle_t   handle,
    IsiSensorCaps_t     *pIsiSensorCaps
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT_EX;
        pIsiSensorCaps->Mode            = ISI_MODE_BAYER;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_BGBGGRGR;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_POS;
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;

        pIsiSensorCaps->Resolution      = ( ISI_RES_TV1080P24   | ISI_RES_TV1080P20 | ISI_RES_TV1080P15
                                            | ISI_RES_TV1080P12 | ISI_RES_TV1080P6 );

        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF );
        pIsiSensorCaps->AGC             = ( ISI_AGC_AUTO | ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_AUTO | ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_AUTO | ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_OFF;
        pIsiSensorCaps->AfpsResolutions = ( ISI_RES_TV1080P24 | ISI_RES_TV1080P20 | ISI_RES_TV1080P15 );
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV2715_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT_EX,      // BusWidth
    ISI_MODE_BAYER,             // Mode
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_TV1080P24,          // Res
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_F11,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_OFF,               // MipiMode
    ( ISI_AFPS_NOTSUPP | ISI_RES_TV1080P24 | ISI_RES_TV1080P20 | ISI_RES_TV1080P15 ), // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          OV2715_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2715 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_SetupOutputFormat
(
    OV2715_Context_t        *pOV2715Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s%s (enter)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_10BIT_EX supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT_EX:
        {
            /* always 10 bit output, if CamerIc input has more than 10 bit,
             * MSBs will be copied to fill up LSBs */
            break;
        }

        default:
        {
            TRACE( OV2715_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* mode */
    switch ( pConfig->Mode )            /* only ISI_MODE_BAYER supported, no configuration needed here */
    {
        case ISI_MODE_BAYER:
        {
            break;
        }

        default:
        {
            TRACE( OV2715_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV2715 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV2715_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
        case ISI_VPOL_POS:
        {
            break;
        }

        default:
        {
            TRACE( OV2715_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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

        default:
        {
            TRACE( OV2715_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV2715_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch(  pConfig->MipiMode )        /* only ISI_MIPI_OFF supported, no configuration needed */
    {
        case ISI_MIPI_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV2715_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( OV2715_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV2715_INFO, "%s%s (exit)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2715 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_SetupOutputWindow
(
    OV2715_Context_t        *pOV2715Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint8_t  ucPreDiv = 0;
    uint8_t  ucDivp = 0;
    uint8_t  ucDivs = 0;

    uint16_t usYStart = 0;
    uint16_t usHSize  = 0;
    uint16_t usVSize  = 0;

    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
    float  rVtPixClkFreq      = 0.0f;

    uint8_t ucHVOffset    = 0;
    uint8_t ucSubSampling = 0;
    uint8_t ucBinningSkip = 0;

    uint16_t usB50Step = 0;
    uint16_t usB60Step = 0;

    uint8_t  ucB50Max = 0;
    uint8_t  ucB60Max = 0;

    uint16_t usAVGVertStart = 0;
    uint16_t usAVGVertEnd   = 0;

    uint8_t ucReg3811 = 0;
    uint8_t ucReg381C = 0;
    uint8_t ucReg381D = 0;
    uint8_t ucReg381E = 0;
    uint8_t ucReg381F = 0;
    uint8_t ucReg3820 = 0;
    uint8_t ucReg3821 = 0;
    uint8_t ucReg401C = 0;

    uint32_t ulRegValue = 0;

    TRACE( OV2715_INFO, "%s%s (enter)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    /* resolution */
    switch ( pConfig->Resolution )
    {
        case ISI_RES_VGA: //FIXME: check framerate => fix PLL & rVtPixClkFreq & flicker
        {
            TRACE( OV2715_INFO, "%s%s: Resolution VGA\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00; // 0x00 -> /1
            ucDivp   = 0x20;
            ucDivs   = 0x30;

            //frame
            usYStart  = 6;              //HREF vertical startpoint
            usHSize   = 640;            //HREF and DVP output horizontal width
            usVSize   = 480;            //HREF and DVP output vertical height
            ucHVOffset    = 0x08;       //horizontal and vertical offset
            ucSubSampling = 0x01;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0xC0;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1344;      //total horizontal size
            usFrameLengthLines = 496;       //total vertical size
            rVtPixClkFreq      = 10000000;  //video timing for AEC

            //flicker
            usB50Step = 0x24E6;
            usB60Step = 0x1EC0;
            ucB50Max  = 0x00;
            ucB60Max  = 0x01;

            //AVG
            usAVGVertStart = 640;
            usAVGVertEnd   = 480;

            //undocumented
            ucReg3811 = 0x02;
            ucReg381C = 0x10;
            ucReg381D = 0x42;
            ucReg381E = 0x03;
            ucReg381F = 0xC8;
            ucReg3820 = 0x0A;
            ucReg3821 = 0x29;
            ucReg401C = 0x04;

            break;
        }

        case ISI_RES_TV720P60: //FIXME: check framerate => fix PLL & rVtPixClkFreq & flicker
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV720P60\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00; // 0x00 -> /1
            ucDivp   = 0x20;
            ucDivs   = 0x10;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1280;           //HREF and DVP output horizontal width
            usVSize   = 720;            //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1792;      //total horizontal size
            usFrameLengthLines = 744;       //total vertical size
            rVtPixClkFreq      = 20000000;  //video timing for AEC, scan frequency of sensor array, derived from measurement above since calculation is unknown

            //flicker
            usB50Step = 0x1BE6;
            usB60Step = 0x1740;
            ucB50Max  = 0x01;
            ucB60Max  = 0x02;

            //AVG
            usAVGVertStart = 1280;
            usAVGVertEnd   = 720;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x10;
            ucReg381D = 0xB8;
            ucReg381E = 0x02;
            ucReg381F = 0xDC;
            ucReg3820 = 0x0A;
            ucReg3821 = 0x29;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV720P30: //FIXME: check framerate => fix PLL & rVtPixClkFreq & flicker
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV720P30\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x02; // 0x02 -> /2
            ucDivp   = 0x20;
            ucDivs   = 0x10;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1280;           //HREF and DVP output horizontal width
            usVSize   = 720;            //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1792;      //total horizontal size
            usFrameLengthLines = 744;       //total vertical size
            rVtPixClkFreq      = 20000000/2;  //video timing for AEC, scan frequency of sensor array, derived from measurement above since calculation is unknown

            //flicker
            usB50Step = 0x1BE6;
            usB60Step = 0x1740;
            ucB50Max  = 0x01;
            ucB60Max  = 0x02;

            //AVG
            usAVGVertStart = 1280;
            usAVGVertEnd   = 720;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x10;
            ucReg381D = 0xB8;
            ucReg381E = 0x02;
            ucReg381F = 0xDC;
            ucReg3820 = 0x0A;
            ucReg3821 = 0x29;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV720P15: //FIXME: check framerate => fix PLL & rVtPixClkFreq & flicker
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV720P15\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x02; // 0x02 -> /2
            ucDivp   = 0x20/2;
            ucDivs   = 0x10;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1280;           //HREF and DVP output horizontal width
            usVSize   = 720;            //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1792;      //total horizontal size
            usFrameLengthLines = 744;       //total vertical size
            rVtPixClkFreq      = 20000000/2/2;  //video timing for AEC, scan frequency of sensor array, derived from measurement above since calculation is unknown

            //flicker
            usB50Step = 0x1BE6;
            usB60Step = 0x1740;
            ucB50Max  = 0x01;
            ucB60Max  = 0x02;

            //AVG
            usAVGVertStart = 1280;
            usAVGVertEnd   = 720;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x10;
            ucReg381D = 0xB8;
            ucReg381E = 0x02;
            ucReg381F = 0xDC;
            ucReg3820 = 0x0A;
            ucReg3821 = 0x29;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV720P5: //FIXME: check framerate => fix PLL & rVtPixClkFreq & flicker
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV720P5\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x06; // 0x06 -> /6
            ucDivp   = 0x20/2;
            ucDivs   = 0x10;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1280;           //HREF and DVP output horizontal width
            usVSize   = 720;            //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1792;      //total horizontal size
            usFrameLengthLines = 744;       //total vertical size
            rVtPixClkFreq      = 20000000/2/6;  //video timing for AEC, scan frequency of sensor array, derived from measurement above since calculation is unknown

            //flicker
            usB50Step = 0x1BE6;
            usB60Step = 0x1740;
            ucB50Max  = 0x01;
            ucB60Max  = 0x02;

            //AVG
            usAVGVertStart = 1280;
            usAVGVertEnd   = 720;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x10;
            ucReg381D = 0xB8;
            ucReg381E = 0x02;
            ucReg381F = 0xDC;
            ucReg3820 = 0x0A;
            ucReg3821 = 0x29;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P24:
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P24\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00; // 0x00 -> /1
            ucDivp   = 0x33;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 2420;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 63750000;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P20:
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P20\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00;
            ucDivp   = 0x2A;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 2420;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 52500000;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P12:
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P12\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00; // 0x00 -> /1
            ucDivp   = 0x33/2;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 2420;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 63750000/2;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P6:
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P6\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x02; // 0x02 -> /2; // 0x05 -> /4 => broken hardware, results in /6 (like 0x06)
            ucDivp   = 0x33/2;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 2420;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 63750000/2/2;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P30: //TODO: check usLineLengthPck & rVtPixClkFreq for being correct
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P30\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x00; // 0x00 -> /1
            ucDivp   = 0x33;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1936;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 63750000;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P15:
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P15\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x01;
            ucDivp   = 0x30;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 2420;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 40000000;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        case ISI_RES_TV1080P5: //TODO: check usLineLengthPck & rVtPixClkFreq for being correct
        {
            TRACE( OV2715_INFO, "%s%s: Resolution TV1080P5\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

            //Clock
            ucPreDiv = 0x06; // 0x06 -> /6
            ucDivp   = 0x33;
            ucDivs   = 0x00;

            //frame
            usYStart  = 10;             //HREF vertical startpoint
            usHSize   = 1920;           //HREF and DVP output horizontal width
            usVSize   = 1080;           //HREF and DVP output vertical height
            ucHVOffset    = 0x10;       //horizontal and vertical offset
            ucSubSampling = 0x00;       //enable vertical subsample /2 for VGA (bit[1]: /4, bit[0]: /2)
            ucBinningSkip = 0x00;       //enable horiontal binning and horizontal skip for VGA (bit[7]: binning, bit[6]: skip)

            //the next three values are also handed over to the AEC module
            usLineLengthPck    = 1936;      //total horizontal size
            usFrameLengthLines = 1104;      //total vertical size
            rVtPixClkFreq      = 63750000/6;  //video timing for AEC

            //flicker
            usB50Step = 0x14C0;
            usB60Step = 0x1140;
            ucB50Max  = 0x03;
            ucB60Max  = 0x04;

            //AVG
            usAVGVertStart = 1952;
            usAVGVertEnd   = 1091;

            //undocumented
            ucReg3811 = 0x06;
            ucReg381C = 0x21;
            ucReg381D = 0x50;
            ucReg381E = 0x01;
            ucReg381F = 0x20;
            ucReg3820 = 0x00;
            ucReg3821 = 0x00;
            ucReg401C = 0x08;

            break;
        }

        default:
        {
            TRACE( OV2715_ERROR, "%s%s: Resolution not supported\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    TRACE( OV2715_DEBUG, "%s%s: Resolution %dx%d @ %f fps\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"", usHSize, usVSize, rVtPixClkFreq / ( usLineLengthPck * usFrameLengthLines ) );

    if (!pOV2715Ctx->isAfpsRun)
    {
        //PLL predivider
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_PLL_PREDIVIDER, &ulRegValue );
        ulRegValue &= ~0x07;
        ulRegValue |= ucPreDiv;
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_PLL_PREDIVIDER, ulRegValue );

        //PLL multiplier
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_PLL_CTRL02, ucDivp );
        //osSleep( 4 ); //wait to avoid any potential problems (not specified in any way)

        //system clock divider
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_PLL_CTRL01, &ulRegValue );
        ulRegValue &= 0x0F;
        ulRegValue |= ucDivs;
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_PLL_CTRL01, ulRegValue );

        //HREF vertical startpoint
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VH_HIGHBYTE0, (usYStart & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VH_LOWBYTE1, usYStart & 0x00FF );

        //HREF horizontal width
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_HW_HIGHBYTE, (usHSize & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_HW_LOWBYTE, usHSize & 0x00FF );

        //HREF vertical height
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VH_HIGHBYTE1, (usVSize & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VH_LOWBYTE2, usVSize & 0x00FF );

        //DVP output horizontal width
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_DVP_HSIZE, (usHSize & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_DVP_HSIZELOW, usHSize & 0x00FF );

        //DVP output vertical height
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_DVP_VSIZEHIGH, (usVSize & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_DVP_VSIZELOW, usVSize & 0x00FF );

        //total vertical size
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VTS_HIGHBYTE, (usFrameLengthLines & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_VTS_LOWBYTE, usFrameLengthLines & 0x00FF );

        //total horizontal size
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_HTS_HIGHBYTE, (usLineLengthPck & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_HTS_LOWBYTE, usLineLengthPck & 0x00FF );

        //horizontal and vertical offset
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL_HV_OFFSET, ucHVOffset );

        //subsampling
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_TIMING_CONTROL18, &ulRegValue );
        ulRegValue &= 0xFC;
        ulRegValue |= ucSubSampling;
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_TIMING_CONTROL18, ulRegValue );

        //horizontal binning and skip
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_ANA_ARRAY_01, &ulRegValue );
        ulRegValue &= 0x3F;
        ulRegValue |= ucBinningSkip;
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ANA_ARRAY_01, ulRegValue );

        //AEC flicker step width for 50Hz and 60Hz
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_B50_STEP0, (usB50Step & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_B50_STEP1, usB50Step & 0x00FF );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_B60_STEP0, (usB60Step & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_B60_STEP1, usB60Step & 0x00FF );

        //AEC flicker max for 50Hz and 60 Hz
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_CONTROLE, ucB50Max );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_CONTROLD, ucB60Max );

        //AVG vertical start and end position
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AVG_START_POSITION_AT_VERTICAL0, (usAVGVertStart & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AVG_START_POSITION_AT_VERTICAL1, usAVGVertStart & 0x00FF );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AVG_END_POSITION_AT_VERTICAL0, (usAVGVertEnd & 0xFF00) >> 8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AVG_END_POSITION_AT_VERTICAL1, usAVGVertEnd & 0x00FF );

        //keep testpattern mode
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_ISP_TEST, &ulRegValue );
        ulRegValue &= ~0x80;
        ulRegValue |= ( BOOL_TRUE == pOV2715Ctx->TestPattern ) ? 0x80 : 0x00;
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ISP_TEST, ulRegValue );

        //undocumented registers
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3811, ucReg3811 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_381C, ucReg381C );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_381D, ucReg381D );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_381E, ucReg381E );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_381F, ucReg381F );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3820, ucReg3820 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3821, ucReg3821 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_401C, ucReg401C );
    }

    //store frame timing for later use in AEC module
    pOV2715Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV2715Ctx->LineLengthPck    = usLineLengthPck;
    pOV2715Ctx->FrameLengthLines = usFrameLengthLines - 3; // three lines shorter than image to avoid dark frames

    TRACE( OV2715_INFO, "%s%s (exit)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV2715 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_SetupImageControl
(
    OV2715_Context_t        *pOV2715Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2715_INFO, "%s%s (enter)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    if (!pOV2715Ctx->isAfpsRun)
    {
        switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
        {
            case ISI_BLS_OFF:
            {
                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }

        /* black level compensation */
        switch ( pConfig->BLC )
        {
            case ISI_BLC_OFF:
            {
                /* turn off black level correction (clear bit 0) */
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_BLC_CONTROL_00, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_BLC_CONTROL_00, RegValue & 0xFE );
                break;
            }

            case ISI_BLC_AUTO:
            {
                /* turn on black level correction (set bit 0)
                 * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_BLC_CONTROL_00, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_BLC_CONTROL_00, RegValue | 0x01 );
                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }

        /* automatic gain control */
        switch ( pConfig->AGC )
        {
            case ISI_AGC_OFF:
            {
                // manual gain (appropriate for AEC with Marvin)
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_AEC_PK_MANUAL, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_MANUAL, RegValue | 0x02 );
                break;
            }

            case ISI_AGC_AUTO:
            {
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_AEC_PK_MANUAL, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_MANUAL, RegValue & ~0x02 );
                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }

        /* automatic white balance */
        switch( pConfig->AWB )
        {
            case ISI_AWB_OFF:
            {
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_ISP_CONTROL1, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ISP_CONTROL1, RegValue & ~0x01 );
                break;
            }

            case ISI_AWB_AUTO:
            {
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_ISP_CONTROL1, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ISP_CONTROL1, RegValue | 0x01 );
                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }

        switch( pConfig->AEC )
        {
            case ISI_AEC_OFF:
            {
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_AEC_PK_MANUAL, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_MANUAL, (RegValue & ~0x04) | 0x01 );  // AEC off but VTS on; we face problems otherwise (occasionl dark frames, reduced frame rate)
                break;
            }

            case ISI_AEC_AUTO:
            {
                /* settings derived from eva kit (note that gain is controlled by ISI_AGC_AUTO/OFF above)
                 *
                 * Also note that the AEC needs parameters (target for exposure, WPT/BPT/VPT, see
                 * datasheet). Static default settings are for 5MP. It is unknown if parameters are
                 * resolution/frametiming dependent. Parameters are not dynamically adapted. */
                result = OV2715_IsiRegReadIss(  pOV2715Ctx, OV2715_AEC_PK_MANUAL, &RegValue );
                result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_MANUAL, RegValue & ~0x05 ); // VTS on, AEC on
                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }


    /*    switch( pConfig->DPCC )
        {
            case ISI_DPCC_OFF:
            {
                // disable white and black pixel cancellation (clear bit 6 and 7)
                result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
                RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
                result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue & 0x3F) );
                RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
                break;
            }

            case ISI_DPCC_AUTO:
            {
                // enable white and black pixel cancellation (set bit 6 and 7)
                result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AUTO1, &RegValue );
                RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
                result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AUTO1, (RegValue | 0xC0U) );
                RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
                break;
            }

             default:
            {
                TRACE( OV5630_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
                return ( RET_NOTSUPP );
            }
        }*/// I have not update this commented part yet, as I did not find DPCC setting in the current 2715 driver of Trillian board. - SRJ
    }

    TRACE( OV2715_INFO, "%s%s (exit)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV2715-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV2715 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_AecSetModeParameters
(
    OV2715_Context_t        *pOV2715Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s%s: (enter)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    if ( pOV2715Ctx->VtPixClkFreq == 0.0f )
    {
        TRACE( OV2715_ERROR, "%s%s: OV2715_AecSetModeParameters: VtPixClkFreq=0.0f (Division by zero !!)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );
        return ( RET_OUTOFRANGE );
    }

    // there are no limits defined in the datasheet, so we assume the max. frame heigth/width is applicable
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )

    pOV2715Ctx->AecMinIntegrationTime = 0.0;
    pOV2715Ctx->AecMaxIntegrationTime = ( ((float)pOV2715Ctx->FrameLengthLines)
                                        * ((float)pOV2715Ctx->LineLengthPck) )
                                        / pOV2715Ctx->VtPixClkFreq;

    TRACE( OV2715_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"", pOV2715Ctx->AecMaxIntegrationTime );

    pOV2715Ctx->AecMinGain = 1;
    pOV2715Ctx->AecMaxGain = OV2715_MAX_GAIN_AEC;

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pOV2715Ctx->AecIntegrationTimeIncrement = (float)pOV2715Ctx->LineLengthPck / (float)pOV2715Ctx->VtPixClkFreq;
    pOV2715Ctx->AecGainIncrement = OV2715_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pOV2715Ctx->AecCurGain            = pOV2715Ctx->AecMinGain;
    pOV2715Ctx->AecCurIntegrationTime = 0.0f;
    pOV2715Ctx->OldGain               = 0;
    pOV2715Ctx->OldIntegrationTime    = 0;

    TRACE( OV2715_INFO, "%s%s: (exit)\n", __FUNCTION__, pOV2715Ctx->isAfpsRun?"(AFPS)":"" );

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV2715 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV2715Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV2715Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV2715_IsiRegReadIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, &RegValue);
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: Can't read OV2715 System Register\n", __FUNCTION__ );
        return ( result );
    }

    result = OV2715_IsiRegWriteIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, (RegValue| 0x80) );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: Can't write OV2715 System Register (sensor reset failed)\n", __FUNCTION__ );
        return ( result );
    }

    TRACE( OV2715_DEBUG, "%s: OV2715 System-Reset executed\n", __FUNCTION__);

    result = OV2715_IsiRegReadIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, &RegValue);
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: Can't read OV2715 System Register\n", __FUNCTION__ );
        return ( result );
    }

    result = OV2715_IsiRegWriteIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, RegValue |0x40);
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: Can't remove OV2715 System Reset (sensor reset failed)\n", __FUNCTION__ );
        return ( result );
    }

    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    result = IsiRegDefaultsApply( pOV2715Ctx, OV2715_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 100 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
    // it seems that OV2715 is not eating the value for AEC_AGC_ADJ Register: 0x350b
    // to avoid check conflicts in Application => I commented the result-check
    // ( AEC_AGC_ADJ @ 0x350b is 0x00000073 instead of 0x00000000 )
    result = IsiRegDefaultsVerify( pOV2715Ctx, OV2715_g_aRegDescription );
    //if ( result != RET_SUCCESS )
    //{
    //    return ( result );
    //}

    result = OV2715_SetupOutputFormat( pOV2715Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV2715_SetupOutputWindow( pOV2715Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV2715_SetupImageControl( pOV2715Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV2715_AecSetModeParameters( pOV2715Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    // define group 0
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_GROUP_ACCESS, 0x03 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3600, (OV2715_AEC_PK_EXPO0 & 0xFF00) >> 8 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3601, (OV2715_AEC_PK_EXPO0 & 0x00FF) >> 0 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3600, (OV2715_AEC_PK_EXPO1 & 0xFF00) >> 8 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3601, (OV2715_AEC_PK_EXPO1 & 0x00FF) >> 0 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3600, (OV2715_AEC_PK_EXPO2 & 0xFF00) >> 8 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3601, (OV2715_AEC_PK_EXPO2 & 0x00FF) >> 0 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3600, (OV2715_AEC_AGC_ADJ0 & 0xFF00) >> 8 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3601, (OV2715_AEC_AGC_ADJ0 & 0x00FF) >> 0 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3600, (OV2715_AEC_AGC_ADJ1 & 0xFF00) >> 8 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_3601, (OV2715_AEC_AGC_ADJ1 & 0x00FF) >> 0 ) );
    UPDATE_RESULT( result, OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_GROUP_ACCESS, 0x13 ) );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: Defining group 0 failed.\n", __FUNCTION__);
        return ( result );
    }

    if (result == RET_SUCCESS)
    {
        pOV2715Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiChangeSensorResolutionIss
 *
 * @brief   Change image sensor resolution while keeping all other static settings.
 *          Dynamic settings like current gain & integration time are kept as
 *          close as possible.
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
static RESULT OV2715_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip

)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV2715Ctx->Configured != BOOL_TRUE) || (pOV2715Ctx->Streaming != BOOL_FALSE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    result = OV2715_IsiGetCapsIss( handle, &Caps);
    if (RET_SUCCESS != result)
    {
        return result;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV2715Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV2715_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pOV2715Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV2715_SetupOutputWindow( pOV2715Ctx, &pOV2715Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2715_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV2715Ctx->AecCurGain;
        float OldIntegrationTime = pOV2715Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV2715_AecSetModeParameters( pOV2715Ctx, &pOV2715Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2715_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV2715_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV2715_ERROR, "%s: OV2715_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
    }

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiSensorSetStreamingIss
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
static RESULT OV2715_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV2715Ctx->Configured != BOOL_TRUE) || (pOV2715Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        TRACE( OV2715_DEBUG, "%s: ON\n", __FUNCTION__);
        result = OV2715_IsiRegReadIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV2715_IsiRegWriteIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, (RegValue & ~0x40U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        TRACE( OV2715_DEBUG, "%s: OFF\n", __FUNCTION__);
        result = OV2715_IsiRegReadIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV2715_IsiRegWriteIss ( pOV2715Ctx, OV2715_SYSTEM_CONTROL00, (RegValue | 0x40U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV2715Ctx->Streaming = on;
    }

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV2715 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV2715Ctx->Configured = BOOL_FALSE;
    pOV2715Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV2715_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV2715_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        TRACE( OV2715_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2715_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2715_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV2715_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV2715Ctx->IsiCtx.HalHandle, pOV2715Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV2715 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV2715_PIDH_DEFAULT;
    RevId = (RevId << 8U) | OV2715_PIDL_DEFAULT;

    result = OV2715_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV2715_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        //for test ,zyc
        return ( RET_FAILURE );
    }

    TRACE( OV2715_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetSensorRevisionIss
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
static RESULT OV2715_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV2715_IsiRegReadIss ( handle, OV2715_PIDH, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = OV2715_IsiRegReadIss ( handle, OV2715_PIDL, &data );
    *p_value |= ( data & 0xFF );

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2715_IsiRegReadIss
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
static RESULT OV2715_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2715_REG_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV2715_g_aRegDescription );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
        TRACE( OV2715_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x)\n", __FUNCTION__, NrOfBytes, address);

        *p_value = 0U;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

    TRACE( OV2715_REG_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiRegWriteIss
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
static RESULT OV2715_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    TRACE( OV2715_REG_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV2715_g_aRegDescription );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
    TRACE( OV2715_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x 0x%08x)\n", __FUNCTION__, NrOfBytes, address, value);

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

    TRACE( OV2715_REG_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV2715 instance
 *
 * @param   handle       OV2715 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV2715_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = 1.0f;
    *pMaxGain = pOV2715Ctx->AecMaxGain;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV2715 instance
 *
 * @param   handle       OV2715 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV2715_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = 0.0f;
    *pMaxIntegrationTime = (float)(pOV2715Ctx->FrameLengthLines * pOV2715Ctx->LineLengthPck) / pOV2715Ctx->VtPixClkFreq;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2715_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pOV2715Ctx->AecCurGain;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV2715Ctx->AecGainIncrement;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    float    Gain                   = 0.0f;
    uint8_t  ucMultiplier           = 0U;
    uint8_t  ucGain                 = 0U;
    uint8_t  ucGainAdj1             = 0U;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum gain is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if( NewGain < pOV2715Ctx->AecMinGain ) NewGain = pOV2715Ctx->AecMinGain;
    if( NewGain > pOV2715Ctx->AecMaxGain ) NewGain = pOV2715Ctx->AecMaxGain;

    //Calculate and set new gain register settings.
    //See mail from Omnivision FAE, analog gain = (Bit[6]+1)*(Bit[5]+1)*(Bit[4]+1)*(Bit[3:0]/16+1).
    //Lower multiplier bits have to be set before higher bits are set.
    //
    //Boundary check should never hit, maybe because of limited floating point precision.
    //If the new gain is >= 8, we have to set all multipliers since we can't reach 8 otherwise and so on.

    Gain = NewGain;

    if ( Gain < 2.0f )
    {
        ucMultiplier = 0U;                          // 2*0 = x1
    }
    else if ( Gain < 4.0f )
    {
        ucMultiplier = 0x10;                        // 2*1 = x2
        Gain /= 2.0f;
    }
    else if ( Gain < 8.0f )
    {
        ucMultiplier = 0x20 + 0x10;                 // 2*2 = x4
        Gain /= 4.0f;
    }
    else if ( Gain < 16.0f )
    {
        ucMultiplier = 0x40 + 0x20 + 0x10;          // 2*2*2 = x8
        Gain /= 8.0f;
    }
    else
    {
        ucMultiplier = 0x80 + 0x40 + 0x20 + 0x10;   // 2*2*2*2 = x16
        Gain /= 16.0f;
    }

    Gain  = 16 * (Gain - 1.0f) + 0.5f;

    ucGain = (uint8_t)Gain;
    if ( ucGain > 0x0F )
    {
        TRACE( OV2715_DEBUG, "%s: ucGain too big (%d), limiting to max (15).\n", __FUNCTION__, ucGain );
        ucGain = 0x0F; //avoid overflow due to limited floating point precision
    }

    ucGainAdj1 = ucMultiplier | ucGain;

    //write new gain into sensor registers
    //do not write if nothing has changed
    if ( ucGainAdj1 != pOV2715Ctx->OldGain )
    {
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_AGC_ADJ0, 0 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_AGC_ADJ1, ucGainAdj1 );

        if (!pOV2715Ctx->GroupHold)
        {
            result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_GROUP_ACCESS, 0xA3 ); // write group 0
        }

        TRACE( OV2715_DEBUG, "%s: set AEC_AGC_ADJ=0x%03x\n", __FUNCTION__, ucGainAdj1 );

        pOV2715Ctx->OldGain = ucGainAdj1;   //remember current gain
    }

    //Actual hardware gain may deviate by 1/16, 2/16, ... , 8/16 from NewGain.
    //Currently OV2715_MAX_GAIN_AEC is 16.0f, so max. deviation is 8/16.

    //calculate gain actually set
    pOV2715Ctx->AecCurGain = ((float)((ucGain + 16) * ((ucMultiplier>>4)+1))) / 16.0;

    //return current state
    *pSetGain = pOV2715Ctx->AecCurGain;
    TRACE( OV2715_DEBUG, "%s: g=%f\n", __FUNCTION__, *pSetGain );

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV2715Ctx->AecCurIntegrationTime;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV2715Ctx->AecIntegrationTimeIncrement;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   NewIntegrationTime      integration time to be set
 * @param   pSetIntegrationTime     set integration time
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t IntegrationTime = 0UL;

    float    ShutterWidthPck = 0.0f;  //shutter width in pixel clock periods

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( OV2715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pOV2715Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV2715Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pOV2715Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV2715Ctx->AecMinIntegrationTime;

    // calculate and set new coarse and fine integration time register settings

    /**
     * The actual integration time is given by:
     * integration_time = ((coarse_integration_time*line_length_pck)+fine_integration_time)/(vt_pix_clk_freq)
     *
     * that leads to:
     * coarse_integration_time = (UINT32)((integration_time*vt_pix_clk_freq)/line_length_pck)
     * fine_integration_time   = (UINT32)((integration_time*vt_pix_clk_freq)-(coarse_integration_time*line_length_pck))
     */

    ShutterWidthPck = NewIntegrationTime * pOV2715Ctx->VtPixClkFreq;

    //calculate the integer part of the integration time in units of line length
    //avoid division by zero
    if ( pOV2715Ctx->LineLengthPck == 0U )
    {
        TRACE( OV2715_ERROR, "%s: LineLengthPck=0U (Division by zero !!!)\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the fractional part of the integration time in units of pixel clocks

    IntegrationTime = (uint32_t)(ShutterWidthPck * 16.0 / (pOV2715Ctx->LineLengthPck));
    uint32_t IntegrationTimeLimit = 0;
    IntegrationTimeLimit = MIN( ((uint32_t)(pOV2715Ctx->FrameLengthLines))<<4, 0xfffffUL );
    if (IntegrationTime > IntegrationTimeLimit)
    {
        TRACE( OV2715_DEBUG, "%s: Integration time (%d) too big, limiting to max (%d).\n", __FUNCTION__, IntegrationTime, IntegrationTimeLimit );
        IntegrationTime = IntegrationTimeLimit;
    }

    //write new integration time settings into the camera registers
    //do not write if nothing has changed
    if ( IntegrationTime != pOV2715Ctx->OldIntegrationTime )
    {
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_EXPO0, (IntegrationTime & 0xf0000) >> 16 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_EXPO1, (IntegrationTime & 0x0ff00) >>  8 );
        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_AEC_PK_EXPO2, (IntegrationTime & 0x000ff) );

        if (!pOV2715Ctx->GroupHold)
        {
            result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_GROUP_ACCESS, 0xA3 ); // write group 0
        }

        TRACE( OV2715_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __FUNCTION__, IntegrationTime );

        pOV2715Ctx->OldIntegrationTime = IntegrationTime;   //remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //calculate integration time actually set
    pOV2715Ctx->AecCurIntegrationTime = ( (float)IntegrationTime * (float)pOV2715Ctx->LineLengthPck ) / ( pOV2715Ctx->VtPixClkFreq ) / 16.0f;

    //return current state
    *pSetIntegrationTime = pOV2715Ctx->AecCurIntegrationTime;

    TRACE( OV2715_DEBUG, "%s: Ti=%f\n", __FUNCTION__, *pSetIntegrationTime );
    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV2715 sensor instance handle
 * @param   NewGain                 newly calculated gain to be set
 * @param   NewIntegrationTime      newly calculated integration time to be set
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV2715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV2715_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );

    pOV2715Ctx->GroupHold = BOOL_TRUE;
    result = OV2715_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = OV2715_IsiSetGainIss( handle, NewGain, pSetGain );
    pOV2715Ctx->GroupHold = BOOL_FALSE;
    result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_GROUP_ACCESS, 0xA3 ); // write group 0

    TRACE( OV2715_DEBUG, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV2715 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV2715Ctx->AecCurGain;
    *pSetIntegrationTime = pOV2715Ctx->AecCurIntegrationTime;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetResolutionIss
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
RESULT OV2715_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV2715Ctx->Config.Resolution;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV2715Ctx             OV2715 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT OV2715_IsiGetAfpsInfoHelperIss(
    OV2715_Context_t   *pOV2715Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV2715Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV2715Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV2715_SetupOutputWindow( pOV2715Ctx, &pOV2715Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV2715_AecSetModeParameters( pOV2715Ctx, &pOV2715Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV2715_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV2715Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV2715Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV2715Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV2715Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV2715Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2715_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV2715 sensor instance handle
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
RESULT OV2715_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        TRACE( OV2715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV2715Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pOV2715Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pOV2715Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pOV2715Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    OV2715_Context_t *pDummyCtx = (OV2715_Context_t*) malloc( sizeof(OV2715_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( OV2715_ERROR,  "%s: Can't allocate dummy ov2715 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pOV2715Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    if ( (pOV2715Ctx->Config.AfpsResolutions & (_res_)) != 0 ) \
    { \
        RESULT lres = OV2715_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
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
            TRACE( OV2715_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
            result = RET_NOTSUPP;
            break;

        // 1080p24 series in ascending integration time order (most probably the same as descending frame rate order)
        case ISI_RES_TV1080P24:
        case ISI_RES_TV1080P20:
        case ISI_RES_TV1080P15:
            AFPSCHECKANDADD( ISI_RES_TV1080P24 );
            AFPSCHECKANDADD( ISI_RES_TV1080P20 );
            AFPSCHECKANDADD( ISI_RES_TV1080P15  );
            break;

        // 720p60 series in ascending integration time order (most probably the same as descending frame rate order)
        case ISI_RES_TV720P60:
        case ISI_RES_TV720P30:
        case ISI_RES_TV720P15:
        case ISI_RES_TV720P5 :
            AFPSCHECKANDADD( ISI_RES_TV720P60 );
            AFPSCHECKANDADD( ISI_RES_TV720P30 );
            AFPSCHECKANDADD( ISI_RES_TV720P15 );
            AFPSCHECKANDADD( ISI_RES_TV720P5  );
            break;

        // check next series here...
    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibKFactor
 *
 * @brief   Returns the OV2715 specific K-Factor
 *
 * @param   handle       OV2715 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV2715_KFactor;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV2715 specific PCA-Matrix
 *
 * @param   handle          OV2715 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV2715_PCAMatrix;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV2715 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV2715_SVDMeanValue;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV2715 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiCenterLine = (IsiLine_t*)&OV2715_CenterLine;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV2715 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiClipParam = (IsiAwbClipParm_t *)&OV2715_AwbClipParm;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV2715 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV2715_AwbGlobalFadeParm;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV2715 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV2715_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV2715_AwbFade2Parm;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV2715_IsiGetIlluProfile
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
static RESULT OV2715_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
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
        for ( i=0U; i<OV2715_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV2715_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV2715_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiGetLscMatrixTable
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
static RESULT OV2715_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
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
                if ( (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P24)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P20)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P12)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P6)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P30)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P15)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P5 ) )
                {
                    *pLscMatrixTable = &OV2715_LscMatrixTable_CIE_A_1920x1080;
                }
                else
                {
                    TRACE( OV2715_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P24)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P20)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P12)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P6)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P30)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P15)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P5 ) )
                {
                    *pLscMatrixTable = &OV2715_LscMatrixTable_CIE_F2_1920x1080;
                }
                else
                {
                    TRACE( OV2715_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
             }

            case ISI_CIEPROF_D50:
            {
                if ( (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P24)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P20)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P12)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P6)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P30)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P15)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P5 ) )
                {
                    *pLscMatrixTable = &OV2715_LscMatrixTable_CIE_D50_1920x1080;
                }
                else
                {
                    TRACE( OV2715_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
             }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P24)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P20)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P12)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P6)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P30)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P15)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P5 ) )
                {
                    *pLscMatrixTable = &OV2715_LscMatrixTable_CIE_D65_1920x1080;
                }
                else
                {
                    TRACE( OV2715_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
             }

            case ISI_CIEPROF_F11:
            {
                if ( (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P24)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P20)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P12)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P6)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P30)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P15)
                  || (pOV2715Ctx->Config.Resolution == ISI_RES_TV1080P5 ) )
                {
                    *pLscMatrixTable = &OV2715_LscMatrixTable_CIE_F11_1920x1080;
                }
                else
                {
                    TRACE( OV2715_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV2715_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV2715_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV2715 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV2715_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t ulRegValue = 0UL;

    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_ISP_TEST, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ISP_TEST, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV2715_IsiRegReadIss( pOV2715Ctx, OV2715_ISP_TEST, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV2715_IsiRegWriteIss( pOV2715Ctx, OV2715_ISP_TEST, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    pOV2715Ctx->TestPattern = enable;

    TRACE( OV2715_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV2715_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
    	TRACE( OV2715_ERROR, "%s: pOV2715Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV2715_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV2715_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV2715_Context_t *pOV2715Ctx = (OV2715_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV2715_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV2715Ctx == NULL )
    {
    	TRACE( OV2715_ERROR, "%s: pOV2715Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV2715_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV2715_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *
 */
/*****************************************************************************/



/*****************************************************************************/
/**
 *          OV2715_IsiGetSensorIss
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
RESULT OV2715_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV2715_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                         = OV2715_g_acName;
        pIsiSensor->pRegisterTable                  = OV2715_g_aRegDescription;
        pIsiSensor->pIsiSensorCaps                  = &OV2715_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer				= OV2715_IsiGetSensorIsiVersion; //oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion	= OV2715_IsiGetSensorTuningXmlVersion;//oyyf
		
        pIsiSensor->pIsiCreateSensorIss             = OV2715_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss            = OV2715_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                  = OV2715_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss              = OV2715_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss   = OV2715_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss       = OV2715_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss           = OV2715_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss    = OV2715_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss        = OV2715_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss             = OV2715_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss            = OV2715_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss          = OV2715_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss            = OV2715_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss = OV2715_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss       = OV2715_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV2715_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV2715_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV2715_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV2715_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV2715_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV2715_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV2715_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV2715_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor             = OV2715_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix           = OV2715_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue        = OV2715_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine          = OV2715_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam           = OV2715_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam     = OV2715_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam           = OV2715_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile              = OV2715_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable           = OV2715_IsiGetLscMatrixTable;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern         = OV2715_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV2715_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV2715_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV2715_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV2715_SLAVE_ADDR;
    pSensorI2cInfo->soft_reg_addr = OV2715_SYSTEM_CONTROL00;
    pSensorI2cInfo->soft_reg_value = 0x80;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    pSensorI2cInfo->resolution = ( ISI_RES_TV1080P24   | ISI_RES_TV1080P20 | ISI_RES_TV1080P15
                                | ISI_RES_TV1080P12 | ISI_RES_TV1080P6 );
    
    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = OV2715_PIDH;  
    pChipIDInfo_H->chipid_reg_value = OV2715_PIDH_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV2715_PIDL;
    pChipIDInfo_L->chipid_reg_value = OV2715_PIDL_DEFAULT;
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
    OV2715_IsiGetSensorIss,
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
        0,						/**< IsiSensor_t.pIsiGetColorIss */  
    },
    OV2715_IsiGetSensorI2cInfo,
};

