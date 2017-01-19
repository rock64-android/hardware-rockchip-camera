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
 * @file OV8810.c
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

#include "OV8810_priv.h"

#define CC_OFFSET_SCALING  2.0f

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV8810_INFO , "OV8810: ", INFO,    0);
CREATE_TRACER( OV8810_WARN , "OV8810: ", WARNING, 1);
CREATE_TRACER( OV8810_ERROR, "OV8810: ", ERROR,   1);

CREATE_TRACER( OV8810_DEBUG, "",         INFO,    0);

#define OV8810_SLAVE_ADDR       0x36                            /**< i2c slave address of the OV8810 camera sensor */
#define OV8810_SLAVE_AF_ADDR    0x0C

#define STEPS_IN_GAIN_NUMBER    ( 0 )       /**< Number of max steps_in_gain */
#define OV8810_MAX_GAIN_AEC     ( 8.0f )    /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */

#define MAX_LOG   64
#define MAX_REG 1023
#define MDI_SLEW_RATE_CTRL 2 /* S3..0 */


/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV8810_g_acName[] = "OV8810";
extern const IsiRegDescription_t OV8810_g_aRegDescription[];
const IsiSensorCaps_t OV8810_g_IsiSensorDefaultConfig;

// The sensor design may not allow to alter integration time from frame to frame
// (for example the classic rolling shutter). So we remember the old integration
// time to figure out if we are changing it and to tell the upper layers how much
// frames they have to wait before executing the AE again.
// static uint32_t OldIntegrationTime = 0UL;

/* AWB specific value (from OV8810_tables.c) */
extern const Isi1x1FloatMatrix_t    OV8810_KFactor;
extern const Isi3x2FloatMatrix_t    OV8810_PCAMatrix;
extern const Isi3x1FloatMatrix_t    OV8810_SVDMeanValue;
extern const IsiLine_t              OV8810_CenterLine;
extern const IsiAwbClipParm_t       OV8810_AwbClipParm;
extern const IsiAwbGlobalFadeParm_t OV8810_AwbGlobalFadeParm;
extern const IsiAwbFade2Parm_t      OV8810_AwbFade2Parm;

/* illumination profiles */
#include "OV8810_a.h"       /* CIE A - default profile */
#include "OV8810_f2.h"      /* CIE F2 (cool white flourescent CWF) */
#include "OV8810_d50.h"     /* CIE D50 (D50 lightbox) */
#include "OV8810_d65.h"     /* CIE D65 (D65) note: indoor because of our lightbox */
#include "OV8810_d75.h"     /* CIE D75 (D75) overcast daylight, 7500K */
#include "OV8810_f11.h"     /* CIE F11 (TL84) */

#define OV8810_ISIILLUPROFILES_DEFAULT  6
static IsiIlluProfile_t OV8810_IlluProfileDefault[OV8810_ISIILLUPROFILES_DEFAULT] =
{
    {
        .p_next             = NULL,

        .name               = "A",
        .id                 = ISI_CIEPROF_A,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_CIE_A,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_CIE_A,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_CIE_A,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_CIE_A,
        .pGaussFactor       = &OV8810_GaussFactor_CIE_A,
        .pThreshold         = &OV8810_Threshold_CIE_A,
        .pComponentGain     = &OV8810_CompGain_CIE_A,

        .pSaturationCurve   = &OV8810_SaturationCurve_CIE_A,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_CIE_A,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_CIE_A,

        .pVignettingCurve   = &OV8810_VignettingCurve_CIE_A,
    },
    {
        .p_next             = NULL,

        .name               = "F2",
        .id                 = ISI_CIEPROF_F2,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_F2,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_F2,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_F2,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_F2,
        .pGaussFactor       = &OV8810_GaussFactor_F2,
        .pThreshold         = &OV8810_Threshold_F2,
        .pComponentGain     = &OV8810_CompGain_F2,

        .pSaturationCurve   = &OV8810_SaturationCurve_F2,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_F2,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_F2,

        .pVignettingCurve   = &OV8810_VignettingCurve_F2,
    },
    {
        .p_next             = NULL,

        .name               = "D50",
        .id                 = ISI_CIEPROF_D50,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,             /* from lightbox */
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_TRUE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_D50,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_D50,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_D50,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_D50,
        .pGaussFactor       = &OV8810_GaussFactor_D50,
        .pThreshold         = &OV8810_Threshold_D50,
        .pComponentGain     = &OV8810_CompGain_D50,

        .pSaturationCurve   = &OV8810_SaturationCurve_D50,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_D50,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_D50,

        .pVignettingCurve   = &OV8810_VignettingCurve_D50,
    },
    {
        .p_next             = NULL,

        .name               = "D65",
        .id                 = ISI_CIEPROF_D65,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_D65,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_D65,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_D65,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_D65,
        .pGaussFactor       = &OV8810_GaussFactor_D65,
        .pThreshold         = &OV8810_Threshold_D65,
        .pComponentGain     = &OV8810_CompGain_D65,

        .pSaturationCurve   = &OV8810_SaturationCurve_D65,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_D65,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_D65,

        .pVignettingCurve   = &OV8810_VignettingCurve_D65,
    },
    {
        .p_next             = NULL,

        .name               = "D75",
        .id                 = ISI_CIEPROF_D75,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_D75,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_D75,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_D75,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_D75,
        .pGaussFactor       = &OV8810_GaussFactor_D75,
        .pThreshold         = &OV8810_Threshold_D75,
        .pComponentGain     = &OV8810_CompGain_D75,

        .pSaturationCurve   = &OV8810_SaturationCurve_D75,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_D75,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_D75,

        .pVignettingCurve   = &OV8810_VignettingCurve_D75,
    },
    {
        .p_next             = NULL,

        .name               = "F11",
        .id                 = ISI_CIEPROF_F11,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV8810_XTalkCoeff_F11,
        .pCrossTalkOffset   = &OV8810_XTalkOffset_F11,

        .pGaussMeanValue    = &OV8810_GaussMeanValue_F11,
        .pCovarianceMatrix  = &OV8810_CovarianceMatrix_F11,
        .pGaussFactor       = &OV8810_GaussFactor_F11,
        .pThreshold         = &OV8810_Threshold_F11,
        .pComponentGain     = &OV8810_CompGain_F11,

        .pSaturationCurve   = &OV8810_SaturationCurve_F11,
        .pCcMatrixTable     = &OV8810_CcMatrixTable_F11,
        .pCcOffsetTable     = &OV8810_CcOffsetTable_F11,

        .pVignettingCurve   = &OV8810_VignettingCurve_F11,
    }
};




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV8810_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV8810_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV8810_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV8810_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV8810_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV8810_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV8810_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV8810_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV8810_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV8810_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV8810_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float newIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );

static RESULT OV8810_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV8810_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV8810_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV8810_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV8810_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV8810_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV8810_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV8810_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV8810_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV8810_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV8810_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV8810_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV8810_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV8810_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV8810_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );
static RESULT OV8810_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);


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
 *          OV8810_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV8810 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV8810_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t   *pConfig
)
{
    RESULT result = RET_SUCCESS;

    OV8810_Context_t *pOV8810Ctx;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV8810Ctx = ( OV8810_Context_t * )malloc ( sizeof (OV8810_Context_t) );
    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR,  "%s: Can't allocate ov8810 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV8810Ctx, 0, sizeof( OV8810_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV8810Ctx );
        return ( result );
    }

    pOV8810Ctx->IsiCtx.HalHandle           = pConfig->HalHandle;
    pOV8810Ctx->IsiCtx.HalDevID            = pConfig->HalDevID;
    pOV8810Ctx->IsiCtx.I2cBusNum           = pConfig->I2cBusNum;
    pOV8810Ctx->IsiCtx.SlaveAddress        = ( pConfig->SlaveAddr == 0U ) ? OV8810_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV8810Ctx->IsiCtx.NrOfAddressBytes    = 2U;

    pOV8810Ctx->IsiCtx.I2cAfBusNum         = pConfig->I2cAfBusNum;
    pOV8810Ctx->IsiCtx.SlaveAfAddress      = ( pConfig->SlaveAfAddr == 0U ) ? OV8810_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV8810Ctx->IsiCtx.NrOfAfAddressBytes  = 0U;

    pOV8810Ctx->IsiCtx.pSensor             = pConfig->pSensor;

    pOV8810Ctx->Configured          = BOOL_FALSE;
    pOV8810Ctx->Streaming           = BOOL_FALSE;

    pConfig->hSensor = ( IsiSensorHandle_t )pOV8810Ctx;

    result = HalSetCamConfig( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, true, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, 10000000UL);

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV8810 sensor instance.
 *
 * @param   handle      OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV8810_IsiReleaseSensorIss
(
    IsiSensorHandle_t   handle
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV8810_IsiSensorSetStreamingIss( pOV8810Ctx, BOOL_FALSE );
    (void)OV8810_IsiSensorSetPowerIss( pOV8810Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV8810Ctx->IsiCtx.HalHandle );

    MEMSET( pOV8810Ctx, 0, sizeof( OV8810_Context_t ) );
    free ( pOV8810Ctx );

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8810_IsiGetCapsIss
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
static RESULT OV8810_IsiGetCapsIss
(
     IsiSensorHandle_t  handle,
     IsiSensorCaps_t    *pIsiSensorCaps
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_12BIT|ISI_BUSWIDTH_10BIT_EX;
        pIsiSensorCaps->Mode            = ISI_MODE_BAYER;
        //        pIsiSensorCaps->ulFieldInv           = ISI_FIELDINV_NOSWAP;
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

        pIsiSensorCaps->Resolution      = ( ISI_RES_TV1080P24 |ISI_RES_TV1080P30 | ISI_RES_3264_2448 );

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
        pIsiSensorCaps->AfpsResolutions = ISI_AFPS_NOTSUPP;
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV8810_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_12BIT,         // BusWidth
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
    //ISI_RES_3264_2448,          // Res
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_F11,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_OFF,               // MipiMode
    ISI_AFPS_NOTSUPP,            // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          OV8810_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8810 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_SetupOutputFormat
(
    OV8810_Context_t        *pOV8810Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_10BIT_EX supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT_EX:
        {
            /* always 10 bit output, if CamerIc input has more than 10 bit,
             * MSBs will be copied to fill up LSBs */
            break;
        }

        case ISI_BUSWIDTH_12BIT: //ov8810 supports 12bits output
            break;

        default:
        {
            TRACE( OV8810_ERROR, "%s: bus width not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: mode not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

/*    switch ( pConfig -> ulFieldInv )  // only ISI_FIELDINV_NOSWAP supported, no configuration needed
    {
          case ISI_FIELDINV_NOSWAP:
          {
              break;
          }

          default:
          {
              TRACE( OV8810_ERROR, "%s: field inversion not supported\n", __FUNCTION__ );
          }
    }*/

    /* field-selection */
    switch ( pConfig->FieldSelection )  /* only ISI_FIELDSEL_BOTH supported, no configuration needed */
    {
        case ISI_FIELDSEL_BOTH:
        {
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: field selection not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV8810 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV8810_ERROR, "%s: 422 conversion not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: bayer pattern not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: HPol not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: VPol not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    /* edge */
    switch ( pConfig->Edge )            /* only ISI_EDGE_RISING supported, no configuration needed */
    {
        case ISI_EDGE_RISING:
        {
#if 0
            result = HalSetCamConfig( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, true, true, false );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

            result = HalSetClock( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, 10000000UL ); // SRJ: whether this is necessary.
            if ( result != RET_SUCCESS )
            {
                return ( result );
            }
#endif
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s:  edge mode not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s:  gamma not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: color conversion not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: SMIA mode not supported\n", __FUNCTION__ );
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
            TRACE( OV8810_ERROR, "%s: MIPI mode not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->AfpsResolutions ) /* only ISI_AFPS_NOTSUPP supported, no configuration needed */
    {
        case ISI_AFPS_NOTSUPP:
        {
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: AFPS not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8810 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_SetupOutputWindow
(
    OV8810_Context_t        *pOV8810Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usXStart   = 0;
    uint16_t usYStart   = 0;
    uint16_t usXEnd     = 0;
    uint16_t usYEnd     = 0;
    uint16_t usHSize    = 0;
    uint16_t usVSize    = 0;
    uint16_t usTMCE     = 0;

    uint16_t usR308a    = 0;
    uint16_t usR3319    = 0;

    uint8_t  ucSubSampling = 0;
    uint8_t  ucSkipping    = 0;
    uint8_t  ucVapEnable   = 0;

    float  rVtPixClkFreq = 0.0f;
    uint16_t usLineLengthPck = 0;
    uint16_t usFrameLengthLines = 0;

    uint16_t usDspHSizeIn = 0;
    uint16_t usDspVSizeIn = 0;

    uint16_t usBandStep50Hz = 0;
    uint16_t usBandStep60Hz = 0;

#if 0
    // uint8_t  ucSize_6 = 0x00;    the horizontal of output window start at 0x00 to crop window
    uint8_t  ucSize_7 = 0x00;
    uint8_t  ucSize_8 = 0x00;
    //uint8_t  ucSize_9 = 0x00;     the vertical of output window start at 0x00 to crop window
    uint8_t  ucSize_A = 0x00;
    uint8_t  ucSize_B = 0x00;
#endif

    uint8_t  ucR_PLL1 = 0;
    uint8_t  ucR_PLL2 = 0;
    uint8_t  ucR_PLL3 = 0;
    uint8_t  ucR_PLL4 = 0;
    uint8_t  ucDSIO_3 = 0;

    uint32_t ulRegValue = 0;


    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    /* resolution */
    switch ( pConfig->Resolution )
    {
        case ISI_RES_TV720P30:
        {
            TRACE( OV8810_INFO, "%s: Resolution 720p30\n", __FUNCTION__ );

#if 0
            // see calculations in OV8810_resolution_size_qxga.xls
            usXStart = 36;
            usYStart = 256;
            usXEnd   = 2603;
            usYEnd   = 1703;
            usHSize  = 1280;                //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 720;                 //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x05;           //full in both directions
            ucSkipping    = 0x00;           //no skipping
            ucVapEnable   = 0x00;           //disabled, no skipping
#endif

            usXStart = 356;
            usYStart = 500;
            usXEnd   = 2939;
            usYEnd   = 1963;
            usHSize  = 1280;
            usVSize  = 720;
            usTMCE   = 0x03;

            usR308a  = 0x02;
            usR3319  = 0x04;

            ucSubSampling = 0x45; //the value is 0x05 in the datasheet
            ucSkipping    = 0x00;
            ucVapEnable   = 0x00;


            // remember that the next three values are also handed over to the AEC module

            rVtPixClkFreq      = 20000000;
            usLineLengthPck    = 1984;
            usFrameLengthLines = 764;

            usDspHSizeIn = 1284;
            usDspVSizeIn = 724;

            //        usBandStep50Hz = 0x005B;
            usBandStep50Hz = 0x0073;
            usBandStep60Hz = 0x004C;
#if 0
            ucSize_7 = 0xFF;
            ucSize_8 = 0x40;
            ucSize_A = 0xCF;
            ucSize_B = 0x20;
#endif
            ucR_PLL1 = 0x84;
            ucR_PLL2 = 0x04; //ucR_PLL2 = 0x48; hacked by Rongjie
            ucR_PLL3 = 0x4C;
            ucR_PLL4 = 0x21;
            ucDSIO_3 = 0x08;
            break;
#if 0
            rVtPixClkFreq      = 48000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 1920;     //see OV8810_resolution_size_qxga.xls
            usFrameLengthLines = 756;

            usDspHSizeIn = 1284;           //see OV8810_resolution_size_qxga.xls
            usDspVSizeIn = 724;

            usBandStep50Hz = 0x019B;
            usBandStep60Hz = 0x0156;



            ucHVPad  = 0x22;

            ucR_PLL1 = 0x2E;
            ucR_PLL2 = 0x10;
            ucR_PLL3 = 0x03;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x00; //[3] enable, [2] reserved, [1:0] divider

            break;
#endif
        }

        case ISI_RES_TV720P60:
        {
            TRACE( OV8810_INFO, "%s: Resolution 720P60\n", __FUNCTION__ );

            usXStart = 356;
            usYStart = 500;
            usXEnd   = 2939;
            usYEnd   = 1963;
            usHSize  = 1280;
            usVSize  = 720;
            usTMCE   = 0x03;

            usR308a  = 0x02;
            usR3319  = 0x04;

            ucSubSampling = 0x45; //the value is 0x05 in the datasheet
            ucSkipping    = 0x00;
            ucVapEnable   = 0x00;

            // remember that the next three values are also handed over to the AEC module

            rVtPixClkFreq      = 95000000;//20000000;
            usLineLengthPck    = 1984;
            usFrameLengthLines = 764;

            usDspHSizeIn = 1284;
            usDspVSizeIn = 724;

            //          usBandStep50Hz = 0x005B;
            usBandStep50Hz = 0x0073;
            usBandStep60Hz = 0x004C;
#if 0
            ucSize_7 = 0xFF;
            ucSize_8 = 0x40;
            ucSize_A = 0xCF;
            ucSize_B = 0x20;
#endif
            ucR_PLL1 = 0x24;//0x84;
            ucR_PLL2 = 0x04; //ucR_PLL2 = 0x48; hacked by Rongjie
            ucR_PLL3 = 0x4C;
            ucR_PLL4 = 0x21;
            ucDSIO_3 = 0x08;

            break;
        }

        case ISI_RES_TV1080P24:
        {
            TRACE( OV8810_INFO, "%s: Resolution 1080P24\n", __FUNCTION__ );

            usXStart = 682;
            usYStart = 686;
            usXEnd   = 2613;
            usYEnd   = 1777;
            usHSize  = 1920;
            usVSize  = 1080;
            usTMCE   = 0x06;

            usR308a  = 0x01;
            usR3319  = 0x08;

            ucSubSampling = 0x40;
            ucSkipping    = 0x00;
            ucVapEnable   = 0x00;

            rVtPixClkFreq      = 72000000;//63750000;//20000000;
            usLineLengthPck    = 2624;
            usFrameLengthLines = 1124;

            usDspHSizeIn = 1922;
            usDspVSizeIn = 1084;

            usBandStep50Hz = 0x005B;//0x0112;
            usBandStep60Hz = 0x004C;//0x00E5;

            ucR_PLL1 = 0x25;    //0x05;
            ucR_PLL2 = 0x24;    //ucR_PLL2 = 0x40; hacked by Rongjie
            ucR_PLL3 = 0x48;    //0x28;
            ucR_PLL4 = 0x21;    //0x28;
            ucDSIO_3 = 0x08;

            break;
        }

        case ISI_RES_TV1080P30:
        {
            TRACE( OV8810_INFO, "%s: Resolution 1080P30\n", __FUNCTION__ );

            usXStart = 682;
            usYStart = 686;
            usXEnd   = 2613;
            usYEnd   = 1777;
            usHSize  = 1920;
            usVSize  = 1080;
            usTMCE   = 0x06;

            usR308a  = 0x01;
            usR3319  = 0x08;

            ucSubSampling = 0x40;
            ucSkipping    = 0x00;
            ucVapEnable   = 0x00;

            rVtPixClkFreq      = 95000000;//20000000;
            usLineLengthPck    = 2624;
            usFrameLengthLines = 1124;

            usDspHSizeIn = 1922;
            usDspVSizeIn = 1084;

            usBandStep50Hz = 0x005B;//0x0027;
            usBandStep60Hz = 0x004C;//0x0020;
#if 0
            ucSize_7 = 0x7F;
            ucSize_8 = 0x70;
            ucSize_A = 0x37;
            ucSize_B = 0x40;
#endif
            /*
                    ucR_PLL1 = 0x2C;
                    ucR_PLL2 = 0x04; //ucR_PLL2 = 0x40; hacked by Rongjie
                    ucR_PLL3 = 0x03;
                    ucR_PLL4 = 0x40;
                    ucDSIO_3 = 0x00;
             */
            ucR_PLL1 = 0x24;//0x05;
            ucR_PLL2 = 0x04; //ucR_PLL2 = 0x40; hacked by Rongjie
            ucR_PLL3 = 0x4c;//0x28;
            ucR_PLL4 = 0x21;//0x28;
            ucDSIO_3 = 0x08;


            break;
        }

        case ISI_RES_3264_2448:
        {
            TRACE( OV8810_INFO, "%s: Resolution 3264x2448\n", __FUNCTION__ );

            usXStart = 0;   //
            usYStart = 0;   //
            usXEnd   = 0x0CDF;  //
            usYEnd   = 0x099F;  //
            usHSize  = 0x0CC0;  //
            usVSize  = 0x0990;  //
            usTMCE   = 0x00;    //
            usR308a  = 0x01;    //
            usR3319  = 0x08;    //

            ucSubSampling = 0x40;   //
            ucSkipping    = 0x00;
            ucVapEnable   = 0x0b;   //

            rVtPixClkFreq      = 20000000;
            usLineLengthPck    = 0x0F78;    //
            usFrameLengthLines = 0x09B4;    //

            usDspHSizeIn = 1922;
            usDspVSizeIn = 1084;

            usBandStep50Hz = 0x003F;    //
            usBandStep60Hz = 0x0020;

            ucR_PLL1 = 0x45;    //
            ucR_PLL2 = 0x28;    //
            ucR_PLL3 = 0x32;    //
            ucR_PLL4 = 0x21;    //
            ucDSIO_3 = 0x08;    //
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    TRACE( OV8810_INFO, "%s: Resolution %dx%d\n", __FUNCTION__, usHSize, usVSize );

    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x308A, usR308a);
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_SIZE_9, usR3319);

    // set camera output margins
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_ADDR_START_2, (usXStart & 0x0000FF00)  >> 8);
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_ADDR_START_1, usXStart & 0xFF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_ADDR_START_2, (usYStart & 0x0000FF00)  >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_ADDR_START_1, usYStart & 0xFF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_ADDR_END_2,   (usXEnd & 0x0000FF00)  >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_ADDR_END_1,   usXEnd & 0xFF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_ADDR_END_2,   (usYEnd & 0x0000FF00) >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_ADDR_END_1,   usYEnd & 0xFF);

    // set camera output size
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_OUTPUTSIZE_2, (usHSize & 0x0000FF00) >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_X_OUTPUTSIZE_1, usHSize & 0xFF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_OUTPUTSIZE_2, (usVSize & 0x0000FF00) >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_Y_OUTPUTSIZE_1, usVSize & 0xFF);

    // set subsampling/skipping
#if 0
    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_IMAGE_TRANSFO, &ulRegValue );
    ulRegValue &= 0xF0;
    ulRegValue |= ucSubSampling;
#endif
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_IMAGE_TRANSFO, ulRegValue );// hacked by Rongjie
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_IMAGE_TRANSFO, ucSubSampling );

    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_0x3313, &ulRegValue );
    ulRegValue &= 0xF0;
    ulRegValue |= ucSkipping;
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x3313, ulRegValue );

    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_TOP1, &ulRegValue );
    ulRegValue &= 0xFF;
    ulRegValue |= ucVapEnable;
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TOP1, ulRegValue );


    // set line and frame length
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_LINE_LENGTH_PCK2,    (usLineLengthPck & 0x0000FF00) >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_LINE_LENGTH_PCK1,    usLineLengthPck & 0x00FF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_FRAME_LENGTH_LINES2, (usFrameLengthLines & 0x0000FF00)>> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_FRAME_LENGTH_LINES1, usFrameLengthLines & 0x00FF);

    // set DSP input size
    // OV8810 hack result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_DSP_HSIZE_IN_2, usDspHSizeIn );
    //OV8810 hack result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_DSP_VSIZE_IN_2, usDspVSizeIn );

    // set banding filter
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_BD50ST_2, (usBandStep50Hz & 0x0000FF00) >> 8 );
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_BD50ST_1, usBandStep50Hz & 0x000000FF);

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_BD60ST_2, (usBandStep60Hz & 0x0000FF00) >> 8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_BD60ST_1, usBandStep60Hz & 0x000000FF);
#if 0
    //xinxin output window size ,start and end   . no error
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_SIZE_7, ucSize_7);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_SIZE_8, ucSize_8);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_SIZE_A, ucSize_A);
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_SIZE_B, ucSize_B);
    // configure clocks
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL1, ucR_PLL1 ); hacked by Rongjie
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL2, ucR_PLL2 ); hacked by Rongjie
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL2, 0x04 );
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL1, 0x05 );

    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_R_PLL3, &ulRegValue );
    ulRegValue &= 0x08;
    ulRegValue |= ucR_PLL3;
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL3, ulRegValue ); hacked by Rongjie
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL3, 0x28 );

    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_R_PLL4, &ulRegValue );
    ulRegValue &= 0x3F;
    ulRegValue |= ucR_PLL4;
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL4, ulRegValue ); hacked by Rongjie
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL4, 0x28 );

    result =  OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_DSIO_3, &ulRegValue );
    ulRegValue &= 0xF0;
    ulRegValue |= ucDSIO_3;
    //result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_DSIO_3, ulRegValue ); hacked by Rongjie
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_DSIO_3, 0x08 );
#endif

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL1, ucR_PLL1 );
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL2, ucR_PLL2 );
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL3, ucR_PLL3 );
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_R_PLL4, ucR_PLL4 );

    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_DSIO_3, ucDSIO_3 );


    //xinxin  set Array bank 6 x_address 608-711
    result =  OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TMCE, usTMCE ); osSleep( 100 );


#if 0
    // store the current image data area information
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.usFrameHSize = usHSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.usFrameVSize = usVSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usHOffs = 0;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usVOffs = 0;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usHSize = usHSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usVSize = usVSize;
#endif

    // store frame timing for later use in AEC module
    pOV8810Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV8810Ctx->LineLengthPck    = usLineLengthPck;
    pOV8810Ctx->FrameLengthLines = usFrameLengthLines;

#if 0
    // store the current image data area information
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.usFrameHSize = usHSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.usFrameVSize = usVSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usHOffs = 0;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usVOffs = 0;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usHSize = usHSize;
    OV8810_g_DriverVariables.apCamInstances[pOV8810Ctx]->tIsiImageDataInfo.tVisibleWindow.usVSize = usVSize;
#endif

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV8810 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_SetupImageControl
(
    OV8810_Context_t        *pOV8810Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_TOP0, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TOP0, RegValue & 0xFE );
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_TOP0, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TOP0, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_AUTO1, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AUTO1, RegValue & ~0x04 );
            break;
        }

        case ISI_AGC_AUTO:
        {
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_AUTO1, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AUTO1, RegValue | 0x04 );
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_TOP0, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TOP0, RegValue & ~0x60 );
            break;
        }

        case ISI_AWB_AUTO:
        {
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_TOP0, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_TOP0, RegValue | 0x60 );
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_AUTO1, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AUTO1, RegValue & 0xFE );
            break;
        }

        case ISI_AEC_AUTO:
        {
            /* settings derived from eva kit (note that gain is controlled by ISI_AGC_AUTO/OFF above)
             *
             * Also note that the AEC needs parameters (target for exposure, WPT/BPT/VPT, see
             * datasheet). Static default settings are for 5MP. It is unknown if parameters are
             * resolution/frametiming dependent. Parameters are not dynamically adapted. */
            result = OV8810_IsiRegReadIss(  pOV8810Ctx, OV8810_AUTO1, &RegValue );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AUTO1, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


/*    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            result = OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_ISP_CTRL00, (RegValue & 0x3F) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            result = OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_AUTO1, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AUTO1, (RegValue | 0xC0U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV8810_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }*/// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV8810-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV8810 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_AecSetModeParameters
(
    OV8810_Context_t        *pOV8810Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    pOV8810Ctx->OldIntegrationTime = 0UL; // this variable reflects the state of the sensor register, it is assumed that
                                          // the sensor initialization sets the exposure register to 0

    if ( pOV8810Ctx->VtPixClkFreq == 0.0f )
    {
        TRACE( OV8810_ERROR, "%s: OV8810_AecSetModeParameters: VtPixClkFreq=0.0f (Division by zero !!)\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    // there are no limits defined in the datasheet, so we assume the max. frame heigth/width is applicable
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )

    pOV8810Ctx->AecMinIntegrationTime = 0.0;
    pOV8810Ctx->AecMaxIntegrationTime = ( ((float)pOV8810Ctx->FrameLengthLines)
                                        * ((float)pOV8810Ctx->LineLengthPck) )
                                        / pOV8810Ctx->VtPixClkFreq;

    pOV8810Ctx->AecMinGain = 1;
    pOV8810Ctx->AecMaxGain = OV8810_MAX_GAIN_AEC;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8810 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV8810Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV8810Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV8810_IsiRegReadIss ( pOV8810Ctx, OV8810_SYS, &RegValue);
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: Can't read OV8810 System Register\n", __FUNCTION__ );
        return ( result );
    }
#if 0
    result = OV8810_IsiRegWriteIss ( pOV8810Ctx, OV8810_SYS, (RegValue| 0x80) );
#endif

    result = OV8810_IsiRegWriteIss ( pOV8810Ctx, OV8810_SYS, 0x80 );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: Can't write OV8810 System Register (sensor reset failed)\n", __FUNCTION__ );
        return ( result );
    }

    TRACE( OV8810_INFO, "%s: OV8810 System-Reset executed\n", __FUNCTION__);

    osSleep( 4 );

    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_IMAGE_SYSTEM, 0x00 );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: Can't write OV8810 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    result = IsiRegDefaultsApply( pOV8810Ctx, OV8810_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 100 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
    result = IsiRegDefaultsVerify( pOV8810Ctx, OV8810_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

#if 0
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x3099, 0x81 ); osSleep( 100 );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x309D, 0x64 ); osSleep( 100 );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
#endif

    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x309E, 0x2d ); osSleep( 100 );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

#if 0
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_0x3087, 0x41 ); osSleep( 100 );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
#endif

    result = OV8810_SetupOutputFormat( pOV8810Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV8810_SetupOutputWindow( pOV8810Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV8810_SetupImageControl( pOV8810Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV8810_AecSetModeParameters( pOV8810Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8810_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    if (result == RET_SUCCESS)
    {
        pOV8810Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiChangeSensorResolutionIss
 *
 * @brief   Change image sensor resolution while keeping all other static settings.
 *          Dynamic settings like current exposure, gain & integration time are
 *          kept as close as possible.
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
static RESULT OV8810_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV8810Ctx->Configured != BOOL_TRUE) || (pOV8810Ctx->Streaming != BOOL_FALSE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    result = OV8810_IsiGetCapsIss( handle, &Caps);
    if (RET_SUCCESS != result)
    {
        return result;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV8810Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution

        //TODO: implement change of resolution while keeping exposure, gain & integration time
        //...

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = 0;

        return RET_OUTOFRANGE;
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiSensorSetStreamingIss
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
static RESULT OV8810_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV8810Ctx->Configured != BOOL_TRUE) || (pOV8810Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = OV8810_IsiRegReadIss ( pOV8810Ctx, OV8810_IMAGE_SYSTEM, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8810_IsiRegWriteIss ( pOV8810Ctx, OV8810_IMAGE_SYSTEM, (RegValue | 0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = OV8810_IsiRegReadIss ( pOV8810Ctx, OV8810_IMAGE_SYSTEM, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8810_IsiRegWriteIss ( pOV8810Ctx, OV8810_IMAGE_SYSTEM, (RegValue & ~0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV8810Ctx->Streaming = on;
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV8810 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV8810Ctx->Configured = BOOL_FALSE;
    pOV8810Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV8810_INFO, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV8810_INFO, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        TRACE( OV8810_INFO, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8810_INFO, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8810_INFO, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8810_INFO, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV8810Ctx->IsiCtx.HalHandle, pOV8810Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV8810_PIDH_DEFAULT;
    RevId = (RevId << 8U) | OV8810_PIDL_DEFAULT;

    result = OV8810_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV8810_INFO, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( OV8810_INFO, "%s RevId, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetSensorRevisionIss
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
static RESULT OV8810_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV8810_IsiRegReadIss ( handle, OV8810_PIDH, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = OV8810_IsiRegReadIss ( handle, OV8810_PIDL, &data );
    *p_value |= ( data & 0xFF );

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8810_IsiRegReadIss
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
static RESULT OV8810_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV8810_g_aRegDescription );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
        *p_value = 0U;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

    TRACE( OV8810_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiRegWriteIss
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
static RESULT OV8810_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV8810_g_aRegDescription );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

    TRACE( OV8810_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV8810 instance
 *
 * @param   handle       OV8810 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV8810_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = 1.0f;
    *pMaxGain = pOV8810Ctx->AecMaxGain;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV8810 instance
 *
 * @param   handle       OV8810 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV8810_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = 0.0f;
    *pMaxIntegrationTime = (float)(pOV8810Ctx->FrameLengthLines * pOV8810Ctx->LineLengthPck) / pOV8810Ctx->VtPixClkFreq;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8810_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV8810 sensor instance handle
 * @param   NewGain                 newly calculated gain to be set
 * @param   NewIntegrationTime      newly calculated integration time to be set
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    float   Gain                    = 0.0f;

    uint8_t  ucMultiplier           = 0U;
    uint8_t  ucGain                 = 0U;
    uint32_t ulGainRegister         = 0U;

    /* variables for exposure calculation */
    uint32_t CoarseIntegrationTime  = 0UL;
    uint32_t FineIntegrationTime    = 0UL;
    float    ShutterWidthPck        = 0.0f;  // Shutter width in pixel clock periods
    float    IntegrationTime        = 0.0f;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum gain & integration time is limited by the sensor, if these limits are
    //not considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if( NewGain < pOV8810Ctx->AecMinGain ) NewGain = pOV8810Ctx->AecMinGain;
    if( NewGain > pOV8810Ctx->AecMaxGain ) NewGain = pOV8810Ctx->AecMaxGain;
    if( NewIntegrationTime > pOV8810Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV8810Ctx->AecMaxIntegrationTime;
    if( NewIntegrationTime < pOV8810Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV8810Ctx->AecMinIntegrationTime;

    /**
     * The actual integration time is given by:
     * integration_time = ((coarse_integration_time*line_length_pck)+fine_integration_time)/(vt_pix_clk_freq)
     *
     * that leads to:
     * coarse_integration_time = (uint32_t)((integration_time*vt_pix_clk_freq)/line_length_pck)
     * fine_integration_time   = (uint32_t)((integration_time*vt_pix_clk_freq)-(coarse_integration_time*line_length_pck))
     */

    ShutterWidthPck = NewIntegrationTime * pOV8810Ctx->VtPixClkFreq;

    // calculate the integer part of the integration time in units of line length
    // avoid division by zero
    if ( pOV8810Ctx->LineLengthPck == 0U )
    {
        TRACE( OV8810_ERROR, "%s: LineLengthPck=0U (Division by zero !!!)\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / pOV8810Ctx->LineLengthPck );

    // calculate the fractional part of the integration time in units of pixel clocks
    FineIntegrationTime   = (uint32_t)ShutterWidthPck - (CoarseIntegrationTime * pOV8810Ctx->LineLengthPck);

    // Write new integration time settings into the camera registers.
    // Do not use fine integration time. The behaviour of the OV8810 fine integration time register
    // is strange and unknown.
    // Do not write if nothing has changed.
    if ( CoarseIntegrationTime != pOV8810Ctx->OldCoarseIntegrationTime )
    {
        result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AECL_2, CoarseIntegrationTime >> 8 );
        result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AECL_1, CoarseIntegrationTime & 0x000000FF );
        *pNumberOfFramesToSkip   = 1;                     // skip 1 frame
        pOV8810Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime; // remember current integration time
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; // no frame skip
    }

    // result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_LAEC_2, FineIntegrationTime );

    // Calculate and set new gain register settings.
    // See mail from Omnivision FAE, analog gain = (Bit[6]+1)*(Bit[5]+1)*(Bit[4]+1)*(Bit[3:0]/16+1).
    // Lower multiplier bits have to be set before higher bits are set.
    Gain = NewGain;

    if ( Gain < 2.0f )
    {
        ucMultiplier = 0U;                  // 2*0 = x1
    }
    else if ( Gain < 4.0f )
    {
        ucMultiplier = 0x10;                // 2*1 = x2
        Gain /= 2.0f;
    }
    else  if ( Gain < 8.0f )
    {
        ucMultiplier = 0x20 + 0x10;         // 2*2 = x4
        Gain /= 4.0f;
    }
    else
    {
        ucMultiplier = 0x40 + 0x20 + 0x10;  // 2*2*2 = x8
        Gain /= 8.0f;
    }

    Gain  = 16 * (Gain - 1.0f) + 0.5f;

    ucGain = (uint8_t)Gain;
    if ( ucGain > 0x0F )
    {
        ucGain = 0x0F; // avoid overflow due to limited floating point precision
    }

    result = OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_AGCL, &ulGainRegister );
    ulGainRegister &= 0x80; // keep only digital gain bit 7
    ulGainRegister += ( ucMultiplier + ucGain );
    result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_AGCL, ulGainRegister );

    // calculate gain & integration time actually set

    // Actual hardware gain may deviate by 1/16, 2/16, ... , 8/16 from NewGain.
    // Currently OV8810_MAX_GAIN_AEC is 8.0f, so max. deviation is 4/16.
    Gain = ucGain * ((ucMultiplier>>4)+1);

    IntegrationTime = ( (float)CoarseIntegrationTime * (float)pOV8810Ctx->LineLengthPck + (float)FineIntegrationTime ) / pOV8810Ctx->VtPixClkFreq;

    pOV8810Ctx->AecCurGain             = NewGain;
    pOV8810Ctx->AecCurIntegrationTime  = IntegrationTime;

    //return current state
    *pSetGain            = pOV8810Ctx->AecCurGain;
    *pSetIntegrationTime = pOV8810Ctx->AecCurIntegrationTime;

    TRACE( OV8810_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime );
    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        TRACE( OV8810_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV8810Ctx->AecCurGain;
    *pSetIntegrationTime = pOV8810Ctx->AecCurIntegrationTime;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibKFactor
 *
 * @brief   Returns the OV8810 specific K-Factor
 *
 * @param   handle       OV8810 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV8810_KFactor;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV8810 specific PCA-Matrix
 *
 * @param   handle          OV8810 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV8810_PCAMatrix;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV8810 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV8810_SVDMeanValue;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV8810 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiCenterLine = (IsiLine_t*)&OV8810_CenterLine;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV8810 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiClipParam = (IsiAwbClipParm_t *)&OV8810_AwbClipParm;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV8810 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV8810_AwbGlobalFadeParm;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV8810 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV8810_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV8810_AwbFade2Parm;

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8810_IsiGetIlluProfile
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
static RESULT OV8810_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
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
        for ( i=0U; i<OV8810_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV8810_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV8810_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiGetLscMatrixTable
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
static RESULT OV8810_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
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
                if ( (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV8810_LscMatrixTable_CIE_A_1920x1080;
                }
                else
                {
                    TRACE( OV8810_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV8810_LscMatrixTable_CIE_F2_1920x1080;
                }
                else
                {
                    TRACE( OV8810_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV8810_LscMatrixTable_CIE_D50_1920x1080;
                }
                else
                {
                    TRACE( OV8810_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV8810_LscMatrixTable_CIE_D65_1920x1080;
                }
                else
                {
                    TRACE( OV8810_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV8810Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV8810_LscMatrixTable_CIE_F11_1920x1080;
                }
                else
                {
                    TRACE( OV8810_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV8810_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8810_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV8810 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pMaxStep = MAX_LOG;

    result = OV8810_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV8810 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0U, 0U };

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    nPosition = ( Position >= MAX_LOG ) ? 0U : ( MAX_REG - (Position * 16) );

    data[0] = (uint8_t)(0x40 | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0

    TRACE( OV8810_DEBUG, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    result = HalWriteI2CMem( pOV8810Ctx->IsiCtx.HalHandle,
                                pOV8810Ctx->IsiCtx.I2cAfBusNum,
                                pOV8810Ctx->IsiCtx.SlaveAfAddress,
                                0U,
                                pOV8810Ctx->IsiCtx.NrOfAfAddressBytes,
                                data,
                                2U );

    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV8810 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0U, 0U };

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV8810Ctx->IsiCtx.HalHandle,
                                pOV8810Ctx->IsiCtx.I2cAfBusNum,
                                pOV8810Ctx->IsiCtx.SlaveAfAddress,
                                0U,
                                pOV8810Ctx->IsiCtx.NrOfAfAddressBytes,
                                data,
                                2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV8810_DEBUG, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see AD5820 datasheet */
    /* Data[1] = D3..D0, S3..S0 */
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

    /* map 0 to 64 -> infinity */
    if( *pAbsStep == 0U )
    {
        *pAbsStep = MAX_LOG;
    }
    else
    {
        *pAbsStep = ( MAX_REG - *pAbsStep ) / 16;
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8810_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8810_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *
 */
/*****************************************************************************/
/* Debug by FeiPeng TODO*/
/*****************************************************************************/
/**
 **          OV8810_IsiActivateTestPattern
 **
 ** @brief   Triggers a forced calibration of the focus hardware.
 **
 ** @param   handle          OV8810 sensor instance handle
 **
 ** @return  Return the result of the function call.
 ** @retval  RET_SUCCESS
 ** @retval  RET_WRONG_HANDLE
 ** @retval  RET_NULL_POINTER
 **
 ******************************************************************************/
static RESULT OV8810_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t ulRegValue = 0UL;

    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_ISP_TEST, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_ISP_TEST, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV8810_IsiRegReadIss( pOV8810Ctx, OV8810_ISP_TEST, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV8810_IsiRegWriteIss( pOV8810Ctx, OV8810_ISP_TEST, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 * @brief   This function dumps the complete OV8810 register map to file
 *
 * @param   handle          OV8810 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
RESULT OV8810_IsiDumpAllRegisters
(
    IsiSensorHandle_t   handle,
    const uint8_t       *filename
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    else
    {
        FILE *f = fopen ( (const char *)filename, "w" );
        if ( f )
        {
            const IsiRegDescription_t *pRegDesc = OV8810_g_aRegDescription;

            fprintf( f, "*************************************************************\n" );
            fprintf( f, "* IMAGE SENSOR REGISTERS                                    *\n" );
            fprintf( f, "*************************************************************\n" );

            while ( pRegDesc->Flags != eTableEnd )
            {
                if ( pRegDesc->Flags & eReadWrite )
                {
                    fprintf( f, " %-30s @ 0x%04x", pRegDesc->pName, pRegDesc->Addr );
                    if ( pRegDesc->Flags & eReadable)
                    {
                        uint32_t value = 0UL;
                        RESULT result;

                        /* read register from image sensor */
                        result = OV8810_IsiRegReadIss( pOV8810Ctx, pRegDesc->Addr, &value );
                        if ( result != RET_SUCCESS )
                        {
                            fclose( f );
                            return ( result );
                        }

                        fprintf( f, " = 0x%08x", value );
                        if ( pRegDesc->Flags & eNoDefault )
                        {
                            fprintf( f, "\n" );
                        }
                        else if ( value == pRegDesc->DefaultValue )
                        {
                            fprintf( f, " (= default value)\n");
                        }
                        else
                        {
                            fprintf( f, " (default was 0x%08x)\n", pRegDesc->DefaultValue);
                        }
                    }
                    else
                    {
                        fprintf( f, " <read failure %d>\n", result );
                    }
                }
                else if ( pRegDesc->Flags & eWritable )
                {
                    fprintf( f, " <is only writable>\n");
                }

                ++pRegDesc;
            }

            fclose( f );
        }
    }


    TRACE( OV8810_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV8810_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV8810_Context_t *pOV8810Ctx = (OV8810_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV8810_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8810Ctx == NULL )
    {
    	TRACE( OV8810_ERROR, "%s: pOV8810Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV8810_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}


/*****************************************************************************/
/**
 *          OV8810_IsiGetSensorIss
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
RESULT OV8810_IsiGetSensorIss
(
    IsiSensor_t     *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8810_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                         = OV8810_g_acName;
        pIsiSensor->pRegisterTable                  = OV8810_g_aRegDescription;
        pIsiSensor->pIsiSensorCaps                  = &OV8810_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer				= OV8810_IsiGetSensorIsiVersion;
		
        pIsiSensor->pIsiCreateSensorIss             = OV8810_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss            = OV8810_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                  = OV8810_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss              = OV8810_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss   = OV8810_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss       = OV8810_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss           = OV8810_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss    = OV8810_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss        = OV8810_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss             = OV8810_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss            = OV8810_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss          = OV8810_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss            = OV8810_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss = OV8810_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss       = OV8810_IsiGetCurrentExposureIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor             = OV8810_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix           = OV8810_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue        = OV8810_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine          = OV8810_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam           = OV8810_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam     = OV8810_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam           = OV8810_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile              = OV8810_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable           = OV8810_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds         = OV8810_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive           = OV8810_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                 = OV8810_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                 = OV8810_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate           = OV8810_IsiMdiFocusCalibrate;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern         = OV8810_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV8810_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV8810_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV8810_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV8810_SLAVE_ADDR;
    pSensorI2cInfo->soft_reg_addr = OV8810_SYS;
    pSensorI2cInfo->soft_reg_value = 0x80;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    pSensorI2cInfo->resolution = ( ISI_RES_TV1080P24 |ISI_RES_TV1080P30 | ISI_RES_3264_2448 );
    
    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = OV8810_PIDH;  
    pChipIDInfo_H->chipid_reg_value = OV8810_PIDH_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV8810_PIDL;
    pChipIDInfo_L->chipid_reg_value = OV8810_PIDL_DEFAULT;
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
    OV8810_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,
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

        0,

        0,                      /**< IsiSensor_t.pIsiActivateTestPattern */
        0,
        0,						/**< IsiSensor_t.pIsiGetColorIss */
    },
    OV8810_IsiGetSensorI2cInfo,
};

