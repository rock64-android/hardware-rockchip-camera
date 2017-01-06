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
 * @file OV5630.c
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

#include "OV5630_priv.h"

#define  OV5630_NEWEST_TUNING_XML "26-Jul-2011_HVO_OV5630_sample01_v1.0"


/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV5630_INFO , "OV5630: ", INFO,    0 );
CREATE_TRACER( OV5630_WARN , "OV5630: ", WARNING, 1 );
CREATE_TRACER( OV5630_ERROR, "OV5630: ", ERROR,   1 );

CREATE_TRACER( OV5630_DEBUG, "",         INFO,    0 );

#define OV5630_SLAVE_ADDR       0x36                            /**< i2c slave address of the OV5630 camera sensor */
#define OV5630_SLAVE_AF_ADDR    0x0C                            /**< i2c slave address of the OV5630 integrated AD5820 */

#define STEPS_IN_GAIN_NUMBER    ( 0 )       /**< Number of max steps_in_gain */
#define OV5630_MAX_GAIN_AEC     ( 8.0f )    /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */



/*!<
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG   64
#define MAX_REG 1023



/*!<
 * Lens movement is triggered every 133ms (VGA, 7.5fps processed frames
 * worst case assumed, usually even much slower, see OV5630 driver for
 * details). Thus the lens has to reach the requested position after
 * max. 133ms. Minimum mechanical ringing is expected with mode 1 ,
 * 100us per step. A movement over the full range needs max. 102.3ms
 * (see table 9 AD5820 datasheet).
 */
#define MDI_SLEW_RATE_CTRL 2 /* S3..0 */

#define AWB_SATURATION_ARRAY_SIZE       4
#define AWB_COLORMATRIX_ARRAY_SIZE      4

#define AWB_VIGNETTING_ARRAY_SIZE       4
#define AWB_LSCMATRIX_ARRAY_SIZE        2


/******************************************************************************
 * local type definitions
 *****************************************************************************/



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV5630_g_acName[] = "OV5630";
extern const IsiRegDescription_t OV5630_g_aRegDescription[];
const IsiSensorCaps_t OV5630_g_IsiSensorDefaultConfig;

// The sensor design may not allow to alter integration time from frame to frame
// (for example the classic rolling shutter). So we remember the old integration
// time to figure out if we are changing it and to tell the upper layers how much
// frames they have to wait before executing the AE again.
// static uint32_t OldCoarseIntegrationTime = 0UL;

/* AWB specific value (from OV5630_tables.c) */
extern const Isi1x1FloatMatrix_t    OV5630_KFactor;
extern const Isi3x2FloatMatrix_t    OV5630_PCAMatrix;
extern const Isi3x1FloatMatrix_t    OV5630_SVDMeanValue;
extern const IsiLine_t              OV5630_CenterLine;
extern const IsiAwbClipParm_t       OV5630_AwbClipParm;
extern const IsiAwbGlobalFadeParm_t OV5630_AwbGlobalFadeParm;
extern const IsiAwbFade2Parm_t      OV5630_AwbFade2Parm;

/* illumination profiles */
#include "OV5630_a.h"       /* CIE A - default profile */
#include "OV5630_d50.h"     /* sunny (D50) */
#include "OV5630_d60.h"     /* day   (D60) */
#include "OV5630_d65.h"     /* CIE D65 (D65) note: indoor because of our lightbox */
#include "OV5630_d75.h"     /* CIE D75 (D75) overcast daylight, 7500K */
#include "OV5630_d80.h"     /* shade (D80) */
#include "OV5630_d120.h"    /* twilight (D120) */
#include "OV5630_f2.h"      /* CIE F2 (cool white flourescent CWF) */
#include "OV5630_f11.h"     /* CIE F11 (TL84) */
#include "OV5630_f12.h"     /* CIE F12 (TL83) */


#define OV5630_ISIILLUPROFILES_DEFAULT  10
static IsiIlluProfile_t OV5630_IlluProfileDefault[OV5630_ISIILLUPROFILES_DEFAULT] =
{
    {
        .p_next             = NULL,

        .name               = "A",
        .id                 = ISI_CIEPROF_A,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_CIE_A,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_CIE_A,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_CIE_A,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_CIE_A,
        .pGaussFactor       = &OV5630_GaussFactor_CIE_A,
        .pThreshold         = &OV5630_Threshold_CIE_A,
        .pComponentGain     = &OV5630_CompGain_CIE_A,

        .pSaturationCurve   = &OV5630_SaturationCurve_CIE_A,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_CIE_A,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_CIE_A,

        .pVignettingCurve   = &OV5630_VignettingCurve_CIE_A,
    },
    {
        .p_next             = NULL,

        .name               = "D50",
        .id                 = ISI_CIEPROF_D50,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_TRUE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_sunny,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_sunny,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_sunny,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_sunny,
        .pGaussFactor       = &OV5630_GaussFactor_sunny,
        .pThreshold         = &OV5630_Threshold_sunny,
        .pComponentGain     = &OV5630_CompGain_sunny,

        .pSaturationCurve   = &OV5630_SaturationCurve_sunny,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_sunny,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_sunny,

        .pVignettingCurve   = &OV5630_VignettingCurve_sunny,
    },
    {
        .p_next             = NULL,

        .name               = "D60",
        .id                 = ISI_CIEPROF_D60,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_day,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_day,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_day,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_day,
        .pGaussFactor       = &OV5630_GaussFactor_day,
        .pThreshold         = &OV5630_Threshold_day,
        .pComponentGain     = &OV5630_CompGain_day,

        .pSaturationCurve   = &OV5630_SaturationCurve_day,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_day,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_day,

        .pVignettingCurve   = &OV5630_VignettingCurve_day,
    },
    {
        .p_next             = NULL,

        .name               = "D65",
        .id                 = ISI_CIEPROF_D65,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_D65,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_D65,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_D65,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_D65,
        .pGaussFactor       = &OV5630_GaussFactor_D65,
        .pThreshold         = &OV5630_Threshold_D65,
        .pComponentGain     = &OV5630_CompGain_D65,

        .pSaturationCurve   = &OV5630_SaturationCurve_D65,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_D65,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_D65,

        .pVignettingCurve   = &OV5630_VignettingCurve_D65,
    },
    {
        .p_next             = NULL,

        .name               = "D75",
        .id                 = ISI_CIEPROF_D75,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_D75,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_D75,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_D75,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_D75,
        .pGaussFactor       = &OV5630_GaussFactor_D75,
        .pThreshold         = &OV5630_Threshold_D75,
        .pComponentGain     = &OV5630_CompGain_D75,

        .pSaturationCurve   = &OV5630_SaturationCurve_D75,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_D75,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_D75,

        .pVignettingCurve   = &OV5630_VignettingCurve_D75,
    },
    {
        .p_next             = NULL,

        .name               = "D80",
        .id                 = ISI_CIEPROF_D80,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_shade,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_shade,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_shade,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_shade,
        .pGaussFactor       = &OV5630_GaussFactor_shade,
        .pThreshold         = &OV5630_Threshold_shade,
        .pComponentGain     = &OV5630_CompGain_shade,

        .pSaturationCurve   = &OV5630_SaturationCurve_shade,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_shade,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_shade,

        .pVignettingCurve   = &OV5630_VignettingCurve_shade,
    },
    {
        .p_next             = NULL,

        .name               = "D120",
        .id                 = ISI_CIEPROF_D120,
        .DoorType           = ISI_DOOR_TYPE_OUTDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_twilight,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_twilight,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_twilight,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_twilight,
        .pGaussFactor       = &OV5630_GaussFactor_twilight,
        .pThreshold         = &OV5630_Threshold_twilight,
        .pComponentGain     = &OV5630_CompGain_twilight,

        .pSaturationCurve   = &OV5630_SaturationCurve_twilight,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_twilight,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_twilight,

        .pVignettingCurve   = &OV5630_VignettingCurve_twilight,
    },
    {
        .p_next             = NULL,

        .name               = "F2",
        .id                 = ISI_CIEPROF_F2,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_F2,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_F2,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_F2,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_F2,
        .pGaussFactor       = &OV5630_GaussFactor_F2,
        .pThreshold         = &OV5630_Threshold_F2,
        .pComponentGain     = &OV5630_CompGain_F2,

        .pSaturationCurve   = &OV5630_SaturationCurve_F2,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_F2,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_F2,

        .pVignettingCurve   = &OV5630_VignettingCurve_F2,
    },
    {
        .p_next             = NULL,

        .name               = "F11",
        .id                 = ISI_CIEPROF_F11,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_F11,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_F11,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_F11,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_F11,
        .pGaussFactor       = &OV5630_GaussFactor_F11,
        .pThreshold         = &OV5630_Threshold_F11,
        .pComponentGain     = &OV5630_CompGain_F11,

        .pSaturationCurve   = &OV5630_SaturationCurve_F11,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_F11,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_F11,

        .pVignettingCurve   = &OV5630_VignettingCurve_F11,
    },
    {
        .p_next             = NULL,

        .name               = "F12",
        .id                 = ISI_CIEPROF_F12,
        .DoorType           = ISI_DOOR_TYPE_INDOOR,
        .AwbType            = ISI_AWB_TYPE_AUTO,

        .bOutdoorClip       = BOOL_FALSE,

        /* legacy stuff */
        .pCrossTalkCoeff    = &OV5630_XTalkCoeff_F12,
        .pCrossTalkOffset   = &OV5630_XTalkOffset_F12,

        .pGaussMeanValue    = &OV5630_GaussMeanValue_F12,
        .pCovarianceMatrix  = &OV5630_CovarianceMatrix_F12,
        .pGaussFactor       = &OV5630_GaussFactor_F12,
        .pThreshold         = &OV5630_Threshold_F12,
        .pComponentGain     = &OV5630_CompGain_F12,

        .pSaturationCurve   = &OV5630_SaturationCurve_F12,
        .pCcMatrixTable     = &OV5630_CcMatrixTable_F12,
        .pCcOffsetTable     = &OV5630_CcOffsetTable_F12,

        .pVignettingCurve   = &OV5630_VignettingCurve_F12,
    },
};



/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV5630_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV5630_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV5630_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV5630_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV5630_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV5630_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV5630_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV5630_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV5630_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV5630_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV5630_IsiExposureControlIss( IsiSensorHandle_t handle, float newGain, float newIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV5630_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV5630_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV5630_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV5630_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV5630_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV5630_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime );

static RESULT OV5630_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV5630_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV5630_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV5630_IsiGetCalibPcaMatrix( IsiSensorHandle_t handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV5630_IsiGetCalibSvdMeanValue( IsiSensorHandle_t handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV5630_IsiGetCalibCenterLine( IsiSensorHandle_t handle, IsiLine_t **ptIsiCenterLine );
static RESULT OV5630_IsiGetCalibClipParam( IsiSensorHandle_t handle, IsiAwbClipParm_t **pIsiClipParam );
static RESULT OV5630_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t handle, IsiAwbGlobalFadeParm_t **ptIsiGlobalFadeParam );
static RESULT OV5630_IsiGetCalibFadeParam( IsiSensorHandle_t handle, IsiAwbFade2Parm_t **ptIsiFadeParam );
static RESULT OV5630_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV5630_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV5630_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV5630_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV5630_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV5630_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT OV5630_IsiActivateTestPattern( IsiSensorHandle_t handle, const bool_t enable );

static RESULT OV5630_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT OV5630_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t handle, char** pTuningXmlVersion);




/*****************************************************************************/
/**
 *          floor
 *
 * @brief   Implements floor function to avoid include math-lib
 *
 *****************************************************************************/
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
 *          OV5630_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV5630 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV5630_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t   *pConfig
)
{
    RESULT result = RET_SUCCESS;

    OV5630_Context_t *pOV5630Ctx;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV5630Ctx = ( OV5630_Context_t * )malloc ( sizeof (OV5630_Context_t) );
    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR,  "%s: Can't allocate ov5630 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV5630Ctx, 0, sizeof( OV5630_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV5630Ctx );
        return ( result );
    }

    pOV5630Ctx->IsiCtx.HalHandle           = pConfig->HalHandle;
    pOV5630Ctx->IsiCtx.HalDevID            = pConfig->HalDevID;
    pOV5630Ctx->IsiCtx.I2cBusNum           = pConfig->I2cBusNum;
    pOV5630Ctx->IsiCtx.SlaveAddress        = ( pConfig->SlaveAddr == 0U ) ? OV5630_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV5630Ctx->IsiCtx.NrOfAddressBytes    = 2U;

    pOV5630Ctx->IsiCtx.I2cAfBusNum         = pConfig->I2cAfBusNum;
    pOV5630Ctx->IsiCtx.SlaveAfAddress      = ( pConfig->SlaveAfAddr == 0U ) ? OV5630_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV5630Ctx->IsiCtx.NrOfAfAddressBytes  = 0U;

    pOV5630Ctx->IsiCtx.pSensor             = pConfig->pSensor;

    pOV5630Ctx->Configured          = BOOL_FALSE;
    pOV5630Ctx->Streaming           = BOOL_FALSE;

    pConfig->hSensor = ( IsiSensorHandle_t )pOV5630Ctx;

    result = HalSetCamConfig( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, true, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, 10000000UL );

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV5630 sensor instance.
 *
 * @param   handle      OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV5630_IsiReleaseSensorIss
(
    IsiSensorHandle_t   handle
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV5630_IsiSensorSetStreamingIss( pOV5630Ctx, BOOL_FALSE );
    (void)OV5630_IsiSensorSetPowerIss( pOV5630Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV5630Ctx->IsiCtx.HalHandle );

    MEMSET( pOV5630Ctx, 0, sizeof( OV5630_Context_t ) );
    free ( pOV5630Ctx );

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCapsIss
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
static RESULT OV5630_IsiGetCapsIss
(
    IsiSensorHandle_t   handle,
    IsiSensorCaps_t     *pIsiSensorCaps
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
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

        pIsiSensorCaps->Resolution      = ( ISI_RES_VGA
                                          | ISI_RES_2592_1944
                                          | ISI_RES_TV720P30
                                          | ISI_RES_TV720P60
                                          | ISI_RES_TV1080P24
                                          | ISI_RES_TV1080P30 );

        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF );
        pIsiSensorCaps->AGC             = ( ISI_AGC_AUTO | ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_AUTO | ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_AUTO | ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11
                                          | ISI_CIEPROF_F12 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_OFF;
        pIsiSensorCaps->AfpsResolutions = ISI_AFPS_NOTSUPP;
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV5630_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT_EX,      // BusWidth
    ISI_MODE_BAYER,             // Mode
    ISI_FIELDSEL_BOTH,          // FieldSelection
    ISI_YCSEQ_YCBYCR,           // YCSequence
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_2592_1944,          // Resolution
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
 *          OV5630_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5630 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_SetupOutputFormat
(
    OV5630_Context_t        *pOV5630Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

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
            TRACE( OV5630_ERROR, "%s: bus width not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: mode not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: field selection not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV5630 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV5630_ERROR, "%s: 422 conversion not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: bayer pattern not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: HPol not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: VPol not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s:  edge mode not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s:  gamma not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: color conversion not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: SMIA mode not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: MIPI mode not supported\n", __FUNCTION__ );
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
            TRACE( OV5630_ERROR, "%s: AFPS not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5630 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_SetupOutputWindow
(
    OV5630_Context_t        *pOV5630Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint16_t usXStart   = 0U;
    uint16_t usYStart   = 0U;
    uint16_t usXEnd     = 0U;
    uint16_t usYEnd     = 0U;
    uint16_t usHSize    = 0U;
    uint16_t usVSize    = 0U;

    uint8_t  ucSubSampling = 0U;
    uint8_t  ucSkipping    = 0U;
    uint8_t  ucVapEnable   = 0U;

    float    rVtPixClkFreq      = 0.0f;
    uint16_t usLineLengthPck    = 0U;
    uint16_t usFrameLengthLines = 0U;


    uint16_t usDspHSizeIn = 0U;
    uint16_t usDspVSizeIn = 0U;

    uint16_t usBandStep50Hz = 0U;
    uint16_t usBandStep60Hz = 0U;

    uint8_t  ucHVPad = 0;

    uint8_t  ucR_PLL1 = 0U;
    uint8_t  ucR_PLL2 = 0U;
    uint8_t  ucR_PLL3 = 0U;
    uint8_t  ucR_PLL4 = 0U;
    uint8_t  ucDSIO_3 = 0U;

    uint32_t ulRegValue = 0;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    /* resolution */
    switch ( pConfig->Resolution )
    {
        case ISI_RES_VGA:
        {
            //see OV5630_clocktree_V02_framefun_25mhz_vga.xls
            //
            //The system is able to process about 8fps (with 2 lines of short debugging output). The cam has to
            //run at 16fps to achieve this (every 2nd frame is skipped with the current processing loop).
            //Unfortunately the max. exposure value drops with higher fps. So we target for 15fps giving 5fps
            //overall performance (measured).
            //
            //HSize = 640
            //VSize = 480
            //
            //LINE_LENGTH_PCK    = 2 * HSize + 640 = 1920 (0x3022/0x3023)  valid calculation for RPCLKdiv=2 (0x30B3[1:0]=1)
            //FRAME_LENGTH_LINES =     VSize +  36 =  516 (0x3020/0x3021)
            //
            //fps = 2 * DVP_PCLK / LINE_LENGTH_PCK / FRAME_LENGTH_LINES) = 15.14 valid calculation for RPCLKdiv=2 (0x30B3[1:0]=1)
            //
            //50Hz/60Hz banding filter from Excel sheet (just for completeness, we use our own AEC anyway):
            //BD50ST = 0x0027 (0x305C/0x305D)
            //BD60ST = 0x0020 (0x305E/0x305F)
            //
            //input clock = 10 MHz
            //
            //from Excel sheet:
            //Sdiv_m and Ldiv just for completeness (we do not use MIPI):
            //PreDiv   = 1, Div_cnt6b = 18 => R_PLL1 = 0x2E (0x300E)
            //Sdiv     = 6, Sdiv_m    = 2  => R_PLL2 = 0x51 (0x300F)
            //Ldiv     = 2, Div45     = 5  => R_PLL3 = 0x07 (0x3010)  PLL charge pump current control from default settings, Bit 3 is reserved
            //CLK_Div  = 2                 => R_PLL4 = 0x40 (0x3011)  Bits [5:0] are reserved
            //RPCLKdiv = 2                 => DSIO_3[1:0]=1 (0x30B3)
            //
            //DVP_PCLK =  input clock / PreDiv * Div_cnt6b * Div45 / Sdiv / Div45 / CLK_Div / RPCLKdiv = 7.5 MHz

            TRACE( OV5630_INFO, "%s: Resolution VGA\n", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_vga.xls
            usXStart =   36;
            usYStart =   12;
            usXEnd   = 2603;               //subsampling/skipping 1:4
            usYEnd   = 1947;
            usHSize  =  640;               //VGA_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  =  480;               //VGA_SIZE_V

            ucSubSampling = 0x09;          //1:4 vertical, 1:2 horizontal
            ucSkipping    = 0x01;          //1:1 vertical, 1:2 horizontal
            ucVapEnable   = 0x01;          //enable for skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 15000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 1920;     //see OV5630_resolution_size_vga.xls
            usFrameLengthLines =  516;

            usDspHSizeIn = 1284;           //see OV5630_resolution_size_vga.xls
            usDspVSizeIn =  484;

            usBandStep50Hz = 0x0027;
            usBandStep60Hz = 0x0020;

            ucHVPad  = 0x20;

            ucR_PLL1 = 0x2E;
            ucR_PLL2 = 0x51;
            ucR_PLL3 = 0x07;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x09; //[3] enable, [2] reserved, [1:0] divider

            break;
        }

        case ISI_RES_TV720P30:
        {
            TRACE( OV5630_INFO, "%s: Resolution 720p30\n", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_qxga.xls
            usXStart = 36;
            usYStart = 256;
            usXEnd   = 2603;
            usYEnd   = 1703;
            usHSize  = 1280;               //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 720;               //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x05;          //full in both directions
            ucSkipping    = 0x00;          //no skipping
            ucVapEnable   = 0x00;          //disabled, no skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 48000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 1920;     //see OV5630_resolution_size_qxga.xls
            usFrameLengthLines = 756;

            usDspHSizeIn = 1284;           //see OV5630_resolution_size_qxga.xls
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
        }

        case ISI_RES_TV720P60:
        {
            TRACE( OV5630_INFO, "%s: Resolution 720P60\n", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_qxga.xls
            usXStart = 36;
            usYStart = 256;
            usXEnd   = 2603;
            usYEnd   = 1703;
            usHSize  = 1280;               //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 720;               //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x05;          //full in both directions
            ucSkipping    = 0x00;          //no skipping
            ucVapEnable   = 0x00;          //disabled, no skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 96000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 1920;     //see OV5630_resolution_size_qxga.xls
            usFrameLengthLines = 756;

            usDspHSizeIn = 1284;           //see OV5630_resolution_size_qxga.xls
            usDspVSizeIn = 724;

            usBandStep50Hz = 0x019B;
            usBandStep60Hz = 0x0156;

            ucHVPad  = 0x22;

            ucR_PLL1 = 0x2E;
            ucR_PLL2 = 0x00;
            ucR_PLL3 = 0x03;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x00; //[3] enable, [2] reserved, [1:0] divider

            break;
        }

        case ISI_RES_TV1080P24:
        {
            TRACE( OV5630_INFO, "%s: Resolution 1080P24\n", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_qxga.xls
            usXStart = 358;
            usYStart = 438;
            usXEnd   = 2281;
            usYEnd   = 1521;
            usHSize  = 1920;               //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 1080;               //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x00;          //full in both directions
            ucSkipping    = 0x00;          //no skipping
            ucVapEnable   = 0x00;          //disabled, no skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 70000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 2560;     //see OV5630_resolution_size_qxga.xls
            usFrameLengthLines = 1116;

            usDspHSizeIn = 1922;           //see OV5630_resolution_size_qxga.xls
            usDspVSizeIn = 1084;

            usBandStep50Hz = 0x0100;
            usBandStep60Hz = 0x00D5;

            ucHVPad  = 0x00;

            ucR_PLL1 = 0x32;
            ucR_PLL2 = 0x00;
            ucR_PLL3 = 0x03;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x00; //[3] enable, [2] reserved, [1:0] divider

            break;
        }

        case ISI_RES_TV1080P30:
        {
            TRACE( OV5630_INFO, "%s: Resolution 1080P30\n", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_qxga.xls
            usXStart = 358;
            usYStart = 438;
            usXEnd   = 2281;
            usYEnd   = 1521;
            usHSize  = 1920;               //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 1080;               //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x00;          //full in both directions
            ucSkipping    = 0x00;          //no skipping
            ucVapEnable   = 0x00;          //disabled, no skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 96000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 2560;     //see OV5630_resolution_size_qxga.xls
            usFrameLengthLines = 1116;

            usDspHSizeIn = 1922;           //see OV5630_resolution_size_qxga.xls
            usDspVSizeIn = 1084;

            usBandStep50Hz = 0x010D;
            usBandStep60Hz = 0x00E0;

            ucHVPad  = 0x00;

            ucR_PLL1 = 0x2D;
            ucR_PLL2 = 0x00;
            ucR_PLL3 = 0x03;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x00; //[3] enable, [2] reserved, [1:0] divider

            break;
        }

        case ISI_RES_2592_1944:
        {
            //see OV5630_clocktree_V02_framefun_25mhz.xls
            //
            //HSize = 2592
            //VSize = 1944
            //
            //LINE_LENGTH_PCK    = HSize + 640 = 3232 (0x3022/0x3023)  valid calculation for RPCLKdiv=1 (0x30B3[1:0]=0)
            //FRAME_LENGTH_LINES = VSize +  36 = 1980 (0x3020/0x3021)
            //
            //fps = DVP_PCLK / LINE_LENGTH_PCK / FRAME_LENGTH_LINES) = 3.91 valid calculation for RPCLKdiv=1 (0x30B3[1:0]=0)
            //
            //50Hz/60Hz banding filter from Excel sheet (just for completeness, we use our own AEC anyway):
            //BD50ST = 0x004D (0x305C/0x305D)
            //BD60ST = 0x0040 (0x305E/0x305F)
            //
            //input clock = 10 MHz
            //
            //from Excel sheet:
            //Sdiv_m and Ldiv just for completeness (we do not use MIPI):
            //PreDiv   = 1, Div_cnt6b = 20 => R_PLL1 = 0x2C (0x300E)
            //Sdiv     = 4, Sdiv_m    = 1  => R_PLL2 = 0x30 (0x300F)
            //Ldiv     = 2, Div45     = 5  => R_PLL3 = 0x07 (0x3010)  PLL charge pump current control from default settings, Bit 3 is reserved
            //CLK_Div  = 2                 => R_PLL4 = 0x40 (0x3011)  Bits [5:0] are reserved
            //RPCLKdiv = 1                 => DSIO_3[1:0]=0 (0x30B3)
            //
            //DVP_PCLK =  input clock / PreDiv * Div_cnt6b * Div45 / Sdiv / Div45 / CLK_Div / RPCLKdiv = 25 MHz
            //
            //The JPEG encoder must run at 2xCLK, make sure to use an appropriate bitstream.

            TRACE( OV5630_INFO, "%s: Resolution QSXGA_2592_1944 ", __FUNCTION__ );

            // see calculations in OV5630_resolution_size_qxga.xls
            usXStart =   22;
            usYStart =    6;
            usXEnd   = 2617;
            usYEnd   = 1953;
            usHSize  = 2592;               //QSXGA_2592_1944_SIZE_H (use hard coded values here for consistency with other hard coded values)
            usVSize  = 1944;               //QSXGA_2592_1944_SIZE_V

            ucSubSampling = 0x00;          //full in both directions
            ucSkipping    = 0x00;          //no skipping
            ucVapEnable   = 0x00;          //disabled, no skipping

            // remember that the next three values are also handed over to the AEC module
            rVtPixClkFreq      = 96000000; //video timing for AEC, scan frequency of sensor array
            usLineLengthPck    = 3232;     //see OV5630_resolution_size_qxga.xls
            usFrameLengthLines = 1980;

            usDspHSizeIn = 2594;           //see OV5630_resolution_size_qxga.xls
            usDspVSizeIn = 1948;

            usBandStep50Hz = 0x004D;
            usBandStep60Hz = 0x0040;

            ucHVPad  = 0x22;

            ucR_PLL1 = 0x2D;
            ucR_PLL2 = 0x00;
            ucR_PLL3 = 0x03;
            ucR_PLL4 = 0x40;
            ucDSIO_3 = 0x00; //[3] enable, [2] reserved, [1:0] divider

            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    TRACE( OV5630_INFO, "%s: Resolution %dx%d\n", __FUNCTION__, usHSize, usVSize );

    // set camera output margins
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_X_ADDR_START_2, usXStart );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_X_ADDR_START_2, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_Y_ADDR_START_2, usYStart );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_X_ADDR_END_2,   usXEnd );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_Y_ADDR_END_2,   usYEnd );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set camera output size
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_X_OUTPUTSIZE_2, usHSize );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_Y_OUTPUTSIZE_2, usVSize );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set subsampling/skipping
    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_IMAGE_TRANSFO, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0xF0;
    ulRegValue |= ucSubSampling;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_IMAGE_TRANSFO, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_0x3313, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0xF0;
    ulRegValue |= ucSkipping;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_0x3313, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL01, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0xFE;
    ulRegValue |= ucVapEnable;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL01, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set line and frame length
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_LINE_LENGTH_PCK_2,    usLineLengthPck );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_FRAME_LENGTH_LINES_2, usFrameLengthLines );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set DSP input size
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_DSP_HSIZE_IN_2, usDspHSizeIn );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_DSP_VSIZE_IN_2, usDspVSizeIn );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set banding filter
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_BD50ST_2, usBandStep50Hz );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_BD60ST_2, usBandStep60Hz );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // set dsp_hpad_out, dsp_vpad_out
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_0x3318, ucHVPad );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // configure clocks
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_R_PLL1, ucR_PLL1 );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_R_PLL2, ucR_PLL2 );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_R_PLL3, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0x08;
    ulRegValue |= ucR_PLL3;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_R_PLL3, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_R_PLL4, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0x3F;
    ulRegValue |= ucR_PLL4;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_R_PLL4, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_DSIO_3, &ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    ulRegValue &= 0xF0;
    ulRegValue |= ucDSIO_3;
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_DSIO_3, ulRegValue );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    // store frame timing for later use in AEC module
    pOV5630Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV5630Ctx->LineLengthPck    = usLineLengthPck;
    pOV5630Ctx->FrameLengthLines = usFrameLengthLines;

#if 0
    // store the current image data area information
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.usFrameHSize = usHSize;
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.usFrameVSize = usVSize;
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.tVisibleWindow.usHOffs = 0;
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.tVisibleWindow.usVOffs = 0;
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.tVisibleWindow.usHSize = usHSize;
    OV5630_g_DriverVariables.apCamInstances[ulCamId]->tIsiImageDataInfo.tVisibleWindow.usVSize = usVSize;
#endif
    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV5630 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_SetupImageControl
(
    OV5630_Context_t        *pOV5630Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue & 0xFEU) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue | 0x01U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AUTO1, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AUTO1, (RegValue & ~0x04U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_AGC_AUTO:
        {
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AUTO1, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AUTO1, (RegValue | 0x04U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            /* turn off AWB (clear bit 4 and 5) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue & ~0x30U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            TRACE( OV5630_DEBUG, "%s: AWB off\n", __FUNCTION__ );
            break;
        }

        case ISI_AWB_AUTO:
        {
            /* turn on AWB (set bit 4 and 5) (not sure about bit 4, just in case) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue | 0x30U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            TRACE( OV5630_DEBUG, "%s: Sensor AWB on\n", __FUNCTION__ );
            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            /* manual exposure, fine int time on (appropriate for AEC with CamerIc) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AUTO1, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AUTO1, (RegValue & 0xF6U) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_AEC_AUTO:
        {
            /* settings derived from eva kit (note that gain is controlled by ISI_AGC_AUTO/OFF above)
             *
             * Also note that the AEC needs parameters (target for exposure, WPT/BPT/VPT, see
             * datasheet). Static default settings are for 5MP. It is unknown if parameters are
             * resolution/frametiming dependent. Parameters are not dynamically adapted. */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AUTO1, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AUTO1, (RegValue | 0xFBU) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV5630_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            /* disable white and black pixel cancellation (clear bit 6 and 7) */
            result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL00, &RegValue );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL00, (RegValue & 0x3F) );
            RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            /* enable white and black pixel cancellation (set bit 6 and 7) */
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
    }

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV5630-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV5630 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_AecSetModeParameters
(
    OV5630_Context_t        *pOV5630Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    pOV5630Ctx->OldCoarseIntegrationTime = 0UL; // this variable reflects the state of the sensor register, it is assumed that
                                                // the sensor initialization sets the exposure register to 0

    if ( pOV5630Ctx->VtPixClkFreq == 0.0f )
    {
        TRACE( OV5630_ERROR, "%s: OV5630_AecSetModeParameters: VtPixClkFreq=0.0f (Division by zero !!)\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    // there are no limits defined in the datasheet, so we assume the max. frame heigth/width is applicable
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )

    pOV5630Ctx->AecMinIntegrationTime = 0.0;
    pOV5630Ctx->AecMaxIntegrationTime = ( ((float)pOV5630Ctx->FrameLengthLines)
                                        * ((float)pOV5630Ctx->LineLengthPck) )
                                        / pOV5630Ctx->VtPixClkFreq;

    pOV5630Ctx->AecMinGain = 1;
    pOV5630Ctx->AecMaxGain = OV5630_MAX_GAIN_AEC;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV5630 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV5630Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV5630Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV5630_IsiRegReadIss ( pOV5630Ctx, OV5630_SYS, &RegValue);
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: Can't read OV5630 System Register\n", __FUNCTION__ );
        return ( result );
    }

    result = OV5630_IsiRegWriteIss ( pOV5630Ctx, OV5630_SYS, (RegValue| 0x80) );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: Can't write OV5630 System Register (sensor reset failed)\n", __FUNCTION__ );
        return ( result );
    }

    TRACE( OV5630_INFO, "%s: OV5630 System-Reset executed\n", __FUNCTION__);

    result = OV5630_IsiRegWriteIss ( pOV5630Ctx, OV5630_SYS, RegValue );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: Can't remove OV5630 System Reset (sensor reset failed)\n", __FUNCTION__ );
        return ( result );
    }

    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    result = IsiRegDefaultsApply( pOV5630Ctx, OV5630_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 100 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
    result = IsiRegDefaultsVerify( pOV5630Ctx, OV5630_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    result = OV5630_SetupOutputFormat( pOV5630Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV5630_SetupOutputWindow( pOV5630Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV5630_SetupImageControl( pOV5630Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV5630_AecSetModeParameters( pOV5630Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV5630_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }

    if (result == RET_SUCCESS)
    {
        pOV5630Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiChangeSensorResolutionIss
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
static RESULT OV5630_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV5630Ctx->Configured != BOOL_TRUE) || (pOV5630Ctx->Streaming != BOOL_FALSE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    result = OV5630_IsiGetCapsIss( handle, &Caps);
    if (RET_SUCCESS != result)
    {
        return result;
    }

    if ( (Resolution & Caps.Resolution) == 0 )
    {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV5630Ctx->Config.Resolution )
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

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiSensorSetStreamingIss
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
static RESULT OV5630_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV5630Ctx->Configured != BOOL_TRUE) || (pOV5630Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = OV5630_IsiRegReadIss ( pOV5630Ctx, OV5630_IMAGE_SYSTEM, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV5630_IsiRegWriteIss ( pOV5630Ctx, OV5630_IMAGE_SYSTEM, (RegValue | 0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = OV5630_IsiRegReadIss ( pOV5630Ctx, OV5630_IMAGE_SYSTEM, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV5630_IsiRegWriteIss ( pOV5630Ctx, OV5630_IMAGE_SYSTEM, (RegValue & ~0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV5630Ctx->Streaming = on;
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV5630 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV5630Ctx->Configured = BOOL_FALSE;
    pOV5630Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV5630_INFO, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5630_INFO, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        TRACE( OV5630_INFO, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5630_INFO, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5630_INFO, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV5630_INFO, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV5630Ctx->IsiCtx.HalHandle, pOV5630Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value = 0UL;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV5630_PIDH_DEFAULT;
    RevId = (RevId << 8U) | OV5630_PIDL_DEFAULT;

    result = OV5630_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        return ( RET_FAILURE );
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetSensorRevisionIss
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
static RESULT OV5630_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV5630_IsiRegReadIss ( handle, OV5630_PIDH, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = OV5630_IsiRegReadIss ( handle, OV5630_PIDL, &data );
    *p_value |= ( data & 0xFF );

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5630_IsiRegReadIss
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
static RESULT OV5630_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV5630_g_aRegDescription );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
        *p_value = 0U;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

    TRACE( OV5630_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiRegWriteIss
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
static RESULT OV5630_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV5630_g_aRegDescription );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

    TRACE( OV5630_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV5630 instance
 *
 * @param   handle       OV5630 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV5630_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = 1.0f;
    *pMaxGain = pOV5630Ctx->AecMaxGain;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV5630 instance
 *
 * @param   handle       OV5630 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV5630_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = 0.0f;
    *pMaxIntegrationTime = (float)(pOV5630Ctx->FrameLengthLines * pOV5630Ctx->LineLengthPck) / pOV5630Ctx->VtPixClkFreq;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5630_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV5630 sensor instance handle
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
RESULT OV5630_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

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

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
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
    if( NewGain < pOV5630Ctx->AecMinGain ) NewGain = pOV5630Ctx->AecMinGain;
    if( NewGain > pOV5630Ctx->AecMaxGain ) NewGain = pOV5630Ctx->AecMaxGain;
    if( NewIntegrationTime > pOV5630Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV5630Ctx->AecMaxIntegrationTime;
    if( NewIntegrationTime < pOV5630Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV5630Ctx->AecMinIntegrationTime;

    // calculate and set new coarse and fine integration time register settings

    /**
     * The actual integration time is given by:
     * integration_time = ((coarse_integration_time*line_length_pck)+fine_integration_time)/(vt_pix_clk_freq)
     *
     * that leads to:
     * coarse_integration_time = (UINT32)((integration_time*vt_pix_clk_freq)/line_length_pck)
     * fine_integration_time   = (UINT32)((integration_time*vt_pix_clk_freq)-(coarse_integration_time*line_length_pck))
     */

    ShutterWidthPck = NewIntegrationTime * pOV5630Ctx->VtPixClkFreq;

    // calculate the integer part of the integration time in units of line length
    // avoid division by zero
    if ( pOV5630Ctx->LineLengthPck == 0U )
    {
        TRACE( OV5630_ERROR, "%s: LineLengthPck=0U (Division by zero !!!)\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / pOV5630Ctx->LineLengthPck );

    // calculate the fractional part of the integration time in units of pixel clocks
    FineIntegrationTime   = (uint32_t)ShutterWidthPck - (CoarseIntegrationTime * pOV5630Ctx->LineLengthPck);

    // Write new integration time settings into the camera registers.
    // Do not use fine integration time. The behaviour of the OV5630 fine integration time register
    // is strange and unknown.
    // Do not write if nothing has changed.
    if ( CoarseIntegrationTime != pOV5630Ctx->OldCoarseIntegrationTime )
    {
        result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AECL_2, CoarseIntegrationTime );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        *pNumberOfFramesToSkip = 1U;                        // skip 1 frame
        pOV5630Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; // no frame skip
    }

    // result = OV5630_IsiReg_WriteIss( ulCamId, OV5630_LAEC_2, FineIntegrationTime );

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

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AGCL, &ulGainRegister );
    ulGainRegister &= 0x80; // keep only digital gain bit 7
    ulGainRegister += ( ucMultiplier + ucGain );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AGCL, ulGainRegister );

    // calculate gain & integration time actually set

    // Actual hardware gain may deviate by 1/16, 2/16, ... , 8/16 from NewGain.
    // Currently OV5630_MAX_GAIN_AEC is 8.0f, so max. deviation is 4/16.
    Gain = ucGain * ((ucMultiplier>>4)+1);

    IntegrationTime = ( (float)CoarseIntegrationTime * (float)pOV5630Ctx->LineLengthPck + (float)FineIntegrationTime ) / pOV5630Ctx->VtPixClkFreq;

    pOV5630Ctx->AecCurGain             = Gain;
    pOV5630Ctx->AecCurIntegrationTime  = IntegrationTime;

    //return current state
    *pSetGain            = pOV5630Ctx->AecCurGain;
    *pSetIntegrationTime = pOV5630Ctx->AecCurIntegrationTime;

    TRACE( OV5630_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime );
    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pOV5630Ctx->AecCurGain;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pIncr = 1.0f/16.0f;//FIXME

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    /* variables for gain */
    float   Gain                    = 0.0f;

    uint8_t  ucMultiplier           = 0U;
    uint8_t  ucGain                 = 0U;
    uint32_t ulGainRegister         = 0U;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    // The maximum and minimum gain is limited by the sensor.
    if ( NewGain > pOV5630Ctx->AecMaxGain )
    {
        Gain = pOV5630Ctx->AecMaxGain;
    }
    else if ( NewGain < 1.0f )
    {
        Gain = 1.0f;
    }
    else
    {
        Gain = NewGain;
    }

    // Calculate and set new gain register settings.
    // See mail from Omnivision FAE, analog gain = (Bit[6]+1)*(Bit[5]+1)*(Bit[4]+1)*(Bit[3:0]/16+1).
    // Lower multiplier bits have to be set before higher bits are set.

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

    result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_AGCL, &ulGainRegister );
    ulGainRegister &= 0x80; // keep only digital gain bit 7
    ulGainRegister += ( ucMultiplier + ucGain );
    result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AGCL, ulGainRegister );

    // Actual hardware gain may deviate by 1/16, 2/16, ... , 8/16 from NewGain.
    // Currently OV5630_MAX_GAIN_AEC is 8.0f, so max. deviation is 4/16.

    *pSetGain            = NewGain;

    pOV5630Ctx->AecCurGain             = NewGain;

    TRACE( OV5630_DEBUG, "%s: g=%f\n", __FUNCTION__, *pSetGain );
    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV5630Ctx->AecCurIntegrationTime;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIncr = (float)pOV5630Ctx->LineLengthPck / (float)pOV5630Ctx->VtPixClkFreq;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *
 * @param   handle                  OV5630 sensor instance handle
 * @param   NewIntegrationTime      integration time to be set
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    /* variables for integration time */
    uint32_t CoarseIntegrationTime  = 0UL;
    uint32_t FineIntegrationTime    = 0UL;
    float    ShutterWidthPck        = 0.0f;  // Shutter width in pixel clock periods

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // The maximum and minimum integration time is limited by the sensor.
    if ( NewIntegrationTime > (float)(pOV5630Ctx->FrameLengthLines * pOV5630Ctx->LineLengthPck) / pOV5630Ctx->VtPixClkFreq )
    {
        NewIntegrationTime = (float)(pOV5630Ctx->FrameLengthLines * pOV5630Ctx->LineLengthPck) / pOV5630Ctx->VtPixClkFreq;
    }
    else if ( NewIntegrationTime < 0.0f )
    {
        NewIntegrationTime = 0.0f;
    }

    // calculate and set new coarse and fine integration time register settings

    /**
     * The actual integration time is given by:
     * integration_time = ((coarse_integration_time*line_length_pck)+fine_integration_time)/(vt_pix_clk_freq)
     *
     * that leads to:
     * coarse_integration_time = (UINT32)((integration_time*vt_pix_clk_freq)/line_length_pck)
     * fine_integration_time   = (UINT32)((integration_time*vt_pix_clk_freq)-(coarse_integration_time*line_length_pck))
     */

    ShutterWidthPck = NewIntegrationTime * pOV5630Ctx->VtPixClkFreq;

    // calculate the integer part of the integration time in units of line length
    // avoid division by zero
    if ( pOV5630Ctx->LineLengthPck == 0U )
    {
        TRACE( OV5630_ERROR, "%s: LineLengthPck=0U (Division by zero !!!)\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / pOV5630Ctx->LineLengthPck );

    // calculate the fractional part of the integration time in units of pixel clocks
    // FineIntegrationTime   = (uint32_t)ShutterWidthPck - (CoarseIntegrationTime * pOV5630Ctx->LineLengthPck);

    // Write new integration time settings into the camera registers.
    // Do not use fine integration time. The behaviour of the OV5630 fine integration time register
    // is strange and unknown.
    // Do not write if nothing has changed.
    if ( CoarseIntegrationTime != pOV5630Ctx->OldCoarseIntegrationTime )
    {
        result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_AECL_2, CoarseIntegrationTime );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        pOV5630Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
    }

    // result = OV5630_IsiReg_WriteIss( ulCamId, OV5630_LAEC_2, FineIntegrationTime );

    NewIntegrationTime = ( (float)CoarseIntegrationTime * (float)pOV5630Ctx->LineLengthPck + (float)FineIntegrationTime ) / pOV5630Ctx->VtPixClkFreq;

    *pSetIntegrationTime = NewIntegrationTime;

    pOV5630Ctx->AecCurIntegrationTime  = NewIntegrationTime;

    TRACE( OV5630_DEBUG, "%s: Ti=%f\n", __FUNCTION__, *pSetIntegrationTime );
    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV5630_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        TRACE( OV5630_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV5630Ctx->AecCurGain;
    *pSetIntegrationTime = pOV5630Ctx->AecCurIntegrationTime;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibKFactor
 *
 * @brief   Returns the OV5630 specific K-Factor
 *
 * @param   handle       OV5630 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV5630_KFactor;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV5630 specific PCA-Matrix
 *
 * @param   handle          OV5630 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV5630_PCAMatrix;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV5630 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV5630_SVDMeanValue;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibCenterLine
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV5630 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiCenterLine = (IsiLine_t*)&OV5630_CenterLine;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV5630 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pIsiClipParam = (IsiAwbClipParm_t *)&OV5630_AwbClipParm;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV5630 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV5630_AwbGlobalFadeParm;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV5630 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV5630_AwbFade2Parm;

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetIlluProfile
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
static RESULT OV5630_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
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
        for ( i=0U; i<OV5630_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV5630_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV5630_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiGetLscMatrixTable
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
static RESULT OV5630_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
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
            default:
            {
                if (  (pOV5630Ctx->Config.Resolution == ISI_RES_TV720P30)
                        || (pOV5630Ctx->Config.Resolution == ISI_RES_TV720P60) )
                {
                    *pLscMatrixTable = &OV5630_LscMatrixTable_CIE_A_1280x720;
                }
                else if ( (pOV5630Ctx->Config.Resolution == ISI_RES_TV1080P24)
                        || (pOV5630Ctx->Config.Resolution == ISI_RES_TV1080P30) )
                {
                    *pLscMatrixTable = &OV5630_LscMatrixTable_CIE_A_1920x1080;
                }
                else if ( pOV5630Ctx->Config.Resolution == ISI_RES_2592_1944 )
                {
                    *pLscMatrixTable = &OV5630_LscMatrixTable_CIE_A_2592x1944;
                }
                else
                {
                    TRACE( OV5630_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

#if 0
            default:
            {
                TRACE( OV5630_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
#endif
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV5630 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pMaxStep = MAX_LOG;

    result = OV5630_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV5630 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0U, 0U };

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    nPosition = ( Position >= MAX_LOG ) ? 0U : ( MAX_REG - (Position * 16) );

    data[0] = (uint8_t)(0x40 | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0

    TRACE( OV5630_DEBUG, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    result = HalWriteI2CMem( pOV5630Ctx->IsiCtx.HalHandle,
                                pOV5630Ctx->IsiCtx.I2cAfBusNum,
                                pOV5630Ctx->IsiCtx.SlaveAfAddress,
                                0U,
                                pOV5630Ctx->IsiCtx.NrOfAfAddressBytes,
                                data,
                                2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV5630 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0U, 0U };

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV5630Ctx->IsiCtx.HalHandle,
                            pOV5630Ctx->IsiCtx.I2cAfBusNum,
                            pOV5630Ctx->IsiCtx.SlaveAfAddress,
                            0U,
                            pOV5630Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV5630_DEBUG, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

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

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV5630_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV5630 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV5630_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t ulRegValue = 0UL;

    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL01, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x32U );

        result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL01, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV5630_IsiRegReadIss( pOV5630Ctx, OV5630_ISP_CTRL01, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x02 );

        result = OV5630_IsiRegWriteIss( pOV5630Ctx, OV5630_ISP_CTRL01, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    TRACE( OV5630_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV5630_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
    	TRACE( OV5630_ERROR, "%s: pOV5630Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV5630_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV5630_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV5630_Context_t *pOV5630Ctx = (OV5630_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV5630_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV5630Ctx == NULL )
    {
    	TRACE( OV5630_ERROR, "%s: pOV5630Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV5630_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV5630_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *
 */
/*****************************************************************************/



/*****************************************************************************/
/**
 *          OV5630_IsiGetSensorIss
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
RESULT OV5630_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV5630_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                         = OV5630_g_acName;
        pIsiSensor->pRegisterTable                  = OV5630_g_aRegDescription;
        pIsiSensor->pIsiSensorCaps                  = &OV5630_g_IsiSensorDefaultConfig;

        pIsiSensor->pIsiCreateSensorIss             = OV5630_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss            = OV5630_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                  = OV5630_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss              = OV5630_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss   = OV5630_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss       = OV5630_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss           = OV5630_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss    = OV5630_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss        = OV5630_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss             = OV5630_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss            = OV5630_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss          = OV5630_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss            = OV5630_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss = OV5630_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss       = OV5630_IsiGetCurrentExposureIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor             = OV5630_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix           = OV5630_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue        = OV5630_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine          = OV5630_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam           = OV5630_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam     = OV5630_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam           = OV5630_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile              = OV5630_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable           = OV5630_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds         = OV5630_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive           = OV5630_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                 = OV5630_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                 = OV5630_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate           = OV5630_IsiMdiFocusCalibrate;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern         = OV5630_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV5630_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV5630_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV5630_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV5630_SLAVE_ADDR;
    pSensorI2cInfo->soft_reg_addr = OV5630_SYS;
    pSensorI2cInfo->soft_reg_value = 0x80;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    pSensorI2cInfo->resolution = ( ISI_RES_VGA
                                  | ISI_RES_2592_1944
                                  | ISI_RES_TV720P30
                                  | ISI_RES_TV720P60
                                  | ISI_RES_TV1080P24
                                  | ISI_RES_TV1080P30 );

    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = OV5630_PIDH;  
    pChipIDInfo_H->chipid_reg_value = OV5630_PIDH_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV5630_PIDL;
    pChipIDInfo_L->chipid_reg_value = OV5630_PIDL_DEFAULT;
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
    OV5630_IsiGetSensorIss,
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
    OV5630_IsiGetSensorI2cInfo,
};

