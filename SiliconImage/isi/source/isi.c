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
 * @file isisup.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"


/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( ISI_INFO , "ISI: ", INFO,    0);
CREATE_TRACER( ISI_WARN , "ISI: ", WARNING, 1);
CREATE_TRACER( ISI_ERROR, "ISI: ", ERROR,   1);



/*****************************************************************************/
/**
 *          IsiCreateSensorIss
 *
 * @brief   Creates a new camera instance.
 *
 * @param   pConfig     configuration of the new sensor instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t   *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    if ( pConfig->pSensor->pIsiCreateSensorIss == NULL )
    {
        (void)HalDelRef( pConfig->HalHandle );
        return ( RET_NOTSUPP );
    }

    result = pConfig->pSensor->pIsiCreateSensorIss( pConfig );
    if ( result != RET_SUCCESS )
    {
        (void)HalDelRef( pConfig->HalHandle );
    }

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiReleaseSensorIss
 *
 * @brief   Destroys a camera instance.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
RESULT IsiReleaseSensorIss
(
    IsiSensorHandle_t   handle
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)HalDelRef( pSensorCtx->HalHandle );

    if ( pSensorCtx->pSensor->pIsiReleaseSensorIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiReleaseSensorIss( pSensorCtx );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCapsIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   handle      Sensor instance handle
 * @param   pCaps
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCapsIss
(
    IsiSensorHandle_t   handle,
    IsiSensorCaps_t     *pCaps
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = pSensorCtx->pSensor->pIsiGetCapsIss( pSensorCtx, pCaps );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );

}



/*****************************************************************************/
/**
 *          IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiSetupSensorIss
(
    IsiSensorHandle_t   handle,
    IsiSensorConfig_t   *pConfig
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiSetupSensorIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiSetupSensorIss( pSensorCtx, pConfig );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiChangeSensorResolutionIss
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
RESULT IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pNumberOfFramesToSkip == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiChangeSensorResolutionIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiChangeSensorResolutionIss( pSensorCtx, Resolution, pNumberOfFramesToSkip );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiSensorSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NOTSUPP
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor == NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiSensorSetStreamingIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiSensorSetStreamingIss( pSensorCtx, on );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
RESULT IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor == NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiSensorSetPowerIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiSensorSetPowerIss( pSensorCtx, on );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiCheckSensorConnectionIss
 *
 * @brief   Performs the power-up sequence of the camera, if possible.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiCheckSensorConnectionIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiCheckSensorConnectionIss( pSensorCtx );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );

   //zyc ,for test
   //return RET_SUCCESS;
}



/*****************************************************************************/
/**
 *          IsiGetSensorRevisionIss
 *
 * @brief   reads the sensor revision register and returns this value
 *
 * @param   Handle      pointer to sensor description struct
 * @param   p_value     pointer to storage value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor == NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetSensorRevisionIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result =  pSensorCtx->pSensor->pIsiGetSensorRevisionIss( handle, p_value );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of a sensor
 *          instance
 *
 * @param   handle       sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetGainLimitsIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetGainLimitsIss( pSensorCtx, pMinGain, pMaxGain );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of a sensor
 *          instance
 *
 * @param   handle       sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetIntegrationTimeLimitsIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetIntegrationTimeLimitsIss( pSensorCtx, pMinIntegrationTime, pMaxIntegrationTime );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiExposureControlIss
 *
 * @brief   Sets the exposure minimal and maximal values of a sensor
 *          instance
 *
 * @param   handle       sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiExposureControlIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiExposureControlIss( pSensorCtx, NewGain, NewIntegrationTime, pNumberOfFramesToSkip, pSetGain, pSetIntegrationTime );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle       sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pCurGain,
    float               *pCurIntegrationTime
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pCurGain == NULL) || (pCurIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCurrentExposureIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCurrentExposureIss( pSensorCtx, pCurGain, pCurIntegrationTime );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV14825 sensor instance handle
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
RESULT IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetAfpsInfoIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    // clear info structure
    uint32_t idx;
    MEMSET( pAfpsInfo, 0, sizeof(*pAfpsInfo) );
    pAfpsInfo->AecMinGain           = 0.0f;
    pAfpsInfo->AecMaxGain           = 0.0f;
    pAfpsInfo->AecMinIntTime        = 0.0f;
    pAfpsInfo->AecMaxIntTime        = 0.0f;
    pAfpsInfo->AecSlowestResolution = 0;
    for( idx = 0; idx < ISI_NUM_AFPS_STAGES; idx++)
    {
        pAfpsInfo->Stage[idx].Resolution = 0;
        pAfpsInfo->Stage[idx].MaxIntTime = 0.0f;
    }
    pAfpsInfo->CurrResolution = 0;
    pAfpsInfo->CurrMinIntTime = 0.0f;
    pAfpsInfo->CurrMaxIntTime = 0.0f;

    result = pSensorCtx->pSensor->pIsiGetAfpsInfoIss( pSensorCtx, Resolution, pAfpsInfo );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetGainIss
 *
 *****************************************************************************/
RESULT IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetGainIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetGainIss( pSensorCtx, pSetGain );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetGainIncrementIss
 *
 *****************************************************************************/
RESULT IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetGainIncrementIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetGainIncrementIss( pSensorCtx, pIncr );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiSetGainIss
 *
 *****************************************************************************/
RESULT IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiSetGainIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiSetGainIss( pSensorCtx, NewGain, pSetGain );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetIntegrationTimeIss
 *
 *****************************************************************************/
RESULT IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetIntegrationTimeIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetIntegrationTimeIss( pSensorCtx, pSetIntegrationTime );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetIntegrationTimeIncrementIss
 *
 *****************************************************************************/
RESULT IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetIntegrationTimeIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetIntegrationTimeIncrementIss( pSensorCtx, pIncr );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiSetIntegrationTimeIss
 *
 *****************************************************************************/
RESULT IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    uint8_t NumberOfFramesToSkip = 0U;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiSetIntegrationTimeIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiSetIntegrationTimeIss( pSensorCtx, NewIntegrationTime, pSetIntegrationTime, &NumberOfFramesToSkip );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetResolutionIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSetResolution          current resolution
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetResolutionIss == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetResolutionIss( pSensorCtx, pSetResolution );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibKFactor
 *
 * @brief   Returns the sensor specific K-Factor
 *
 * @param   handle       sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibKFactor == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibKFactor( pSensorCtx, pIsiKFactor );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibPcaMatrix
 *
 * @brief   Returns the sensor specific PCA-Matrix
 *
 * @param   handle          sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibPcaMatrix == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibPcaMatrix( pSensorCtx, pIsiPcaMatrix );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibSvdMeanValue == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibSvdMeanValue( pSensorCtx, pIsiSvdMeanValue );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibCenterLine
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              sensor instance handle
 * @param   ptIsiCenterLine     Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibCenterLine == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibCenterLine( pSensorCtx, ptIsiCenterLine );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              sensor instance handle
 * @param   ptIsiClipParam      Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **ptIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibClipParam == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibClipParam( pSensorCtx, ptIsiClipParam );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle               sensor instance handle
 * @param   ptIsiGlobalFadeParam Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibGlobalFadeParam == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibGlobalFadeParam( pSensorCtx, ptIsiGlobalFadeParam );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle          sensor instance handle
 * @param   ptIsiFadeParam  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetCalibFadeParam == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetCalibFadeParam( pSensorCtx, ptIsiFadeParam );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetIlluProfile
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
RESULT IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetIlluProfile == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetIlluProfile( pSensorCtx, CieProfile, ptIsiIlluProfile );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiGetLscMatrixTable
 *
 * @brief   Returns a pointer to vignetted LSC-Matrices identified by
 *          CieProfile bitmask for a configured resolution
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   pLscMatrixTable     Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor==NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiGetLscMatrixTable == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiGetLscMatrixTable( pSensorCtx, CieProfile, pLscMatrixTable );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiMdiInitMotoDrive
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle          sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiMdiInitMotoDrive
(
    IsiSensorHandle_t   handle
)
{
//for test,zyc
  //  return RET_SUCCESS;
    
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor == NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiMdiInitMotoDriveMds == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiMdiInitMotoDriveMds( pSensorCtx );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
//for test,zyc
//    return RET_SUCCESS;
    
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( (pSensorCtx == NULL) || (pSensorCtx->pSensor == NULL) )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiMdiSetupMotoDrive == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiMdiSetupMotoDrive( pSensorCtx, pMaxStep );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      AbsStep
)
{
//for test,zyc
 //   return RET_SUCCESS;

    
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiMdiFocusSet == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiMdiFocusSet( pSensorCtx, AbsStep );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
//for test,zyc
//    return RET_SUCCESS;
    
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->pSensor->pIsiMdiFocusGet == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiMdiFocusGet( pSensorCtx, pAbsStep );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
//for test,zyc
 //   return RET_SUCCESS;
    
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiMdiFocusCalibrate == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiMdiFocusCalibrate( pSensorCtx );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiActivateTestPattern
 *
 * @brief   Activates or deactivates sensor's test-pattern (normally a defined
 *          colorbar )
 *
 * @param   handle          sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pIsiActivateTestPattern == NULL )
    {
        return ( RET_NOTSUPP );
    }

    result = pSensorCtx->pSensor->pIsiActivateTestPattern( pSensorCtx, enable );

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IsiDumpAllRegisters
 *
 * @brief   Activates or deactivates sensor's test-pattern (normally a defined
 *          colorbar )
 *
 * @param   handle          sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IsiDumpAllRegisters
(
    IsiSensorHandle_t   handle,
    const uint8_t       *filename
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( ISI_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pSensorCtx->pSensor->pRegisterTable == NULL )
    {
        return ( RET_NOTSUPP );
    }

    const IsiRegDescription_t* ptReg = pSensorCtx->pSensor->pRegisterTable;

    FILE* pFile;
    pFile = fopen ( (const char *)filename, "w" );

    fprintf(pFile,"*************************************************************\n");
    fprintf(pFile,"* IMAGE SENSOR REGISTERS                                    *\n");
    fprintf(pFile,"*************************************************************\n");

    while (ptReg->Flags != eTableEnd)
    {
        if (ptReg->Flags & (eReadWrite))
        {
            fprintf(pFile, " %-30s @ 0x%04X", ptReg->pName, ptReg->Addr);

            if (ptReg->Flags & eReadable)
            {
                uint32_t value = 0U;
                RESULT Res = RET_SUCCESS;
                Res = IsiReadRegister( handle, ptReg->Addr, &value );
                if (Res == RET_SUCCESS)
                {
                    fprintf(pFile, " = 0x%08X", value);
                    if (ptReg->Flags & eNoDefault)
                    {
                        fprintf(pFile, "\n");
                    }
                    else if (value == ptReg->DefaultValue)
                    {
                        fprintf(pFile, " (= default value)\n");
                    }
                    else
                    {
                        fprintf(pFile, " (default was 0x%08X)\n", ptReg->DefaultValue);
                    }
                }
                else
                {
                    fprintf(pFile, " <read failure %d>\n", Res);
                }
            }
            else if (ptReg->Flags & eWritable)
            {
                fprintf(pFile, " <is only writable>\n");
            }
        }
        ++ptReg;
    }

    fclose (pFile);

    TRACE( ISI_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IsiTryToSetConfigFromPreferredCaps
 *
 * @brief   Tries to set the referenced sensor config parameter to the first of the
 *          given preferred capabilities that is included in the given capability
 *          mask. If none of the preferred capabilities is supported, the config
 *          parameter value remains unchanged.
 *
 * @note    Use this function for example to modify the retrieved default sensor
 *          config parameter for parameter according to some external preferences
 *          while taking the retrieved sensor capabilities for that config parameter
 *          into account.
 *
 * @param   pConfigParam    reference to parameter of sensor config structure
 * @param   prefList        reference to 0 (zero) terminated array of preferred
 *                          capability values in descending order
 * @param   capsmask        bitmask of supported capabilites for that parameter
 *
 * @return  Return the result of the function call.
 * @retval  BOOL_TRUE       preferred capability set in referenced config parameter
 * @retval  BOOL_FALSE      preferred capability not supported
 *
 *****************************************************************************/
bool_t IsiTryToSetConfigFromPreferredCaps
(
    uint32_t    *pConfigParam,
    uint32_t    *prefList,
    uint32_t    capsmask
)
{
    uint32_t i;

    for(i=0; prefList[i]!=0 ; i++)
    {
        if( (capsmask & prefList[i]) != 0 )
        {
            break;
        }
    }

    if (prefList[i] != 0)
    {
        *pConfigParam = prefList[i];
        return ( BOOL_TRUE );
    }

    return ( BOOL_FALSE );
}

/*****************************************************************************/
/**
 *          IsiTryToSetConfigFromPreferredCap
 *
 * @brief   Tries to set referenced sensor config parameter to the given preferred
 *          capability while checking that capability against the given capability
 *          mask. If that capability isn't supported, the config parameter value
 *          remains unchanged.
 *
 * @note    Use this function for example to modify the retrieved default sensor
 *          config parameter for parameter according to some external preferences
 *          while taking the retrieved sensor capabilities for that config parameter
 *          into account.
 *
 * @param   pConfigParam    reference to parameter of sensor config structure
 * @param   prefcap         preferred capability value
 * @param   capsmask        bitmask of supported capabilites for that parameter
 *
 * @return  Return the result of the function call.
 * @retval  BOOL_TRUE       preferred capability set in referenced config parameter
 * @retval  BOOL_FALSE      preferred capability not supported
 *
 *****************************************************************************/
bool_t IsiTryToSetConfigFromPreferredCap
(
    uint32_t        *pConfigParam,
    const uint32_t  prefcap,
    const uint32_t  capsmask
)
{
    if( (capsmask & prefcap) != 0 )
    {
        *pConfigParam = prefcap;
        return ( BOOL_TRUE );
    }

    return ( BOOL_FALSE );
}

/*****************************************************************************/
/**
 *          IsiGetResolutionName
 *
 * @brief   Returns a pointer to the zero terminated name of the given resolution.
 *
 * @param   Resolution      resolution to query name for
 * @param   pszName         reference to string pointer
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFRANGE  pszName nevertheless points to a valid string
 *
 *****************************************************************************/
RESULT IsiGetResolutionName
(
    uint32_t    Resolution,
    char        **pszName
)
{
    if (pszName == NULL)
    {
        return RET_NULL_POINTER;
    }

    switch(Resolution)
    {
        case ISI_RES_VGA      : *pszName = "VGA"         ; return ( RET_SUCCESS );
        case ISI_RES_2592_1944: *pszName = "2592x1944"   ; return ( RET_SUCCESS );
        case ISI_RES_3264_2448: *pszName = "3264x2448"   ; return ( RET_SUCCESS );
        case ISI_RES_4416_3312: *pszName = "4416x3312"   ; return ( RET_SUCCESS );
        case ISI_RES_TV720P5  : *pszName = "1280x720P5"  ; return ( RET_SUCCESS );
        case ISI_RES_TV720P15 : *pszName = "1280x720P15" ; return ( RET_SUCCESS );
        case ISI_RES_TV720P30 : *pszName = "1280x720P30" ; return ( RET_SUCCESS );
        case ISI_RES_TV720P60 : *pszName = "1280x720P60" ; return ( RET_SUCCESS );
        case ISI_RES_TV1080P5 : *pszName = "1920x1080P5" ; return ( RET_SUCCESS );
        case ISI_RES_TV1080P6 : *pszName = "1920x1080P6" ; return ( RET_SUCCESS );
        case ISI_RES_TV1080P10: *pszName = "1920x1080P10"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P12: *pszName = "1920x1080P12"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P15: *pszName = "1920x1080P15"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P20: *pszName = "1920x1080P20"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P24: *pszName = "1920x1080P24"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P25: *pszName = "1920x1080P25"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P30: *pszName = "1920x1080P30"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P50: *pszName = "1920x1080P50"; return ( RET_SUCCESS );
        case ISI_RES_TV1080P60: *pszName = "1920x1080P60"; return ( RET_SUCCESS );

        default: break; // make lint happy ;-)
    }

    *pszName = "<unknown resolution>";

    return ( RET_OUTOFRANGE );
}


/*****************************************************************************/
/**
 *          IsiGetResolutionParam
 *
 * @brief   Returns a 
 *
 * @param   Resolution      resolution to query parameter for
 * @param   width           reference to width
 * @param   height          reference to height
 * @param   fps             reference to framerate per second
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFRANGE  pszName nevertheless points to a valid string
 *
 *****************************************************************************/
RESULT IsiGetResolutionParam
(
    uint32_t    Resolution,
    uint16_t    *width,
    uint16_t    *height,
    uint16_t    *fps
)
{
    if ( (NULL==width) || (NULL==height) || (NULL==fps) )
    {
        return RET_NULL_POINTER;
    }

    switch( Resolution )
    {
        case ISI_RES_VGA      : *width =  640; *height =  480; *fps =  0; return ( RET_SUCCESS );
        case ISI_RES_2592_1944: *width = 2592; *height = 1944; *fps =  0; return ( RET_SUCCESS );
        case ISI_RES_3264_2448: *width = 3264; *height = 2448; *fps =  0; return ( RET_SUCCESS );
        case ISI_RES_4416_3312: *width = 4416; *height = 3312; *fps =  0; return ( RET_SUCCESS );
        case ISI_RES_TV720P5  : *width = 1280; *height =  720; *fps =  5; return ( RET_SUCCESS );
        case ISI_RES_TV720P15 : *width = 1280; *height =  720; *fps = 15; return ( RET_SUCCESS );
        case ISI_RES_TV720P30 : *width = 1280; *height =  720; *fps = 30; return ( RET_SUCCESS );
        case ISI_RES_TV720P60 : *width = 1280; *height =  720; *fps = 60; return ( RET_SUCCESS );
        case ISI_RES_TV1080P5 : *width = 1920; *height = 1080; *fps =  5; return ( RET_SUCCESS );
        case ISI_RES_TV1080P6 : *width = 1920; *height = 1080; *fps =  6; return ( RET_SUCCESS );
        case ISI_RES_TV1080P10: *width = 1920; *height = 1080; *fps = 10; return ( RET_SUCCESS );
        case ISI_RES_TV1080P12: *width = 1920; *height = 1080; *fps = 12; return ( RET_SUCCESS );
        case ISI_RES_TV1080P15: *width = 1920; *height = 1080; *fps = 15; return ( RET_SUCCESS );
        case ISI_RES_TV1080P20: *width = 1920; *height = 1080; *fps = 20; return ( RET_SUCCESS );
        case ISI_RES_TV1080P24: *width = 1920; *height = 1080; *fps = 24; return ( RET_SUCCESS );
        case ISI_RES_TV1080P25: *width = 1920; *height = 1080; *fps = 25; return ( RET_SUCCESS );
        case ISI_RES_TV1080P30: *width = 1920; *height = 1080; *fps = 30; return ( RET_SUCCESS );
        case ISI_RES_TV1080P50: *width = 1920; *height = 1080; *fps = 50; return ( RET_SUCCESS );
        case ISI_RES_TV1080P60: *width = 1920; *height = 1080; *fps = 60; return ( RET_SUCCESS );

        default: break; // make lint happy ;-)
    }

    *width  = 0UL;
    *height = 0UL;
    *fps    = 0UL;
    
    return ( RET_OUTOFRANGE );
}

