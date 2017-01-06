/*****************************************************************************/
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/

#ifndef __OV8865_PRIV_H__
#define __OV8865_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>



#ifdef __cplusplus
extern "C"
{
#endif

/*
*              SILICONIMAGE LIBISP VERSION NOTE
*
*v0.1.0x00 : first version,2015-10-09

*/


#define CONFIG_SENSOR_DRV_VERSION  KERNEL_VERSION(1, 2, 0)

/*****************************************************************************
 * System control registers
 *****************************************************************************/
//hkw  
#define SENSOR_MODE_SELECT                  (0x0100) // rw - Bit[7:1]not used  Bit[0]Streaming set 0: software_standby  1: streaming       
#define SENSOR_MODE_SELECT_OFF              (0x00U)
#define SENSOR_MODE_SELECT_ON				(0x01U)

#define SENSOR_SOFTWARE_RST                 (0x0103) // rw - Bit[7:1]not used  Bit[0]software_reset
#define SENSOR_SOFTWARE_RST_VALUE			(0x01)

#define SENSOR_CHIP_ID_HIGH_BYTE            (0x300a) // r - 
#define SENSOR_CHIP_ID_MIDDLE_BYTE          (0x300b) // r - 
#define SENSOR_CHIP_ID_LOW_BYTE             (0x300c) // r -  
#define SENSOR_CHIP_ID_HIGH_BYTE_DEFAULT            (0x00) // r - 
#define SENSOR_CHIP_ID_MIDDLE_BYTE_DEFAULT          (0x88) // r - 
#define SENSOR_CHIP_ID_LOW_BYTE_DEFAULT             (0x65) // r - 

                                                    //     Bit[3] gain manual enable 0:manual disable use register 0x350a/0x350b   1:manual enable use register 0x3504/0x3505
#define SENSOR_AEC_AGC_ADJ_H                (0x3508) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define SENSOR_AEC_AGC_ADJ_L                (0x3509) // rw- Bit[7:0]gain output to sensor Gain[7:0] 

#define SENSOR_AEC_EXPO_H                   (0x3500) // rw- Bit[3:0] exposure[19:16]
#define SENSOR_AEC_EXPO_M                   (0x3501) // rw- Bit[7:0] exposure[15:8]
#define SENSOR_AEC_EXPO_L                   (0x3502) // rw- Bit[7:0] exposure[7:0] low 4 bits are fraction bits which are not supportted and should always be 0.



/*****************************************************************************
 * ov14825 context structure
 *****************************************************************************/
typedef struct Sensor_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} Sensor_VcmInfo_t;
typedef struct Sensor_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    //// modify below here ////

    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that sensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    bool_t              isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    bool_t              GroupHold;

    float               VtPixClkFreq;           /**< pixel clock */
    uint16_t            LineLengthPck;          /**< line length with blanking */
    uint16_t            FrameLengthLines;       /**< frame line length */

    float               AecMaxGain;
    float               AecMinGain;
    float               AecMaxIntegrationTime;
    float               AecMinIntegrationTime;

    float               AecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    float               AecCurGain;
    float               AecCurIntegrationTime;

    uint16_t            OldGain;               /**< gain multiplier */
    uint32_t            OldCoarseIntegrationTime;
    uint32_t            OldFineIntegrationTime;

    IsiSensorMipiInfo   IsiSensorMipiInfo;
	Sensor_VcmInfo_t    VcmInfo; 
} Sensor_Context_t;

#ifdef __cplusplus
}
#endif

#endif
