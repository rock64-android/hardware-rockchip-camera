#ifndef __HM8040_PRIV_H__
#define __HM8040_PRIV_H__

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
*v0.1.0 : first version

*/


#define CONFIG_SENSOR_DRV_VERSION  KERNEL_VERSION(0, 1, 0)

/*****************************************************************************
 * System control registers
 *****************************************************************************/
#define HM8040_MODE_SELECT                  (0x0100) // rw - Bit[7:1]not used  Bit[0]Streaming set 0: software_standby  1: streaming       
#define HM8040_MODE_SELECT_OFF              (0x00U)
#define HM8040_MODE_SELECT_ON				(0x01U)

#define HM8040_SOFTWARE_RST                 (0x0103) // rw - Bit[7:1]not used  Bit[0]software_reset
#define HM8040_SOFTWARE_RST_VALUE			(0x00)

#define HM8040_CHIP_ID_HIGH_BYTE            (0x0000) // r - 
#define HM8040_CHIP_ID_MIDDLE_BYTE          (0x0001) // r - 
#define HM8040_CHIP_ID_LOW_BYTE             (0x0002) // r -  
#define HM8040_CHIP_ID_HIGH_BYTE_DEFAULT    (0x80) // r - 
#define HM8040_CHIP_ID_MIDDLE_BYTE_DEFAULT  (0x40) // r - 
#define HM8040_CHIP_ID_LOW_BYTE_DEFAULT     (0x04) //r-

#define HM8040_AEC_AGC_ADJ                	(0x0205) // rw- Bit[7:0] gain output to sensor Gain

#define HM8040_AEC_EXPO_H                   (0x0202) // rw- Bit[7:0]  exposure[15:8]
#define HM8040_AEC_EXPO_L                   (0x0203) // rw- Bit[7:0]  exposure[7:0] 



/*****************************************************************************
 * ov14825 context structure
 *****************************************************************************/
typedef struct HM8040_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} HM8040_VcmInfo_t;
typedef struct HM8040_Context_s
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
	HM8040_VcmInfo_t    VcmInfo; 
} HM8040_Context_t;

#ifdef __cplusplus
}
#endif

#endif
