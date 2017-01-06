//HM8040 the same with ov14825

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
 * @file HM8040.c
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

#include "HM8040_MIPI_priv.h"

#define  HM8040_NEWEST_TUNING_XML "17-8-2016_wsq_hm8040_mtd8040_v1.0.1"

//hkw no use;
#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 ****************************************************************************/
CREATE_TRACER( HM8040_INFO , "HM8040: ", INFO,    0U );
CREATE_TRACER( HM8040_WARN , "HM8040: ", WARNING, 1U );
CREATE_TRACER( HM8040_ERROR, "HM8040: ", ERROR,   1U );
CREATE_TRACER( HM8040_DEBUG, "HM8040: ", INFO,     0U );
CREATE_TRACER( HM8040_NOTICE0 , "HM8040: ", TRACE_NOTICE0, 1);
CREATE_TRACER( HM8040_NOTICE1, "HM8040: ", TRACE_NOTICE1, 1U );


#define HM8040_SLAVE_ADDR       0x48U                     /**< i2c slave address of the HM8040 camera sensor */
#define HM8040_SLAVE_ADDR2      0x68U
#define HM8040_SLAVE_AF_ADDR    0x18U                      /**< i2c slave address of the HM8040 integrated AD5820 */
#define Sensor_OTP_SLAVE_ADDR   0x48U
#define Sensor_OTP_SLAVE_ADDR2  0x68U

#define HM8040_MAXN_GAIN 		(16.0f)
#define HM8040_MIN_GAIN_STEP   	(1.0f / HM8040_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define HM8040_MAX_GAIN_AEC    	(8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
const char HM8040_g_acName[] = "HM8040_MIPI";
//extern const IsiRegDescription_t HM8040_g_aRegDescription[];
extern const IsiRegDescription_t HM8040_g_aRegDescription_onelane[];
extern const IsiRegDescription_t HM8040_g_aRegDescription_twolane[];
extern const IsiRegDescription_t HM8040_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t HM8040_g_1632x1224_onelane[];
extern const IsiRegDescription_t HM8040_g_1632x1224_twolane[];
//extern const IsiRegDescription_t HM8040_g_1632x1224P20_twolane[];
//extern const IsiRegDescription_t HM8040_g_1632x1224P10_twolane[];
extern const IsiRegDescription_t HM8040_g_1632x1224P30_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P25_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P20_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P15_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P10_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224_fourlane[];
extern const IsiRegDescription_t HM8040_g_1632x1224P30_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P25_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P20_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P15_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_1632x1224P10_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448_onelane[];
extern const IsiRegDescription_t HM8040_g_3264x2448_twolane[];
extern const IsiRegDescription_t HM8040_g_3264x2448P15_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P7_twolane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448_fourlane[];
extern const IsiRegDescription_t HM8040_g_3264x2448P30_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P25_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P20_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P15_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P10_fourlane_fpschg[];
extern const IsiRegDescription_t HM8040_g_3264x2448P7_fourlane_fpschg[];

const IsiSensorCaps_t HM8040_g_IsiSensorDefaultConfig;

#define HM8040_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define HM8040_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define HM8040_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_FOUR_LANE


/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT HM8040_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT HM8040_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT HM8040_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT HM8040_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT HM8040_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT HM8040_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT HM8040_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT HM8040_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT HM8040_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT HM8040_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT HM8040_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT HM8040_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT HM8040_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT HM8040_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT HM8040_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT HM8040_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT HM8040_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT HM8040_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT HM8040_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT HM8040_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT HM8040_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT HM8040_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT HM8040_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT HM8040_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT HM8040_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT HM8040_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT HM8040_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT HM8040_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT HM8040_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT HM8040_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT HM8040_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT HM8040_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT HM8040_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT HM8040_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT HM8040_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT HM8040_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT HM8040_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT HM8040_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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

/* OTP START*/
static int HM8040_read_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address
){
    uint32_t temp = 0;
    if(HM8040_IsiRegReadIss(handle,address,&temp) != RET_SUCCESS){
        TRACE( HM8040_ERROR, "%s read OTP register 0x%x erro!\n", __FUNCTION__,address);
    }
    return temp;
}

static RESULT HM8040_write_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
){
    RESULT result = RET_SUCCESS;
    if((result = HM8040_IsiRegWriteIss(handle,address,value)) != RET_SUCCESS){
        TRACE( HM8040_ERROR, "%s write OTP register (0x%x,0x%x) erro!\n", __FUNCTION__,address,value);
    }
    return result;
}

struct otp_struct {
    int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
    int module_integrator_id;
    int lens_id;
    int production_year;
    int production_month;
    int production_day;
    int rg_ratio;
    int bg_ratio;
    int lenc[240];
	int checksum;
    int VCM_start;
    int VCM_end;
    int VCM_dir;
};

static struct otp_struct g_otp_info ={0};

//for test,just for compile
int  RG_Ratio_Typical=0x100;
int  BG_Ratio_Typical=0x100;

#define  RG_Ratio_Typical_RK3288 (0x144)
#define  BG_Ratio_Typical_RK3288 (0x12f)

static int check_read_otp_rk3288(
    sensor_i2c_write_t*  sensor_i2c_write_p,
    sensor_i2c_read_t*  sensor_i2c_read_p,
    void* context,
    int camsys_fd
)
{
    struct otp_struct *otp_ptr = &g_otp_info;
	int otp_flag=0x0, addr, temp, temp1, i;
    int i2c_base_info[3];

    i2c_base_info[0] = 0; //otp i2c addr
    i2c_base_info[1] = 2; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size

	TRACE( HM8040_ERROR,  "%s: not support right now.\n",  __FUNCTION__ );
	
	return RET_NOTSUPP;

}

static int check_read_otp(
	sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	void* context,
	int camsys_fd
)
{
	return	check_read_otp_rk3288(sensor_i2c_write_p, sensor_i2c_read_p, context, camsys_fd);
}

static int apply_otp_rk3288(IsiSensorHandle_t   handle,struct otp_struct *otp_ptr)
{
	int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
	
	// apply OTP WB Calibration
	if ((*otp_ptr).flag & 0x40) {
		rg = (*otp_ptr).rg_ratio;
		bg = (*otp_ptr).bg_ratio;
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
			Base_gain = R_gain;
			else
			Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);
		// update sensor WB gain
		if (R_gain>0x400) {
			HM8040_write_i2c(handle, 0x5032, R_gain>>8);
			HM8040_write_i2c(handle, 0x5033, R_gain & 0x00ff);
		}
		if(G_gain>0x400) {
			HM8040_write_i2c(handle, 0x5034, G_gain>>8);
			HM8040_write_i2c(handle, 0x5035, G_gain & 0x00ff);
		}
		if(B_gain>0x400) {
			HM8040_write_i2c(handle, 0x5036, B_gain>>8);
			HM8040_write_i2c(handle, 0x5037, B_gain & 0x00ff);
		}
	}
	// apply OTP Lenc Calibration
	if ((*otp_ptr).flag & 0x10) {
		temp = HM8040_read_i2c(handle, 0x5000);
		temp = 0x80 | temp;
		HM8040_write_i2c(handle, 0x5000, temp);
		for(i=0;i<240;i++) {
			HM8040_write_i2c(handle, 0x5800 + i, (*otp_ptr).lenc[i]);
		}
	}
	TRACE( HM8040_NOTICE0,  "%s: success!!!\n",  __FUNCTION__ );
	return (*otp_ptr).flag;
}


/* OTP END*/


/*****************************************************************************/
/**
 *          HM8040_IsiCreateSensorIss
 *
 * @brief   This function creates a new HM8040 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT HM8040_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    HM8040_Context_t *pHM8040Ctx;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pHM8040Ctx = ( HM8040_Context_t * )malloc ( sizeof (HM8040_Context_t) );
    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR,  "%s: Can't allocate HM8040 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pHM8040Ctx, 0, sizeof( HM8040_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pHM8040Ctx );
        return ( result );
    }
    
    pHM8040Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pHM8040Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pHM8040Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pHM8040Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? HM8040_SLAVE_ADDR : pConfig->SlaveAddr;
    pHM8040Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pHM8040Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pHM8040Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? HM8040_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pHM8040Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pHM8040Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pHM8040Ctx->Configured             = BOOL_FALSE;
    pHM8040Ctx->Streaming              = BOOL_FALSE;
    pHM8040Ctx->TestPattern            = BOOL_FALSE;
    pHM8040Ctx->isAfpsRun              = BOOL_FALSE;

	/* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pHM8040Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pHM8040Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pHM8040Ctx->VcmInfo.RatedCurrent   = pHM8040Ctx->VcmInfo.StartCurrent + MAX_LOG*pHM8040Ctx->VcmInfo.Step;
    pHM8040Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pHM8040Ctx->IsiSensorMipiInfo.sensorHalDevID = pHM8040Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( HM8040_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pHM8040Ctx;

    result = HalSetCamConfig( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an HM8040 sensor instance.
 *
 * @param   handle      HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT HM8040_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)HM8040_IsiSensorSetStreamingIss( pHM8040Ctx, BOOL_FALSE );
    (void)HM8040_IsiSensorSetPowerIss( pHM8040Ctx, BOOL_FALSE );

    (void)HalDelRef( pHM8040Ctx->IsiCtx.HalHandle );

    MEMSET( pHM8040Ctx, 0, sizeof( HM8040_Context_t ) );
    free ( pHM8040Ctx );

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCapsIss
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
static RESULT HM8040_IsiGetCapsIssInternal
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
        if (mipi_lanes == SUPPORT_MIPI_FOUR_LANE) {            
            switch (pIsiSensorCaps->Index) 
            {  
            	#if 1
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P30;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P25;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P20;
                    break;
                }
                case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P15;
                    break;
                }
                case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P10;
                    break;
                }
                case 5:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P7;
                    break;
                }
                case 6:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P30;
                    break;
                }
                case 7:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P25;
                    break;
                }
                case 8:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P20;
                    break;
                }
                case 9:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P15;
                    break;
                }
                case 10:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P10;
                    break;
                }
				#else
				case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P20;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P20;
                    break;
                }
				#endif
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
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P15;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P7;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P30;
                    break;
                }

                case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P25;
                    break;
                }

                case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P20;
                    break;
                }

                case 5:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P15;
                    break;
                }
                
                case 6:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P10;
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
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P7;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1632_1224P15;
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
        pIsiSensorCaps->BPat            = ISI_BPAT_RGRGGBGB;	//ISI_BPAT_RGRGGBGB
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS; //hsync?
        pIsiSensorCaps->VPol            = ISI_VPOL_NEG; //VPolarity
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING; //?
        pIsiSensorCaps->Bls             = ISI_BLS_OFF; //close;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;//close;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;//close;<
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );//close;
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );//坏点

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
    return result;
}
 
static RESULT HM8040_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = HM8040_IsiGetCapsIssInternal(pIsiSensorCaps,pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t HM8040_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_RGRGGBGB,          // BPat	//ISI_BPAT_RGRGGBGB
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_NEG,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_3264_2448P15, 
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_F11,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_RAW_10,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          HM8040_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      HM8040 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT HM8040_SetupOutputFormat
(
    HM8040_Context_t       *pHM8040Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s%s (enter)\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by HM8040 sensor, so the YCSequence parameter is not checked */
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
            TRACE( HM8040_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )   
    {
        case ISI_BPAT_RGRGGBGB:	//ISI_BPAT_BGBGGRGR
        {
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( HM8040_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( HM8040_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( HM8040_INFO, "%s%s (exit)\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}


int HM8040_get_PCLK( HM8040_Context_t *pHM8040Ctx, int XVCLK)
{
    // calculate PCLK
    uint32_t SCLK, temp1;
	int sys_clk_div,Pll_multiplier,pll_pre_div;

    HM8040_IsiRegReadIss(  pHM8040Ctx, 0x0303, &temp1 );
    sys_clk_div = temp1 & 0x03;

	HM8040_IsiRegReadIss(  pHM8040Ctx, 0x0305, &temp1 );
	pll_pre_div = temp1 & 0x1F;

	HM8040_IsiRegReadIss(  pHM8040Ctx, 0x0307, &temp1 );
	Pll_multiplier = temp1 & 0x7F;

	TRACE( HM8040_INFO, "%s XVCLK %d %d %d %d\n", __FUNCTION__,XVCLK,Pll_multiplier,pll_pre_div,sys_clk_div);

	SCLK = XVCLK * Pll_multiplier / pll_pre_div / sys_clk_div;

	TRACE( HM8040_INFO, "%s SCLK %d\n", __FUNCTION__,SCLK);

	return SCLK;
}

/*****************************************************************************/
/**
 *          HM8040_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      HM8040 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT HM8040_SetupOutputWindowInternal
(
    HM8040_Context_t        *pHM8040Ctx,
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
    int xclk = 24000000;	//24Mhz
    
	TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	
		pHM8040Ctx->IsiSensorMipiInfo.ulMipiFreq = 720;
		switch ( pConfig->Resolution )
		{
			case ISI_RES_1632_1224P15:
			{				
				if (set2Sensor == BOOL_TRUE) {
				    TRACE( HM8040_NOTICE1, "%s(%d): Resolution 1632x1224\n", __FUNCTION__,__LINE__ );
    				result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224_onelane);
    				if ( result != RET_SUCCESS )
    				{
    					return ( result );
    				}
    		    }
				usTimeHts = 0x0f10; //hkw
				usTimeVts = 0x04dc;
				/* sleep a while, that sensor can take over new default values */
				osSleep( 10 );
				break;
				
			}
			
			case ISI_RES_3264_2448P7:
			{				
				if (set2Sensor == BOOL_TRUE) {
				    TRACE( HM8040_NOTICE1, "%s(%d): Resolution 3264x2448\n", __FUNCTION__,__LINE__ );
    				result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448_onelane);
    				if ( result != RET_SUCCESS )
    				{
    					return ( result );
    				}
                }
				usTimeHts = 0x0f28;
				usTimeVts = 0x09aa;
				/* sleep a while, that sensor can take over new default values */
				osSleep( 10 );
				break;
				
			}
	
			default:
			{
				TRACE( HM8040_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
				return ( RET_NOTSUPP );
			}
		}
	} 
	else if(pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

        pHM8040Ctx->IsiSensorMipiInfo.ulMipiFreq = 400;
		
    	switch ( pConfig->Resolution )
        {
            case ISI_RES_1632_1224P30:
            case ISI_RES_1632_1224P25:
            case ISI_RES_1632_1224P20:
            case ISI_RES_1632_1224P15:
            case ISI_RES_1632_1224P10:            
            {
                if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
						result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224_twolane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P30_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P25_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P20_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P10_twolane_fpschg);
                    }

					HM8040_IsiRegWriteIss(pHM8040Ctx,0x0104,0x00);
        		}

    			usTimeHts = 0x0e16;
				
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x050E;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0x0610;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0x07c6;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x0a30;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x0fcc;
                }
                
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
            }

            case ISI_RES_3264_2448P7:
            case ISI_RES_3264_2448P15:
            {
                if (set2Sensor == BOOL_TRUE) {
                    if (res_no_chg == BOOL_FALSE) {
						result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448_twolane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P7_twolane_fpschg);
                    }
					
        		    HM8040_IsiRegWriteIss(pHM8040Ctx,0x0104,0x00);
        		}
        		
    			usTimeHts = 0x0e7A;
				
                if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x13e0;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x2d70;
                }

    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
            }

            default:
            {
                TRACE( HM8040_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
    } 
	else if(pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE) {

		pHM8040Ctx->IsiSensorMipiInfo.ulMipiFreq = 400;//384
		
        switch ( pConfig->Resolution )
        {
            case ISI_RES_1632_1224P30:
            case ISI_RES_1632_1224P25:
            case ISI_RES_1632_1224P20:
            case ISI_RES_1632_1224P15:
            case ISI_RES_1632_1224P10:            
            {
                if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
						result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224_fourlane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_1632x1224P10_fourlane_fpschg);
                    }

					HM8040_IsiRegWriteIss(pHM8040Ctx,0x0104,0x00);
        		}

    			usTimeHts = 0x0e16; //0x1c2c 0x0e16
				
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x050E;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0x0610;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0x07c6;	//0x0795
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x0a30;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x0fcc;
                }

				/* sleep a while, that sensor can take over new default values */
    		   	osSleep( 10 );
				TRACE( HM8040_INFO,"Resolution 1632x1224 %dfps usTimeHts %d usTimeVts %d\n",ISI_FPS_GET(pConfig->Resolution),usTimeHts,usTimeVts);
							
    			break;
            }

            case ISI_RES_3264_2448P7:
            case ISI_RES_3264_2448P10:
            case ISI_RES_3264_2448P15:
            case ISI_RES_3264_2448P20:
            case ISI_RES_3264_2448P25:
            case ISI_RES_3264_2448P30:
            {
                if (set2Sensor == BOOL_TRUE) {
                    if (res_no_chg == BOOL_FALSE) {
						result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448_fourlane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P10_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_3264x2448P7_fourlane_fpschg);
                    }

					HM8040_IsiRegWriteIss(pHM8040Ctx,0x0104,0x00);
        		}
        		
    			usTimeHts = 0x0e7A; 		//fps = rVtPixClkFreq / (usLineLengthPck * usFrameLengthLines ) *2

                if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                    usTimeVts = 0x09d6;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                    usTimeVts = 0x0bce;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                    usTimeVts = 0x0f22;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x13e0;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                    usTimeVts = 0x1ec0;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x2d70;
                }

				/* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
				TRACE( HM8040_INFO,"Resolution 3264x2448 %dfps usTimeHts %d usTimeVts %d\n",ISI_FPS_GET(pConfig->Resolution),usTimeHts,usTimeVts);
						
    			break;
            }
        }        

    }
    
	/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	rVtPixClkFreq = HM8040_get_PCLK(pHM8040Ctx, xclk);		//dafault 140Mhz for HM8040
    
    // store frame timing for later use in AEC module
    pHM8040Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pHM8040Ctx->LineLengthPck    = usLineLengthPck;
    pHM8040Ctx->FrameLengthLines = usFrameLengthLines;

    TRACE( HM8040_DEBUG, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f usLineLengthPck %d usFrameLengthLines %d\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),
                        ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq,usLineLengthPck,usFrameLengthLines);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      HM8040 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT HM8040_SetupImageControl
(
    HM8040_Context_t        *pHM8040Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = HM8040_IsiRegReadIss(  pHM8040Ctx, HM8040_BLC_CTRL00, &RegValue );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = HM8040_IsiRegReadIss(  pHM8040Ctx, HM8040_BLC_CTRL00, &RegValue );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = HM8040_IsiRegReadIss(  pHM8040Ctx, HM8040_AEC_MANUAL, &RegValue );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = HM8040_IsiRegReadIss(  pHM8040Ctx, HM8040_ISP_CTRL01, &RegValue );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = HM8040_IsiRegReadIss(  pHM8040Ctx, HM8040_AEC_MANUAL, &RegValue );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = HM8040_IsiRegReadIss( pHM8040Ctx, HM8040_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = HM8040_IsiRegReadIss( pHM8040Ctx, HM8040_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( HM8040_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT HM8040_SetupOutputWindow
(
    HM8040_Context_t        *pHM8040Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pHM8040Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pHM8040Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return HM8040_SetupOutputWindowInternal(pHM8040Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          HM8040_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in HM8040-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      HM8040 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT HM8040_AecSetModeParameters
(
    HM8040_Context_t       *pHM8040Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"",
        pHM8040Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pHM8040Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( HM8040_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)  change accord datasheet
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pHM8040Ctx->AecMaxIntegrationTime = ( ((float)(pHM8040Ctx->FrameLengthLines - 10)) * ((float)pHM8040Ctx->LineLengthPck) ) / (pHM8040Ctx->VtPixClkFreq*2);
    pHM8040Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( HM8040_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"", pHM8040Ctx->AecMaxIntegrationTime  );

    pHM8040Ctx->AecMaxGain = HM8040_MAX_GAIN_AEC;	//gain  limit
    pHM8040Ctx->AecMinGain = 1.0f; 					//as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pHM8040Ctx->AecIntegrationTimeIncrement = ((float)pHM8040Ctx->LineLengthPck) / (pHM8040Ctx->VtPixClkFreq*2);
    pHM8040Ctx->AecGainIncrement = HM8040_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pHM8040Ctx->AecCurGain               = pHM8040Ctx->AecMinGain;
    pHM8040Ctx->AecCurIntegrationTime    = 0.0f;
    pHM8040Ctx->OldCoarseIntegrationTime = 0;
    pHM8040Ctx->OldFineIntegrationTime   = 0;
    //pHM8040Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( HM8040_INFO, "%s%s (exit)\n", __FUNCTION__, pHM8040Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          HM8040_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      HM8040 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT HM8040_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pHM8040Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pHM8040Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = HM8040_IsiRegWriteIss ( pHM8040Ctx, HM8040_SOFTWARE_RST, HM8040_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( HM8040_DEBUG, "%s: HM8040 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_MODE_SELECT, HM8040_MODE_SELECT_OFF );//HM8040_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: Can't write HM8040 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    if(pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_aRegDescription_fourlane);
    }
	else if(pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE)
        result = IsiRegDefaultsApply( pHM8040Ctx, HM8040_g_aRegDescription_twolane);

    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pHM8040Ctx, HM8040_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = HM8040_IsiRegWriteIss( pHM8040Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = HM8040_SetupOutputFormat( pHM8040Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = HM8040_SetupOutputWindow( pHM8040Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = HM8040_SetupImageControl( pHM8040Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = HM8040_AecSetModeParameters( pHM8040Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pHM8040Ctx->Configured = BOOL_TRUE;
    }

    result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_MODE_SELECT, HM8040_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: Can't write HM8040 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    //set OTP
    //result = apply_otp_rk3288(handle,&g_otp_info);
    
    result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_MODE_SELECT, HM8040_MODE_SELECT_OFF );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: Can't write HM8040 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiChangeSensorResolutionIss
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
static RESULT HM8040_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pHM8040Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (HM8040_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pHM8040Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pHM8040Ctx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pHM8040Ctx->Config.Resolution))) ) {

            if (pHM8040Ctx->Streaming != BOOL_FALSE) {
                TRACE( HM8040_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( HM8040_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pHM8040Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = HM8040_SetupOutputWindowInternal( pHM8040Ctx, &pHM8040Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( HM8040_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pHM8040Ctx->AecCurGain;
        float OldIntegrationTime = pHM8040Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = HM8040_AecSetModeParameters( pHM8040Ctx, &pHM8040Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( HM8040_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = HM8040_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( HM8040_ERROR, "%s: HM8040_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( HM8040_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          HM8040_IsiSensorSetStreamingIss
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
static RESULT HM8040_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pHM8040Ctx->Configured != BOOL_TRUE) || (pHM8040Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = HM8040_IsiRegReadIss ( pHM8040Ctx, HM8040_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = HM8040_IsiRegWriteIss ( pHM8040Ctx, HM8040_MODE_SELECT, (RegValue | HM8040_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = HM8040_IsiRegReadIss ( pHM8040Ctx, HM8040_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = HM8040_IsiRegWriteIss ( pHM8040Ctx, HM8040_MODE_SELECT, (RegValue & ~HM8040_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pHM8040Ctx->Streaming = on;
    }

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      HM8040 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT HM8040_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pHM8040Ctx->Configured = BOOL_FALSE;
    pHM8040Ctx->Streaming  = BOOL_FALSE;

    TRACE( HM8040_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( HM8040_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( HM8040_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( HM8040_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( HM8040_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( HM8040_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pHM8040Ctx->IsiCtx.HalHandle, pHM8040Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT HM8040_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = HM8040_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 16U) | (HM8040_CHIP_ID_MIDDLE_BYTE_DEFAULT<<8U);
    RevId = RevId | HM8040_CHIP_ID_LOW_BYTE_DEFAULT;

    result = HM8040_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( HM8040_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( HM8040_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetSensorRevisionIss
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
static RESULT HM8040_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = HM8040_IsiRegReadIss ( handle, HM8040_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 16U );
    result = HM8040_IsiRegReadIss ( handle, HM8040_CHIP_ID_MIDDLE_BYTE, &data );
    *p_value |= ( (data & 0xFF) << 8U );
    result = HM8040_IsiRegReadIss ( handle, HM8040_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiRegReadIss
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
static RESULT HM8040_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, HM8040_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( HM8040_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiRegWriteIss
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
static RESULT HM8040_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, HM8040_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( HM8040_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          HM8040 instance
 *
 * @param   handle       HM8040 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT HM8040_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( HM8040_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pHM8040Ctx->AecMinGain;
    *pMaxGain = pHM8040Ctx->AecMaxGain;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          HM8040 instance
 *
 * @param   handle       HM8040 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT HM8040_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( HM8040_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pHM8040Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pHM8040Ctx->AecMaxIntegrationTime;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          HM8040_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  HM8040 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT HM8040_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }
	
	result = HM8040_IsiRegReadIss ( pHM8040Ctx, HM8040_AEC_AGC_ADJ, &data);
	result_gain = (data & 0xFF) ;
	result_gain = result_gain + 16;
	*pSetGain = ( (float)result_gain ) / HM8040_MAXN_GAIN;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  HM8040 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT HM8040_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pHM8040Ctx->AecGainIncrement;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  HM8040 sensor instance handle
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
RESULT HM8040_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
	uint32_t data= 0;

    TRACE( HM8040_INFO, "%s: (enter) pHM8040Ctx->AecMaxGain(%f) \n", __FUNCTION__,pHM8040Ctx->AecMaxGain);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( HM8040_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pHM8040Ctx->AecMinGain ) NewGain = pHM8040Ctx->AecMinGain;
    if( NewGain > pHM8040Ctx->AecMaxGain ) NewGain = pHM8040Ctx->AecMaxGain;

    usGain = (uint16_t)(NewGain * HM8040_MAXN_GAIN - 16);	//change to sensor data

    // write new gain into sensor registers, do not write if nothing has changed
    if(usGain != pHM8040Ctx->OldGain)
    {
        result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_AEC_AGC_ADJ, usGain & 0xFF);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

		result = HM8040_IsiRegWriteIss( pHM8040Ctx, 0x0104, 0x00);	//CMU update
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pHM8040Ctx->OldGain = usGain;
	}

    //calculate gain actually set
    pHM8040Ctx->AecCurGain = ( (float)(usGain+16) ) / HM8040_MAXN_GAIN;

    //return current state
    *pSetGain = pHM8040Ctx->AecCurGain;
    //TRACE( HM8040_ERROR, "%s: psetgain=%f, NewGain=%f\n", __FUNCTION__, *pSetGain, NewGain);

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          HM8040_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  HM8040 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT HM8040_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pHM8040Ctx->AecCurIntegrationTime;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  HM8040 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT HM8040_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pHM8040Ctx->AecIntegrationTimeIncrement;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  HM8040 sensor instance handle
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
RESULT HM8040_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( HM8040_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pHM8040Ctx->AecMinIntegrationTime,
        pHM8040Ctx->AecMaxIntegrationTime);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( HM8040_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pHM8040Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pHM8040Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pHM8040Ctx->AecMinIntegrationTime ) NewIntegrationTime = pHM8040Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck ) / (vt_pix_clk_freq(MHz)x 2 x 10^6)
    //=>
    //coarse_integration_time = (int)( integration_time * (vt_pix_clk_freq(MHz) x 2 x 10^6)  / line_length_pck )
    ShutterWidthPck = NewIntegrationTime * ( (float)(pHM8040Ctx->VtPixClkFreq*2) );

    // avoid division by zero
    if ( pHM8040Ctx->LineLengthPck == 0 )
    {
        TRACE( HM8040_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pHM8040Ctx->LineLengthPck) );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pHM8040Ctx->LineLengthPck)+ 0.5f);

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pHM8040Ctx->OldCoarseIntegrationTime )
    {
    	//result = HM8040_IsiRegWriteIss( pHM8040Ctx, 0x0340, pHM8040Ctx->FrameLengthLines >> 8);
        //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        //result = HM8040_IsiRegWriteIss( pHM8040Ctx, 0x0341, pHM8040Ctx->FrameLengthLines & 0xFE);
        //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_AEC_EXPO_H, (CoarseIntegrationTime >> 8) & 0xFF);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_AEC_EXPO_L, CoarseIntegrationTime & 0xFE);	//必须为偶数
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

		result = HM8040_IsiRegWriteIss( pHM8040Ctx, 0x0104, 0x00);	//CMU update
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pHM8040Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //calculate integration time actually set
    pHM8040Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pHM8040Ctx->LineLengthPck) / (pHM8040Ctx->VtPixClkFreq*2);

    //return current state
    *pSetIntegrationTime = pHM8040Ctx->AecCurIntegrationTime;

    TRACE( HM8040_INFO, "%s:\n"
         "VtPixClkFreq:%f LineLengthPck:%x FrameLengthLines %x\n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n", __FUNCTION__, 
         pHM8040Ctx->VtPixClkFreq,pHM8040Ctx->LineLengthPck,pHM8040Ctx->FrameLengthLines,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime);
	
    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          HM8040_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  HM8040 sensor instance handle
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
RESULT HM8040_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( HM8040_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( HM8040_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );

    result = HM8040_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = HM8040_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( HM8040_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT HM8040_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pHM8040Ctx->AecCurGain;
    *pSetIntegrationTime = pHM8040Ctx->AecCurIntegrationTime;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetResolutionIss
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
RESULT HM8040_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pHM8040Ctx->Config.Resolution;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pHM8040Ctx             HM8040 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT HM8040_IsiGetAfpsInfoHelperIss(
    HM8040_Context_t   *pHM8040Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pHM8040Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pHM8040Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = HM8040_SetupOutputWindowInternal( pHM8040Ctx, &pHM8040Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = HM8040_AecSetModeParameters( pHM8040Ctx, &pHM8040Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( HM8040_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pHM8040Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pHM8040Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pHM8040Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pHM8040Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pHM8040Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          HM8040_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  HM8040 sensor instance handle
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
RESULT HM8040_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        TRACE( HM8040_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pHM8040Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pHM8040Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pHM8040Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pHM8040Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    HM8040_Context_t *pDummyCtx = (HM8040_Context_t*) malloc( sizeof(HM8040_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( HM8040_ERROR,  "%s: Can't allocate dummy HM8040 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pHM8040Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = HM8040_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes)
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
                    TRACE( HM8040_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
                    result = RET_NOTSUPP;
                    break;
                   
                case ISI_RES_1632_1224P30:
                case ISI_RES_1632_1224P25:
                case ISI_RES_1632_1224P20:
                case ISI_RES_1632_1224P15:
                case ISI_RES_1632_1224P10:
                    AFPSCHECKANDADD( ISI_RES_1632_1224P30 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P25 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P20 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P15 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P10 );
                    break;

                case ISI_RES_3264_2448P15:
                case ISI_RES_3264_2448P7:
                    AFPSCHECKANDADD( ISI_RES_3264_2448P15 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P7 );
                    break;

                // check next series here...
            }
        

            break;
        }

        case SUPPORT_MIPI_FOUR_LANE:
        {
            switch(Resolution)
            {
                default:
                    TRACE( HM8040_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
                    result = RET_NOTSUPP;
                    break;
                   
                case ISI_RES_1632_1224P30:
                case ISI_RES_1632_1224P25:
                case ISI_RES_1632_1224P20:
                case ISI_RES_1632_1224P15:
                case ISI_RES_1632_1224P10:
                    AFPSCHECKANDADD( ISI_RES_1632_1224P30 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P25 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P20 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P15 );
                    AFPSCHECKANDADD( ISI_RES_1632_1224P10 );
                    break;
                    
                case ISI_RES_3264_2448P30:
                case ISI_RES_3264_2448P25:
                case ISI_RES_3264_2448P20:
                case ISI_RES_3264_2448P15:
                case ISI_RES_3264_2448P10:
                case ISI_RES_3264_2448P7:
                    AFPSCHECKANDADD( ISI_RES_3264_2448P30 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P25 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P20 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P15 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P10 );
                    AFPSCHECKANDADD( ISI_RES_3264_2448P7 );
                    break;

                // check next series here...
            }
        

            break;
        }

        default:
            TRACE( HM8040_ERROR,  "%s: pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibKFactor
 *
 * @brief   Returns the HM8040 specific K-Factor
 *
 * @param   handle       HM8040 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&HM8040_KFactor;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the HM8040 specific PCA-Matrix
 *
 * @param   handle          HM8040 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&HM8040_PCAMatrix;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              HM8040 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&HM8040_SVDMeanValue;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              HM8040 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&HM8040_CenterLine;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              HM8040 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&HM8040_AwbClipParm;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              HM8040 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&HM8040_AwbGlobalFadeParm;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              HM8040 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT HM8040_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&HM8040_AwbFade2Parm;

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          HM8040_IsiGetIlluProfile
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
static RESULT HM8040_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
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
        for ( i=0U; i<HM8040_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( HM8040_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &HM8040_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetLscMatrixTable
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
static RESULT HM8040_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
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
                if ( ( pHM8040Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pHM8040Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( HM8040_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pHM8040Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pHM8040Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( HM8040_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pHM8040Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pHM8040Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( HM8040_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pHM8040Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pHM8040Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( HM8040_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pHM8040Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pHM8040Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &HM8040_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( HM8040_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( HM8040_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          HM8040_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT HM8040_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          HM8040 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT HM8040_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    //TRACE( HM8040_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
	
	if ((pHM8040Ctx->VcmInfo.StepMode & 0x0c) != 0) {
		vcm_movefull_t = 64* (1<<(pHM8040Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pHM8040Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
	}else{
		vcm_movefull_t =64*1023/1000;
	TRACE( HM8040_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
	}
 
	*pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
	// *pMaxStep = MAX_LOG;

	result = HM8040_IsiMdiFocusSet( handle, MAX_LOG );

	//TRACE( HM8040_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          HM8040 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT HM8040_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( HM8040_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( HM8040_ERROR, "%s: pHM8040Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pHM8040Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pHM8040Ctx->VcmInfo.StartCurrent + (pHM8040Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( HM8040_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pHM8040Ctx->VcmInfo.StepMode );
	
    //TRACE( HM8040_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pHM8040Ctx->IsiCtx.HalHandle,
                             pHM8040Ctx->IsiCtx.I2cAfBusNum,
                             pHM8040Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pHM8040Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( HM8040_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          HM8040 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT HM8040_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( HM8040_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pHM8040Ctx->IsiCtx.HalHandle,
                            pHM8040Ctx->IsiCtx.I2cAfBusNum,
                            pHM8040Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pHM8040Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( HM8040_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

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
	if( *pAbsStep <= pHM8040Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pHM8040Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pHM8040Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pHM8040Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pHM8040Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( HM8040_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT HM8040_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          HM8040_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT HM8040_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = HM8040_IsiRegReadIss( pHM8040Ctx, HM8040_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = HM8040_IsiRegReadIss( pHM8040Ctx, HM8040_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = HM8040_IsiRegWriteIss( pHM8040Ctx, HM8040_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pHM8040Ctx->TestPattern = enable;
    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          HM8040_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          HM8040 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT HM8040_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pHM8040Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pHM8040Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pHM8040Ctx->IsiSensorMipiInfo.sensorHalDevID;
	TRACE( HM8040_INFO, "ucMipiLanes %d ulMipiFreq %d sensorHalDevID %0x08\n", ptIsiSensorMipiInfo->ucMipiLanes,
		ptIsiSensorMipiInfo->ulMipiFreq,ptIsiSensorMipiInfo->sensorHalDevID);
    TRACE( HM8040_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT HM8040_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
    	TRACE( HM8040_ERROR, "%s: pHM8040Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( HM8040_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT HM8040_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    HM8040_Context_t *pHM8040Ctx = (HM8040_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( HM8040_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pHM8040Ctx == NULL )
    {
    	TRACE( HM8040_ERROR, "%s: pHM8040Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( HM8040_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = HM8040_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          HM8040_IsiGetSensorIss
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
RESULT HM8040_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( HM8040_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = HM8040_g_acName;
        pIsiSensor->pRegisterTable                      = HM8040_g_aRegDescription_fourlane;
        pIsiSensor->pIsiSensorCaps                      = &HM8040_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= HM8040_IsiGetSensorIsiVersion;
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= HM8040_IsiGetSensorTuningXmlVersion;
		pIsiSensor->pIsiCheckOTPInfo                    = check_read_otp;
        pIsiSensor->pIsiCreateSensorIss                 = HM8040_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = HM8040_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = HM8040_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = HM8040_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = HM8040_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = HM8040_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = HM8040_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = HM8040_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = HM8040_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = HM8040_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = HM8040_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = HM8040_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = HM8040_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = HM8040_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = HM8040_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = HM8040_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = HM8040_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = HM8040_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = HM8040_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = HM8040_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = HM8040_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = HM8040_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = HM8040_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = HM8040_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = HM8040_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = HM8040_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = HM8040_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = HM8040_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = HM8040_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = HM8040_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = HM8040_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = HM8040_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = HM8040_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = HM8040_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = HM8040_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = HM8040_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = HM8040_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = HM8040_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = HM8040_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( HM8040_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT HM8040_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( HM8040_ERROR,  "%s: Can't allocate HM8040 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = HM8040_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = HM8040_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = HM8040_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = HM8040_SOFTWARE_RST_VALUE;
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
                while(HM8040_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = HM8040_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = HM8040_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = HM8040_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = HM8040_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = HM8040_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = HM8040_CHIP_ID_LOW_BYTE_DEFAULT;
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
    HM8040_IsiGetSensorIss,
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
    HM8040_IsiGetSensorI2cInfo,
};


