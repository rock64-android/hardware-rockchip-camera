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
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>
#include <common/misc.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"

#include "OV8865_MIPI_priv.h"

#define  OV8865_NEWEST_TUNING_XML "09-10-2015_OV8865_CMK-CB0407-FV1_v0.1.0"

#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( SENSOR_INFO , "OV8865: ", INFO,    0U );
CREATE_TRACER( SENSOR_WARN , "OV8865: ", WARNING, 1U );
CREATE_TRACER( SENSOR_ERROR, "OV8865: ", ERROR,   1U );

CREATE_TRACER( SENSOR_DEBUG, "OV8865: ", INFO,     0U );

CREATE_TRACER( SENSOR_NOTICE0 , "OV8865: ", TRACE_NOTICE0, 1);
CREATE_TRACER( SENSOR_NOTICE1, "OV8865: ", TRACE_NOTICE1, 1U );


#define SENSOR_SLAVE_ADDR       0x6cU            /**< i2c slave address camera sensor */
#define SENSOR_SLAVE_ADDR2      0x20U
#define SENSOR_SLAVE_AF_ADDR    0x18U           /**< i2c slave address of the vcm*/
#define Sensor_OTP_SLAVE_ADDR   0x6cU
#define Sensor_OTP_SLAVE_ADDR2   0x20U

#define SENSOR_MAXN_GAIN 		(128.0f)
#define SENSOR_MIN_GAIN_STEP   ( 1.0f / SENSOR_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define SENSOR_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
#define MDI_SLEW_RATE_CTRL 5U /* S3..0 for MOTOR*/



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char Sensor_g_acName[] = "OV8865_MIPI";

extern const IsiRegDescription_t Sensor_g_aRegDescription_onelane[];
extern const IsiRegDescription_t Sensor_g_aRegDescription_twolane[];
extern const IsiRegDescription_t Sensor_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t Sensor_g_1632x1224_twolane[];
extern const IsiRegDescription_t Sensor_g_1632x1224P30_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P25_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P20_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P15_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P10_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224_fourlane[];
extern const IsiRegDescription_t Sensor_g_1632x1224P30_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P25_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P20_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P15_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_1632x1224P10_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448_twolane[];
extern const IsiRegDescription_t Sensor_g_3264x2448P15_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P7_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448_fourlane[];
extern const IsiRegDescription_t Sensor_g_3264x2448P30_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P25_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P20_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P15_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P10_fourlane_fpschg[];
extern const IsiRegDescription_t Sensor_g_3264x2448P7_fourlane_fpschg[];

const IsiSensorCaps_t Sensor_g_IsiSensorDefaultConfig;

#define SENSOR_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define SENSOR_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define SENSOR_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT Sensor_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT Sensor_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT Sensor_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT Sensor_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT Sensor_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT Sensor_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT Sensor_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT Sensor_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT Sensor_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT Sensor_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT Sensor_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT Sensor_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT Sensor_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT Sensor_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT Sensor_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT Sensor_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT Sensor_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT Sensor_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT Sensor_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT Sensor_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT Sensor_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT Sensor_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT Sensor_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT Sensor_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT Sensor_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT Sensor_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT Sensor_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT Sensor_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT Sensor_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT Sensor_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT Sensor_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT Sensor_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT Sensor_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT Sensor_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT Sensor_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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
static int Sensor_read_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address
){
    uint32_t temp = 0;
    if(Sensor_IsiRegReadIss(handle,address,&temp) != RET_SUCCESS){
        TRACE( SENSOR_ERROR, "%s read OTP register 0x%x erro!\n", __FUNCTION__,address);
    }
    return temp;
}

static RESULT Sensor_write_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
){
    RESULT result = RET_SUCCESS;
    if((result = Sensor_IsiRegWriteIss(handle,address,value)) != RET_SUCCESS){
        TRACE( SENSOR_ERROR, "%s write OTP register (0x%x,0x%x) erro!\n", __FUNCTION__,address,value);
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
    int light_rg;
    int light_bg;
    int lenc[62];
    int VCM_start;
    int VCM_end;
    int VCM_dir;
};

static struct otp_struct g_otp_info ={0};

//for test,just for compile
int  RG_Ratio_Typical=0x10F;
int  BG_Ratio_Typical=0x161;

#if 1
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
static int apply_otp(IsiSensorHandle_t   handle,struct otp_struct *otp_ptr)
{
    int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
	
    // apply OTP WB Calibration
    if ((*otp_ptr).flag & 0x40) {
        if((*otp_ptr).light_rg == 0){
            // no light source infomation in OTP ,light factor = 1
            rg = (*otp_ptr).rg_ratio;
        }else{
            rg = (*otp_ptr).rg_ratio * ((*otp_ptr).light_rg+512)/1024;
        }
        
        if((*otp_ptr).light_bg == 0){
            // no light source infomation in OTP ,light factor = 1
            bg = (*otp_ptr).bg_ratio;
        }else{
            bg = (*otp_ptr).bg_ratio * ((*otp_ptr).light_bg+512)/1024;
        }
        //calculate G gain
        R_gain = (RG_Ratio_Typical*0x400) / rg;
        B_gain = (BG_Ratio_Typical*0x400) / bg;
        G_gain = 0x400;

		Base_gain = G_gain;

        if (R_gain < Base_gain)
             Base_gain = R_gain;
        if (B_gain < Base_gain)
            Base_gain = B_gain;
  
        R_gain = 0x400 * R_gain / (Base_gain);
        B_gain = 0x400 * B_gain / (Base_gain);
        G_gain = 0x400 * G_gain / (Base_gain);
    // update sensor WB gain
        if (R_gain>0x400) {
            Sensor_write_i2c( handle,0x5018, R_gain>>6);
            Sensor_write_i2c( handle,0x5019, R_gain & 0x003f);
        }
        if (G_gain>0x400) {
            Sensor_write_i2c( handle,0x501a, G_gain>>6);
            Sensor_write_i2c( handle,0x501b, G_gain & 0x003f);
        }
        if (B_gain>0x400) {
            Sensor_write_i2c( handle,0x501c, B_gain>>6);
            Sensor_write_i2c( handle,0x501d, B_gain & 0x003f);
        }
    }
    // apply OTP Lenc Calibration
    if ((*otp_ptr).flag & 0x10) {
        temp = Sensor_read_i2c( handle,0x5000);
        temp = 0x80 | temp;
        Sensor_write_i2c( handle,0x5000, temp);
        for(i=0;i<62;i++) {
            Sensor_write_i2c( handle,0x5800 + i, (*otp_ptr).lenc[i]);
        }
    }
    TRACE( SENSOR_NOTICE0,  "%s: success!!!\n",  __FUNCTION__ );
	
    return (*otp_ptr).flag;
}

static int check_read_otp(
    sensor_i2c_write_t*  sensor_i2c_write_p,
    sensor_i2c_read_t*  sensor_i2c_read_p,
    void* context,
    int camsys_fd
)
{
    struct otp_struct *otp_ptr = &g_otp_info;

    int otp_flag=0x0, addr, temp, i;
    //set 0x5002[3] to “0”
    int temp1;
    int i2c_base_info[3];

    i2c_base_info[0] = Sensor_OTP_SLAVE_ADDR; //otp i2c addr
    i2c_base_info[1] = 2; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size
    //stream on 
    sensor_i2c_write_p(context,camsys_fd, SENSOR_MODE_SELECT, SENSOR_MODE_SELECT_ON, i2c_base_info );
    
    temp1 = sensor_i2c_read_p(context,camsys_fd,0x5002,i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x5002, (0x00 & 0x08) | (temp1 & (~0x08)), i2c_base_info);
    // read OTP into buffer
    sensor_i2c_write_p(context,camsys_fd,0x3d84, 0xC0, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d88, 0x70, i2c_base_info); // OTP start address
    sensor_i2c_write_p(context,camsys_fd,0x3d89, 0x10, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d8A, 0x71, i2c_base_info); // OTP end address
    sensor_i2c_write_p(context,camsys_fd,0x3d8B, 0x84, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d81, 0x01, i2c_base_info); // load otp into buffer
    osSleep(10);
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7010, i2c_base_info);

    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7011; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7016; // base address of info group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
        addr = 0x701b; // base address of info group 3
    }
    if(addr != 0) {
        (*otp_ptr).flag = 0x80; // valid info and AWB in OTP
        (*otp_ptr).module_integrator_id = sensor_i2c_read_p(context,camsys_fd,addr, i2c_base_info);
        (*otp_ptr).lens_id = sensor_i2c_read_p(context,camsys_fd, addr + 1, i2c_base_info);
        (*otp_ptr).production_year = sensor_i2c_read_p(context,camsys_fd, addr + 2, i2c_base_info);
        (*otp_ptr).production_month = sensor_i2c_read_p(context,camsys_fd, addr + 3, i2c_base_info);
        (*otp_ptr).production_day = sensor_i2c_read_p(context,camsys_fd,addr + 4, i2c_base_info);
    }
    else {
        (*otp_ptr).flag = 0x00; // not info and AWB in OTP
        (*otp_ptr).module_integrator_id = 0;
        (*otp_ptr).lens_id = 0;
        (*otp_ptr).production_year = 0;
        (*otp_ptr).production_month = 0;
        (*otp_ptr).production_day = 0;
    }

    // OTP base information and WB calibration data
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7020, i2c_base_info);

    addr = 0;
    // OTP AWB Calibration
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7021; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7026; // base address of info group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
        addr = 0x702b; // base address of info group 3
    }

    if(addr != 0) {
        (*otp_ptr).flag |= 0x40; // valid info and AWB in OTP
        temp = sensor_i2c_read_p(context,camsys_fd,addr + 4, i2c_base_info);
        (*otp_ptr).rg_ratio = (sensor_i2c_read_p(context,camsys_fd,addr, i2c_base_info)<<2) + ((temp>>6) & 0x03);
        (*otp_ptr).bg_ratio = (sensor_i2c_read_p(context,camsys_fd,addr + 1, i2c_base_info)<<2) + ((temp>>4) & 0x03);
        (*otp_ptr).light_rg = (sensor_i2c_read_p(context,camsys_fd,addr + 2, i2c_base_info)<<2) + ((temp>>2) & 0x03);
        (*otp_ptr).light_bg = (sensor_i2c_read_p(context,camsys_fd,addr + 3, i2c_base_info)<<2) + ((temp) & 0x03);
        TRACE( SENSOR_ERROR, "%s awb info in OTP(rg,bg,light_rg,light_bg)=(0x%x,0x%x,0x%x,0x%x)!\n", __FUNCTION__,(*otp_ptr).rg_ratio,(*otp_ptr).bg_ratio,
                                (*otp_ptr).light_rg,(*otp_ptr).light_bg);

    }else {
        (*otp_ptr).rg_ratio = 0;
        (*otp_ptr).bg_ratio = 0;
        (*otp_ptr).light_rg = 0;
        (*otp_ptr).light_bg = 0;
        TRACE( SENSOR_ERROR, "%s no awb info in OTP!\n", __FUNCTION__);
    }
    
    // OTP VCM Calibration
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7030, i2c_base_info);
    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7031; // base address of VCM Calibration group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7034; // base address of VCM Calibration group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
        addr = 0x7037; // base address of VCM Calibration group 3
    }
    if(addr != 0) {
        (*otp_ptr).flag |= 0x20;
        temp = sensor_i2c_read_p(context,camsys_fd,addr + 2, i2c_base_info);
        (* otp_ptr).VCM_start = (sensor_i2c_read_p(context,camsys_fd,addr, i2c_base_info)<<2) | ((temp>>6) & 0x03);
        (* otp_ptr).VCM_end = (sensor_i2c_read_p(context,camsys_fd,addr + 1, i2c_base_info) << 2) | ((temp>>4) & 0x03);
        (* otp_ptr).VCM_dir = (temp>>2) & 0x03;
    }
    else {
        (* otp_ptr).VCM_start = 0;
        (* otp_ptr).VCM_end = 0;
        (* otp_ptr).VCM_dir = 0;
        TRACE( SENSOR_INFO, "%s no VCM info in OTP!\n", __FUNCTION__);
    }
    // OTP Lenc Calibration
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x703a, i2c_base_info);
    addr = 0;
    int  checksum2=0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x703b; // base address of Lenc Calibration group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7079; // base address of Lenc Calibration group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
        addr = 0x70b7; // base address of Lenc Calibration group 3
    }
    if(addr != 0) {
        (*otp_ptr).flag |= 0x10;
        for(i=0;i<62;i++) {
            (* otp_ptr).lenc[i]=sensor_i2c_read_p(context,camsys_fd,addr + i, i2c_base_info);
            TRACE( SENSOR_INFO, "%s lsc 0x%x!\n", __FUNCTION__,(*otp_ptr).lenc[i]);
        }
    }
    else {
        for(i=0;i<62;i++) {
        (* otp_ptr).lenc[i]=0;
        }
    }
    for(i=0x7010;i<=0x70f4;i++) {
        sensor_i2c_write_p(context,camsys_fd,i,0, i2c_base_info); // clear OTP buffer, recommended use continuous write to accelarate
    }
    //set 0x5002[3] to “1”
    temp1 = sensor_i2c_read_p(context,camsys_fd,0x5002, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x5002, (0x08 & 0x08) | (temp1 & (~0x08)), i2c_base_info);

    //stream off 
    sensor_i2c_write_p(context,camsys_fd, SENSOR_MODE_SELECT, SENSOR_MODE_SELECT_OFF, i2c_base_info );
    if((*otp_ptr).flag != 0)
        return RET_SUCCESS;
    else
        return RET_NOTSUPP;
}
#endif

/* OTP END*/


/*****************************************************************************/
/**
 *          Sensor_IsiCreateSensorIss
 *
 * @brief   This function creates a new Sensor sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT Sensor_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    Sensor_Context_t *pSensorCtx;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pSensorCtx = ( Sensor_Context_t * )malloc ( sizeof (Sensor_Context_t) );
    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR,  "%s: Can't allocate Sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorCtx, 0, sizeof( Sensor_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pSensorCtx );
        return ( result );
    }
    
    pSensorCtx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pSensorCtx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pSensorCtx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pSensorCtx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? SENSOR_SLAVE_ADDR : pConfig->SlaveAddr;
    pSensorCtx->IsiCtx.NrOfAddressBytes       = 2U;

    pSensorCtx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pSensorCtx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? SENSOR_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pSensorCtx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pSensorCtx->IsiCtx.pSensor                = pConfig->pSensor;

    pSensorCtx->Configured             = BOOL_FALSE;
    pSensorCtx->Streaming              = BOOL_FALSE;
    pSensorCtx->TestPattern            = BOOL_FALSE;
    pSensorCtx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pSensorCtx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.RatedCurrent   = pSensorCtx->VcmInfo.StartCurrent + MAX_LOG*pSensorCtx->VcmInfo.Step;
    pSensorCtx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pSensorCtx->IsiSensorMipiInfo.sensorHalDevID = pSensorCtx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( SENSOR_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pSensorCtx;

    result = HalSetCamConfig( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an Sensor sensor instance.
 *
 * @param   handle      Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT Sensor_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)Sensor_IsiSensorSetStreamingIss( pSensorCtx, BOOL_FALSE );
    (void)Sensor_IsiSensorSetPowerIss( pSensorCtx, BOOL_FALSE );

    (void)HalDelRef( pSensorCtx->IsiCtx.HalHandle );

    MEMSET( pSensorCtx, 0, sizeof( Sensor_Context_t ) );
    free ( pSensorCtx );

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCapsIss
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
static RESULT Sensor_IsiGetCapsIssInternal
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
        pIsiSensorCaps->BPat            = ISI_BPAT_BGBGGRGR;
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
 
static RESULT Sensor_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = Sensor_IsiGetCapsIssInternal(pIsiSensorCaps,pSensorCtx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t Sensor_g_IsiSensorDefaultConfig =
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
 *          Sensor_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT Sensor_SetupOutputFormat
(
    Sensor_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by Sensor sensor, so the YCSequence parameter is not checked */
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
            TRACE( SENSOR_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( SENSOR_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( SENSOR_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( SENSOR_INFO, "%s%s (exit)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int Sensor_get_PCLK( Sensor_Context_t *pSensorCtx, int XVCLK)
{
    // calculate PCLK
    uint32_t SCLK, temp1, temp2, temp3;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_sys_pre_div, Pll2_sys_divider2x, Sys_pre_div, sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_pre_div_map[] = {1, 2, 4, 1};

    
    //temp1 = ReadSCCB(0x6c, 0x3007);
    Sensor_IsiRegReadIss(  pSensorCtx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	Sensor_IsiRegReadIss(  pSensorCtx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	Sensor_IsiRegReadIss(  pSensorCtx, 0x030c, &temp1 );
	Sensor_IsiRegReadIss(  pSensorCtx, 0x030d, &temp3 );
	temp1 = temp1 & 0x03;
	temp2 = (temp1<<8) + temp3;
	if(!temp2) {
 		Pll2_multiplier = 1;
 	}
	else {
 	Pll2_multiplier = temp2;
	}
	
    Sensor_IsiRegReadIss(  pSensorCtx, 0x030f, &temp1 );
	temp1 = temp1 & 0x0f;
	Pll2_sys_pre_div = temp1 + 1;
	Sensor_IsiRegReadIss(  pSensorCtx, 0x030e, &temp1 );
	temp1 = temp1 & 0x07;
	Pll2_sys_divider2x = Pll2_sys_divider2x_map[temp1];

	Sensor_IsiRegReadIss(  pSensorCtx, 0x3106, &temp1 );
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
	Sensor_IsiRegWriteIss(pSensorCtx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	Sensor_IsiRegWriteIss(pSensorCtx, 0x350a, temp1);
	return SCLK*10000;
}

/*****************************************************************************/
/**
 *          Sensor_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT Sensor_SetupOutputWindowInternal
(
    Sensor_Context_t        *pSensorCtx,
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
    int xclk = 2400;
    
	TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	
		pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 720;
		switch ( pConfig->Resolution )
		{
			case ISI_RES_1632_1224P15:
			{				

				break;
				
			}
			
			case ISI_RES_3264_2448P7:
			{				

				break;
				
			}
	
			default:
			{
				TRACE( SENSOR_ERROR, "%s: Resolution not supported\n", __FUNCTION__ );
				return ( RET_NOTSUPP );
			}
		}
	} else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

        pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 720;
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
						result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224_twolane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P30_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P25_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P20_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P10_twolane_fpschg);
                    }
        		}

                usTimeHts = 0x0788; 
                    
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x04dc;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0x5d4;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0x074a;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x9b8;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x0e94;
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
						result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448_twolane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P7_twolane_fpschg);
                    }
        		    
        		}
        		
    			usTimeHts = 0x0794;                
                if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x09aa;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x1354;
                }
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }

            default:
            {
                TRACE( SENSOR_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
    } else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE) {
    	pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 720;

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
						result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224_fourlane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_1632x1224P10_fourlane_fpschg);
                    }
        		}
    			usTimeHts = 0x0788; 
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x04dc;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0x5d4;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0x074a;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x9b8;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x0e94;
                }
                
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
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
						result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448_fourlane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P10_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_3264x2448P7_fourlane_fpschg);
                    }
        		    
        		}
        		
    			usTimeHts = 0x0794;                
                if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                    usTimeVts = 0x09aa;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                    usTimeVts = 0x0b98;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                    usTimeVts = 0x0e7f;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x1354;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                    usTimeVts = 0x1cfe;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x26a8;
                }
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }
        }        

    }
    
	
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	rVtPixClkFreq = Sensor_get_PCLK(pSensorCtx, xclk);
    
    // store frame timing for later use in AEC module
    pSensorCtx->VtPixClkFreq     = rVtPixClkFreq;
    pSensorCtx->LineLengthPck    = usLineLengthPck;
    pSensorCtx->FrameLengthLines = usFrameLengthLines;

    TRACE( SENSOR_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      Sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT Sensor_SetupImageControl
(
    Sensor_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = Sensor_IsiRegReadIss(  pSensorCtx, OV8858_BLC_CTRL00, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = Sensor_IsiRegReadIss(  pSensorCtx, OV8858_BLC_CTRL00, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = Sensor_IsiRegReadIss(  pSensorCtx, OV8858_AEC_MANUAL, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = Sensor_IsiRegReadIss(  pSensorCtx, OV8858_ISP_CTRL01, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = Sensor_IsiRegReadIss(  pSensorCtx, OV8858_AEC_MANUAL, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = Sensor_IsiRegReadIss( pSensorCtx, OV8858_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = Sensor_IsiRegReadIss( pSensorCtx, OV8858_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( SENSOR_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT Sensor_SetupOutputWindow
(
    Sensor_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return Sensor_SetupOutputWindowInternal(pSensorCtx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          Sensor_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in Sensor-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      Sensor context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT Sensor_AecSetModeParameters
(
    Sensor_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
        pSensorCtx->Config.Resolution, pConfig->Resolution);

    if ( (pSensorCtx->VtPixClkFreq == 0.0f) )
    {
        TRACE( SENSOR_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pSensorCtx->AecMaxIntegrationTime = ( ((float)(pSensorCtx->FrameLengthLines - 4)) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecMinIntegrationTime = 0.0001f;

    TRACE( SENSOR_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"", pSensorCtx->AecMaxIntegrationTime  );

    pSensorCtx->AecMaxGain = SENSOR_MAX_GAIN_AEC;
    pSensorCtx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pSensorCtx->AecIntegrationTimeIncrement = ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecGainIncrement = SENSOR_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pSensorCtx->AecCurGain               = pSensorCtx->AecMinGain;
    pSensorCtx->AecCurIntegrationTime    = 0.0f;
    pSensorCtx->OldCoarseIntegrationTime = 0;
    pSensorCtx->OldFineIntegrationTime   = 0;
    //pSensorCtx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( SENSOR_INFO, "%s%s (exit)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Sensor_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pSensorCtx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = Sensor_IsiRegWriteIss ( pSensorCtx, SENSOR_SOFTWARE_RST, SENSOR_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( SENSOR_DEBUG, "%s: Sensor System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_MODE_SELECT, SENSOR_MODE_SELECT_OFF );//OV8858_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: Can't write Sensor Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription );

    if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_fourlane);
    }
	else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE)
        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_twolane);
	else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE)
        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_onelane);
	
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pSensorCtx, Sensor_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = Sensor_IsiRegWriteIss( pSensorCtx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = Sensor_SetupOutputFormat( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = Sensor_SetupOutputWindow( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = Sensor_SetupImageControl( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = Sensor_AecSetModeParameters( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pSensorCtx->Configured = BOOL_TRUE;
    }

    //set OTP info

    result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_MODE_SELECT, SENSOR_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: Can't write Sensor Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    //set OTP
    {
        if(g_otp_info.flag != 0){
            TRACE( SENSOR_NOTICE0, "%s: apply OTP info !!\n", __FUNCTION__);
			apply_otp(handle,&g_otp_info);
        }
    }
    
    result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_MODE_SELECT, SENSOR_MODE_SELECT_OFF );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: Can't write Sensor Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
   

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiChangeSensorResolutionIss
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
static RESULT Sensor_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (Sensor_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pSensorCtx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution))) ) {

            if (pSensorCtx->Streaming != BOOL_FALSE) {
                TRACE( SENSOR_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( SENSOR_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pSensorCtx->Config.Resolution = Resolution;

        // tell sensor about that
        result = Sensor_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( SENSOR_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pSensorCtx->AecCurGain;
        float OldIntegrationTime = pSensorCtx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = Sensor_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( SENSOR_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = Sensor_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( SENSOR_ERROR, "%s: Sensor_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( SENSOR_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiSensorSetStreamingIss
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
static RESULT Sensor_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) || (pSensorCtx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = Sensor_IsiRegReadIss ( pSensorCtx, SENSOR_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = Sensor_IsiRegWriteIss ( pSensorCtx, SENSOR_MODE_SELECT, (RegValue | SENSOR_MODE_SELECT_ON) );//OV8858_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = Sensor_IsiRegReadIss ( pSensorCtx, SENSOR_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = Sensor_IsiRegWriteIss ( pSensorCtx, SENSOR_MODE_SELECT, (RegValue & ~SENSOR_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pSensorCtx->Streaming = on;
    }

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      Sensor sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT Sensor_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pSensorCtx->Configured = BOOL_FALSE;
    pSensorCtx->Streaming  = BOOL_FALSE;

    TRACE( SENSOR_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( SENSOR_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( SENSOR_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( SENSOR_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( SENSOR_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( SENSOR_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT Sensor_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = SENSOR_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 16U) | (SENSOR_CHIP_ID_MIDDLE_BYTE_DEFAULT<<8U);
    RevId = RevId | SENSOR_CHIP_ID_LOW_BYTE_DEFAULT;

    result = Sensor_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( SENSOR_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( SENSOR_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorRevisionIss
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
static RESULT Sensor_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = Sensor_IsiRegReadIss ( handle, SENSOR_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 16U );
    result = Sensor_IsiRegReadIss ( handle, SENSOR_CHIP_ID_MIDDLE_BYTE, &data );
    *p_value |= ( (data & 0xFF) << 8U );
    result = Sensor_IsiRegReadIss ( handle, SENSOR_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiRegReadIss
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
static RESULT Sensor_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, Sensor_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( SENSOR_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiRegWriteIss
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
static RESULT Sensor_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, Sensor_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( SENSOR_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          Sensor instance
 *
 * @param   handle       Sensor sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT Sensor_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( SENSOR_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pSensorCtx->AecMinGain;
    *pMaxGain = pSensorCtx->AecMaxGain;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          Sensor instance
 *
 * @param   handle       Sensor sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT Sensor_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( SENSOR_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pSensorCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pSensorCtx->AecMaxIntegrationTime;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  Sensor sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT Sensor_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

	
	result = Sensor_IsiRegReadIss ( pSensorCtx, SENSOR_AEC_AGC_ADJ_H, &data);
	TRACE( SENSOR_INFO, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = Sensor_IsiRegReadIss ( pSensorCtx, SENSOR_AEC_AGC_ADJ_L, &data);
	TRACE( SENSOR_INFO, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / SENSOR_MAXN_GAIN;
	
    //*pSetGain = pSensorCtx->AecCurGain;
    

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  Sensor sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT Sensor_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecGainIncrement;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  Sensor sensor instance handle
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
RESULT Sensor_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
	uint32_t data= 0;
	uint32_t result_gain= 0;

    TRACE( SENSOR_INFO, "%s: (enter) pSensorCtx->AecMaxGain(%f) \n", __FUNCTION__,pSensorCtx->AecMaxGain);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( SENSOR_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pSensorCtx->AecMinGain ) NewGain = pSensorCtx->AecMinGain;
    if( NewGain > pSensorCtx->AecMaxGain ) NewGain = pSensorCtx->AecMaxGain;

    usGain = (uint16_t)(NewGain * SENSOR_MAXN_GAIN+0.5); //大概加0.5 hkw

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pSensorCtx->OldGain) )
    {
        result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_AEC_AGC_ADJ_H, (usGain>>8)&0x07); //fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_AEC_AGC_ADJ_L, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pSensorCtx->OldGain = usGain;

		/*osSleep(30);
		result = Sensor_IsiRegReadIss ( pSensorCtx, OV8858_AEC_AGC_ADJ_H, &data);
		TRACE( SENSOR_ERROR, " -------reg35088888888:%x-------\n",data );
		result_gain = (data & 0x07) ;
		result = Sensor_IsiRegReadIss ( pSensorCtx, OV8858_AEC_AGC_ADJ_L, &data);
		TRACE( SENSOR_ERROR, " -------reg35099999999:%x-------\n",data );
		result_gain = (result_gain<<8) + data;*/
		
    }

    //calculate gain actually set
    pSensorCtx->AecCurGain = ( (float)usGain ) / SENSOR_MAXN_GAIN;

    //return current state
    *pSetGain = pSensorCtx->AecCurGain;
    TRACE( SENSOR_INFO, "-----------%s: psetgain=%f, NewGain=%f,result_gain=%x\n", __FUNCTION__, *pSetGain, NewGain,result_gain);

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  Sensor sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT Sensor_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  Sensor sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT Sensor_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecIntegrationTimeIncrement;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  Sensor sensor instance handle
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
RESULT Sensor_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;
	uint32_t result_intertime= 0;
	
    //uint32_t FineIntegrationTime   = 0; //not supported by OV8858

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( SENSOR_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pSensorCtx->AecMinIntegrationTime,
        pSensorCtx->AecMaxIntegrationTime);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pSensorCtx->AecMaxIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pSensorCtx->AecMinIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by OV8858
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pSensorCtx->VtPixClkFreq );

    // avoid division by zero
    if ( pSensorCtx->LineLengthPck == 0 )
    {
        TRACE( SENSOR_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pSensorCtx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pSensorCtx->OldCoarseIntegrationTime )
    {//
        result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_AEC_EXPO_H, (CoarseIntegrationTime & 0x0000F000U) >> 12U );//fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_AEC_EXPO_M, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = Sensor_IsiRegWriteIss( pSensorCtx, SENSOR_AEC_EXPO_L, (CoarseIntegrationTime & 0x0000000FU) << 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pSensorCtx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
        
		/*osSleep(30);
		result = Sensor_IsiRegReadIss ( pSensorCtx, OV8858_AEC_EXPO_H, &data);
		TRACE( Sensor_ERROR, " -------reg3500:%x-------\n",data );
		result_intertime = (data & 0x0f) << 8;
		result = Sensor_IsiRegReadIss ( pSensorCtx, OV8858_AEC_EXPO_M, &data);
		TRACE( Sensor_ERROR, " -------reg3501:%x-------\n",data );
		result_intertime = result_intertime + data;
		result = Sensor_IsiRegReadIss ( pSensorCtx, OV8858_AEC_EXPO_L, &data);
		TRACE( SENSOR_ERROR, " -------reg3502:%x-------\n",data );
		result_intertime = (result_intertime << 4) + (data >> 4);*/
		
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //if( FineIntegrationTime != pSensorCtx->OldFineIntegrationTime )
    //{
    //    result = Sensor_IsiRegWriteIss( pSensorCtx, ... , FineIntegrationTime );
    //    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    //    pSensorCtx->OldFineIntegrationTime = FineIntegrationTime; //remember current integration time
    //    *pNumberOfFramesToSkip = 1U; //skip 1 frame
    //}

    //calculate integration time actually set
    //pSensorCtx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pSensorCtx->LineLengthPck) + ((float)FineIntegrationTime) ) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( SENSOR_DEBUG, "%s:\n"
         "pSensorCtx->VtPixClkFreq:%f pSensorCtx->LineLengthPck:%x \n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, 
         pSensorCtx->VtPixClkFreq,pSensorCtx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x0000F000U) >> 12U ,
         (CoarseIntegrationTime & 0x00000FF0U) >> 4U,
         (CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          Sensor_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  Sensor sensor instance handle
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
RESULT Sensor_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( SENSOR_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = Sensor_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = Sensor_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( SENSOR_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT Sensor_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pSensorCtx->AecCurGain;
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetResolutionIss
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
RESULT Sensor_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pSensorCtx->Config.Resolution;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pSensorCtx             Sensor sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT Sensor_IsiGetAfpsInfoHelperIss(
    Sensor_Context_t   *pSensorCtx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pSensorCtx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pSensorCtx->Config.Resolution = Resolution;

    // tell sensor about that
    result = Sensor_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = Sensor_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( SENSOR_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pSensorCtx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pSensorCtx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  Sensor sensor instance handle
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
RESULT Sensor_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( SENSOR_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pSensorCtx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pSensorCtx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pSensorCtx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    Sensor_Context_t *pDummyCtx = (Sensor_Context_t*) malloc( sizeof(Sensor_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( SENSOR_ERROR,  "%s: Can't allocate dummy Sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pSensorCtx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = Sensor_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pSensorCtx->IsiSensorMipiInfo.ucMipiLanes)
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
                    TRACE( SENSOR_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
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
                    TRACE( SENSOR_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
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
            TRACE( SENSOR_ERROR,  "%s: pSensorCtx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibKFactor
 *
 * @brief   Returns the Sensor specific K-Factor
 *
 * @param   handle       Sensor sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&Sensor_KFactor;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the Sensor specific PCA-Matrix
 *
 * @param   handle          Sensor sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&Sensor_PCAMatrix;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              Sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV8858_SVDMeanValue;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              Sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&OV8858_CenterLine;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              Sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&OV8858_AwbClipParm;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              Sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&Sensor_AwbGlobalFadeParm;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              Sensor sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT Sensor_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&Sensor_AwbFade2Parm;

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiGetIlluProfile
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
static RESULT Sensor_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
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
        for ( i=0U; i<Sensor_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( Sensor_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &Sensor_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetLscMatrixTable
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
static RESULT Sensor_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
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
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( SENSOR_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( SENSOR_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( SENSOR_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( SENSOR_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( SENSOR_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( SENSOR_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          Sensor_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT Sensor_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          Sensor sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT Sensor_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    //TRACE( SENSOR_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
 if ((pSensorCtx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pSensorCtx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pSensorCtx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( SENSOR_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = Sensor_IsiMdiFocusSet( handle, MAX_LOG );

    //TRACE( SENSOR_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          Sensor sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT Sensor_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( SENSOR_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( SENSOR_ERROR, "%s: pSensorCtx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pSensorCtx->VcmInfo.StartCurrent;
    else 
        nPosition = pSensorCtx->VcmInfo.StartCurrent + (pSensorCtx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( SENSOR_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pSensorCtx->VcmInfo.StepMode );
	
    //TRACE( SENSOR_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( SENSOR_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          Sensor sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT Sensor_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( SENSOR_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pSensorCtx->IsiCtx.HalHandle,
                            pSensorCtx->IsiCtx.I2cAfBusNum,
                            pSensorCtx->IsiCtx.SlaveAfAddress,
                            0,
                            pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( SENSOR_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

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
	if( *pAbsStep <= pSensorCtx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pSensorCtx->VcmInfo.StartCurrent) && (*pAbsStep<=pSensorCtx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pSensorCtx->VcmInfo.RatedCurrent - *pAbsStep ) / pSensorCtx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( SENSOR_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT Sensor_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT Sensor_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8858Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = Sensor_IsiRegReadIss( pSensorCtx, OV8858_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = Sensor_IsiRegReadIss( pSensorCtx, OV8858_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = Sensor_IsiRegWriteIss( pSensorCtx, OV8858_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pSensorCtx->TestPattern = enable;
    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT Sensor_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pSensorCtx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pSensorCtx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pSensorCtx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( SENSOR_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT Sensor_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( SENSOR_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( SENSOR_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT Sensor_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( SENSOR_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( SENSOR_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( SENSOR_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV8865_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorIss
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
RESULT Sensor_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( SENSOR_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = Sensor_g_acName;
        pIsiSensor->pRegisterTable                      = Sensor_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &Sensor_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= Sensor_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= Sensor_IsiGetSensorTuningXmlVersion;//oyyf
		pIsiSensor->pIsiCheckOTPInfo                    = check_read_otp;//zyc
        pIsiSensor->pIsiCreateSensorIss                 = Sensor_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = Sensor_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = Sensor_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = Sensor_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = Sensor_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = Sensor_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = Sensor_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = Sensor_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = Sensor_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = Sensor_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = Sensor_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = Sensor_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = Sensor_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = Sensor_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = Sensor_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = Sensor_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = Sensor_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = Sensor_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = Sensor_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = Sensor_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = Sensor_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = Sensor_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = Sensor_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = Sensor_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = Sensor_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = Sensor_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = Sensor_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = Sensor_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = Sensor_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = Sensor_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = Sensor_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = Sensor_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = Sensor_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = Sensor_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = Sensor_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = Sensor_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = Sensor_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = Sensor_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = Sensor_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( SENSOR_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT Sensor_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( SENSOR_ERROR,  "%s: Can't allocate Sensor context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = SENSOR_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = SENSOR_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = SENSOR_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = SENSOR_SOFTWARE_RST_VALUE;
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
                while(Sensor_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = SENSOR_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = SENSOR_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = SENSOR_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = SENSOR_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = SENSOR_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = SENSOR_CHIP_ID_LOW_BYTE_DEFAULT;
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
    Sensor_IsiGetSensorIss,
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
    Sensor_IsiGetSensorI2cInfo,
};


