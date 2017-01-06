//OV8856_MIPI.c
/*****************************************************************************/
/*!
 *  \file        OV8856_MIPI.c \n
 *  \version     1.0 \n
 *  \author      ethan \n
 *  \Date: 2016-08-05 10:18:22 
 *  \brief       Image-sensor driver and other
 */
/*  
 */
/*****************************************************************************/

/******************************************************************************
 *
 *
 *****************************************************************************/
/**
 * @file OV8856.c
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

#include "OV8856_MIPI_priv.h"

#define  OV8856_NEWEST_TUNING_XML "2016-08-05_ethan-hkw_OV8856_LiteArray_GEGR160163-2_v1.0"

//hkw no use;
#define CC_OFFSET_SCALING  1.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV8856_INFO , "OV8856: ", INFO,    0U );
CREATE_TRACER( OV8856_WARN , "OV8856: ", WARNING, 1U );
CREATE_TRACER( OV8856_ERROR, "OV8856: ", ERROR,   1U );
CREATE_TRACER( OV8856_DEBUG, "OV8856: ", INFO,     0U );
CREATE_TRACER( OV8856_NOTICE0 , "OV8856: ", TRACE_NOTICE0, 1);
CREATE_TRACER( OV8856_NOTICE1, "OV8856: ", TRACE_NOTICE1, 1U );


#define OV8856_SLAVE_ADDR       0x6cU                           /**< i2c slave address of the OV8856 camera sensor */
#define OV8856_SLAVE_ADDR2      0x20U
#define OV8856_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the OV8856 integrated AD5820 */
#define Sensor_OTP_SLAVE_ADDR   0x6cU
#define Sensor_OTP_SLAVE_ADDR2   0x20U

#define OV8856_MAXN_GAIN 		(128.0f)
#define OV8856_MIN_GAIN_STEP   ( 1.0f / OV8856_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define OV8856_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
const char OV8856_g_acName[] = "OV8856_MIPI";
extern const IsiRegDescription_t OV8856_g_aRegDescription_twolane[];
extern const IsiRegDescription_t OV8856_g_1632x1224_twolane[];
extern const IsiRegDescription_t OV8856_g_1632x1224P30_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P25_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P20_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P15_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P10_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448_twolane[];
extern const IsiRegDescription_t OV8856_g_3264x2448P15_twolane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P7_twolane_fpschg[];

extern const IsiRegDescription_t OV8856_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t OV8856_g_1632x1224_fourlane[];
extern const IsiRegDescription_t OV8856_g_1632x1224P30_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P25_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P20_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P15_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_1632x1224P10_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448_fourlane[];
extern const IsiRegDescription_t OV8856_g_3264x2448P30_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P25_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P20_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P15_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P10_fourlane_fpschg[];
extern const IsiRegDescription_t OV8856_g_3264x2448P7_fourlane_fpschg[];

const IsiSensorCaps_t OV8856_g_IsiSensorDefaultConfig;



#define OV8856_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define OV8856_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define OV8856_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV8856_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV8856_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV8856_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV8856_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV8856_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV8856_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV8856_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV8856_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV8856_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV8856_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV8856_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV8856_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV8856_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT OV8856_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV8856_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV8856_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV8856_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV8856_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV8856_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV8856_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT OV8856_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV8856_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV8856_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV8856_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV8856_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV8856_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV8856_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV8856_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV8856_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV8856_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV8856_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV8856_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV8856_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV8856_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV8856_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT OV8856_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT OV8856_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT OV8856_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


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
static int OV8856_read_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address
){
    uint32_t temp = 0;
    if(OV8856_IsiRegReadIss(handle,address,&temp) != RET_SUCCESS){
        TRACE( OV8856_ERROR, "%s read OTP register 0x%x erro!\n", __FUNCTION__,address);
    }
    return temp;
}

static RESULT OV8856_write_i2c(    
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
){
    RESULT result = RET_SUCCESS;
    if((result = OV8856_IsiRegWriteIss(handle,address,value)) != RET_SUCCESS){
        TRACE( OV8856_ERROR, "%s write OTP register (0x%x,0x%x) erro!\n", __FUNCTION__,address,value);
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
int  RG_Ratio_Typical=0x0;
int  BG_Ratio_Typical=0x0;
bool bOTP_switch = false;

//#define  RG_Ratio_Typical_tongju (0x15c)
//#define  BG_Ratio_Typical_tongju (0x13f)

// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc
static int check_read_otp(
	sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	sensor_version_get_t* sensor_version_get_p,
	void* context,
	int camsys_fd
)
{
	return RET_NOTSUPP;
	
    struct otp_struct *otp_ptr = &g_otp_info;

    int otp_flag=0x0, addr, temp, i;
    //set 0x5002[3] to “0”
    int temp1;
    int i2c_base_info[3];
	int ret=0;

    i2c_base_info[0] = Sensor_OTP_SLAVE_ADDR; //otp i2c addr
    i2c_base_info[1] = 2; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size
    //stream on 
    ret = sensor_i2c_write_p(context,camsys_fd, OV8856_MODE_SELECT, OV8856_MODE_SELECT_ON, i2c_base_info );
    if(ret < 0){
		TRACE( OV8856_ERROR, "%s: Don't worry, we will try OTP slave addr2!\n", __FUNCTION__);
		i2c_base_info[0] = Sensor_OTP_SLAVE_ADDR2; //otp i2c addr
		ret = sensor_i2c_write_p(context,camsys_fd, OV8856_MODE_SELECT, OV8856_MODE_SELECT_ON, i2c_base_info );
		if(ret < 0)
			return RET_NOTSUPP;
		TRACE( OV8856_ERROR, "%s: OTP slave addr2 works!\n", __FUNCTION__);
	}
	
    temp1 = sensor_i2c_read_p(context,camsys_fd,0x5001,i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x5001, (0x00 & 0x08) | (temp1 & (~0x08)), i2c_base_info);
    // read OTP into buffer
    sensor_i2c_write_p(context,camsys_fd,0x3d84, 0xC0, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d88, 0x70, i2c_base_info); // OTP start address
    sensor_i2c_write_p(context,camsys_fd,0x3d89, 0x10, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d8A, 0x72, i2c_base_info); // OTP end address
    sensor_i2c_write_p(context,camsys_fd,0x3d8B, 0x0a, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x3d81, 0x01, i2c_base_info); // load otp into buffer
    osSleep(10);

	// OTP base information and WB calibration data
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7010, i2c_base_info);

    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7011; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7019; // base address of info group 3
    }
    if(addr != 0) {
        (*otp_ptr).flag = 0xc0; // valid info and AWB in OTP
        (*otp_ptr).module_integrator_id = sensor_i2c_read_p(context,camsys_fd,addr, i2c_base_info);
        (*otp_ptr).lens_id = sensor_i2c_read_p(context,camsys_fd, addr + 1, i2c_base_info);
        (*otp_ptr).production_year = sensor_i2c_read_p(context,camsys_fd, addr + 2, i2c_base_info);
        (*otp_ptr).production_month = sensor_i2c_read_p(context,camsys_fd, addr + 3, i2c_base_info);
        (*otp_ptr).production_day = sensor_i2c_read_p(context,camsys_fd,addr + 4, i2c_base_info);
		temp = sensor_i2c_read_p(context,camsys_fd,addr + 7, i2c_base_info);
		(*otp_ptr).rg_ratio = (sensor_i2c_read_p(context,camsys_fd,addr + 5, i2c_base_info)<<2) + ((temp>>6) & 0x03);
		(*otp_ptr).bg_ratio = (sensor_i2c_read_p(context,camsys_fd,addr + 6, i2c_base_info)<<2) + ((temp>>4) & 0x03);
    }
    else {
        (*otp_ptr).flag = 0x00; // not info and AWB in OTP
        (*otp_ptr).module_integrator_id = 0;
        (*otp_ptr).lens_id = 0;
        (*otp_ptr).production_year = 0;
        (*otp_ptr).production_month = 0;
        (*otp_ptr).production_day = 0;
		(*otp_ptr).rg_ratio = 0;
		(*otp_ptr).bg_ratio = 0;
    }

    // OTP VCM Calibration
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7021, i2c_base_info);
    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7022; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7025; // base address of info group 2
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
        TRACE( OV8856_ERROR, "%s no awb info in OTP!\n", __FUNCTION__);
    }
   
    // OTP Lenc Calibration
    otp_flag = sensor_i2c_read_p(context,camsys_fd,0x7028, i2c_base_info);
    addr = 0;
    int  checksum2=0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7029; // base address of Lenc Calibration group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x711a; // base address of Lenc Calibration group 2
    }
    if(addr != 0) {
        for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i]=sensor_i2c_read_p(context,camsys_fd,addr + i, i2c_base_info);
			checksum2 += (* otp_ptr).lenc[i];
        }
		checksum2 = (checksum2)%255 +1;
		(* otp_ptr).checksum = sensor_i2c_read_p(context,camsys_fd,addr + 240, i2c_base_info);
		if((* otp_ptr).checksum == checksum2){
			(*otp_ptr).flag |= 0x10;
		}
    }
    else {
        for(i=0;i<240;i++) {
        (* otp_ptr).lenc[i]=0;
        }
    }
    for(i=0x7010;i<=0x720a;i++) {
        sensor_i2c_write_p(context,camsys_fd,i,0, i2c_base_info); // clear OTP buffer, recommended use continuous write to accelarate
    }
    //set 0x5002[3] to “1”
    temp1 = sensor_i2c_read_p(context,camsys_fd,0x5001, i2c_base_info);
    sensor_i2c_write_p(context,camsys_fd,0x5001, (0x08 & 0x08) | (temp1 & (~0x08)), i2c_base_info);

    //stream off 
    sensor_i2c_write_p(context,camsys_fd, OV8856_MODE_SELECT, OV8856_MODE_SELECT_OFF, i2c_base_info );
    if((*otp_ptr).flag != 0)
        return RET_SUCCESS;
    else
        return RET_NOTSUPP;
}


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
        // no light source infomation in OTP ,light factor = 1
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
			OV8856_write_i2c(handle,0x5019, R_gain>>8);
			OV8856_write_i2c(handle,0x501a, R_gain & 0x00ff);
		}
		if (G_gain>0x400) {
			OV8856_write_i2c(handle,0x501b, G_gain>>8);
			OV8856_write_i2c(handle,0x501c, G_gain & 0x00ff);
		}
		if (B_gain>0x400) {
			OV8856_write_i2c(handle,0x501d, B_gain>>8);
			OV8856_write_i2c(handle,0x501e, B_gain & 0x00ff);
		}
			
      }


	if ((*otp_ptr).flag & 0x10) {
		temp = OV8856_read_i2c(handle,0x5000);
		temp = 0x20 | temp;
		OV8856_write_i2c(handle,0x5000, temp);
		for(i=0;i<240;i++) {
			OV8856_write_i2c(handle,0x5900 + i, (*otp_ptr).lenc[i]);
	}
	}

    TRACE( OV8856_NOTICE0,  "%s: success!!!\n",  __FUNCTION__ );
    return (*otp_ptr).flag;
}

static RESULT OV8856_IsiSetOTPInfo
(
    IsiSensorHandle_t       handle,
    uint32_t OTPInfo
)
{
	RESULT result = RET_SUCCESS;

    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

	RG_Ratio_Typical = OTPInfo>>16;
	BG_Ratio_Typical = OTPInfo&0xffff;
	TRACE( OV8856_ERROR, "%s:  ----AWB(RG,BG)->(0x%x, 0x%x)----\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);

	return (result);
}

static RESULT OV8856_IsiEnableOTP
(
    IsiSensorHandle_t       handle,
    const bool_t enable
)
{
	RESULT result = RET_SUCCESS;

    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }
	bOTP_switch = enable;
	return (result);
}
/* OTP END*/


/*****************************************************************************/
/**
 *          OV8856_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV8856 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV8856_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    OV8856_Context_t *pOV8856Ctx;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV8856Ctx = ( OV8856_Context_t * )malloc ( sizeof (OV8856_Context_t) );
    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR,  "%s: Can't allocate OV8856 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV8856Ctx, 0, sizeof( OV8856_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV8856Ctx );
        return ( result );
    }
    
    pOV8856Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pOV8856Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pOV8856Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pOV8856Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? OV8856_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV8856Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pOV8856Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pOV8856Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? OV8856_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV8856Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pOV8856Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pOV8856Ctx->Configured             = BOOL_FALSE;
    pOV8856Ctx->Streaming              = BOOL_FALSE;
    pOV8856Ctx->TestPattern            = BOOL_FALSE;
    pOV8856Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV8856Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pOV8856Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV8856Ctx->VcmInfo.RatedCurrent   = pOV8856Ctx->VcmInfo.StartCurrent + MAX_LOG*pOV8856Ctx->VcmInfo.Step;
    pOV8856Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pOV8856Ctx->IsiSensorMipiInfo.sensorHalDevID = pOV8856Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( OV8856_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pOV8856Ctx;

    result = HalSetCamConfig( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV8856 sensor instance.
 *
 * @param   handle      OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV8856_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV8856_IsiSensorSetStreamingIss( pOV8856Ctx, BOOL_FALSE );
    (void)OV8856_IsiSensorSetPowerIss( pOV8856Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV8856Ctx->IsiCtx.HalHandle );

    MEMSET( pOV8856Ctx, 0, sizeof( OV8856_Context_t ) );
    free ( pOV8856Ctx );

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCapsIss
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
static RESULT OV8856_IsiGetCapsIssInternal
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
 
static RESULT OV8856_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = OV8856_IsiGetCapsIssInternal(pIsiSensorCaps,pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV8856_g_IsiSensorDefaultConfig =
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
 *          OV8856_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8856 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT OV8856_SetupOutputFormat
(
    OV8856_Context_t       *pOV8856Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s%s (enter)\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV8856 sensor, so the YCSequence parameter is not checked */
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
            TRACE( OV8856_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( OV8856_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( OV8856_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV8856_INFO, "%s%s (exit)\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int OV8856_get_PCLK( OV8856_Context_t *pOV8856Ctx, int XVCLK)
{
    // calculate PCLK
    uint32_t SCLK, temp1, temp2;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_divs, Sys_divider2x, Sys_prediv, Sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_prediv_map[] = {1, 2, 4, 1};
	int Sclk_pdiv_map[] = { 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    
    //temp1 = ReadSCCB(0x6c, 0x3007);
    OV8856_IsiRegReadIss(  pOV8856Ctx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	OV8856_IsiRegReadIss(  pOV8856Ctx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	OV8856_IsiRegReadIss(  pOV8856Ctx, 0x030c, &temp1 );
	OV8856_IsiRegReadIss(  pOV8856Ctx, 0x030d, &temp2 );
	temp1 = temp1 & 0x03;
	Pll2_multiplier = (temp1<<8) + temp2;
	if(!Pll2_multiplier) {
 		Pll2_multiplier = 1;
 	}
	
	
    OV8856_IsiRegReadIss(  pOV8856Ctx, 0x030f, &temp1 );
	temp2 = temp1 & 0x0f;
	Pll2_divs = temp2 + 1;
	
	OV8856_IsiRegReadIss(  pOV8856Ctx, 0x030e, &temp1 );
	temp2 = temp1 & 0x07;
	Sys_divider2x = Pll2_sys_divider2x_map[temp1];

	OV8856_IsiRegReadIss(  pOV8856Ctx, 0x3106, &temp1 );
	temp2 = (temp1>>4) & 0x0f;
	
	Sclk_pdiv = Sclk_pdiv_map[temp2];
	
    temp2 = (temp1 >> 2) & 0x03;
	
	Sys_prediv = Sys_prediv_map[temp2];
	 
  	SCLK = XVCLK * 2 / Pll2_prediv0 / Pll2_prediv2x * Pll2_multiplier / Pll2_divs * 2 / Sys_divider2x / Sys_prediv / Sclk_pdiv;

	/*
	temp1 = SCLK>>8;
	OV8856_IsiRegWriteIss(pOV8856Ctx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	OV8856_IsiRegWriteIss(pOV8856Ctx, 0x350a, temp1);
	*/
	return SCLK;
}

/*****************************************************************************/
/**
 *          OV8856_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8856 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT OV8856_SetupOutputWindowInternal
(
    OV8856_Context_t        *pOV8856Ctx,
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
    
	TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

        pOV8856Ctx->IsiSensorMipiInfo.ulMipiFreq = 720;
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
						result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224_twolane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P30_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P25_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P20_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P10_twolane_fpschg);
                    }
        		}
                usTimeHts = 0x0ea0; 
                    
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x04de;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0x5d7;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0x074d;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x9bc;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x0e9a;
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
						result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448_twolane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P15_twolane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P7_twolane_fpschg);
                    }
        		    
        		}
        		
    			usTimeHts = 0x0f20;                
                if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x09b2;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x14c6;
                }
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }

            default:
            {
                TRACE( OV8856_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
    } else if(pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE) {
    	pOV8856Ctx->IsiSensorMipiInfo.ulMipiFreq = 720;

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
						result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224_fourlane);
                    }
     
                    if (pConfig->Resolution == ISI_RES_1632_1224P30) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_1632x1224P10_fourlane_fpschg);
                    }
        		}
    			usTimeHts = 0x78c; 
                if (pConfig->Resolution == ISI_RES_1632_1224P30) {
                    usTimeVts = 0x9b2;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P25) {
                    usTimeVts = 0xba2;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P20) {
                    usTimeVts = 0xe8b;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P15) {
                    usTimeVts = 0x1364;
                } else if (pConfig->Resolution == ISI_RES_1632_1224P10) {
                    usTimeVts = 0x1d16;
                }
                TRACE( OV8856_DEBUG, "%s ethan run 1632x1224,usTimeVts = %d",__FUNCTION__,usTimeVts);
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
						result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448_fourlane);
        		    }

                    if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P10_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_3264x2448P7_fourlane_fpschg);
                    }
        		    
        		}
        		
    			usTimeHts = 0x78c;
				
                if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                    usTimeVts = 0x9b2;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                    usTimeVts = 0xba2;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                    usTimeVts = 0xe8b;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x1364;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                    usTimeVts = 0x1d16;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x298d;
                }/**/
				TRACE( OV8856_DEBUG, "%s ethan run 3264x2448,usTimeVts = %d",__FUNCTION__,usTimeVts);
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }
        }        

    }
    
	
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	rVtPixClkFreq = OV8856_get_PCLK(pOV8856Ctx, xclk);
    
    // store frame timing for later use in AEC module
    pOV8856Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV8856Ctx->LineLengthPck    = usLineLengthPck;
    pOV8856Ctx->FrameLengthLines = usFrameLengthLines;

    TRACE( OV8856_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV8856 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT OV8856_SetupImageControl
(
    OV8856_Context_t        *pOV8856Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = OV8856_IsiRegReadIss(  pOV8856Ctx, OV8856_BLC_CTRL00, &RegValue );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = OV8856_IsiRegReadIss(  pOV8856Ctx, OV8856_BLC_CTRL00, &RegValue );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = OV8856_IsiRegReadIss(  pOV8856Ctx, OV8856_AEC_MANUAL, &RegValue );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = OV8856_IsiRegReadIss(  pOV8856Ctx, OV8856_ISP_CTRL01, &RegValue );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = OV8856_IsiRegReadIss(  pOV8856Ctx, OV8856_AEC_MANUAL, &RegValue );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = OV8856_IsiRegReadIss( pOV8856Ctx, OV8856_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = OV8856_IsiRegReadIss( pOV8856Ctx, OV8856_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV8856_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT OV8856_SetupOutputWindow
(
    OV8856_Context_t        *pOV8856Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV8856Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV8856Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return OV8856_SetupOutputWindowInternal(pOV8856Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          OV8856_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV8856-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV8856 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT OV8856_AecSetModeParameters
(
    OV8856_Context_t       *pOV8856Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"",
        pOV8856Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pOV8856Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( OV8856_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pOV8856Ctx->AecMaxIntegrationTime = ( ((float)(pOV8856Ctx->FrameLengthLines - 6)) * ((float)pOV8856Ctx->LineLengthPck) ) / pOV8856Ctx->VtPixClkFreq;
    pOV8856Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( OV8856_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"", pOV8856Ctx->AecMaxIntegrationTime  );

    pOV8856Ctx->AecMaxGain = OV8856_MAX_GAIN_AEC;
    pOV8856Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pOV8856Ctx->AecIntegrationTimeIncrement = ((float)pOV8856Ctx->LineLengthPck) / pOV8856Ctx->VtPixClkFreq;
    pOV8856Ctx->AecGainIncrement = OV8856_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pOV8856Ctx->AecCurGain               = pOV8856Ctx->AecMinGain;
    pOV8856Ctx->AecCurIntegrationTime    = 0.0f;
    pOV8856Ctx->OldCoarseIntegrationTime = 0;
    pOV8856Ctx->OldFineIntegrationTime   = 0;
    //pOV8856Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( OV8856_INFO, "%s%s (exit)\n", __FUNCTION__, pOV8856Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8856_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV8856 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV8856_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV8856Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV8856Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV8856_IsiRegWriteIss ( pOV8856Ctx, OV8856_SOFTWARE_RST, OV8856_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( OV8856_DEBUG, "%s: OV8856 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_MODE_SELECT, OV8856_MODE_SELECT_OFF );//OV8856_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: Can't write OV8856 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_aRegDescription );
   
    if(pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
        result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_aRegDescription_fourlane);
		TRACE( OV8856_DEBUG, "%s: ethan :apply 4lane registers OK)\n", __FUNCTION__ );
    }
	else if(pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
		result = IsiRegDefaultsApply( pOV8856Ctx, OV8856_g_aRegDescription_twolane);
		TRACE( OV8856_DEBUG, "%s: ethan :apply 2lane registers OK)\n", __FUNCTION__ );
	}
        

    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pOV8856Ctx, OV8856_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = OV8856_IsiRegWriteIss( pOV8856Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = OV8856_SetupOutputFormat( pOV8856Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = OV8856_SetupOutputWindow( pOV8856Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV8856_SetupImageControl( pOV8856Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV8856_AecSetModeParameters( pOV8856Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pOV8856Ctx->Configured = BOOL_TRUE;
    }

    //set OTP info

    result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_MODE_SELECT, OV8856_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: Can't write OV8856 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }

    //set OTP
	
    char prop_value[PROPERTY_VALUE_MAX];
	property_get("sys_graphic.cam_otp", prop_value, "true");
    //set OTP
	bOTP_switch = false;
    if(bOTP_switch && !strcmp(prop_value,"true")){
        //struct otp_struct  otp_ptr;
        //read_otp(handle,&otp_ptr);
        if(g_otp_info.flag != 0){
            TRACE( OV8856_NOTICE0, "%s: apply OTP info !!\n", __FUNCTION__);
			apply_otp(handle,&g_otp_info);
        }
    }
    
    result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_MODE_SELECT, OV8856_MODE_SELECT_OFF );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: Can't write OV8856 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
   

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiChangeSensorResolutionIss
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
static RESULT OV8856_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV8856Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (OV8856_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV8856Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV8856Ctx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV8856Ctx->Config.Resolution))) ) {

            if (pOV8856Ctx->Streaming != BOOL_FALSE) {
                TRACE( OV8856_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV8856_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pOV8856Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV8856_SetupOutputWindowInternal( pOV8856Ctx, &pOV8856Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( OV8856_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV8856Ctx->AecCurGain;
        float OldIntegrationTime = pOV8856Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV8856_AecSetModeParameters( pOV8856Ctx, &pOV8856Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV8856_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV8856_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV8856_ERROR, "%s: OV8856_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( OV8856_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8856_IsiSensorSetStreamingIss
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
static RESULT OV8856_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV8856Ctx->Configured != BOOL_TRUE) || (pOV8856Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8856_IsiRegWriteIss ( pOV8856Ctx, OV8856_MODE_SELECT, (RegValue | OV8856_MODE_SELECT_ON) );//OV8856_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8856_IsiRegWriteIss ( pOV8856Ctx, OV8856_MODE_SELECT, (RegValue & ~OV8856_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV8856Ctx->Streaming = on;
    }

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV8856 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT OV8856_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV8856Ctx->Configured = BOOL_FALSE;
    pOV8856Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV8856_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV8856_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( OV8856_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8856_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8856_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV8856_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV8856Ctx->IsiCtx.HalHandle, pOV8856Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT OV8856_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV8856_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 16U) | (OV8856_CHIP_ID_MIDDLE_BYTE_DEFAULT<<8U);
    RevId = RevId | OV8856_CHIP_ID_LOW_BYTE_DEFAULT;

    result = OV8856_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV8856_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( OV8856_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetSensorRevisionIss
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
static RESULT OV8856_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV8856_IsiRegReadIss ( handle, OV8856_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 16U );
    result = OV8856_IsiRegReadIss ( handle, OV8856_CHIP_ID_MIDDLE_BYTE, &data );
    *p_value |= ( (data & 0xFF) << 8U );
    result = OV8856_IsiRegReadIss ( handle, OV8856_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiRegReadIss
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
static RESULT OV8856_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV8856_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( OV8856_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiRegWriteIss
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
static RESULT OV8856_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV8856_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( OV8856_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV8856 instance
 *
 * @param   handle       OV8856 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT OV8856_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV8856_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pOV8856Ctx->AecMinGain;
    *pMaxGain = pOV8856Ctx->AecMaxGain;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV8856 instance
 *
 * @param   handle       OV8856 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT OV8856_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV8856_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pOV8856Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pOV8856Ctx->AecMaxIntegrationTime;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8856_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV8856 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT OV8856_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

	
	result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_AGC_ADJ_H, &data);
	TRACE( OV8856_INFO, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_AGC_ADJ_L, &data);
	TRACE( OV8856_INFO, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / OV8856_MAXN_GAIN;
	
    //*pSetGain = pOV8856Ctx->AecCurGain;
    

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV8856 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT OV8856_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV8856Ctx->AecGainIncrement;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  OV8856 sensor instance handle
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
RESULT OV8856_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
	uint32_t data= 0;
	uint32_t result_gain= 0;

    TRACE( OV8856_INFO, "%s: (enter) pOV8856Ctx->AecMaxGain(%f) \n", __FUNCTION__,pOV8856Ctx->AecMaxGain);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( OV8856_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pOV8856Ctx->AecMinGain ) NewGain = pOV8856Ctx->AecMinGain;
    if( NewGain > pOV8856Ctx->AecMaxGain ) NewGain = pOV8856Ctx->AecMaxGain;

    usGain = (uint16_t)(NewGain * OV8856_MAXN_GAIN+0.5); //大概加0.5 hkw

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pOV8856Ctx->OldGain) )
    {
        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_AGC_ADJ_H, (usGain>>8)&0x07); //fix by OV8856 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_AGC_ADJ_L, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pOV8856Ctx->OldGain = usGain;

		/*osSleep(30);
		result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_AGC_ADJ_H, &data);
		TRACE( OV8856_ERROR, " -------reg35088888888:%x-------\n",data );
		result_gain = (data & 0x07) ;
		result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_AGC_ADJ_L, &data);
		TRACE( OV8856_ERROR, " -------reg35099999999:%x-------\n",data );
		result_gain = (result_gain<<8) + data;*/
		
    }

    //calculate gain actually set
    pOV8856Ctx->AecCurGain = ( (float)usGain ) / OV8856_MAXN_GAIN;

    //return current state
    *pSetGain = pOV8856Ctx->AecCurGain;
    TRACE( OV8856_INFO, "-----------%s: psetgain=%f, NewGain=%f,result_gain=%x\n", __FUNCTION__, *pSetGain, NewGain,result_gain);

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8856_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV8856 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT OV8856_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV8856Ctx->AecCurIntegrationTime;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV8856 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT OV8856_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV8856Ctx->AecIntegrationTimeIncrement;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  OV8856 sensor instance handle
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
RESULT OV8856_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;
	uint32_t result_intertime= 0;
	
    //uint32_t FineIntegrationTime   = 0; //not supported by OV8856

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( OV8856_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pOV8856Ctx->AecMinIntegrationTime,
        pOV8856Ctx->AecMaxIntegrationTime);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( OV8856_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pOV8856Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV8856Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pOV8856Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV8856Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by OV8856
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pOV8856Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pOV8856Ctx->LineLengthPck == 0 )
    {
        TRACE( OV8856_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV8856Ctx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pOV8856Ctx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV8856Ctx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if(CoarseIntegrationTime < 6)
		CoarseIntegrationTime = 6;
    if( CoarseIntegrationTime != pOV8856Ctx->OldCoarseIntegrationTime )
    {//
        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_EXPO_H, (CoarseIntegrationTime & 0x0000F000U) >> 12U );//fix by OV8856 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_EXPO_M, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_AEC_EXPO_L, (CoarseIntegrationTime & 0x0000000FU) << 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pOV8856Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
        
		/*osSleep(30);
		result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_EXPO_H, &data);
		TRACE( OV8856_ERROR, " -------reg3500:%x-------\n",data );
		result_intertime = (data & 0x0f) << 8;
		result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_EXPO_M, &data);
		TRACE( OV8856_ERROR, " -------reg3501:%x-------\n",data );
		result_intertime = result_intertime + data;
		result = OV8856_IsiRegReadIss ( pOV8856Ctx, OV8856_AEC_EXPO_L, &data);
		TRACE( OV8856_ERROR, " -------reg3502:%x-------\n",data );
		result_intertime = (result_intertime << 4) + (data >> 4);*/
		
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //calculate integration time actually set
    //pOV8856Ctx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pOV8856Ctx->LineLengthPck) + ((float)FineIntegrationTime) ) / pOV8856Ctx->VtPixClkFreq;
    pOV8856Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pOV8856Ctx->LineLengthPck) / pOV8856Ctx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pOV8856Ctx->AecCurIntegrationTime;

    TRACE( OV8856_DEBUG, "%s:\n"
         "pOV8856Ctx->VtPixClkFreq:%f pOV8856Ctx->LineLengthPck:%x \n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, 
         pOV8856Ctx->VtPixClkFreq,pOV8856Ctx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x0000F000U) >> 12U ,
         (CoarseIntegrationTime & 0x00000FF0U) >> 4U,
         (CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          OV8856_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV8856 sensor instance handle
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
RESULT OV8856_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV8856_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV8856_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = OV8856_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = OV8856_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( OV8856_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT OV8856_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV8856Ctx->AecCurGain;
    *pSetIntegrationTime = pOV8856Ctx->AecCurIntegrationTime;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetResolutionIss
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
RESULT OV8856_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV8856Ctx->Config.Resolution;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV8856Ctx             OV8856 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV8856_IsiGetAfpsInfoHelperIss(
    OV8856_Context_t   *pOV8856Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV8856Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV8856Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV8856_SetupOutputWindowInternal( pOV8856Ctx, &pOV8856Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV8856_AecSetModeParameters( pOV8856Ctx, &pOV8856Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV8856_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV8856Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV8856Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV8856Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV8856Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV8856Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8856_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV8856 sensor instance handle
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
RESULT OV8856_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        TRACE( OV8856_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV8856Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pOV8856Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pOV8856Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pOV8856Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    OV8856_Context_t *pDummyCtx = (OV8856_Context_t*) malloc( sizeof(OV8856_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( OV8856_ERROR,  "%s: Can't allocate dummy OV8856 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pOV8856Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = OV8856_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes)
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
                    TRACE( OV8856_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
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
                    TRACE( OV8856_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
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
            TRACE( OV8856_ERROR,  "%s: pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibKFactor
 *
 * @brief   Returns the OV8856 specific K-Factor
 *
 * @param   handle       OV8856 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV8856_KFactor;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV8856 specific PCA-Matrix
 *
 * @param   handle          OV8856 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV8856_PCAMatrix;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV8856 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV8856_SVDMeanValue;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV8856 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&OV8856_CenterLine;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV8856 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&OV8856_AwbClipParm;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV8856 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV8856_AwbGlobalFadeParm;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV8856 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV8856_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV8856_AwbFade2Parm;

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV8856_IsiGetIlluProfile
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
static RESULT OV8856_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
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
        for ( i=0U; i<OV8856_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV8856_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV8856_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetLscMatrixTable
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
static RESULT OV8856_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
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
                if ( ( pOV8856Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pOV8856Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV8856_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pOV8856Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pOV8856Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV8856_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pOV8856Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pOV8856Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV8856_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pOV8856Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pOV8856Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV8856_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pOV8856Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pOV8856Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV8856_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV8856_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV8856_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          OV8856_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT OV8856_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV8856 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT OV8856_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    //TRACE( OV8856_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
 if ((pOV8856Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pOV8856Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pOV8856Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( OV8856_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = OV8856_IsiMdiFocusSet( handle, MAX_LOG );

    //TRACE( OV8856_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV8856 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT OV8856_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( OV8856_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( OV8856_ERROR, "%s: pOV8856Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pOV8856Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pOV8856Ctx->VcmInfo.StartCurrent + (pOV8856Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( OV8856_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pOV8856Ctx->VcmInfo.StepMode );
	
    //TRACE( OV8856_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pOV8856Ctx->IsiCtx.HalHandle,
                             pOV8856Ctx->IsiCtx.I2cAfBusNum,
                             pOV8856Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pOV8856Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( OV8856_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV8856 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT OV8856_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( OV8856_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV8856Ctx->IsiCtx.HalHandle,
                            pOV8856Ctx->IsiCtx.I2cAfBusNum,
                            pOV8856Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pOV8856Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( OV8856_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

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
	if( *pAbsStep <= pOV8856Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pOV8856Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pOV8856Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pOV8856Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pOV8856Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( OV8856_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV8856_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV8856_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT OV8856_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV8856_IsiRegReadIss( pOV8856Ctx, OV8856_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV8856_IsiRegReadIss( pOV8856Ctx, OV8856_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV8856_IsiRegWriteIss( pOV8856Ctx, OV8856_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pOV8856Ctx->TestPattern = enable;
    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          OV8856_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV8856 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT OV8856_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pOV8856Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pOV8856Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pOV8856Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( OV8856_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV8856_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
    	TRACE( OV8856_ERROR, "%s: pOV8856Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV8856_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV8856_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV8856_Context_t *pOV8856Ctx = (OV8856_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV8856_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV8856Ctx == NULL )
    {
    	TRACE( OV8856_ERROR, "%s: pOV8856Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV8856_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV8856_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          OV8856_IsiGetSensorIss
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
RESULT OV8856_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV8856_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = OV8856_g_acName;
        pIsiSensor->pRegisterTable                      = OV8856_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &OV8856_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= OV8856_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= OV8856_IsiGetSensorTuningXmlVersion;//oyyf
		pIsiSensor->pIsiCheckOTPInfo                    = check_read_otp;//zyc
		pIsiSensor->pIsiSetSensorOTPInfo				= OV8856_IsiSetOTPInfo;
		pIsiSensor->pIsiEnableSensorOTP					= OV8856_IsiEnableOTP;
        pIsiSensor->pIsiCreateSensorIss                 = OV8856_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = OV8856_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = OV8856_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = OV8856_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = OV8856_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = OV8856_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = OV8856_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = OV8856_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = OV8856_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = OV8856_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = OV8856_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = OV8856_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = OV8856_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = OV8856_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = OV8856_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV8856_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV8856_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV8856_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV8856_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV8856_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV8856_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV8856_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV8856_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = OV8856_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = OV8856_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = OV8856_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = OV8856_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = OV8856_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = OV8856_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = OV8856_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = OV8856_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = OV8856_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = OV8856_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = OV8856_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = OV8856_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = OV8856_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = OV8856_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = OV8856_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = OV8856_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV8856_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT OV8856_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV8856_ERROR,  "%s: Can't allocate OV8856 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV8856_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = OV8856_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = OV8856_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = OV8856_SOFTWARE_RST_VALUE;
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
                while(OV8856_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = OV8856_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = OV8856_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = OV8856_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = OV8856_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV8856_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = OV8856_CHIP_ID_LOW_BYTE_DEFAULT;
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
    OV8856_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,						/**< IsiSensor_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiSensor_t.pIsiCheckOTPInfo>*/  //zyc 
        0,						/**< IsiSensor_t.pIsiSetSensorOTPInfo>*/
        0,						/**< IsiSensor_t.pIsiEnableSensorOTP>*/
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
    OV8856_IsiGetSensorI2cInfo,
};


