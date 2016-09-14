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
 * @file OV2715_priv.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup ov2715_priv
 * @{
 *
 */
#ifndef __OV2715_PRIV_H__
#define __OV2715_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>



#ifdef __cplusplus
extern "C"
{
#endif

/*v0.1.0:
*   1).add senosr drv version in get sensor i2c info func
*v0.2.0:
*   1). support for isi v0.5.0
*v0.3.0
*   1). support for isi v0.0xc.0
*   2). change VPol from ISI_VPOL_NEG to ISI_VPOL_POS
*v0.4.0
*   1). support isi v0.0xd.0
*/
#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 4, 0) 


/*****************************************************************************
 * SC control registers
 *****************************************************************************/
#define OV2715_SYSTEM_CONTROL00             (0x3008)  //RW - System Control
                                                      //     Bit[7]:   System software reset
                                                      //        0: Normal work mode
                                                      //        1: Software reset mode
                                                      //     Bit[6]:   System sleep mode
                                                      //          0: Normal work mode
                                                      //          1: Software sleep power down mode
                                                      //     Bit[5:1]: Debug only Changing these values are not allowed
                                                      //     Bit[0]:   Not used
// (0x3009) reserved
#define OV2715_PIDH                         (0x300A)  //R  - Product ID High Byte MSBs
#define OV2715_PIDL                         (0x300B)  //R  - Product ID Low Byte LSBs
// (0x300C~ 0x300D) reserved
#define OV2715_MIPI_CTRL00                  (0x300E)  //RW - Bit[7:5]: Not used
                                                      //     Bit[4]:   MIPI high speed power down control
                                                      //        0:  Enable MIPI PHY HS TX module
                                                      //        1:  Power down MIPI PHY HS TX module
                                                      //     Bit[3]:   MIPI low power, power-down control
                                                      //        0:  Enable MIPI PHY HS RX module
                                                      //        1:  Power down MIPI PHY LP RX module
                                                      //     Bit[2]:   MIPI enable
                                                      //        0:  DVP enable
                                                      //          1:  MIPI enable
                                                      //     Bit[1]:   MIPI system suspend control
                                                      //        0:  MIPI power ON
                                                      //        1:  MIPI suspend
                                                      //     Bit[0]:   Not used
#define OV2715_PLL_CTRL00                   (0x300F)  //RW - Bit[7:6]: Not used as of datasheet / Div1to2p5 as of ov2710_PLL_Dreamchip.xls
                                                      //     Bit[5:3]: Charge pump control
                                                      //     Bit[2]:   Not used
                                                      //     Bit[1:0]: PLL SELD5 divider
                                                      //        0x:   Bypass
                                                      //        10: Divided by 4 when in 8-bit mode
                                                      //        11: Divided by 5 when in 10-bit mode
#define OV2715_PLL_CTRL01                   (0x3010)  //RW - Bit[7:4]: PLL DIVS divider System divider ratio
                                                      //     Bit[3:0]: PLL DIVM divider MIPI divider ratio
#define OV2715_PLL_CTRL02                   (0x3011)  //RW - Bit[7]:   PLL bypass
                                                      //     Bit[5:0]: PLL DIVP
#define OV2715_PLL_PREDIVIDER               (0x3012)  //RW - Bit[7:3]:  Not used
                                                      //     Bit[2:0]:  PLL1 pre-divider ratio
                                                      //         000: 1
                                                      //         001: 1.5
                                                      //         010: 2
                                                      //         011: 2.5
                                                      //         100: 3
                                                      //         101: 4
                                                      //           110: 6
                                                      //           111: 8
// (0x3013~ 0x3015) reserved
#define OV2715_PAD_OUTPUT_ENABLE00          (0x3016)  //RW - Input/Output Control (0: input; 1: output)
                                                      //     Bit[7:2]: Not used
                                                      //     Bit[1]:   Strobe OE
                                                      //     Bit[0]:   SDA OEN trobe O
#define OV2715_PAD_OUTPUT_ENABLE01          (0x3017)  //RW - Input/Output Control (0: input; 1: output)
                                                      //     Bit[7]:   Not used
                                                      //     Bit[6]:   VSYNC OEN
                                                      //     Bit[5]:   HREF OEN
                                                      //     Bit[4]:   PCLK OEN
                                                      //     Bit[3:0]: D[9:6] OEN
#define OV2715_PAD_OUTPUT_ENABLE02          (0x3018)  //RW - Input/Output Control (0: input; 1: output)
                                                      //     Bit[7:2]: D[5:0] OEN
                                                      //     Bit[1:0]: Not used
#define OV2715_PAD_OUTPUT_VALUE00           (0x3019)  //RW - GPIO Output Value
                                                      //     Bit[7:2]: Not used
                                                      //     Bit[1]:   Strobe
                                                      //     Bit[0]:   SDA
#define OV2715_PAD_OUTPUT_VALUE01           (0x301A)  //RW - GPIO Output Value
                                                      //     Bit[7]:    Not used
                                                      //     Bit[6]:    VSYNC
                                                      //     Bit[5]:    HREF
                                                      //     Bit[4]:    PCLK
                                                      //     Bit[3:0]:  D[9:6] output
#define OV2715_PAD_OUTPUT_VALUE02           (0x301B)  //RW - GPIO Output Value
                                                      //     Bit[7:2]: D[5:0] output
                                                      //     Bit[1:0]: Not used
#define OV2715_PAD_OUTPUT_SELECT00          (0x301C)  //RW - Output Selection for GPIO (0: normal data path; 1: register controlled GPIO)
                                                      //     Bit[7:2]: Not used
                                                      //     Bit[1]:   Strobe select
                                                      //     Bit[0]:   SDA select
#define OV2715_PAD_OUTPUT_SELECT01          (0x301D)  //RW - Output Selection for GPIO (0: normal data path; 1: register controlled GPIO)
                                                      //     Bit[7]:   Not used
                                                      //     Bit[6]:   VSYNC select
                                                      //     Bit[5]:   HREF select
                                                      //     Bit[4]:   PCLK select
                                                      //     Bit[3:0]: D[9:6] select
#define OV2715_PAD_OUTPUT_SELECT02          (0x301E)  //RW - Output Selection for GPIO (0: normal data path; 1: register controlled GPIO)
                                                      //     Bit[7:2]: D[5:0] select
                                                      //     Bit[1:0]: Not used
#define OV2715_CHIP_REVISION                (0x3025)  //R  - Chip Revision ID (for Rev 1C only)
#define OV2715_PAD_OUTPUT_DRIVE_CAPABILITY  (0x302C)  //RW - Bit[7:6]: Output drive capability
                                                      //        00: 1x
                                                      //        01: 2x
                                                      //        10: 3x
                                                      //        11: 4x
                                                      //     Bit[5:0]: Not used
#define OV2715_PAD_INPUT_VALUE00            (0x3040)  //R  - Pad Input Status
                                                      //     Bit[7:6]: Not used
                                                      //     Bit[5]:   TM
                                                      //     Bit[4]:   Not used
                                                      //     Bit[3]:   PWDN
                                                      //     Bit[2]:   PWUP
                                                      //     Bit[1]:   SCL
                                                      //     Bit[0]:   SDA
#define OV2715_PAD_INPUT_VALUE01           (0x3041)   //R  - Pad Input Status
                                                      //     Bit[7]: OTP
                                                      //     Bit[6]: VSYNC
                                                      //     Bit[5]: HREF
                                                      //     BIt[4]: PCLK
                                                      //     Bit[3:0]: D[9:6]input
#define OV2715_PAD_INPUT_VALUE02           (0x3042)   //R  - Pad Input Status
                                                      //     Bit[7:2]: D[5:0]input
                                                      //     Bit[1:0]: Not used
/*****************************************************************************
 * SCCB control registers
 *****************************************************************************/
#define OV2715_SCCB_ID                   (0x3100) //RW  - SCCB Slave ID
#define OV2715_PLL_CLOCK_SELECT          (0x3103) //RW  - PLL Clock Select
                                                  //  Bit[7:2]: Not used
                                                  //  Bit[1]: Select PLL input clock
                                                  //  0: From pad clock
                                                  //  1: From pre divider
                                                  //  (clock modulator)
#define OV2715_SCCB_PAD_CLOCK_DIVIDER    (0x3104) //RW  - Pad Clock Divider for SCCB Clock

/*****************************************************************************
 * group sharing registers
 *****************************************************************************/
#define OV2715_GROUP_ADDR0            (0x3200) //RW - Start Address for Group0 {group_addr0[7:0], 4'h0}
#define OV2715_GROUP_ADDR1            (0x3201) //RW - Start Address for Group1 {group_addr1[7:0], 4'h0}
#define OV2715_GROUP_ADDR2            (0x3202) //RW - Start Address for Group2 {group_addr2[7:0], 4'h0}
#define OV2715_GROUP_ADDR3            (0x3203) //RW - Start Address for Group3 {group_addr3[7:0], 4'h0}
#define OV2715_GROUP_ACCESS           (0x3212) //RW - Bit[7]:  group_launch_en
                                               //     Bit[6]:  Debug mode (must be 0)
                                               //     Bit[5]:  group_launch
                                               //     Bit[4]:  group_hold_end
                                               //     Bit[3:0]:  group_id
                                               //      00~11: ID of the group to hold register
/*****************************************************************************
 * analog registers
 *****************************************************************************/
#define OV2715_ANA_ARRAY_01         (0x3621) //RW - Array Control
                                             //     Bit[7]:   Horizontal binning
                                             //     Bit[6]:   Horizontal skip
#define OV2715_SENSOR_REG0D         (0x370D) //RW - Bit[7]:   Debug mode
                                             //     Bit[6]:   Vertical binning
                                             //     Bit[5:0]: Debug mode
/*****************************************************************************
 * timing control registers
 *****************************************************************************/
#define OV2715_TIMING_CONTROL_HS_HIGHBYTE   (0x3800) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: HREF horizontal start point[11:8]
#define OV2715_TIMING_CONTROL_VH_LOWBYTE0   (0x3801) //RW -  Bit[7:0]: HREF horizontal start point[7:0]
#define OV2715_TIMING_CONTROL_VH_HIGHBYTE0  (0x3802) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: HREF vertical start point[11:8]
#define OV2715_TIMING_CONTROL_VH_LOWBYTE1   (0x3803) //RW -  Bit[7:0]: HREF vertical start point[7:0]
#define OV2715_TIMING_CONTROL_HW_HIGHBYTE   (0x3804) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: HREF horizontal width[11:8]
#define OV2715_TIMING_CONTROL_HW_LOWBYTE    (0x3805) //RW -  Bit[7:0]: HREF horizontal width[7:0]
#define OV2715_TIMING_CONTROL_VH_HIGHBYTE1  (0x3806) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: HREF vertical height[11:8]
#define OV2715_TIMING_CONTROL_VH_LOWBYTE2   (0x3807) //RW -  Bit[7:0]: HREF vertical height[7:0]
#define OV2715_TIMING_CONTROL_DVP_HSIZE     (0x3808) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: DVP output horizontal width[11:8]
#define OV2715_TIMING_CONTROL_DVP_HSIZELOW  (0x3809) //RW -  Bit[7:0]: DVP output horizontal width[7:0]
#define OV2715_TIMING_CONTROL_DVP_VSIZEHIGH (0x380A) //RW -  Bit[7:4]: Not used
                                                     //       Bit[3:0]: DVP output vertical height[11:8]
#define OV2715_TIMING_CONTROL_DVP_VSIZELOW  (0x380B) //RW -  Bit[7:0]: DVP output vertical height[7:0]
#define OV2715_TIMING_CONTROL_HTS_HIGHBYTE  (0x380C) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: Total horizontal size[11:8]
#define OV2715_TIMING_CONTROL_HTS_LOWBYTE   (0x380D) //RW -  Bit[7:0]: Total horizontal size[7:0]
#define OV2715_TIMING_CONTROL_VTS_HIGHBYTE  (0x380E) //RW -  Bit[7:4]: Not used
                                                     //      Bit[3:0]: Total vertical size[11:8]
#define OV2715_TIMING_CONTROL_VTS_LOWBYTE   (0x380F) //RW -  Bit[7:0]: Total vertical size[7:0]
#define OV2715_TIMING_CONTROL_HV_OFFSET     (0x3810) //RW -  Bit[7:4]: Horizontal offset
                                                     //     Bit[3:0]: Vertical offset
// (0x3814~ 0x3815) reserved
#define OV2715_TIMING_CONTROL18             (0x3818) //RW -  Bit[7]:   Not used
                                                     //  Bit[6]:   Horizontal mirror
                                                     //  Bit[5]:   Vertical flip
                                                     //  Bit[4:2]: Not used
                                                     //   Bit[1]:   Vertical subsample /4
                                                     //   Bit[0]:   Vertical subsample /2
/*****************************************************************************
 * AEC/AGC registers
 *****************************************************************************/
#define OV2715_AEC_PK_EXPO0         (0x3500) //RW - Bit[7:4]: Not used
                                             // Bit[3:0]: AEC exposure[19:16]
#define OV2715_AEC_PK_EXPO1         (0x3501) //RW - Bit[7:0]: AEC exposure[15:8]
#define OV2715_AEC_PK_EXPO2         (0x3502) //RW - Bit[7:0]: AEC exposure[7:0]
#define OV2715_AEC_PK_MANUAL        (0x3503) //RW - Bit[7:3]: Not used
                                             //     Bit[2]: VTS manual enable
                                             //     Bit[1]: AGC manual enable
                                             //     Bit[0]: AEC manual enable
// (0x3505~ 0x3509) reserved
#define OV2715_AEC_AGC_ADJ0         (0x350A) //RW - Gain Output to Sensor
                                             //     Bit[7:1]: Not used
                                             //   Bit[0]: Gain high bit
                                             //   Gain = (0x350A[0]+1) x(0x350B[7]+1) x
                                             //   (0x350B[6]+1) x(0x350B[5]+1) x
                                             //   (0x350B[4]+1) x(0x350B[3:0]/16+1)
#define OV2715_AEC_AGC_ADJ1         (0x350B) //RW - Gain Output to Sensor
                                             //     Bit[7:0]: Gain low bits
                                             //     Gain = (0x350A[0]+1) x(0x350B[7]+1) x
                                             //     (0x350B[6]+1) x(0x350B[5]+1) x
                                             //     (0x350B[4]+1) x(0x350B[3:0]/16+1)
#define OV2715_AEC_PK_VTS0          (0x350C) //RW - Bit[7:0]: AEC VTS[15:8]
#define OV2715_AEC_PK_VTS1          (0x350D) //RW - Bit[7:0]: AEC VTS[7:0]
#define OV2715_AEC_CONTROL0         (0x3A00) //RW - Bit[7]:   Not used
                                             //     Bit[6]:   Less 1 line enable
                                             //       0:  Less 1 line function disable
                                             //       1:  Less 1 line function enable
                                             //     Bit[5]:   Band enable
                                             //       0:  Band function disable
                                             //       1:  Band function enable
                                             //     Bit[4]:   Auto band enable
                                             //       0:  Exposure can not be less than 1 band
                                             //       1:  Exposure can be less than 1 band
                                             //     Bit[3]:   Line complete
                                             //       0:  Fractional line enable
                                             //       1:  Fractional line disable
                                             //     Bit[2]:   Night mode
                                             //       0:  Night mode disable
                                             //       1:  Night mode enable
                                             //     Bit[1]:   Not used
                                             //     Bit[0]:   Freeze
                                             //         0:  Freeze disable
                                             //         1:  Freeze enable
#define OV2715_AEC_CONTROL1         (0x3A01) //RW - Bit[7:0]:   Minimum exposure
#define OV2715_AEC_MAX_EXPO_60A     (0x3A02) //RW - Bit[7:4]: Not used
                                             // Bit[3:0]: AEC maximum exposure for 60Hz[1   9:16]
#define OV2715_AEC_MAX_EXPO_60B     (0x3A03) //RW - Bit[7:0]: AEC maximum exposure for 60Hz[15:8]
#define OV2715_AEC_MAX_EXPO_60C     (0x3A04) //RW - Bit[7:0]: AEC maximum exposure for 60Hz[7:0]
// (0x3A05~ 0x3A07) reserved
#define OV2715_AEC_B50_STEP0        (0x3A08) //RW - Bit[7:6]: Not used
                                             // Bit[5:0]: AEC band50 step[13:8]
#define OV2715_AEC_B50_STEP1        (0x3A09) //RW - Bit[7:0]: AEC band50 step[7:0]
#define OV2715_AEC_B60_STEP0        (0x3A0A) //RW - Bit[7:6]: Not used
                                             // Bit[5:0]: AEC band60 step[13:8]
#define OV2715_AEC_B60_STEP1        (0x3A0B) //RW - Bit[7:0]: AEC band60 step[7:0]
// (0x3A0C) reserved
#define OV2715_AEC_CONTROLD         (0x3A0D) //RW - Bit[7:6]: Not used
                                             // Bit[5:0]: Band60 max[5:0]
#define OV2715_AEC_CONTROLE         (0x3A0E) //RW - Bit[7:6]: Not used
                                             // Bit[5:0]: Band50 max[5:0]
#define OV2715_AEC_CONTROLF         (0x3A0F) //RW - Bit[7:0]: Stable range high threshold 1
#define OV2715_AEC_CONTROL10        (0x3A10) //RW - Bit[7:0]: Stable range low threshold 1
#define OV2715_AEC_CONTROL11        (0x3A11) //RW - Bit[7:0]: Fast zone high threshold
#define OV2715_AEC_CONTROL12        (0x3A12) //RW - Bit[7:0]: Manual average setting
#define OV2715_AEC_CONTROL13        (0x3A13) //RW - Bit[7:6]: Not used
                                             // Bit[5]:   Pre gain enable
                                             // Bit[4:0]: Pre gain
#define OV2715_AEC_MAX_EXPO_50A     (0x3A14) //RW - Bit[7:4]: Not used
                                             // Bit[3:0]: AEC maximum exposure for 50Hz[19:16]
#define OV2715_AEC_MAX_EXPO_50B     (0x3A15) //RW - Bit[7:0]: AEC maximum exposure for 50Hz[15:8]
#define OV2715_AEC_MAX_EXPO_50C     (0x3A16) //RW - Bit[7:0]: AEC maximum exposure for 50Hz[7:0]
#define OV2715_AEC_CONTROL17        (0x3A17) //RW - Bit[7:2]: Not used
                                             //     Bit[1:0]: Gain night threshold[1:0]
                                             //         00: Night mode gain threshold as 1x
                                             //         01: Night mode gain threshold as 2x
                                             //   10: Night mode gain threshold as 4x
                                             //   11: Not used
#define OV2715_AEC_G_CEIL0          (0x3A18) //RW - Bit[7:1]: Not used
                                             // Bit[0]:   AEC gain ceiling
#define OV2715_AEC_G_CEIL1          (0x3A19) //RW - Bit[7:0]: AEC gain ceiling
// (0x3A1A)  reserved as of datasheet, but will be used, see undocumented registers
#define OV2715_AEC_CONTROL1B        (0x3A1B) //RW - Bit[7:0]: Stable range high threshold 2
#define OV2715_AEC_LED_ADD_ROW0     (0x3A1C) //RW - Bit[7:0]: Row adding number[15:8] in AEC LED mode
#define OV2715_AEC_LED_ADD_ROW1     (0x3A1D) //RW - Bit[7:0]: Row adding number[7:0] in AEC LED mode
#define OV2715_AEC_CONTROL1E        (0x3A1E) //RW - Bit[7:0]: Stable range low threshold 2
#define OV2715_AEC_CONTROL1F        (0x3A1F) //RW - Bit[7:0]: Fast zone low threshold
#define OV2715_AEC_CONTROL20        (0x3A20) //RW - Bit[7:3]:   Not used
                                             // Bit[2]:   Strobe option
                                             // Bit[1]:   Manual average enable
                                             //     0: Average data manual set disable
                                             //     1: Average data manual set enable
                                             // Bit[0]:   Not used
// (0x3A2E~ 0x3A30)   reserved
/*****************************************************************************
 * OTP registers
 *****************************************************************************/
#define OV2715_OTP_DATA_0           (0x3D00) //RW - OTP Dump/Load Data 0
#define OV2715_OTP_DATA_1           (0x3D01) //RW - OTP Dump/Load Data 1
#define OV2715_OTP_DATA_2           (0x3D02) //RW - OTP Dump/Load Data 2
#define OV2715_OTP_DATA_3           (0x3D03) //RW - OTP Dump/Load Data 3
#define OV2715_OTP_DATA_4           (0x3D04) //RW - OTP Dump/Load Data 4
#define OV2715_OTP_DATA_5           (0x3D05) //RW - OTP Dump/Load Data 5
#define OV2715_OTP_DATA_6           (0x3D06) //RW - OTP Dump/Load Data 6
#define OV2715_OTP_DATA_7           (0x3D07) //RW - OTP Dump/Load Data 7
#define OV2715_OTP_DATA_8           (0x3D08) //RW - OTP Dump/Load Data 8
#define OV2715_OTP_DATA_9           (0x3D09) //RW - OTP Dump/Load Data 9
#define OV2715_OTP_DATA_A           (0x3D0A) //RW - OTP Dump/Load Data A
#define OV2715_OTP_DATA_B           (0x3d0b) //RW - OTP Dump/Load Data B
#define OV2715_OTP_DATA_C           (0x3d0c) //RW - OTP Dump/Load Data C
#define OV2715_OTP_DATA_D           (0x3D0D) //RW - OTP Dump/Load Data D
#define OV2715_OTP_DATA_E           (0x3D0E) //RW - OTP Dump/Load Data E
#define OV2715_OTP_DATA_F           (0x3D0F) //RW - OTP Dump/Load Data F
#define OV2715_OTP_CONTROL          (0x3D10) //RW - Bit[7:3]: Not used
                                             //     Bit[2]:   OTP load option
                                             //       0: Fast
                                             //       1: Slow
                                             //     Bit[1:0]: OTP mode
                                             //       00: OTP OFF
                                             //       01: Load/dump OTP
                                             //       10: Write/program OTP
                                             //       11: OTP OFF
/*****************************************************************************
 * BLC registers
 *****************************************************************************/
#define OV2715_BLC_CONTROL_00         (0x4000) //RW - Bit[7:4]: Not used
                                               //     Bit[3]:   AVG frame enable
                                               //       0: BLC is the calculating value
                                               //       1: BLC will be the average of the calculating value and the original BLC
                                               //     Bit[2]: Not used
                                               //     Bit[1]: Freeze enable
                                               //       0: BLC will be updated in some cases
                                               //       1: BLC will be keep original value
                                               //     Bit[0]: BLC enable
                                               //       0: Disable
                                               //       1: Enable
#define OV2715_BLC_CONTROL_01         (0x4001) //RW - Bit[7:5]: Not used
                                               //     Bit[4:0]: Start line
                                               //         Start statistic black line
                                               //         Range from 0 to 31
#define OV2715_BLC_CONTROL_02         (0x4002) //RW - Bit[7]: Not used
                                               //     Bit[6]: Manual offset enable
                                               //     Bit[5:0]: Not used
// (0x4003~ 0x401B)   reserved
// (0x401C)  reserved as of datasheet, but will be used, see undocumented registers
#define OV2715_BLC_FRAME_CONTROL      (0x401D) //RW - Bit[7:6]:   Not used
                                               //     Bit[5:4]:   BLC enable mode
                                               //         00: Always update
                                               //         10: Update when gain changes
                                               //         x1: No update
                                               //     Bit[3:2]:   Not used
                                               //     Bit[1]:   Format change enable
                                               //     Bit[0]:   Not used
/***************************************************************      **************
 * FC control registers
 ***************************************************************      **************/
#define OV2715_FRAME_CTRL00         (0x4201) //RW - Bit[7:4]:   Not used
                                             //           Bit[3:0]:   Frame ON number
                                             //           Control passed frame number When both ON and OFF numbers are set
                                             //           to 0x00, frame control is in bypass mode.

#define OV2715_FRAME_CTRL01         (0x4202) //RW - Bit[7:4]:   Not used
                                             //           Bit[3:0]:   Frame OFF number Control masked frame number
                                             //           When both ON and OFF numbers are set
                                             //            to 0x00, frame control is in bypass mode.
/*****************************************************************************
 * DVP registers
 *****************************************************************************/
#define OV2715_DVP_CTRL00           (0x4700) //RW -   Bit[7:4]: Not used
                                             //       Bit[3]: CCIR656 v select
                                             //       Bit[2]: CCIR656 f value
                                             //       Bit[1]: CCIR656 mode enable
                                             //       Bit[0]: HSYNC mode enable
// (0x4701~ 0x4703) reserved
#define OV2715_DVP_CTRL01           (0x4704) //RW -   Bit[7:4]: Not used
                                             //       Bit[3:2]: Debug only Changing these values is not recommended
                                             //       Bit[1:0]: VSYNC mode enable
                                             //         00: VSYNC1 mode enabled
                                             //         01: VSYNC2 mode enabled
                                             //         10: VSYNC3 mode enabled
                                             //         11: Not used
// (0x4705~ 0x4707) reserved
#define OV2715_DVP_CTRL02           (0x4708) //RW -   Bit[7]: CLK DDR mode enable
                                             //       Bit[6]: Not used
                                             //       Bit[5]: VSYNC gate CLK enable
                                             //       Bit[4]: HREF gate CLK enable
                                             //       Bit[3]: Not used
                                             //       Bit[2]: HREF polarity
                                             //       Bit[1]: VSYNC polarity
                                             //       Bit[0]: PCLK gate low enable

#define OV2715_DVP_CTRL03           (0x4709) //RW -   Bit[7]: Video FIFO bypass mode
                                             //       Bit[6:4]: Data bit swap
                                             //         000: Output data is the same order as input that is [9:0]
                                             //         001: Output data is reversed, [0:9]
                                             //         010: Output data is reordered as {[2:9], [1:0]}
                                             //         011: Output data is reordered as {[7:0], [9:8]}
                                             //         100: Output data is reordered as {[9:8], [0:7]}
                                             //         101: Output data is reordered as {[9], [0:8]}
                                             //         110: Output data is reordered as {[1:9], [0]}
                                             //         111: Output data is reordered as {[8:0], [9]}
                                             //       Bit[3]: Test mode
                                             //       Bit[2]: Test mode 10-bit
                                             //       Bit[1]: Test mode 8-bit
                                             //       Bit[0]: Test mode enable
/***************************************************************************
 * MIPI registers
 ***************************************************************************/
#define OV2715_MIPI_CTRL_00           (0x4800) //RW - Bit[7:6]: Not used
                                               //     Bit[5]: Clock lane gate enable
                                               //       0: Enable
                                               //       1: Disable
                                               //     Bit[4]: Line sync enable
                                               //       0: Do not send line short packet for each line
                                               //       1: Send line short packet for each line
                                               //     Bit[3]: Lane select
                                               //       0: Use lane1 as default data lane
                                               //       1: Use lane2 as default data lane
                                               //     Bit[2]: Idle status
                                               //       0: MIPI bus will be LP00 when no packet is transmitted
                                               //       1: MIPI bus will be LP11 when no packet is transmitted
                                               //     Bit[1:0]: Not used
#define OV2715_MIPI_CTRL_01           (0x4801) //RW - Bit[7]: Long packet data type manual enable
                                               //       0: Use MIPI data type 1: Use manual data type
                                               //     Bit[6]: Short packet data type manual enable
                                               //       0: Use auto value
                                               //       1: Use manual value as short
                                               //     Bit[5]: packet data Short packet WORD COUNTER manual enable
                                               //       0: Use frame counter or line counter
                                               //       1: Select manual value
                                               //     Bit[4]: PH bit order for ECC
                                               //       0: {DI[7:0],WC[7:0], WC[15:8]}
                                               //       1: {DI[0:7],WC[0:7], WC[8:15]}
                                               //     Bit[3]: PH byte order for ECC
                                               //       0: {DI,WC_l,WC_h}
                                               //       1: {DI,WC_h,WC_l}
                                               //     Bit[2]: PH byte order2 for ECC
                                               //       0: {DI,WC}
                                               //       1: {WC,DI}
                                               //     Bit[1]: MARK1 enable for data lane1
                                               //       0: Disable MARK1
                                               //       1: When resume, lane1 should send MARK1
                                               //     Bit[0]: MARK1 enable for data lane2
                                               //       0: Disable MARK1
                                               //       1: When resume, lane2 should send MARK1
// (0x4802) reserved
#define OV2715_MIPI_CTRL_03           (0x4803) //RW - Bit[7:4]: Not used
                                               //     Bit[3]: Enable LP CD when HS TX for lane1
                                               //       0:  Disable
                                               //       1:  Enable
                                               //     Bit[2]: Enable LP CD when HS TX for lane2
                                               //       0:  Disable
                                               //       1:  Enable
                                               //     Bit[1]: Enable LP CD when LP TX for lane2
                                               //       0:  Disable
                                               //       1:  Enable
                                               //     Bit[0]: Enable LP CD when LP TX for lane1
                                               //       0:  Disable
                                               //       1:  Enable

#define OV2715_MIPI_CTRL_04           (0x4804) //RW - Bit[7:5]: Not used
                                               //     BIt[4]: Enable MIPI LP RX to read/write registers
                                               //       0: Disable, RX LP data will write to VFIFO
                                               //       1: Enable
                                               //     Bit[3]: Address read/write register will auto add 1
                                               //       0: Disable
                                               //       1: Enable
                                               //     Bit[2]: LP TX lane select
                                               //       0: Select lane1 to transmit LP data
                                               //       1: Select lane2 to transmit LP data
                                               //     Bit[1:0]: Not used
#define OV2715_MIPI_CTRL_05           (0x4805) //RW - Bit[7]: MIPI lane2 disable
                                               //       0: Enable lane2
                                               //       1: Disable MIPI data lane1, lane1 will be LP00
                                               //     Bit[6]: MIPI lane1 disable
                                               //       0: Enable lane1
                                               //       1: Disable MIPI data lane1, lane1 will be LP00
                                               //     Bit[5]: LPX Global timing select
                                               //       0: Auto calculate T LPX in PCLK2X domain, unit CLK2X
                                               //       1: Use lp_p_min[7:0]
                                               //     Bit[4:3]: Not used
                                               //     Bit[2]: MIPI read/write registers disable
                                               //       0: Enable MIPI access SRB
                                               //       1: Disable MIPI access SRB
                                               //     Bit[1:0]: Not used
#define OV2715_MAX_FCNT_H             (0x4810) //RW - High Byte of Max Frame Counter of Frame Sync Short Packet
#define OV2715_MAX_FCNT_L             (0x4811) //RW - Low Byte of Max Frame Counter of Frame Sync Short Packet
#define OV2715_MIN_SPKT_WC_REG_H      (0x4812) //RW - High Byte of Manual Short Packet Word Counter
#define OV2715_MIN_SPKT_WC_REG_L      (0x4813) //RW - Low Byte of Manual Short Packet Word Counter
#define OV2715_MIPI_CTRL_14           (0x4814) //RW - Bit[7:6]: Virtual channel of MIPI packet
                                               // Bit[5:0]: Data type manual
#define OV2715_MIPI_SPKT_DT           (0x4815) //RW - Manual Data Type for Short Packet
#define OV2715_MIN_HS_ZERO_H          (0x4818) //RW - High Byte of Minimum Value of hs_zero, unit ns
#define OV2715_MIN_HS_ZERO_L          (0x4819) //RW - Low Byte of Minimum Value of hs_zero hs_zero_real = hs_zero_min_o + tui ?ui_hs_zero_min_o
#define OV2715_MIN_MIPI_HS_TRAIL_H    (0x481A) //RW - High Byte of Minimum Value of hs_trail, unit ns
#define OV2715_MIN_MIPI_HS_TRAIL_L    (0x481B) //RW - Low Byte of Minimum Value of hs_trail hs_trail_real = hs_trail_min_o + tui ?ui_hs_trail_min_o
#define OV2715_MIN_MIPI_CLK_ZERO_H    (0x481C) //RW - High Byte of Minimum Value of clk_zero, unit ns
#define OV2715_MIN_MIPI_CLK_ZERO_L    (0x481D) //RW - Low Byte of Minimum Value of clk_zero clk_zero_real = clk_zero_min_o + tui ?ui_clk_zero_min_o
#define OV2715_MIN_MIPI_CLK_PREPARE_H (0x481E) //RW - High Byte of Minimum Value of clk_prepare, unit ns
#define OV2715_MIN_MIPI_CLK_PREPARE_L (0x481F) //RW - Low Byte of Minimum Value of clk_prepare clk_prepare_real = clk_prepare_min_o + tui ?ui_clk_prepare_min_o
#define OV2715_MIN_CLK_POST_H         (0x4820) //RW - High Byte of Minimum Value of clk_post, unit ns
#define OV2715_MIN_CLK_POST_L         (0x4821) //RW - Low Byte of Minimum Value of clk_post clk_post_real = clk_post_min_o + tui ?ui_clk_post_min_o
#define OV2715_MIN_CLK_TRAIL_H        (0x4822) //RW - High Byte of Minimum Value of clk_trail, unit ns
#define OV2715_MIN_CLK_TRAIL_L        (0x4823) //RW - Low Byte of Minimum Value of clk_trail clk_trail_real = clk_trail_min_o + tui ?ui_clk_trail_min_o
#define OV2715_MIN_LPX_PCLK_H         (0x4824) //RW - High Byte of Minimum Value of lpx_p, unit ns
#define OV2715_MIN_LPX_PCLK_L         (0x4825) //RW - Low Byte of Minimum Value of lpx_p lpx_p_real = lpx_p_min_o + tui ?ui_lpx_p_min_o
#define OV2715_MIN_HS_PREPARE_H       (0x4826) //RW - High Byte of Minimum Value of hs_prepare, unit ns
#define OV2715_MIN_HS_PREPARE_L       (0x4827) //RW - Low Byte of Minimum Value of hs_prepare hs_prepare_real = hs_prepare_min_o + tui ?ui_hs_prepare_min_o
#define OV2715_MIN_HS_EXIT_H          (0x4828) //RW - High Byte of Minimum Value of hs_exit, unit ns
#define OV2715_MIN_HS_EXIT_L          (0x4829) //RW - Low Byte of Minimum Value of hs_exit hs_exit_real = hs_exit_min_o + tui ?ui_hs_exit_min_o
#define OV2715_MIN_HS_ZERO_UI         (0x482A) //RW - Minimum UI Value of hs_zero, unit UI
#define OV2715_MIN_HS_TRAIL_UI        (0x482B) //RW - Minimum UI Value of hs_trail, unit UI
#define OV2715_MIN_CLK_ZERO_UI        (0x482C) //RW - Minimum UI Value of clk_zero, unit UI
#define OV2715_MIN_CLK_PREPARE_UI     (0x482D) //RW - Minimum UI Value of clk_prepare, unit UI
#define OV2715_MIN_CLK_POST_UI        (0x482E) //RW - Minimum UI Value of clk_post, unit UI
#define OV2715_MIN_CLK_TRAIL_UI       (0x482F) //RW - Minimum UI Value of clk_trail, unit UI
#define OV2715_MIN_LPX_PCLK_UI        (0x4830) //RW - Minimum UI Value of lpx_p (pclk2x domain), unit UI
#define OV2715_MIN_HS_PREPARE_UI      (0x4831) //RW - Minimum UI Value of hs_prepare, unit UI
#define OV2715_MIN_HS_EXIT_UI         (0x4832) //RW - Minimum UI Value of hs_exit, unit UI
// (0x4865)   reserved
/***************************************************************************
 * ISP control registers
 ***************************************************************************/
#define OV2715_ISP_CONTROL0               (0x5000) //RW - Bit[7]: LENC enable
                                                   //       0: Disable
                                                   //       1: Enable
                                                   //     Bit[6:3]: Not used
                                                   //     Bit[2]: Black pixel cancellation enable
                                                   //       0: Disable
                                                   //       1: Enable
                                                   //     Bit[1]: White pixel cancellation enable
                                                   //       0: Disable
                                                   //       1: Enable
                                                   //     Bit[0]: Not used
#define OV2715_ISP_CONTROL1               (0x5001) //RW - Bit[7:1]: Not used
                                                   //     Bit[0]: AWB enable
                                                   //       0: Disable
                                                   //       1: Enable
#define OV2715_ISP_CONTROL2               (0x5002) //RW - Bit[7:3]: Not used
                                                   //     Bit[2]: VAP enable
                                                   //       0: Disable
                                                   //       1: Enable
                                                   //     Bit[1:0]: Not used
// (0x5003~ 0x5004) reserved
#define OV2715_ISP_CONTROL5               (0x5005) //RW - Bit[7:5]: Not used
                                                   //     Bit[4]: AWB bias ON
                                                   //     BIt[3:0]: Not used
//(0x5006~ 0x501E)  reserved
#define OV2715_ISP_CONTROL31              (0x501F) //RW - Bit[7:3]: Not used
                                                   //     Bit[2:0]: Format select
                                                   //       000: Not used
                                                   //       001: Not used
                                                   //       010: Not used
                                                   //       011: ISP RAW
                                                   //       100: INT CIF RAW
                                                   //       101: Not used
                                                   //       111: Not used
//(0x5020~ 0x503C)  reserved
#define OV2715_ISP_TEST                   (0x503D) //RW - Bit[7]: Color bar enable
                                                   //       0: Color bar OFF
                                                   //       1: Color bar enable
                                                   //     Bit[6]: Not used
                                                   //     Bit[5:4]: Color bar pattern select
                                                   //       10: Color bar pattern
                                                   //     Bit[3]: Not used
                                                   //     Bit[2]: Color bar rolling enable
                                                   //     Bit[1:0]: Not used
#define OV2715_ISP_SENSOR_BIAS_READOUT    (0x503E) //R  - ISP Sensor Bias Readout
#define OV2715_ISP_SENSOR_GAIN_READOUT    (0x503F) //R  - ISP Sensor Gain Readout
/***************************************************************************
 * AWB control registers
 ***************************************************************************/
#define OV2715_AWB_GAIN_PK_RED_GAIN0      (0x3400) //RW - Bit[7:4]: Not used
                                                   //     Bit[3:0]: AWB R GAIN[11:8]
#define OV2715_AWB_GAIN_PK_RED_GAIN1      (0x3401) //RW - Bit[7:0]: AWB R GAIN[7:0]
#define OV2715_AWB_GAIN_PK_GREEN_GAIN0    (0x3402) //RW - Bit[7:4]: Not used
                                                   //     Bit[3:0]: AWB G GAIN[11:8]
#define OV2715_AWB_GAIN_PK_GREEN_GAIN1    (0x3403) //RW - Bit[7:0]: AWB G GAIN[7:0]
#define OV2715_AWB_GAIN_PK_BLUE_GAIN0     (0x3404) //RW - Bit[7:4]: Not used
                                                   //     Bit[3:0]: AWB B GAIN[11:8]
#define OV2715_AWB_GAIN_PK_BLUE_GAIN1     (0x3405) //RW - Bit[7:0]: AWB B GAIN[7:0]
#define OV2715_AWB_GAIN_PK_AWB_MAN_CTRL   (0x3406) //RW - Bit[7:1]: Not used
                                                   //     Bit[0]: AWB manual control
                                                   //       0: AWB gain automatically updated
                                                   //       1: AWB gain manual enable
#define OV2715_AWB_CONTROL_00             (0x5180) //R/W- Bit[7]:   Not used
                                                   //     Bit[6]:   fast_awb
                                                   //     Bit[5]:   freeze_gain_en
                                                   //     Bit[4]:   freeze_sum_en
                                                   //     Bit[3]:   Not used
                                                   //     Bit[2]:   start_sel
                                                   //     Bit[1:0]:   Not used
#define OV2715_AWB_CONTROL_01             (0x5181) //R/W- Bit[7:0]: Delta
#define OV2715_AWB_CONTROL_02             (0x5182) //RW - Bit[7:0]: Stable range
#define OV2715_STABLE_RANGE_WIDE          (0x5183) //RW - Bit[7:0]: Stable range to determine whether it is in stable status when it is already in stable status
// (0x5184~ 0x518B) reserved
#define OV2715_RED_GAIN_LIMIT             (0x518C) //RW - Bit[7:4]: Red gain up limit
                                                   //     Maximum red gain is: red gain up limit *256 + 0xFF
                                                   //     Bit[3:0]: Red gain down limit
                                                   //     Minimum red gain is: red gain down limit *256 + 0
#define OV2715_GREEN_GAIN_LIMIT           (0x518D) //RW - Bit[7:4]:   Green gain up limit
                                                   // Maximum green gain is:
                                                   // green gain up limit*256 + 0xFF
                                                   // Bit[3:0]:   Green gain down limit
                                                   // Minimum green gain is:
                                                   // green gain down limit*256 + 0
#define OV2715_BLUE_GAIN_LIMIT            (0x518E) //RW - Bit[7:4]:   Blue gain up limit
                                                   // Maximum blue gain is:
                                                   // blue gain up limit*256
                                                   // Bit[3:0]:   Blue gain down limit
                                                   // Minimum blue gain is:
                                                   // blue gain down limit*256 + 0
#define OV2715_AWB_FRAME_COUNTER          (0x518F) //RW - Bit[7:4]: Not used
                                                   // Bit[3:0]: Number of frames to do AWB when AWB is in stable mode
// (0x5190~ 0x5195) reserved
#define OV2715_RED_BEFORE_GAIN_AVERAGE    (0x5196) //R  - Bit[7:0]: Before AWB gain's red data average
#define OV2715_GREEN_BEFORE_GAIN_AVERAGE  (0x5197) //R  - Bit[7:0]: Before AWB gain's green data average
#define OV2715_BLUE_BEFORE_GAIN_AVERAGE   (0x5198) //R  - Bit[7:0]: Before AWB gain's blue data average
#define OV2715_RED_AFTER_GAIN_AVERAGE0    (0x5199) //R  - Bit[7:0]: After AWB gain's red data average high byte
#define OV2715_RED_AFTER_GAIN_AVERAGE1    (0x519A) //R  - Bit[7:0]: After AWB gain's red data average low byte
#define OV2715_GREEN_AFTER_GAIN_AVERAGE0  (0x519B) //R  - Bit[7:0]: After AWB gain's green data average high byte
#define OV2715_GREEN_AFTER_GAIN_AVERAGE1  (0x519C) //R  - Bit[7:0]: After AWB gain's green data average low byte
#define OV2715_BLUE_AFTER_GAIN_AVERAGE0   (0x519D) //R  - Bit[7:0]: After AWB gain's blue data average high byte
#define OV2715_BLUE_AFTER_GAIN_AVERAGE1   (0x519E) //R  - Bit[7:0]: After AWB gain's blue average low byte
// (0x519F~ 0x51BD) reserved
/***************************************************************************
 *  AVG registers
 ***************************************************************************/
#define OV2715_AVG_START_POSITION_AT_HORIZONTAL0 (0x5680) //RW -  Bit[7:4]: Not used
                                                          // Bit[3:0]: AVG start position[11:8] at horizontal
#define OV2715_AVG_START_POSITION_AT_HORIZONTAL1 (0x5681) //RW -  Bit[7:0]: AVG start position[7:0] at horizontal
#define OV2715_AVG_END_POSITION_AT_HORIZONTAL0   (0x5682) //RW - Bit[7:4]: Not used
                                                          // Bit[3:0]: AVG end position[11:8] at horizontal
#define OV2715_AVG_END_POSITION_AT_HORIZONTAL1   (0x5683) //RW - Bit[7:0]: AVG end position[7:0] at horizontal
#define OV2715_AVG_START_POSITION_AT_VERTICAL0   (0x5684) //RW - Bit[7:3]: Not used
                                                          // Bit[2:0]: AVG start position[10:8] at vertical
#define OV2715_AVG_START_POSITION_AT_VERTICAL1   (0x5685) //RW - Bit[7:0]: AVG start position[7:0] at vertical
#define OV2715_AVG_END_POSITION_AT_VERTICAL0     (0x5686) //RW - Bit[7:3]: Not used
                                                          // Bit[2:0]: AVG end position[10:8] at vertical
#define OV2715_AVG_END_POSITION_AT_VERTICAL1     (0x5687) //RW - Bit[7:0]: AVG end position[7:0] at vertical
// (0x5688)  reserved as of datasheet, but will be used, see undocumented registers
// (0x5689~ 0x568F)  reserved
#define OV2715_AVG_R10                           (0x5690) //R  - Bit[7:0]: Average of raw image[9:2]
// (0x5691) reserved
/***************************************************************************
 *  DPC registers
 ***************************************************************************/
#define OV2715_DPC_CTRL00                   (0x5780) //RW - Bit[7]: Not used
                                                     //     Bit[6]: Keep vertical channel
                                                     //       0: Do not keep vertical line
                                                     //       1: Keep vertical line
                                                     //     Bit[5]: Enable same channel connected
                                                     //       0: Disable removing the same channel connected defected pixels
                                                     //       1: Enable removing the same channel connected defected pixels
                                                     //     Bit[4]: Enable different channel connected
                                                     //       0: Disable removing the different channel connected defected pixels
                                                     //       1: Enable removing the different channel connected defected pixels
                                                     //     Bit[3:2]: Matching index selection
                                                     //       00: No matching point check
                                                     //       01: Check the before and after pixels from the third to fourth pixels
                                                     //       10: Check the before and after pixels from the third to fifth pixels
                                                     //       11: Check the before and after pixels from the third to sixth pixels
                                                     //     Bit[1:0]: Not used
#define OV2715_WHITE_THRESHOLD_LIST0        (0x5781) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: White pixel threshold list 0
#define OV2715_WHITE_THRESHOLD_LIST1        (0x5782) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: White pixel threshold list 1
#define OV2715_WHITE_THRESHOLD_LIST2        (0x5783) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: White pixel threshold list 2
#define OV2715_WHITE_THRESHOLD_LIST3        (0x5784) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: White pixel threshold list 3
#define OV2715_BLACK_THRESHOLD_LIST0        (0x5785) //RW - Bit[7:0]: Black pixel threshold list 0
#define OV2715_BLACK_THRESHOLD_LIST1        (0x5786) //RW - Bit[7:0]: Black pixel threshold list 1
#define OV2715_BLACK_THRESHOLD_LIST2        (0x5787) //RW - Bit[7:0]:   Black pixel threshold list 2
#define OV2715_BLACK_THRESHOLD_LIST3        (0x5788) //RW - Bit[7:0]:   Black pixel threshold list 3
#define OV2715_GAIN_LIST1                   (0x5789) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: Gain list 1
#define OV2715_GAIN_LIST2                   (0x578A) //RW - Bit[7]: Not used
                                                     //     Bit[6:0]: Gain list 2
#define OV2715_DPC_CTRL01                   (0x578B) //RW - Bit[7:4]: Not used
                                                     //     Bit[3]: Mode 1 enable
                                                     //       0: Disable
                                                     //       1: Enable only remove cluster
                                                     //     Bit[2]: Mode 2 enable
                                                     //       0: Disable
                                                     //       1: Enable
                                                     //     Bit[1]: Not used
                                                     //     Bit[0]: Enable mode 3
                                                     //       0: Disable
                                                     //       1: Enable
#define OV2715_DPC_SATURATE                 (0x578C) //RW - Saturate Value Set for Cross Cluster
#define OV2715_PATTERN_THRESHOLD_LIST0A     (0x5790) //RW - Bit[7:2]:   Not used
                                                     //     Bit[1:0]:   Pattern threshold list 0[9:8]
#define OV2715_PATTERN_THRESHOLD_LIST0B     (0x5791) //RW - Bit[7:0]:   Pattern threshold list 0[7:0]
#define OV2715_PATTERN_THRESHOLD_LIST1A     (0x5792) //RW - Bit[7:2]:   Not used
                                                     //     Bit[1:0]:   Pattern threshold list 1[9:8]
#define OV2715_PATTERN_THRESHOLD_LIST1B     (0x5793) //RW - Bit[7:0]:   Pattern threshold list 1[7:0]
#define OV2715_PATTERN_THRESHOLD_LIST2A     (0x5794) //RW - Bit[7:2]:   Not used
                                                     //     Bit[1:0]:   Pattern threshold list 2[9:8]
#define OV2715_PATTERN_THRESHOLD_LIST2B     (0x5795) //RW - Bit[7:0]:   Pattern threshold list 2[7:0]
#define OV2715_PATTERN_THRESHOLD_LIST3A     (0x5796) //RW - Bit[7:2]:   Not used
                                                     //     Bit[1:0]:   Pattern threshold list 3[9:8]
#define OV2715_PATTERN_THRESHOLD_LIST3B     (0x5797) //RW - Bit[7:0]:   Pattern threshold list 3[7:0]
/***************************************************************************
 *  LENC regist
 ***************************************************************************/
#define OV2715_LENC_RED_X0A     (0x5800) //RW - Bit[7:0]: Red center horizontal position (x0) high bits
#define OV2715_LENC_RED_X0B     (0x5801) //RW - Bit[7:0]: Red center horizontal position (x0) low eight bits
#define OV2715_LENC_RED_Y0A     (0x5802) //RW - Bit[7:0]: Red center vertical position (y0) high bits
#define OV2715_LENC_RED_Y0B     (0x5803) //RW - Bit[7:0]: Red center vertical position (y0) low eight bits
#define OV2715_LENC_RED_A1      (0x5804) //RW - Bit[7:0]: Red parameter a1 a1 and a2 are used to generate the a parameter
#define OV2715_LENC_RED_A2      (0x5805) //RW - Bit[7:0]: Red parameter a2 a1 and a2 are used to generate the parameter a
#define OV2715_LENC_RED_B1      (0x5806) //RW - Bit[7]: Sign bit
                                         //       0: b1 is positive
                                         //       1: b1 is negative
                                         //     Bit[6:0]: Red parameter b1
                                         //       b1 and b2 are used to generate the parameter b
#define OV2715_LENC_RED_B2      (0x5807) //RW - Bit[3:0]: Red parameter b2 b1 and b2 are used to generate the parameter b
#define OV2715_LENC_GRN_X0A     (0x5808) //RW - Bit[7:0]: Green center horizontal position (x0) high bits
#define OV2715_LENC_GRN_X0B     (0x5809) //RW - Bit[7:0]: Green center horizontal position (x0) low eight bits
#define OV2715_LENC_GRN_Y0A     (0x580A) //RW - Bit[7:0]: Green center vertical position (y0) high bits
#define OV2715_LENC_GRN_Y0B     (0x580B) //RW - Bit[7:0]: Green center vertical position (y0) low eight bits
#define OV2715_LENC_GRN_A1      (0x580C) //RW - Bit[7:0]: Green parameter a1; a1 and a2 are used to generate the parameter a
#define OV2715_LENC_GRN_A2      (0x580D) //RW - Bit[7:0]: Green parameter a2; a1 and a2 are used to generate the parameter a
#define OV2715_LENC_GRN_B1      (0x580E) //RW -  Bit[7]: Sign bit
                                         //         0: b1 is positive
                                         //         1: b1 is negative
                                         //       Bit[6:0]: Green parameter b1
                                         //       b1 and b2 are used to generate the parameter b
#define OV2715_LENC_GRN_B2      (0x580F) //RW - Bit[3:0]: Green parameter b2
                                         //       b1 and b2 are used to generate the parameter b
#define OV2715_LENC_BLU_X0A     (0x5810) //RW - Bit[7:0]: Blue center horizontal position (x0) high bits
#define OV2715_LENC_BLU_X0B     (0x5811) //RW - Bit[7:0]: Blue center horizontal position (x0) low eight bits
#define OV2715_LENC_BLU_Y0A     (0x5812) //RW - Bit[7:0]: Blue center vertical position (y0) high bits
#define OV2715_LENC_BLU_Y0B     (0x5813) //RW - Bit[7:0]: Blue center vertical position (y0) low eight bits
#define OV2715_LENC_BLU_A1      (0x5814) //RW - Bit[7:0]: Blue parameter a1 a1 and a2 are used to generate the parameter a
#define OV2715_LENC_BLU_A2      (0x5815) //RW - Bit[7:0]: Blue parameter a2 a1 and a2 are used to generate the parameter a
#define OV2715_LENC_BLU_B1      (0x5816) //RW - Bit[7:0]: Sign bit
                                         //         0: b1 is positive
                                         //         1: b1 is negative
                                         //       Bit[6:0]: blue parameter b1
                                         //         b1 and b2 are used to generate the parameter b
#define OV2715_LENC_BLU_B2      (0x5817) //RW -   Bit[7:4]: Not used
                                         //       Bit[3:0]: Blue parameter b2
                                         //         b1 and b2 are used to generate the parameter b
#define OV2715_LENC_CTRL00      (0x5818) //RW - Bit[7:3]: Not used
                                         //       Bit[2]: Round enable
                                         //         0: Round disable, do not use random round bit
                                         //         1: Round enable, generate random round bit
                                         //       Bit[1]: Coefficient manual enable
                                         //         0: Disable coefficient manual mode
                                         //         1: Enable coefficient manual mode, use auto mode to calculate the coefficient
                                         //       Bit[0]: gain coefficient enable
                                         //         0: Use the gain = 128 to calculate the coefficient
                                         //         1: Use the sensor gain to calculate the coefficient
#define OV2715_LENC_COEF_TH     (0x5819) //RW - Bit[7:0]:   LENC coefficient threshol
#define OV2715_LENC_GAIN_THRE1  (0x581A) //RW - Bit[7:0]:   LENC gain low threshold (eshold t1)
#define OV2715_LENC_GAIN_THRE2  (0x581B) //RW - Bit[7:0]:   LENC gain high threshold reshold(t2)
#define OV2715_LENC_COEF_MAN    (0x581C) //RW - Bit[7:0]:   Coefficient manual input
/***************************************************************************
 *  AFC registers
 ***************************************************************************/
#define OV2715_AFC_CTRL00       (0x6000) //RW - Bit[7:5]: Not used
                                         //     Bit[4]: Edge filter enable
                                         //         0: afc_edge module will not update
                                         //         1: afc_edge module will update
                                         //       Bit[3]: Edge filter b select
                                         //         0: DGE selects b2
                                         //         1: Edge selects b1
                                         //       Bit[2:0]: Edge filter a select
                                         //         000: Edge selects a1
                                         //         001: Edge selects a2
                                         //         010: Edge selects a3
                                         //         100: Edge selects a4
                                         //         101: Edge selects a5
#define OV2715_AFC_CTRL01       (0x6001) //RW - Bit[7:0]: Edge window0 left coordinate
#define OV2715_AFC_CTRL02       (0x6002) //RW - Bit[7:0]: Edge window0 top coordinate
#define OV2715_AFC_CTRL03       (0x6003) //RW - Bit[7:0]: Edge window0 right coordinate
#define OV2715_AFC_CTRL04       (0x6004) //RW - Bit[7:0]: Edge window0 bottom coordinate This bottom must be larger than any other
#define OV2715_AFC_CTRL05       (0x6005) //RW - Bit[7:0]: Edge window1 left coordinate
#define OV2715_AFC_CTRL06       (0x6006) //RW - Bit[7:0]: Edge window1 top coordinate
#define OV2715_AFC_CTRL07       (0x6007) //RW - Bit[7:0]: Edge window1 right coordinate
#define OV2715_AFC_CTRL08       (0x6008) //RW - Bit[7:0]: Edge window1 bottom coordinate
#define OV2715_AFC_CTRL09       (0x6009) //RW - Bit[7:0]: Edge window2 left coordinate
#define OV2715_AFC_CTRL10       (0x600A) //RW - Bit[7:0]: Edge window2 top coordinate
#define OV2715_AFC_CTRL11       (0x600B) //RW - Bit[7:0]: Edge window2 right coordinate
#define OV2715_AFC_CTRL12       (0x600C) //RW - Bit[7:0]: Edge window2 bottom coordinate
#define OV2715_AFC_CTRL13       (0x600D) //RW - Bit[7:0]: Edge window3 left coordinate
#define OV2715_AFC_CTRL14       (0x600E) //RW - Bit[7:0]: Edge window3 top coordinate
#define OV2715_AFC_CTRL15       (0x600F) //RW - Bit[7:0]: Edge window3 right coordinate
#define OV2715_AFC_CTRL16       (0x6010) //RW - Bit[7:0]: Edge window3 bottom coordinate
#define OV2715_AFC_CTRL17       (0x6011) //RW - Bit[7:0]: Edge window4 left coordinate
#define OV2715_AFC_CTRL18       (0x6012) //RW - Bit[7:0]: Edge window4 top coordinate
#define OV2715_AFC_CTRL19       (0x6013) //RW - Bit[7:0]: Edge window4 right coordinate
#define OV2715_AFC_CTRL20       (0x6014) //RW - Bit[7:0]: Edge window4 bottom coordinate
//(0x6015~ 0x603C)  reserved

/***************************************************************************
 *  undocumented registers
 ***************************************************************************/
#define OV2715_302D (0x302D)
#define OV2715_3600 (0x3600)
#define OV2715_3601 (0x3601)
#define OV2715_3602 (0x3602)
#define OV2715_3603 (0x3603)
#define OV2715_3604 (0x3604)
#define OV2715_3605 (0x3605)
#define OV2715_3606 (0x3606)
#define OV2715_3620 (0x3620)
#define OV2715_3623 (0x3623)
#define OV2715_3630 (0x3630)
#define OV2715_3631 (0x3631)
#define OV2715_3A1A (0x3A1A)
#define OV2715_3702 (0x3702)
#define OV2715_3703 (0x3703)
#define OV2715_3704 (0x3704)
#define OV2715_3706 (0x3706)
#define OV2715_370B (0x370B)
#define OV2715_3710 (0x3710)
#define OV2715_3712 (0x3712)
#define OV2715_3713 (0x3713)
#define OV2715_3714 (0x3714)
#define OV2715_3811 (0x3811)
#define OV2715_381C (0x381C)
#define OV2715_381D (0x381D)
#define OV2715_381E (0x381E)
#define OV2715_381F (0x381F)
#define OV2715_3820 (0x3820)
#define OV2715_3821 (0x3821)
#define OV2715_401C (0x401C)
#define OV2715_4301 (0x4301)
#define OV2715_4303 (0x4303)
#define OV2715_5688 (0x5688)

/*****************************************************************************
 * Default values
 *****************************************************************************/

// Reset values as of datasheet OV2715_CSP3_DS_2.01_Dream Chip.pdf from
// 28.05.2010, altered and extended by settings from
// 1920x1080_raw_Dreamchip_101012.txt (got from FAE Europe Thordis).
//
// Make sure that these static settings are reflecting the capabilities defined
// in IsiGetCapsIss (further dynamic setup may alter these default settings but
// often does not if there is no choice available).

/*****************************************************************************
 * SC control registers
 *****************************************************************************/
#define OV2715_SYSTEM_CONTROL00_DEFAULT            (0x42) //streaming disabled, after reset the debug bits are set to this value, so we can safely write the same value
#define OV2715_PIDH_DEFAULT                        (0x27) //read only
#define OV2715_PIDL_DEFAULT                        (0x10) //read only
#define OV2715_MIPI_CTRL00_DEFAULT                 (0x1A) //default as of datasheet is 0x00, 0x18 after reset, 0x1A = enable DVP/disable and power down MIPI
#define OV2715_PLL_CTRL00_DEFAULT                  (0x88) //default as of datasheet is 0x8A
#define OV2715_PLL_CTRL01_DEFAULT                  (0x10) //default as of datasheet is 0x00, see ov2710_PLL_Dreamchip_...xls
#define OV2715_PLL_CTRL02_DEFAULT                  (0x28) //default as of datasheet is 0x0A, see ov2710_PLL_Dreamchip_...xls
#define OV2715_PLL_PREDIVIDER_DEFAULT              (0x00)
#define OV2715_PAD_OUTPUT_ENABLE00_DEFAULT         (0x00)
#define OV2715_PAD_OUTPUT_ENABLE01_DEFAULT         (0x7F) //default as of datasheet is 0x00
#define OV2715_PAD_OUTPUT_ENABLE02_DEFAULT         (0xFC) //default as of datasheet is 0x00
#define OV2715_PAD_OUTPUT_VALUE00_DEFAULT          (0x00)
#define OV2715_PAD_OUTPUT_VALUE01_DEFAULT          (0x00)
#define OV2715_PAD_OUTPUT_VALUE02_DEFAULT          (0x00)
#define OV2715_PAD_OUTPUT_SELECT00_DEFAULT         (0x00)
#define OV2715_PAD_OUTPUT_SELECT01_DEFAULT         (0x00)
#define OV2715_PAD_OUTPUT_SELECT02_DEFAULT         (0x00)
#define OV2715_CHIP_REVISION_DEFAULT               (0x01) //OK read only anyway, from the OV2710 I read 0x00
#define OV2715_PAD_OUTPUT_DRIVE_CAPABILITY_DEFAULT (0x02)
/*****************************************************************************
 * SCCB control registers
 *****************************************************************************/
#define OV2715_SCCB_ID_DEFAULT                  (0x6C)
#define OV2715_PLL_CLOCK_SELECT_DEFAULT         (0x03) //default as of datasheet is 0x01
#define OV2715_SCCB_PAD_CLOCK_DIVIDER_DEFAULT   (0x01)
/*****************************************************************************
 * group sharing registers
 *****************************************************************************/
#define OV2715_GROUP_ADDR0_DEFAULT            (0x40)
#define OV2715_GROUP_ADDR1_DEFAULT            (0x4A)
#define OV2715_GROUP_ADDR2_DEFAULT            (0x54)
#define OV2715_GROUP_ADDR3_DEFAULT            (0x5E)
#define OV2715_GROUP_ACCESS_DEFAULT           (0x10) //default as of datasheet is 0x00 but writing it during init triggers group
                                                     //access, 0x10 = group_hold_end (not sticky, reg returns to 0x00 by itself)
/*****************************************************************************
 * analog registers
 *****************************************************************************/
#define OV2715_ANA_ARRAY_01_DEFAULT           (0x04) //default as of datasheet is 0x10
#define OV2715_SENSOR_REG0D_DEFAULT           (0x07) //default as of datasheet is 0x0C
/*****************************************************************************
 * timing control registers
 *****************************************************************************/
#define OV2715_TIMING_CONTROL_HS_HIGHBYTE_DEFAULT     (0x01)
#define OV2715_TIMING_CONTROL_VH_LOWBYTE0_DEFAULT     (0xC4) //default as of datasheet is 0x08
#define OV2715_TIMING_CONTROL_VH_HIGHBYTE0_DEFAULT    (0x00)
#define OV2715_TIMING_CONTROL_VH_LOWBYTE1_DEFAULT     (0x0A)

#define OV2715_TIMING_CONTROL_HW_HIGHBYTE_DEFAULT     (0x07) //default as of datasheet is 0x08, 0x07 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_HW_LOWBYTE_DEFAULT      (0x80) //default as of datasheet is 0x00, 0x80 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_VH_HIGHBYTE1_DEFAULT    (0x04) //default as of datasheet is 0x06, 0x04 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_VH_LOWBYTE2_DEFAULT     (0x38) //default as of datasheet is 0x00, 0x38 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_DVP_HSIZE_DEFAULT       (0x07) //default as of datasheet is 0x08, 0x07 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_DVP_HSIZELOW_DEFAULT    (0x80) //default as of datasheet is 0x00, 0x80 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_DVP_VSIZEHIGH_DEFAULT   (0x04) //default as of datasheet is 0x06, 0x04 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_DVP_VSIZELOW_DEFAULT    (0x38) //default as of datasheet is 0x00, 0x38 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_HTS_HIGHBYTE_DEFAULT    (0x09)
#define OV2715_TIMING_CONTROL_HTS_LOWBYTE_DEFAULT     (0x74) //default as of datasheet is 0x48
#define OV2715_TIMING_CONTROL_VTS_HIGHBYTE_DEFAULT    (0x04) //default as of datasheet is 0x06, 0x04 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_VTS_LOWBYTE_DEFAULT     (0x50) //default as of datasheet is 0x18, 0x50 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL_HV_OFFSET_DEFAULT       (0x10) //default as of datasheet is 0xC2, 0x10 after reset, ref settings from Thordis rely on reset value
#define OV2715_TIMING_CONTROL18_DEFAULT               (0x80)
/*****************************************************************************
 * AEC/AGC registers
 *****************************************************************************/
#define OV2715_AEC_PK_EXPO0_DEFAULT       (0x00) //changes with illumination when camera AEC is on (register 0x3503)
#define OV2715_AEC_PK_EXPO1_DEFAULT       (0x00) //changes with illumination when camera AEC is on (register 0x3503)
#define OV2715_AEC_PK_EXPO2_DEFAULT       (0x20) //changes with illumination when camera AEC is on (register 0x3503)
#define OV2715_AEC_PK_MANUAL_DEFAULT      (0x1F) //default as of datasheet is 0x00
#define OV2715_AEC_AGC_ADJ0_DEFAULT       (0x00) //changes with illumination when camera AGC is on (register 0x3503)
#define OV2715_AEC_AGC_ADJ1_DEFAULT       (0x00) //changes with illumination when camera AGC is on (register 0x3503)
#define OV2715_AEC_PK_VTS0_DEFAULT        (0x00)
#define OV2715_AEC_PK_VTS1_DEFAULT        (0x00)
#define OV2715_AEC_CONTROL0_DEFAULT       (0x78) //default as of datasheet is 0x7C
#define OV2715_AEC_CONTROL1_DEFAULT       (0x04)
#define OV2715_AEC_MAX_EXPO_60A_DEFAULT   (0x02)
#define OV2715_AEC_MAX_EXPO_60B_DEFAULT   (0x28)
#define OV2715_AEC_MAX_EXPO_60C_DEFAULT   (0x00)
#define OV2715_AEC_B50_STEP0_DEFAULT      (0x14) //default as of datasheet is 0x11, 0x14 after reset, ref settings from Thordis rely on reset value, TODO: calculate correct settings dependent on frame
#define OV2715_AEC_B50_STEP1_DEFAULT      (0xC0) //default as of datasheet is 0x40, 0xC0 after reset, ref settings from Thordis rely on reset value,       rate, see formula for "Band Step" in datasheet
#define OV2715_AEC_B60_STEP0_DEFAULT      (0x11)
#define OV2715_AEC_B60_STEP1_DEFAULT      (0x40)
#define OV2715_AEC_CONTROLD_DEFAULT       (0x04)
#define OV2715_AEC_CONTROLE_DEFAULT       (0x03)
#define OV2715_AEC_CONTROLF_DEFAULT       (0x40) //default as of datasheet is 0x78
#define OV2715_AEC_CONTROL10_DEFAULT      (0x38) //default as of datasheet is 0x68
#define OV2715_AEC_CONTROL11_DEFAULT      (0x90) //default as of datasheet is 0xD0
#define OV2715_AEC_CONTROL12_DEFAULT      (0x00)
#define OV2715_AEC_CONTROL13_DEFAULT      (0x54) //default as of datasheet is 0x50
#define OV2715_AEC_MAX_EXPO_50A_DEFAULT   (0x01) //default as of datasheet is 0x02, 0x01 after reset, ref settings from Thordis rely on reset value
#define OV2715_AEC_MAX_EXPO_50B_DEFAULT   (0xF2) //default as of datasheet is 0x28, 0xF2 after reset, ref settings from Thordis rely on reset value
#define OV2715_AEC_MAX_EXPO_50C_DEFAULT   (0x00)
#define OV2715_AEC_CONTROL17_DEFAULT      (0x89)
#define OV2715_AEC_G_CEIL0_DEFAULT        (0x00) //default as of datasheet is 0x03
#define OV2715_AEC_G_CEIL1_DEFAULT        (0x7A) //default as of datasheet is 0xE0
#define OV2715_AEC_CONTROL1B_DEFAULT      (0x48) //default as of datasheet is 0x78
#define OV2715_AEC_LED_ADD_ROW0_DEFAULT   (0x06)
#define OV2715_AEC_LED_ADD_ROW1_DEFAULT   (0x18)
#define OV2715_AEC_CONTROL1E_DEFAULT      (0x30) //default as of datasheet is 0x68
#define OV2715_AEC_CONTROL1F_DEFAULT      (0x10) //default as of datasheet is 0x40
#define OV2715_AEC_CONTROL20_DEFAULT      (0x20)
/*****************************************************************************
 * OTP registers
 *****************************************************************************/
#define OV2715_OTP_DATA_0_DEFAULT       (0x00)
#define OV2715_OTP_DATA_1_DEFAULT       (0x00)
#define OV2715_OTP_DATA_2_DEFAULT       (0x00)
#define OV2715_OTP_DATA_3_DEFAULT       (0x00)
#define OV2715_OTP_DATA_4_DEFAULT       (0x00)
#define OV2715_OTP_DATA_5_DEFAULT       (0x00)
#define OV2715_OTP_DATA_6_DEFAULT       (0x00)
#define OV2715_OTP_DATA_7_DEFAULT       (0x00)
#define OV2715_OTP_DATA_8_DEFAULT       (0x00)
#define OV2715_OTP_DATA_9_DEFAULT       (0x00)
#define OV2715_OTP_DATA_A_DEFAULT       (0x00)
#define OV2715_OTP_DATA_B_DEFAULT       (0x00)
#define OV2715_OTP_DATA_C_DEFAULT       (0x00)
#define OV2715_OTP_DATA_D_DEFAULT       (0x00)
#define OV2715_OTP_DATA_E_DEFAULT       (0x00)
#define OV2715_OTP_DATA_F_DEFAULT       (0x00)
#define OV2715_OTP_CONTROL_DEFAULT      (0x00)
/*****************************************************************************
 * BLC registers
 *****************************************************************************/
#define OV2715_BLC_CONTROL_00_DEFAULT       (0x01)
#define OV2715_BLC_CONTROL_01_DEFAULT       (0x00)
#define OV2715_BLC_CONTROL_02_DEFAULT       (0x00)
#define OV2715_BLC_FRAME_CONTROL_DEFAULT    (0x22)
/*****************************************************************************
 * FC control registers
 *****************************************************************************/
#define OV2715_FRAME_CTRL00_DEFAULT       (0x00)
#define OV2715_FRAME_CTRL01_DEFAULT       (0x00)
/*****************************************************************************
 * DVP registers
 *****************************************************************************/
#define OV2715_DVP_CTRL00_DEFAULT         (0x04)
#define OV2715_DVP_CTRL01_DEFAULT         (0x00)
#define OV2715_DVP_CTRL02_DEFAULT         (0x01)
#define OV2715_DVP_CTRL03_DEFAULT         (0x00)
/*****************************************************************************
 * MIPI regist
*****************************************************************************/
#define OV2715_MIPI_CTRL_00_DEFAULT           (0x04)
#define OV2715_MIPI_CTRL_01_DEFAULT           (0x03) //default as of datasheet is 0x04, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_MIPI_CTRL_03_DEFAULT           (0x50) //default as of datasheet is 0x5F, 0x50 after reset, ref settings from Thordis rely on reset value
#define OV2715_MIPI_CTRL_04_DEFAULT           (0x8D) //default as of datasheet is 0x8C, 0x8D after reset, ref settings from Thordis rely on reset value
#define OV2715_MIPI_CTRL_05_DEFAULT           (0x10)
#define OV2715_MAX_FCNT_H_DEFAULT             (0xFF)
#define OV2715_MAX_FCNT_L_DEFAULT             (0xFF)
#define OV2715_MIN_SPKT_WC_REG_H_DEFAULT      (0x00)
#define OV2715_MIN_SPKT_WC_REG_L_DEFAULT      (0x00)
#define OV2715_MIPI_CTRL_14_DEFAULT           (0x2A)
#define OV2715_MIPI_SPKT_DT_DEFAULT           (0x00)
#define OV2715_MIN_HS_ZERO_H_DEFAULT          (0x00)
#define OV2715_MIN_HS_ZERO_L_DEFAULT          (0x96)
#define OV2715_MIN_MIPI_HS_TRAIL_H_DEFAULT    (0x00)
#define OV2715_MIN_MIPI_HS_TRAIL_L_DEFAULT    (0x3C)
#define OV2715_MIN_MIPI_CLK_ZERO_H_DEFAULT    (0x01)
#define OV2715_MIN_MIPI_CLK_ZERO_L_DEFAULT    (0x86)
#define OV2715_MIN_MIPI_CLK_PREPARE_H_DEFAULT (0x00)
#define OV2715_MIN_MIPI_CLK_PREPARE_L_DEFAULT (0x3C)
#define OV2715_MIN_CLK_POST_H_DEFAULT         (0x00)
#define OV2715_MIN_CLK_POST_L_DEFAULT         (0x56)
#define OV2715_MIN_CLK_TRAIL_H_DEFAULT        (0x00)
#define OV2715_MIN_CLK_TRAIL_L_DEFAULT        (0x3C)
#define OV2715_MIN_LPX_PCLK_H_DEFAULT         (0x00)
#define OV2715_MIN_LPX_PCLK_L_DEFAULT         (0x32)
#define OV2715_MIN_HS_PREPARE_H_DEFAULT       (0x00)
#define OV2715_MIN_HS_PREPARE_L_DEFAULT       (0x32)
#define OV2715_MIN_HS_EXIT_H_DEFAULT          (0x00)
#define OV2715_MIN_HS_EXIT_L_DEFAULT          (0x64)
#define OV2715_MIN_HS_ZERO_UI_DEFAULT         (0x05)
#define OV2715_MIN_HS_TRAIL_UI_DEFAULT        (0x04)
#define OV2715_MIN_CLK_ZERO_UI_DEFAULT        (0x00)
#define OV2715_MIN_CLK_PREPARE_UI_DEFAULT     (0x00)
#define OV2715_MIN_CLK_POST_UI_DEFAULT        (0x34)
#define OV2715_MIN_CLK_TRAIL_UI_DEFAULT       (0x00)
#define OV2715_MIN_LPX_PCLK_UI_DEFAULT        (0x00)
#define OV2715_MIN_HS_PREPARE_UI_DEFAULT      (0x04)
#define OV2715_MIN_HS_EXIT_UI_DEFAULT         (0x00)
/*****************************************************************************
 * ISP control registers
*****************************************************************************/
#define OV2715_ISP_CONTROL0_DEFAULT     (0x59)
#define OV2715_ISP_CONTROL1_DEFAULT     (0x4E) //default as of datasheet is 0x4F
#define OV2715_ISP_CONTROL2_DEFAULT     (0xE0)
#define OV2715_ISP_CONTROL5_DEFAULT     (0xDC)
#define OV2715_ISP_CONTROL31_DEFAULT    (0x03)
#define OV2715_ISP_TEST_DEFAULT         (0x00)
/*****************************************************************************
 * AWB control registers
*****************************************************************************/
#define OV2715_AWB_GAIN_PK_RED_GAIN0_DEFAULT    (0x04)
#define OV2715_AWB_GAIN_PK_RED_GAIN1_DEFAULT    (0x00)
#define OV2715_AWB_GAIN_PK_GREEN_GAIN0_DEFAULT  (0x04)
#define OV2715_AWB_GAIN_PK_GREEN_GAIN1_DEFAULT  (0x00)
#define OV2715_AWB_GAIN_PK_BLUE_GAIN0_DEFAULT   (0x04)
#define OV2715_AWB_GAIN_PK_BLUE_GAIN1_DEFAULT   (0x00)
#define OV2715_AWB_GAIN_PK_AWB_MAN_CTRL_DEFAULT (0x00)
#define OV2715_AWB_CONTROL_00_DEFAULT           (0x40)
#define OV2715_AWB_CONTROL_01_DEFAULT           (0x20) //default as of datasheet is 0x02
#define OV2715_AWB_CONTROL_02_DEFAULT           (0x04)
#define OV2715_STABLE_RANGE_WIDE_DEFAULT        (0x08)
#define OV2715_RED_GAIN_LIMIT_DEFAULT           (0xF0)
#define OV2715_GREEN_GAIN_LIMIT_DEFAULT         (0xF0)
#define OV2715_BLUE_GAIN_LIMIT_DEFAULT          (0xF0)
#define OV2715_AWB_FRAME_COUNTER_DEFAULT        (0x00) //default as of datasheet is 0x04
/*****************************************************************************
 * AVG registers
*****************************************************************************/
#define OV2715_AVG_START_POSITION_AT_HORIZONTAL0_DEFAULT (0x00)
#define OV2715_AVG_START_POSITION_AT_HORIZONTAL1_DEFAULT (0x00)
#define OV2715_AVG_END_POSITION_AT_HORIZONTAL0_DEFAULT   (0x00) //default as of datasheet is 0x08, 0x00 after reset, can't be altered
#define OV2715_AVG_END_POSITION_AT_HORIZONTAL1_DEFAULT   (0x00)
#define OV2715_AVG_START_POSITION_AT_VERTICAL0_DEFAULT   (0x07) //default as of datasheet is 0x00
#define OV2715_AVG_START_POSITION_AT_VERTICAL1_DEFAULT   (0xA0) //default as of datasheet is 0x80
#define OV2715_AVG_END_POSITION_AT_VERTICAL0_DEFAULT     (0x04) //default as of datasheet is 0x00
#define OV2715_AVG_END_POSITION_AT_VERTICAL1_DEFAULT     (0x43) //default as of datasheet is 0x38
/*****************************************************************************
 * DPC registers
*****************************************************************************/
#define OV2715_DPC_CTRL00_DEFAULT               (0x7F)
#define OV2715_WHITE_THRESHOLD_LIST0_DEFAULT    (0x20)
#define OV2715_WHITE_THRESHOLD_LIST1_DEFAULT    (0x18)
#define OV2715_WHITE_THRESHOLD_LIST2_DEFAULT    (0x08)
#define OV2715_WHITE_THRESHOLD_LIST3_DEFAULT    (0x04)
#define OV2715_BLACK_THRESHOLD_LIST0_DEFAULT    (0x40)
#define OV2715_BLACK_THRESHOLD_LIST1_DEFAULT    (0x18)
#define OV2715_BLACK_THRESHOLD_LIST2_DEFAULT    (0x08)
#define OV2715_BLACK_THRESHOLD_LIST3_DEFAULT    (0x04)
#define OV2715_GAIN_LIST1_DEFAULT               (0x08)
#define OV2715_GAIN_LIST2_DEFAULT               (0x20)
#define OV2715_DPC_CTRL01_DEFAULT               (0x07)
#define OV2715_DPC_SATURATE_DEFAULT             (0x00)
#define OV2715_PATTERN_THRESHOLD_LIST0A_DEFAULT (0x00)
#define OV2715_PATTERN_THRESHOLD_LIST0B_DEFAULT (0x08)
#define OV2715_PATTERN_THRESHOLD_LIST1A_DEFAULT (0x00)
#define OV2715_PATTERN_THRESHOLD_LIST1B_DEFAULT (0x18)
#define OV2715_PATTERN_THRESHOLD_LIST2A_DEFAULT (0x00)
#define OV2715_PATTERN_THRESHOLD_LIST2B_DEFAULT (0x80)
#define OV2715_PATTERN_THRESHOLD_LIST3A_DEFAULT (0x01)
#define OV2715_PATTERN_THRESHOLD_LIST3B_DEFAULT (0x00)
/*****************************************************************************
 * LENC registers
*****************************************************************************/
#define OV2715_LENC_RED_X0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_RED_X0B_DEFAULT     (0x0C)
#define OV2715_LENC_RED_Y0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_RED_Y0B_DEFAULT     (0x06)
#define OV2715_LENC_RED_A1_DEFAULT      (0x22)
#define OV2715_LENC_RED_A2_DEFAULT      (0x07)
#define OV2715_LENC_RED_B1_DEFAULT      (0xC2)
#define OV2715_LENC_RED_B2_DEFAULT      (0x08)
#define OV2715_LENC_GRN_X0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_GRN_X0B_DEFAULT     (0x0C)
#define OV2715_LENC_GRN_Y0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_GRN_Y0B_DEFAULT     (0x06)
#define OV2715_LENC_GRN_A1_DEFAULT      (0x22)
#define OV2715_LENC_GRN_A2_DEFAULT      (0x07)
#define OV2715_LENC_GRN_B1_DEFAULT      (0xC2)
#define OV2715_LENC_GRN_B2_DEFAULT      (0x08)
#define OV2715_LENC_BLU_X0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_BLU_X0B_DEFAULT     (0x0C)
#define OV2715_LENC_BLU_Y0A_DEFAULT     (0x03) //default as of datasheet is 0x00, 0x03 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_BLU_Y0B_DEFAULT     (0x06)
#define OV2715_LENC_BLU_A1_DEFAULT      (0x22)
#define OV2715_LENC_BLU_A2_DEFAULT      (0x07)
#define OV2715_LENC_BLU_B1_DEFAULT      (0xC2)
#define OV2715_LENC_BLU_B2_DEFAULT      (0x08)
#define OV2715_LENC_CTRL00_DEFAULT      (0x04)
#define OV2715_LENC_COEF_TH_DEFAULT     (0x80)
#define OV2715_LENC_GAIN_THRE1_DEFAULT  (0x06) //default as of datasheet is 0x00, 0x06 after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_GAIN_THRE2_DEFAULT  (0x0C) //default as of datasheet is 0x00, 0x0C after reset, ref settings from Thordis rely on reset value
#define OV2715_LENC_COEF_MAN_DEFAULT    (0x80)
/*****************************************************************************
 * AFC registers
*****************************************************************************/
#define OV2715_AFC_CTRL00_DEFAULT       (0x1F)
#define OV2715_AFC_CTRL01_DEFAULT       (0x01)
#define OV2715_AFC_CTRL02_DEFAULT       (0x00)
#define OV2715_AFC_CTRL03_DEFAULT       (0x76)
#define OV2715_AFC_CTRL04_DEFAULT       (0x42)
#define OV2715_AFC_CTRL05_DEFAULT       (0x01)
#define OV2715_AFC_CTRL06_DEFAULT       (0x00)
#define OV2715_AFC_CTRL07_DEFAULT       (0x76)
#define OV2715_AFC_CTRL08_DEFAULT       (0x42)
#define OV2715_AFC_CTRL09_DEFAULT       (0x01)
#define OV2715_AFC_CTRL10_DEFAULT       (0x00)
#define OV2715_AFC_CTRL11_DEFAULT       (0x76)
#define OV2715_AFC_CTRL12_DEFAULT       (0x42)
#define OV2715_AFC_CTRL13_DEFAULT       (0x01)
#define OV2715_AFC_CTRL14_DEFAULT       (0x00)
#define OV2715_AFC_CTRL15_DEFAULT       (0x76)
#define OV2715_AFC_CTRL16_DEFAULT       (0x42)
#define OV2715_AFC_CTRL17_DEFAULT       (0x01)
#define OV2715_AFC_CTRL18_DEFAULT       (0x00)
#define OV2715_AFC_CTRL19_DEFAULT       (0x76)
#define OV2715_AFC_CTRL20_DEFAULT       (0x42)

/***************************************************************************
 *  undocumented registers
 *  defaults taken from Thordis 1920x1080_raw_Dreamchip_101012.txt
 ***************************************************************************/
#define OV2715_302D_DEFAULT (0x90)
#define OV2715_3600_DEFAULT (0x04)
#define OV2715_3601_DEFAULT (0x04)
#define OV2715_3602_DEFAULT (0x04)
#define OV2715_3603_DEFAULT (0xa7)
#define OV2715_3604_DEFAULT (0x60)
#define OV2715_3605_DEFAULT (0x05)
#define OV2715_3606_DEFAULT (0x12)
#define OV2715_3620_DEFAULT (0x07)
#define OV2715_3623_DEFAULT (0x40)
//#define OV2715_3630_DEFAULT (0x6d)
//#define OV2715_3631_DEFAULT (0x26)
#define OV2715_3630_DEFAULT (0x6B)
#define OV2715_3631_DEFAULT (0x24)
//#define OV2715_3630_DEFAULT (0x6B)
//#define OV2715_3631_DEFAULT (0x20)
#define OV2715_3702_DEFAULT (0x9e)
#define OV2715_3703_DEFAULT (0x74)
#define OV2715_3704_DEFAULT (0x10)
#define OV2715_3706_DEFAULT (0x61)
#define OV2715_370B_DEFAULT (0x40)
#define OV2715_3710_DEFAULT (0x9e)
#define OV2715_3712_DEFAULT (0x0c)
#define OV2715_3713_DEFAULT (0x8b)
#define OV2715_3714_DEFAULT (0x74)
#define OV2715_3811_DEFAULT (0x06) //read from sensor directly after softreset
#define OV2715_381C_DEFAULT (0x21) //read from sensor directly after softreset
#define OV2715_381D_DEFAULT (0x50) //read from sensor directly after softreset
#define OV2715_381E_DEFAULT (0x01) //read from sensor directly after softreset
#define OV2715_381F_DEFAULT (0x20) //read from sensor directly after softreset
#define OV2715_3820_DEFAULT (0x00) //read from sensor directly after softreset
#define OV2715_3821_DEFAULT (0x00) //read from sensor directly after softreset
#define OV2715_401C_DEFAULT (0x08) //read from sensor directly after softreset
#define OV2715_3A1A_DEFAULT (0x06)
#define OV2715_4301_DEFAULT (0xff)
#define OV2715_4303_DEFAULT (0x00)
#define OV2715_5688_DEFAULT (0x03)



typedef struct OV2715_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    //// modify below here ////

    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that csensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    bool_t              isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    float               VtPixClkFreq;           /**< pixel clock */
    uint16_t            LineLengthPck;          /**< line length with blanking */
    uint16_t            FrameLengthLines;       /**< frame line length */

    float               AecMinGain;
    float               AecMaxGain;
    float               AecMinIntegrationTime;
    float               AecMaxIntegrationTime;

    float               AecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    float               AecCurGain;
    float               AecCurIntegrationTime;

    bool                GroupHold;
    uint8_t             OldGain;
    uint32_t            OldIntegrationTime;
	uint32_t			preview_minimum_framerate;
} OV2715_Context_t;



#ifdef __cplusplus
}
#endif

/* @} ov2715_priv */

#endif /* __OV2715_PRIV_H__ */

