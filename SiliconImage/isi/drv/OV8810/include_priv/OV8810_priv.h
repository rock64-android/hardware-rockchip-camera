/*****************************************************************************/
/*!
 *  \file        OV8810_priv.h \n
 *  \version     1.0 \n
 *  \author      Meinicke \n
 *  \brief       Private header file for sensor specific code of the OV8810. \n
 *
 *  \revision    $Revision: 432 $ \n
 *               $Author: neugebaa $ \n
 *               $Date: 2009-06-30 11:48:59 +0200 (Di, 30 Jun 2009) $ \n
 *               $Id: OV8810_priv.h 432 2009-06-30 09:48:59Z neugebaa $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/

/*Modify by Fei*/
/*
#ifndef _OV8810_PRIV_H
#define _OV8810_PRIV_H

#include "isi_priv.h"

#if( OV8810_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#ifndef __OV8810_PRIV_H__
#define __OV8810_PRIV_H__

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
 * System control registers
 *****************************************************************************/
#define OV8810_AGCL                 (0x3000) // rw - Sensor Gain, or Sensor Long Exposure Gain, when used in HDR mode auto or manual
#define OV8810_AECL_2               (0x3002) // rw - Coarse Exposure Time, Auto or Manual, Units: Lines, Byte: High
#define OV8810_AECL_1               (0x3003) // rw - Coarse Exposure Time, Auto or Manual, Units: Lines, Byte: Low
#define OV8810_LAEC_2               (0x3004) // rw - Fine Exposure Time, Auto or Manual, Units: System clocks, Byte: High
#define OV8810_LAEC_1               (0x3005) // rw - Fine Exposure Time, Auto or Manual, Units: System clocks, Byte: Low
#define OV8810_AGC					(0x3006) // rw - Bit[0] Digital Gain Stage x2 on/off
// (0x3007~0X3009) reserved
#define OV8810_PIDH                 (0x300A) // rw - ID, Byte: High
#define OV8810_PIDL                 (0x300B) // rw - ID, Byte: Low
#define OV8810_SCCB_ID              (0x300C) // rw - SCCB ID
#define OV8810_R_PLL				(0x300D) // rw - PLL Analog control
											 //	     Bit[7:3]: Debug mode
											 //       Bit[2:0]: PLL change pump current control
#define OV8810_R_PLL1               (0x300E) // rw - Bit[7:6]: PLL pre divider, 00:1, 01:1.5, 10:2, 11:3
                                             //      Bit[5:0]: PLL divider, PLLDiv=64-R_PLL1[5:0]
#define OV8810_R_PLL2               (0x300F) // rw - Bit[7:4]: System clock divider, divider=R_PLL2[7:4]+1
                                             //      Bit[3:0]: MIPI divider,         divider=R_PLL2[3:0]+1
#define OV8810_R_PLL3               (0x3010) // rw - Bit[7]:   Bypass PLL
                                             //      Bit[6:4]: PLL charge pump current control
                                             //      Bit[3]:   Reserved
                                             //      Bit[2]:   LaneDiv control, LaneDiv= 0:1, 1:2
                                             //      Bit[1:0]: DIV4/5 control, Ratio= 00:1, 01:1, 10:4, 11:5
#define OV8810_R_PLL4               (0x3011) // rw - Bit[7:6]: Digital clock input divider, 00:1, 01:2, 10:4, 11:4
                                             //      Bit[5:0]: Reserved
#define OV8810_SYS                  (0x3012) // rw - System Control
                                             //      Bit[7]:   Software system reset
                                             //      Bit[6:0]: Reserved
#define OV8810_AUTO1                (0x3013) // rw - AEC Enable
                                             //      Bit[7]: AEC speed select, 0:Standard, 1:Faste
                                             //	     Bit[6]: AEC big steep enable, 0:Diaable, 1:Enable
                                             //      Bit[5]: Banding filter enable, 0:ON, 1:OFF
                                             //      Bit[4]: Auto banding enable(turn off banding automatically when it is too bright), 0:OFF, 1:ON
                                             //      Bit[3]: LAEC enable, 0:ON, 1:OFF
                                             //      Bit[2]: AGC enable, 0:Manual, 1:Auto
                                             //      Bit[0]: AEC enable, 0:Manual, 1:Auto
#define OV8810_AUTO2                (0x3014) // rw - Bit[7]:  Manual 50/60Hz selection, 0:60Hz, 1:50Hz
                                             //      Bit[6]:   50/60Hz auto detection enable, 0:OFF, 1:ON
                                             //      Bit[5:4]: Debug mode
                                             //      Bit[3]: VAEC enable, 0:OFF, 1:ON
                                             //      Bit[2]: Debug mode
                                             //      Bit[1]: Manual LAEC enable, 0:auto mode, 1:maual mode
                                             //      Bit[0]: Debug mode
#define OV8810_AUTO3                (0x3015) // rw -Bit[7]:   Debug mode
                                             //      Bit[6:4]: VAEC ceiling, number of frames: 000:1, 001:1.5, 010:2, 011:3, 100:4, 101:6, 110:8, 111:12
                                             //      Bit[3]:   Debug mode
                                             //      Bit[2:0]: Gain celling, 001:4x, 010:8x, 011:16x, 100:32x, 101-111:Debug mode
#define OV8810_0x3016				(0x3016)
// (0X3017)  Debug mode
#define OV8810_WPT                  (0x3018) // rw - Luminance Signal/High Range for AEC/AGC operation.
                                             //      AEC/AGC value decreases in auto mode, when average luminance is greater than WPT[7:0].
#define OV8810_BPT                  (0x3019) // rw - Luminance Signal/ Low Range for AEC/AGC operation.
                                             //      AEC/AGC value increases in auto mode when average luminance is less than BPT[7:0].
#define OV8810_VPT                  (0x301A) // rw - Fast Mode Large Step Range Thresholds, effective only in AEC/AGC fast mode
                                             //      Bit[7:4]: High threshold
                                             //      Bit[3:0]: Low threshold
                                             //      AEC/AGC may change in larger steps, when luminance average is greater than (VPT[7:4],4b0 ??)
                                             //                                                                    or less than (VPT[3:0], 4b0 ??)
#define OV8810_YAVG                 (0x301B) // ro - Luminance Average - this register will auto update.
                                             //      Average luminance is calculated from the B/Gb/Gr/R channel average as follows:
                                             //      B/Gb/Gr/R channel average = (BAVG[7:0] + GbAVG[7:0] + GrAVG[7:0] + RAVG[7:0]) x 0.25
#define OV8810_AECG_MAX50           (0x301C) // rw - 50 Hz Smooth Banding Maximum Steps Control
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5:0]: maximum band for 50 Hz in terms of row exposure
#define OV8810_AECG_MAX60           (0x301D) // rw - 60 Hz Smooth Banding Maximum Steps Control
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5:0]: maximum band for 60 Hz in terms of row exposure
#define OV8810_ADDVS_2              (0x301E) // rw - Extra VSYNC Pulse Width, Units: Lines, Byte: High
#define OV8810_ADDVS_1              (0x301F) // rw - Extra VSYNC Pulse Width, Units: Lines, Byte: Low
#define OV8810_FRAME_LENGTH_LINES2  (0x3020) // rw - Frame Length, Units: Lines, Byte: High
#define OV8810_FRAME_LENGTH_LINES1  (0x3021) // rw - Frame Length, Units: Lines, Byte: Low
#define OV8810_LINE_LENGTH_PCK2     (0x3022) // rw - Line Length, Units: System clocks, Byte: High
#define OV8810_LINE_LENGTH_PCK1     (0x3023) // rw - Line Length, Units: System clocks, Byte: Low
#define OV8810_X_ADDR_START_2       (0x3024) // rw - X Address of the Top Left Corner of the Visible Pixels, Units: Pixels, Byte: High
#define OV8810_X_ADDR_START_1       (0x3025) // rw - X Address of the Top Left Corner of the Visible Pixels, Units: Pixels, Byte: Low
#define OV8810_Y_ADDR_START_2       (0x3026) // rw - Y Address of the Top Left Corner of the Visible Pixels, Units: Lines, Byte: High
#define OV8810_Y_ADDR_START_1       (0x3027) // rw - Y Address of the Top Left Corner of the Visible Pixels, Units: Lines, Byte: Low
#define OV8810_X_ADDR_END_2         (0x3028) // rw - X Address of the Bottom Right Corner of the Visible Pixels, Units: Pixels, Byte: High
#define OV8810_X_ADDR_END_1         (0x3029) // rw - X Address of the Bottom Right Corner of the Visible Pixels, Units: Pixels, Byte: Low
#define OV8810_Y_ADDR_END_2         (0x302A) // rw - Y Address of the Bottom Right Corner of the Visible Pixels, Units: Lines, Byte: High
#define OV8810_Y_ADDR_END_1         (0x302B) // rw - Y Address of the Bottom Right Corner of the Visible Pixels, Units: Lines, Byte: Low
#define OV8810_X_OUTPUTSIZE_2       (0x302C) // rw - Width of Image Data Output from the Sensor, Units: Pixels, Byte: High
#define OV8810_X_OUTPUTSIZE_1       (0x302D) // rw - Width of Image Data Output from the Sensor, Units: Pixels, Byte: Low
#define OV8810_Y_OUTPUTSIZE_2       (0x302E) // rw - Height of Image Data Output from the Sensor, Units: Pixels, Byte: High
#define OV8810_Y_OUTPUTSIZE_1       (0x302F) // rw - Height of Image Data Output from the Sensor, Units: Pixels, Byte: Low
// (0x3030 - 0x303B) reserved
#define OV8810_DATR_D56             (0x303C) // ro - Sigma5060 Register
#define OV8810_DATR_OTP             (0x303E) // ro - OTP read/write control, 0x55: Read from OTP memory, 0xAA: Write to OTP memory
// (0x303F - 0x304B) reserved
#define OV8810_R_SIGMA            	(0x304C) // rw - R_sigma[39:32]
#define OV8810_0x3058				(0x3058)
#define OV8810_0x3059				(0x3059)
#define OV8810_BD50ST_2             (0x305C) // rw - Band Step for 50 Hz, Byte: High
#define OV8810_BD50ST_1             (0x305D) // rw - Band Step for 50 Hz, Byte: Low
#define OV8810_BD60ST_2             (0x305E) // rw - Band Step for 60 Hz, Byte: High
#define OV8810_BD60ST_1             (0x305F) // rw - Band Step for 60 Hz, Byte: Low
#define OV8810_0x3065				(0x3065)
#define OV8810_0x3067				(0x3067)
#define OV8810_0x3068				(0x3068)
#define OV8810_0x3069				(0x3069)
#define OV8810_0x306A				(0x306A)
#define OV8810_0x306B				(0x306B)
#define OV8810_TMC1          		(0x3071) // reserved but written by Omnivision evaluation kit
#define OV8810_0x3072				(0x3072)
// (0x3073 - 0x3074) reserved
#define OV8810_TMC5          (0x3075) // reserved but written by Omnivision evaluation kit
#define OV8810_TMC6          (0x3076) // reserved but written by Omnivision evaluation kit
#define OV8810_TMC7          (0x3077) // reserved but written by Omnivision evaluation kit
#define OV8810_TMC8          (0x3078) // reserved but written by Omnivision evaluation kit
#define OV8810_TMC9          (0x3079) // reserved but written by Omnivision evaluation kit
#define OV8810_TMCE          (0x307E) // reserved but written by Omnivision evaluation kit
#define OV8810_0x3080		 (0x3080)
#define OV8810_0x3082		 (0x3082)
#define OV8810_0x3084		 (0x3084)
#define OV8810_0x3087		 (0x3087)
#define OV8810_0x308A		 (0x308A)
#define OV8810_0x308D		 (0x308D)
//0x3086 Debug mode
#define OV8810_0x3090				(0x3090)
#define OV8810_R_ARRAY          	(0x3091) // reserved but written by Omnivision evaluation kit
#define OV8810_0x3092				(0x3092)
#define OV8810_0x3094				(0x3094)
#define OV8810_0x3095				(0x3095)
#define OV8810_0x3098				(0x3098)
#define OV8810_0x3099				(0x3099)
#define OV8810_0x309A				(0x309A)
#define OV8810_0x309B				(0x309B)
#define OV8810_0x309C				(0x309C)
#define OV8810_0x309D				(0x309D)
#define OV8810_0x309E				(0x309E)
#define OV8810_0x309F				(0x309F)
#define OV8810_0x30A0				(0x30A0)
#define OV8810_SLEW					(0x30A5)
#define OV8810_R_PWC				(0x30A9)
#define OV8810_0x30AA				(0x30AA)
#define OV8810_0x30AB				(0x30AB)
#define OV8810_IO_CTRL0             (0x30B0) // rw - Enable of Second Camera Interface
                                             //      CY[7:0]
#define OV8810_IO_CTRL1             (0x30B1) // rw - Enable of Second Camera Interface
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5]:   C_VSYNC
                                             //      Bit[4]:   C_STROBE
                                             //      Bit[3]:   C_PCLK
                                             //      Bit[2]:   C_HREF
                                             //      Bit[1:0]: CY[9:8]
#define OV8810_IO_CTRL2             (0x30B2) // rw - Enable of Second Camera Interface
                                             //      Bit[7:5]: Reserved
                                             //      Bit[4]:   C_FREX
                                             //      Bit[3:0]: R_PAD[3:0]
#define OV8810_DSIO_3               (0x30B3) // rw - DSIO[7:0]
                                             //      Bit[7]: Reserved
                                             //      Bit[6]:   Rpclkinv, Invert OP_PCLK
                                             //      Bit[5:4]: Rpclksw, Switch PCLK and OP_PCLK
                                             //      Bit[3]:   RPCKman, 0:Division from ISP subsample, 1:Manual control use RPCLKdiv
                                             //      Bit[2]:   Reserved
                                             //      Bit[1:0]: RPCLKdiv, PCLK divisor, 00:1, 01:2, 10:4, 11:8
#define OV8810_DSIO_4               (0x30B4) // rw - DSIO[15:8]
                                             //      Bit[7:6]: Rtest, 00:IMG to PAD, 01:Mix signals to PAD, 10:Array address and timing to PAD, 11:Array timing to PAD
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3]:   RdspblueLat
                                             //      Bit[2]:   OTP_TST
                                             //      Bit[1:0]: OTP_SCK_OPT
//0x30B5~0x30B6 DEBUG MODE
#define OV8810_FRS_30B7            	(0x30B7) // rw - Bit[7]:   dis_ISP_rw, Disable RW of registers from ISP/MIPI
                                             //      Bit[6:4]: Reserved
                                             //      Bit[3]:   Rgrp_wr_en, Enable group write
                                             //      Bit[2]:   grp_i2c2uc_wr
                                             //      Bit[1]:   Reserved
                                             //      Bit[0]:   Ri2c_isp_wrB
#define OV8810_0x30BE            	(0x30BE)
#define OV8810_0x30BF            	(0x30BF)
#define OV8810_0x30E3				(0x30E3)
#define OV8810_FRS_30E4            	(0x30E4) // rw - FECNT
#define OV8810_FRS_30E5          	(0x30E5) // rw - FFCNT[7:0]
#define OV8810_FRS_30E6          	(0x30E6) // rw - FFCNT[15:0]
#define OV8810_FRS_30E7            	(0x30E7) // rw - Bit[7]:   RSTR_FREX_Sync
                                             //      Bit[6]:   RSTR_FREX
                                             //      Bit[5]:   RSTR_EDGE
                                             //      Bit[4]:   RSTR_INV
                                             //      Bit[3:1]: Reserved
                                             //      Bit[0]:   STR_EN
#define OV8810_FRS_30E8            	(0x30E8) // rw - Bit[7]:   Strobe function enable, 0:Strobe disable, 1:Start strobe enable
                                             //      Bit[6]:   Strobe output polarity control, 0:Positive pulse, 1:Negative pulse
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3:2]: Xenon mode pulse width, 00:1 line, 01:2 lines, 10:3 lines, 11:4 lines
                                             //      Bit[1:0]: Strobe mode select, 00:Xenon mode, 01:LED 1 and 2 mode, 10:LED 1 and 2 mode, 11:LED 3 mode
#define OV8810_0x30E9            	(0x30E9)
#define OV8810_FRS_30EA             (0x30EA) // rw - Bit[7]:   Reserved
                                             //      Bit[6]:   Rjump_man
                                             //      Bit[5]:   Ryoffs_man
                                             //      Bit[4]:   Rxoffs_man
                                             //      Bit[3:0]: Reserved
#define OV8810_FRS_30EB             (0x30EB) // rw - Bit[7:4]: isp_h_offset_man[11:8], [7:0] is at 0x307B
                                             //      Bit[3]:   Reserved
                                             //      Bit[2:0]: isp_v_offset_man[10:8], [7:0] is at 0x307B
#define OV8810_VCM_H				(0x30EC)
#define OV8810_VCM_L				(0x30ED)
#define OV8810_SLEW_H				(0x30EE)
#define OV8810_SLEW_L				(0x30EF)
#define OV8810_0x30F0				(0x30F0)
#define OV8810_0x30F2				(0x30F2)
#define OV8810_0x30F4				(0x30F4)
//0x30F7 DEBUG MODE
#define OV8810_IMAGE_TRANSFO        (0x30F8) // rw - Bit[7]:   Flip ON/OFF select, 0:Flip OFF, 1:Flip ON
                                             //      Bit[6]:   Mirror ON/OFF select, 0:Mirror OFF, 1:Mirror ON
                                             //      Bit[5]:   RISPsubV, Enable V subsample in ISP
                                             //      Bit[4]:   RISPsub, Enable H subsample in ISP
                                             //      Bit[3:2]: VSUB, 00:Full, 01:(1:2), 10:(1:4), 11:Reserved
                                             //      Bit[1:0]: HSUB, 00:Full, 01:(1:2), 10:(1:4), 11:Reserved
                                             //      Note: H subsample can be implemented in ARRAY and ISP depending on the value of RISPsub
                                             //      Note: V subsample is performed in schematic and ARRAY only
//0x30F9 DEBUG MODE
#define OV8810_IMAGE_SYSTEM         (0x30FA) // rw - Bit[7]:   software_reset, 0:Normal open, 1:All including I2C is reset to default, then go to stand by
                                             //      Bit[6:3]: Reserved
                                             //      Bit[2]:   mask_corrupted_frames, 0:Frames are dropped due to change of timing, size etc., 1:No frame drop
                                             //      Bit[1]:   grouped_parameter_hold for gain, integration and video timing signal,
                                             //                0:Timing critical signals are written immediately,
                                             //                1:Timing critical signals are written in blanking lines
                                             //      Bit[0]:   mode_select, 0:Software sleep/standby, 1:Streaming
#define OV8810_0x30FB				(0x30FB)
// (0x30FC - 0x30FE) DEBUG MODE
#define OV8810_GROUP_WR             (0x30FF) // rw - Group_write Flag Register
#define OV8810_SYS_RST				(0x3100)
#define OV8810_CLOCK_EN				(0x3101)
/*****************************************************************************
 * ISP control registers
 *****************************************************************************/
#define OV8810_TOP0           		(0x3300) // rw - ISP enable control (corresponding clock will be stopped if one module is disabled)
                                             //      Bit[7]: white pixel cancellation enable, 0:Disable, 1:Enable
                                             //      Bit[6]: black pixel cancellation enable, 0:Disable, 1:Enable
                                             //      Bit[5]: AWB ON/OFF select, 0:Auto white balance OFF, 1:Auto white balance ON
                                             //      Bit[4]: WB gain ON/OFF select, 0:Do not apply the RGB gain to sensor, 1:Apply the R/G/B gain to sensor
                                             //      Bit[3]: digital gain enable, 0:Disable, 1:Enable
                                             //      Bit[2]: Lenc_en
                                             //      Bit[1]: Even_en (disabled if blc_en = 0)
                                             //      Bit[0]: BLC enable, 0:OFF, 1:ON
#define OV8810_TOP1           		(0x3301) // rw - Bit[7]: flip_on
                                             //      Bit[6]:   mirror_on
                                             //      Bit[5:4]: Color bar pattern select
                                             //                00:Standard color bar
                                             //                01:Color bar with luminance change in V direction
                                             //                10:Color bar with luminance change in H direction
                                             //                11:Standard color bar
                                             //      Bit[3]:   bar_bl
                                             //      Bit[2]:   win_bl
                                             //      Bit[1]:   Color bar enable, 0:Color bar OFF, 1:Color bar enable
                                             //      Bit[0]:   vap_en
#define OV8810_TOP2           		(0x3302) // rw - Bit[7]: Manual WB ON/OFF select, 0:Manual mode white balance gain disable, 1: Manual mode white balance gain enable
                                             //      Bit[6]: Manual digital gain enable, 0:Auto digital gain enable, 1:Manual digital gain enable
                                             //      Bit[5]: awb_bias_en
                                             //      Bit[4]: bias_man_en
                                             //      Bit[3]: sof_sel
                                             //      Bit[2]: g_first
                                             //      Bit[1]: neg_edge
                                             //      Bit[0]: ext_sensor, Get data from external sensor
#define OV8810_CBAR               	(0x3303) // rw - Bit[7:5]: Reserved
                                             //      Bit[4]:   awb_long, 0:AWB can get long and short exposure data, 1:AWB can only get long exposure data
                                             //      Bit[3:2]: rblue_ctrl
                                             //      Bit[1]:   rblue_change_en, Enable rblue change at VSYNC fall edge
                                             //      Bit[0]:   rblue_change
#define OV8810_TOP4         		(0x3304) // rw - digital_gain_manual
#define OV8810_0x3306				(0x3306)
//0x3305~0x3307  DEBUG MODE
#define OV8810_BLC         			(0x3308) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: r_gain_man[11:8]
#define OV8810_DIG_GAIN		        (0x3309) // rw - r_gain_man[7:0]
#define OV8810_WBC_A         		(0x330A) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: g_gain_man[11:8]
#define OV8810_WBC_B         		(0x330B) // rw - g_gain_man[7:0]
#define OV8810_WBC_C         		(0x330C) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: b_gain_man[11:8]
#define OV8810_WBC_D		        (0x330D) // rw - b_gain_man[7:0]
#define OV8810_TOP_330E        		(0x330E) // rw - Range from stable to unstable, stable_rangew -> stable_range
#define OV8810_TOP_330F        		(0x330F) // rw - AWB gain will change one step every frame until it is stable
#define OV8810_0x3313				(0x3313)
// (0x3310~0x3315) DEBUG MODE
#define OV8810_SIZE_6       		(0x3316) // rw - Vertical Input Size High Bits
                                             //      Bit[7:5]: Reserved
                                             //      Bit[2:0]: dsp_vsize_in[10:8]
#define OV8810_SIZE_7       		(0x3317) // rw - Vertical Input Size Low Bits
                                             //      dsp_vsize_in[7:0]
#define OV8810_SIZE_8               (0x3318) // rw - Bit[7:4]: dsp_vpad_out
                                             //      Bit[3:0]: dsp_hpad_out
#define OV8810_SIZE_9               (0x3319) // rw - Bit[7]:   start_line
                                             //      Bit[6:4]: start_line
                                             //      Bit[3]:   even_man_en
                                             //      Bit[2]:   even_man1_en, evenodd man enable
                                             //      Bit[1]:   Evenodd from black line
                                             //      Bit[0]:   even_avg, Average current BLC with previous frame BLC
#define OV8810_SIZE_A            	(0x331A) // rw - even_man0
#define OV8810_SIZE_B            	(0x331B) // rw - even_man1
#define OV8810_0x331E				(0x331E)
#define OV8810_0x331F				(0x331F)
//0x331C~0x331F AVG - RW Frame Average Control
#define OV8810_AWB0       			(0x3320) // rw - blc_lmt_option, BLC threshold
#define OV8810_AWB1             	(0x3321) // rw - Bit[7]:   Reserved
                                             //      Bit[6:0]: blc_thre, Evenodd threshold
#define OV8810_AWB2               	(0x3322) // rw - Target for Short Exposure
#define OV8810_AWB3               	(0x3323) // rw - Target for Long Exposure
#define OV8810_AWB4           		(0x3324) // rw - BLC Offset for Short Exposure B Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man0[9:8]
#define OV8810_AWB5           		(0x3325) // rw - BLC Offset for Short Exposure B Pixel
                                             //      blc_man0[7:0]
#define OV8810_AWB6           		(0x3326) // rw - BLC Offset for Short Exposure Gb Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man1[9:8]
#define OV8810_AWB7           		(0x3327) // rw - BLC Offset for Short Exposure Gb Pixel
                                             //      blc_man1[7:0]
#define OV8810_AWB8		           	(0x3328) // rw - BLC Offset for Short Exposure Gr Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man2[9:8]
#define OV8810_WBC_29           		(0x3329) // rw - BLC Offset for Short Exposure Gr Pixel
                                             //      blc_man2[7:0]
#define OV8810_WBC_2A           		(0x332A) // rw - BLC Offset for Short Exposure R Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_ma3[9:8]
#define OV8810_WBC_2B           		(0x332B) // rw - BLC Offset for Short Exposure R Pixel
                                             //      blc_man3[7:0]
#define OV8810_WBC_2C           		(0x332C) // rw - BLC Offset for Long Exposure B Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man4[9:8]
#define OV8810_WBC_2D           		(0x332D) // rw - BLC Offset for Long Exposure B Pixel
                                             //      blc_man4[7:0]
//0x332E~0x332F  DEBUG MODE -- Debug mode
#define OV8810_BLC0           		(0x3330) // rw - BLC Offset for Long Exposure Gr Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man6[9:8]
#define OV8810_BLC1           		(0x3331) // rw - BLC Offset for Long Exposure Gr Pixel
                                             //      blc_man6[7:0]
#define OV8810_BLC2           		(0x3332) // rw - BLC Offset for Long Exposure R Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_ma7[9:8]
#define OV8810_BLC3           		(0x3333) // rw - BLC Offset for Long Exposure R Pixel
                                             //      blc_man7[7:0]
#define OV8810_BLC4					(0x3334)
#define OV8810_BLC_MAN0				(0x3336)
#define OV8810_BLC_MAN1				(0x3337)
#define OV8810_BLC_MAN2				(0x3338)
#define OV8810_BLC_MAN3				(0x3339)
#define OV8810_BLC_MAN4				(0x333A)
#define OV8810_BLC_MAN5				(0x333B)
#define OV8810_BLC_MAN6				(0x333C)
#define OV8810_BLC_MAN7				(0x333D)
#define OV8810_BLC_MAN0_1			(0x333E)
#define OV8810_BLC_MAN2_3			(0x333F)
#define OV8810_BLC_MAN5_6			(0x3340)
#define OV8810_BLC_MAN6_7			(0x3341)
//0x3342~0x3346  DEBUG MODE � � Debug Mode
#define OV8810_BLC_AVG				(0x3347)
//0x3348~0x334F  DEBUG MODE �C �C Debug Mode
//0x3350~0x33E3  LENC �C RW LENC Control
#define OV8810_LENC    				(0x33E4)
#define OV8810_VAP5    				(0x33E5)
#define OV8810_VAP6    				(0x33E6)
//0x33E7~0x33FF  DEBUG MODE �C �C Debug Mode
#define OV8810_VAP_H    			(0x3700)
//0x3701~0x3719  DEBUG MODE �C �C Debug Mode

/*****************************************************************************
 * DVP control registers
 *****************************************************************************/
#define OV8810_DVP_CTRL00           (0x3500) // rw - Bit[7]: vsync_sel2, 1:VSYNC get high after eof2v_dly from last data and get low after SOF or vsync_width PCLKS
                                             //      Bit[6]: vsync_sel, 1:Select VSYNC new
                                             //      Bit[5]: pclk_gate_en, 1:Gate dvp_pclk when no data transfer
                                             //      Bit[4]: vsync_gate, 0:Not gate dvp_pclk when VSYNC high, 1:Gate dvp_pclk when VSYNC and pclk_gate_en
                                             //      Bit[3]: dmy_line_sel, 0:Auto generate dummy lines, 1:Use first lines as dummylines
                                             //      Bit[2]: pclk_pol, Change polarity of PCLK
                                             //      Bit[1]: href_pol, Change polarity of HREF
                                             //      Bit[0]: vsync_pol, Change polarity of VSYNC
#define OV8810_DVP_CTRL01           (0x3501) // rw - Bit[7]: ccir656_en
                                             //      Bit[6]: sync_code_sel, 0:Auto generate sync_code, 1:Use FS, FE, LS and LE as CCIR656 sync_code
                                             //      Bit[5]: vhref_tst, 1:Use dvp_data_o output vhref_i
                                             //      Bit[4]: data_order, 0:DVP output dvp_data[11:0], 1:DVP output dvp_data[0:11]
                                             //      Bit[3]: dvp_bit8, 0:Swap 2 bit when dvp_h and dvp_l, 1:Swap 4 bit
                                             //      Bit[2]: dvp_h, 0:Output dvp_data[11:0], 1:Output dvp_data{n:0,11:n-1}, n:7 or 9
                                             //      Bit[1]: dvp_l, 0:Select dvp_data[11:0], 1:Select dvp_data{n:0, 11:n+1}, n:3 or 1
                                             //      Bit[0]: ch_flag, write it to 1 generate change flag for hsync mode
#define OV8810_DVP_CTRL02           (0x3502) // rw - FS
#define OV8810_DVP_CTRL03           (0x3503) // rw - FE
#define OV8810_DVP_CTRL04           (0x3504) // rw - LS
#define OV8810_DVP_CTRL05           (0x3505) // rw - LE
#define OV8810_DVP_CTRL06           (0x3506) // rw - Bit[7]:   cd_tst_sel
                                             //      Bit[6]:   Bypass DVP
                                             //      Bit[5]:   dvp_en
                                             //      Bit[4]:   hsync_en
                                             //      Bit[3]:   dvp_pclk_sel_o, s2p mode
                                             //      Bit[1:0]: vskip_man_o, for mipi
#define OV8810_DVP_CTRL07           (0x3507) // rw - Bit[7:1]: vsync_width[6:0], Width of vsync when select vsync_old and vsync_3
                                             //      Bit[0]:   hskip_man_o[0]
#define OV8810_DVP_CTRL08           (0x3508) // rw - Bit[7]:   Test pattern ON/OFF select, 0:OFF, 1:ON
                                             //      Bit[6]:   tst_bit8
                                             //      Bit[5]:   tst_bit12
                                             //      Bit[4]:   tst_mode
                                             //      Bit[3:0]: dmy_line_nu
#define OV8810_DVP_CTRL09           (0x3509) // rw - eof2v_dly[23:16]
#define OV8810_DVP_CTRL0A           (0x350A) // rw - eof2v_dly[15:8]
#define OV8810_DVP_CTRL0B           (0x350B) // rw - eof2v_dly[7:0]
#define OV8810_DVP_CTRL0C           (0x350C) // rw - pad_r
#define OV8810_DVP_CTRL0D           (0x350D) // rw - pad_l
#define OV8810_DVP_CTRL0E           (0x350E) // rw - Bit[7]:   snr_hsync_en 1, Use hsync from sensor as output href
                                             //      Bit[6]:   vsync_sel3, 1: VSYNC gets high soon after eof2v_dly from last data and gets low after vsync_width pclks, vsync_sel3 should be valid when vsync_sel2 is valid
                                             //      Bit[5:4]: vsync_width[8:7]
                                             //      Bit[3]:   sel_sync, 1: Select synced SNR VSYNC
                                             //      Bit[2]:   snr_vsync_sel, 1: Select VSYNC from sesnor
                                             //      Bit[1]:   dvp_dbg_sel
                                             //      Bit[0]:   skip_man_en_o
#define OV8810_DVP_CTRL0F           (0x350F) // rw - Bit[7]: eav_first, 0:sav_first, 1:eav_first
                                             //      Bit[6]:   f_sel
                                             //      Bit[5]:   f_value
                                             //      Bit[4]:   fix_f, 0:Auto generate ccir_f, 1:Use f_value as ccir_f
                                             //      Bit[3:2]: blk_sel, 00:Select 10'h200 and 10'h040 as toggle data
                                             //                         01:Select tog0 and tog1 as toggle data
                                             //                         10:Select 10'h000 as toggle data
                                             //                         11:Select 10'h000 as toggle data
                                             //      Bit[1]:   no_sof, 0:Reset state machine at sof, 1:Do not reset state machine when sof
                                             //      Bit[0]:   no_clip, 0:Clip output data between 10'h004 and 10'h3fb, 1:Do not clip output data
#define OV8810_DVP_CTRL10           (0x3510) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:2]: tog0[9:8]
                                             //      Bit[1:0]: tog1[9:8]
#define OV8810_DVP_CTRL11           (0x3511) // rw - tog0[7:0], Toggle data0 when line blanking or dummy lines
#define OV8810_DVP_CTRL12           (0x3512) // rw - tog1[7:0], Toggle data1 when line blanking or dummy lines
#define OV8810_DVP_CTRL13           (0x3513) // rw - Bit[0]: uv_first
#define OV8810_DVP_CTRL14           (0x3514) // rw - h2v_dly
#define OV8810_DVP_CTRL15           (0x3515) // rw - v2h_dly

/*****************************************************************************
 * MIPI control registers
 *****************************************************************************/
#define OV8810_MIPI_CTRL00          (0x3600) // rw - Bit[7]: Reserved
                                             //      Bit[6]: ck_mark1_en
                                             //      Bit[5]: gate_sc_en
                                             //      Bit[4]: line_sync_en
                                             //      Bit[3]: lane_sel_o
                                             //      Bit[2]: idel_sts
                                             //      Bit[1]: first_bit
                                             //      Bit[0]: clk_lane_dis
#define OV8810_MIPI_CTRL01          (0x3601) // rw - Bit[7]: lpkt_dt_sel
                                             //      Bit[6]: spkt_dt_sel
                                             //      Bit[5]: spkt_wc_sel
                                             //      Bit[4]: ph_bit_order
                                             //      Bit[3]: ph_byte_order
                                             //      Bit[2]: ph_byte_order2
                                             //      Bit[1]: mark1_en1
                                             //      Bit[0]: mark1_en2
#define OV8810_MIPI_CTRL02          (0x3602) // rw - Bit[7]: hs_prepare_sel
                                             //      Bit[6]: clk_prepare_sel
                                             //      Bit[5]: clk_post_sel
                                             //      Bit[4]: clk_trail_sel
                                             //      Bit[3]: hs_exit_sel
                                             //      Bit[2]: hs_zero_sel
                                             //      Bit[1]: hs_trail_sel
                                             //      Bit[0]: clk_zero_sel
#define OV8810_MIPI_CTRL03          (0x3603) // rw - Bit[7:6]: lp_glitch_nu
                                             //      Bit[5:4]: cd_glitch_nu
                                             //      Bit[3]:   cd1_int_en
                                             //      Bit[2]:   cd2_int_en
                                             //      Bit[1]:   lp_cd1_en
                                             //      Bit[0]:   lp_cd2_en
#define OV8810_MIPI_CTRL04          (0x3604) // rw - Bit[7]: wait_pkt_end
                                             //      Bit[6]: tx_lsb_first
                                             //      Bit[5]: dir_recover_sel
                                             //      Bit[4]: mipi_reg_en
                                             //      Bit[3]: inc_en
                                             //      Bit[2]: lp_tx_lane_sel
                                             //      Bit[1]: wr_first_byte
                                             //      Bit[0]: rd_ta_en
#define OV8810_MIPI_CTRL05          (0x3605) // rw - Bit[7]: lane_disable2
                                             //      Bit[6]: lane_disable1
                                             //      Bit[5]: lpx_p_sel
                                             //      Bit[4]: lp_rx_intr_sel
                                             //      Bit[3]: cd_tst_sel
                                             //      Bit[2]: mipi_reg_mask
                                             //      Bit[1]: bist_en
                                             //      Bit[0]: hd_sk_en
#define OV8810_MIPI_CTRL06          (0x3606) // rw - Bit[7]: hdr_wbc_en
                                             //      Bit[6]:   ahead_3line
                                             //      Bit[5:0]: start_size
#define OV8810_MIPI_CTRL07          (0x3607) // rw - llps_dly
// (0x3608 - 0x3609) DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL0A          (0x360A)
#define OV8810_MIPI_CTRL0B          (0x360B)
#define OV8810_MIPI_CTRL0C          (0x360C)
//0x360D DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL0E          (0x360E)
//0x360F DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL10          (0x3610) // rw - fcnt_max[15:8]
#define OV8810_MIPI_CTRL11          (0x3611) // rw - fcnt_max[7:0]
//0x3612 DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL13          (0x3613) // rw - spkt_wc_reg[7:0]
#define OV8810_MIPI_CTRL14          (0x3614) // rw - Bit[7:6]: vc
                                             //      Bit[5:0]: dt_man
#define OV8810_MIPI_CTRL15          (0x3615) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: dt_spkt
#define OV8810_MIPI_CTRL16          (0x3616) // rw - Bit[7]:   Reserved
                                             //      Bit[6:4]: reg_wr_dly
                                             //      Bit[3]:   Reserved
                                             //      Bit[2:0]: reg_rd_dly
#define OV8810_MIPI_CTRL17          (0x3617) // rw - Reserved
#define OV8810_MIPI_CTRL18          (0x3618) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_zero_min[9:8]
#define OV8810_MIPI_CTRL19          (0x3619) // rw - hs_zero_min[7:0]
#define OV8810_MIPI_CTRL1A          (0x361A) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_trail_min[9:8]
#define OV8810_MIPI_CTRL1B          (0x361B) // rw - hs_trail_min[7:0]
#define OV8810_MIPI_CTRL1C          (0x361C) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_zero_min[9:8]
#define OV8810_MIPI_CTRL1D          (0x361D) // rw - clk_zero_min[7:0]
#define OV8810_MIPI_CTRL1E          (0x361E) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_prepare_min[9:8]
#define OV8810_MIPI_CTRL1F          (0x361F) // rw - clk_prepare_min[7:0]
#define OV8810_MIPI_CTRL20          (0x3620) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_post_min[9:8]
#define OV8810_MIPI_CTRL21          (0x3621) // rw - clk_post_min[7:0]
#define OV8810_MIPI_CTRL22          (0x3622) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_trail_min[9:8]
#define OV8810_MIPI_CTRL23          (0x3623) // rw - clk_trail_min[7:0]
#define OV8810_MIPI_CTRL24          (0x3624) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: lpx_p_min[9:8]
#define OV8810_MIPI_CTRL25          (0x3625) // rw - lpx_p_min[7:0]
#define OV8810_MIPI_CTRL26          (0x3626) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_prepare_min[9:8]
#define OV8810_MIPI_CTRL27          (0x3627) // rw - hs_prepare_min[7:0]
#define OV8810_MIPI_CTRL28          (0x3628) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_exit_min[9:8]
#define OV8810_MIPI_CTRL29          (0x3629) // rw - hs_exit_min[7:0]
//0x362A~0x362D  DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL2F          (0x362F) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_clk_trail_min
#define OV8810_MIPI_CTRL30          (0x3630) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_lpx_p_min

/*****************************************************************************
 * Default values
 *****************************************************************************/

// column 1)
//    Reset values as of datasheet OV8810_DS_1.1_SiliconImage.pdf from
//    6.12.2008, values not given in the datasheet are values from 2) or 3).
// column 2)
//    For Rev. A sensor. Values deviating from the datasheet are derived from
//    evaluation kit settings OV8810R1A_A01.ovd SEN_2592x1944. AWB, AE, AGC
//    etc. of the sensor is on.
// column 3)
//    For Rev. C sensors. Values deviating from the datasheet are derived from
//    evaluation kit settings OV8810R1C_A04.ovd SEN_2592x1944_3.75fps. AWB, AE,
//    AGC etc. of the sensor is on.
//    May also be used for Rev. A sensor (not tested so far).
//
// Make sure that these static settings are reflecting the capabilities defined
// in IsiGetCapsIss (further dynamic setup may alter these default settings but
// often does not if there is no choice available).

//                                              3)         2)       1)

/*****************************************************************************
 * System control registers
 *****************************************************************************/
#define OV8810_AGCL_DEFAULT                   (0x30) //  (0x00)   (0x00)// hacked by Rongjie
#define OV8810_AECL_2_DEFAULT                 (0x09) //(0x0000) (0x0000)
#define OV8810_AECL_1_DEFAULT                 (0xB2) //(0x0000) (0x0000)
#define OV8810_LAEC_2_DEFAULT                 (0x00) //(0x0000) (0x0000)
#define OV8810_LAEC_1_DEFAULT                 (0x00) //(0x0000) (0x0000)
#define OV8810_AGC_DEFAULT            		  (0x00) //  (0x00)   (0x00) reserved in the register list but [0] referenced by manual digital gain
#define OV8810_PIDH_DEFAULT                   (0x88) //  (0x88)   (0x88)
#define OV8810_PIDL_DEFAULT                   (0x13) //  (0x32)   (0x32) 0x32 = Rev. A, 0x33 = Rev. C  /*FeiP: modify from 0x12*/
#define OV8810_SCCB_ID_DEFAULT                (0x6C) //  (0x6C)   (0x6C)
#define OV8810_R_PLL_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV8810_R_PLL1_DEFAULT                 (0x84) //  (0x38)   (0x38)
#define OV8810_R_PLL2_DEFAULT                 (0x48) //  (0x10)   (0x01)
#define OV8810_R_PLL3_DEFAULT                 (0x28) //  (0x02)   (0x02)
#define OV8810_R_PLL4_DEFAULT                 (0x28) //  (0x80)   (0x40)
#define OV8810_SYS_DEFAULT                    (0x00) //  (0x00)   (0x00)
#define OV8810_AUTO1_DEFAULT                  (0xC7) //  (0xFF)   (0x00)
#define OV8810_AUTO2_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV8810_AUTO3_DEFAULT                  (0x33) //  (0x33)   (0x00)
#define OV8810_0x3016_DEFAULT				  (0x03)
#define OV8810_WPT_DEFAULT                    (0x44) //  (0x50)   (0x78)
#define OV8810_BPT_DEFAULT                    (0x3C) //  (0x40)   (0x68)
#define OV8810_VPT_DEFAULT                    (0x82) //  (0x82)   (0xD4)
#define OV8810_YAVG_DEFAULT                   (0x00) //  (0x00)   (0x00)
#define OV8810_AECG_MAX50_DEFAULT             (0x05) //  (0x05)   (0x05)
#define OV8810_AECG_MAX60_DEFAULT             (0x07) //  (0x07)   (0x07)
#define OV8810_ADDVS_2_DEFAULT                (0x00) //  (0x00) (0x00)
#define OV8810_ADDVS_1_DEFAULT                (0x00) //  (0x00) (0x00)
#define OV8810_FRAME_LENGTH_LINES2_DEFAULT    (0x07) //  (0x07BC) (0x07BC)
#define OV8810_FRAME_LENGTH_LINES1_DEFAULT    (0xBC) //  (0x07BC) (0x07BC)
#define OV8810_LINE_LENGTH_PCK2_DEFAULT       (0x0C) //  (0x0CA0) (0x0CA0)
#define OV8810_LINE_LENGTH_PCK1_DEFAULT       (0xA0) //  (0x0CA0) (0x0CA0)
#define OV8810_X_ADDR_START_2_DEFAULT         (0x00) //  (0x0000) (0x0000)
#define OV8810_X_ADDR_START_1_DEFAULT         (0x00) //  (0x0000) (0x0000)
#define OV8810_Y_ADDR_START_2_DEFAULT         (0x00) //  (0x0000) (0x0000)
#define OV8810_Y_ADDR_START_1_DEFAULT         (0x00) //  (0x0000) (0x0000)
#define OV8810_X_ADDR_END_2_DEFAULT           (0x0A) //  (0x0A1F) (0x0A1F)
#define OV8810_X_ADDR_END_1_DEFAULT           (0x1F) //  (0x0A1F) (0x0A1F)
#define OV8810_Y_ADDR_END_2_DEFAULT           (0x07) //  (0x0797) (0x0797)
#define OV8810_Y_ADDR_END_1_DEFAULT           (0xab) //  (0x0797) (0x0797)
#define OV8810_X_OUTPUTSIZE_2_DEFAULT         (0x0A) //  (0x0A20) (0x0A20)
#define OV8810_X_OUTPUTSIZE_1_DEFAULT         (0x20) //  (0x0A20) (0x0A20)
#define OV8810_Y_OUTPUTSIZE_2_DEFAULT         (0x07) //  (0x0798) (0x0798)
#define OV8810_Y_OUTPUTSIZE_1_DEFAULT         (0x98) //  (0x0798) (0x0798)
#define OV8810_DATR_D56_DEFAULT				  (0x00)
#define OV8810_DATR_OTP_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV8810_R_SIGMA_DEFAULT                (0x09) //  (0xA7)   (0xA7)
#define OV8810_0x3058_DEFAULT				  (0x01)
#define OV8810_0x3059_DEFAULT				  (0xa0)
#define OV8810_BD50ST_2_DEFAULT               (0x00) //  (0x0129) (0x0055)
#define OV8810_BD50ST_1_DEFAULT               (0x73) //  (0x0129) (0x0055)
#define OV8810_BD60ST_2_DEFAULT               (0x55) //  (0x00f7) (0x5501)
#define OV8810_BD60ST_1_DEFAULT               (0x01) //  (0x00f7) (0x5501)
#define OV8810_0x3065_DEFAULT				  (0x40)
#define OV8810_0x3067_DEFAULT				  (0x40)
#define OV8810_0x3068_DEFAULT				  (0x08)
#define OV8810_0x3069_DEFAULT				  (0x80)
#define OV8810_0x306A_DEFAULT				  (0x05)
#define OV8810_0x306B_DEFAULT				  (0x00)
#define OV8810_TMC1_DEFAULT            		  (0x40) //  (0x20)   (0x20) reserved but written by eva kit
#define OV8810_0x3072_DEFAULT				  (0x0D)
#define OV8810_TMC5_DEFAULT            		  (0x29) //  (0x22)   (0x22) reserved but written by eva kit
#define OV8810_TMC6_DEFAULT            		  (0x29) //  (0x22)   (0x22) reserved but written by eva kit
#define OV8810_TMC7_DEFAULT            		  (0x29) //  (0x24)   (0x24) reserved but written by eva kit
#define OV8810_TMC8_DEFAULT            		  (0x29) //  (0x24)   (0x24) reserved but written by eva kit
#define OV8810_TMC9_DEFAULT            		  (0x0A) //  (0x44)   (0x44) reserved but written by eva kit
#define OV8810_TMCE_DEFAULT            		  (0x00)
#define OV8810_0x3080_DEFAULT				  (0x40)
#define OV8810_0x3082_DEFAULT				  (0x00)
#define OV8810_0x3084_DEFAULT				  (0x44)
#define OV8810_0x3087_DEFAULT				  (0x41)
#define OV8810_0x308A_DEFAULT				  (0x02)
#define OV8810_0x308D_DEFAULT				  (0x00)
#define OV8810_0x3090_DEFAULT				  (0x36)
#define OV8810_R_ARRAY_DEFAULT            	  (0x00) //  (0x04)   (0x04) reserved but written by eva kit
#define OV8810_0x3092_DEFAULT				  (0x08)
#define OV8810_0x3094_DEFAULT				  (0x01)
#define OV8810_0x3095_DEFAULT				  (0x0a)
#define OV8810_0x3098_DEFAULT				  (0x24)
#define OV8810_0x3099_DEFAULT				  (0x81)
#define OV8810_0x309A_DEFAULT				  (0x64)
#define OV8810_0x309B_DEFAULT				  (0x00)
#define OV8810_0x309C_DEFAULT				  (0x00)
#define OV8810_0x309D_DEFAULT				  (0x64)
#define OV8810_0x309E_DEFAULT				  (0x1B)
#define OV8810_0x309F_DEFAULT				  (0x23)
#define OV8810_0x30A0_DEFAULT				  (0x40)
#define OV8810_SLEW_DEFAULT            		  (0x00) //  (0x44)   (0x44) reserved but written by eva kit (0x44 read after reset)
#define OV8810_R_PWC_DEFAULT            	  (0x00) //  (0x06)   (0x06) reserved but written by eva kit
#define OV8810_0x30AA_DEFAULT				  (0x59)
#define OV8810_0x30AB_DEFAULT				  (0x44)
#define OV8810_IO_CTRL0_DEFAULT               (0xFF) //  (0xFF)   (0xFF)
#define OV8810_IO_CTRL1_DEFAULT               (0xEF) //  (0xFF)   (0xFF)
#define OV8810_IO_CTRL2_DEFAULT               (0x10) //  (0x32)   (0x00)// hacked by Rongjie
#define OV8810_DSIO_3_DEFAULT                 (0x08) //  (0x00)   (0x00)
#define OV8810_DSIO_4_DEFAULT                 (0x0C) //   the default value is 0x0c
#define OV8810_FRS_30B7_DEFAULT               (0x80) //  (0x80)   (0x80)
#define OV8810_0x30BE_DEFAULT				  (0x08)
#define OV8810_0x30BF_DEFAULT				  (0x80)
#define OV8810_0x30E3_DEFAULT				  (0X0E)
#define OV8810_FRS_30E4_DEFAULT               (0x07) //  (0x07)   (0x07)
#define OV8810_FRS_30E5_DEFAULT            	  (0x00) //(0x0020) (0x0020)
#define OV8810_FRS_30E6_DEFAULT            	  (0x00) //(0x0020) (0x0020)
#define OV8810_FRS_30E7_DEFAULT               (0x01) //  (0x00)   (0x00)
#define OV8810_FRS_30E8_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV8810_0x30E9_DEFAULT                 (0x09)
#define OV8810_FRS_30EA_DEFAULT               (0x38) //  (0x00)   (0x00)
#define OV8810_FRS_30EB_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV8810_VCM_H_DEFAULT				  (0x01)
#define OV8810_VCM_L_DEFAULT				  (0x50)
#define OV8810_SLEW_H_DEFAULT				  (0x05)
#define OV8810_SLEW_L_DEFAULT				  (0x46)
#define OV8810_0x30F0_DEFAULT				  (0x00)
#define OV8810_0x30F2_DEFAULT				  (0x00)
#define OV8810_0x30F4_DEFAULT				  (0x90)
#define OV8810_IMAGE_TRANSFO_DEFAULT          (0x40) //  (0x00)   (0x00)
#define OV8810_IMAGE_SYSTEM_DEFAULT           (0x00) //  (0x00)   (0x00) do not enable streaming here, streaming is off during sensor setup and will be enabled afterwards in IsiSensorSetStreamingIss
#define OV8810_0x30FB_DEFAULT				  (0xC9)
#define OV8810_GROUP_WR_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV8810_SYS_RST_DEFAULT				  (0x00)
#define OV8810_CLOCK_EN_DEFAULT				  (0xFF)

/*****************************************************************************
 * ISP control registers
 *****************************************************************************/
#define OV8810_TOP0           	  	(0x3300) // rw - ISP enable control 00 - corresponding clock will be stopped if one module is disabled(0:disable, 1: enable)
												 // 	 Bit[7]: ISP_en
												 // 	 Bit[6]: AWB_stat_en
												 // 	 Bit[5]: AWB_gain_en
												 // 	 Bit[4]: LENC_en
												 // 	 Bit[3]: WC_en(white pixel cancellation)
												 // 	 Bit[2]: BC_en(black pixel cancellation)
												 // 	 Bit[1]: EvenOdd_en (functions only when Bit[0] = 1)
												 // 	 Bit[0]: BLC_en
#define OV8810_TOP1           	  	(0x3301) // rw - ISP enable control 01 - corresponding clock will be stopped if one module is disabled
												 // 	 Bit[7:4]: Debug mode
												 // 	 Bit[3]:   frame_average_en
												 // 	 Bit[2]:   vario_pixel_en
												 // 	 Bit[1]:   digital_gain_en
												 // 	 Bit[0]:   Debug mode
#define OV8810_TOP2           	  	(0x3302) // rw - ISP enable control 02 - corresponding clock will be stopped if one module is disabled(0:disable, 1: enable)
												 // 	 Bit[7:4]: Debug mode
												 // 	 Bit[3]:   horizontal_vario_pixel_en
												 // 	 Bit[2]:	 combine_en
												 // 	 Bit[1]:   stretch_en
												 // 	 Bit[0]:   Debug mode
#define OV8810_CBAR               	(0x3303) // rw - Bit[7]:   Bar move enable, 0: Disable bar move, 1:Enable a H bar moving from top to bottom
												 // 	 Bit[0]:   Color bar enable, 0: Color bar OFF, 1: Color bar enable
#define OV8810_TOP4         	  	(0x3304) // rw - Bit[7:6] avg_sel[1:0], Frame average input select
												 // 	 Bit[5:3] Debug mode
												 // 	 Bit[2]   buf_ctrl_en
												 // 	 Bit[1]   bist_en
												 // 	 Bit[0]   Debug mode
#define OV8810_0x3306		  		(0x3306)
	//0x3305~0x3307  DEBUG MODE
#define OV8810_BLC         		  	(0x3308) // rw - Bit[3]:  dg_man_en(fuction only when register 0x3301[1]=1), 0: dig_gain auto mode, 1: dig_gain manual mode
#define OV8810_DIG_GAIN		  		(0x3309) // rw - Bit[7:0]: dg_gain[7:0] Manual digital gain, Digital gain = (Bit[7]+1)*[Bit[6]+1)+Bit[5:0]/64
#define OV8810_WBC_A         	  	(0x330A) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5:4]: wbc_bd_sel[1:0] Boundary select options
												 // 	 Bit[3]:   wbc_se_en Enable same channel detection
												 // 	 Bit[2]:   wbc_dc_en Enable different channel detection
												 // 	 Bit[1]:   wbc_smooth_en Enable using average G values when doing recovery
												 // 	 Bit[0]:   wbc_detail_en Enable detail detection method
#define OV8810_WBC_B         	  	(0x330B) // rw - Bit[7]:   Debug mode
												 // 	 Bit[6:0]: wbc_wthre[6:0] Threshold value for detecting white pixels
#define OV8810_WBC_C         	  	(0x330C) // rw - Bit[7:0]: wbc_bthre[7:0] Threshold value for detecting black pixels
#define OV8810_WBC_D		        (0x330D) // rw - Bit[7:0]: wbc_thre[7:0] Threshold value used in recovery
#define OV8810_TOP_330E        	  	(0x330E) // rw - Indirect Read Address
#define OV8810_TOP_330F        	  	(0x330F) // ro - Indirect Read Data
#define OV8810_0x3313		  		(0x3313)
	// (0x3310~0x3315) DEBUG MODE
#define OV8810_SIZE_6       	  	(0x3316) // rw - Bit[7:0]: H_pad_start[7:0]
#define OV8810_SIZE_7       	  	(0x3317) // rw - Bit[7:0]: H_pad_end[7:0]
#define OV8810_SIZE_8               (0x3318) // rw - Bit[7:4]: H_pad_end[11:8]
												 // 	 Bit[3:0]: H_pad_start[11:8]
#define OV8810_SIZE_9               (0x3319) // rw - Bit[7:0]: V_pad_start[7:0]
#define OV8810_SIZE_A		  		(0x331A) // rw - Bit[7:0]: V_pad_end[7:0]
#define OV8810_SIZE_B            	(0x331B) // rw - Bit[7:0]: V_pad_end[11:8]
												 // 	 Bit[3:0]: V_pad_start[11:8]
	//0x331C~0x331D AVG - RW Frame Average Control
#define OV8810_AVG_E			  	(0x331E) // rw - Frame Average Control
#define OV8810_AVG_F			  	(0x331F) // rw - Frame Average Control
#define OV8810_AWB0       		  	(0x3320) // rw - Bit[7]:   fast_AWB
												 // 	 Bit[6]:   awb_man_en
												 // 	 Bit[5:0]  awb_delta  AWB gain adjustment step
#define OV8810_AWB1             	(0x3321) // rw - Bit[7:0]: stable_range[7:0] Threshold from unstable to stable
#define OV8810_AWB2                 (0x3322) // rw - Bit[7:0]: stable_rangew[7:0] Threshold from stable to unstable. This should be greater than stable_range
#define OV8810_AWB3               	(0x3323) // rw - Bit[7:0]: AWB_frame_cnt[7:] Change AWB gain speed
#define OV8810_AWB4           	  	(0x3324) // rw - Bit[7:0]: AWB_R_gain[11:4]
#define OV8810_AWB5           	  	(0x3325) // rw - Bit[7:0]: AWB_G_gain[11:4]
#define OV8810_AWB6           	  	(0x3326) // rw - Bit[7:0]: AWB_B_gain[11:4]
#define OV8810_AWB7           	  	(0x3327) // rw - Bit[7:4]: AWB_R_gain[3:0]
												 // 	 Bit[3:0]: AWB_G_gain[3:0]
#define OV8810_AWB8		         	(0x3328) // rw - Bit[7]:   AWB_freeze_en
												 // 	 Bit[6]:   AWB_sel, 0:From VarioPixel, 1:From AWB_gain
												 // 	 Bit[5:4]: Debug mode
												 // 	 Bit[3:0]: AWB_B_gain[3:0]
#define OV8810_WBC_29           	(0x3329) // rw - Bit[7]:   Remove cross cluster option enable
												 // 	 Bit[6]:   Remove tail option enable
												 // 	 Bit[5]:   Anti-artifact option enable
#define OV8810_WBC_2A           	(0x332A) // rw - Bit[7]:   WBC_man Enable manual mode in which wthre,bthre and detail_en can be set manually
												 // 	 Bit[6:0]: wbc_wthre[6:0] Reference threshold when determining wthre in auto mode
#define OV8810_WBC_2B           	(0x332B) // rw - Bit[7]:   WBC_gain_man_en
												 // 	 Bit[6:4]: WBC_refgain_pwr[2:0], Range:0-5
												 // 	 Bit[3]:   Debug mode
												 // 	 Bit[2:0]: WBC_shift[2:0], Range:0-7
#define OV8810_WBC_2C           	(0x332C) // rw - Bit[7:3]: Debug mode
												 // 	 Bit[2:0]: WBC_gain_pwr[2:0], Range 0-5
#define OV8810_WBC_2D           	(0x332D) // rw - Bit[7]:   WBC_gain_man[7:0]
	//0x332E~0x332F  DEBUG MODE -- Debug mode
#define OV8810_BLC0           	  	(0x3330) // rw - BLC Control
#define OV8810_BLC1           	  	(0x3331) // rw - Bit[7:0]: BLC target[7:0] low byte
#define OV8810_BLC2           	  	(0x3332) // rw - Bit[7:0]: BLC debug must be set to 0x3331
#define OV8810_BLC3           	  	(0x3333) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5:4]: BLC target[8:9] high byte
												 // 	 Bit[3:2]: BLC debug must be set to Bit[5:4]
												 // 	 Bit[0]:   BLC option, 0: BLC value is controlled by register 0x3347[1]
												 // 			 1: If gain is changed, no matter BLC is in average mode(register 0x3347[1]=1,BLC will use current frame BLC value
#define OV8810_BLC4			  		(0x3334) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5]:   Manual BLC offset option(functions only when Bit[4]=1)
												 // 			 0:Used different registers for different channels, 1:use blc_man0 to compensate all channels BLC
												 // 	 Bit[4]:   Manual BLC adjust enable(functions only when 0x3300[0]=1), 0:Auto BLC, 1:Manual BLC
												 // 	 Bit[3:0]: Debug mode
#define OV8810_BLC_MAN0		  		(0x3336) // rw - Bit[7:0] B channel BLC offset[7:0] low byte
#define OV8810_BLC_MAN1		  		(0x3337) // rw - Bit[7:0] Gb channel BLC offset[7:0] low byte
#define OV8810_BLC_MAN2		  		(0x3338) // rw - Bit[7:0] Gr channel BLC offset[7:0] low byte
#define OV8810_BLC_MAN3		  		(0x3339) // rw - Bit[7:0] R channel BLC offset[7:0] low byte
#define OV8810_BLC_MAN4		  		(0x333A) // rw - Bit[7:0] BLC debug must be set to same value as register 0x3336
#define OV8810_BLC_MAN5		  		(0x333B) // rw - Bit[7:0] BLC debug must be set to same value as register 0x3337
#define OV8810_BLC_MAN6		  		(0x333C) // rw - Bit[7:0] BLC debug must be set to same value as register 0x3338
#define OV8810_BLC_MAN7		  		(0x333D) // rw - Bit[7:0] BLC debug must be set to same value as register 0x3339
#define OV8810_BLC_MAN0_1		  	(0x333E) // rw - Bit[7:4] B channel BLC offset[11:8] high byte
												 // 	 Bit[3:0] Gb channel BLC offset[11:8] high byte
#define OV8810_BLC_MAN2_3		  	(0x333F) // rw - Bit[7:4] Gr channel BLC offset[11:8] high byte
												 // 	 Bit[3:0] R channel BLC offset[11:8] high byte
#define OV8810_BLC_MAN5_6		  	(0x3340) // rw - Bit[7:4]: BLC debug must be set to same value as register 0x333E[7:4]
												 // 	 Bit[3:0]: BLC debug must be set to same value as register 0x333E[3:0]

#define OV8810_BLC_MAN6_7		  	(0x3341) // rw - Bit[7:4]: BLC debug must be set to same value as register 0x333F[7:4]
												 // 	 Bit[3:0]: BLC debug must be set to same value as register 0x333F[3:0]
	//0x3342~0x3346  DEBUG MODE ??Debug Mode
#define OV8810_BLC_AVG		  		(0x3347) // rw - Bit[7:2]: Debug mode
												 // 	 Bit[1]:   BLC_avg, 0: User current BLC, 1:Average current BLC with previous frame BLC
												 // 	 Bit[0]:   Even/Odd option, 0: Extract even/odd information from prebious frame
												 // 						1: Extract even/odd information from blank line
	//0x3348~0x334F  DEBUG MODE �C �C Debug Mode
	//0x3350~0x33E3  LENC �C RW LENC Control
#define OV8810_LENC    		  		(0x33E4) // rw - Bit[7:4]: Debug mode
												 // 	 Bit[3:2]: LENC vertical skip
												 // 	 Bit[1]:   LENC auto gain enable
												 // 	 Bit[0]:   LENC horizontal skip, For full resolution or cropping from full resolution, set register 0x33E4 = 0x02
												 // 			For 2:1 downsampling, set register 0x33E4 = 0x07, For 4:1 downsampling, set register 0x33E4 = 0x0B
												 // 			For 8:1 downsampling, set register 0x33E4 = 0x0F
#define OV8810_VAP5    		  		(0x33E5) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5:4]: vap_addopt[1:0] VarioPixel option
												 // 	 Bit[3:2]: vap_vskip[1:0], Vertical skip, 00:(1:1), 01:(1:2), 10:(1:4), 11:(1:1)
												 // 	 Bit[1:0]: vap_hskip[1:0], Horizontal skip, 00:(1:1), 01:(1:2), 10:(1:4), 11:(1:1)
#define OV8810_VAP6    		  		(0x33E6) // rw - Bit[7:4]: Debug mode
												 // 	 Bit[3:0]: vap_avg_eb[3:0], Average enable
	//0x33E7~0x33FF  DEBUG MODE �C �C Debug Mode
#define OV8810_VAP_H    		  	(0x3700) // rw - VAP_H_mode[7:0]
												 // 	 Bit[7:6]: B_mode[1:0], 00:Average, 01:Drop second, 1x: Drop first
												 // 	 Bit[5:4]: Gb_mode[1:0], 00:Average, 01:Drop second, 1x: Drop first
												 // 	 Bit[3:2]: Gr_mode[1:0], 00:Average, 01:Drop second, 1x: Drop first
												 // 	 Bit[1:0]: R_mode[1:0], 00:Average, 01:Drop second, 1x: Drop first
	//0x3701~0x3719  DEBUG MODE �C �C Debug Mode
#define OV8810_ISP_TEST             (0x3701)    /* Debug by FeiPeng TODO*/

	/*****************************************************************************
	 * DVP control registers
	 *****************************************************************************/
#define OV8810_DVP_CTRL00           (0x3500) // rw - DVP Control 00
												 // 	 Bit[7:6]: VSYNC select, 00:select vsync_old, 01:select vsync_new, 10: select vsync_three, 11:Debug mode
												 // 	 Bit[5]:   pclk_gate_en, 1:Gate dvp_pclk when HREF is low
												 // 	 Bit[4]:   vsync_gate, 0:Gate dvp_pclk when VSYNC and pclk_gate_en is high, 1:Do not gate dvp_pclk when VSYNC high
												 // 	 Bit[3]:   dmy_line_sel, 0:Auto generate dummy lines, 1:Use first lines as dummylines
												 // 	 Bit[2]:   pclk_pol, Change polarity of PCLK
												 // 	 Bit[1]:   href_pol, Change polarity of HREF
												 // 	 Bit[0]:   vsync_pol, 0: VSYNC is frame blanking time
#define OV8810_DVP_CTRL01           (0x3501) // rw - Bit[7]: ccir656_en
												 // 	 Bit[6]: sync_code_sel, 0:Auto generate sync_code, 1:Use FS, FE, LS and LE as CCIR656 sync_code
												 // 	 Bit[5]: Debug mode
												 // 	 Bit[4]: data_order, 0:DVP output dvp_data[11:0], 1:DVP output dvp_data[0:11]
												 // 	 Bit[3]: dvp_bit8, 0:Swap 2 bit when dvp_h and dvp_l, 1:Swap 4 bit
												 // 	 Bit[2]: dvp_h, 0:Output dvp_data[11:0], 1:Output dvp_data{n:0,11:n-1}, n:7 or 9
												 // 	 Bit[1]: dvp_l, 0:Select dvp_data[11:0], 1:Select dvp_data{n:0, 11:n+1}, n:3 or 1
												 // 	 Bit[0]: ch_flag, write it to 1 generate change flag for hsync mode
#define OV8810_DVP_CTRL02           (0x3502) // rw - DVP Control 02, Bit[7:0]: CCIR656 sync code for FS
#define OV8810_DVP_CTRL03           (0x3503) // rw - DVP Control 03, Bit[7:0]: CCIR656 sync code for FE
#define OV8810_DVP_CTRL04           (0x3504) // rw - DVP Control 04, Bit[7:0]: CCIR656 sync code for LS
#define OV8810_DVP_CTRL05           (0x3505) // rw - DVP Control 05, Bit[7:0]: CCIR656 sync code for LE
#define OV8810_DVP_CTRL06           (0x3506) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5]:   dvp_en
												 // 	 Bit[4]:   hsync_en
												 // 	 Bit[3:0]: Debug mode
#define OV8810_DVP_CTRL07           (0x3507) // rw - Bit[7:1]: vsync_width[7:1], Width of vsync when select vsync_old and vsync_three
												 // 	 Bit[0]:   hskip_man_o[0]
#define OV8810_DVP_CTRL08           (0x3508) // rw - Bit[7]:   Test pattern ON/OFF select, 0:OFF, 1:ON
												 // 	 Bit[6]:   tst_bit8
												 // 	 Bit[5]:   tst_bit12
												 // 	 Bit[4]:   tst_mode, 0:00 .....FF, 1:00 00....FF FF
												 // 	 Bit[3:0]: dmy_line_nu
#define OV8810_DVP_CTRL09           (0x3509) // rw - eof2v_dly[23:16]
#define OV8810_DVP_CTRL0A           (0x350A) // rw - eof2v_dly[15:8]
#define OV8810_DVP_CTRL0B           (0x350B) // rw - eof2v_dly[7:0]
#define OV8810_DVP_CTRL0C           (0x350C) // rw - pad_right
#define OV8810_DVP_CTRL0D           (0x350D) // rw - pad_left
#define OV8810_DVP_CTRL0E           (0x350E) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5:4]: vsync_width[9:8]
												 // 	 Bit[3:1]: Debug mode
												 // 	 Bit[0]:   skip_man_en_o
#define OV8810_DVP_CTRL0F           (0x350F) // rw - Bit[7]: eav_first, 0:sav_first, 1:eav_first
												 // 	 Bit[6]:   f_sel
												 // 	 Bit[5]:   f_value
												 // 	 Bit[4]:   fix_f, 0:Auto generate ccir_f, 1:Use f_value as ccir_f
												 // 	 Bit[3:2]: blk_sel, 00:Select 12'h800 and 12'h100 as toggle data
												 // 						x1:Select 12'h000 as toggle data
												 // 						10:Select tog0 and tog1 as toggle data
												 // 	 Bit[1]:   no_sof, 0:Reset state machine at sof, 1:Do not reset state machine when sof
												 // 	 Bit[0]:   no_clip, 0:Clip output data between 10'h004 and 10'h3fb, 1:Do not clip output data when in CCIR656 mode
#define OV8810_DVP_CTRL10           (0x3510) // rw - Bit[7:4]: Debug mode
												 // 	 Bit[3:2]: tog0[11:10] Toggle data0 when line blanking or dummy lines
												 // 	 Bit[1:0]: tog1[11:10] Toggle data1 when line blanking or dummy lines
#define OV8810_DVP_CTRL11           (0x3511) // rw - Bit[7:0]: tog0[9:2], Toggle data0 when line blanking or dummy lines
#define OV8810_DVP_CTRL12           (0x3512) // rw - Bit[7:0]: tog1[9:2], Toggle data1 when line blanking or dummy lines
#define OV8810_DVP_CTRL13           (0x3513) // rw - Bit[7:0]: Debug mode
#define OV8810_DVP_CTRL14           (0x3514) // rw - Bit[7:0]: h2v_dly
#define OV8810_DVP_CTRL15           (0x3515) // rw - Bit[7:0]: v2h_dly

	/*****************************************************************************
	 * MIPI control registers
	 *****************************************************************************/
#define OV8810_MIPI_CTRL00          (0x3600) // rw - Bit[7]: lp_p1_out
												 // 	 Bit[6]: lp_n1_out
												 // 	 Bit[5]: lp_p2_out
												 // 	 Bit[4]: lp_n2_out
												 // 	 Bit[3]: lp_sel1, 0: Auto generate lp_dir1, 1: Use lp_dir_man1 as lp_dir1
												 // 	 Bit[2]: lp_sel2, 0: Auto generate lp_dir2, 1: Use lp_dir_man2 as lp_dir2
												 // 	 Bit[1]: lp_dir_man1, 0: Input, 1: Output
												 // 	 Bit[0]: lp_dir_man2, 0: Input, 1: Output
#define OV8810_MIPI_CTRL01          (0x3601) // rw - Bit[7]: Debug mode
												 // 	 Bit[6]: line_sync_en, 1: Enable line sync short packet
												 // 	 Bit[5]: lane_sel,	0: Use lane1 as default data lane, 1: Select lane 2 as default data lane
												 // 	 Bit[4]: lp_tx_lane_sel, 0: Select lane1 as lp_tx_lane, 1: Select lane2 as lp_tx_lane
												 // 	 Bit[3]: Debug mode
												 // 	 Bit[2]: ph_byte_order2, PH sequence for ECC calculation, 0: {DI, WC}, 1: {WC, DI}
												 // 	 Bit[1]: ph_byte_order, PH sequence for ECC calculation, 0: {WC_l, WC_h}, 1: {WC_h, WC_l}
												 // 	 Bit[0]: ph_bit_order, 0: DI[7:0], WC[7:0], WC[15:8], 1: DI[0:7], WC[0:7], WC[8:15]
#define OV8810_MIPI_CTRL02          (0x3602) // rw - Bit[7:6]: VC, Virtual channel ID
												 // 	 Bit[5:0]: dt_dmy, Data type for dummy lines
#define OV8810_MIPI_CTRL03          (0x3603) // rw - Bit[7:6]: Debug mode
												 // 	 Bit[5:0]: dt_man, Manually set data type
#define OV8810_MIPI_CTRL04          (0x3604) // rw - Bit[7:3]: Debug mode
												 // 	 Bit[2]: spkt_dt_sel,  1: Use dt_dmy as short packet
												 // 	 Bit[1]: lpkt_dt_sel, 0: Use dt_auto, 1: Use dt_man as long packet
												 // 	 Bit[0]: spkt_wc_sel, 1: Select spkt_wc_reg as short packet, wc
#define OV8810_MIPI_CTRL05          (0x3605) // rw - Bit[7:0]: spkt_wc_reg[15:8]
#define OV8810_MIPI_CTRL06          (0x3606) // rw - Bit[7:0]: spkt_wc_reg[7:0]
#define OV8810_MIPI_CTRL07          (0x3607) // rw - Bit[7]: Debug mode
												 // 	 Bit[6]: ta_en, Enable send turnaround command after read data transmission is finished
												 // 	 Bit[0]: Debug mode
	// (0x3608 - 0x3609) DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL0A          (0x360A) // rw - Bit[7]: Debug mode
												 // 	 Bit[6]: gate_sc_en, 0: Clock is in free running mode, 1: Gate MIPI clock when there is no packet
												 // 	 Bit[5:0]: Debug mode
#define OV8810_MIPI_CTRL0B          (0x360B) // rw - Bit[7]: cd1_int_en, 1: Enable internal cd1 enable
												 // 	 Bit[6]: cd2_int_en, 1: Enable internal cd2 enable
												 // 	 Bit[5]: lp_cd_en1, 1: Enable LP CD detection on lane1
												 // 	 Bit[4]: lp_cd_en2, 1: Enable LP CD detection on lane2
												 // 	 Bit[3:2]: Debug mode
												 // 	 Bit[1]: lane_disable1, 1: Disable MIPI data lane1 to LP00 state
												 // 	 Bit[0]: lane_disable2, 1: Disable MIPI data lane2 to LP00 state
#define OV8810_MIPI_CTRL0C          (0x360C) // rw - Bit[7:2]: Debug mode
												 // 	 Bit[1]: inc_en, 1: mipi_reg_addr will auto increment by 1
												 // 	 Bit[0]: Debug mode
	//0x360D DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL0E          (0x360E) // rw - Bit[7]: lpx_p_sel, 0: Select lpx_p_cal, 1: Select lpx_p_reg[7:0] as t_lpx_p
												 // 	 Bit[6]: First bit, Change clk_lane first bit, 0: Output 0x55, 1: Output 0xAA
												 // 	 Bit[5:0]: wkup_dly, 1 ms wakeup delay/4096 for MIPI ultra low power resume
	//0x360F DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL10          (0x3610) // rw - Bit[7:0]: pclk_period, pclk2x period, 1-bit decimal, unit ns
#define OV8810_MIPI_CTRL11          (0x3611) // rw - Bit[7:4]: T_lpx, SCLK domain
												 // 	 Bit[3:0]: T_clk_pre, unit pclk2x cycle
	//0x3612 DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL13          (0x3613) // rw - MIPI Control 13 - MIPI Timing Register Select(0: auto calculated parameter, relate to register
												 // 									0x3622 to 0x3637; 1: use low byte of parameter)
												 // 	 Bit[7]: clk_zero_sel, 0: Auto calculate t_clk_zero, 1: Use t_clk_zero_reg
												 // 	 Bit[6]: hs_trail_sel, 0: Auto calculate t_hs_trail, 1: Use t_hs_trail_reg
												 // 	 Bit[5]: hs_zero_sel, 0: Auto calculate t_hs_zero, 1: Use t_hs_zero_reg
												 // 	 Bit[4]: hs_exit_sel, 0: Auto calculate t_hs_exit, 1: Use t_hs_exit_reg
												 // 	 Bit[3]: clk_trail_sel, 0: Auto calculate t_clk_trail, 1: Use t_clk_trail_reg
												 // 	 Bit[2]: clk_post_sel, 0: Auto calculate t_clk_post, 1: Use t_clk_post_reg
												 // 	 Bit[1]: clk_prepare_sel, 0: Auto calculate t_clk_prepare, 1: Use t_clk_prepare_reg
												 // 	 Bit[0]: hs_prepare_sel, 0: Auto calculate t_hs_prepare, 1: Use t_hs_prepare_reg
#define OV8810_MIPI_CTRL14          (0x3614) // rw - Bit[7:2]: N �� UI for t_hs_zero, Minimum high speed zero
												 // 	 Bit[1:0]: Min_hs_zero_high[9:8], Unit is pclk2x cycles when hs_zero_sel = 1 and unit is ns when hs_zero_sel = 0
#define OV8810_MIPI_CTRL15          (0x3615) // rw - Bit[7:0]: Min_hs_zero_low[7:0]
#define OV8810_MIPI_CTRL16          (0x3616) // rw - Bit[7:2]: N �� UI for t_hs_trail, Minimum high speed trail
												 // 	 Bit[1:0]: Min_hs_trail_high[9:8], Unit is pclk2x cycles when hs_trail_sel = 1 and ns when hs_trail_sel = 0

#define OV8810_MIPI_CTRL17          (0x3617) // rw - Bit[7:0]: Min_hs_trail_low[7:0]
#define OV8810_MIPI_CTRL18          (0x3618) // rw - Bit[7:2]: N �� UI for t_clk_zero, Minimum clock zero
												 // 	 Bit[1:0]: Min_clk_zero_high[9:8], Unit is pclk2x cycles when clk_zero_sel = 1 and ns when clk_zero_sel = 0
#define OV8810_MIPI_CTRL19          (0x3619) // rw - Bit[7:0]: Min_clk_zero_low[7:0]
#define OV8810_MIPI_CTRL1A          (0x361A) // rw - Bit[7:2]: N �� UI for min_clk_pre Minimum clock prepare time
												 // 	 Bit[1:0]: Min_clk_pre_high[9:8], Unit is pclk2x cycles when clk_prepare_sel = 1 and ns when clk_prepare_sel = 0
#define OV8810_MIPI_CTRL1B          (0x361B) // rw - Bit[7:0]: Min_clk_pre_low[7:0]
#define OV8810_MIPI_CTRL1C          (0x361C) // rw - Bit[7:2]: N �� UI for max_clk_pre Maximum clock prepare time
												 // 	 Bit[1:0]: Max_clk_pre_high[9:8], Unit is ns

#define OV8810_MIPI_CTRL1D          (0x361D) // rw - Bit[7:0]: Max_clk_pre_low[7:0]
#define OV8810_MIPI_CTRL1E          (0x361E) // rw - Bit[7:2]: N �� UI for min_clk_post, Minimum clock post time
												 // 	 Bit[1:0]: Min_clk_post_high[9:8], Unit is pclk2x cycles when clk_post_sel = 1 and ns when clk_post_sel = 0
#define OV8810_MIPI_CTRL1F          (0x361F) // rw - Bit[7:0]: Min_clk_post_low[9:8]
#define OV8810_MIPI_CTRL20          (0x3620) // rw - Bit[7:2]: N �� UI for min_clk_trail, Minimum clock trail
												 // 	 Bit[1:0]: Min_clk_trail_high[9:8], Unit is pclk2x cycles when clk_trail_sel = 1 and ns when clk_trail_sel = 0
#define OV8810_MIPI_CTRL21          (0x3621) // rw - Bit[7:0]: Min_clk_trail_low[7:0]
#define OV8810_MIPI_CTRL22          (0x3622) // rw - Bit[7:2]: N �� UI for min_lpx_p
												 // 	 Bit[1:0]: Min_lpx_p_high[9:8], Unit is pclk2x cycles when lpx_p_sel = 1 and ns when lpx_p_sel = 0
#define OV8810_MIPI_CTRL23          (0x3623) // rw - Bit[7:0]: Min_lpx_p_low[7:0]
#define OV8810_MIPI_CTRL24          (0x3624) // rw - Bit[7:2]: N �� UI for min_hs_prepare
												 // 	 Bit[1:0]: Min_hs_prepare_high[9:8], Unit is pclk2x cycles when hs_prepare_sel = 1 and ns when hs_prepare_sel = 0
#define OV8810_MIPI_CTRL25          (0x3625) // rw - Bit[7:0]: Min_hs_prepare_low[7:0]
#define OV8810_MIPI_CTRL26          (0x3626) // rw - Bit[7:2]: N �� UI for max_hs_prepare
												 // 	 Bit[1:0]: Max_hs_prepare_high[9:8], Unit is ns
#define OV8810_MIPI_CTRL27          (0x3627) // rw - Bit[7:0]: Max_hs_prepare_low[7:0]
#define OV8810_MIPI_CTRL28          (0x3628) // rw - Bit[7:2]: N �� UI for min_hs_exit
												 // 	 Bit[1:0]: Min_hs_exit_high[9:8], Unit is pclk2x cycles when hs_exit_sel = 1 and ns when hs_exit_sel = 0
#define OV8810_MIPI_CTRL29          (0x3629) // rw - Bit[7:0]: Max_hs_exit_low[7:0]
	//0x362A~0x362D  DEBUG MODE �C �C Debug Mode
#define OV8810_MIPI_CTRL2F          (0x362F) // rw - Bit[7:0]: t_ta_sure
#define OV8810_MIPI_CTRL30          (0x3630) // rw - Bit[7:0]: t_ta_get


	/*****************************************************************************
	 * ISP control registers
	 *****************************************************************************/
#define OV8810_TOP0_DEFAULT             	  (0xEF) //  (0xFF)   (0xEF)
#define OV8810_TOP1_DEFAULT             	  (0x0b) //  (0x08)   (0x0B)
#define OV8810_TOP2_DEFAULT             	  (0x20) //  (0x00)   (0x20)
#define OV8810_CBAR_DEFAULT                   (0x80) //  (0x80)   (0x)
#define OV8810_TOP4_DEFAULT           		  (0x05) //  (0x05)   (0x)
#define OV8810_0x3306_DEFAULT				  (0x00) //		    (0x00)
#define OV8810_BLC_DEFAULT           		  (0x33) //	(0x33)   (0x)
#define OV8810_DIG_GAIN_DEFAULT               (0x40) //	(0x40)   (0x) //refer to datasheet
#define OV8810_WBC_A_DEFAULT           		  (0x3F) //	(0x3F)   (0x)
#define OV8810_WBC_B_DEFAULT				  (0x20) //	(0x20)   (0x)
#define OV8810_WBC_C_DEFAULT				  (0x40) //	(0x40)   (0x)
#define OV8810_WBC_D_DEFAULT				  (0x30) //	(0x30)   (0x)
#define OV8810_TOP_330E_DEFAULT				  (0x00) //	(0x00)   (0x)
#define OV8810_TOP_330F_DEFAULT				  (0x03) //	(0x03)   (0x)
#define OV8810_0x3313_DEFAULT				  (0xF0) //		    (0xF0)
#define OV8810_SIZE_6_DEFAULT				  (0x03) //	(0x00)   (0x03)
#define OV8810_SIZE_7_DEFAULT				  (0x00) //	(0x00)   (0x)
#define OV8810_SIZE_8_DEFAULT				  (0x00) //	(0x00)   (0x)
#define OV8810_SIZE_9_DEFAULT				  (0x04) //	(0x00)   (0x04)
#define OV8810_SIZE_A_DEFAULT				  (0x00) //	(0x00)   (0x)
#define OV8810_SIZE_B_DEFAULT				  (0x00) //	(0x00)   (0x)
	//0x331C~0x331F AVG - RW Frame Average Control. No default value in datasheet, but evaluated by text file.
#define OV8810_AVG_E_DEFAULT				  (0x94) //		    (0x94)
#define OV8810_AVG_F_DEFAULT				  (0x6e) //		    (0x6E)
#define OV8810_AWB0_DEFAULT					  (0x82) //	(0x82)   (0x82)
#define OV8810_AWB1_DEFAULT					  (0x02) //	(0x04)   (0x02)
#define OV8810_AWB2_DEFAULT					  (0x04) //	(0x08)   (0x04)
#define OV8810_AWB3_DEFAULT					  (0x00) //	(0x00)   (0x)
#define OV8810_AWB4_DEFAULT					  (0x40) //	(0x40)   (0x)
#define OV8810_AWB5_DEFAULT					  (0x40) //	(0x40)   (0x)
#define OV8810_AWB6_DEFAULT					  (0x40) //	(0x40)   (0x)
#define OV8810_AWB7_DEFAULT					  (0x00) //	(0x00)   (0x)
#define OV8810_AWB8_DEFAULT					  (0x40) //	(0x00)   (0x40)
#define OV8810_WBC_29_DEFAULT				  (0xe3) //	(0x03)   (0xE3)
#define OV8810_WBC_2A_DEFAULT				  (0x20) //	(0x20)   (0x)
#define OV8810_WBC_2B_DEFAULT				  (0x31) //	(0x31)   (0x)
#define OV8810_WBC_2C_DEFAULT				  (0x03) //	(0x03)   (0x)
#define OV8810_WBC_2D_DEFAULT				  (0x02) //	(0x02)   (0x)
#define OV8810_BLC0_DEFAULT					  (0x00) //there is no default in datasheet
#define OV8810_BLC1_DEFAULT					  (0x20) //	(0x10)   (0x20)
#define OV8810_BLC2_DEFAULT					  (0x20) //	(0x10)   (0x20)
#define OV8810_BLC3_DEFAULT					  (0x41) //	(0x02)   (0x41)
#define OV8810_BLC4_DEFAULT					  (0x02) //	(0x00)   (0x02)
#define OV8810_BLC_MAN0_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN1_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN2_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN3_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN4_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN5_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN6_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN7_DEFAULT               (0x00) //  (0x00)   (0x)
#define OV8810_BLC_MAN0_1_DEFAULT			  (0x00) //	(0x00)   (0x00)
#define OV8810_BLC_MAN2_3_DEFAULT			  (0x00) //	(0x00)   (0x)
#define OV8810_BLC_MAN5_6_DEFAULT			  (0x00) //	(0x00)   (0x)
#define OV8810_BLC_MAN6_7_DEFAULT			  (0x00) //	(0x00)   (0x)
#define OV8810_BLC_AVG_DEFAULT				  (0x00) //	(0x06)   (0x00)    (0x00)
#define OV8810_LENC_DEFAULT					  (0x02) //	(0x02)   (0x)
#define OV8810_VAP5_DEFAULT					  (0x00) //	(0x00)   (0x00)
#define OV8810_VAP6_DEFAULT					  (0x00) //	(0x00)   (0x)
#define OV8810_VAP_H_DEFAULT				  (0x00) //	(0x00)   (0x)
#define OV8810_ISP_TEST_DEFAULT               (0x00)

	/*****************************************************************************
	 * DVP control registers
	 *****************************************************************************/
#define OV8810_DVP_CTRL00_DEFAULT             (0x80) //  (0x80)   (0x)
#define OV8810_DVP_CTRL01_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL02_DEFAULT             (0xAB) //  (0xAB)   (0x)
#define OV8810_DVP_CTRL03_DEFAULT             (0xB6) //  (0xB6)   (0x)
#define OV8810_DVP_CTRL04_DEFAULT             (0x80) //  (0x80)   (0x)
#define OV8810_DVP_CTRL05_DEFAULT             (0x9D) //  (0x9D)   (0x)
#define OV8810_DVP_CTRL06_DEFAULT             (0x20) //  (0x20)   (0x)
#define OV8810_DVP_CTRL07_DEFAULT             (0x80) //  (0x80)   (0x)
#define OV8810_DVP_CTRL08_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL09_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL0A_DEFAULT             (0x01) //  (0x01)   (0x)
#define OV8810_DVP_CTRL0B_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL0C_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL0D_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL0E_DEFAULT             (0x40) //  (0x40)   (0x)
#define OV8810_DVP_CTRL0F_DEFAULT             (0x88) //  (0x88)   (0x)
#define OV8810_DVP_CTRL10_DEFAULT             (0x09) //  (0x09)   (0x)
#define OV8810_DVP_CTRL11_DEFAULT             (0xAA) //  (0xAA)   (0x)
#define OV8810_DVP_CTRL12_DEFAULT             (0x55) //  (0x55)   (0x)
#define OV8810_DVP_CTRL13_DEFAULT             (0x02) //  (0x01)   (0x)
#define OV8810_DVP_CTRL14_DEFAULT             (0x00) //  (0x00)   (0x)
#define OV8810_DVP_CTRL15_DEFAULT             (0x00) //  (0x00)   (0x)

	/*****************************************************************************
	 * MIPI control registers
	 *****************************************************************************/
#define OV8810_MIPI_CTRL00_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL01_DEFAULT            (0x10) //  (0x10)   (0x)
#define OV8810_MIPI_CTRL02_DEFAULT            (0x12) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL03_DEFAULT            (0x2A) //  (0x2A)   (0x)
#define OV8810_MIPI_CTRL04_DEFAULT            (0x58) //  (0x58)   (0x)
#define OV8810_MIPI_CTRL05_DEFAULT            (0x00) //	(0x00)   (0x)
#define OV8810_MIPI_CTRL06_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL07_DEFAULT            (0x68) //  (0x68)   (0x)
#define OV8810_MIPI_CTRL0A_DEFAULT            (0x01) //	(0x01)   (0x)
#define OV8810_MIPI_CTRL0B_DEFAULT            (0x3C) //	(0x3C)   (0x)
#define OV8810_MIPI_CTRL0C_DEFAULT            (0x03) //	(0x03)   (0x)
#define OV8810_MIPI_CTRL0E_DEFAULT            (0x02) //	(0x02)   (0x)
#define OV8810_MIPI_CTRL10_DEFAULT            (0x10) //  (0x10)   (0x)
#define OV8810_MIPI_CTRL11_DEFAULT            (0x4F) //  (0x4F)   (0x)
#define OV8810_MIPI_CTRL13_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL14_DEFAULT            (0x28) //  (0x28)   (0x)
#define OV8810_MIPI_CTRL15_DEFAULT            (0x96) //  (0x96)   (0x)
#define OV8810_MIPI_CTRL16_DEFAULT            (0x10) //  (0x10)   (0x)
#define OV8810_MIPI_CTRL17_DEFAULT            (0x3C) //  (0x3C)   (0x)
#define OV8810_MIPI_CTRL18_DEFAULT            (0x01) //  (0x01)   (0x)
#define OV8810_MIPI_CTRL19_DEFAULT            (0x86) //  (0x86)   (0x)
#define OV8810_MIPI_CTRL1A_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL1B_DEFAULT            (0x32) //  (0x32)   (0x)
#define OV8810_MIPI_CTRL1C_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL1D_DEFAULT            (0x8C) //  (0x8C)   (0x)
#define OV8810_MIPI_CTRL1E_DEFAULT            (0xD0) //  (0xD0)   (0x)
#define OV8810_MIPI_CTRL1F_DEFAULT            (0x56) //  (0x56)   (0x)
#define OV8810_MIPI_CTRL20_DEFAULT            (0x08) //  (0x08)   (0x)
#define OV8810_MIPI_CTRL21_DEFAULT            (0x3C) //  (0x3C)   (0x)
#define OV8810_MIPI_CTRL22_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL23_DEFAULT            (0x32) //  (0x32)   (0x)
#define OV8810_MIPI_CTRL24_DEFAULT            (0x10) //  (0x10)   (0x)
#define OV8810_MIPI_CTRL25_DEFAULT            (0x2A) //  (0x2A)   (0x)
#define OV8810_MIPI_CTRL26_DEFAULT            (0x18) //  (0x18)   (0x)
#define OV8810_MIPI_CTRL27_DEFAULT            (0x55) //  (0x55)   (0x)
#define OV8810_MIPI_CTRL28_DEFAULT            (0x00) //  (0x00)   (0x)
#define OV8810_MIPI_CTRL29_DEFAULT            (0x64) //  (0x64)   (0x)
#define OV8810_MIPI_CTRL2F_DEFAULT            (0x08) //  (0x08)   (0x)
#define OV8810_MIPI_CTRL30_DEFAULT            (0x14) //  (0x14)   (0x)

	/*****************************************************************************
	 * Further defines for driver management
	 *****************************************************************************/
#define OV8810_DRIVER_INIT              (0x00000001)






typedef struct OV8810_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    //// modify below here ////

    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that csensor is streaming data */

    float               VtPixClkFreq;           /**< pixel clock */
    uint16_t            LineLengthPck;          /**< line length with blanking */
    uint16_t            FrameLengthLines;       /**< frame line length */

    float               AecMinGain;
    float               AecMaxGain;
    float               AecMinIntegrationTime;
    float               AecMaxIntegrationTime;

    float               AecCurGain;
    float               AecCurIntegrationTime;

    uint32_t            OldIntegrationTime;
    uint32_t            OldCoarseIntegrationTime;
	uint32_t			preview_minimum_framerate;

} OV8810_Context_t;



#ifdef __cplusplus
}
#endif

/* @} ov8810_priv */

#endif /* __OV8810_PRIV_H__ */


