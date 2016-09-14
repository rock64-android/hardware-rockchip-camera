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
 * @file isi_iss.h
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
 * @defgroup ov5630_priv
 * @{
 *
 */
#ifndef __OV5630_PRIV_H__
#define __OV5630_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>



#ifdef __cplusplus
extern "C"
{
#endif

/*v0.1.0:
*   1).add senosr drv version in get sensor i2c info func
*v0.2.0
*   1). support for isi v0.0xc.0
*   2). change VPol from ISI_VPOL_NEG to ISI_VPOL_POS
*v0.3.0
*   1). support isi v0.0xd.0
*/
#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 3, 0) 


/*****************************************************************************
 * System control registers
 *****************************************************************************/
#define OV5630_AGCL                 (0x3000) // rw - Sensor Gain, or Sensor Long Exposure Gain, when used in HDR mode auto or manual
#define OV5630_AGCS                 (0x3001) // rw - Sensor Short Exposure Gain, only used in HDR mode auto or manual
#define OV5630_AECL_2               (0x3002) // rw - Coarse Exposure Time, Auto or Manual, Units: Lines, Byte: High
#define OV5630_AECL_1               (0x3003) // rw - Coarse Exposure Time, Auto or Manual, Units: Lines, Byte: Low
#define OV5630_LAEC_2               (0x3004) // rw - Fine Exposure Time, Auto or Manual, Units: System clocks, Byte: High
#define OV5630_LAEC_1               (0x3005) // rw - Fine Exposure Time, Auto or Manual, Units: System clocks, Byte: Low
#define OV5630_RSVD_0x3006          (0x3006) // rw - Bit[0] Digital Gain Stage x2 on/off
// (0x3007) reserved
#define OV5630_AECS_2               (0x3008) // rw - Coarse Exposure Time, HDR Auto or Manual, Units: Lines, Byte: High
#define OV5630_AECS_1               (0x3009) // rw - Coarse Exposure Time, HDR Auto or Manual, Units: Lines, Byte: Low
#define OV5630_PIDH                 (0x300A) // rw - ID, Byte: High
#define OV5630_PIDL                 (0x300B) // rw - ID, Byte: Low
#define OV5630_SCCB_ID              (0x300C) // rw - SCCB ID
// (0x300D) reserved
#define OV5630_R_PLL1               (0x300E) // rw - Bit[7:6]: PLL pre divider, 00:1, 01:1.5, 10:2, 11:3
                                             //      Bit[5:0]: PLL divider, PLLDiv=64-R_PLL1[5:0]
#define OV5630_R_PLL2               (0x300F) // rw - Bit[7:4]: System clock divider, divider=R_PLL2[7:4]+1
                                             //      Bit[3:0]: MIPI divider,         divider=R_PLL2[3:0]+1
#define OV5630_R_PLL3               (0x3010) // rw - Bit[7]:   Bypass PLL
                                             //      Bit[6:4]: PLL charge pump current control
                                             //      Bit[3]:   Reserved
                                             //      Bit[2]:   LaneDiv control, LaneDiv= 0:1, 1:2
                                             //      Bit[1:0]: DIV4/5 control, Ratio= 00:1, 01:1, 10:4, 11:5
#define OV5630_R_PLL4               (0x3011) // rw - Bit[7:6]: Digital clock input divider, 00:1, 01:2, 10:4, 11:4
                                             //      Bit[5:0]: Reserved
#define OV5630_SYS                  (0x3012) // rw - System Control
                                             //      Bit[7]:   Software system reset
                                             //      Bit[6:0]: Reserved
#define OV5630_AUTO1                (0x3013) // rw - AEC Enable
                                             //      Bit[7]: AEC speed select, 0:Normal, 1:Faster AEC adjustment
                                             //      Bit[5]: Banding ON/OFF, 0:ON, 1:OFF
                                             //      Bit[4]: Banding auto ON/OFF select, 0:OFF, 1:ON
                                             //      Bit[3]: LAEC ON/OFF, 0:ON, 1:OFF
                                             //      Bit[2]: Auto gain control, 0:Manual, 1:Auto
                                             //      Bit[1]: Reserved
                                             //      Bit[0]: Auto/manual exposure control select, 0:Manual, 1:Auto
#define OV5630_AUTO2                (0x3014) // rw - Bit[7]:   Reserved
                                             //      Bit[6]:   Auto 50/60Hz banding ON/OFF, 0:ON, 1:OFF
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3]:   VAEC ON/OFF, 0:ON, 1:OFF
                                             //      Bit[2:0]: Reserved
#define OV5630_AUTO3                (0x3015) // rw - High Gain Limit
                                             //      Bit[7]:   Reserved
                                             //      Bit[6:4]: VAEC ceiling, number of frames: 000:1, 001:1.5, 010:2, 011:3, 100:4, 101:6, 110:8, 111:12
                                             //      Bit[3]:   Reserved
                                             //      Bit[2:0]: Maximum gain, 001:4x, 010:8x, 011:16x, 100:32x, 101-111:Reserved
#define OV5630_AUTO4                (0x3016) // rw - Bit[7]:   LGOPT
                                             //      Bit[6]:   T7_opt
                                             //      Bit[5:4]: HLGopt
                                             //      Bit[3]:   AddVS_opt
                                             //      Bit[2]:   Reserved
                                             //      Bit[1:0]: sub_exp_x2
#define OV5630_AUTO5                (0x3017) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: RAECG
#define OV5630_WPT                  (0x3018) // rw - Luminance Signal/High Range for AEC/AGC operation.
                                             //      AEC/AGC value decreases in auto mode, when average luminance is greater than WPT[7:0].
#define OV5630_BPT                  (0x3019) // rw - Luminance Signal/ Low Range for AEC/AGC operation.
                                             //      AEC/AGC value increases in auto mode when average luminance is less than BPT[7:0].
#define OV5630_VPT                  (0x301A) // rw - Fast Mode Large Step Range Thresholds, effective only in AEC/AGC fast mode
                                             //      Bit[7:4]: High threshold
                                             //      Bit[3:0]: Low threshold
                                             //      AEC/AGC may change in larger steps, when luminance average is greater than (VPT[7:4],4b0 ??)
                                             //                                                                    or less than (VPT[3:0], 4b0 ??)
#define OV5630_YAVG                 (0x301B) // ro - Luminance Average - this register will auto update.
                                             //      Average luminance is calculated from the B/Gb/Gr/R channel average as follows:
                                             //      B/Gb/Gr/R channel average = (BAVG[7:0] + GbAVG[7:0] + GrAVG[7:0] + RAVG[7:0]) x 0.25
#define OV5630_AECG_MAX50           (0x301C) // rw - 50 Hz Smooth Banding Maximum Steps Control
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5:0]: maximum band for 50 Hz in terms of row exposure
#define OV5630_AECG_MAX60           (0x301D) // rw - 60 Hz Smooth Banding Maximum Steps Control
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5:0]: maximum band for 60 Hz in terms of row exposure
#define OV5630_ADDVS_2              (0x301E) // rw - Extra VSYNC Pulse Width, Units: Lines, Byte: High
#define OV5630_ADDVS_1              (0x301F) // rw - Extra VSYNC Pulse Width, Units: Lines, Byte: Low
#define OV5630_FRAME_LENGTH_LINES_2 (0x3020) // rw - Frame Length, Units: Lines, Byte: High
#define OV5630_FRAME_LENGTH_LINES_1 (0x3021) // rw - Frame Length, Units: Lines, Byte: Low
#define OV5630_LINE_LENGTH_PCK_2    (0x3022) // rw - Line Length, Units: System clocks, Byte: High
#define OV5630_LINE_LENGTH_PCK_1    (0x3023) // rw - Line Length, Units: System clocks, Byte: Low
#define OV5630_X_ADDR_START_2       (0x3024) // rw - X Address of the Top Left Corner of the Visible Pixels, Units: Pixels, Byte: High
#define OV5630_X_ADDR_START_1       (0x3025) // rw - X Address of the Top Left Corner of the Visible Pixels, Units: Pixels, Byte: Low
#define OV5630_Y_ADDR_START_2       (0x3026) // rw - Y Address of the Top Left Corner of the Visible Pixels, Units: Lines, Byte: High
#define OV5630_Y_ADDR_START_1       (0x3027) // rw - Y Address of the Top Left Corner of the Visible Pixels, Units: Lines, Byte: Low
#define OV5630_X_ADDR_END_2         (0x3028) // rw - X Address of the Bottom Right Corner of the Visible Pixels, Units: Pixels, Byte: High
#define OV5630_X_ADDR_END_1         (0x3029) // rw - X Address of the Bottom Right Corner of the Visible Pixels, Units: Pixels, Byte: Low
#define OV5630_Y_ADDR_END_2         (0x302A) // rw - Y Address of the Bottom Right Corner of the Visible Pixels, Units: Lines, Byte: High
#define OV5630_Y_ADDR_END_1         (0x302B) // rw - Y Address of the Bottom Right Corner of the Visible Pixels, Units: Lines, Byte: Low
#define OV5630_X_OUTPUTSIZE_2       (0x302C) // rw - Width of Image Data Output from the Sensor, Units: Pixels, Byte: High
#define OV5630_X_OUTPUTSIZE_1       (0x302D) // rw - Width of Image Data Output from the Sensor, Units: Pixels, Byte: Low
#define OV5630_Y_OUTPUTSIZE_2       (0x302E) // rw - Height of Image Data Output from the Sensor, Units: Pixels, Byte: High
#define OV5630_Y_OUTPUTSIZE_1       (0x302F) // rw - Height of Image Data Output from the Sensor, Units: Pixels, Byte: Low
#define OV5630_FRAME_CNT            (0x3030) // ro - Frame Count
// (0x3031 - 0x3037) reserved
#define OV5630_DATR_LMO_3           (0x3038) // ro - Sigma5060 LMO[7:0]
#define OV5630_DATR_LMO_2           (0x3039) // ro - Sigma5060 LMO[15:8]
#define OV5630_DATR_LMO_1           (0x303A) // ro - Bit[7:4]: Reserved
                                             //      Bit[3:0]: Sigma5060 LMO[19:16]
// (0x303B - 0x303C) reserved
#define OV5630_DATR_D56             (0x303D) // ro - Sigma5060 Register
#define OV5630_DATR_EF              (0x303E) // ro - OTP read/write control, 0x55: Read from OTP memory, 0xAA: Write to OTP memory
// (0x303F - 0x3047) reserved
#define OV5630_R_SIGMA_6            (0x3048) // rw - R_sigma[7:0]
#define OV5630_R_SIGMA_5            (0x3049) // rw - R_sigma[15:8]
#define OV5630_R_SIGMA_4            (0x304A) // rw - R_sigma[23:16]
#define OV5630_R_SIGMA_3            (0x304B) // rw - R_sigma[31:24]
#define OV5630_R_SIGMA_2            (0x304C) // rw - R_sigma[39:32]
#define OV5630_R_SIGMA_1            (0x304D) // rw - R_sigma[47:40]
#define OV5630_D56COM               (0x304E) // rw - Sigma5060 Clock Control
                                             //      Bit[7]:   Clock source select, 0: CLK_SCCB, 1: DSPCLK
                                             //      Bit[6]:   5060 auto threshold enable
                                             //      Bit[5]:   Enable 5060 detection every 4 seconds
                                             //      Bit[4:0]: Clock divider
#define OV5630_RSVD_0x304F          (0x304F) // reserved but written by Omnivision evaluation kit
#define OV5630_R5060TH              (0x3050) // ro - Read 5060 Auto Threshold Value
// (0x3051 - 0x3057) reserved
#define OV5630_LMO_TH1              (0x3058) // rw - Light Meter Output, Lowest Value for D5060 Auto Threshold Curve, Range from 1 to 7
#define OV5630_LMO_TH2              (0x3059) // rw - Light Meter Output, Highest Value for D5060 Auto Threshold Curve, Range from 0x16 to 0x40
#define OV5630_LMO_K                (0x305A) // rw - Bit[7:5]: Reserved
                                             //      Bit[4:0]: Light Meter High Threshold. Range from 0x16 to 0x40. Range from 5'b00000 to 5'b11111
// (0x305B) reserved
#define OV5630_BD50ST_2             (0x305C) // rw - Band Step for 50 Hz, Byte: High
#define OV5630_BD50ST_1             (0x305D) // rw - Band Step for 50 Hz, Byte: Low
#define OV5630_BD60ST_2             (0x305E) // rw - Band Step for 60 Hz, Byte: High
#define OV5630_BD60ST_1             (0x305F) // rw - Band Step for 60 Hz, Byte: Low
// (0x3060 - 0x3064) reserved
#define OV5630_RSVD_0x3065          (0x3065) // reserved but written by Omnivision evaluation kit
// (0x3066 - 0x3067) reserved
#define OV5630_RSVD_0x3068          (0x3068) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3069          (0x3069) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x306A          (0x306A) // reserved but written by Omnivision evaluation kit
// (0x306B - 0x306C) reserved
#define OV5630_HSYNST               (0x306D) // rw - HSYNC Start, Units: System clocks
                                             //      HSYNST[7:0]
#define OV5630_HSYNED               (0x306E) // rw - HSYNC End, Units: System clocks
                                             //      HSYNED[7:0]
#define OV5630_HSYNED_HSYNST        (0x306F) // rw - Bit[7:4]: HSYNED[11:8]
                                             //      Bit[3:0]: HSYNST[11:8]
#define OV5630_TMC_RWIN0            (0x3070) // rw - System Timing and Window Control
                                             //      Bit[7]: RSA1_sel Fixed SA1 end point
                                             //      Bit[6]: Reserved
                                             //      Bit[5]: VREF_opt2 Same number of lines as from array
                                             //      Bit[4]: VREF_opt
                                             //      Bit[3]: Reserved
                                             //      Bit[2]: HREF_opt Same number of pixels as from array
                                             //      Bit[1]: Rcsft_opt
                                             //      Bit[0]: Reserved
#define OV5630_RSVD_0x3071          (0x3071) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3072          (0x3072) // reserved but written by Omnivision evaluation kit
// (0x3073 - 0x3074) reserved
#define OV5630_RSVD_0x3075          (0x3075) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3076          (0x3076) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3077          (0x3077) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3078          (0x3078) // reserved but written by Omnivision evaluation kit
// (0x3079 - 0x3083) reserved
#define OV5630_RSVD_0x3084          (0x3084) // reserved but written by Omnivision evaluation kit
// (0x3085 - 0x3089) reserved
#define OV5630_RSVD_0x308A          (0x308A) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x308B          (0x308B) // reserved but written by Omnivision evaluation kit
// (0x308C) reserved
#define OV5630_RSVD_0x308D          (0x308D) // reserved but written by Omnivision evaluation kit
// (0x308E - 0x308F) reserved
#define OV5630_RSVD_0x3090          (0x3090) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3091          (0x3091) // reserved but written by Omnivision evaluation kit
// (0x3092 - 0x3097) reserved
#define OV5630_RSVD_0x3098          (0x3098) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x3099          (0x3099) // reserved but written by Omnivision evaluation kit
// (0x309A - 0x309C) reserved
#define OV5630_RSVD_0x309D          (0x309D) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x309E          (0x309E) // reserved but written by Omnivision evaluation kit
// (0x309F - 0x30A0) reserved
#define OV5630_RSVD_0x30A1          (0x30A1) // reserved but written by Omnivision evaluation kit
// (0x30A2 - 0x30AB) reserved
#define OV5630_RSVD_0x30AC          (0x30AC) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x30AD          (0x30AD) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x30AE          (0x30AE) // reserved but written by Omnivision evaluation kit
#define OV5630_RSVD_0x30AF          (0x30AF) // reserved but written by Omnivision evaluation kit
#define OV5630_IO_CTRL0             (0x30B0) // rw - Enable of Second Camera Interface
                                             //      CY[7:0]
#define OV5630_IO_CTRL1             (0x30B1) // rw - Enable of Second Camera Interface
                                             //      Bit[7:6]: Reserved
                                             //      Bit[5]:   C_VSYNC
                                             //      Bit[4]:   C_STROBE
                                             //      Bit[3]:   C_PCLK
                                             //      Bit[2]:   C_HREF
                                             //      Bit[1:0]: CY[9:8]
#define OV5630_IO_CTRL2             (0x30B2) // rw - Enable of Second Camera Interface
                                             //      Bit[7:5]: Reserved
                                             //      Bit[4]:   C_FREX
                                             //      Bit[3:0]: R_PAD[3:0]
#define OV5630_DSIO_3               (0x30B3) // rw - DSIO[7:0]
                                             //      Bit[7]: Reserved
                                             //      Bit[6]:   Rpclkinv, Invert OP_PCLK
                                             //      Bit[5:4]: Rpclksw, Switch PCLK and OP_PCLK
                                             //      Bit[3]:   RPCKman, 0:Division from ISP subsample, 1:Manual control use RPCLKdiv
                                             //      Bit[2]:   Reserved
                                             //      Bit[1:0]: RPCLKdiv, PCLK divisor, 00:1, 01:2, 10:4, 11:8
#define OV5630_DSIO_2               (0x30B4) // rw - DSIO[15:8]
                                             //      Bit[7:6]: Rtest, 00:IMG to PAD, 01:Mix signals to PAD, 10:Array address and timing to PAD, 11:Array timing to PAD
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3]:   RdspblueLat
                                             //      Bit[2]:   OTP_TST
                                             //      Bit[1:0]: OTP_SCK_OPT
#define OV5630_DSIO_1               (0x30B5) // rw - DSIO[23:16]
                                             //      Bit[7]:   RmirrCKINV
                                             //      Bit[6]:   Rnewt
                                             //      Bit[5:4]: long_exp_opt[1:0]
                                             //      Bit[3]:   VSLatopt
                                             //      Bit[2]:   VSLatdly
                                             //      Bit[1]:   HDR_en
                                             //      Bit[0]:   RHDR_fix gain
#define OV5630_TMC_TMC10            (0x30B6) // rw - Bit[7:6]: Reserved
                                             //      Bit[5]:   Rposispck
                                             //      Bit[4]:   RframeENhi
                                             //      Bit[3]:   Tri_B
                                             //      Bit[2]:   RselNewslp
                                             //      Bit[1:0]: Reserved
#define OV5630_TMC_TMC12            (0x30B7) // rw - Bit[7]:   dis_ISP_rw, Disable RW of registers from ISP/MIPI
                                             //      Bit[6:4]: Reserved
                                             //      Bit[3]:   Rgrp_wr_en, Enable group write
                                             //      Bit[2]:   grp_i2c2uc_wr
                                             //      Bit[1]:   Reserved
                                             //      Bit[0]:   Ri2c_isp_wrB
// (0x30B8) reserved
#define OV5630_TMC_TMC14            (0x30B9) // rw - Bit[7]:   T_RWopt_buf
                                             //      Bit[6:4]: Reserved
                                             //      Bit[3]:   RuclatchB
                                             //      Bit[2]:   Reserved
                                             //      Bit[1]:   RmipiRegrst, Enable the MIPI sys reset to clear register
                                             //      Bit[0]:   RmipirstEn, Enable the reset control from MIPI
#define OV5630_TMC_COM4             (0x30BA) // rw - Bit[7]: RANAdsb
                                             //      Bit[6]: RPLLdsb
                                             //      Bit[5]: RCCP2dsb
                                             //      Bit[4]: RADCdsb
                                             //      Bit[3]: RDSPBLUE_opt1
                                             //      Bit[2]: RDSPBLUE_opt0
                                             //      Bit[1]: RDSPdsb
                                             //      Bit[0]: Ri2cPLLrsen
#define OV5630_TMC_REG6C            (0x30BB) // rw - Drop Frame Type Control
                                             //      Bit[7]:   ARBLUE_inv
                                             //      Bit[6]:   VFLIP_DropF_EN
                                             //      Bit[5]:   QVGA_DropF_EN
                                             //      Bit[4]:   fl_DropF_EN
                                             //      Bit[3]:   fl_DropF_WR_EN
                                             //      Bit[2:1]: Reserved
                                             //      Bit[0]:   y_addr_DropF_dis
#define OV5630_TMC_REG6E            (0x30BC) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:4]: Fmaxopt
                                             //      Bit[3]:   Lminopt
                                             //      Bit[2]:   RHDRLaec
                                             //      Bit[1]:   Reserved
                                             //      Bit[0]:   Glatch_opt
#define OV5630_R_CLK_RSCLK          (0x30BD) // rw - System Clock Control
                                             //      Bit[7]:   REQ50ph2, Switch sync PH1 and PH2 for analog clock
                                             //      Bit[6]:   RPCK delay
                                             //      Bit[5:4]: RPCK, Switch source of PCLK, 00:CLK_IN, 01:CLK_IN inverted, 10:PH2, 11:PH1
                                             //      Bit[3:2]: isp_i2c_clk option
                                             //      Bit[1:0]: RPHGAP, Gap between PH1 and PH2, 00:1ns + transfer delay, 01:2ns, 10:3ns, 11:0ns
#define OV5630_R_CLK_RACLK          (0x30BE) // rw - Analog Clock Control
                                             //      Bit[7:6]: CLKNrate, PUMP N derating, 00:(1:1), 01:(1:2), 10:(1:4), 11:(1:8)
                                             //      Bit[5:4]: CLKPrate, PUMP P derating, 00:(1:1), 01:(1:2), 10:(1:4), 11:(1:8)
                                             //      Bit[3]:   PUMPxclk, Switch between PCLK and CLK_SCCB
                                             //      Bit[2]:   RADCIFree, ADC clock digital free running
                                             //      Bit[1]:   RADCFree, ADC clock free running
                                             //      Bit[0]:   RADRFree, Array clock free running
#define OV5630_R_CLK_RACLK1         (0x30BF) // rw - Analog Clock Control 1
                                             //      Bit[7]:   RADRinv, Invert array clock
                                             //      Bit[6:4]: RADRdly, Array clock delay, 000:0ns, 001:2ns, 010:4ns, 011:6ns, 100:8ns, 101:10ns, 110:12ns, 111:14ns
                                             //      Bit[3]:   RADAinv, Invert analog clock
                                             //      Bit[2:0]: RADAdly, Analog clock delay, 000:0ns, 001:2ns, 010:4ns, 011:6ns, 100:8ns, 101:10ns, 110:12ns, 111:14ns
// (0x30C0 - 0x30CF) reserved
// (0x30D0 - 0x30DF) Data to be Programmed/Read Into/From OTP for Customer
#define OV5630_FRS_RFRES0           (0x30E0) // rw - Frame/Flash Exposure Control
                                             //      Bit[7]: RflwFX
                                             //      Bit[6]: FRSdly
                                             //      Bit[5]: Fdly10
                                             //      Bit[4]: FLRS
                                             //      Bit[3]: FnoSA1
                                             //      Bit[2]: FTXsep
                                             //      Bit[1]: FRSTsep
                                             //      Bit[0]: FTXdly
#define OV5630_FRS_RFRES1           (0x30E1) // rw - Bit[7:5]: AFTX_D
                                             //      Bit[4]:   Reserved
                                             //      Bit[3]:   TXHEN
                                             //      Bit[2]:   MFRMCLR
                                             //      Bit[1]:   Reserved
                                             //      Bit[0]:   FRMCLREN
#define OV5630_FRS_RFRES2           (0x30E2) // rw - Bit[7]:   FE1PIN
                                             //      Bit[6]:   sel_ADD_all
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3:2]: RFRSlag
                                             //      Bit[1:0]: Fpul
#define OV5630_FRS_RFRES3           (0x30E3) // rw - Bit[7:2]: Reserved
                                             //      Bit[1]:   Rpre2df
                                             //      Bit[0]:   REQopt
#define OV5630_FRS_FECNT            (0x30E4) // rw - FECNT
#define OV5630_FRS_FFCNT_2          (0x30E5) // rw - FFCNT[7:0]
#define OV5630_FRS_FFCNT_1          (0x30E6) // rw - FFCNT[15:0]
#define OV5630_FRS_RFRM             (0x30E7) // rw - Bit[7]:   RSTR_FREX_Sync
                                             //      Bit[6]:   RSTR_FREX
                                             //      Bit[5]:   RSTR_EDGE
                                             //      Bit[4]:   RSTR_INV
                                             //      Bit[3:1]: Reserved
                                             //      Bit[0]:   STR_EN
#define OV5630_FRS_RSTRB            (0x30E8) // rw - Bit[7]:   Strobe function enable, 0:Strobe disable, 1:Start strobe enable
                                             //      Bit[6]:   Strobe output polarity control, 0:Positive pulse, 1:Negative pulse
                                             //      Bit[5:4]: Reserved
                                             //      Bit[3:2]: Xenon mode pulse width, 00:1 line, 01:2 lines, 10:3 lines, 11:4 lines
                                             //      Bit[1:0]: Strobe mode select, 00:Xenon mode, 01:LED 1 and 2 mode, 10:LED 1 and 2 mode, 11:LED 3 mode
#define OV5630_SA1TMC               (0x30E9) // rw - Bit[7]: SWB fix
                                             //      Bit[6]: TX inverse
                                             //      Bit[5]: PCBAR_ALL
                                             //      Bit[4]: Rbsun_fix
                                             //      Bit[3]: Rshift_hi
                                             //      Bit[2]: Rbit1
                                             //      Bit[1]: RSA1EN_low
                                             //      Bit[0]: Rbsun_dis
#define OV5630_TMC_30EA             (0x30EA) // rw - Bit[7]:   Reserved
                                             //      Bit[6]:   Rjump_man
                                             //      Bit[5]:   Ryoffs_man
                                             //      Bit[4]:   Rxoffs_man
                                             //      Bit[3:0]: Reserved
#define OV5630_TMC_30EB             (0x30EB) // rw - Bit[7:4]: isp_h_offset_man[11:8], [7:0] is at 0x307B
                                             //      Bit[3]:   Reserved
                                             //      Bit[2:0]: isp_v_offset_man[10:8], [7:0] is at 0x307B
// (0x30EC - 0x30EF) reserved
#define OV5630_FLEX_REG_TXP         (0x30F0) // rw - Transmit Precharge Time
#define OV5630_FLEX_REG_FLT         (0x30F1) // rw - Charge Floating Point Time
#define OV5630_FLEX_REG_TXT         (0x30F2) // rw - Transmit Transfer Time
#define OV5630_FLEX_REG_HBK         (0x30F3) // rw - Hold Black Time
#define OV5630_FLEX_REG_HSG         (0x30F4) // rw - Hold Signal Time
#define OV5630_FLEX_SA1SFT          (0x30F5) // rw - Shift of SA1 Ending Point
#define OV5630_RVSOPT               (0x30F6) // rw - Bit[7]:   Reserved
                                             //      Bit[6]:   Width of VS pulse, 0:1-3 lines, 1:Dozens of system clocks
                                             //      Bit[5]:   VS start point, 0:Frame start, 1:Frame endtmc
                                             //      Bit[4]:   VS end point, 0:Frame start, 1:Frame end
                                             //      Bit[3:2]: VS end line
                                             //      Bit[1:0]: VS start line
#define OV5630_AUTO6                (0x30F7) // rw - Auto Control for AEC/AGC
                                             //      Bit[7]:   Rvario
                                             //      Bit[6]:   RfixR
                                             //      Bit[5]:   Reserved
                                             //      Bit[4]:   DropF_EN
                                             //      Bit[3]:   EXPNG
                                             //      Bit[2:0]: Ratio[2:0]
#define OV5630_IMAGE_TRANSFO        (0x30F8) // rw - Bit[7]:   Flip ON/OFF select, 0:Flip OFF, 1:Flip ON
                                             //      Bit[6]:   Mirror ON/OFF select, 0:Mirror OFF, 1:Mirror ON
                                             //      Bit[5]:   RISPsubV, Enable V subsample in ISP
                                             //      Bit[4]:   RISPsub, Enable H subsample in ISP
                                             //      Bit[3:2]: VSUB, 00:Full, 01:(1:2), 10:(1:4), 11:Reserved
                                             //      Bit[1:0]: HSUB, 00:Full, 01:(1:2), 10:(1:4), 11:Reserved
                                             //      Note: H subsample can be implemented in ARRAY and ISP depending on the value of RISPsub
                                             //      Note: V subsample is performed in schematic and ARRAY only
#define OV5630_IMAGE_LUM            (0x30F9) // rw - Bit[7]:   Ravglatch, Latch average, 0:Use latch sig from ISP, 1:Use latch sig from timing in blanking lines
                                             //      Bit[6]:   Reserved
                                             //      Bit[5]:   SLEEP_opt
                                             //      Bit[4]:   SLEEP_NoLatch
                                             //      Bit[3:2]: Reserved
                                             //      Bit[1]:   Vertical skip
                                             //      Bit[0]:   analog_gain_NODELAY
#define OV5630_IMAGE_SYSTEM         (0x30FA) // rw - Bit[7]:   software_reset, 0:Normal open, 1:All including I2C is reset to default, then go to stand by
                                             //      Bit[6:3]: Reserved
                                             //      Bit[2]:   mask_corrupted_frames, 0:Frames are dropped due to change of timing, size etc., 1:No frame drop
                                             //      Bit[1]:   grouped_parameter_hold for gain, integration and video timing signal,
                                             //                0:Timing critical signals are written immediately,
                                             //                1:Timing critical signals are written in blanking lines
                                             //      Bit[0]:   mode_select, 0:Software sleep/standby, 1:Streaming
// (0x30FB - 0x30FE) reserved
#define OV5630_GROUP_WR             (0x30FF) // rw - Group_write Flag Register
// (0x3100 - 0x3102) reserved
#define OV5630_RSVD_0x3103          (0x3103) // reserved but written by Omnivision evaluation kit

/*****************************************************************************
 * CIF control registers
 *****************************************************************************/
// (0x3200 - 0x3201) reserved
#define OV5630_CIF_CTRL2            (0x3202) // rw - Bit[7]: Reserved
                                             //      Bit[6]: rev_rblue, 0:Use ptn_rblue as CIF rblue input, 1:Use reversed ptn_rblue as CIF rblue input
                                             //      Bit[5]: rev_pclk, 0:Use ext_snr PCLK as CIF PCLK input, 1:Use reversed ext_snr PCLK as CIF pclk input
                                             //      Bit[4]: rev_href, 0:Use ext_snr HREF as CIF HREF input, 1:Use reversed ext_snr HREF as CIF HREF input
                                             //      Bit[3]: rev_vsync, 0:Use ext_snr VSYNC as CIF VSYNC input, 1:Use reversed  ext_snr VSYNC as CIF VSYNC input
                                             //      Bit[2]: rev_cif_rblue, 0:Use CIF ouput rblue as ISP rblue input, 1:Use reversed CIF output rblue as ISP rblue input
                                             //      Bit[1]: rev_ptn_rblue, generate ptn rblue, 0:First line of ext_snr is BG line, 1:First line of ext_snr is GR line
                                             //      Bit[0]: Reserved

/*****************************************************************************
 * ISP control registers
 *****************************************************************************/
#define OV5630_ISP_CTRL00           (0x3300) // rw - ISP enable control (corresponding clock will be stopped if one module is disabled)
                                             //      Bit[7]: white pixel cancellation enable, 0:Disable, 1:Enable
                                             //      Bit[6]: black pixel cancellation enable, 0:Disable, 1:Enable
                                             //      Bit[5]: AWB ON/OFF select, 0:Auto white balance OFF, 1:Auto white balance ON
                                             //      Bit[4]: WB gain ON/OFF select, 0:Do not apply the RGB gain to sensor, 1:Apply the R/G/B gain to sensor
                                             //      Bit[3]: digital gain enable, 0:Disable, 1:Enable
                                             //      Bit[2]: Lenc_en
                                             //      Bit[1]: Even_en (disabled if blc_en = 0)
                                             //      Bit[0]: BLC enable, 0:OFF, 1:ON
#define OV5630_ISP_CTRL01           (0x3301) // rw - Bit[7]: flip_on
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
#define OV5630_ISP_CTRL02           (0x3302) // rw - Bit[7]: Manual WB ON/OFF select, 0:Manual mode white balance gain disable, 1: Manual mode white balance gain enable
                                             //      Bit[6]: Manual digital gain enable, 0:Auto digital gain enable, 1:Manual digital gain enable
                                             //      Bit[5]: awb_bias_en
                                             //      Bit[4]: bias_man_en
                                             //      Bit[3]: sof_sel
                                             //      Bit[2]: g_first
                                             //      Bit[1]: neg_edge
                                             //      Bit[0]: ext_sensor, Get data from external sensor
#define OV5630_0x3303               (0x3303) // rw - Bit[7:5]: Reserved
                                             //      Bit[4]:   awb_long, 0:AWB can get long and short exposure data, 1:AWB can only get long exposure data
                                             //      Bit[3:2]: rblue_ctrl
                                             //      Bit[1]:   rblue_change_en, Enable rblue change at VSYNC fall edge
                                             //      Bit[0]:   rblue_change
#define OV5630_DIG_GAIN_MAN         (0x3304) // rw - digital_gain_manual
#define OV5630_BIAS_MAN             (0x3305) // rw - bias_manual
#define OV5630_0x3306               (0x3306) // rw - Bit[7]:   awb_freeze_en
                                             //      Bit[6]:   fast_awb
                                             //      Bit[5:0]: awb step size
#define OV5630_STABLE_RANGE         (0x3307) // rw - Range from Unstable to Stable, stable_range[7:0]
#define OV5630_R_GAIN_MAN_2         (0x3308) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: r_gain_man[11:8]
#define OV5630_R_GAIN_MAN_1         (0x3309) // rw - r_gain_man[7:0]
#define OV5630_G_GAIN_MAN_2         (0x330A) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: g_gain_man[11:8]
#define OV5630_G_GAIN_MAN_1         (0x330B) // rw - g_gain_man[7:0]
#define OV5630_B_GAIN_MAN_2         (0x330C) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:0]: b_gain_man[11:8]
#define OV5630_B_GAIN_MAN_1         (0x330D) // rw - b_gain_man[7:0]
#define OV5630_STABLE_RANGEW        (0x330E) // rw - Range from stable to unstable, stable_rangew -> stable_range
#define OV5630_AWB_FRAME_CNT        (0x330F) // rw - AWB gain will change one step every frame until it is stable
// (0x3310) reserved
#define OV5630_0x3311               (0x3311) // rw - Bit[7]:   wbc_opt_limit, 0:Use second big, 1:Use neighbor average
                                             //      Bit[6:5]: Border value sel, 00:0x00, 01:0xFF, 10:0x80, 11:dup
                                             //      Bit[4]:   th_opt (white_pixel only)
                                             //      Bit[3:0]: th_value (white pixel only)
#define OV5630_0x3312               (0x3312) // rw - Bit[7:5]: Reserved
                                             //      Bit[4]:   vap_addopt
                                             //      Bit[3:0]: wbc_th
#define OV5630_0x3313               (0x3313) // rw - Bit[7:4]: vap_avg_en[3:0]
                                             //      Bit[3:2]: vap_vskip[1:0], 00:Reserved, 01:(1:1), 10:(1:2), 11:(1:4)
                                             //      Bit[1:0]: vap_hskip, 00:Reserved, 01:(1:1), 10:(1:2), 11:(1:4)
#define OV5630_DSP_HSIZE_IN_2       (0x3314) // rw - Horizontal Input Size High Bits
                                             //      Bit[7:4]: Reserved
                                             //      Bit[3:0]: dsp_hsize_in[11:8]
#define OV5630_DSP_HSIZE_IN_1       (0x3315) // rw - Horizontal Input Size Low Bits
                                             //      dsp_hsize_in[7:0]
#define OV5630_DSP_VSIZE_IN_2       (0x3316) // rw - Vertical Input Size High Bits
                                             //      Bit[7:5]: Reserved
                                             //      Bit[2:0]: dsp_vsize_in[10:8]
#define OV5630_DSP_VSIZE_IN_1       (0x3317) // rw - Vertical Input Size Low Bits
                                             //      dsp_vsize_in[7:0]
#define OV5630_0x3318               (0x3318) // rw - Bit[7:4]: dsp_vpad_out
                                             //      Bit[3:0]: dsp_hpad_out
#define OV5630_0x3319               (0x3319) // rw - Bit[7]:   start_line
                                             //      Bit[6:4]: start_line
                                             //      Bit[3]:   even_man_en
                                             //      Bit[2]:   even_man1_en, evenodd man enable
                                             //      Bit[1]:   Evenodd from black line
                                             //      Bit[0]:   even_avg, Average current BLC with previous frame BLC
#define OV5630_EVEN_MAN0            (0x331A) // rw - even_man0
#define OV5630_EVEN_MAN1            (0x331B) // rw - even_man1
#define OV5630_EVEN_MAN2            (0x331C) // rw - even_man2
#define OV5630_EVEN_MAN3            (0x331D) // rw - even_man3
#define OV5630_0x331E               (0x331E) // rw - Bit[7]:   Manual BLC, 0:Auto BLC, 1:Manual BLC
                                             //      Bit[6]:   blc_man1_en, BLC manual enable
                                             //      Bit[5:4]: bypass_mode[1:0], Data bypass mode
                                             //      Bit[3]:   Reserved
                                             //      Bit[2]:   blc_cut_bline, Cut black line enable
                                             //      Bit[1]:   Average current BLC with previous frame BLC
                                             //      Bit[0]:   blc_agc_on, Frame agcchange
#define OV5630_0x331F               (0x331F) // rw - Bit[7:4]: black_num
                                             //      Bit[3:0]: blc_minnum
#define OV5630_BLC_LMT_OPTION       (0x3320) // rw - blc_lmt_option, BLC threshold
#define OV5630_BLC_THRE             (0x3321) // rw - Bit[7]:   Reserved
                                             //      Bit[6:0]: blc_thre, Evenodd threshold
#define OV5630_0x3322               (0x3322) // rw - Target for Short Exposure
#define OV5630_0x3323               (0x3323) // rw - Target for Long Exposure
#define OV5630_BLC_MAN0_2           (0x3324) // rw - BLC Offset for Short Exposure B Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man0[9:8]
#define OV5630_BLC_MAN0_1           (0x3325) // rw - BLC Offset for Short Exposure B Pixel
                                             //      blc_man0[7:0]
#define OV5630_BLC_MAN1_2           (0x3326) // rw - BLC Offset for Short Exposure Gb Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man1[9:8]
#define OV5630_BLC_MAN1_1           (0x3327) // rw - BLC Offset for Short Exposure Gb Pixel
                                             //      blc_man1[7:0]
#define OV5630_BLC_MAN2_2           (0x3328) // rw - BLC Offset for Short Exposure Gr Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man2[9:8]
#define OV5630_BLC_MAN2_1           (0x3329) // rw - BLC Offset for Short Exposure Gr Pixel
                                             //      blc_man2[7:0]
#define OV5630_BLC_MAN3_2           (0x332A) // rw - BLC Offset for Short Exposure R Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_ma3[9:8]
#define OV5630_BLC_MAN3_1           (0x332B) // rw - BLC Offset for Short Exposure R Pixel
                                             //      blc_man3[7:0]
#define OV5630_BLC_MAN4_2           (0x332C) // rw - BLC Offset for Long Exposure B Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man4[9:8]
#define OV5630_BLC_MAN4_1           (0x332D) // rw - BLC Offset for Long Exposure B Pixel
                                             //      blc_man4[7:0]
#define OV5630_BLC_MAN5_2           (0x332E) // rw - BLC Offset for Long Exposure Gb Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man5[9:8]
#define OV5630_BLC_MAN5_1           (0x332F) // rw - BLC Offset for Long Exposure Gb Pixel
                                             //      blc_man5[7:0]
#define OV5630_BLC_MAN6_2           (0x3330) // rw - BLC Offset for Long Exposure Gr Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_man6[9:8]
#define OV5630_BLC_MAN6_1           (0x3331) // rw - BLC Offset for Long Exposure Gr Pixel
                                             //      blc_man6[7:0]
#define OV5630_BLC_MAN7_2           (0x3332) // rw - BLC Offset for Long Exposure R Pixel
                                             //      Bit[7:2]: Reserved
                                             //      Bit[1:0]: blc_ma7[9:8]
#define OV5630_BLC_MAN7_1           (0x3333) // rw - BLC Offset for Long Exposure R Pixel
                                             //      blc_man7[7:0]
// (0x3334 - 0x33CE) reserved
#define OV5630_AWB_RGB_INDIR_ADR    (0x33CD) // rw - Auto white balance gain and current average RGB indirect mapping: Addr
#define OV5630_AWB_RGB_INDIR_DAT    (0x33FF) // ro - Auto white balance gain and current average RGB indirect mapping: Data
                                             //      Addr : Data
                                             //      0x08 : {Reserved, B_gain[3:0]}
                                             //      0x07 : {G_Gain[3:0], R_Gain[3:0]}
                                             //      0x06 : B_Gain[11:4]
                                             //      0x05 : G_Gain[11:4]
                                             //      0x04 : R_Gain[11:4]
                                             //      0x03 : {Reserved, B_AVG[1:0], G_AVG[1:0], R_AVG[1:0]}
                                             //      0x02 : R_AVG[9:2]
                                             //      0x01 : G_AVG[9:2]
                                             //      0x00 : B_AVG[9:2]

/*****************************************************************************
 * Clipping control registers
 *****************************************************************************/
#define OV5630_CLIP_CTRL0           (0x3400) // rw - b_max
#define OV5630_CLIP_CTRL1           (0x3401) // rw - b_min
#define OV5630_CLIP_CTRL2           (0x3402) // rw - g_max
#define OV5630_CLIP_CTRL3           (0x3403) // rw - g_min
#define OV5630_CLIP_CTRL4           (0x3404) // rw - r_max
#define OV5630_CLIP_CTRL5           (0x3405) // rw - r_min
#define OV5630_CLIP_CTRL6           (0x3406) // rw - Bit[7:6]: g_min[9:8]
                                             //      Bit[5:4]: g_max[9:8]
                                             //      Bit[3:2]: b_min[9:8]
                                             //      Bit[1:0]: b_max[9:8]
#define OV5630_CLIP_CTRL7           (0x3407) // rw - Bit[7]:   Reserved
                                             //      Bit[6]:   Bypass clipping, 1:Bypass
                                             //      Bit[5]:   r_flip
                                             //      Bit[4]:   r_mirror
                                             //      Bit[3:2]: r_min[9:8]
                                             //      Bit[1:0]: r_max[9:8]

/*****************************************************************************
 * DVP control registers
 *****************************************************************************/
#define OV5630_DVP_CTRL00           (0x3500) // rw - Bit[7]: vsync_sel2, 1:VSYNC get high after eof2v_dly from last data and get low after SOF or vsync_width PCLKS
                                             //      Bit[6]: vsync_sel, 1:Select VSYNC new
                                             //      Bit[5]: pclk_gate_en, 1:Gate dvp_pclk when no data transfer
                                             //      Bit[4]: vsync_gate, 0:Not gate dvp_pclk when VSYNC high, 1:Gate dvp_pclk when VSYNC and pclk_gate_en
                                             //      Bit[3]: dmy_line_sel, 0:Auto generate dummy lines, 1:Use first lines as dummylines
                                             //      Bit[2]: pclk_pol, Change polarity of PCLK
                                             //      Bit[1]: href_pol, Change polarity of HREF
                                             //      Bit[0]: vsync_pol, Change polarity of VSYNC
#define OV5630_DVP_CTRL01           (0x3501) // rw - Bit[7]: ccir656_en
                                             //      Bit[6]: sync_code_sel, 0:Auto generate sync_code, 1:Use FS, FE, LS and LE as CCIR656 sync_code
                                             //      Bit[5]: vhref_tst, 1:Use dvp_data_o output vhref_i
                                             //      Bit[4]: data_order, 0:DVP output dvp_data[11:0], 1:DVP output dvp_data[0:11]
                                             //      Bit[3]: dvp_bit8, 0:Swap 2 bit when dvp_h and dvp_l, 1:Swap 4 bit
                                             //      Bit[2]: dvp_h, 0:Output dvp_data[11:0], 1:Output dvp_data{n:0,11:n-1}, n:7 or 9
                                             //      Bit[1]: dvp_l, 0:Select dvp_data[11:0], 1:Select dvp_data{n:0, 11:n+1}, n:3 or 1
                                             //      Bit[0]: ch_flag, write it to 1 generate change flag for hsync mode
#define OV5630_DVP_CTRL02           (0x3502) // rw - FS
#define OV5630_DVP_CTRL03           (0x3503) // rw - FE
#define OV5630_DVP_CTRL04           (0x3504) // rw - LS
#define OV5630_DVP_CTRL05           (0x3505) // rw - LE
#define OV5630_DVP_CTRL06           (0x3506) // rw - Bit[7]:   cd_tst_sel
                                             //      Bit[6]:   Bypass DVP
                                             //      Bit[5]:   dvp_en
                                             //      Bit[4]:   hsync_en
                                             //      Bit[3]:   dvp_pclk_sel_o, s2p mode
                                             //      Bit[1:0]: vskip_man_o, for mipi
#define OV5630_DVP_CTRL07           (0x3507) // rw - Bit[7:1]: vsync_width[6:0], Width of vsync when select vsync_old and vsync_3
                                             //      Bit[0]:   hskip_man_o[0]
#define OV5630_DVP_CTRL08           (0x3508) // rw - Bit[7]:   Test pattern ON/OFF select, 0:OFF, 1:ON
                                             //      Bit[6]:   tst_bit8
                                             //      Bit[5]:   tst_bit12
                                             //      Bit[4]:   tst_mode
                                             //      Bit[3:0]: dmy_line_nu
#define OV5630_DVP_CTRL09           (0x3509) // rw - eof2v_dly[23:16]
#define OV5630_DVP_CTRL0A           (0x350A) // rw - eof2v_dly[15:8]
#define OV5630_DVP_CTRL0B           (0x350B) // rw - eof2v_dly[7:0]
#define OV5630_DVP_CTRL0C           (0x350C) // rw - pad_r
#define OV5630_DVP_CTRL0D           (0x350D) // rw - pad_l
#define OV5630_DVP_CTRL0E           (0x350E) // rw - Bit[7]:   snr_hsync_en 1, Use hsync from sensor as output href
                                             //      Bit[6]:   vsync_sel3, 1: VSYNC gets high soon after eof2v_dly from last data and gets low after vsync_width pclks, vsync_sel3 should be valid when vsync_sel2 is valid
                                             //      Bit[5:4]: vsync_width[8:7]
                                             //      Bit[3]:   sel_sync, 1: Select synced SNR VSYNC
                                             //      Bit[2]:   snr_vsync_sel, 1: Select VSYNC from sesnor
                                             //      Bit[1]:   dvp_dbg_sel
                                             //      Bit[0]:   skip_man_en_o
#define OV5630_DVP_CTRL0F           (0x350F) // rw - Bit[7]: eav_first, 0:sav_first, 1:eav_first
                                             //      Bit[6]:   f_sel
                                             //      Bit[5]:   f_value
                                             //      Bit[4]:   fix_f, 0:Auto generate ccir_f, 1:Use f_value as ccir_f
                                             //      Bit[3:2]: blk_sel, 00:Select 10'h200 and 10'h040 as toggle data
                                             //                         01:Select tog0 and tog1 as toggle data
                                             //                         10:Select 10'h000 as toggle data
                                             //                         11:Select 10'h000 as toggle data
                                             //      Bit[1]:   no_sof, 0:Reset state machine at sof, 1:Do not reset state machine when sof
                                             //      Bit[0]:   no_clip, 0:Clip output data between 10'h004 and 10'h3fb, 1:Do not clip output data
#define OV5630_DVP_CTRL10           (0x3510) // rw - Bit[7:4]: Reserved
                                             //      Bit[3:2]: tog0[9:8]
                                             //      Bit[1:0]: tog1[9:8]
#define OV5630_DVP_CTRL11           (0x3511) // rw - tog0[7:0], Toggle data0 when line blanking or dummy lines
#define OV5630_DVP_CTRL12           (0x3512) // rw - tog1[7:0], Toggle data1 when line blanking or dummy lines
#define OV5630_DVP_CTRL13           (0x3513) // rw - Bit[0]: uv_first
#define OV5630_DVP_CTRL14           (0x3514) // rw - h2v_dly
#define OV5630_DVP_CTRL15           (0x3515) // rw - v2h_dly

/*****************************************************************************
 * MIPI control registers
 *****************************************************************************/
#define OV5630_MIPI_CTRL00          (0x3600) // rw - Bit[7]: Reserved
                                             //      Bit[6]: ck_mark1_en
                                             //      Bit[5]: gate_sc_en
                                             //      Bit[4]: line_sync_en
                                             //      Bit[3]: lane_sel_o
                                             //      Bit[2]: idel_sts
                                             //      Bit[1]: first_bit
                                             //      Bit[0]: clk_lane_dis
#define OV5630_MIPI_CTRL01          (0x3601) // rw - Bit[7]: lpkt_dt_sel
                                             //      Bit[6]: spkt_dt_sel
                                             //      Bit[5]: spkt_wc_sel
                                             //      Bit[4]: ph_bit_order
                                             //      Bit[3]: ph_byte_order
                                             //      Bit[2]: ph_byte_order2
                                             //      Bit[1]: mark1_en1
                                             //      Bit[0]: mark1_en2
#define OV5630_MIPI_CTRL02          (0x3602) // rw - Bit[7]: hs_prepare_sel
                                             //      Bit[6]: clk_prepare_sel
                                             //      Bit[5]: clk_post_sel
                                             //      Bit[4]: clk_trail_sel
                                             //      Bit[3]: hs_exit_sel
                                             //      Bit[2]: hs_zero_sel
                                             //      Bit[1]: hs_trail_sel
                                             //      Bit[0]: clk_zero_sel
#define OV5630_MIPI_CTRL03          (0x3603) // rw - Bit[7:6]: lp_glitch_nu
                                             //      Bit[5:4]: cd_glitch_nu
                                             //      Bit[3]:   cd1_int_en
                                             //      Bit[2]:   cd2_int_en
                                             //      Bit[1]:   lp_cd1_en
                                             //      Bit[0]:   lp_cd2_en
#define OV5630_MIPI_CTRL04          (0x3604) // rw - Bit[7]: wait_pkt_end
                                             //      Bit[6]: tx_lsb_first
                                             //      Bit[5]: dir_recover_sel
                                             //      Bit[4]: mipi_reg_en
                                             //      Bit[3]: inc_en
                                             //      Bit[2]: lp_tx_lane_sel
                                             //      Bit[1]: wr_first_byte
                                             //      Bit[0]: rd_ta_en
#define OV5630_MIPI_CTRL05          (0x3605) // rw - Bit[7]: lane_disable2
                                             //      Bit[6]: lane_disable1
                                             //      Bit[5]: lpx_p_sel
                                             //      Bit[4]: lp_rx_intr_sel
                                             //      Bit[3]: cd_tst_sel
                                             //      Bit[2]: mipi_reg_mask
                                             //      Bit[1]: bist_en
                                             //      Bit[0]: hd_sk_en
#define OV5630_MIPI_CTRL06          (0x3606) // rw - Bit[7]: hdr_wbc_en
                                             //      Bit[6]:   ahead_3line
                                             //      Bit[5:0]: start_size
#define OV5630_MIPI_CTRL07          (0x3607) // rw - llps_dly
#define OV5630_MIPI_CTRL08          (0x3608) // rw - lsps_dly
// (0x3609 - 0x360F) reserved
#define OV5630_MIPI_CTRL10          (0x3610) // rw - fcnt_max[15:8]
#define OV5630_MIPI_CTRL11          (0x3611) // rw - fcnt_max[7:0]
#define OV5630_MIPI_CTRL12          (0x3612) // rw - spkt_wc_reg[15:8]
#define OV5630_MIPI_CTRL13          (0x3613) // rw - spkt_wc_reg[7:0]
#define OV5630_MIPI_CTRL14          (0x3614) // rw - Bit[7:6]: vc
                                             //      Bit[5:0]: dt_man
#define OV5630_MIPI_CTRL15          (0x3615) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: dt_spkt
#define OV5630_MIPI_CTRL16          (0x3616) // rw - Bit[7]:   Reserved
                                             //      Bit[6:4]: reg_wr_dly
                                             //      Bit[3]:   Reserved
                                             //      Bit[2:0]: reg_rd_dly
#define OV5630_MIPI_CTRL17          (0x3617) // rw - Reserved
#define OV5630_MIPI_CTRL18          (0x3618) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_zero_min[9:8]
#define OV5630_MIPI_CTRL19          (0x3619) // rw - hs_zero_min[7:0]
#define OV5630_MIPI_CTRL1A          (0x361A) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_trail_min[9:8]
#define OV5630_MIPI_CTRL1B          (0x361B) // rw - hs_trail_min[7:0]
#define OV5630_MIPI_CTRL1C          (0x361C) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_zero_min[9:8]
#define OV5630_MIPI_CTRL1D          (0x361D) // rw - clk_zero_min[7:0]
#define OV5630_MIPI_CTRL1E          (0x361E) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_prepare_min[9:8]
#define OV5630_MIPI_CTRL1F          (0x361F) // rw - clk_prepare_min[7:0]
#define OV5630_MIPI_CTRL20          (0x3620) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_post_min[9:8]
#define OV5630_MIPI_CTRL21          (0x3621) // rw - clk_post_min[7:0]
#define OV5630_MIPI_CTRL22          (0x3622) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: clk_trail_min[9:8]
#define OV5630_MIPI_CTRL23          (0x3623) // rw - clk_trail_min[7:0]
#define OV5630_MIPI_CTRL24          (0x3624) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: lpx_p_min[9:8]
#define OV5630_MIPI_CTRL25          (0x3625) // rw - lpx_p_min[7:0]
#define OV5630_MIPI_CTRL26          (0x3626) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_prepare_min[9:8]
#define OV5630_MIPI_CTRL27          (0x3627) // rw - hs_prepare_min[7:0]
#define OV5630_MIPI_CTRL28          (0x3628) // rw - Bit[7:2]: Reserved
                                             //      Bit[1:0]: hs_exit_min[9:8]
#define OV5630_MIPI_CTRL29          (0x3629) // rw - hs_exit_min[7:0]
#define OV5630_MIPI_CTRL2A          (0x362A) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_hs_zero_min
#define OV5630_MIPI_CTRL2B          (0x362B) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_hs_trail_min
#define OV5630_MIPI_CTRL2C          (0x362C) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_clk_zero_min
#define OV5630_MIPI_CTRL2D          (0x362D) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_clk_prepare_min
#define OV5630_MIPI_CTRL2E          (0x362E) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_clk_post_min
#define OV5630_MIPI_CTRL2F          (0x362F) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_clk_trail_min
#define OV5630_MIPI_CTRL30          (0x3630) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_lpx_p_min
#define OV5630_MIPI_CTRL31          (0x3631) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_hs_prepare_min
#define OV5630_MIPI_CTRL32          (0x3632) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: ui_hs_exit_min
#define OV5630_MIPI_CTRL33          (0x3633) // rw - mipi_reg_min[15:8]
#define OV5630_MIPI_CTRL34          (0x3634) // rw - mipi_reg_min[7:0]
#define OV5630_MIPI_CTRL35          (0x3635) // rw - mipi_reg_max[15:8]
#define OV5630_MIPI_CTRL36          (0x3636) // rw - mipi_reg_max[7:0]
#define OV5630_MIPI_CTRL37          (0x3637) // rw - pclk_period
#define OV5630_MIPI_CTRL38          (0x3638) // rw - Bit[5:0]: wkup_dly
#define OV5630_MIPI_CTRL39          (0x3639) // rw - Reserved
#define OV5630_MIPI_CTRL3A          (0x363A) // rw - Bit[7:6]: Reserved
                                             //      Bit[5:0]: dir_dly
#define OV5630_MIPI_CTRL3B          (0x363B) // rw - Bit[7]: lp_sel1
                                             //      Bit[6]: lp_dir_man1
                                             //      Bit[5]: lp_p1
                                             //      Bit[4]: lp_n1
                                             //      Bit[3]: lp_sel2
                                             //      Bit[2]: lp_dir_man2
                                             //      Bit[1]: lp_p2
                                             //      Bit[0]: lp_n2
#define OV5630_MIPI_CTRL3C          (0x363C) // rw - Bit[7:4]: t_lpx
                                             //      Bit[3:0]: t_clk_pre
#define OV5630_MIPI_CTRL3D          (0x363D) // rw - t_ta_go
#define OV5630_MIPI_CTRL3E          (0x363E) // rw - t_ta_sure
#define OV5630_MIPI_CTRL3F          (0x363F) // rw - t_ta_get
#define OV5630_MIPI_RO61            (0x3661) // ro - hd_sk0_reg
#define OV5630_MIPI_RO62            (0x3662) // ro - hd_sk1_reg
#define OV5630_MIPI_RO63            (0x3663) // ro - hd_sk2_reg
#define OV5630_MIPI_RO64            (0x3664) // ro - hd_sk3_reg
#define OV5630_MIPI_RO65            (0x3665) // ro - Bit[7:6]: Reserved
                                             //      Bit[5]:   lp_rx_se
                                             //      Bit[4]:   tx_busy
                                             //      Bit[3]:   mipi_lp_p1
                                             //      Bit[2]:   mipi_lp_n1
                                             //      Bit[1]:   mipi_lp_p2
                                             //      Bit[0]:   mipi_lp_n2
#define OV5630_MIPI_RO66            (0x3666) // ro - Bit[7:2]: Reserved
                                             //      Bit[1]:   bist_err
                                             //      Bit[0]:   bist_ok

/*****************************************************************************
 * Default values
 *****************************************************************************/

// column 1)
//    Reset values as of datasheet OV5630_DS_1.1_SiliconImage.pdf from
//    6.12.2008, values not given in the datasheet are values from 2) or 3).
// column 2)
//    For Rev. A sensor. Values deviating from the datasheet are derived from
//    evaluation kit settings OV5630R1A_A01.ovd SEN_2592x1944. AWB, AE, AGC
//    etc. of the sensor is on.
// column 3)
//    For Rev. C sensors. Values deviating from the datasheet are derived from
//    evaluation kit settings OV5630R1C_A04.ovd SEN_2592x1944_3.75fps. AWB, AE,
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
#define OV5630_AGCL_DEFAULT                   (0x00) //  (0x00)   (0x00)
#define OV5630_AGCS_DEFAULT                   (0x00) //  (0x00)   (0x00)
#define OV5630_AECL_DEFAULT                 (0x0000) //(0x0000) (0x0000)
#define OV5630_LAEC_DEFAULT                 (0x0000) //(0x0000) (0x0000)
#define OV5630_RSVD_0x3006_DEFAULT            (0x00) //  (0x00)   (0x00) reserved in the register list but [0] referenced by manual digital gain
#define OV5630_AECS_DEFAULT                 (0x0000) //(0x0000) (0x0000)
#define OV5630_PIDH_DEFAULT                   (0x56) //  (0x56)   (0x56)
#define OV5630_PIDL_DEFAULT                   (0x33) //  (0x32)   (0x32) 0x32 = Rev. A, 0x33 = Rev. C
#define OV5630_SCCB_ID_DEFAULT                (0x6C) //  (0x6C)   (0x6C)
#define OV5630_R_PLL1_DEFAULT                 (0xBC) //  (0x38)   (0x38)
#define OV5630_R_PLL2_DEFAULT                 (0x00) //  (0x10)   (0x01)
#define OV5630_R_PLL3_DEFAULT                 (0x03) //  (0x02)   (0x02)
#define OV5630_R_PLL4_DEFAULT                 (0x40) //  (0x80)   (0x40)
#define OV5630_SYS_DEFAULT                    (0x00) //  (0x00)   (0x00)
#define OV5630_AUTO1_DEFAULT                  (0xff) //  (0xFF)   (0x00)
#define OV5630_AUTO2_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV5630_AUTO3_DEFAULT                  (0x03) //  (0x03)   (0x00)
#define OV5630_AUTO4_DEFAULT                  (0x01) //  (0x01)   (0x00)
#define OV5630_AUTO5_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV5630_WPT_DEFAULT                    (0x3C) //  (0x50)   (0x78)
#define OV5630_BPT_DEFAULT                    (0x38) //  (0x40)   (0x68)
#define OV5630_VPT_DEFAULT                    (0x82) //  (0x82)   (0xD4)
#define OV5630_YAVG_DEFAULT                   (0x00) //  (0x00)   (0x00)
#define OV5630_AECG_MAX50_DEFAULT             (0x05) //  (0x05)   (0x05)
#define OV5630_AECG_MAX60_DEFAULT             (0x07) //  (0x07)   (0x07)
#define OV5630_ADDVS_DEFAULT                (0x0000) //(0x0000) (0x0000)
#define OV5630_FRAME_LENGTH_LINES_DEFAULT   (0x07BC) //(0x07BC) (0x07BC)
#define OV5630_LINE_LENGTH_PCK_DEFAULT      (0x0CA0) //(0x0CA0) (0x0CA0)
#define OV5630_X_ADDR_START_DEFAULT         (0x0020) //(0x0000) (0x0000)
#define OV5630_Y_ADDR_START_DEFAULT         (0x0008) //(0x0000) (0x0000)
#define OV5630_X_ADDR_END_DEFAULT           (0x0A3F) //(0x0A1F) (0x0A1F)
#define OV5630_Y_ADDR_END_DEFAULT           (0x07A3) //(0x0797) (0x0797)
#define OV5630_X_OUTPUTSIZE_DEFAULT         (0x0A20) //(0x0A20) (0x0A20)
#define OV5630_Y_OUTPUTSIZE_DEFAULT         (0x0798) //(0x0798) (0x0798)
#define OV5630_FRAME_CNT_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_DATR_LMO_3_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DATR_LMO_2_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DATR_LMO_1_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DATR_D56_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_DATR_EF_DEFAULT                (0x00) //  (0x00)   (0x00)
#define OV5630_R_SIGMA_6_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_R_SIGMA_5_DEFAULT              (0x30) //  (0x30)   (0x30)
#define OV5630_R_SIGMA_4_DEFAULT              (0x00) //  (0x00)   (0x20)
#define OV5630_R_SIGMA_3_DEFAULT              (0x30) //  (0x30)   (0x30)
#define OV5630_R_SIGMA_2_DEFAULT              (0xA7) //  (0xA7)   (0xA7)
#define OV5630_R_SIGMA_1_DEFAULT              (0xA0) //  (0x29)   (0x29)
#define OV5630_D56COM_DEFAULT                 (0x22) //  (0x22)   (0x02)
#define OV5630_RSVD_0x304F_DEFAULT            (0xa0) //  (0xa0)   (0xa0) reserved but written by eva kit to 0x00 and then 0xa0
#define OV5630_R5060TH_DEFAULT                (0x00) //  (0x00)   (0x00)
#define OV5630_LMO_TH1_DEFAULT                (0x00) //  (0x00)   (0x00)
#define OV5630_LMO_TH2_DEFAULT                (0xff) //  (0xff)   (0x00)
#define OV5630_LMO_K_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV5630_BD50ST_DEFAULT               (0x0129) //(0x0129) (0x0055)
#define OV5630_BD60ST_DEFAULT               (0x00f7) //(0x00f7) (0x5501)
#define OV5630_RSVD_0x3065_DEFAULT            (0x50) //  (0x50)   (0x50) reserved but written by eva kit
#define OV5630_RSVD_0x3068_DEFAULT            (0x08) //  (0x08)   (0x08) reserved but written by eva kit
#define OV5630_RSVD_0x3069_DEFAULT            (0x80) //  (0x80)   (0x80) reserved but written by eva kit to 0x00 and then 0x80
#define OV5630_RSVD_0x306A_DEFAULT            (0x05) //  (0x04)   (0x04) reserved but written by eva kit (0x04 read after reset)
#define OV5630_HSYNST_DEFAULT                 (0x08) //  (0x08)   (0x08)
#define OV5630_HSYNED_DEFAULT                 (0x20) //  (0x20)   (0x20)
#define OV5630_HSYNED_HSYNST_DEFAULT          (0x00) //  (0x00)   (0x00)
#define OV5630_TMC_RWIN0_DEFAULT              (0x24) //  (0x24)   (0x24)
#define OV5630_RSVD_0x3071_DEFAULT            (0x34) //  (0x20)   (0x20) reserved but written by eva kit
#define OV5630_RSVD_0x3072_DEFAULT            (0x0d) //  (0x0d)   (0x0d) reserved but written by eva kit
#define OV5630_RSVD_0x3075_DEFAULT            (0x22) //  (0x22)   (0x22) reserved but written by eva kit
#define OV5630_RSVD_0x3076_DEFAULT            (0x23) //  (0x22)   (0x22) reserved but written by eva kit
#define OV5630_RSVD_0x3077_DEFAULT            (0x24) //  (0x24)   (0x24) reserved but written by eva kit
#define OV5630_RSVD_0x3078_DEFAULT            (0x25) //  (0x24)   (0x24) reserved but written by eva kit
#define OV5630_RSVD_0x3084_DEFAULT            (0x44) //  (0x44)   (0x44) reserved but written by eva kit
#define OV5630_RSVD_0x308A_DEFAULT            (0x25) //  (0x25)   (0x25) reserved but written by eva kit
#define OV5630_RSVD_0x308B_DEFAULT            (0x82) //  (0x82)   (0x82) reserved but written by eva kit
#define OV5630_RSVD_0x308D_DEFAULT            (0x0b) //  (0x0b)   (0x0b) reserved but written by eva kit
#define OV5630_RSVD_0x3090_DEFAULT            (0x67) //  (0x94)   (0x94) reserved but written by eva kit
#define OV5630_RSVD_0x3091_DEFAULT            (0x04) //  (0x04)   (0x04) reserved but written by eva kit
#define OV5630_RSVD_0x3098_DEFAULT            (0x54) //  (0x54)   (0x54) reserved but written by eva kit to 0x5c and then 0x54
#define OV5630_RSVD_0x3099_DEFAULT            (0x49) //  (0x45)   (0x45) reserved but written by eva kit (0x45 read after reset)
#define OV5630_RSVD_0x309D_DEFAULT            (0x04) //  (0x04)   (0x04) reserved but written by eva kit to 0x00 and then 0x04
#define OV5630_RSVD_0x309E_DEFAULT            (0x24) //  (0x24)   (0x24) reserved but written by eva kit
#define OV5630_RSVD_0x30A1_DEFAULT            (0xC4) //  (0x44)   (0x44) reserved but written by eva kit (0x44 read after reset)
#define OV5630_RSVD_0x30AC_DEFAULT            (0x05) //  (0x06)   (0x06) reserved but written by eva kit
#define OV5630_RSVD_0x30AD_DEFAULT            (0x20) //  (0x10)   (0x10) reserved but written by eva kit
#define OV5630_RSVD_0x30AE_DEFAULT            (0x15) //  (0x05)   (0x05) reserved but written by eva kit
#define OV5630_RSVD_0x30AF_DEFAULT            (0x10) //  (0x10)   (0x10) reserved but written by eva kit
#define OV5630_IO_CTRL0_DEFAULT               (0xFF) //  (0xFF)   (0xFF)
#define OV5630_IO_CTRL1_DEFAULT               (0xFF) //  (0xFF)   (0xFF)
#define OV5630_IO_CTRL2_DEFAULT               (0x32) //  (0x32)   (0x00)
#define OV5630_DSIO_3_DEFAULT                 (0x00) //  (0x00)   (0x00)
#define OV5630_DSIO_2_DEFAULT                 (0x00) //  (0x00)   (0x00)
#define OV5630_DSIO_1_DEFAULT                 (0x1c) //  (0x0c)   (0x10)
#define OV5630_TMC_TMC10_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_TMC_TMC12_DEFAULT              (0x80) //  (0x80)   (0x80)
#define OV5630_TMC_TMC14_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_TMC_COM4_DEFAULT               (0x01) //  (0x01)   (0x01)
#define OV5630_TMC_REG6C_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_TMC_REG6E_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_R_CLK_RSCLK_DEFAULT            (0x07) //  (0x07)   (0x07)
#define OV5630_R_CLK_RACLK_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_R_CLK_RACLK1_DEFAULT           (0x02) //  (0x12)   (0x80)
#define OV5630_FRS_RFRES0_DEFAULT             (0x06) //  (0x06)   (0x06)
#define OV5630_FRS_RFRES1_DEFAULT             (0x80) //  (0x80)   (0x80)
#define OV5630_FRS_RFRES2_DEFAULT             (0x88) //  (0x88)   (0x88)
#define OV5630_FRS_RFRES3_DEFAULT             (0x0C) //  (0x0C)   (0x0C)
#define OV5630_FRS_FECNT_DEFAULT              (0x07) //  (0x07)   (0x07)
#define OV5630_FRS_FFCNT_DEFAULT            (0x0020) //(0x0020) (0x0020)
#define OV5630_FRS_RFRM_DEFAULT               (0x01) //  (0x80)   (0x80)
#define OV5630_FRS_RSTRB_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_SA1TMC_DEFAULT                 (0x04) //  (0x05)   (0x00)
#define OV5630_TMC_30EA_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_TMC_30EB_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_FLEX_REG_TXP_DEFAULT           (0x10) //  (0x10)   (0x10)
#define OV5630_FLEX_REG_FLT_DEFAULT           (0x08) //  (0x08)   (0x08)
#define OV5630_FLEX_REG_TXT_DEFAULT           (0x10) //  (0x10)   (0x10)
#define OV5630_FLEX_REG_HBK_DEFAULT           (0x20) //  (0x20)   (0x20)
#define OV5630_FLEX_REG_HSG_DEFAULT           (0x20) //  (0x20)   (0x20)
#define OV5630_FLEX_SA1SFT_DEFAULT            (0x40) //  (0x40)   (0x40)
#define OV5630_RVSOPT_DEFAULT                 (0xB0) //  (0xB0)   (0xB0)
#define OV5630_AUTO6_DEFAULT                  (0x00) //  (0x00)   (0x00)
#define OV5630_IMAGE_TRANSFO_DEFAULT          (0x00) //  (0x00)   (0x00)
#define OV5630_IMAGE_LUM_DEFAULT              (0x11) //  (0x91)   (0x00)
#define OV5630_IMAGE_SYSTEM_DEFAULT           (0x00) //  (0x00)   (0x00) do not enable streaming here, streaming is off during sensor setup and will be enabled afterwards in IsiSensorSetStreamingIss
#define OV5630_GROUP_WR_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_RSVD_0x3103_DEFAULT            (0x10) //  (0x10)   (0x10) reserved but written by eva kit

/*****************************************************************************
 * CIF control registers
 *****************************************************************************/
#define OV5630_CIF_CTRL2_DEFAULT              (0x00) // (0x00)    (0x00)

/*****************************************************************************
 * ISP control registers
 *****************************************************************************/
#define OV5630_ISP_CTRL00_DEFAULT             (0xFB) //  (0xF3)   (0xFF)
#define OV5630_ISP_CTRL01_DEFAULT             (0xC0) //  (0xC0)   (0xC0)
#define OV5630_ISP_CTRL02_DEFAULT             (0x60) //  (0x00)   (0x00)
#define OV5630_0x3303_DEFAULT                 (0x00) //  (0x00)   (0x00)
#define OV5630_DIG_GAIN_MAN_DEFAULT           (0x4c) //  (0x40)   (0x40)
#define OV5630_BIAS_MAN_DEFAULT               (0x40) //  (0x40)   (0x40)
#define OV5630_0x3306_DEFAULT                 (0x42) //  (0x42)   (0x42)
#define OV5630_STABLE_RANGE_DEFAULT           (0x04) //  (0x04)   (0x04)
#define OV5630_R_GAIN_MAN_DEFAULT           (0x0400) //(0x0400) (0x0400)
#define OV5630_G_GAIN_MAN_DEFAULT           (0x0400) //(0x0400) (0x0400)
#define OV5630_B_GAIN_MAN_DEFAULT           (0x0400) //(0x0400) (0x0400)
#define OV5630_STABLE_RANGEW_DEFAULT          (0x08) //  (0x08)   (0x08)
#define OV5630_AWB_FRAME_CNT_DEFAULT          (0x00) //  (0x00)   (0x00)
#define OV5630_0x3311_DEFAULT                 (0x80) //  (0x00)   (0x00)
#define OV5630_0x3312_DEFAULT                 (0x1F) //  (0x10)   (0x00)
#define OV5630_0x3313_DEFAULT                 (0xF0) //  (0xF0)   (0xF0)
#define OV5630_DSP_HSIZE_IN_DEFAULT         (0x0A22) //(0x0A20) (0x0A20)
#define OV5630_DSP_VSIZE_IN_DEFAULT         (0x079C) //(0x0798) (0x0798)
#define OV5630_0x3318_DEFAULT                 (0x22) //  (0x00)   (0x00)
#define OV5630_0x3319_DEFAULT                 (0x22) //  (0x43)   (0x01)
#define OV5630_EVEN_MAN0_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_EVEN_MAN1_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_EVEN_MAN2_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_EVEN_MAN3_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_0x331E_DEFAULT                 (0x05) //  (0x07)   (0x07) do not set [7], it is assumed to be 0 by SetupImageControl
#define OV5630_0x331F_DEFAULT                 (0x22) //  (0x22)   (0x02)
#define OV5630_BLC_LMT_OPTION_DEFAULT         (0xFF) //  (0xFF)   (0xFF)
#define OV5630_BLC_THRE_DEFAULT               (0x04) //  (0x04)   (0x10)
#define OV5630_0x3322_DEFAULT                 (0x10) //  (0x10)   (0x10)
#define OV5630_0x3323_DEFAULT                 (0x10) //  (0x10)   (0x10)
#define OV5630_BLC_MAN0_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN1_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN2_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN3_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN4_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN5_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN6_DEFAULT               (0x00) //  (0x00)   (0x00)
#define OV5630_BLC_MAN7_DEFAULT               (0x00) //  (0x00)   (0x00)
// no defaults defined for indirect mapping registers//, so 0x00 is used
#define OV5630_AWB_RGB_INDIR_ADR_DEFAULT      (0x00) //  (0x00)   (0x00)
#define OV5630_AWB_RGB_INDIR_DAT_DEFAULT      (0x00) //  (0x00)   (0x00)

/*****************************************************************************
 * Clipping control registers
 *****************************************************************************/
#define OV5630_CLIP_CTRL0_DEFAULT             (0xFF) //  (0xFF)   (0xFF)
#define OV5630_CLIP_CTRL1_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_CLIP_CTRL2_DEFAULT             (0xFF) //  (0xFF)   (0xFF)
#define OV5630_CLIP_CTRL3_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_CLIP_CTRL4_DEFAULT             (0xFF) //  (0xFF)   (0xFF)
#define OV5630_CLIP_CTRL5_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_CLIP_CTRL6_DEFAULT             (0x33) //  (0x33)   (0x33)
#define OV5630_CLIP_CTRL7_DEFAULT             (0x03) //  (0x03)   (0x03)

/*****************************************************************************
 * DVP control registers
 *****************************************************************************/
#define OV5630_DVP_CTRL00_DEFAULT             (0x80) //  (0x80)   (0x80)
#define OV5630_DVP_CTRL01_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL02_DEFAULT             (0xAB) //  (0xAB)   (0xAB)
#define OV5630_DVP_CTRL03_DEFAULT             (0xB6) //  (0xB6)   (0xB6)
#define OV5630_DVP_CTRL04_DEFAULT             (0x80) //  (0x80)   (0x80)
#define OV5630_DVP_CTRL05_DEFAULT             (0x9D) //  (0x9D)   (0x9D)
#define OV5630_DVP_CTRL06_DEFAULT             (0x20) //  (0x20)   (0x20)
#define OV5630_DVP_CTRL07_DEFAULT             (0x80) //  (0x80)   (0x80)
#define OV5630_DVP_CTRL08_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL09_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL0A_DEFAULT             (0x01) //  (0x01)   (0x01)
#define OV5630_DVP_CTRL0B_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL0C_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL0D_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL0E_DEFAULT             (0x40) //  (0x40)   (0x40)
#define OV5630_DVP_CTRL0F_DEFAULT             (0x88) //  (0x88)   (0x88)
#define OV5630_DVP_CTRL10_DEFAULT             (0x09) //  (0x09)   (0x09)
#define OV5630_DVP_CTRL11_DEFAULT             (0xAA) //  (0xAA)   (0xAA)
#define OV5630_DVP_CTRL12_DEFAULT             (0x55) //  (0x55)   (0x55)
#define OV5630_DVP_CTRL13_DEFAULT             (0x01) //  (0x01)   (0x01)
#define OV5630_DVP_CTRL14_DEFAULT             (0x00) //  (0x00)   (0x00)
#define OV5630_DVP_CTRL15_DEFAULT             (0x00) //  (0x00)   (0x00)

/*****************************************************************************
 * MIPI control registers
 *****************************************************************************/
#define OV5630_MIPI_CTRL00_DEFAULT            (0x04) //  (0x04)   (0x04)
#define OV5630_MIPI_CTRL01_DEFAULT            (0x03) //  (0x03)   (0x03)
#define OV5630_MIPI_CTRL02_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL03_DEFAULT            (0x5F) //  (0x5F)   (0x5F)
#define OV5630_MIPI_CTRL04_DEFAULT            (0x8D) //  (0x8D)   (0x8D)
#define OV5630_MIPI_CTRL05_DEFAULT            (0x10) //  (0x10)   (0x10)
#define OV5630_MIPI_CTRL06_DEFAULT            (0x77) //  (0x77)   (0x77)
#define OV5630_MIPI_CTRL07_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL08_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL10_DEFAULT            (0xFF) //  (0xFF)   (0xFF)
#define OV5630_MIPI_CTRL11_DEFAULT            (0xFF) //  (0xFF)   (0xFF)
#define OV5630_MIPI_CTRL12_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL13_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL14_DEFAULT            (0x2A) //  (0x2A)   (0x2A)
#define OV5630_MIPI_CTRL15_DEFAULT            (0x01) //  (0x01)   (0x01)
#define OV5630_MIPI_CTRL16_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL17_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL18_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL19_DEFAULT            (0x96) //  (0x96)   (0x96)
#define OV5630_MIPI_CTRL1A_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL1B_DEFAULT            (0x3C) //  (0x3C)   (0x3C)
#define OV5630_MIPI_CTRL1C_DEFAULT            (0x01) //  (0x01)   (0x01)
#define OV5630_MIPI_CTRL1D_DEFAULT            (0x86) //  (0x86)   (0x86)
#define OV5630_MIPI_CTRL1E_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL1F_DEFAULT            (0x3C) //  (0x3C)   (0x3C)
#define OV5630_MIPI_CTRL20_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL21_DEFAULT            (0x56) //  (0x56)   (0x56)
#define OV5630_MIPI_CTRL22_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL23_DEFAULT            (0x3C) //  (0x3C)   (0x3C)
#define OV5630_MIPI_CTRL24_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL25_DEFAULT            (0x32) //  (0x32)   (0x32)
#define OV5630_MIPI_CTRL26_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL27_DEFAULT            (0x32) //  (0x32)   (0x32)
#define OV5630_MIPI_CTRL28_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL29_DEFAULT            (0x64) //  (0x64)   (0x64)
#define OV5630_MIPI_CTRL2A_DEFAULT            (0x05) //  (0x05)   (0x05)
#define OV5630_MIPI_CTRL2B_DEFAULT            (0x04) //  (0x04)   (0x04)
#define OV5630_MIPI_CTRL2C_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL2D_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL2E_DEFAULT            (0x34) //  (0x34)   (0x34)
#define OV5630_MIPI_CTRL2F_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL30_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL31_DEFAULT            (0x04) //  (0x04)   (0x04)
#define OV5630_MIPI_CTRL32_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL33_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL34_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL35_DEFAULT            (0xFF) //  (0xFF)   (0xFF)
#define OV5630_MIPI_CTRL36_DEFAULT            (0xFF) //  (0xFF)   (0xFF)
#define OV5630_MIPI_CTRL37_DEFAULT            (0x10) //  (0x10)   (0x10)
#define OV5630_MIPI_CTRL38_DEFAULT            (0x08) //  (0x08)   (0x08)
#define OV5630_MIPI_CTRL39_DEFAULT            (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_CTRL3A_DEFAULT            (0x08) //  (0x08)   (0x08)
#define OV5630_MIPI_CTRL3B_DEFAULT            (0x33) //  (0x33)   (0x33)
#define OV5630_MIPI_CTRL3C_DEFAULT            (0x4F) //  (0x4F)   (0x4F)
#define OV5630_MIPI_CTRL3D_DEFAULT            (0x10) //  (0x10)   (0x10)
#define OV5630_MIPI_CTRL3E_DEFAULT            (0x06) //  (0x06)   (0x06)
#define OV5630_MIPI_CTRL3F_DEFAULT            (0x14) //  (0x14)   (0x14)
#define OV5630_MIPI_RO61_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_RO62_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_RO63_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_RO64_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_RO65_DEFAULT              (0x00) //  (0x00)   (0x00)
#define OV5630_MIPI_RO66_DEFAULT              (0x00) //  (0x00)   (0x00)



typedef struct OV5630_Context_s
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

    uint32_t            OldCoarseIntegrationTime;
	uint32_t			preview_minimum_framerate;
} OV5630_Context_t;



#ifdef __cplusplus
}
#endif

/* @} ov5630_priv */

#endif /* __OV5630_PRIV_H__ */

