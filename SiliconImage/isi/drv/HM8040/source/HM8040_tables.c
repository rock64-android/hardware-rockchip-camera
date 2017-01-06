//HM8040_tables.c
/*****************************************************************************/
/*!
 *  \file        hm8040_tables.c \n
 *  \version  1.0 \n
 *  \author   wsq \n
 *  \brief      Image-sensor-specific tables and other
 *               constant values/structures for HM8040. \n
 *
 *  \revision $Revision: 803 $ \n
 *               $Author: $ \n
 *               $Date: 2016-08-18 11:35:22 +0100 (Fr, 18 08 2016) $ \n
 *               $Id: HM8040_tables.c 803 2016-08-18 15:35:22Z  $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "HM8040_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/
// Image sensor register settings default values taken from data sheet OV8810_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.
const IsiRegDescription_t HM8040_g_aRegDescription_onelane[] =
{
	
	// MIPI=720Mbps, SysClk=144Mhz,Dac Clock=360Mhz.
	// v00_01_00 (05/29/2013) : initial setting
	{0x0103 ,0x01 ,"0x0100",eReadWrite},// ; software reset
	{0x0100 ,0x00 ,"0x0100",eReadWrite},//; software standby
	{0x0100 ,0x00 ,"0x0100",eReadWrite},// ;
	{0x0100 ,0x00 ,"0x0100",eReadWrite},// ;
	{0x0100 ,0x00 ,"0x0100",eReadWrite},//;
	{0x0302 ,0x1e ,"0x0100",eReadWrite},//; pll1_multi
	{0x0303 ,0x00 ,"0x0100",eReadWrite},//; pll1_divm
	{0x0304 ,0x03 ,"0x0100",eReadWrite},//; pll1_div_mipi
	{0x030e ,0x00 ,"0x0100",eReadWrite},//; pll2_rdiv
	{0x030f ,0x09 ,"0x0100",eReadWrite},//; pll2_divsp
	{0x0312 ,0x01 ,"0x0100",eReadWrite},//; pll2_pre_div0, pll2_r_divdac
	{0x031e ,0x0c ,"0x0100",eReadWrite},//; pll1_no_lat
	{0x3600 ,0x00,"0x0100",eReadWrite},//
	{0x3601 ,0x00,"0x0100",eReadWrite},//
	{0x3602 ,0x00,"0x0100",eReadWrite},//
	{0x3603 ,0x00,"0x0100",eReadWrite},//
	{0x3604 ,0x22,"0x0100",eReadWrite},//
	{0x3605 ,0x30,"0x0100",eReadWrite},//
	{0x3606 ,0x00,"0x0100",eReadWrite},//
	{0x3607 ,0x20,"0x0100",eReadWrite},//
	{0x3608 ,0x11,"0x0100",eReadWrite},//
	{0x3609 ,0x28,"0x0100",eReadWrite},//
	{0x360a ,0x00,"0x0100",eReadWrite},//
	{0x360b ,0x06,"0x0100",eReadWrite},//
	{0x360c ,0xdc,"0x0100",eReadWrite},//
	{0x360d ,0x40,"0x0100",eReadWrite},//
	{0x360e ,0x0c,"0x0100",eReadWrite},//
	{0x360f ,0x20,"0x0100",eReadWrite},//
	{0x3610 ,0x07,"0x0100",eReadWrite},//
	{0x3611 ,0x20,"0x0100",eReadWrite},//
	{0x3612 ,0x88,"0x0100",eReadWrite},//
	{0x3613 ,0x80,"0x0100",eReadWrite},//
	{0x3614 ,0x58,"0x0100",eReadWrite},//
	{0x3615 ,0x00,"0x0100",eReadWrite},//
	{0x3616 ,0x4a,"0x0100",eReadWrite},//
	{0x3617 ,0x90,"0x0100",eReadWrite},//
	{0x3618 ,0x56,"0x0100",eReadWrite},//
	{0x3619 ,0x70,"0x0100",eReadWrite},//
	{0x361a ,0x99,"0x0100",eReadWrite},//
	{0x361b ,0x00,"0x0100",eReadWrite},//
	{0x361c ,0x07,"0x0100",eReadWrite},//
	{0x361d ,0x00,"0x0100",eReadWrite},//
	{0x361e ,0x00,"0x0100",eReadWrite},//
	{0x361f ,0x00,"0x0100",eReadWrite},//
	{0x3638 ,0xff,"0x0100",eReadWrite},//
	{0x3633 ,0x0c,"0x0100",eReadWrite},//
	{0x3634 ,0x0c,"0x0100",eReadWrite},//
	{0x3635 ,0x0c,"0x0100",eReadWrite},//
	{0x3636 ,0x0c,"0x0100",eReadWrite},//
	{0x3645 ,0x13,"0x0100",eReadWrite},//
	{0x3646 ,0x83,"0x0100",eReadWrite},//
	{0x364a ,0x07,"0x0100",eReadWrite},//
	{0x3015 ,0x01,"0x0100",eReadWrite},// ;
	{0x3018 ,0x12,"0x0100",eReadWrite},//;	MIPI 1 lane
	{0x3020 ,0x93,"0x0100",eReadWrite},// ; Clock switch output normal, pclk_div =/1
	{0x3022 ,0x01,"0x0100",eReadWrite},// ; pd_mipi enable when rst_sync
	{0x3031 ,0x0a,"0x0100",eReadWrite},// ; MIPI 10-bit mode
	{0x3034 ,0x00,"0x0100",eReadWrite},//
	{0x3106 ,0x01,"0x0100",eReadWrite},// ; sclk_div, sclk_pre_div
	{0x3305 ,0xf1,"0x0100",eReadWrite},//
	{0x3308 ,0x00,"0x0100",eReadWrite},//
	{0x3309 ,0x28,"0x0100",eReadWrite},//
	{0x330a ,0x00,"0x0100",eReadWrite},//
	{0x330b ,0x20,"0x0100",eReadWrite},//
	{0x330c ,0x00,"0x0100",eReadWrite},//
	{0x330d ,0x00,"0x0100",eReadWrite},//
	{0x330e ,0x00,"0x0100",eReadWrite},//
	{0x330f ,0x40,"0x0100",eReadWrite},//
	{0x3307 ,0x04,"0x0100",eReadWrite},//
	{0x3500 ,0x00,"0x0100",eReadWrite},// ; exposure H
	{0x3501 ,0x4d,"0x0100",eReadWrite},// ; exposure M
	{0x3502 ,0x40,"0x0100",eReadWrite},// ; exposure L
	{0x3503 ,0x00,"0x0100",eReadWrite},// ; gain delay 1 frame, exposure delay 1 frame, real gain
	{0x3505 ,0x80,"0x0100",eReadWrite},// ; gain option
	{0x3508 ,0x04,"0x0100",eReadWrite},// ; gain H
	{0x3509 ,0x00,"0x0100",eReadWrite},// ; gain L
	{0x350c ,0x00,"0x0100",eReadWrite},// ; short gain H
	{0x350d ,0x80,"0x0100",eReadWrite},// ; short gain L
	{0x3510 ,0x00,"0x0100",eReadWrite},// ; short exposure H
	{0x3511 ,0x02,"0x0100",eReadWrite},// ; short exposure M
	{0x3512 ,0x00,"0x0100",eReadWrite},// ; short exposure L
	{0x3700 ,0x18,"0x0100",eReadWrite},//
	{0x3701 ,0x0c,"0x0100",eReadWrite},//
	{0x3702 ,0x28,"0x0100",eReadWrite},//
	{0x3703 ,0x19,"0x0100",eReadWrite},//
	{0x3704 ,0x14,"0x0100",eReadWrite},//
	{0x3705 ,0x00,"0x0100",eReadWrite},//
	{0x3706 ,0x35,"0x0100",eReadWrite},//
	{0x3707 ,0x04,"0x0100",eReadWrite},//
	{0x3708 ,0x24,"0x0100",eReadWrite},//
	{0x3709 ,0x33,"0x0100",eReadWrite},//
	{0x370a ,0x00,"0x0100",eReadWrite},//
	{0x370b ,0xb5,"0x0100",eReadWrite},//
	{0x370c ,0x04,"0x0100",eReadWrite},//
	{0x3718 ,0x12,"0x0100",eReadWrite},//
	{0x3719 ,0x31,"0x0100",eReadWrite},//
	{0x3712 ,0x42,"0x0100",eReadWrite},//
	{0x3714 ,0x24,"0x0100",eReadWrite},//
	{0x371e ,0x19,"0x0100",eReadWrite},//
	{0x371f ,0x40,"0x0100",eReadWrite},//
	{0x3720 ,0x05,"0x0100",eReadWrite},//
	{0x3721 ,0x05,"0x0100",eReadWrite},//
	{0x3724 ,0x06,"0x0100",eReadWrite},//
	{0x3725 ,0x01,"0x0100",eReadWrite},//
	{0x3726 ,0x06,"0x0100",eReadWrite},//
	{0x3728 ,0x05,"0x0100",eReadWrite},//
	{0x3729 ,0x02,"0x0100",eReadWrite},//
	{0x372a ,0x03,"0x0100",eReadWrite},//
	{0x372b ,0x53,"0x0100",eReadWrite},//
	{0x372c ,0xa3,"0x0100",eReadWrite},//
	{0x372d ,0x53,"0x0100",eReadWrite},//
	{0x372e ,0x06,"0x0100",eReadWrite},//
	{0x372f ,0x10,"0x0100",eReadWrite},//
	{0x3730 ,0x01,"0x0100",eReadWrite},//
	{0x3731 ,0x06,"0x0100",eReadWrite},//
	{0x3732 ,0x14,"0x0100",eReadWrite},//
	{0x3733 ,0x10,"0x0100",eReadWrite},//
	{0x3734 ,0x40,"0x0100",eReadWrite},//
	{0x3736 ,0x20,"0x0100",eReadWrite},//
	{0x373a ,0x05,"0x0100",eReadWrite},//
	{0x373b ,0x06,"0x0100",eReadWrite},//
	{0x373c ,0x0a,"0x0100",eReadWrite},//
	{0x373e ,0x03,"0x0100",eReadWrite},//
	{0x3755 ,0x10,"0x0100",eReadWrite},//
	{0x3758 ,0x00,"0x0100",eReadWrite},//
	{0x3759 ,0x4c,"0x0100",eReadWrite},//
	{0x375a ,0x06,"0x0100",eReadWrite},//
	{0x375b ,0x13,"0x0100",eReadWrite},//
	{0x375c ,0x20,"0x0100",eReadWrite},//
	{0x375d ,0x02,"0x0100",eReadWrite},//
	{0x375e ,0x00,"0x0100",eReadWrite},//
	{0x375f ,0x14,"0x0100",eReadWrite},//
	{0x3768 ,0x22,"0x0100",eReadWrite},//
	{0x3769 ,0x44,"0x0100",eReadWrite},//
	{0x376a ,0x44,"0x0100",eReadWrite},//
	{0x3761 ,0x00,"0x0100",eReadWrite},//
	{0x3762 ,0x00,"0x0100",eReadWrite},//
	{0x3763 ,0x00,"0x0100",eReadWrite},//
	{0x3766 ,0xff,"0x0100",eReadWrite},//
	{0x376b ,0x00,"0x0100",eReadWrite},//
	{0x3772 ,0x23,"0x0100",eReadWrite},//
	{0x3773 ,0x02,"0x0100",eReadWrite},//
	{0x3774 ,0x16,"0x0100",eReadWrite},//
	{0x3775 ,0x12,"0x0100",eReadWrite},//
	{0x3776 ,0x04,"0x0100",eReadWrite},//
	{0x3777 ,0x00,"0x0100",eReadWrite},//
	{0x3778 ,0x1b,"0x0100",eReadWrite},//
	{0x37a0 ,0x44,"0x0100",eReadWrite},//
	{0x37a1 ,0x3d,"0x0100",eReadWrite},//
	{0x37a2 ,0x3d,"0x0100",eReadWrite},//
	{0x37a3 ,0x00,"0x0100",eReadWrite},//
	{0x37a4 ,0x00,"0x0100",eReadWrite},//
	{0x37a5 ,0x00,"0x0100",eReadWrite},//
	{0x37a6 ,0x00,"0x0100",eReadWrite},//
	{0x37a7 ,0x44,"0x0100",eReadWrite},//
	{0x37a8 ,0x4c,"0x0100",eReadWrite},//
	{0x37a9 ,0x4c,"0x0100",eReadWrite},//
	{0x3760 ,0x00,"0x0100",eReadWrite},//
	{0x376f ,0x01,"0x0100",eReadWrite},//
	{0x37aa ,0x44,"0x0100",eReadWrite},//
	{0x37ab ,0x2e,"0x0100",eReadWrite},//
	{0x37ac ,0x2e,"0x0100",eReadWrite},//
	{0x37ad ,0x33,"0x0100",eReadWrite},//
	{0x37ae ,0x0d,"0x0100",eReadWrite},//
	{0x37af ,0x0d,"0x0100",eReadWrite},//
	{0x37b0 ,0x00,"0x0100",eReadWrite},//
	{0x37b1 ,0x00,"0x0100",eReadWrite},//
	{0x37b2 ,0x00,"0x0100",eReadWrite},//
	{0x37b3 ,0x42,"0x0100",eReadWrite},//
	{0x37b4 ,0x42,"0x0100",eReadWrite},//
	{0x37b5 ,0x33,"0x0100",eReadWrite},//
	{0x37b6 ,0x00,"0x0100",eReadWrite},//
	{0x37b7 ,0x00,"0x0100",eReadWrite},//
	{0x37b8 ,0x00,"0x0100",eReadWrite},//
	{0x37b9 ,0xff,"0x0100",eReadWrite},//
	{0x3800 ,0x00,"0x0100",eReadWrite},// ; x start H
	{0x3801 ,0x0c,"0x0100",eReadWrite},// ; x start L
	{0x3802 ,0x00,"0x0100",eReadWrite},// ; y start H
	{0x3803 ,0x0c,"0x0100",eReadWrite},//,"0x0100",eReadWrite},//y start L
	{0x3804 ,0x0c,"0x0100",eReadWrite},//x end H
	{0x3805 ,0xd3,"0x0100",eReadWrite},//x end L
	{0x3806 ,0x09,"0x0100",eReadWrite},//y end H
	{0x3807 ,0xa3,"0x0100",eReadWrite},//y end L
	{0x3808 ,0x06,"0x0100",eReadWrite},//x output size H
	{0x3809 ,0x60,"0x0100",eReadWrite},//x output size L
	{0x380a ,0x04,"0x0100",eReadWrite},//y output size H
	{0x380b ,0xc8,"0x0100",eReadWrite},//y output size L
	{0x380c ,0x0f,"0x0100",eReadWrite},//;07, 03,"0x0100",eReadWrite},//HTS H
	{0x380d ,0x10,"0x0100",eReadWrite},//88,c4,"0x0100",eReadWrite},//HTS L
	{0x380e ,0x04,"0x0100",eReadWrite},//VTS H
	{0x380f ,0xdc,"0x0100",eReadWrite},//VTS L
	{0x3810 ,0x00,"0x0100",eReadWrite},//ISP x win H
	{0x3811 ,0x04,"0x0100",eReadWrite},//ISP x win L
	{0x3813 ,0x02,"0x0100",eReadWrite},//ISP y win L
	{0x3814 ,0x03,"0x0100",eReadWrite},//x odd inc
	{0x3815 ,0x01,"0x0100",eReadWrite},//x even inc
	{0x3820 ,0x00,"0x0100",eReadWrite},//vflip off
	{0x3821 ,0x67,"0x0100",eReadWrite},//mirror on, bin on
	{0x382a ,0x03,"0x0100",eReadWrite},//y odd inc
	{0x382b ,0x01,"0x0100",eReadWrite},//y even inc
	{0x3830 ,0x08,"0x0100",eReadWrite},//
	{0x3836 ,0x02,"0x0100",eReadWrite},//
	{0x3837 ,0x18,"0x0100",eReadWrite},//
	{0x3841 ,0xff,"0x0100",eReadWrite},//window auto size enable
	{0x3846 ,0x48,"0x0100",eReadWrite},//
	{0x3d85 ,0x14,"0x0100",eReadWrite},//OTP power up load data enable, OTP powerr up load setting disable
	{0x3f08 ,0x08,"0x0100",eReadWrite},//
	{0x3f0a ,0x80,"0x0100",eReadWrite},//
	{0x4000 ,0xf1,"0x0100",eReadWrite},//out_range_trig, format_chg_trig, gain_trig, exp_chg_trig, median filter enable
	{0x4001 ,0x10,"0x0100",eReadWrite},//total 128 black column
	{0x4005 ,0x10,"0x0100",eReadWrite},//BLC target L
	{0x4002 ,0x27,"0x0100",eReadWrite},//value used to limit BLC offset
	{0x4009 ,0x81,"0x0100",eReadWrite},//final BLC offset limitation enable
	{0x400b ,0x0c,"0x0100",eReadWrite},//DCBLC on, DCBLC manual mode on
	{0x401b ,0x00,"0x0100",eReadWrite},//zero line R coefficient
	{0x401d ,0x00,"0x0100",eReadWrite},//zoro line T coefficient
	{0x4020 ,0x00,"0x0100",eReadWrite},//Anchor left start H
	{0x4021 ,0x04,"0x0100",eReadWrite},//Anchor left start L
	{0x4022 ,0x04,"0x0100",eReadWrite},//Anchor left end H
	{0x4023 ,0xb9,"0x0100",eReadWrite},//Anchor left end L
	{0x4024 ,0x05,"0x0100",eReadWrite},//Anchor right start H
	{0x4025 ,0x2a,"0x0100",eReadWrite},//Anchor right start L
	{0x4026 ,0x05,"0x0100",eReadWrite},//Anchor right end H
	{0x4027 ,0x2b,"0x0100",eReadWrite},//Anchor right end L
	{0x4028 ,0x00,"0x0100",eReadWrite},//top zero line start
	{0x4029 ,0x02,"0x0100",eReadWrite},//top zero line number
	{0x402a ,0x04,"0x0100",eReadWrite},//top black line start
	{0x402b ,0x04,"0x0100",eReadWrite},//top black line number
	{0x402c ,0x02,"0x0100",eReadWrite},//bottom zero line start
	{0x402d ,0x02,"0x0100",eReadWrite},//bottom zoro line number
	{0x402e ,0x08,"0x0100",eReadWrite},//bottom black line start
	{0x402f ,0x02,"0x0100",eReadWrite},//bottom black line number
	{0x401f ,0x00,"0x0100",eReadWrite},//interpolation x disable, interpolation y disable, Anchor one disable
	{0x4034 ,0x3f,"0x0100",eReadWrite},//
	{0x403d ,0x04,"0x0100",eReadWrite},//md_precison_en
	{0x4300 ,0xff,"0x0100",eReadWrite},//clip max H
	{0x4301 ,0x00,"0x0100",eReadWrite},//clip min H
	{0x4302 ,0x0f,"0x0100",eReadWrite},//clip min L, clip max L
	{0x4316 ,0x00,"0x0100",eReadWrite},//
	{0x4500 ,0x38,"0x0100",eReadWrite},//
	{0x4503 ,0x18,"0x0100",eReadWrite},//
	{0x4600 ,0x00,"0x0100",eReadWrite},//
	{0x4601 ,0xcb,"0x0100",eReadWrite},//
	{0x481f ,0x32,"0x0100",eReadWrite},//clk prepare min
	{0x4837 ,0x16,"0x0100",eReadWrite},//global timing
	{0x4850 ,0x10,"0x0100",eReadWrite},//lane 1 = 1, lane 0 = 0
	{0x4851 ,0x32,"0x0100",eReadWrite},//lane 3 = 3, lane 2 = 2
	{0x4b00 ,0x2a,"0x0100",eReadWrite},//
	{0x4b0d ,0x00,"0x0100",eReadWrite},//
	{0x4d00 ,0x04,"0x0100",eReadWrite},//temperature sensor
	{0x4d01 ,0x18,"0x0100",eReadWrite},// ;
	{0x4d02 ,0xc3,"0x0100",eReadWrite},// ;
	{0x4d03 ,0xff,"0x0100",eReadWrite},// ;
	{0x4d04 ,0xff,"0x0100",eReadWrite},// ;
	{0x4d05 ,0xff,"0x0100",eReadWrite},//temperature sensor
	{0x5000 ,0x7e,"0x0100",eReadWrite},//slave AWB gain enable, slave AWB statistics enable, master AWB gain enable,master AWB statistics enable, BPC on, WPC on
	{0x5001 ,0x01,"0x0100",eReadWrite},//BLC on
	{0x5002 ,0x08,"0x0100",eReadWrite},//H scale off, WBMATCH select slave sensor's gain, WBMATCH off, OTP_DPCoff,
	{0x5003 ,0x20,"0x0100",eReadWrite},//DPC_DBC buffer control enable, WB
	{0x5046 ,0x12,"0x0100",eReadWrite},//
	{0x5780 ,0xfc,"0x0100",eReadWrite},//tail en, saturate cross cluster en, remove corss cluster en,; remove connected dp in same channel,; en remove connected dp in different channel, smooth en
	{0x5784 ,0x0c,"0x0100",eReadWrite},//black threshold
	{0x5787 ,0x40,"0x0100",eReadWrite},//thre3
	{0x5788 ,0x08,"0x0100",eReadWrite},//thre4
	{0x578a ,0x02,"0x0100",eReadWrite},//wthre_list1
	{0x578b ,0x01,"0x0100",eReadWrite},//wthre_list2
	{0x578c ,0x01,"0x0100",eReadWrite},//wthre_list3
	{0x578e ,0x02,"0x0100",eReadWrite},//bthre_list1
	{0x578f ,0x01,"0x0100",eReadWrite},//bthre_list2
	{0x5790 ,0x01,"0x0100",eReadWrite},//bthre_list3
	{0x5901 ,0x00,"0x0100",eReadWrite},//H skip off, V skip off
	{0x5b00 ,0x02,"0x0100",eReadWrite},//OTP DPC start address
	{0x5b01 ,0x10,"0x0100",eReadWrite},//OTP DPC start address
	{0x5b02 ,0x03,"0x0100",eReadWrite},//OTP DPC end address
	{0x5b03 ,0xcf,"0x0100",eReadWrite},//OTP DPC end address
	{0x5b05 ,0x6c,"0x0100",eReadWrite},//recover method = 2b11, use fixed pattern to recover cluster,; use 0x3ff to recover cluster
	{0x5e00 ,0x00,"0x0100",eReadWrite},//test pattern off
	{0x5e01 ,0x41,"0x0100",eReadWrite},//window cut enable
	{0x382d ,0x7f,"0x0100",eReadWrite},//
	{0x4825 ,0x3a,"0x0100",eReadWrite},//lpx_p_min
	{0x4826 ,0x40,"0x0100",eReadWrite},//hs_prepare_min
	{0x4808 ,0x25,"0x0100",eReadWrite},//wake up delay in 1/1024 s
	//{0x0100 01,"0x0100",eReadWrite},//
	{0x0000 ,0x00,"eTableEnd",eTableEnd}

};
const IsiRegDescription_t HM8040_g_1632x1224_onelane[] =
{

//5.1.2.2 Raw 10bit 1632x1224 15fps 1lane 720M bps/lane
	//;; MIPI=720Mbps, SysClk=144Mhz,Dac Clock=360Mhz.
	{0x0100 ,0x00,"0x0100",eReadWrite},
	{0x030e ,0x00,"0x0100",eReadWrite},//pll2_rdiv
	{0x030f ,0x09,"0x0100",eReadWrite},//pll2_divsp
	{0x0312 ,0x01,"0x0100",eReadWrite},//pll2_pre_div0, pll2_r_divdac
	{0x3015 ,0x01,"0x0100",eReadWrite},
	{0x3501 ,0x4d,"0x0100",eReadWrite},//exposure M
	{0x3502 ,0x40,"0x0100",eReadWrite},//exposure L
	//{0x3508 ,0x04,"0x0100",eReadWrite},//gain H
	{0x3706 ,0x35,"0x0100",eReadWrite},
	{0x370a ,0x00,"0x0100",eReadWrite},
	{0x370b ,0xb5,"0x0100",eReadWrite},
	{0x3778 ,0x1b,"0x0100",eReadWrite},
	{0x3808 ,0x06,"0x0100",eReadWrite},//x output size H
	{0x3809 ,0x60,"0x0100",eReadWrite},//x output size L
	{0x380a ,0x04,"0x0100",eReadWrite},//y output size H
	{0x380b ,0xc8,"0x0100",eReadWrite},//y output size L
	{0x380c ,0x0f,"0x0100",eReadWrite},//HTS H
	{0x380d ,0x10,"0x0100",eReadWrite},//HTS L
	{0x380e ,0x04,"0x0100",eReadWrite},//VTS H
	{0x380f ,0xdc,"0x0100",eReadWrite},//VTS L
	{0x3814 ,0x03,"0x0100",eReadWrite},//x odd inc
	{0x3821 ,0x67,"0x0100",eReadWrite},//mirror on, bin on
	{0x382a ,0x03,"0x0100",eReadWrite},//y odd inc
	{0x3830 ,0x08,"0x0100",eReadWrite},
	{0x3836 ,0x02,"0x0100",eReadWrite},
	{0x3f0a ,0x80,"0x0100",eReadWrite},
	{0x4001 ,0x10,"0x0100",eReadWrite},//total 128 black column
	{0x4022 ,0x04,"0x0100",eReadWrite},//Anchor left end H
	{0x4023 ,0xb9,"0x0100",eReadWrite},//Anchor left end L
	{0x4024 ,0x05,"0x0100",eReadWrite},//Anchor right start H
	{0x4025 ,0x2a,"0x0100",eReadWrite},//Anchor right start L
	{0x4026 ,0x05,"0x0100",eReadWrite},//Anchor right end H
	{0x4027 ,0x2b,"0x0100",eReadWrite},//Anchor right end L
	{0x402b ,0x04,"0x0100",eReadWrite},//top black line number
	{0x402e ,0x08,"0x0100",eReadWrite},//bottom black line start
	{0x4500 ,0x38,"0x0100",eReadWrite},
	{0x4600 ,0x00,"0x0100",eReadWrite},
	{0x4601 ,0xcb,"0x0100",eReadWrite},
	{0x382d ,0x7f,"0x0100",eReadWrite},
	//{0x0100 ,0x01,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448_onelane[] =
{
	//@@5.1.2.3 Raw 10bit 3264x2448 7.5fps 1lane 720M bps/lane
	//;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.
	{0x0100 ,0x00,"0x0100",eReadWrite},//
	{0x030e ,0x02,"0x0100",eReadWrite},// ; pll2_rdiv
	{0x030f ,0x04,"0x0100",eReadWrite},// ; pll2_divsp
	{0x0312 ,0x03,"0x0100",eReadWrite},// ; pll2_pre_div0, pll2_r_divdac
	{0x3015 ,0x00,"0x0100",eReadWrite},//
	{0x3501 ,0x9a,"0x0100",eReadWrite},//
	{0x3502 ,0x20,"0x0100",eReadWrite},//
//{0x3508 ,0x02,"0x0100",eReadWrite},//
	{0x3706 ,0x6a,"0x0100",eReadWrite},//
	{0x370a ,0x01,"0x0100",eReadWrite},//
	{0x370b ,0x6a,"0x0100",eReadWrite},//
	{0x3778 ,0x32,"0x0100",eReadWrite},//
	{0x3808 ,0x0c,"0x0100",eReadWrite},// ; x output size H
	{0x3809 ,0xc0,"0x0100",eReadWrite},// ; x output size L
	{0x380a ,0x09,"0x0100",eReadWrite},// ; y output size H
	{0x380b ,0x90,"0x0100",eReadWrite},// ; y output size L
	{0x380c ,0x0f,"0x0100",eReadWrite},//;07 ; HTS H
	{0x380d ,0x28,"0x0100",eReadWrite},//;94 ; HTS L
	{0x380e ,0x09,"0x0100",eReadWrite},//; VTS H
	{0x380f ,0xaa,"0x0100",eReadWrite},// ; VTS L
	{0x3814 ,0x01,"0x0100",eReadWrite},// ; x odd inc
	{0x3821 ,0x46,"0x0100",eReadWrite},// ; mirror on, bin off
	{0x382a ,0x01,"0x0100",eReadWrite},// ; y odd inc
	{0x3830 ,0x06,"0x0100",eReadWrite},//
	{0x3836 ,0x01,"0x0100",eReadWrite},//
	{0x3f0a ,0x00,"0x0100",eReadWrite},//
	{0x4001 ,0x00,"0x0100",eReadWrite},// ; total 256 black column
	{0x4022 ,0x0b,"0x0100",eReadWrite},// ; Anchor left end H
	{0x4023 ,0xc3,"0x0100",eReadWrite},// ; Anchor left end L
	{0x4024 ,0x0c,"0x0100",eReadWrite},// ; Anchor right start H
	{0x4025 ,0x36,"0x0100",eReadWrite},// ; Anchor right start L
	{0x4026 ,0x0c,"0x0100",eReadWrite},// ; Anchor right end H
	{0x4027 ,0x37,"0x0100",eReadWrite},// ; Anchor right end L
	{0x402b ,0x08,"0x0100",eReadWrite},// ; top black line number
	{0x402e ,0x0c,"0x0100",eReadWrite},// ; bottom black line start
	{0x4500 ,0x58,"0x0100",eReadWrite},//
	{0x4600 ,0x01,"0x0100",eReadWrite},//
	{0x4601 ,0x97,"0x0100",eReadWrite},//
	{0x382d ,0xff,"0x0100",eReadWrite},//
	//{0x0100 ,0x01,"0x0100",eReadWrite},//
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_aRegDescription_twolane[] =
{
	//------------------------------------------------------------------------
	// Initial HM8040
	//------------------------------------------------------------------------
	{0x0103, 0x00, "0x0100",eReadWrite}, //software reset
	{0x3506, 0x01, "0x0100",eReadWrite}, //LSC autoload
	{0xBA90, 0x01, "0x0100",eReadWrite}, //Enable OTP load to user space
	{0x4405, 0x24, "0x0100",eReadWrite},
	{0x0100, 0x02, "0x0100",eReadWrite}, //power up

	//{0x4001, 0x80, "0x0100",eReadWrite}, //ck_cfg - choose mclk_pll
		
	//------------------------------------------------------------------------
	// PLL HM8040
	//------------------------------------------------------------------------
	// PCLK (RAW)
	{0x0303, 0x01, "0x0100",eReadWrite}, //CLK_DIV, 1:divide by 1, 2:divide by 2
	{0x0305, 0x0C, "0x0100",eReadWrite}, //PLL N,	 N = 12, while mclk 24mhz
	{0x0307, 0x46, "0x0100",eReadWrite}, //PLL M,	 M = 70, pclk_raw=(mclk*(M[6:0]/N[4:0])  )/(  CLK_DIV)
	// PKTCLK (MIPI)
	{0x0309, 0x01, "0x0100",eReadWrite}, //CLK_DIV, 1:divide by 1, 2:divide by 2
	{0x030D, 0x0C, "0x0100",eReadWrite}, //PLL N,	 N = 12, while mclk 24mhz
	//{0x030F, 0xBC, "0x0100",eReadWrite}, //PLL M,	 M = 188, pkt_clk =(mclk*(M[6:0]/N[4:0])*2)/(4*CLK_DIV)
	{0x030F, 0xC8, "0x0100",eReadWrite},
	
	{0x4287, 0x01, "0x0100",eReadWrite}, //Enable UHS mode if packet clock is larger than 300 MHz
	//------------------------------------------------------------------------
	// Analog for HM8040
	//------------------------------------------------------------------------
	{0x4131, 0x01, "0x0100",eReadWrite}, // Digital BLI Enable
	{0x4132, 0x20, "0x0100",eReadWrite}, // Digital BLI Target
	{0x4140, 0x20, "0x0100",eReadWrite}, // BLC Target
	{0x4144, 0x03, "0x0100",eReadWrite}, // BLC ON
	
	{0x4145, 0x33, "0x0100",eReadWrite}, // CFPN ON
	{0x4146, 0x31, "0x0100",eReadWrite}, // CFPN IIR
	{0x4147, 0x03, "0x0100",eReadWrite}, // CFPN control
	{0x4149, 0x60, "0x0100",eReadWrite}, // BLC 4x2 channel + Dithering enable (06/11)
	{0x414A, 0x00, "0x0100",eReadWrite}, // BLC control
	
	{0x4261, 0x00, "0x0100",eReadWrite}, 
	{0x4262, 0x01, "0x0100",eReadWrite},
	{0x4264, 0x0A, "0x0100",eReadWrite}, 
	
	{0x4012, 0x00, "0x0100",eReadWrite}, // CDS Timing 
	
	{0x4270, 0x22, "0x0100",eReadWrite}, // CDS Timing 
	{0x4271, 0x68, "0x0100",eReadWrite}, // CDS Timing
	{0x4272, 0x43, "0x0100",eReadWrite}, // CDS Timing
	{0x4273, 0x44, "0x0100",eReadWrite}, // CDS Timing
	
	{0x4280, 0x10, "0x0100",eReadWrite}, // CDS Timing
	{0x4281, 0x10, "0x0100",eReadWrite}, // CDS Timing
	{0x4283, 0x07, "0x0100",eReadWrite}, // ADC Timing
	{0x4284, 0x00, "0x0100",eReadWrite}, // CDS Timing
	{0x4285, 0x0C, "0x0100",eReadWrite}, // CDS Timing
	
	{0x4380, 0xA6, "0x0100",eReadWrite}, // 
	{0x4381, 0x71, "0x0100",eReadWrite}, 
	{0x4383, 0x71, "0x0100",eReadWrite},
	{0x4384, 0x05, "0x0100",eReadWrite},
	{0x4385, 0xA2, "0x0100",eReadWrite}, // RAMP OUTPUT LEVEL -> set to A5 if 438E = 00
	{0x438A, 0x80, "0x0100",eReadWrite}, // [7]: CA bias current 
	{0x438B, 0x80, "0x0100",eReadWrite},
	{0x438C, 0x80, "0x0100",eReadWrite}, // Pixel VDD level 
	
	{0x4400, 0x36, "0x0100",eReadWrite}, // CA reset clamp control
	{0x4401, 0x36, "0x0100",eReadWrite},
	{0x4404, 0x01, "0x0100",eReadWrite}, // Dark clamp extend
	
	{0x45FC, 0x02, "0x0100",eReadWrite},
	{0x45FD, 0x00, "0x0100",eReadWrite}, // Pixel VDD switch control
	{0x45FE, 0x10, "0x0100",eReadWrite}, // Pixel VDD switch control
	
	{0x4490, 0xE7, "0x0100",eReadWrite}, //1x
	{0x4491, 0xE3, "0x0100",eReadWrite}, //2x 
	{0x4492, 0xE1, "0x0100",eReadWrite}, //4x
	{0x4493, 0xE0, "0x0100",eReadWrite}, //8x 
	{0x4494, 0xE8, "0x0100",eReadWrite}, //16x 
	{0x4495, 0xF8, "0x0100",eReadWrite}, //32x 
	
	{0x44D0, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 1
	{0x44D1, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 2 
	{0x44D2, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 4 
	{0x44D3, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 8 
	{0x44D4, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 16 
	{0x44D5, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 32 
	
	{0x44A0, 0xE7, "0x0100",eReadWrite},   ///	binning gain table
	{0x44A1, 0xE3, "0x0100",eReadWrite},
	{0x44A2, 0xE1, "0x0100",eReadWrite},
	{0x44A3, 0xE0, "0x0100",eReadWrite},
	{0x44A4, 0xE8, "0x0100",eReadWrite},
	{0x44A5, 0xF8, "0x0100",eReadWrite},
	
	{0x44B0, 0xE1, "0x0100",eReadWrite},   ///	summing gain table
	{0x44B1, 0xE0, "0x0100",eReadWrite},
	{0x44B2, 0xE8, "0x0100",eReadWrite},
	{0x44B3, 0xE8, "0x0100",eReadWrite},
	{0x44B4, 0xF8, "0x0100",eReadWrite},
	{0x44B5, 0xF8, "0x0100",eReadWrite},
	
	{0x4600, 0x00, "0x0100",eReadWrite},   ///	summing enable (sum)
	{0x4601, 0x00, "0x0100",eReadWrite},
	{0x4602, 0x00, "0x0100",eReadWrite},
	{0x4603, 0x01, "0x0100",eReadWrite},
	{0x4604, 0x01, "0x0100",eReadWrite},
	{0x4605, 0x01, "0x0100",eReadWrite},
	//end of Analog
	//---------------------------------------------------------
	
	//------------------------------------------------------------------------
	// Digital
	//------------------------------------------------------------------------
	//{0x4149, 0x5C, "0x0100",eReadWrite},
	//{0x414A, 0x02, "0x0100",eReadWrite},
	//{0x400D, 0x04, "0x0100",eReadWrite},
	{0x3110, 0x01, "0x0100",eReadWrite},
	
	//------------------------------------------------------------------------
	// Frame Timing
	//------------------------------------------------------------------------
	{0x0340, 0x09, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0xD6, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 1294= 2448+70)
	{0x0342, 0x0E, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x7A, "0x0100",eReadWrite}, //smia Line length Lb 	(7212)
	
	//------------------------------------------------------------------------
	// Image Size
	//------------------------------------------------------------------------
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (   0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end   of x Hb		
	{0x0349, 0xBF, "0x0100",eReadWrite}, //end   of x Lb (3263)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (   0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end   of y Hb		
	{0x034B, 0x8F, "0x0100",eReadWrite}, //end   of y Lb (2447)
	
	//------------------------------------------------------------------------
	// Output Size / Digiatl Window
	//------------------------------------------------------------------------
	{0x034C, 0x0C, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0xC0, "0x0100",eReadWrite}, //output size of x Lb (3264)
	{0x034E, 0x09, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0x90, "0x0100",eReadWrite}, //output size of y Lb (2448)
	
	//{0x0350, 0x73, "0x0100",eReadWrite},
	
	//------------------------------------------------------------------------
	// MIPI
	//------------------------------------------------------------------------
	{0x0111, 0x01, "0x0100",eReadWrite}, //0:2lane, 1:4lane 
	{0x0112, 0x0A, "0x0100",eReadWrite}, //DataFormat : 0x08 : RAW8, 0x0A : RAW10	
	{0x0113, 0x0A, "0x0100",eReadWrite}, //DataFormat : 0x08 : RAW8, 0x0A : RAW10 
	{0x4B01, 0x2B, "0x0100",eReadWrite}, //DataFormat : 0x2A : RAW8, 0x2B : RAW10 
	{0x4B20, 0xDE, "0x0100",eReadWrite}, //CC0 : Clock lane on always. 			 (9E) / woLSLE (8E) 
					   				     //GC2 : Clock lane on while frame active.  (DE) / woLSLE (CE) 
					   					 //GC1 : Clock lane on while packet issued. (BE) / woLSLE (AE) 
					   					 //GC3 : Clock lane on while line	active.  (FE) / woLSLE (EE) 
	
	{0x4B18, 0x25, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	{0x4B19, 0x60, "0x0100",eReadWrite},
	
	{0x4B02, 0x04, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x05, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x18, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0E, 0x00, "0x0100",eReadWrite}, //CLK_FRONT_PORCH (CLK_PRE)
	{0x4B0F, 0x08, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x05, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x09, "0x0100",eReadWrite}, //CLK_EXIT
	
	{0x4B42, 0x06, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x09, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x05, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x09, "0x0100",eReadWrite}, //HS_EXIT
	
	{0x4B57, 0x03, "0x0100",eReadWrite}, //Dynamic Range Setting for Dynamic Trigger Value Correction, 0:no correction, 1:-63~+63, 2:-127~+127, 3:-255~+255, 
	{0x4B51, 0x80, "0x0100",eReadWrite}, //enabled MIPI Status Monitor 
	{0x4B52, 0xC9, "0x0100",eReadWrite}, //enabled Dynamic Trigger Value Correction 
	
	{0x4B11, 0x0F, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B45, 0x00, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B46, 0x1F, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B47, 0xC7, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B3B, 0x13, "0x0100",eReadWrite}, //adjust slew rate parameters
	
	{0x4024, 0x40, "0x0100",eReadWrite}, //Enabled MIPI

	{0x0101, 0x03, "0x0100",eReadWrite}, //mirror
	//---------------------------------------------------
	// CMU update for HM8040
	//---------------------------------------------------
	{0x0104, 0x00, "0x0100",eReadWrite}, 
	{0x0000, 0x0A, "0x0100",eDelay},	//delay 10 ms
	
	{0x0100, 0x01, "0x0100",eReadWrite}, 
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224_twolane[] =
{	
	{0x0100, 0x00, "0x0100",eReadWrite},
	{0x0000, 0x0A, "0x0100",eDelay},

	{0x0303, 0x02, "0x0100",eReadWrite}, //70M pclk
	//{0x0309, 0x02, "0x0100",eReadWrite},
	//{0x4287, 0x00, "0x0100",eReadWrite},

	{0x0340, 0x05, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0x0E, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 1294= 1224+70)
	{0x0342, 0x0E, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x16, "0x0100",eReadWrite}, //smia Line length Lb 	(7212)//3606
	
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (	 0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end	 of x Hb		
	{0x0349, 0xBD, "0x0100",eReadWrite}, //end	 of x Lb (3261)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (	 0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end	 of y Hb		
	{0x034B, 0x8D, "0x0100",eReadWrite}, //end	 of y Lb (2445)
	
	{0x034C, 0x06, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0x60, "0x0100",eReadWrite}, //output size of x Lb (1632)
	{0x034E, 0x04, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0xC8, "0x0100",eReadWrite}, //output size of y Lb (1224)
											
	{0x0383, 0x03, "0x0100",eReadWrite}, //x odd increment
	{0x0387, 0x03, "0x0100",eReadWrite}, //y odd increment
	{0x0390, 0x03, "0x0100",eReadWrite}, //binning enable

	{0x0111, 0x00, "0x0100",eReadWrite},

	{0x4B18, 0x1B, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	
	{0x4B02, 0x02, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x03, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x0B, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0F, 0x09, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x02, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x04, "0x0100",eReadWrite}, //CLK_EXIT
	
	{0x4B42, 0x03, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x05, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x03, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x04, "0x0100",eReadWrite}, //HS_EXIT

	{0x4B46, 0x7F, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B47, 0xC0, "0x0100",eReadWrite},
	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t HM8040_g_1632x1224P30_twolane_fpschg[] =
{    
	{0x0340, 0x05,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x0E,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P25_twolane_fpschg[] =
{  	
	{0x0340, 0x06,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x10,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};


const IsiRegDescription_t HM8040_g_1632x1224P20_twolane_fpschg[] =
{
    {0x0340, 0x07,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xC6,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P15_twolane_fpschg[] =
{
		
	{0x0340, 0x0a,"0x0100",eReadWrite}, // VTS H	
	{0x0341, 0x30,"0x0100",eReadWrite}, // VTS L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P10_twolane_fpschg[] =
{ 
	{0x0340, 0x0f,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xcc,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448_twolane[] =
{								
	{0x0100, 0x00, "0x0100",eReadWrite},
	{0x0000, 0x0A, "0x0100",eDelay},

	{0x0303, 0x01, "0x0100",eReadWrite}, //140M pclk
	//{0x0309, 0x02, "0x0100",eReadWrite},
	//{0x4287, 0x01, "0x0100",eReadWrite},

	{0x0340, 0x09, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0xD6, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 2518= 2448+70)
	{0x0342, 0x0E, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x7A, "0x0100",eReadWrite}, //smia Line length Lb 	(3706)
	
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (   0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end   of x Hb		
	{0x0349, 0xBF, "0x0100",eReadWrite}, //end   of x Lb (3263)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (   0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end   of y Hb		
	{0x034B, 0x8F, "0x0100",eReadWrite}, //end   of y Lb (2447)
	
	{0x034C, 0x0C, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0xC0, "0x0100",eReadWrite}, //output size of x Lb (3264)
	{0x034E, 0x09, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0x90, "0x0100",eReadWrite}, //output size of y Lb (2448)
											
	{0x0383, 0x01, "0x0100",eReadWrite}, //x odd increment
	{0x0387, 0x01, "0x0100",eReadWrite}, //y odd increment
	{0x0390, 0x00, "0x0100",eReadWrite}, //binning enable

	{0x0111, 0x01, "0x0100",eReadWrite},

	{0x4B18, 0x25, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	
	{0x4B02, 0x04, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x05, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x18, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0F, 0x08, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x05, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x09, "0x0100",eReadWrite}, //CLK_EXIT
	
	{0x4B42, 0x06, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x09, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x05, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x09, "0x0100",eReadWrite}, //HS_EXIT

	{0x4B46, 0x1F, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B47, 0xC7, "0x0100",eReadWrite},
	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448P7_twolane_fpschg[] =
{
    {0x0340, 0x2d,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x70,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448P15_twolane_fpschg[] =
{
    {0x0340, 0x13,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xe0,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_aRegDescription_fourlane[] =
{
	//------------------------------------------------------------------------
	// Initial HM8040
	//------------------------------------------------------------------------
	{0x0103, 0x00, "0x0100",eReadWrite}, //software reset
	{0x3506, 0x01, "0x0100",eReadWrite}, //LSC autoload
	{0xBA90, 0x01, "0x0100",eReadWrite}, //Enable OTP load to user space
	{0x4405, 0x24, "0x0100",eReadWrite},
	{0x0100, 0x02, "0x0100",eReadWrite}, //power up

	//{0x0601, 0x03, "0x0100",eReadWrite},     //test parten
	
	//------------------------------------------------------------------------
	// PLL HM8040
	//------------------------------------------------------------------------
	// PCLK (RAW)
	{0x0303, 0x02, "0x0100",eReadWrite}, //CLK_DIV, 1:divide by 1, 2:divide by 2
	{0x0305, 0x0C, "0x0100",eReadWrite}, //PLL N,	 N = 12, while mclk 24mhz
	{0x0307, 0x46, "0x0100",eReadWrite}, //PLL M,	 M = 70, pclk_raw=(mclk*(M[6:0]/N[4:0])  )/(  CLK_DIV)
	// PKTCLK (MIPI)
	{0x0309, 0x01, "0x0100",eReadWrite}, //CLK_DIV, 1:divide by 1, 2:divide by 2
	{0x030D, 0x0C, "0x0100",eReadWrite}, //PLL N,	 N = 12, while mclk 24mhz
	//{0x030F, 0x60, "0x0100",eReadWrite}, //PLL M,	 M = 96, pkt_clk =(mclk*(M[6:0]/N[4:0])*2)/(4*CLK_DIV)
	{0x030F, 0x64, "0x0100",eReadWrite}, //PLL M,	 M = 100
	
	{0x4287, 0x01, "0x0100",eReadWrite}, //Enable UHS mode if packet clock is larger than 300 MHz
	//------------------------------------------------------------------------
	// Analog for HM8040
	//------------------------------------------------------------------------
	{0x4131, 0x01, "0x0100",eReadWrite}, // Digital BLI Enable
	{0x4132, 0x20, "0x0100",eReadWrite}, // Digital BLI Target
	{0x4140, 0x20, "0x0100",eReadWrite}, // BLC Target
	{0x4144, 0x03, "0x0100",eReadWrite}, // BLC ON
	
	{0x4145, 0x33, "0x0100",eReadWrite}, // CFPN ON
	{0x4146, 0x31, "0x0100",eReadWrite}, // CFPN IIR
	{0x4147, 0x03, "0x0100",eReadWrite}, // CFPN control
	{0x4149, 0x60, "0x0100",eReadWrite}, // BLC 4x2 channel + Dithering enable (06/11)
	{0x414A, 0x00, "0x0100",eReadWrite}, // BLC control
	
	{0x4261, 0x00, "0x0100",eReadWrite}, 
	{0x4262, 0x01, "0x0100",eReadWrite},
	{0x4264, 0x0A, "0x0100",eReadWrite}, 
	
	{0x4012, 0x00, "0x0100",eReadWrite}, // CDS Timing 
	
	{0x4270, 0x22, "0x0100",eReadWrite}, // CDS Timing 
	{0x4271, 0x68, "0x0100",eReadWrite}, // CDS Timing
	{0x4272, 0x43, "0x0100",eReadWrite}, // CDS Timing
	{0x4273, 0x44, "0x0100",eReadWrite}, // CDS Timing
	
	{0x4280, 0x10, "0x0100",eReadWrite}, // CDS Timing
	{0x4281, 0x10, "0x0100",eReadWrite}, // CDS Timing
	{0x4283, 0x07, "0x0100",eReadWrite}, // ADC Timing
	{0x4284, 0x00, "0x0100",eReadWrite}, // CDS Timing
	{0x4285, 0x0C, "0x0100",eReadWrite}, // CDS Timing
	
	{0x4380, 0xA6, "0x0100",eReadWrite}, // 
	{0x4381, 0x71, "0x0100",eReadWrite}, 
	{0x4383, 0x71, "0x0100",eReadWrite},
	{0x4384, 0x05, "0x0100",eReadWrite},
	{0x4385, 0xA2, "0x0100",eReadWrite}, // RAMP OUTPUT LEVEL -> set to A5 if 438E = 00
	{0x438A, 0x80, "0x0100",eReadWrite}, // [7]: CA bias current 
	{0x438B, 0x80, "0x0100",eReadWrite},
	{0x438C, 0x80, "0x0100",eReadWrite}, // Pixel VDD level 
	
	{0x4400, 0x36, "0x0100",eReadWrite}, // CA reset clamp control
	{0x4401, 0x36, "0x0100",eReadWrite},
	{0x4404, 0x01, "0x0100",eReadWrite}, // Dark clamp extend
	
	{0x45FC, 0x02, "0x0100",eReadWrite},
	{0x45FD, 0x00, "0x0100",eReadWrite}, // Pixel VDD switch control
	{0x45FE, 0x10, "0x0100",eReadWrite}, // Pixel VDD switch control
	
	{0x4490, 0xE7, "0x0100",eReadWrite}, //1x
	{0x4491, 0xE3, "0x0100",eReadWrite}, //2x 
	{0x4492, 0xE1, "0x0100",eReadWrite}, //4x
	{0x4493, 0xE0, "0x0100",eReadWrite}, //8x 
	{0x4494, 0xE8, "0x0100",eReadWrite}, //16x 
	{0x4495, 0xF8, "0x0100",eReadWrite}, //32x 
	
	{0x44D0, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 1
	{0x44D1, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 2 
	{0x44D2, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 4 
	{0x44D3, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 8 
	{0x44D4, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 16 
	{0x44D5, 0x33, "0x0100",eReadWrite}, //Comparator bias @ AGAIN = 32 
	
	{0x44A0, 0xE7, "0x0100",eReadWrite},   ///	binning gain table
	{0x44A1, 0xE3, "0x0100",eReadWrite},
	{0x44A2, 0xE1, "0x0100",eReadWrite},
	{0x44A3, 0xE0, "0x0100",eReadWrite},
	{0x44A4, 0xE8, "0x0100",eReadWrite},
	{0x44A5, 0xF8, "0x0100",eReadWrite},
	
	{0x44B0, 0xE1, "0x0100",eReadWrite},   ///	summing gain table
	{0x44B1, 0xE0, "0x0100",eReadWrite},
	{0x44B2, 0xE8, "0x0100",eReadWrite},
	{0x44B3, 0xE8, "0x0100",eReadWrite},
	{0x44B4, 0xF8, "0x0100",eReadWrite},
	{0x44B5, 0xF8, "0x0100",eReadWrite},
	
	{0x4600, 0x00, "0x0100",eReadWrite},   ///	summing enable (sum)
	{0x4601, 0x00, "0x0100",eReadWrite},
	{0x4602, 0x00, "0x0100",eReadWrite},
	{0x4603, 0x01, "0x0100",eReadWrite},
	{0x4604, 0x01, "0x0100",eReadWrite},
	{0x4605, 0x01, "0x0100",eReadWrite},
	//end of Analog
	//---------------------------------------------------------
	
	//------------------------------------------------------------------------
	// Digital
	//------------------------------------------------------------------------
	{0x3110, 0x01, "0x0100",eReadWrite},
	//{0x3110, 0x17, "0x0100",eReadWrite},
	
	//------------------------------------------------------------------------
	// Frame Timing
	//------------------------------------------------------------------------
	{0x0340, 0x05, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0x0E, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 1294= 1224+70)
	{0x0342, 0x1C, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x2C, "0x0100",eReadWrite}, //smia Line length Lb 	(7212)
	
	//------------------------------------------------------------------------
	// Image Size
	//------------------------------------------------------------------------
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (	 0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end	 of x Hb		
	{0x0349, 0xBD, "0x0100",eReadWrite}, //end	 of x Lb (3261)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (	 0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end	 of y Hb		
	{0x034B, 0x8D, "0x0100",eReadWrite}, //end	 of y Lb (2445)
	
	//------------------------------------------------------------------------
	// Output Size / Digiatl Window
	//------------------------------------------------------------------------
	{0x034C, 0x06, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0x60, "0x0100",eReadWrite}, //output size of x Lb (1632)
	{0x034E, 0x04, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0xC8, "0x0100",eReadWrite}, //output size of y Lb (1224)
	//{0x0350, 0x73, "0x0100",eReadWrite},

	{0x0381, 0x01, "0x0100",eReadWrite},
	{0x0383, 0x03, "0x0100",eReadWrite}, //x odd increment
	{0x0385, 0x01, "0x0100",eReadWrite},
	{0x0387, 0x03, "0x0100",eReadWrite}, //y odd increment
	{0x0390, 0x03, "0x0100",eReadWrite}, //binning enable
	
	//{0x0350, 0x73, "0x0100",eReadWrite},
	
	//------------------------------------------------------------------------
	// MIPI
	//------------------------------------------------------------------------
	{0x0111, 0x01, "0x0100",eReadWrite}, //0:2lane, 1:4lane 
	{0x0112, 0x0A, "0x0100",eReadWrite}, //DataFormat : 0x08 : RAW8, 0x0A : RAW10	
	{0x0113, 0x0A, "0x0100",eReadWrite}, //DataFormat : 0x08 : RAW8, 0x0A : RAW10 
	{0x4B01, 0x2B, "0x0100",eReadWrite}, //DataFormat : 0x2A : RAW8, 0x2B : RAW10 
	{0x4B20, 0xDE, "0x0100",eReadWrite}, //CC0 : Clock lane on always. 			 (9E) / woLSLE (8E) 
					   				     //GC2 : Clock lane on while frame active.  (DE) / woLSLE (CE) 
					   					 //GC1 : Clock lane on while packet issued. (BE) / woLSLE (AE) 
					   					 //GC3 : Clock lane on while line	active.  (FE) / woLSLE (EE) 
	
	{0x4B18, 0x11, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	{0x4B02, 0x02, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x03, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x0B, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0E, 0x00, "0x0100",eReadWrite}, //CLK_FRONT_PORCH (CLK_PRE)
	{0x4B0F, 0x09, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x02, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x04, "0x0100",eReadWrite}, //CLK_EXIT
	{0x4B42, 0x03, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x05, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x03, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x04, "0x0100",eReadWrite}, //HS_EXIT
	
	{0x4B57, 0x03, "0x0100",eReadWrite}, //Dynamic Range Setting for Dynamic Trigger Value Correction, 0:no correction, 1:-63~+63, 2:-127~+127, 3:-255~+255, 
	{0x4B51, 0x80, "0x0100",eReadWrite}, //enabled MIPI Status Monitor 
	{0x4B52, 0xC9, "0x0100",eReadWrite}, //enabled Dynamic Trigger Value Correction 
	
	{0x4B11, 0x0F, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B45, 0x00, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B46, 0x7F, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B47, 0xC0, "0x0100",eReadWrite}, //adjust skew relative parameters
	{0x4B3B, 0x13, "0x0100",eReadWrite}, //adjust slew rate parameters
	
	{0x4024, 0x40, "0x0100",eReadWrite}, //Enabled MIPI

	{0x0101, 0x03, "0x0100",eReadWrite}, //mirror
	//---------------------------------------------------
	// CMU update for HM8040
	//---------------------------------------------------
	{0x0104, 0x00, "0x0100",eReadWrite}, 

	{0x0000, 0x0A, "0x0100",eDelay},	//delay 10 ms
	
	{0x0100, 0x01, "0x0100",eReadWrite}, 
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224_fourlane[] =
{
	{0x0100, 0x00, "0x0100",eReadWrite},
	{0x0000, 0x32, "0x0100",eDelay},

	{0x0303, 0x02, "0x0100",eReadWrite}, //70M pclk
	//{0x0309, 0x02, "0x0100",eReadWrite},
	//{0x4287, 0x00, "0x0100",eReadWrite},

	{0x0340, 0x05, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0x0E, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 1294= 1224+70)
	{0x0342, 0x0E, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x16, "0x0100",eReadWrite}, //smia Line length Lb 	(7212)//3606
	
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (	 0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end	 of x Hb		
	{0x0349, 0xBD, "0x0100",eReadWrite}, //end	 of x Lb (3261)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (	 0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end	 of y Hb		
	{0x034B, 0x8D, "0x0100",eReadWrite}, //end	 of y Lb (2445)
	
	{0x034C, 0x06, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0x60, "0x0100",eReadWrite}, //output size of x Lb (1632)
	{0x034E, 0x04, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0xC8, "0x0100",eReadWrite}, //output size of y Lb (1224)
											
	{0x0383, 0x03, "0x0100",eReadWrite}, //x odd increment
	{0x0387, 0x03, "0x0100",eReadWrite}, //y odd increment
	{0x0390, 0x03, "0x0100",eReadWrite}, //binning enable

	{0x4B18, 0x11, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	{0x4B02, 0x02, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x03, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x0B, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0F, 0x09, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x02, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x04, "0x0100",eReadWrite}, //CLK_EXIT
	{0x4B42, 0x03, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x05, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x03, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x04, "0x0100",eReadWrite}, //HS_EXIT
	{0x4B56, 0x02, "0x0100",eReadWrite},
	
	//{0x0104, 0x00, "0x0100",eReadWrite},  //cmu update 	
	//{0x0000, 0x32, "0x0100",eDelay},
	//{0x0100, 0x01, "0x0100",eReadWrite},
	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P30_fourlane_fpschg[] =
{    
	{0x0340, 0x05,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x0E,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P25_fourlane_fpschg[] =
{  	
	{0x0340, 0x06,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x10,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P20_fourlane_fpschg[] =
{
    {0x0340, 0x07,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xC6,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P15_fourlane_fpschg[] =
{
	{0x0340, 0x0a,"0x0100",eReadWrite}, // VTS H	
	{0x0341, 0x30,"0x0100",eReadWrite}, // VTS L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_1632x1224P10_fourlane_fpschg[] =
{ 
	{0x0340, 0x0f,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xcc,"0x0100",eReadWrite}, // VTS L	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448_fourlane[] =
{
	{0x0100, 0x00, "0x0100",eReadWrite},
	{0x0000, 0x32, "0x0100",eDelay},

	{0x0303, 0x01, "0x0100",eReadWrite},
	//{0x0309, 0x01, "0x0100",eReadWrite},
	//{0x4287, 0x01, "0x0100",eReadWrite},

	{0x0340, 0x09, "0x0100",eReadWrite}, //smia Frame length Hb 
	{0x0341, 0xD6, "0x0100",eReadWrite}, //smia Frame length Lb (VSYNC : 2518= 2448+70    )
	{0x0342, 0x0E, "0x0100",eReadWrite}, //smia Line length Hb 
	{0x0343, 0x7A, "0x0100",eReadWrite}, //smia Line length Lb 	3706
	
	{0x0344, 0x00, "0x0100",eReadWrite}, //start of x Hb		
	{0x0345, 0x00, "0x0100",eReadWrite}, //start of x Lb (   0)
	{0x0348, 0x0C, "0x0100",eReadWrite}, //end   of x Hb		
	{0x0349, 0xBF, "0x0100",eReadWrite}, //end   of x Lb (3263)
	{0x0346, 0x00, "0x0100",eReadWrite}, //start of y Hb		
	{0x0347, 0x00, "0x0100",eReadWrite}, //start of y Lb (   0)
	{0x034A, 0x09, "0x0100",eReadWrite}, //end   of y Hb		
	{0x034B, 0x8F, "0x0100",eReadWrite}, //end   of y Lb (2447)
	
	{0x034C, 0x0C, "0x0100",eReadWrite}, //output size of x Hb
	{0x034D, 0xC0, "0x0100",eReadWrite}, //output size of x Lb (3264)
	{0x034E, 0x09, "0x0100",eReadWrite}, //output size of y Hb
	{0x034F, 0x90, "0x0100",eReadWrite}, //output size of y Lb (2448)

	{0x0383, 0x01, "0x0100",eReadWrite}, //x odd increment
	{0x0387, 0x01, "0x0100",eReadWrite}, //y odd increment
	{0x0390, 0x00, "0x0100",eReadWrite}, //binning enable

	{0x4B18, 0x2C, "0x0100",eReadWrite}, //[7:0] FULL_TRIGGER_TIME 
	{0x4B02, 0x04, "0x0100",eReadWrite}, //TLPX
	{0x4B43, 0x06, "0x0100",eReadWrite}, //CLK_PREPARE
	{0x4B05, 0x18, "0x0100",eReadWrite}, //CLK_ZERO
	{0x4B0F, 0x0C, "0x0100",eReadWrite}, //CLK_BACK_PORCH  (CLK_POST)
	{0x4B06, 0x05, "0x0100",eReadWrite}, //CLK_TRAIL
	{0x4B39, 0x09, "0x0100",eReadWrite}, //CLK_EXIT
	{0x4B42, 0x06, "0x0100",eReadWrite}, //HS_PREPARE
	{0x4B03, 0x0A, "0x0100",eReadWrite}, //HS_ZERO
	{0x4B04, 0x06, "0x0100",eReadWrite}, //HS_TRAIL
	{0x4B3A, 0x09, "0x0100",eReadWrite}, //HS_EXIT
	{0x4B56, 0x01, "0x0100",eReadWrite},
	
	//{0x0104, 0x00, "0x0100",eReadWrite},  //cmu update 	
	//{0x0000, 0x32, "0x0100",eDelay},
	//{0x0100, 0x01, "0x0100",eReadWrite},
	
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448P7_fourlane_fpschg[] =
{
    {0x0340, 0x2d,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x70,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t HM8040_g_3264x2448P10_fourlane_fpschg[] =
{
    {0x0340, 0x1e,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xc0,"0x0100",eReadWrite}, // VTS L
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t HM8040_g_3264x2448P15_fourlane_fpschg[] =
{
	{0x0340, 0x13,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xe0,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t HM8040_g_3264x2448P20_fourlane_fpschg[] =
{
	{0x0340, 0x0f,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0x22,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t HM8040_g_3264x2448P25_fourlane_fpschg[] =
{
    {0x0340, 0x0b,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xce,"0x0100",eReadWrite}, // VTS L	
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t HM8040_g_3264x2448P30_fourlane_fpschg[] =
{
    {0x0340, 0x09,"0x0100",eReadWrite}, // VTS H						
	{0x0341, 0xd6,"0x0100",eReadWrite}, // VTS L
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

