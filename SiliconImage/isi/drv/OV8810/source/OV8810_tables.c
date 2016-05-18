/*****************************************************************************/
/*!
 *  \file        OV8810_tables.c \n
 *  \version     1.0 \n
 *  \author      Meinicke \n
 *  \brief       Image-sensor-specific tables and other
 *               constant values/structures for OV8810. \n
 *
 *  \revision    $Revision: 803 $ \n
 *               $Author: $ \n
 *               $Date: 2010-02-26 16:35:22 +0100 (Fr, 26 Feb 2010) $ \n
 *               $Id: OV8810_tables.c 803 2010-02-26 15:35:22Z  $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/
/*
#include "stdinc.h"

#if( OV8810_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "OV8810_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV8810_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.
const IsiRegDescription_t OV8810_g_aRegDescription[] =
{
  //{ulAddr                     , ulDefaultValue                     , pszName               , ulFlags}
    {OV8810_AGCL                , OV8810_AGCL_DEFAULT                , "AGCL"                , eReadWrite},
    {OV8810_AECL_2              , OV8810_AECL_2_DEFAULT              , "AECL_2"              , eReadWrite},
    {OV8810_AECL_1              , OV8810_AECL_1_DEFAULT              , "AECL_1"              , eReadWrite},
    {OV8810_LAEC_2              , OV8810_LAEC_2_DEFAULT              , "LAEC_2"              , eReadWrite},
    {OV8810_LAEC_1              , OV8810_LAEC_1_DEFAULT              , "LAEC_1"              , eReadWrite},
    {OV8810_AGC         		, OV8810_AGC_DEFAULT         		 , "AGC"         		 , eReadWrite},
    {OV8810_PIDH                , OV8810_PIDH_DEFAULT                , "PIDH"                , eReadable},
    {OV8810_PIDL                , OV8810_PIDL_DEFAULT                , "PIDL"                , eReadable},
    {OV8810_SCCB_ID             , OV8810_SCCB_ID_DEFAULT             , "SCCB_ID"             , eReadWrite},
    {OV8810_R_PLL               , OV8810_R_PLL_DEFAULT               , "R_PLL"               , eReadWrite},
    {OV8810_R_PLL1              , OV8810_R_PLL1_DEFAULT              , "R_PLL1"              , eReadWrite},
    {OV8810_R_PLL2              , OV8810_R_PLL2_DEFAULT              , "R_PLL2"              , eReadWrite},
    {OV8810_R_PLL3              , OV8810_R_PLL3_DEFAULT              , "R_PLL3"              , eReadWrite},
    {OV8810_R_PLL4              , OV8810_R_PLL4_DEFAULT              , "R_PLL4"              , eReadWrite},
    {OV8810_SYS                 , OV8810_SYS_DEFAULT                 , "SYS"                 , eReadWrite},
    {OV8810_AUTO1               , OV8810_AUTO1_DEFAULT               , "AUTO1"               , eReadWrite},
    {OV8810_AUTO2               , OV8810_AUTO2_DEFAULT               , "AUTO2"               , eReadWrite},
    {OV8810_AUTO3               , OV8810_AUTO3_DEFAULT               , "AUTO3"               , eReadWrite},
  	{OV8810_0x3016				, OV8810_0x3016_DEFAULT				 , "0x3016"				 , eReadWrite},
    {OV8810_WPT                 , OV8810_WPT_DEFAULT                 , "WPT"                 , eReadWrite},
    {OV8810_BPT                 , OV8810_BPT_DEFAULT                 , "BPT"                 , eReadWrite},
    {OV8810_VPT                 , OV8810_VPT_DEFAULT                 , "VPT"                 , eReadWrite},
    {OV8810_YAVG                , OV8810_YAVG_DEFAULT                , "YAVG"                , eReadOnly},
    {OV8810_AECG_MAX50          , OV8810_AECG_MAX50_DEFAULT          , "AECG_MAX50"          , eReadWrite},
    {OV8810_AECG_MAX60          , OV8810_AECG_MAX60_DEFAULT          , "AECG_MAX60"          , eReadWrite},
    {OV8810_ADDVS_2             , OV8810_ADDVS_2_DEFAULT             , "ADDVS_2"             , eReadWrite},
    {OV8810_ADDVS_1             , OV8810_ADDVS_1_DEFAULT             , "ADDVS_1"             , eReadWrite},
    {OV8810_FRAME_LENGTH_LINES2 , OV8810_FRAME_LENGTH_LINES2_DEFAULT , "FRAME_LENGTH_LINES2" , eReadWrite},
    {OV8810_FRAME_LENGTH_LINES1 , OV8810_FRAME_LENGTH_LINES1_DEFAULT , "FRAME_LENGTH_LINES1" , eReadWrite},
    {OV8810_LINE_LENGTH_PCK2    , OV8810_LINE_LENGTH_PCK2_DEFAULT    , "LINE_LENGTH_PCK2"    , eReadWrite},
    {OV8810_LINE_LENGTH_PCK1    , OV8810_LINE_LENGTH_PCK1_DEFAULT    , "LINE_LENGTH_PCK1"    , eReadWrite},
    {OV8810_X_ADDR_START_2      , OV8810_X_ADDR_START_2_DEFAULT      , "X_ADDR_START_2"      , eReadWrite},
    {OV8810_X_ADDR_START_1      , OV8810_X_ADDR_START_1_DEFAULT      , "X_ADDR_START_1"      , eReadWrite},
    {OV8810_Y_ADDR_START_2      , OV8810_Y_ADDR_START_2_DEFAULT      , "Y_ADDR_START_2"      , eReadWrite},
    {OV8810_Y_ADDR_START_1      , OV8810_Y_ADDR_START_1_DEFAULT      , "Y_ADDR_START_1"      , eReadWrite},
    {OV8810_X_ADDR_END_2        , OV8810_X_ADDR_END_2_DEFAULT        , "X_ADDR_END_2"        , eReadWrite},
    {OV8810_X_ADDR_END_1        , OV8810_X_ADDR_END_1_DEFAULT        , "X_ADDR_END_1"        , eReadWrite},
    {OV8810_Y_ADDR_END_2        , OV8810_Y_ADDR_END_2_DEFAULT        , "Y_ADDR_END_2"        , eReadWrite},
    {OV8810_Y_ADDR_END_1        , OV8810_Y_ADDR_END_1_DEFAULT        , "Y_ADDR_END_1"        , eReadWrite},
    {OV8810_X_OUTPUTSIZE_2      , OV8810_X_OUTPUTSIZE_2_DEFAULT      , "X_OUTPUTSIZE_2"      , eReadWrite},
    {OV8810_X_OUTPUTSIZE_1      , OV8810_X_OUTPUTSIZE_1_DEFAULT      , "X_OUTPUTSIZE_1"      , eReadWrite},
    {OV8810_Y_OUTPUTSIZE_2      , OV8810_Y_OUTPUTSIZE_2_DEFAULT      , "Y_OUTPUTSIZE_2"      , eReadWrite},
    {OV8810_Y_OUTPUTSIZE_1      , OV8810_Y_OUTPUTSIZE_1_DEFAULT      , "Y_OUTPUTSIZE_1"      , eReadWrite},
    {OV8810_DATR_D56            , OV8810_DATR_D56_DEFAULT            , "DATR_D56"            , eReadOnly},
  	{OV8810_DATR_OTP			, OV8810_DATR_OTP_DEFAULT 			 , "DATR_OTP"	 		 , eReadWrite},
  	{OV8810_R_SIGMA				, OV8810_R_SIGMA_DEFAULT			 , "R_SIGMA"			 , eReadWrite},
  	{OV8810_0x3058				, OV8810_0x3058_DEFAULT				 , "0x3058"				 , eReadWrite},
  	{OV8810_0x3059				, OV8810_0x3059_DEFAULT				 , "0x3059"				 , eReadWrite},
    {OV8810_BD50ST_2            , OV8810_BD50ST_2_DEFAULT            , "BD50ST_2"            , eReadWrite},
    {OV8810_BD50ST_1            , OV8810_BD50ST_1_DEFAULT            , "BD50ST_1"            , eReadWrite},
    {OV8810_BD60ST_2            , OV8810_BD60ST_2_DEFAULT            , "BD60ST_2"            , eReadWrite},
    {OV8810_BD60ST_1            , OV8810_BD60ST_1_DEFAULT            , "BD60ST_1"            , eReadWrite},
    {OV8810_0x3065				, OV8810_0x3065_DEFAULT				 , "0x3065"				 , eReadWrite},
    {OV8810_0x3067				, OV8810_0x3067_DEFAULT				 , "0x3067"				 , eReadWrite},
    {OV8810_0x3068				, OV8810_0x3068_DEFAULT				 , "0x3068"				 , eReadWrite},
    {OV8810_0x3069				, OV8810_0x3069_DEFAULT				 , "0x3069"				 , eReadWrite},
    {OV8810_0x306A				, OV8810_0x306A_DEFAULT				 , "0x306A"				 , eReadWrite},
  	{OV8810_0x306B				, OV8810_0x306B_DEFAULT				 , "0x306B"				 , eReadWrite},
  	{OV8810_TMC1				, OV8810_TMC1_DEFAULT				 , "TMC1"				 , eReadWrite},
  	{OV8810_0x3072				, OV8810_0x3072_DEFAULT				 , "0x3072"				 , eReadWrite},
  	{OV8810_TMC5				, OV8810_TMC5_DEFAULT				 , "TMC5"				 , eReadWrite},
  	{OV8810_TMC6				, OV8810_TMC6_DEFAULT				 , "TMC6"				 , eReadWrite},
  	{OV8810_TMC7				, OV8810_TMC7_DEFAULT				 , "TMC7"		 		 , eReadWrite},
  	{OV8810_TMC8				, OV8810_TMC8_DEFAULT				 , "TMC8"		 		 , eReadWrite},
  	{OV8810_TMC9				, OV8810_TMC9_DEFAULT				 , "TMC9"				 , eReadWrite},
  	{OV8810_TMCE				, OV8810_TMCE_DEFAULT				 , "TMCE"		 		 , eReadWrite},
  	{OV8810_0x3080				, OV8810_0x3080_DEFAULT				 , "0x3080"				 , eReadWrite},
  	{OV8810_0x3082				, OV8810_0x3082_DEFAULT				 , "0x3082"				 , eReadWrite},
  	{OV8810_0x3084				, OV8810_0x3084_DEFAULT				 , "0x3084"				 , eReadWrite},
  	{OV8810_0x3087				, OV8810_0x3087_DEFAULT				 , "0x3087"				 , eReadWrite},
  	{OV8810_0x308A				, OV8810_0x308A_DEFAULT				 , "0x308A"				 , eReadWrite},
  	{OV8810_0x308D				, OV8810_0x308D_DEFAULT				 , "0x308D"				 , eReadWrite},
  	{OV8810_0x3090				, OV8810_0x3090_DEFAULT				 , "0x3090"				 , eReadWrite},
  	{OV8810_R_ARRAY				, OV8810_R_ARRAY_DEFAULT			 , "R_ARRAY"			 , eReadWrite},
  	{OV8810_0x3092				, OV8810_0x3092_DEFAULT				 , "0x3092"				 , eReadWrite},
  	{OV8810_0x3094				, OV8810_0x3094_DEFAULT				 , "0x3094"				 , eReadWrite},
  	{OV8810_0x3095				, OV8810_0x3095_DEFAULT				 , "0x3095"				 , eReadWrite},
  	{OV8810_0x3098				, OV8810_0x3098_DEFAULT				 , "0x3098"				 , eReadWrite},
  	{OV8810_0x3099				, OV8810_0x3099_DEFAULT				 , "0x3099"				 , eReadWrite},
  	{OV8810_0x309A				, OV8810_0x309A_DEFAULT				 , "0x309A"				 , eReadWrite},
  	{OV8810_0x309B				, OV8810_0x309B_DEFAULT				 , "0x309B"				 , eReadWrite},
  	{OV8810_0x309C				, OV8810_0x309C_DEFAULT				 , "0x309C"				 , eReadWrite},
  	{OV8810_0x309D				, OV8810_0x309D_DEFAULT				 , "0x309D"				 , eReadWrite},
  	{OV8810_0x309E				, OV8810_0x309E_DEFAULT				 , "0x309E"				 , eReadWrite},
  	{OV8810_0x309F				, OV8810_0x309F_DEFAULT				 , "0x309F"				 , eReadWrite},
  	{OV8810_0x30A0				, OV8810_0x30A0_DEFAULT				 , "0x30A0"				 , eReadWrite},
  	{OV8810_SLEW				, OV8810_SLEW_DEFAULT				 , "SLEW"		 		 , eReadWrite},
  	{OV8810_R_PWC				, OV8810_R_PWC_DEFAULT				 , "R_PWC"		 		 , eReadWrite},
  	{OV8810_0x30AA				, OV8810_0x30AA_DEFAULT				 , "0x30AA"				 , eReadWrite},
  	{OV8810_0x30AB				, OV8810_0x30AB_DEFAULT				 , "0x30AB"				 , eReadWrite},
    {OV8810_IO_CTRL0            , OV8810_IO_CTRL0_DEFAULT            , "IO_CTRL0"            , eReadWrite},
    {OV8810_IO_CTRL1            , OV8810_IO_CTRL1_DEFAULT            , "IO_CTRL1"            , eReadable},
    {OV8810_IO_CTRL2            , OV8810_IO_CTRL2_DEFAULT            , "IO_CTRL2"            , eReadWrite},
    {OV8810_DSIO_3              , OV8810_DSIO_3_DEFAULT              , "DSIO_3"              , eReadWrite},
    {OV8810_DSIO_4              , OV8810_DSIO_4_DEFAULT              , "DSIO_4"              , eReadWrite},
  	{OV8810_FRS_30B7			, OV8810_FRS_30B7_DEFAULT			 , "FRS_30B7"			 , eReadWrite},
  	{OV8810_0x30BE				, OV8810_0x30BE_DEFAULT				 , "0x30BE"				 , eReadWrite},
  	{OV8810_0x30BF				, OV8810_0x30BF_DEFAULT				 , "0x30BF"				 , eReadWrite},
  	{OV8810_0x30E3				, OV8810_0x30E3_DEFAULT				 , "0x30E3"				 , eReadWrite},
  	{OV8810_FRS_30E4			, OV8810_FRS_30E4_DEFAULT			 , "FRS_30E4"	 		 , eReadWrite},
  	{OV8810_FRS_30E5			, OV8810_FRS_30E5_DEFAULT			 , "FRS_30E5"	 		 , eReadWrite},
  	{OV8810_FRS_30E6			, OV8810_FRS_30E6_DEFAULT			 , "FRS_30E6"	 		 , eReadWrite},
  	{OV8810_FRS_30E7			, OV8810_FRS_30E7_DEFAULT			 , "FRS_30E7"	 		 , eReadWrite},
  	{OV8810_FRS_30E8			, OV8810_FRS_30E8_DEFAULT			 , "FRS_30E8"			 , eReadWrite},
  	{OV8810_0x30E9				, OV8810_0x30E9_DEFAULT			 	 , "0x30E9"	 		 	 , eReadWrite},
  	{OV8810_FRS_30EA			, OV8810_FRS_30EA_DEFAULT			 , "FRS_30EA"	 		 , eReadWrite},
  	{OV8810_FRS_30EB			, OV8810_FRS_30EB_DEFAULT			 , "FRS_30EB"	 		 , eReadWrite},
  	{OV8810_VCM_H				, OV8810_VCM_H_DEFAULT				 , "VCM_H"		 		 , eReadWrite},
  	{OV8810_VCM_L				, OV8810_VCM_L_DEFAULT				 , "VCM_L"		 		 , eReadWrite},
  	{OV8810_SLEW_H				, OV8810_SLEW_H_DEFAULT			 	 , "SLEW_H"		  		 , eReadWrite},
  	{OV8810_SLEW_L				, OV8810_SLEW_L_DEFAULT			 	 , "SLEW_L"		  		 , eReadWrite},
  	{OV8810_0x30F0				, OV8810_0x30F0_DEFAULT				 , "0x30F0"				 , eReadWrite},
  	{OV8810_0x30F2				, OV8810_0x30F2_DEFAULT				 , "0x30F2"				 , eReadWrite},
  	{OV8810_0x30F4				, OV8810_0x30F4_DEFAULT				 , "0x30F4"				 , eReadWrite},
    {OV8810_IMAGE_TRANSFO       , OV8810_IMAGE_TRANSFO_DEFAULT       , "IMAGE_TRANSFO"       , eReadWrite},
    {OV8810_IMAGE_SYSTEM        , OV8810_IMAGE_SYSTEM_DEFAULT        , "IMAGE_SYSTEM"        , eReadWrite},
    {OV8810_0x30FB				, OV8810_0x30FB_DEFAULT				 , "0x30FB"				 , eReadWrite},
    {OV8810_GROUP_WR            , OV8810_GROUP_WR_DEFAULT            , "GROUP_WR"            , eReadWrite},
  	{OV8810_SYS_RST				, OV8810_SYS_RST_DEFAULT			 , "SYS_RST"			 , eReadWrite},
  	{OV8810_CLOCK_EN			, OV8810_CLOCK_EN_DEFAULT			 , "CLOCK_EN"			 , eReadWrite},
  	{OV8810_TOP0				, OV8810_TOP0_DEFAULT				 , "TOP0"		 		 , eReadWrite},
  	{OV8810_TOP1				, OV8810_TOP1_DEFAULT				 , "TOP1"				 , eReadWrite},
  	{OV8810_TOP2				, OV8810_TOP2_DEFAULT				 , "TOP2"				 , eReadWrite},
  	{OV8810_CBAR				, OV8810_CBAR_DEFAULT				 , "CBAR"				 , eReadWrite},
  	{OV8810_TOP4				, OV8810_TOP4_DEFAULT				 , "TOP4"				 , eReadWrite},
  	{OV8810_0x3306				, OV8810_0x3306_DEFAULT				 , "0x3306"				 , eReadWrite},
  	{OV8810_BLC					, OV8810_BLC_DEFAULT				 , "BLC"				 , eReadWrite},
  	{OV8810_DIG_GAIN			, OV8810_DIG_GAIN_DEFAULT			 , "DIG_GAIN"			 , eReadable},
  	{OV8810_WBC_A				, OV8810_WBC_A_DEFAULT				 , "WBC_A"		 		 , eReadWrite},
	{OV8810_WBC_B				, OV8810_WBC_B_DEFAULT				 , "WBC_B"				 , eReadWrite},
	{OV8810_WBC_C				, OV8810_WBC_C_DEFAULT				 , "WBC_C"				 , eReadWrite},
	{OV8810_WBC_D				, OV8810_WBC_D_DEFAULT				 , "WBC_D"				 , eReadWrite},
  	{OV8810_TOP_330E			, OV8810_TOP_330E_DEFAULT			 , "TOP_330E"			 , eReadWrite},
  	{OV8810_TOP_330F			, OV8810_TOP_330F_DEFAULT			 , "TOP_330F"			 , eReadOnly},
  	{OV8810_0x3313				, OV8810_0x3313_DEFAULT				 , "0x3313"		 		 , eReadWrite},
  	{OV8810_SIZE_6				, OV8810_SIZE_6_DEFAULT				 , "SIZE_6"				 , eReadWrite},
  	{OV8810_SIZE_7				, OV8810_SIZE_7_DEFAULT				 , "SIZE_7"				 , eReadWrite},
  	{OV8810_SIZE_8				, OV8810_SIZE_8_DEFAULT				 , "SIZE_8"				 , eReadWrite},
  	{OV8810_SIZE_9				, OV8810_SIZE_9_DEFAULT				 , "SIZE_9"				 , eReadWrite},
  	{OV8810_SIZE_A				, OV8810_SIZE_A_DEFAULT				 , "SIZE_A"				 , eReadWrite},
  	{OV8810_SIZE_B				, OV8810_SIZE_B_DEFAULT				 , "SIZE_B"				 , eReadWrite},
  	{OV8810_AVG_E				, OV8810_AVG_E_DEFAULT				 , "AVG_E"				 , eReadWrite},
  	{OV8810_AVG_F				, OV8810_AVG_F_DEFAULT				 , "AVG_F"				 , eReadWrite},
  	{OV8810_AWB0				, OV8810_AWB0_DEFAULT				 , "AWB0"				 , eReadWrite},
  	{OV8810_AWB1				, OV8810_AWB1_DEFAULT				 , "AWB1"		 		 , eReadWrite},
  	{OV8810_AWB2				, OV8810_AWB2_DEFAULT				 , "AWB2"				 , eReadWrite},
  	{OV8810_AWB3				, OV8810_AWB3_DEFAULT				 , "AWB3"				 , eReadWrite},
  	{OV8810_AWB4				, OV8810_AWB4_DEFAULT				 , "AWB4"				 , eReadWrite},
  	{OV8810_AWB5				, OV8810_AWB5_DEFAULT				 , "AWB5"				 , eReadWrite},
  	{OV8810_AWB6				, OV8810_AWB6_DEFAULT				 , "AWB6"				 , eReadWrite},
  	{OV8810_AWB7				, OV8810_AWB7_DEFAULT				 , "AWB7"				 , eReadWrite},
  	{OV8810_AWB8				, OV8810_AWB8_DEFAULT				 , "AWB8"				 , eReadWrite},
  	{OV8810_WBC_29				, OV8810_WBC_29_DEFAULT				 , "WBC_29"				 , eReadWrite},
  	{OV8810_WBC_2A				, OV8810_WBC_2A_DEFAULT				 , "WBC_2A"				 , eReadWrite},
  	{OV8810_WBC_2B				, OV8810_WBC_2B_DEFAULT				 , "WBC_2B"				 , eReadWrite},
  	{OV8810_WBC_2C				, OV8810_WBC_2C_DEFAULT				 , "WBC_2C"				 , eReadWrite},
  	{OV8810_WBC_2D				, OV8810_WBC_2D_DEFAULT				 , "WBC_2D"				 , eReadWrite},
  	{OV8810_BLC0				, OV8810_BLC0_DEFAULT				 , "BLC0"				 , eReadWrite},
  	{OV8810_BLC1				, OV8810_BLC1_DEFAULT				 , "BLC1"				 , eReadWrite},
  	{OV8810_BLC2				, OV8810_BLC2_DEFAULT				 , "BLC2"				 , eReadWrite},
  	{OV8810_BLC3				, OV8810_BLC3_DEFAULT				 , "BLC3"				 , eReadWrite},
  	{OV8810_BLC4				, OV8810_BLC4_DEFAULT				 , "BLC4"				 , eReadWrite},
    {OV8810_BLC_MAN0            , OV8810_BLC_MAN0_DEFAULT            , "BLC_MAN0"          	 , eReadWrite},
    {OV8810_BLC_MAN1            , OV8810_BLC_MAN1_DEFAULT            , "BLC_MAN1"            , eReadWrite},
    {OV8810_BLC_MAN2            , OV8810_BLC_MAN2_DEFAULT            , "BLC_MAN2"            , eReadWrite},
    {OV8810_BLC_MAN3            , OV8810_BLC_MAN3_DEFAULT            , "BLC_MAN3"            , eReadWrite},
    {OV8810_BLC_MAN4            , OV8810_BLC_MAN4_DEFAULT            , "BLC_MAN4"            , eReadWrite},
    {OV8810_BLC_MAN5            , OV8810_BLC_MAN5_DEFAULT            , "BLC_MAN5"            , eReadWrite},
    {OV8810_BLC_MAN6            , OV8810_BLC_MAN6_DEFAULT            , "BLC_MAN6"            , eReadWrite},
    {OV8810_BLC_MAN7            , OV8810_BLC_MAN7_DEFAULT            , "BLC_MAN7"            , eReadWrite},
    {OV8810_BLC_MAN0_1          , OV8810_BLC_MAN0_1_DEFAULT          , "BLC_MAN0_1"          , eReadWrite},
    {OV8810_BLC_MAN2_3          , OV8810_BLC_MAN2_3_DEFAULT          , "BLC_MAN2_3"          , eReadWrite},
    {OV8810_BLC_MAN5_6          , OV8810_BLC_MAN5_6_DEFAULT          , "BLC_MAN5_6"          , eReadWrite},
    {OV8810_BLC_MAN6_7          , OV8810_BLC_MAN6_7_DEFAULT          , "BLC_MAN6_7"          , eReadWrite},
  	{OV8810_BLC_AVG				, OV8810_BLC_AVG_DEFAULT			 , "BLC_AVG"			 , eReadWrite},
  	{OV8810_LENC				, OV8810_LENC_DEFAULT				 , "LENC"				 , eReadWrite},
  	{OV8810_VAP5				, OV8810_VAP5_DEFAULT				 , "VAP5"				 , eReadWrite},
  	{OV8810_VAP6				, OV8810_VAP6_DEFAULT				 , "VAP6"				 , eReadWrite},
  	{OV8810_VAP_H				, OV8810_VAP_H_DEFAULT				 , "VAP_H"				 , eReadWrite},
    {OV8810_ISP_TEST            , OV8810_ISP_TEST_DEFAULT            , "ISP_TEST"            , eReadWrite}, /*Debug by FeiPeng*/
    {OV8810_DVP_CTRL00          , OV8810_DVP_CTRL00_DEFAULT          , "DVP_CTRL00"          , eReadWrite},
    {OV8810_DVP_CTRL01          , OV8810_DVP_CTRL01_DEFAULT          , "DVP_CTRL01"          , eReadWrite},
    {OV8810_DVP_CTRL02          , OV8810_DVP_CTRL02_DEFAULT          , "DVP_CTRL02"          , eReadWrite},
    {OV8810_DVP_CTRL03          , OV8810_DVP_CTRL03_DEFAULT          , "DVP_CTRL03"          , eReadWrite},
    {OV8810_DVP_CTRL04          , OV8810_DVP_CTRL04_DEFAULT          , "DVP_CTRL04"          , eReadWrite},
    {OV8810_DVP_CTRL05          , OV8810_DVP_CTRL05_DEFAULT          , "DVP_CTRL05"          , eReadWrite},
    {OV8810_DVP_CTRL06          , OV8810_DVP_CTRL06_DEFAULT          , "DVP_CTRL06"          , eReadWrite},
    {OV8810_DVP_CTRL07          , OV8810_DVP_CTRL07_DEFAULT          , "DVP_CTRL07"          , eReadWrite},
    {OV8810_DVP_CTRL08          , OV8810_DVP_CTRL08_DEFAULT          , "DVP_CTRL08"          , eReadWrite},
    {OV8810_DVP_CTRL09          , OV8810_DVP_CTRL09_DEFAULT          , "DVP_CTRL09"          , eReadWrite},
    {OV8810_DVP_CTRL0A          , OV8810_DVP_CTRL0A_DEFAULT          , "DVP_CTRL0A"          , eReadWrite},
    {OV8810_DVP_CTRL0B          , OV8810_DVP_CTRL0B_DEFAULT          , "DVP_CTRL0B"          , eReadWrite},
    {OV8810_DVP_CTRL0C          , OV8810_DVP_CTRL0C_DEFAULT          , "DVP_CTRL0C"          , eReadWrite},
    {OV8810_DVP_CTRL0D          , OV8810_DVP_CTRL0D_DEFAULT          , "DVP_CTRL0D"          , eReadWrite},
    {OV8810_DVP_CTRL0E          , OV8810_DVP_CTRL0E_DEFAULT          , "DVP_CTRL0E"          , eReadWrite},
    {OV8810_DVP_CTRL0F          , OV8810_DVP_CTRL0F_DEFAULT          , "DVP_CTRL0F"          , eReadWrite},
    {OV8810_DVP_CTRL10          , OV8810_DVP_CTRL10_DEFAULT          , "DVP_CTRL10"          , eReadWrite},
    {OV8810_DVP_CTRL11          , OV8810_DVP_CTRL11_DEFAULT          , "DVP_CTRL11"          , eReadWrite},
    {OV8810_DVP_CTRL12          , OV8810_DVP_CTRL12_DEFAULT          , "DVP_CTRL12"          , eReadWrite},
    {OV8810_DVP_CTRL13          , OV8810_DVP_CTRL13_DEFAULT          , "DVP_CTRL13"          , eReadWrite},
    {OV8810_DVP_CTRL14          , OV8810_DVP_CTRL14_DEFAULT          , "DVP_CTRL14"          , eReadWrite},
    {OV8810_DVP_CTRL15          , OV8810_DVP_CTRL15_DEFAULT          , "DVP_CTRL15"          , eReadWrite},
    {OV8810_MIPI_CTRL00         , OV8810_MIPI_CTRL00_DEFAULT         , "MIPI_CTRL00"         , eReadWrite},
    {OV8810_MIPI_CTRL01         , OV8810_MIPI_CTRL01_DEFAULT         , "MIPI_CTRL01"         , eReadWrite},
    {OV8810_MIPI_CTRL02         , OV8810_MIPI_CTRL02_DEFAULT         , "MIPI_CTRL02"         , eReadWrite},
    {OV8810_MIPI_CTRL03         , OV8810_MIPI_CTRL03_DEFAULT         , "MIPI_CTRL03"         , eReadWrite},
    {OV8810_MIPI_CTRL04         , OV8810_MIPI_CTRL04_DEFAULT         , "MIPI_CTRL04"         , eReadWrite},
    {OV8810_MIPI_CTRL05         , OV8810_MIPI_CTRL05_DEFAULT         , "MIPI_CTRL05"         , eReadWrite},
    {OV8810_MIPI_CTRL06         , OV8810_MIPI_CTRL06_DEFAULT         , "MIPI_CTRL06"         , eReadWrite},
    {OV8810_MIPI_CTRL07         , OV8810_MIPI_CTRL07_DEFAULT         , "MIPI_CTRL07"         , eReadWrite},
    {OV8810_MIPI_CTRL0A         , OV8810_MIPI_CTRL0A_DEFAULT         , "MIPI_CTRL0A"         , eReadWrite},
    {OV8810_MIPI_CTRL0B         , OV8810_MIPI_CTRL0B_DEFAULT         , "MIPI_CTRL0B"         , eReadWrite},
    {OV8810_MIPI_CTRL0C         , OV8810_MIPI_CTRL0C_DEFAULT         , "MIPI_CTRL0C"         , eReadWrite},
    {OV8810_MIPI_CTRL0E         , OV8810_MIPI_CTRL0E_DEFAULT         , "MIPI_CTRL0E"         , eReadWrite},
    {OV8810_MIPI_CTRL10         , OV8810_MIPI_CTRL10_DEFAULT         , "MIPI_CTRL10"         , eReadWrite},
    {OV8810_MIPI_CTRL11         , OV8810_MIPI_CTRL11_DEFAULT         , "MIPI_CTRL11"         , eReadWrite},
    {OV8810_MIPI_CTRL13         , OV8810_MIPI_CTRL13_DEFAULT         , "MIPI_CTRL13"         , eReadWrite},
    {OV8810_MIPI_CTRL14         , OV8810_MIPI_CTRL14_DEFAULT         , "MIPI_CTRL14"         , eReadWrite},
    {OV8810_MIPI_CTRL15         , OV8810_MIPI_CTRL15_DEFAULT         , "MIPI_CTRL15"         , eReadWrite},
    {OV8810_MIPI_CTRL16         , OV8810_MIPI_CTRL16_DEFAULT         , "MIPI_CTRL16"         , eReadWrite},
    {OV8810_MIPI_CTRL17         , OV8810_MIPI_CTRL17_DEFAULT         , "MIPI_CTRL17"         , eReadWrite},
    {OV8810_MIPI_CTRL18         , OV8810_MIPI_CTRL18_DEFAULT         , "MIPI_CTRL18"         , eReadWrite},
    {OV8810_MIPI_CTRL19         , OV8810_MIPI_CTRL19_DEFAULT         , "MIPI_CTRL19"         , eReadWrite},
    {OV8810_MIPI_CTRL1A         , OV8810_MIPI_CTRL1A_DEFAULT         , "MIPI_CTRL1A"         , eReadWrite},
    {OV8810_MIPI_CTRL1B         , OV8810_MIPI_CTRL1B_DEFAULT         , "MIPI_CTRL1B"         , eReadWrite},
    {OV8810_MIPI_CTRL1C         , OV8810_MIPI_CTRL1C_DEFAULT         , "MIPI_CTRL1C"         , eReadWrite},
    {OV8810_MIPI_CTRL1D         , OV8810_MIPI_CTRL1D_DEFAULT         , "MIPI_CTRL1D"         , eReadWrite},
    {OV8810_MIPI_CTRL1E         , OV8810_MIPI_CTRL1E_DEFAULT         , "MIPI_CTRL1E"         , eReadWrite},
    {OV8810_MIPI_CTRL1F         , OV8810_MIPI_CTRL1F_DEFAULT         , "MIPI_CTRL1F"         , eReadWrite},
    {OV8810_MIPI_CTRL20         , OV8810_MIPI_CTRL20_DEFAULT         , "MIPI_CTRL20"         , eReadWrite},
    {OV8810_MIPI_CTRL21         , OV8810_MIPI_CTRL21_DEFAULT         , "MIPI_CTRL21"         , eReadWrite},
    {OV8810_MIPI_CTRL22         , OV8810_MIPI_CTRL22_DEFAULT         , "MIPI_CTRL22"         , eReadWrite},
    {OV8810_MIPI_CTRL23         , OV8810_MIPI_CTRL23_DEFAULT         , "MIPI_CTRL23"         , eReadWrite},
    {OV8810_MIPI_CTRL24         , OV8810_MIPI_CTRL24_DEFAULT         , "MIPI_CTRL24"         , eReadWrite},
    {OV8810_MIPI_CTRL25         , OV8810_MIPI_CTRL25_DEFAULT         , "MIPI_CTRL25"         , eReadWrite},
    {OV8810_MIPI_CTRL26         , OV8810_MIPI_CTRL26_DEFAULT         , "MIPI_CTRL26"         , eReadWrite},
    {OV8810_MIPI_CTRL27         , OV8810_MIPI_CTRL27_DEFAULT         , "MIPI_CTRL27"         , eReadWrite},
    {OV8810_MIPI_CTRL28         , OV8810_MIPI_CTRL28_DEFAULT         , "MIPI_CTRL28"         , eReadWrite},
    {OV8810_MIPI_CTRL29         , OV8810_MIPI_CTRL29_DEFAULT         , "MIPI_CTRL29"         , eReadWrite},
    {OV8810_MIPI_CTRL2F         , OV8810_MIPI_CTRL2F_DEFAULT         , "MIPI_CTRL2F"         , eReadWrite},
    {OV8810_MIPI_CTRL30         , OV8810_MIPI_CTRL30_DEFAULT         , "MIPI_CTRL30"         , eReadWrite},
    {0x0000                     , 0x00                               , "TableEnd"            , eTableEnd}
};



/*****************************************************************************
 * AWB-Calibration data
 *****************************************************************************/

// Calibration (e.g. Matlab) is done in double precision. Parameters are then stored and handle as float
// types here in the software. Finally these parameters are written to hardware registers with fixed
// point precision.
// Some thoughts about the error between a real value, rounded to a constant with a finite number of
// fractional digits, and the resulting binary fixed point value:
// The total absolute error is the absolute error of the conversion real to constant plus the absolute
// error of the conversion from the constant to fixed point.
// For example the distance between two figures of a a fixed point value with 8 bit fractional part
// is 1/256. The max. absolute error is half of that, thus 1/512. So 3 digits fractional part could
// be chosen for the constant with an absolut error of 1/2000. The total absolute error would then be
// 1/2000 + 1/512.
// To avoid any problems we take one more digit. And another one to account for error propagation in
// the calculations of the SLS algorithms. Finally we end up with reasonable 5 digits fractional part.

/*****************************************************************************
 *
 *****************************************************************************/

// K-Factor
// calibration factor to map exposure of current sensor to the exposure of the
// reference sensor
//
// Important: This value is determinde for OV5630_1_CLKIN = 10000000 MHz and
//            need to be changed for other values
const Isi1x1FloatMatrix_t OV8810_KFactor =
{
    { 6.838349f }   // or 3.94f (to be checked)
};


// PCA matrix
const Isi3x2FloatMatrix_t OV8810_PCAMatrix =
{
    {
        -0.62791f, -0.13803f,  0.76595f,
        -0.52191f,  0.80474f, -0.28283f
    }
};


// mean values from SVD
const Isi3x1FloatMatrix_t OV8810_SVDMeanValue =
{
    {
        0.34165f,  0.37876f,  0.27959f
    }
};



/*****************************************************************************
 * Rg/Bg color space (clipping and out of range)
 *****************************************************************************/
// Center line of polygons {f_N0_Rg, f_N0_Bg, f_d}
const IsiLine_t OV8810_CenterLine =
{
//    .f_N0_Rg    = -0.6611259877402997f,
//    .f_N0_Bg    = -0.7502749018422601f,
//    .f_d        = -2.0771246154391578f
    .f_N0_Rg    = -0.6965157268655040f,
    .f_N0_Bg    = -0.7175415264840207f,
    .f_d        = -2.0547830071543265f
};



/* parameter arrays for Rg/Bg color space clipping */
#define AWB_CLIP_PARM_ARRAY_SIZE_1 16
#define AWB_CLIP_PARM_ARRAY_SIZE_2 16

// bottom left (clipping area)
float afRg1[AWB_CLIP_PARM_ARRAY_SIZE_1]      = { 0.68560f, 0.82556f,   0.89599f,   0.96987f,   1.03512f,   1.11087f,   1.18302f,   1.25407f,   1.32402f,   1.39534f,   1.46604f,   1.54006f,   1.61483f,   1.68898f,   1.76107f,   1.83316f};
float afMaxDist1[AWB_CLIP_PARM_ARRAY_SIZE_1] = { -0.06477f,    -0.02014f,   0.00013f,   0.01861f,   0.03144f,   0.04451f,   0.05378f,   0.05876f,   0.05946f,   0.05541f,   0.04642f,   0.03005f,   0.00400f,  -0.01372f,  -0.05657f,  -0.10927f};

// top right (clipping area)
float afRg2[AWB_CLIP_PARM_ARRAY_SIZE_2]      = { 0.68884f, 0.82280f,   0.90309f,   0.91978f,   0.93757f,   1.02093f,   1.18619f,   1.25186f,   1.32429f,   1.39057f,   1.46432f,   1.54130f,   1.61242f,   1.68898f,   1.76107f,   1.83316f};
float afMaxDist2[AWB_CLIP_PARM_ARRAY_SIZE_2] = { 0.07493f,  0.03084f,   0.26486f,   0.31978f,   0.37305f,   0.27866f,  -0.00038f,  -0.05020f,  -0.05042f,  -0.04452f,  -0.03720f,  -0.01971f,   0.01295f,   0.02372f,   0.06657f,   0.11927f};

#if 0
// bottom left (clipping area)
float afRg1[AWB_CLIP_PARM_ARRAY_SIZE_1]         =
{
    0.68560f, 0.82556f, 0.89599f, 0.96987f,
    1.03512f, 1.11087f, 1.18302f, 1.25407f,
    1.32402f, 1.39534f, 1.46604f, 1.54006f,
    1.61483f, 1.68898f, 1.76107f, 1.83316f
};

float afMaxDist1[AWB_CLIP_PARM_ARRAY_SIZE_1]    =
{
    -0.06477f, -0.02014f,  0.00013f,  0.01861f,
     0.03144f,  0.04451f,  0.05378f,  0.05876f,
     0.05946f,  0.05541f,  0.04642f,  0.03005f,
     0.00400f, -0.01372f, -0.05657f, -0.10927f
};

// top right (clipping area)
float afRg2[AWB_CLIP_PARM_ARRAY_SIZE_2]         =
{
    0.68884f, 0.82280f, 0.90309f, 0.91978f,
    0.93757f, 1.02093f, 1.18619f, 1.25186f,
    1.32429f, 1.39057f, 1.46432f, 1.54130f,
    1.61242f, 1.68898f, 1.76107f, 1.83316f
};

float afMaxDist2[AWB_CLIP_PARM_ARRAY_SIZE_2]    =
{
     0.07493f,  0.03084f,  0.26486f,  0.31978f,
     0.37305f,  0.27866f, -0.00038f, -0.05020f,
    -0.05042f, -0.04452f, -0.03720f, -0.01971f,
     0.01295f,  0.02372f,  0.06657f,  0.11927f
};
#endif

// structure holding pointers to above arrays
// and their sizes
const IsiAwbClipParm_t OV8810_AwbClipParm =
{
    .pRg1       = &afRg1[0],
    .pMaxDist1  = &afMaxDist1[0],
    .ArraySize1 = AWB_CLIP_PARM_ARRAY_SIZE_1,
    .pRg2       = &afRg2[0],
    .pMaxDist2  = &afMaxDist2[0],
    .ArraySize2 = AWB_CLIP_PARM_ARRAY_SIZE_2
};



/* parameter arrays for AWB out of range handling */
#define AWB_GLOBAL_FADE1_ARRAY_SIZE 16
#define AWB_GLOBAL_FADE2_ARRAY_SIZE 16

float afGlobalFade1[AWB_GLOBAL_FADE1_ARRAY_SIZE]         = { 0.50566f,  0.82658f,   0.90134f,   0.97609f,   1.05085f,   1.12560f,   1.20036f,   1.27512f,   1.34987f,   1.42463f,   1.49938f,   1.57414f,   1.64889f,   1.72365f,   1.79840f,   1.87316f};
float afGlobalGainDistance1[AWB_GLOBAL_FADE1_ARRAY_SIZE] = { -0.07183f,  0.06684f,   0.09091f,   0.11286f,   0.13161f,   0.14731f,   0.15911f,   0.16626f,   0.16816f,   0.16378f,   0.15189f,   0.13131f,   0.10173f,   0.06157f,   0.01270f,  -0.04571f};

float afGlobalFade2[AWB_GLOBAL_FADE2_ARRAY_SIZE]         = { 0.51621f,  0.73291f,   0.83798f,   0.88724f,   0.96854f,   1.07438f,   1.20953f,   1.26501f,   1.33392f,   1.40777f,   1.47237f,   1.55365f,   1.64889f,   1.72365f,   1.79840f,   1.87316f};
float afGlobalGainDistance2[AWB_GLOBAL_FADE2_ARRAY_SIZE] = { 0.39624f,  0.32108f,   0.37367f,   0.45523f,   0.46804f,   0.34652f,   0.17707f,   0.12084f,   0.09216f,   0.09031f,   0.09848f,   0.10563f,   0.09827f,   0.13843f,   0.18730f,   0.24571f};

#if 0
//bottom left
float afGlobalFade1[AWB_GLOBAL_FADE1_ARRAY_SIZE]         =
{
     0.50566f,  0.82658f,  0.90134f,  0.97609f,
     1.05085f,  1.12560f,  1.20036f,  1.27512f,
     1.34987f,  1.42463f,  1.49938f,  1.57414f,
     1.64889f,  1.72365f,  1.79840f,  1.87316f
};

float afGlobalGainDistance1[AWB_GLOBAL_FADE1_ARRAY_SIZE] =
{
    -0.07183f,  0.06684f,  0.09091f,  0.11286f,
     0.13161f,  0.14731f,  0.15911f,  0.16626f,
     0.16816f,  0.16378f,  0.15189f,  0.13131f,
     0.10173f,  0.06157f,  0.01270f, -0.04571f
};

//top right
float afGlobalFade2[AWB_GLOBAL_FADE2_ARRAY_SIZE]         =
{
     0.51621f,  0.73291f,  0.83798f,  0.88724f,
     0.96854f,  1.07438f,  1.20489f,  1.27083f,
     1.33644f,  1.40777f,  1.47237f,  1.55365f,
     1.64889f,  1.72365f,  1.79840f,  1.87316f
};

float afGlobalGainDistance2[AWB_GLOBAL_FADE2_ARRAY_SIZE] =
{
     0.39624f,  0.32108f,  0.37367f,  0.45523f,
     0.46804f,  0.34652f,  0.11493f,  0.07960f,
     0.07718f,  0.09031f,  0.09848f,  0.10563f,
     0.09827f,  0.13843f,  0.18730f,  0.24571f
};
#endif

// structure holding pointers to above arrays and their sizes
const IsiAwbGlobalFadeParm_t OV8810_AwbGlobalFadeParm =
{
    .pGlobalFade1           = &afGlobalFade1[0],
    .pGlobalGainDistance1   = &afGlobalGainDistance1[0],
    .ArraySize1             = AWB_GLOBAL_FADE1_ARRAY_SIZE,
    .pGlobalFade2           = &afGlobalFade2[0],
    .pGlobalGainDistance2   = &afGlobalGainDistance2[0],
    .ArraySize2             = AWB_GLOBAL_FADE2_ARRAY_SIZE
};


/*****************************************************************************
 * Near white pixel discrimination
 *****************************************************************************/
// parameter arrays for near white pixel parameter calculations
#define AWB_FADE2_ARRAY_SIZE 6

float afFade2[AWB_FADE2_ARRAY_SIZE]   =    {0.50, 0.75, 0.85, 1.35, 1.4, 1.5};

float afCbMinRegionMax[AWB_FADE2_ARRAY_SIZE]   = { 114, 114, 110, 85, 80,  80 };
float afCrMinRegionMax[AWB_FADE2_ARRAY_SIZE]   = {  89, 89, 90, 110, 122, 122 };
float afMaxCSumRegionMax[AWB_FADE2_ARRAY_SIZE] = {  25, 25, 30, 30, 30, 30 };

float afCbMinRegionMin[AWB_FADE2_ARRAY_SIZE]   = { 125, 125, 125, 125, 125, 120 };
float afCrMinRegionMin[AWB_FADE2_ARRAY_SIZE]   = { 125, 125, 125, 125, 125, 126 };
float afMaxCSumRegionMin[AWB_FADE2_ARRAY_SIZE] = {   5.000f,   5.000f,   5.0000f,   5.000f,   5.000f,   5.000f };

#if 0
float afFade2[AWB_FADE2_ARRAY_SIZE] =
{
    0.50000f,   1.12500f,   1.481500f,
    1.63100f,   1.74500f,   1.900000f
};

float afCbMinRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
    114.000f,   114.000f,   110.000f,
    110.000f,    95.000f,    90.000f
};

float afCrMinRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
     90.000f,    90.000f,   110.000f,
    110.000f,   122.000f,   128.000f
};

float afMaxCSumRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
     19.000f,    19.000f,    18.000f,
     16.000f,     9.000f,     9.000f
};

float afCbMinRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
    125.000f,   125.000f,   125.000f,
    125.000f,   125.000f,   120.000f
};

float afCrMinRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
    125.000f,   125.000f,   125.000f,
    125.000f,   125.000f,   126.000f
};

float afMaxCSumRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
      5.000f,     5.000f,     5.000f,
      5.000f,     5.000f,     5.000f
};
#endif


// structure holding pointers to above arrays and their sizes
const IsiAwbFade2Parm_t OV8810_AwbFade2Parm =
{
    .pFade              = &afFade2[0],
    .pCbMinRegionMax    = &afCbMinRegionMax[0],
    .pCrMinRegionMax    = &afCrMinRegionMax[0],
    .pMaxCSumRegionMax  = &afMaxCSumRegionMax[0],
    .pCbMinRegionMin    = &afCbMinRegionMin[0],
    .pCrMinRegionMin    = &afCrMinRegionMin[0],
    .pMaxCSumRegionMin  = &afMaxCSumRegionMin[0],
    .ArraySize          = AWB_FADE2_ARRAY_SIZE
};

