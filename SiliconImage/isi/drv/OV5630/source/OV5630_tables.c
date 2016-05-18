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
 * @file OV5630_tables.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"

#include "OV5630_priv.h"



/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV5630_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.
const IsiRegDescription_t OV5630_g_aRegDescription[] =
{
    /* {Address                 , DefaultValue                       , pName                 , Flags } */
    {OV5630_AGCL                , OV5630_AGCL_DEFAULT                , "AGCL"                , eReadWrite},
    {OV5630_AGCS                , OV5630_AGCS_DEFAULT                , "AGCS"                , eReadWrite},
    {OV5630_AECL_2              , OV5630_AECL_DEFAULT                , "AECL_2"              , eReadWrite_16},
    {OV5630_LAEC_2              , OV5630_LAEC_DEFAULT                , "LAEC_2"              , eReadWrite_16},
    {OV5630_RSVD_0x3006         , OV5630_RSVD_0x3006_DEFAULT         , "RSVD_0x3006"         , eReadWrite},
    {OV5630_AECS_2              , OV5630_AECS_DEFAULT                , "AECS_2"              , eReadWrite_16},
    {OV5630_PIDH                , OV5630_PIDH_DEFAULT                , "PIDH"                , eReadWrite},
    {OV5630_PIDL                , OV5630_PIDL_DEFAULT                , "PIDL"                , eReadWrite},
    {OV5630_SCCB_ID             , OV5630_SCCB_ID_DEFAULT             , "SCCB_ID"             , eReadWrite},
    {OV5630_R_PLL1              , OV5630_R_PLL1_DEFAULT              , "R_PLL1"              , eReadWrite},
    {OV5630_R_PLL2              , OV5630_R_PLL2_DEFAULT              , "R_PLL2"              , eReadWrite},
    {OV5630_R_PLL3              , OV5630_R_PLL3_DEFAULT              , "R_PLL3"              , eReadWrite},
    {OV5630_R_PLL4              , OV5630_R_PLL4_DEFAULT              , "R_PLL4"              , eReadWrite},
    {OV5630_SYS                 , OV5630_SYS_DEFAULT                 , "SYS"                 , eReadWrite},
    {OV5630_AUTO1               , OV5630_AUTO1_DEFAULT               , "AUTO1"               , eReadWrite},
    {OV5630_AUTO2               , OV5630_AUTO2_DEFAULT               , "AUTO2"               , eReadWrite},
    {OV5630_AUTO3               , OV5630_AUTO3_DEFAULT               , "AUTO3"               , eReadWrite},
    {OV5630_AUTO4               , OV5630_AUTO4_DEFAULT               , "AUTO4"               , eReadWrite},
    {OV5630_AUTO5               , OV5630_AUTO5_DEFAULT               , "AUTO5"               , eReadWrite},
    {OV5630_WPT                 , OV5630_WPT_DEFAULT                 , "WPT"                 , eReadWrite},
    {OV5630_BPT                 , OV5630_BPT_DEFAULT                 , "BPT"                 , eReadWrite},
    {OV5630_VPT                 , OV5630_VPT_DEFAULT                 , "VPT"                 , eReadWrite},
    {OV5630_YAVG                , OV5630_YAVG_DEFAULT                , "YAVG"                , eReadOnly},
    {OV5630_AECG_MAX50          , OV5630_AECG_MAX50_DEFAULT          , "AECG_MAX50"          , eReadWrite},
    {OV5630_AECG_MAX60          , OV5630_AECG_MAX60_DEFAULT          , "AECG_MAX60"          , eReadWrite},
    {OV5630_ADDVS_2             , OV5630_ADDVS_DEFAULT               , "ADDVS_2"             , eReadWrite_16},
    {OV5630_FRAME_LENGTH_LINES_2, OV5630_FRAME_LENGTH_LINES_DEFAULT  , "FRAME_LENGTH_LINES_2", eReadWrite_16},
    {OV5630_LINE_LENGTH_PCK_2   , OV5630_LINE_LENGTH_PCK_DEFAULT     , "LINE_LENGTH_PCK_2"   , eReadWrite_16},
    {OV5630_X_ADDR_START_2      , OV5630_X_ADDR_START_DEFAULT        , "X_ADDR_START_2"      , eReadWrite_16},
    {OV5630_Y_ADDR_START_2      , OV5630_Y_ADDR_START_DEFAULT        , "Y_ADDR_START_2"      , eReadWrite_16},
    {OV5630_X_ADDR_END_2        , OV5630_X_ADDR_END_DEFAULT          , "X_ADDR_END_2"        , eReadWrite_16},
    {OV5630_Y_ADDR_END_2        , OV5630_Y_ADDR_END_DEFAULT          , "Y_ADDR_END_2"        , eReadWrite_16},
    {OV5630_X_OUTPUTSIZE_2      , OV5630_X_OUTPUTSIZE_DEFAULT        , "X_OUTPUTSIZE_2"      , eReadWrite_16},
    {OV5630_Y_OUTPUTSIZE_2      , OV5630_Y_OUTPUTSIZE_DEFAULT        , "Y_OUTPUTSIZE_2"      , eReadWrite_16},
    {OV5630_FRAME_CNT           , OV5630_FRAME_CNT_DEFAULT           , "FRAME_CNT"           , eReadOnly},
    {OV5630_DATR_LMO_3          , OV5630_DATR_LMO_3_DEFAULT          , "DATR_LMO_3"          , eReadOnly},
    {OV5630_DATR_LMO_2          , OV5630_DATR_LMO_2_DEFAULT          , "DATR_LMO_2"          , eReadOnly},
    {OV5630_DATR_LMO_1          , OV5630_DATR_LMO_1_DEFAULT          , "DATR_LMO_1"          , eReadOnly},
    {OV5630_DATR_D56            , OV5630_DATR_D56_DEFAULT            , "DATR_D56"            , eReadOnly},
    {OV5630_DATR_EF             , OV5630_DATR_EF_DEFAULT             , "DATR_EF"             , eReadOnly},
    {OV5630_R_SIGMA_6           , OV5630_R_SIGMA_6_DEFAULT           , "R_SIGMA_6"           , eReadWrite},
    {OV5630_R_SIGMA_5           , OV5630_R_SIGMA_5_DEFAULT           , "R_SIGMA_5"           , eReadWrite},
    {OV5630_R_SIGMA_4           , OV5630_R_SIGMA_4_DEFAULT           , "R_SIGMA_4"           , eReadWrite},
    {OV5630_R_SIGMA_3           , OV5630_R_SIGMA_3_DEFAULT           , "R_SIGMA_3"           , eReadWrite},
    {OV5630_R_SIGMA_2           , OV5630_R_SIGMA_2_DEFAULT           , "R_SIGMA_2"           , eReadWrite},
    {OV5630_R_SIGMA_1           , OV5630_R_SIGMA_1_DEFAULT           , "R_SIGMA_1"           , eReadWrite},
    {OV5630_D56COM              , OV5630_D56COM_DEFAULT              , "D56COM"              , eReadWrite},
    {OV5630_RSVD_0x304F         , OV5630_RSVD_0x304F_DEFAULT         , "RSVD_0x304F"         , eReadWrite},
    {OV5630_R5060TH             , OV5630_R5060TH_DEFAULT             , "R5060TH"             , eReadOnly},
    {OV5630_LMO_TH1             , OV5630_LMO_TH1_DEFAULT             , "LMO_TH1"             , eReadWrite},
    {OV5630_LMO_TH2             , OV5630_LMO_TH2_DEFAULT             , "LMO_TH2"             , eReadWrite},
    {OV5630_LMO_K               , OV5630_LMO_K_DEFAULT               , "LMO_K"               , eReadWrite},
    {OV5630_BD50ST_2            , OV5630_BD50ST_DEFAULT              , "BD50ST_2"            , eReadWrite_16},
    {OV5630_BD60ST_2            , OV5630_BD60ST_DEFAULT              , "BD60ST_2"            , eReadWrite_16},
    {OV5630_RSVD_0x3065         , OV5630_RSVD_0x3065_DEFAULT         , "RSVD_0x3065"         , eReadWrite},
    {OV5630_RSVD_0x3068         , OV5630_RSVD_0x3068_DEFAULT         , "RSVD_0x3068"         , eReadWrite},
    {OV5630_RSVD_0x3069         , OV5630_RSVD_0x3069_DEFAULT         , "RSVD_0x3069"         , eReadWrite},
    {OV5630_RSVD_0x306A         , OV5630_RSVD_0x306A_DEFAULT         , "RSVD_0x306A"         , eReadWrite},
    {OV5630_HSYNST              , OV5630_HSYNST_DEFAULT              , "HSYNST"              , eReadWrite},
    {OV5630_HSYNED              , OV5630_HSYNED_DEFAULT              , "HSYNED"              , eReadWrite},
    {OV5630_HSYNED_HSYNST       , OV5630_HSYNED_HSYNST_DEFAULT       , "HSYNED_HSYNST"       , eReadWrite},
    {OV5630_TMC_RWIN0           , OV5630_TMC_RWIN0_DEFAULT           , "TMC_RWIN0"           , eReadWrite},
    {OV5630_RSVD_0x3071         , OV5630_RSVD_0x3071_DEFAULT         , "RSVD_0x3071"         , eReadWrite},
    {OV5630_RSVD_0x3072         , OV5630_RSVD_0x3072_DEFAULT         , "RSVD_0x3072"         , eReadWrite},
    {OV5630_RSVD_0x3075         , OV5630_RSVD_0x3075_DEFAULT         , "RSVD_0x3075"         , eReadWrite},
    {OV5630_RSVD_0x3076         , OV5630_RSVD_0x3076_DEFAULT         , "RSVD_0x3076"         , eReadWrite},
    {OV5630_RSVD_0x3077         , OV5630_RSVD_0x3077_DEFAULT         , "RSVD_0x3077"         , eReadWrite},
    {OV5630_RSVD_0x3078         , OV5630_RSVD_0x3078_DEFAULT         , "RSVD_0x3078"         , eReadWrite},
    {OV5630_RSVD_0x3084         , OV5630_RSVD_0x3084_DEFAULT         , "RSVD_0x3084"         , eReadWrite},
    {OV5630_RSVD_0x308A         , OV5630_RSVD_0x308A_DEFAULT         , "RSVD_0x308A"         , eReadWrite},
    {OV5630_RSVD_0x308B         , OV5630_RSVD_0x308B_DEFAULT         , "RSVD_0x308B"         , eReadWrite},
    {OV5630_RSVD_0x308D         , OV5630_RSVD_0x308D_DEFAULT         , "RSVD_0x308D"         , eReadWrite},
    {OV5630_RSVD_0x3090         , OV5630_RSVD_0x3090_DEFAULT         , "RSVD_0x3090"         , eReadWrite},
    {OV5630_RSVD_0x3091         , OV5630_RSVD_0x3091_DEFAULT         , "RSVD_0x3091"         , eReadWrite},
    {OV5630_RSVD_0x3098         , OV5630_RSVD_0x3098_DEFAULT         , "RSVD_0x3098"         , eReadWrite},
    {OV5630_RSVD_0x3099         , OV5630_RSVD_0x3099_DEFAULT         , "RSVD_0x3099"         , eReadWrite},
    {OV5630_RSVD_0x309D         , OV5630_RSVD_0x309D_DEFAULT         , "RSVD_0x309D"         , eReadWrite},
    {OV5630_RSVD_0x309E         , OV5630_RSVD_0x309E_DEFAULT         , "RSVD_0x309E"         , eReadWrite},
    {OV5630_RSVD_0x30A1         , OV5630_RSVD_0x30A1_DEFAULT         , "RSVD_0x30A1"         , eReadWrite},
    {OV5630_RSVD_0x30AC         , OV5630_RSVD_0x30AC_DEFAULT         , "RSVD_0x30AC"         , eReadWrite},
    {OV5630_RSVD_0x30AD         , OV5630_RSVD_0x30AD_DEFAULT         , "RSVD_0x30AD"         , eReadWrite},
    {OV5630_RSVD_0x30AE         , OV5630_RSVD_0x30AE_DEFAULT         , "RSVD_0x30AE"         , eReadWrite},
    {OV5630_RSVD_0x30AF         , OV5630_RSVD_0x30AF_DEFAULT         , "RSVD_0x30AF"         , eReadWrite},
    {OV5630_IO_CTRL0            , OV5630_IO_CTRL0_DEFAULT            , "IO_CTRL0"            , eReadWrite},
    {OV5630_IO_CTRL1            , OV5630_IO_CTRL1_DEFAULT            , "IO_CTRL1"            , eReadWrite},
    {OV5630_IO_CTRL2            , OV5630_IO_CTRL2_DEFAULT            , "IO_CTRL2"            , eReadWrite},
    {OV5630_DSIO_3              , OV5630_DSIO_3_DEFAULT              , "DSIO_3"              , eReadWrite},
    {OV5630_DSIO_2              , OV5630_DSIO_2_DEFAULT              , "DSIO_2"              , eReadWrite},
    {OV5630_DSIO_1              , OV5630_DSIO_1_DEFAULT              , "DSIO_1"              , eReadWrite},
    {OV5630_TMC_TMC10           , OV5630_TMC_TMC10_DEFAULT           , "TMC_TMC10"           , eReadWrite},
    {OV5630_TMC_TMC12           , OV5630_TMC_TMC12_DEFAULT           , "TMC_TMC12"           , eReadWrite},
    {OV5630_TMC_TMC14           , OV5630_TMC_TMC14_DEFAULT           , "TMC_TMC14"           , eReadWrite},
    {OV5630_TMC_COM4            , OV5630_TMC_COM4_DEFAULT            , "TMC_COM4"            , eReadWrite},
    {OV5630_TMC_REG6C           , OV5630_TMC_REG6C_DEFAULT           , "TMC_REG6C"           , eReadWrite},
    {OV5630_TMC_REG6E           , OV5630_TMC_REG6E_DEFAULT           , "TMC_REG6E"           , eReadWrite},
    {OV5630_R_CLK_RSCLK         , OV5630_R_CLK_RSCLK_DEFAULT         , "R_CLK_RSCLK"         , eReadWrite},
    {OV5630_R_CLK_RACLK         , OV5630_R_CLK_RACLK_DEFAULT         , "R_CLK_RACLK"         , eReadWrite},
    {OV5630_R_CLK_RACLK1        , OV5630_R_CLK_RACLK1_DEFAULT        , "R_CLK_RACLK1"        , eReadWrite},
    {OV5630_FRS_RFRES0          , OV5630_FRS_RFRES0_DEFAULT          , "FRS_RFRES0"          , eReadWrite},
    {OV5630_FRS_RFRES1          , OV5630_FRS_RFRES1_DEFAULT          , "FRS_RFRES1"          , eReadWrite},
    {OV5630_FRS_RFRES2          , OV5630_FRS_RFRES2_DEFAULT          , "FRS_RFRES2"          , eReadWrite},
    {OV5630_FRS_RFRES3          , OV5630_FRS_RFRES3_DEFAULT          , "FRS_RFRES3"          , eReadWrite},
    {OV5630_FRS_FECNT           , OV5630_FRS_FECNT_DEFAULT           , "FRS_FECNT"           , eReadWrite},
    {OV5630_FRS_FFCNT_2         , OV5630_FRS_FFCNT_DEFAULT           , "FRS_FFCNT_2"         , eReadWrite_16},
    {OV5630_FRS_RFRM            , OV5630_FRS_RFRM_DEFAULT            , "FRS_RFRM"            , eReadWrite},
    {OV5630_FRS_RSTRB           , OV5630_FRS_RSTRB_DEFAULT           , "FRS_RSTRB"           , eReadWrite},
    {OV5630_SA1TMC              , OV5630_SA1TMC_DEFAULT              , "SA1TMC"              , eReadWrite},
    {OV5630_TMC_30EA            , OV5630_TMC_30EA_DEFAULT            , "TMC_30EA"            , eReadWrite},
    {OV5630_TMC_30EB            , OV5630_TMC_30EB_DEFAULT            , "TMC_30EB"            , eReadWrite},
    {OV5630_FLEX_REG_TXP        , OV5630_FLEX_REG_TXP_DEFAULT        , "FLEX_REG_TXP"        , eReadWrite},
    {OV5630_FLEX_REG_FLT        , OV5630_FLEX_REG_FLT_DEFAULT        , "FLEX_REG_FLT"        , eReadWrite},
    {OV5630_FLEX_REG_TXT        , OV5630_FLEX_REG_TXT_DEFAULT        , "FLEX_REG_TXT"        , eReadWrite},
    {OV5630_FLEX_REG_HBK        , OV5630_FLEX_REG_HBK_DEFAULT        , "FLEX_REG_HBK"        , eReadWrite},
    {OV5630_FLEX_REG_HSG        , OV5630_FLEX_REG_HSG_DEFAULT        , "FLEX_REG_HSG"        , eReadWrite},
    {OV5630_FLEX_SA1SFT         , OV5630_FLEX_SA1SFT_DEFAULT         , "FLEX_SA1SFT"         , eReadWrite},
    {OV5630_RVSOPT              , OV5630_RVSOPT_DEFAULT              , "RVSOPT"              , eReadWrite},
    {OV5630_AUTO6               , OV5630_AUTO6_DEFAULT               , "AUTO6"               , eReadWrite},
    {OV5630_IMAGE_TRANSFO       , OV5630_IMAGE_TRANSFO_DEFAULT       , "IMAGE_TRANSFO"       , eReadWrite},
    {OV5630_IMAGE_LUM           , OV5630_IMAGE_LUM_DEFAULT           , "IMAGE_LUM"           , eReadWrite},
    {OV5630_IMAGE_SYSTEM        , OV5630_IMAGE_SYSTEM_DEFAULT        , "IMAGE_SYSTEM"        , eReadWrite},
    {OV5630_GROUP_WR            , OV5630_GROUP_WR_DEFAULT            , "GROUP_WR"            , eReadWrite},
    {OV5630_RSVD_0x3103         , OV5630_RSVD_0x3103_DEFAULT         , "RSVD_0x3103"         , eReadWrite},
    {OV5630_CIF_CTRL2           , OV5630_CIF_CTRL2_DEFAULT           , "CIF_CTRL2"           , eReadWrite},
    {OV5630_ISP_CTRL00          , OV5630_ISP_CTRL00_DEFAULT          , "ISP_CTRL00"          , eReadWrite},
    {OV5630_ISP_CTRL01          , OV5630_ISP_CTRL01_DEFAULT          , "ISP_CTRL01"          , eReadWrite},
    {OV5630_ISP_CTRL02          , OV5630_ISP_CTRL02_DEFAULT          , "ISP_CTRL02"          , eReadWrite},
    {OV5630_0x3303              , OV5630_0x3303_DEFAULT              , "0x3303"              , eReadWrite},
    {OV5630_DIG_GAIN_MAN        , OV5630_DIG_GAIN_MAN_DEFAULT        , "DIG_GAIN_MAN"        , eReadWrite},
    {OV5630_BIAS_MAN            , OV5630_BIAS_MAN_DEFAULT            , "BIAS_MAN"            , eReadWrite},
    {OV5630_0x3306              , OV5630_0x3306_DEFAULT              , "0x3306"              , eReadWrite},
    {OV5630_STABLE_RANGE        , OV5630_STABLE_RANGE_DEFAULT        , "STABLE_RANGE"        , eReadWrite},
    {OV5630_R_GAIN_MAN_2        , OV5630_R_GAIN_MAN_DEFAULT          , "R_GAIN_MAN_2"        , eReadWrite_16},
    {OV5630_G_GAIN_MAN_2        , OV5630_G_GAIN_MAN_DEFAULT          , "G_GAIN_MAN_2"        , eReadWrite_16},
    {OV5630_B_GAIN_MAN_2        , OV5630_B_GAIN_MAN_DEFAULT          , "B_GAIN_MAN_2"        , eReadWrite_16},
    {OV5630_STABLE_RANGEW       , OV5630_STABLE_RANGEW_DEFAULT       , "STABLE_RANGEW"       , eReadWrite},
    {OV5630_AWB_FRAME_CNT       , OV5630_AWB_FRAME_CNT_DEFAULT       , "AWB_FRAME_CNT"       , eReadWrite},
    {OV5630_0x3311              , OV5630_0x3311_DEFAULT              , "0x3311"              , eReadWrite},
    {OV5630_0x3312              , OV5630_0x3312_DEFAULT              , "0x3312"              , eReadWrite},
    {OV5630_0x3313              , OV5630_0x3313_DEFAULT              , "0x3313"              , eReadWrite},
    {OV5630_DSP_HSIZE_IN_2      , OV5630_DSP_HSIZE_IN_DEFAULT        , "DSP_HSIZE_IN_2"      , eReadWrite_16},
    {OV5630_DSP_VSIZE_IN_2      , OV5630_DSP_VSIZE_IN_DEFAULT        , "DSP_VSIZE_IN_2"      , eReadWrite_16},
    {OV5630_0x3318              , OV5630_0x3318_DEFAULT              , "0x3318"              , eReadWrite},
    {OV5630_0x3319              , OV5630_0x3319_DEFAULT              , "0x3319"              , eReadWrite},
    {OV5630_EVEN_MAN0           , OV5630_EVEN_MAN0_DEFAULT           , "EVEN_MAN0"           , eReadWrite},
    {OV5630_EVEN_MAN1           , OV5630_EVEN_MAN1_DEFAULT           , "EVEN_MAN1"           , eReadWrite},
    {OV5630_EVEN_MAN2           , OV5630_EVEN_MAN2_DEFAULT           , "EVEN_MAN2"           , eReadWrite},
    {OV5630_EVEN_MAN3           , OV5630_EVEN_MAN3_DEFAULT           , "EVEN_MAN3"           , eReadWrite},
    {OV5630_0x331E              , OV5630_0x331E_DEFAULT              , "0x331E"              , eReadWrite},
    {OV5630_0x331F              , OV5630_0x331F_DEFAULT              , "0x331F"              , eReadWrite},
    {OV5630_BLC_LMT_OPTION      , OV5630_BLC_LMT_OPTION_DEFAULT      , "BLC_LMT_OPTION"      , eReadWrite},
    {OV5630_BLC_THRE            , OV5630_BLC_THRE_DEFAULT            , "BLC_THRE"            , eReadWrite},
    {OV5630_0x3322              , OV5630_0x3322_DEFAULT              , "0x3322"              , eReadWrite},
    {OV5630_0x3323              , OV5630_0x3323_DEFAULT              , "0x3323"              , eReadWrite},
    {OV5630_BLC_MAN0_2          , OV5630_BLC_MAN0_DEFAULT            , "BLC_MAN0_2"          , eReadWrite_16},
    {OV5630_BLC_MAN1_2          , OV5630_BLC_MAN1_DEFAULT            , "BLC_MAN1_2"          , eReadWrite_16},
    {OV5630_BLC_MAN2_2          , OV5630_BLC_MAN2_DEFAULT            , "BLC_MAN2_2"          , eReadWrite_16},
    {OV5630_BLC_MAN3_2          , OV5630_BLC_MAN3_DEFAULT            , "BLC_MAN3_2"          , eReadWrite_16},
    {OV5630_BLC_MAN4_2          , OV5630_BLC_MAN4_DEFAULT            , "BLC_MAN4_2"          , eReadWrite_16},
    {OV5630_BLC_MAN5_2          , OV5630_BLC_MAN5_DEFAULT            , "BLC_MAN5_2"          , eReadWrite_16},
    {OV5630_BLC_MAN6_2          , OV5630_BLC_MAN6_DEFAULT            , "BLC_MAN6_2"          , eReadWrite_16},
    {OV5630_BLC_MAN7_2          , OV5630_BLC_MAN7_DEFAULT            , "BLC_MAN7_2"          , eReadWrite_16},
    {OV5630_AWB_RGB_INDIR_ADR   , OV5630_AWB_RGB_INDIR_ADR_DEFAULT   , "AWB_RGB_INDIR_ADR"   , eReadWrite},
    {OV5630_AWB_RGB_INDIR_DAT   , OV5630_AWB_RGB_INDIR_DAT_DEFAULT   , "AWB_RGB_INDIR_DAT"   , eReadOnlyVolNoDef},
    {OV5630_CLIP_CTRL0          , OV5630_CLIP_CTRL0_DEFAULT          , "CLIP_CTRL0"          , eReadWrite},
    {OV5630_CLIP_CTRL1          , OV5630_CLIP_CTRL1_DEFAULT          , "CLIP_CTRL1"          , eReadWrite},
    {OV5630_CLIP_CTRL2          , OV5630_CLIP_CTRL2_DEFAULT          , "CLIP_CTRL2"          , eReadWrite},
    {OV5630_CLIP_CTRL3          , OV5630_CLIP_CTRL3_DEFAULT          , "CLIP_CTRL3"          , eReadWrite},
    {OV5630_CLIP_CTRL4          , OV5630_CLIP_CTRL4_DEFAULT          , "CLIP_CTRL4"          , eReadWrite},
    {OV5630_CLIP_CTRL5          , OV5630_CLIP_CTRL5_DEFAULT          , "CLIP_CTRL5"          , eReadWrite},
    {OV5630_CLIP_CTRL6          , OV5630_CLIP_CTRL6_DEFAULT          , "CLIP_CTRL6"          , eReadWrite},
    {OV5630_CLIP_CTRL7          , OV5630_CLIP_CTRL7_DEFAULT          , "CLIP_CTRL7"          , eReadWrite},
    {OV5630_DVP_CTRL00          , OV5630_DVP_CTRL00_DEFAULT          , "DVP_CTRL00"          , eReadWrite},
    {OV5630_DVP_CTRL01          , OV5630_DVP_CTRL01_DEFAULT          , "DVP_CTRL01"          , eReadWrite},
    {OV5630_DVP_CTRL02          , OV5630_DVP_CTRL02_DEFAULT          , "DVP_CTRL02"          , eReadWrite},
    {OV5630_DVP_CTRL03          , OV5630_DVP_CTRL03_DEFAULT          , "DVP_CTRL03"          , eReadWrite},
    {OV5630_DVP_CTRL04          , OV5630_DVP_CTRL04_DEFAULT          , "DVP_CTRL04"          , eReadWrite},
    {OV5630_DVP_CTRL05          , OV5630_DVP_CTRL05_DEFAULT          , "DVP_CTRL05"          , eReadWrite},
    {OV5630_DVP_CTRL06          , OV5630_DVP_CTRL06_DEFAULT          , "DVP_CTRL06"          , eReadWrite},
    {OV5630_DVP_CTRL07          , OV5630_DVP_CTRL07_DEFAULT          , "DVP_CTRL07"          , eReadWrite},
    {OV5630_DVP_CTRL08          , OV5630_DVP_CTRL08_DEFAULT          , "DVP_CTRL08"          , eReadWrite},
    {OV5630_DVP_CTRL09          , OV5630_DVP_CTRL09_DEFAULT          , "DVP_CTRL09"          , eReadWrite},
    {OV5630_DVP_CTRL0A          , OV5630_DVP_CTRL0A_DEFAULT          , "DVP_CTRL0A"          , eReadWrite},
    {OV5630_DVP_CTRL0B          , OV5630_DVP_CTRL0B_DEFAULT          , "DVP_CTRL0B"          , eReadWrite},
    {OV5630_DVP_CTRL0C          , OV5630_DVP_CTRL0C_DEFAULT          , "DVP_CTRL0C"          , eReadWrite},
    {OV5630_DVP_CTRL0D          , OV5630_DVP_CTRL0D_DEFAULT          , "DVP_CTRL0D"          , eReadWrite},
    {OV5630_DVP_CTRL0E          , OV5630_DVP_CTRL0E_DEFAULT          , "DVP_CTRL0E"          , eReadWrite},
    {OV5630_DVP_CTRL0F          , OV5630_DVP_CTRL0F_DEFAULT          , "DVP_CTRL0F"          , eReadWrite},
    {OV5630_DVP_CTRL10          , OV5630_DVP_CTRL10_DEFAULT          , "DVP_CTRL10"          , eReadWrite},
    {OV5630_DVP_CTRL11          , OV5630_DVP_CTRL11_DEFAULT          , "DVP_CTRL11"          , eReadWrite},
    {OV5630_DVP_CTRL12          , OV5630_DVP_CTRL12_DEFAULT          , "DVP_CTRL12"          , eReadWrite},
    {OV5630_DVP_CTRL13          , OV5630_DVP_CTRL13_DEFAULT          , "DVP_CTRL13"          , eReadWrite},
    {OV5630_DVP_CTRL14          , OV5630_DVP_CTRL14_DEFAULT          , "DVP_CTRL14"          , eReadWrite},
    {OV5630_DVP_CTRL15          , OV5630_DVP_CTRL15_DEFAULT          , "DVP_CTRL15"          , eReadWrite},
    {OV5630_MIPI_CTRL00         , OV5630_MIPI_CTRL00_DEFAULT         , "MIPI_CTRL00"         , eReadWrite},
    {OV5630_MIPI_CTRL01         , OV5630_MIPI_CTRL01_DEFAULT         , "MIPI_CTRL01"         , eReadWrite},
    {OV5630_MIPI_CTRL02         , OV5630_MIPI_CTRL02_DEFAULT         , "MIPI_CTRL02"         , eReadWrite},
    {OV5630_MIPI_CTRL03         , OV5630_MIPI_CTRL03_DEFAULT         , "MIPI_CTRL03"         , eReadWrite},
    {OV5630_MIPI_CTRL04         , OV5630_MIPI_CTRL04_DEFAULT         , "MIPI_CTRL04"         , eReadWrite},
    {OV5630_MIPI_CTRL05         , OV5630_MIPI_CTRL05_DEFAULT         , "MIPI_CTRL05"         , eReadWrite},
    {OV5630_MIPI_CTRL06         , OV5630_MIPI_CTRL06_DEFAULT         , "MIPI_CTRL06"         , eReadWrite},
    {OV5630_MIPI_CTRL07         , OV5630_MIPI_CTRL07_DEFAULT         , "MIPI_CTRL07"         , eReadWrite},
    {OV5630_MIPI_CTRL08         , OV5630_MIPI_CTRL08_DEFAULT         , "MIPI_CTRL08"         , eReadWrite},
    {OV5630_MIPI_CTRL10         , OV5630_MIPI_CTRL10_DEFAULT         , "MIPI_CTRL10"         , eReadWrite},
    {OV5630_MIPI_CTRL11         , OV5630_MIPI_CTRL11_DEFAULT         , "MIPI_CTRL11"         , eReadWrite},
    {OV5630_MIPI_CTRL12         , OV5630_MIPI_CTRL12_DEFAULT         , "MIPI_CTRL12"         , eReadWrite},
    {OV5630_MIPI_CTRL13         , OV5630_MIPI_CTRL13_DEFAULT         , "MIPI_CTRL13"         , eReadWrite},
    {OV5630_MIPI_CTRL14         , OV5630_MIPI_CTRL14_DEFAULT         , "MIPI_CTRL14"         , eReadWrite},
    {OV5630_MIPI_CTRL15         , OV5630_MIPI_CTRL15_DEFAULT         , "MIPI_CTRL15"         , eReadWrite},
    {OV5630_MIPI_CTRL16         , OV5630_MIPI_CTRL16_DEFAULT         , "MIPI_CTRL16"         , eReadWrite},
    {OV5630_MIPI_CTRL17         , OV5630_MIPI_CTRL17_DEFAULT         , "MIPI_CTRL17"         , eReadWrite},
    {OV5630_MIPI_CTRL18         , OV5630_MIPI_CTRL18_DEFAULT         , "MIPI_CTRL18"         , eReadWrite},
    {OV5630_MIPI_CTRL19         , OV5630_MIPI_CTRL19_DEFAULT         , "MIPI_CTRL19"         , eReadWrite},
    {OV5630_MIPI_CTRL1A         , OV5630_MIPI_CTRL1A_DEFAULT         , "MIPI_CTRL1A"         , eReadWrite},
    {OV5630_MIPI_CTRL1B         , OV5630_MIPI_CTRL1B_DEFAULT         , "MIPI_CTRL1B"         , eReadWrite},
    {OV5630_MIPI_CTRL1C         , OV5630_MIPI_CTRL1C_DEFAULT         , "MIPI_CTRL1C"         , eReadWrite},
    {OV5630_MIPI_CTRL1D         , OV5630_MIPI_CTRL1D_DEFAULT         , "MIPI_CTRL1D"         , eReadWrite},
    {OV5630_MIPI_CTRL1E         , OV5630_MIPI_CTRL1E_DEFAULT         , "MIPI_CTRL1E"         , eReadWrite},
    {OV5630_MIPI_CTRL1F         , OV5630_MIPI_CTRL1F_DEFAULT         , "MIPI_CTRL1F"         , eReadWrite},
    {OV5630_MIPI_CTRL20         , OV5630_MIPI_CTRL20_DEFAULT         , "MIPI_CTRL20"         , eReadWrite},
    {OV5630_MIPI_CTRL21         , OV5630_MIPI_CTRL21_DEFAULT         , "MIPI_CTRL21"         , eReadWrite},
    {OV5630_MIPI_CTRL22         , OV5630_MIPI_CTRL22_DEFAULT         , "MIPI_CTRL22"         , eReadWrite},
    {OV5630_MIPI_CTRL23         , OV5630_MIPI_CTRL23_DEFAULT         , "MIPI_CTRL23"         , eReadWrite},
    {OV5630_MIPI_CTRL24         , OV5630_MIPI_CTRL24_DEFAULT         , "MIPI_CTRL24"         , eReadWrite},
    {OV5630_MIPI_CTRL25         , OV5630_MIPI_CTRL25_DEFAULT         , "MIPI_CTRL25"         , eReadWrite},
    {OV5630_MIPI_CTRL26         , OV5630_MIPI_CTRL26_DEFAULT         , "MIPI_CTRL26"         , eReadWrite},
    {OV5630_MIPI_CTRL27         , OV5630_MIPI_CTRL27_DEFAULT         , "MIPI_CTRL27"         , eReadWrite},
    {OV5630_MIPI_CTRL28         , OV5630_MIPI_CTRL28_DEFAULT         , "MIPI_CTRL28"         , eReadWrite},
    {OV5630_MIPI_CTRL29         , OV5630_MIPI_CTRL29_DEFAULT         , "MIPI_CTRL29"         , eReadWrite},
    {OV5630_MIPI_CTRL2A         , OV5630_MIPI_CTRL2A_DEFAULT         , "MIPI_CTRL2A"         , eReadWrite},
    {OV5630_MIPI_CTRL2B         , OV5630_MIPI_CTRL2B_DEFAULT         , "MIPI_CTRL2B"         , eReadWrite},
    {OV5630_MIPI_CTRL2C         , OV5630_MIPI_CTRL2C_DEFAULT         , "MIPI_CTRL2C"         , eReadWrite},
    {OV5630_MIPI_CTRL2D         , OV5630_MIPI_CTRL2D_DEFAULT         , "MIPI_CTRL2D"         , eReadWrite},
    {OV5630_MIPI_CTRL2E         , OV5630_MIPI_CTRL2E_DEFAULT         , "MIPI_CTRL2E"         , eReadWrite},
    {OV5630_MIPI_CTRL2F         , OV5630_MIPI_CTRL2F_DEFAULT         , "MIPI_CTRL2F"         , eReadWrite},
    {OV5630_MIPI_CTRL30         , OV5630_MIPI_CTRL30_DEFAULT         , "MIPI_CTRL30"         , eReadWrite},
    {OV5630_MIPI_CTRL31         , OV5630_MIPI_CTRL31_DEFAULT         , "MIPI_CTRL31"         , eReadWrite},
    {OV5630_MIPI_CTRL32         , OV5630_MIPI_CTRL32_DEFAULT         , "MIPI_CTRL32"         , eReadWrite},
    {OV5630_MIPI_CTRL33         , OV5630_MIPI_CTRL33_DEFAULT         , "MIPI_CTRL33"         , eReadWrite},
    {OV5630_MIPI_CTRL34         , OV5630_MIPI_CTRL34_DEFAULT         , "MIPI_CTRL34"         , eReadWrite},
    {OV5630_MIPI_CTRL35         , OV5630_MIPI_CTRL35_DEFAULT         , "MIPI_CTRL35"         , eReadWrite},
    {OV5630_MIPI_CTRL36         , OV5630_MIPI_CTRL36_DEFAULT         , "MIPI_CTRL36"         , eReadWrite},
    {OV5630_MIPI_CTRL37         , OV5630_MIPI_CTRL37_DEFAULT         , "MIPI_CTRL37"         , eReadWrite},
    {OV5630_MIPI_CTRL38         , OV5630_MIPI_CTRL38_DEFAULT         , "MIPI_CTRL38"         , eReadWrite},
    {OV5630_MIPI_CTRL39         , OV5630_MIPI_CTRL39_DEFAULT         , "MIPI_CTRL39"         , eReadWrite},
    {OV5630_MIPI_CTRL3A         , OV5630_MIPI_CTRL3A_DEFAULT         , "MIPI_CTRL3A"         , eReadWrite},
    {OV5630_MIPI_CTRL3B         , OV5630_MIPI_CTRL3B_DEFAULT         , "MIPI_CTRL3B"         , eReadWrite},
    {OV5630_MIPI_CTRL3C         , OV5630_MIPI_CTRL3C_DEFAULT         , "MIPI_CTRL3C"         , eReadWrite},
    {OV5630_MIPI_CTRL3D         , OV5630_MIPI_CTRL3D_DEFAULT         , "MIPI_CTRL3D"         , eReadWrite},
    {OV5630_MIPI_CTRL3E         , OV5630_MIPI_CTRL3E_DEFAULT         , "MIPI_CTRL3E"         , eReadWrite},
    {OV5630_MIPI_CTRL3F         , OV5630_MIPI_CTRL3F_DEFAULT         , "MIPI_CTRL3F"         , eReadWrite},
    {OV5630_MIPI_RO61           , OV5630_MIPI_RO61_DEFAULT           , "MIPI_RO61"           , eReadOnly},
    {OV5630_MIPI_RO62           , OV5630_MIPI_RO62_DEFAULT           , "MIPI_RO62"           , eReadOnly},
    {OV5630_MIPI_RO63           , OV5630_MIPI_RO63_DEFAULT           , "MIPI_RO63"           , eReadOnly},
    {OV5630_MIPI_RO64           , OV5630_MIPI_RO64_DEFAULT           , "MIPI_RO64"           , eReadOnly},
    {OV5630_MIPI_RO65           , OV5630_MIPI_RO65_DEFAULT           , "MIPI_RO65"           , eReadOnly},
    {OV5630_MIPI_RO66           , OV5630_MIPI_RO66_DEFAULT           , "MIPI_RO66"           , eReadOnly},
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
const Isi1x1FloatMatrix_t OV5630_KFactor =
{
    { 3.944625f }   // or 3.94f (to be checked)
};


// PCA matrix
const Isi3x2FloatMatrix_t OV5630_PCAMatrix =
{
    {
        -0.66582f,  -0.07638f,  0.74220f,
         0.47261f,  -0.81292f,  0.34031f
    }
};


// mean values from SVD
const Isi3x1FloatMatrix_t OV5630_SVDMeanValue =
{
    {
        0.32893f,  0.37670f,  0.29437f
    }
};


// exposure where the probability for inside and outside lighting is equal (reference sensor)
const Isi1x1FloatMatrix_t OV5630_GExpMiddle =
{
    { 0.033892f }
};


// standard deviation of the normal distribution of the inside light sources (sigmaI)
const Isi1x1FloatMatrix_t OV5630_VarDistrIn =
{
    { 0.931948f }
};


// mean value of the normal distribution of the inside light sources (muI)
const Isi1x1FloatMatrix_t OV5630_MeanDistrIn =
{
    { -3.529613f }
};


// variance of the normal distribution of the outside light sources (sigmaO)
const Isi1x1FloatMatrix_t OV5630_VarDistrOut =
{
    { 1.545416f }
};


// mean value of the normal distribution of the outside light sources (muO)
const Isi1x1FloatMatrix_t OV5630_MeanDistrOut =
{
    { -6.464629f }
};



/*****************************************************************************
 * Rg/Bg color space (clipping and out of range)
 *****************************************************************************/
/* Center line of polygons { f_N0_Rg, f_N0_Bg, f_d } */
const IsiLine_t OV5630_CenterLine =
{
    .f_N0_Rg    = -0.7597481694833567f,
    .f_N0_Bg    = -0.6502174397589539f,
    .f_d        = -1.8497657347968266f
};


/* parameter arrays for Rg/Bg color space clipping */
#define AWB_CLIP_PARM_ARRAY_SIZE_1 16
#define AWB_CLIP_PARM_ARRAY_SIZE_2 16

// top right (clipping area)
float afRg2[AWB_CLIP_PARM_ARRAY_SIZE_2] =
{
    0.95000f,   1.10076f,   1.14336f,   1.18854f,
    1.25372f,   1.31546f,   1.34483f,   1.39015f,
    1.43547f,   1.48079f,   1.52610f,   1.57142f,
    1.61674f,   1.66206f,   1.70738f,   1.75269f
};

float afMaxDist2[AWB_CLIP_PARM_ARRAY_SIZE_2] =
{
    0.08862f,   0.05392f,   0.11547f,   0.14087f,
    0.06364f,  -0.01615f,  -0.01701f,  -0.01825f,
   -0.01774f,  -0.01532f,  -0.01073f,  -0.00379f,
    0.00570f,   0.01789f,   0.03299f,   0.06072f
};

//bottom left (clipping area)
float afRg1[AWB_CLIP_PARM_ARRAY_SIZE_1] =
{
    0.95000f,   1.08378f,   1.15729f,   1.20968f,
    1.25420f,   1.29951f,   1.34483f,   1.39015f,
    1.43547f,   1.48079f,   1.52610f,   1.57142f,
    1.61674f,   1.66206f,   1.70738f,   1.75269f
};

float afMaxDist1[AWB_CLIP_PARM_ARRAY_SIZE_1] =
{
    0.00004f,   0.04196f,   0.03743f,   0.03620f,
    0.01975f,   0.02413f,   0.02701f,   0.02825f,
    0.02774f,   0.02532f,   0.02073f,   0.01379f,
    0.00430f,  -0.00789f,  -0.02299f,  -0.05072f
};

// structure holding pointers to above arrays
// and their sizes
const IsiAwbClipParm_t OV5630_AwbClipParm =
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

//top right
float afGlobalFade2[AWB_GLOBAL_FADE2_ARRAY_SIZE] =
{
    0.80064f,   0.88574f,   0.96285f,   1.01108f,
    1.05638f,   1.11035f,   1.18078f,   1.28386f,
    1.33501f,   1.41174f,   1.46283f,   1.50743f,
    1.57952f,   1.62709f,   1.72651f,   1.79269f
};

float afGlobalGainDistance2[AWB_GLOBAL_FADE2_ARRAY_SIZE] =
{
    0.15477f,   0.13279f,   0.12127f,   0.12486f,
    0.11925f,   0.17622f,   0.19771f,   0.08278f,
    0.08465f,   0.08746f,   0.10719f,   0.12248f,
    0.10881f,   0.09918f,   0.13533f,   0.17657f
};

//bottom left
float afGlobalFade1[AWB_GLOBAL_FADE1_ARRAY_SIZE] =
{
    0.80291f,   0.86210f,   0.92531f,   1.01277f,
    1.05696f,   1.15824f,   1.21259f,   1.25626f,
    1.32019f,   1.38210f,   1.44976f,   1.52407f,
    1.57684f,   1.66033f,   1.72651f,   1.79269f
};

float afGlobalGainDistance1[AWB_GLOBAL_FADE1_ARRAY_SIZE] =
{
   -0.01758f,  -0.00363f,   0.03190f,   0.09184f,
    0.12491f,   0.19927f,   0.09898f,   0.09375f,
    0.09136f,   0.09258f,   0.09133f,   0.09382f,
    0.09054f,   0.08761f,   0.06467f,   0.02343f
};

// structure holding pointers to above arrays and their sizes
const IsiAwbGlobalFadeParm_t OV5630_AwbGlobalFadeParm =
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

float afFade2[AWB_FADE2_ARRAY_SIZE] =
{
    0.50000f,   1.00125f, 1.318535f,
    1.45159f,   1.55305f, 1.640000f
};

float afCbMinRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
    114.000f,   114.000f,   105.000f,
     95.000f,    95.000f,    90.000f
};

float afCrMinRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
     83.000f,    83.000f,   110.000f,
    120.000f,   122.000f,   128.000f
};

float afMaxCSumRegionMax[AWB_FADE2_ARRAY_SIZE] =
{
     28.000f,    27.000f,    18.000f,
     16.000f,     9.000f,     9.000f
};

float afCbMinRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
    123.000f,   123.000f,   123.000f,
    123.000f,   123.000f,   120.000f
};

float afCrMinRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
    123.000f,   123.000f,   123.000f,
    123.000f,   123.000f,   126.000f
};

float afMaxCSumRegionMin[AWB_FADE2_ARRAY_SIZE] =
{
      5.000f,     5.000f,     5.000f,
      5.000f,     5.000f,     5.000f
};

// structure holding pointers to above arrays and their sizes
const IsiAwbFade2Parm_t OV5630_AwbFade2Parm =
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

