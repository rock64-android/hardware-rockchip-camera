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
 * @file OV2715_tables.c
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

#include "OV2715_priv.h"



/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV2715_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.
const IsiRegDescription_t OV2715_g_aRegDescription[] =
{
  //{ulAddr                                 ,ulDefaultValue                                     ,pszName                                    ,ulFlags       }
    { OV2715_SYSTEM_CONTROL00               ,OV2715_SYSTEM_CONTROL00_DEFAULT                    ,"SYSTEM_CONTROL00"                         ,eReadWrite    },
    { OV2715_PIDH                           ,OV2715_PIDH_DEFAULT                                ,"PIDH"                                     ,eReadOnly     },
    { OV2715_PIDL                           ,OV2715_PIDL_DEFAULT                                ,"PIDL"                                     ,eReadOnly     },
    { OV2715_MIPI_CTRL00                    ,OV2715_MIPI_CTRL00_DEFAULT                         ,"MIPI_CTRL00"                              ,eReadWrite    },
    { OV2715_PLL_CTRL00                     ,OV2715_PLL_CTRL00_DEFAULT                          ,"PLL_CTRL00"                               ,eReadWrite    },
    { OV2715_PLL_CTRL01                     ,OV2715_PLL_CTRL01_DEFAULT                          ,"PLL_CTRL01"                               ,eReadWrite    },
    { OV2715_PLL_CTRL02                     ,OV2715_PLL_CTRL02_DEFAULT                          ,"PLL_CTRL02"                               ,eReadWrite    },
    { OV2715_PLL_PREDIVIDER                 ,OV2715_PLL_PREDIVIDER_DEFAULT                      ,"PLL_PREDIVIDER"                           ,eReadWrite    },
    { OV2715_PAD_OUTPUT_ENABLE00            ,OV2715_PAD_OUTPUT_ENABLE00_DEFAULT                 ,"PAD_OUTPUT_ENABLE00"                      ,eReadWrite    },
    { OV2715_PAD_OUTPUT_ENABLE01            ,OV2715_PAD_OUTPUT_ENABLE01_DEFAULT                 ,"PAD_OUTPUT_ENABLE01"                      ,eReadWrite    },
    { OV2715_PAD_OUTPUT_ENABLE02            ,OV2715_PAD_OUTPUT_ENABLE02_DEFAULT                 ,"PAD_OUTPUT_ENABLE02"                      ,eReadWrite    },
    { OV2715_PAD_OUTPUT_VALUE00             ,OV2715_PAD_OUTPUT_VALUE00_DEFAULT                  ,"PAD_OUTPUT_VALUE00"                       ,eReadWrite    },
    { OV2715_PAD_OUTPUT_VALUE01             ,OV2715_PAD_OUTPUT_VALUE01_DEFAULT                  ,"PAD_OUTPUT_VALUE01"                       ,eReadWrite    },
    { OV2715_PAD_OUTPUT_VALUE02             ,OV2715_PAD_OUTPUT_VALUE02_DEFAULT                  ,"PAD_OUTPUT_VALUE02"                       ,eReadWrite    },
    { OV2715_PAD_OUTPUT_SELECT00            ,OV2715_PAD_OUTPUT_SELECT00_DEFAULT                 ,"PAD_OUTPUT_SELECT00"                      ,eReadWrite    },
    { OV2715_PAD_OUTPUT_SELECT01            ,OV2715_PAD_OUTPUT_SELECT01_DEFAULT                 ,"PAD_OUTPUT_SELECT01"                      ,eReadWrite    },
    { OV2715_PAD_OUTPUT_SELECT02            ,OV2715_PAD_OUTPUT_SELECT02_DEFAULT                 ,"PAD_OUTPUT_SELECT02"                      ,eReadWrite    },
    { OV2715_CHIP_REVISION                  ,OV2715_CHIP_REVISION_DEFAULT                       ,"CHIP_REVISION"                            ,eReadOnly     },
    { OV2715_PAD_OUTPUT_DRIVE_CAPABILITY    ,OV2715_PAD_OUTPUT_DRIVE_CAPABILITY_DEFAULT         ,"PAD_OUTPUT_DRIVE_CAPABILITY"              ,eReadWrite    },
    { OV2715_SCCB_ID                        ,OV2715_SCCB_ID_DEFAULT                             ,"SCCB_ID"                                  ,eReadWrite    },
    { OV2715_PLL_CLOCK_SELECT               ,OV2715_PLL_CLOCK_SELECT_DEFAULT                    ,"PLL_CLOCK_SELECT"                         ,eReadWrite    },
    { OV2715_SCCB_PAD_CLOCK_DIVIDER         ,OV2715_SCCB_PAD_CLOCK_DIVIDER_DEFAULT              ,"SCCB_PAD_CLOCK_DIVIDER"                   ,eReadWrite    },
    { OV2715_GROUP_ADDR0                    ,OV2715_GROUP_ADDR0_DEFAULT                         ,"GROUP_ADDR0"                              ,eReadWrite    },
    { OV2715_GROUP_ADDR1                    ,OV2715_GROUP_ADDR1_DEFAULT                         ,"GROUP_ADDR1"                              ,eReadWrite    },
    { OV2715_GROUP_ADDR2                    ,OV2715_GROUP_ADDR2_DEFAULT                         ,"GROUP_ADDR2"                              ,eReadWrite    },
    { OV2715_GROUP_ADDR3                    ,OV2715_GROUP_ADDR3_DEFAULT                         ,"GROUP_ADDR3"                              ,eReadWrite    },
    { OV2715_GROUP_ACCESS                   ,OV2715_GROUP_ACCESS_DEFAULT                        ,"GROUP_ACCESS"                             ,eReadWriteVolatile },
    { OV2715_ANA_ARRAY_01                   ,OV2715_ANA_ARRAY_01_DEFAULT                        ,"ANA_ARRAY_01"                             ,eReadWrite    },
    { OV2715_SENSOR_REG0D                   ,OV2715_SENSOR_REG0D_DEFAULT                        ,"SENSOR_REG0D"                             ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HS_HIGHBYTE     ,OV2715_TIMING_CONTROL_HS_HIGHBYTE_DEFAULT          ,"TIMING_CONTROL_HS_HIGHBYTE"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VH_LOWBYTE0     ,OV2715_TIMING_CONTROL_VH_LOWBYTE0_DEFAULT          ,"TIMING_CONTROL_VH_LOWBYTE0"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VH_HIGHBYTE0    ,OV2715_TIMING_CONTROL_VH_HIGHBYTE0_DEFAULT         ,"TIMING_CONTROL_VH_HIGHBYTE0"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VH_LOWBYTE1     ,OV2715_TIMING_CONTROL_VH_LOWBYTE1_DEFAULT          ,"TIMING_CONTROL_VH_LOWBYTE1"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HW_HIGHBYTE     ,OV2715_TIMING_CONTROL_HW_HIGHBYTE_DEFAULT          ,"TIMING_CONTROL_HW_HIGHBYTE"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HW_LOWBYTE      ,OV2715_TIMING_CONTROL_HW_LOWBYTE_DEFAULT           ,"TIMING_CONTROL_HW_LOWBYTE"                ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VH_HIGHBYTE1    ,OV2715_TIMING_CONTROL_VH_HIGHBYTE1_DEFAULT         ,"TIMING_CONTROL_VH_HIGHBYTE1"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VH_LOWBYTE2     ,OV2715_TIMING_CONTROL_VH_LOWBYTE2_DEFAULT          ,"TIMING_CONTROL_VH_LOWBYTE2"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_DVP_HSIZE       ,OV2715_TIMING_CONTROL_DVP_HSIZE_DEFAULT            ,"TIMING_CONTROL_DVP_HSIZE"                 ,eReadWrite    },
    { OV2715_TIMING_CONTROL_DVP_HSIZELOW    ,OV2715_TIMING_CONTROL_DVP_HSIZELOW_DEFAULT         ,"TIMING_CONTROL_DVP_HSIZELOW"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_DVP_VSIZEHIGH   ,OV2715_TIMING_CONTROL_DVP_VSIZEHIGH_DEFAULT        ,"TIMING_CONTROL_DVP_VSIZEHIGH"             ,eReadWrite    },
    { OV2715_TIMING_CONTROL_DVP_VSIZELOW    ,OV2715_TIMING_CONTROL_DVP_VSIZELOW_DEFAULT         ,"TIMING_CONTROL_DVP_VSIZELOW"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HTS_HIGHBYTE    ,OV2715_TIMING_CONTROL_HTS_HIGHBYTE_DEFAULT         ,"TIMING_CONTROL_HTS_HIGHBYTE"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HTS_LOWBYTE     ,OV2715_TIMING_CONTROL_HTS_LOWBYTE_DEFAULT          ,"TIMING_CONTROL_HTS_LOWBYTE"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VTS_HIGHBYTE    ,OV2715_TIMING_CONTROL_VTS_HIGHBYTE_DEFAULT         ,"TIMING_CONTROL_VTS_HIGHBYTE"              ,eReadWrite    },
    { OV2715_TIMING_CONTROL_VTS_LOWBYTE     ,OV2715_TIMING_CONTROL_VTS_LOWBYTE_DEFAULT          ,"TIMING_CONTROL_VTS_LOWBYTE"               ,eReadWrite    },
    { OV2715_TIMING_CONTROL_HV_OFFSET       ,OV2715_TIMING_CONTROL_HV_OFFSET_DEFAULT            ,"TIMING_CONTROL_HV_OFFSET"                 ,eReadWrite    },
    { OV2715_TIMING_CONTROL18               ,OV2715_TIMING_CONTROL18_DEFAULT                    ,"TIMING_CONTROL18"                         ,eReadWrite    },
    { OV2715_AEC_PK_EXPO0                   ,OV2715_AEC_PK_EXPO0_DEFAULT                        ,"AEC_PK_EXPO0"                             ,eReadWrite    },
    { OV2715_AEC_PK_EXPO1                   ,OV2715_AEC_PK_EXPO1_DEFAULT                        ,"AEC_PK_EXPO1"                             ,eReadWrite    },
    { OV2715_AEC_PK_EXPO2                   ,OV2715_AEC_PK_EXPO2_DEFAULT                        ,"AEC_PK_EXPO2"                             ,eReadWrite    },
    { OV2715_AEC_PK_MANUAL                  ,OV2715_AEC_PK_MANUAL_DEFAULT                       ,"AEC_PK_MANUAL"                            ,eReadWrite    },
    { OV2715_AEC_AGC_ADJ0                   ,OV2715_AEC_AGC_ADJ0_DEFAULT                        ,"AEC_AGC_ADJ"                              ,eReadWrite    },
    { OV2715_AEC_AGC_ADJ1                   ,OV2715_AEC_AGC_ADJ1_DEFAULT                        ,"AEC_AGC_ADJ"                              ,eReadWrite    },
    { OV2715_AEC_PK_VTS0                    ,OV2715_AEC_PK_VTS0_DEFAULT                         ,"AEC_PK_VTS"                               ,eReadWrite    },
    { OV2715_AEC_PK_VTS1                    ,OV2715_AEC_PK_VTS1_DEFAULT                         ,"AEC_PK_VTS"                               ,eReadWrite    },
    { OV2715_AEC_CONTROL0                   ,OV2715_AEC_CONTROL0_DEFAULT                        ,"AEC_CONTROL0"                             ,eReadWrite    },
    { OV2715_AEC_CONTROL1                   ,OV2715_AEC_CONTROL1_DEFAULT                        ,"AEC_CONTROL1"                             ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_60A               ,OV2715_AEC_MAX_EXPO_60A_DEFAULT                    ,"AEC_MAX_EXPO_60A"                         ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_60B               ,OV2715_AEC_MAX_EXPO_60B_DEFAULT                    ,"AEC_MAX_EXPO_60B"                         ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_60C               ,OV2715_AEC_MAX_EXPO_60C_DEFAULT                    ,"AEC_MAX_EXPO_60C"                         ,eReadWrite    },
    { OV2715_AEC_B50_STEP0                  ,OV2715_AEC_B50_STEP0_DEFAULT                       ,"AEC_B50_STEP0"                            ,eReadWrite    },
    { OV2715_AEC_B50_STEP1                  ,OV2715_AEC_B50_STEP1_DEFAULT                       ,"AEC_B50_STEP1"                            ,eReadWrite    },
    { OV2715_AEC_B60_STEP0                  ,OV2715_AEC_B60_STEP0_DEFAULT                       ,"AEC_B60_STEP0"                            ,eReadWrite    },
    { OV2715_AEC_B60_STEP1                  ,OV2715_AEC_B60_STEP1_DEFAULT                       ,"AEC_B60_STEP1"                            ,eReadWrite    },
    { OV2715_AEC_CONTROLD                   ,OV2715_AEC_CONTROLD_DEFAULT                        ,"AEC_CONTROLD"                             ,eReadWrite    },
    { OV2715_AEC_CONTROLE                   ,OV2715_AEC_CONTROLE_DEFAULT                        ,"AEC_CONTROLE"                             ,eReadWrite    },
    { OV2715_AEC_CONTROLF                   ,OV2715_AEC_CONTROLF_DEFAULT                        ,"AEC_CONTROLF"                             ,eReadWrite    },
    { OV2715_AEC_CONTROL10                  ,OV2715_AEC_CONTROL10_DEFAULT                       ,"AEC_CONTROL10"                            ,eReadWrite    },
    { OV2715_AEC_CONTROL11                  ,OV2715_AEC_CONTROL11_DEFAULT                       ,"AEC_CONTROL11"                            ,eReadWrite    },
    { OV2715_AEC_CONTROL12                  ,OV2715_AEC_CONTROL12_DEFAULT                       ,"AEC_CONTROL12"                            ,eReadWrite    },
    { OV2715_AEC_CONTROL13                  ,OV2715_AEC_CONTROL13_DEFAULT                       ,"AEC_CONTROL13"                            ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_50A               ,OV2715_AEC_MAX_EXPO_50A_DEFAULT                    ,"AEC_MAX_EXPO_50A"                         ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_50B               ,OV2715_AEC_MAX_EXPO_50B_DEFAULT                    ,"AEC_MAX_EXPO_50B"                         ,eReadWrite    },
    { OV2715_AEC_MAX_EXPO_50C               ,OV2715_AEC_MAX_EXPO_50C_DEFAULT                    ,"AEC_MAX_EXPO_50C"                         ,eReadWrite    },
    { OV2715_AEC_CONTROL17                  ,OV2715_AEC_CONTROL17_DEFAULT                       ,"AEC_CONTROL17"                            ,eReadWrite    },
    { OV2715_AEC_G_CEIL0                    ,OV2715_AEC_G_CEIL0_DEFAULT                         ,"AEC_G_CEIL"                               ,eReadWrite    },
    { OV2715_AEC_G_CEIL1                    ,OV2715_AEC_G_CEIL1_DEFAULT                         ,"AEC_G_CEIL"                               ,eReadWrite    },
    { OV2715_AEC_CONTROL1B                  ,OV2715_AEC_CONTROL1B_DEFAULT                       ,"AEC_CONTROL1B"                            ,eReadWrite    },
    { OV2715_AEC_LED_ADD_ROW0               ,OV2715_AEC_LED_ADD_ROW0_DEFAULT                    ,"AEC_LED_ADD_ROW"                          ,eReadWrite    },
    { OV2715_AEC_LED_ADD_ROW1               ,OV2715_AEC_LED_ADD_ROW1_DEFAULT                    ,"AEC_LED_ADD_ROW"                          ,eReadWrite    },
    { OV2715_AEC_CONTROL1E                  ,OV2715_AEC_CONTROL1E_DEFAULT                       ,"AEC_CONTROL1E"                            ,eReadWrite    },
    { OV2715_AEC_CONTROL1F                  ,OV2715_AEC_CONTROL1F_DEFAULT                       ,"AEC_CONTROL1F"                            ,eReadWrite    },
    { OV2715_AEC_CONTROL20                  ,OV2715_AEC_CONTROL20_DEFAULT                       ,"AEC_CONTROL20"                            ,eReadWrite    },
    { OV2715_OTP_DATA_0                     ,OV2715_OTP_DATA_0_DEFAULT                          ,"OTP_DATA_0"                               ,eReadWrite    },
    { OV2715_OTP_DATA_1                     ,OV2715_OTP_DATA_1_DEFAULT                          ,"OTP_DATA_1"                               ,eReadWrite    },
    { OV2715_OTP_DATA_2                     ,OV2715_OTP_DATA_2_DEFAULT                          ,"OTP_DATA_2"                               ,eReadWrite    },
    { OV2715_OTP_DATA_3                     ,OV2715_OTP_DATA_3_DEFAULT                          ,"OTP_DATA_3"                               ,eReadWrite    },
    { OV2715_OTP_DATA_4                     ,OV2715_OTP_DATA_4_DEFAULT                          ,"OTP_DATA_4"                               ,eReadWrite    },
    { OV2715_OTP_DATA_5                     ,OV2715_OTP_DATA_5_DEFAULT                          ,"OTP_DATA_5"                               ,eReadWrite    },
    { OV2715_OTP_DATA_6                     ,OV2715_OTP_DATA_6_DEFAULT                          ,"OTP_DATA_6"                               ,eReadWrite    },
    { OV2715_OTP_DATA_7                     ,OV2715_OTP_DATA_7_DEFAULT                          ,"OTP_DATA_7"                               ,eReadWrite    },
    { OV2715_OTP_DATA_8                     ,OV2715_OTP_DATA_8_DEFAULT                          ,"OTP_DATA_8"                               ,eReadWrite    },
    { OV2715_OTP_DATA_9                     ,OV2715_OTP_DATA_9_DEFAULT                          ,"OTP_DATA_9"                               ,eReadWrite    },
    { OV2715_OTP_DATA_A                     ,OV2715_OTP_DATA_A_DEFAULT                          ,"OTP_DATA_A"                               ,eReadWrite    },
    { OV2715_OTP_DATA_B                     ,OV2715_OTP_DATA_B_DEFAULT                          ,"OTP_DATA_B"                               ,eReadWrite    },
    { OV2715_OTP_DATA_C                     ,OV2715_OTP_DATA_C_DEFAULT                          ,"OTP_DATA_C"                               ,eReadWrite    },
    { OV2715_OTP_DATA_D                     ,OV2715_OTP_DATA_D_DEFAULT                          ,"OTP_DATA_D"                               ,eReadWrite    },
    { OV2715_OTP_DATA_E                     ,OV2715_OTP_DATA_E_DEFAULT                          ,"OTP_DATA_E"                               ,eReadWrite    },
    { OV2715_OTP_DATA_F                     ,OV2715_OTP_DATA_F_DEFAULT                          ,"OTP_DATA_F"                               ,eReadWrite    },
    { OV2715_OTP_CONTROL                    ,OV2715_OTP_CONTROL_DEFAULT                         ,"OTP_CONTROL"                              ,eReadWrite    },
    { OV2715_BLC_CONTROL_00                 ,OV2715_BLC_CONTROL_00_DEFAULT                      ,"BLC_CONTROL_00"                           ,eReadWrite    },
    { OV2715_BLC_CONTROL_01                 ,OV2715_BLC_CONTROL_01_DEFAULT                      ,"BLC_CONTROL_01"                           ,eReadWrite    },
    { OV2715_BLC_CONTROL_02                 ,OV2715_BLC_CONTROL_02_DEFAULT                      ,"BLC_CONTROL_02"                           ,eReadWrite    },
    { OV2715_BLC_FRAME_CONTROL              ,OV2715_BLC_FRAME_CONTROL_DEFAULT                   ,"BLC_FRAME_CONTROL"                        ,eReadWrite    },
    { OV2715_FRAME_CTRL00                   ,OV2715_FRAME_CTRL00_DEFAULT                        ,"FRAME_CTRL00"                             ,eReadWrite    },
    { OV2715_FRAME_CTRL01                   ,OV2715_FRAME_CTRL01_DEFAULT                        ,"FRAME_CTRL01"                             ,eReadWrite    },
    { OV2715_DVP_CTRL00                     ,OV2715_DVP_CTRL00_DEFAULT                          ,"DVP_CTRL00"                               ,eReadWrite    },
    { OV2715_DVP_CTRL01                     ,OV2715_DVP_CTRL01_DEFAULT                          ,"DVP_CTRL01"                               ,eReadWrite    },
    { OV2715_DVP_CTRL02                     ,OV2715_DVP_CTRL02_DEFAULT                          ,"DVP_CTRL02"                               ,eReadWrite    },
    { OV2715_DVP_CTRL03                     ,OV2715_DVP_CTRL03_DEFAULT                          ,"DVP_CTRL03"                               ,eReadWrite    },
    { OV2715_MIPI_CTRL_00                   ,OV2715_MIPI_CTRL_00_DEFAULT                        ,"MIPI_CTRL_00"                             ,eReadWrite    },
    { OV2715_MIPI_CTRL_01                   ,OV2715_MIPI_CTRL_01_DEFAULT                        ,"MIPI_CTRL_01"                             ,eReadWrite    },
    { OV2715_MIPI_CTRL_03                   ,OV2715_MIPI_CTRL_03_DEFAULT                        ,"MIPI_CTRL_03"                             ,eReadWrite    },
    { OV2715_MIPI_CTRL_04                   ,OV2715_MIPI_CTRL_04_DEFAULT                        ,"MIPI_CTRL_04"                             ,eReadWrite    },
    { OV2715_MIPI_CTRL_05                   ,OV2715_MIPI_CTRL_05_DEFAULT                        ,"MIPI_CTRL_05"                             ,eReadWrite    },
    { OV2715_MAX_FCNT_H                     ,OV2715_MAX_FCNT_H_DEFAULT                          ,"MAX_FCNT_H"                               ,eReadWrite    },
    { OV2715_MAX_FCNT_L                     ,OV2715_MAX_FCNT_L_DEFAULT                          ,"MAX_FCNT_L"                               ,eReadWrite    },
    { OV2715_MIN_SPKT_WC_REG_H              ,OV2715_MIN_SPKT_WC_REG_H_DEFAULT                   ,"MIN_SPKT_WC_REG_H"                        ,eReadWrite    },
    { OV2715_MIN_SPKT_WC_REG_L              ,OV2715_MIN_SPKT_WC_REG_L_DEFAULT                   ,"MIN_SPKT_WC_REG_L"                        ,eReadWrite    },
    { OV2715_MIPI_CTRL_14                   ,OV2715_MIPI_CTRL_14_DEFAULT                        ,"MIPI_CTRL_14"                             ,eReadWrite    },
    { OV2715_MIPI_SPKT_DT                   ,OV2715_MIPI_SPKT_DT_DEFAULT                        ,"MIPI_SPKT_DT"                             ,eReadWrite    },
    { OV2715_MIN_HS_ZERO_H                  ,OV2715_MIN_HS_ZERO_H_DEFAULT                       ,"MIN_HS_ZERO_H"                            ,eReadWrite    },
    { OV2715_MIN_HS_ZERO_L                  ,OV2715_MIN_HS_ZERO_L_DEFAULT                       ,"MIN_HS_ZERO_L"                            ,eReadWrite    },
    { OV2715_MIN_MIPI_HS_TRAIL_H            ,OV2715_MIN_MIPI_HS_TRAIL_H_DEFAULT                 ,"MIN_MIPI_HS_TRAIL_H"                      ,eReadWrite    },
    { OV2715_MIN_MIPI_HS_TRAIL_L            ,OV2715_MIN_MIPI_HS_TRAIL_L_DEFAULT                 ,"MIN_MIPI_HS_TRAIL_L"                      ,eReadWrite    },
    { OV2715_MIN_MIPI_CLK_ZERO_H            ,OV2715_MIN_MIPI_CLK_ZERO_H_DEFAULT                 ,"MIN_MIPI_CLK_ZERO_H"                      ,eReadWrite    },
    { OV2715_MIN_MIPI_CLK_ZERO_L            ,OV2715_MIN_MIPI_CLK_ZERO_L_DEFAULT                 ,"MIN_MIPI_CLK_ZERO_L"                      ,eReadWrite    },
    { OV2715_MIN_MIPI_CLK_PREPARE_H         ,OV2715_MIN_MIPI_CLK_PREPARE_H_DEFAULT              ,"MIN_MIPI_CLK_PREPARE_H"                   ,eReadWrite    },
    { OV2715_MIN_MIPI_CLK_PREPARE_L         ,OV2715_MIN_MIPI_CLK_PREPARE_L_DEFAULT              ,"MIN_MIPI_CLK_PREPARE_L"                   ,eReadWrite    },
    { OV2715_MIN_CLK_POST_H                 ,OV2715_MIN_CLK_POST_H_DEFAULT                      ,"MIN_CLK_POST_H"                           ,eReadWrite    },
    { OV2715_MIN_CLK_POST_L                 ,OV2715_MIN_CLK_POST_L_DEFAULT                      ,"MIN_CLK_POST_L"                           ,eReadWrite    },
    { OV2715_MIN_CLK_TRAIL_H                ,OV2715_MIN_CLK_TRAIL_H_DEFAULT                     ,"MIN_CLK_TRAIL_H"                          ,eReadWrite    },
    { OV2715_MIN_CLK_TRAIL_L                ,OV2715_MIN_CLK_TRAIL_L_DEFAULT                     ,"MIN_CLK_TRAIL_L"                          ,eReadWrite    },
    { OV2715_MIN_LPX_PCLK_H                 ,OV2715_MIN_LPX_PCLK_H_DEFAULT                      ,"MIN_LPX_PCLK_H"                           ,eReadWrite    },
    { OV2715_MIN_LPX_PCLK_L                 ,OV2715_MIN_LPX_PCLK_L_DEFAULT                      ,"MIN_LPX_PCLK_L"                           ,eReadWrite    },
    { OV2715_MIN_HS_PREPARE_H               ,OV2715_MIN_HS_PREPARE_H_DEFAULT                    ,"MIN_HS_PREPARE_H"                         ,eReadWrite    },
    { OV2715_MIN_HS_PREPARE_L               ,OV2715_MIN_HS_PREPARE_L_DEFAULT                    ,"MIN_HS_PREPARE_L"                         ,eReadWrite    },
    { OV2715_MIN_HS_EXIT_H                  ,OV2715_MIN_HS_EXIT_H_DEFAULT                       ,"MIN_HS_EXIT_H"                            ,eReadWrite    },
    { OV2715_MIN_HS_EXIT_L                  ,OV2715_MIN_HS_EXIT_L_DEFAULT                       ,"MIN_HS_EXIT_L"                            ,eReadWrite    },
    { OV2715_MIN_HS_ZERO_UI                 ,OV2715_MIN_HS_ZERO_UI_DEFAULT                      ,"MIN_HS_ZERO_UI"                           ,eReadWrite    },
    { OV2715_MIN_HS_TRAIL_UI                ,OV2715_MIN_HS_TRAIL_UI_DEFAULT                     ,"MIN_HS_TRAIL_UI"                          ,eReadWrite    },
    { OV2715_MIN_CLK_ZERO_UI                ,OV2715_MIN_CLK_ZERO_UI_DEFAULT                     ,"MIN_CLK_ZERO_UI"                          ,eReadWrite    },
    { OV2715_MIN_CLK_PREPARE_UI             ,OV2715_MIN_CLK_PREPARE_UI_DEFAULT                  ,"MIN_CLK_PREPARE_UI"                       ,eReadWrite    },
    { OV2715_MIN_CLK_POST_UI                ,OV2715_MIN_CLK_POST_UI_DEFAULT                     ,"MIN_CLK_POST_UI"                          ,eReadWrite    },
    { OV2715_MIN_CLK_TRAIL_UI               ,OV2715_MIN_CLK_TRAIL_UI_DEFAULT                    ,"MIN_CLK_TRAIL_UI"                         ,eReadWrite    },
    { OV2715_MIN_LPX_PCLK_UI                ,OV2715_MIN_LPX_PCLK_UI_DEFAULT                     ,"MIN_LPX_PCLK_UI"                          ,eReadWrite    },
    { OV2715_MIN_HS_PREPARE_UI              ,OV2715_MIN_HS_PREPARE_UI_DEFAULT                   ,"MIN_HS_PREPARE_UI"                        ,eReadWrite    },
    { OV2715_MIN_HS_EXIT_UI                 ,OV2715_MIN_HS_EXIT_UI_DEFAULT                      ,"MIN_HS_EXIT_UI"                           ,eReadWrite    },
    { OV2715_ISP_CONTROL0                   ,OV2715_ISP_CONTROL0_DEFAULT                        ,"ISP_CONTROL0"                             ,eReadWrite    },
    { OV2715_ISP_CONTROL1                   ,OV2715_ISP_CONTROL1_DEFAULT                        ,"ISP_CONTROL1"                             ,eReadWrite    },
    { OV2715_ISP_CONTROL2                   ,OV2715_ISP_CONTROL2_DEFAULT                        ,"ISP_CONTROL2"                             ,eReadWrite    },
    { OV2715_ISP_CONTROL5                   ,OV2715_ISP_CONTROL5_DEFAULT                        ,"ISP_CONTROL5"                             ,eReadWrite    },
    { OV2715_ISP_CONTROL31                  ,OV2715_ISP_CONTROL31_DEFAULT                       ,"ISP_CONTROL31"                            ,eReadWrite    },
    { OV2715_ISP_TEST                       ,OV2715_ISP_TEST_DEFAULT                            ,"ISP_TEST"                                 ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_RED_GAIN0          ,OV2715_AWB_GAIN_PK_RED_GAIN0_DEFAULT               ,"AWB_GAIN_PK_RED_GAIN0"                    ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_RED_GAIN1          ,OV2715_AWB_GAIN_PK_RED_GAIN1_DEFAULT               ,"AWB_GAIN_PK_RED_GAIN1"                    ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_GREEN_GAIN0        ,OV2715_AWB_GAIN_PK_GREEN_GAIN0_DEFAULT             ,"AWB_GAIN_PK_GREEN_GAIN0"                  ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_GREEN_GAIN1        ,OV2715_AWB_GAIN_PK_GREEN_GAIN1_DEFAULT             ,"AWB_GAIN_PK_GREEN_GAIN1"                  ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_BLUE_GAIN0         ,OV2715_AWB_GAIN_PK_BLUE_GAIN0_DEFAULT              ,"AWB_GAIN_PK_BLUE_GAIN0"                   ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_BLUE_GAIN1         ,OV2715_AWB_GAIN_PK_BLUE_GAIN1_DEFAULT              ,"AWB_GAIN_PK_BLUE_GAIN1"                   ,eReadWrite    },
    { OV2715_AWB_GAIN_PK_AWB_MAN_CTRL       ,OV2715_AWB_GAIN_PK_AWB_MAN_CTRL_DEFAULT            ,"AWB_GAIN_PK_AWB_MAN_CTRL"                 ,eReadWrite    },
    { OV2715_AWB_CONTROL_00                 ,OV2715_AWB_CONTROL_00_DEFAULT                      ,"AWB_CONTROL_00"                           ,eReadWrite    },
    { OV2715_AWB_CONTROL_01                 ,OV2715_AWB_CONTROL_01_DEFAULT                      ,"AWB_CONTROL_01"                           ,eReadWrite    },
    { OV2715_AWB_CONTROL_02                 ,OV2715_AWB_CONTROL_02_DEFAULT                      ,"AWB_CONTROL_02"                           ,eReadWrite    },
    { OV2715_STABLE_RANGE_WIDE              ,OV2715_STABLE_RANGE_WIDE_DEFAULT                   ,"STABLE_RANGE_WIDE"                        ,eReadWrite    },
    { OV2715_RED_GAIN_LIMIT                 ,OV2715_RED_GAIN_LIMIT_DEFAULT                      ,"RED_GAIN_LIMIT"                           ,eReadWrite    },
    { OV2715_GREEN_GAIN_LIMIT               ,OV2715_GREEN_GAIN_LIMIT_DEFAULT                    ,"GREEN_GAIN_LIMIT"                         ,eReadWrite    },
    { OV2715_BLUE_GAIN_LIMIT                ,OV2715_BLUE_GAIN_LIMIT_DEFAULT                     ,"BLUE_GAIN_LIMIT"                          ,eReadWrite    },
    { OV2715_AWB_FRAME_COUNTER              ,OV2715_AWB_FRAME_COUNTER_DEFAULT                   ,"AWB_FRAME_COUNTER"                        ,eReadWrite    },
    { OV2715_AVG_START_POSITION_AT_HORIZONTAL0 ,OV2715_AVG_START_POSITION_AT_HORIZONTAL0_DEFAULT,"AVG_START_POSITION_AT_HORIZONTAL0"        ,eReadWrite    },
    { OV2715_AVG_START_POSITION_AT_HORIZONTAL1 ,OV2715_AVG_START_POSITION_AT_HORIZONTAL1_DEFAULT,"AVG_START_POSITION_AT_HORIZONTAL1"        ,eReadWrite    },
    { OV2715_AVG_END_POSITION_AT_HORIZONTAL0   ,OV2715_AVG_END_POSITION_AT_HORIZONTAL0_DEFAULT  ,"AVG_END_POSITION_AT_HORIZONTAL0"          ,eReadWrite    },
    { OV2715_AVG_END_POSITION_AT_HORIZONTAL1   ,OV2715_AVG_END_POSITION_AT_HORIZONTAL1_DEFAULT  ,"AVG_END_POSITION_AT_HORIZONTAL1"          ,eReadWrite    },
    { OV2715_AVG_START_POSITION_AT_VERTICAL0   ,OV2715_AVG_START_POSITION_AT_VERTICAL0_DEFAULT  ,"AVG_START_POSITION_AT_VERTICAL0"          ,eReadWrite    },
    { OV2715_AVG_START_POSITION_AT_VERTICAL1   ,OV2715_AVG_START_POSITION_AT_VERTICAL1_DEFAULT  ,"AVG_START_POSITION_AT_VERTICAL1"          ,eReadWrite    },
    { OV2715_AVG_END_POSITION_AT_VERTICAL0  ,OV2715_AVG_END_POSITION_AT_VERTICAL0_DEFAULT       ,"AVG_END_POSITION_AT_VERTICAL0"            ,eReadWrite    },
    { OV2715_AVG_END_POSITION_AT_VERTICAL1  ,OV2715_AVG_END_POSITION_AT_VERTICAL1_DEFAULT       ,"AVG_END_POSITION_AT_VERTICAL1"            ,eReadWrite    },
    { OV2715_DPC_CTRL00                     ,OV2715_DPC_CTRL00_DEFAULT                          ,"DPC_CTRL00"                               ,eReadWrite    },
    { OV2715_WHITE_THRESHOLD_LIST0          ,OV2715_WHITE_THRESHOLD_LIST0_DEFAULT               ,"WHITE_THRESHOLD_LIST0"                    ,eReadWrite    },
    { OV2715_WHITE_THRESHOLD_LIST1          ,OV2715_WHITE_THRESHOLD_LIST1_DEFAULT               ,"WHITE_THRESHOLD_LIST1"                    ,eReadWrite    },
    { OV2715_WHITE_THRESHOLD_LIST2          ,OV2715_WHITE_THRESHOLD_LIST2_DEFAULT               ,"WHITE_THRESHOLD_LIST2"                    ,eReadWrite    },
    { OV2715_WHITE_THRESHOLD_LIST3          ,OV2715_WHITE_THRESHOLD_LIST3_DEFAULT               ,"WHITE_THRESHOLD_LIST3"                    ,eReadWrite    },
    { OV2715_BLACK_THRESHOLD_LIST0          ,OV2715_BLACK_THRESHOLD_LIST0_DEFAULT               ,"BLACK_THRESHOLD_LIST0"                    ,eReadWrite    },
    { OV2715_BLACK_THRESHOLD_LIST1          ,OV2715_BLACK_THRESHOLD_LIST1_DEFAULT               ,"BLACK_THRESHOLD_LIST1"                    ,eReadWrite    },
    { OV2715_BLACK_THRESHOLD_LIST2          ,OV2715_BLACK_THRESHOLD_LIST2_DEFAULT               ,"BLACK_THRESHOLD_LIST2"                    ,eReadWrite    },
    { OV2715_BLACK_THRESHOLD_LIST3          ,OV2715_BLACK_THRESHOLD_LIST3_DEFAULT               ,"BLACK_THRESHOLD_LIST3"                    ,eReadWrite    },
    { OV2715_GAIN_LIST1                     ,OV2715_GAIN_LIST1_DEFAULT                          ,"GAIN_LIST1"                               ,eReadWrite    },
    { OV2715_GAIN_LIST2                     ,OV2715_GAIN_LIST2_DEFAULT                          ,"GAIN_LIST2"                               ,eReadWrite    },
    { OV2715_DPC_CTRL01                     ,OV2715_DPC_CTRL01_DEFAULT                          ,"DPC_CTRL01"                               ,eReadWrite    },
    { OV2715_DPC_SATURATE                   ,OV2715_DPC_SATURATE_DEFAULT                        ,"DPC_SATURATE"                             ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST0A       ,OV2715_PATTERN_THRESHOLD_LIST0A_DEFAULT            ,"PATTERN_THRESHOLD_LIST0"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST0B       ,OV2715_PATTERN_THRESHOLD_LIST0B_DEFAULT            ,"PATTERN_THRESHOLD_LIST0"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST1A       ,OV2715_PATTERN_THRESHOLD_LIST1A_DEFAULT            ,"PATTERN_THRESHOLD_LIST1"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST1B       ,OV2715_PATTERN_THRESHOLD_LIST1B_DEFAULT            ,"PATTERN_THRESHOLD_LIST1"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST2A       ,OV2715_PATTERN_THRESHOLD_LIST2A_DEFAULT            ,"PATTERN_THRESHOLD_LIST2"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST2B       ,OV2715_PATTERN_THRESHOLD_LIST2B_DEFAULT            ,"PATTERN_THRESHOLD_LIST2"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST3A       ,OV2715_PATTERN_THRESHOLD_LIST3A_DEFAULT            ,"PATTERN_THRESHOLD_LIST3"                  ,eReadWrite    },
    { OV2715_PATTERN_THRESHOLD_LIST3B       ,OV2715_PATTERN_THRESHOLD_LIST3B_DEFAULT            ,"PATTERN_THRESHOLD_LIST3"                  ,eReadWrite    },
    { OV2715_LENC_RED_X0A                   ,OV2715_LENC_RED_X0A_DEFAULT                        ,"LENC_RED_X0A"                             ,eReadWrite    },
    { OV2715_LENC_RED_X0B                   ,OV2715_LENC_RED_X0B_DEFAULT                        ,"LENC_RED_X0B"                             ,eReadWrite    },
    { OV2715_LENC_RED_Y0A                   ,OV2715_LENC_RED_Y0A_DEFAULT                        ,"LENC_RED_Y0A"                             ,eReadWrite    },
    { OV2715_LENC_RED_Y0B                   ,OV2715_LENC_RED_Y0B_DEFAULT                        ,"LENC_RED_Y0B"                             ,eReadWrite    },
    { OV2715_LENC_RED_A1                    ,OV2715_LENC_RED_A1_DEFAULT                         ,"LENC_RED_A1"                              ,eReadWrite    },
    { OV2715_LENC_RED_A2                    ,OV2715_LENC_RED_A2_DEFAULT                         ,"LENC_RED_A2"                              ,eReadWrite    },
    { OV2715_LENC_RED_B1                    ,OV2715_LENC_RED_B1_DEFAULT                         ,"LENC_RED_B1"                              ,eReadWrite    },
    { OV2715_LENC_RED_B2                    ,OV2715_LENC_RED_B2_DEFAULT                         ,"LENC_RED_B2"                              ,eReadWrite    },
    { OV2715_LENC_GRN_X0A                   ,OV2715_LENC_GRN_X0A_DEFAULT                        ,"LENC_GRN_X0A"                             ,eReadWrite    },
    { OV2715_LENC_GRN_X0B                   ,OV2715_LENC_GRN_X0B_DEFAULT                        ,"LENC_GRN_X0B"                             ,eReadWrite    },
    { OV2715_LENC_GRN_Y0A                   ,OV2715_LENC_GRN_Y0A_DEFAULT                        ,"LENC_GRN_Y0A"                             ,eReadWrite    },
    { OV2715_LENC_GRN_Y0B                   ,OV2715_LENC_GRN_Y0B_DEFAULT                        ,"LENC_GRN_Y0B"                             ,eReadWrite    },
    { OV2715_LENC_GRN_A1                    ,OV2715_LENC_GRN_A1_DEFAULT                         ,"LENC_GRN_A1"                              ,eReadWrite    },
    { OV2715_LENC_GRN_A2                    ,OV2715_LENC_GRN_A2_DEFAULT                         ,"LENC_GRN_A2"                              ,eReadWrite    },
    { OV2715_LENC_GRN_B1                    ,OV2715_LENC_GRN_B1_DEFAULT                         ,"LENC_GRN_B1"                              ,eReadWrite    },
    { OV2715_LENC_GRN_B2                    ,OV2715_LENC_GRN_B2_DEFAULT                         ,"LENC_GRN_B2"                              ,eReadWrite    },
    { OV2715_LENC_BLU_X0A                   ,OV2715_LENC_BLU_X0A_DEFAULT                        ,"LENC_BLU_X0A"                             ,eReadWrite    },
    { OV2715_LENC_BLU_X0B                   ,OV2715_LENC_BLU_X0B_DEFAULT                        ,"LENC_BLU_X0B"                             ,eReadWrite    },
    { OV2715_LENC_BLU_Y0A                   ,OV2715_LENC_BLU_Y0A_DEFAULT                        ,"LENC_BLU_Y0A"                             ,eReadWrite    },
    { OV2715_LENC_BLU_Y0B                   ,OV2715_LENC_BLU_Y0B_DEFAULT                        ,"LENC_BLU_Y0B"                             ,eReadWrite    },
    { OV2715_LENC_BLU_A1                    ,OV2715_LENC_BLU_A1_DEFAULT                         ,"LENC_BLU_A1"                              ,eReadWrite    },
    { OV2715_LENC_BLU_A2                    ,OV2715_LENC_BLU_A2_DEFAULT                         ,"LENC_BLU_A2"                              ,eReadWrite    },
    { OV2715_LENC_BLU_B1                    ,OV2715_LENC_BLU_B1_DEFAULT                         ,"LENC_BLU_B1"                              ,eReadWrite    },
    { OV2715_LENC_BLU_B2                    ,OV2715_LENC_BLU_B2_DEFAULT                         ,"LENC_BLU_B2"                              ,eReadWrite    },
    { OV2715_LENC_CTRL00                    ,OV2715_LENC_CTRL00_DEFAULT                         ,"LENC_CTRL00"                              ,eReadWrite    },
    { OV2715_LENC_COEF_TH                   ,OV2715_LENC_COEF_TH_DEFAULT                        ,"LENC_COEF_TH"                             ,eReadWrite    },
    { OV2715_LENC_GAIN_THRE1                ,OV2715_LENC_GAIN_THRE1_DEFAULT                     ,"LENC_GAIN_THRE1"                          ,eReadWrite    },
    { OV2715_LENC_GAIN_THRE2                ,OV2715_LENC_GAIN_THRE2_DEFAULT                     ,"LENC_GAIN_THRE2"                          ,eReadWrite    },
    { OV2715_LENC_COEF_MAN                  ,OV2715_LENC_COEF_MAN_DEFAULT                       ,"LENC_COEF_MAN"                            ,eReadWrite    },
    { OV2715_AFC_CTRL00                     ,OV2715_AFC_CTRL00_DEFAULT                          ,"AFC_CTRL00"                               ,eReadWrite    },
    { OV2715_AFC_CTRL01                     ,OV2715_AFC_CTRL01_DEFAULT                          ,"AFC_CTRL01"                               ,eReadWrite    },
    { OV2715_AFC_CTRL02                     ,OV2715_AFC_CTRL02_DEFAULT                          ,"AFC_CTRL02"                               ,eReadWrite    },
    { OV2715_AFC_CTRL03                     ,OV2715_AFC_CTRL03_DEFAULT                          ,"AFC_CTRL03"                               ,eReadWrite    },
    { OV2715_AFC_CTRL04                     ,OV2715_AFC_CTRL04_DEFAULT                          ,"AFC_CTRL04"                               ,eReadWrite    },
    { OV2715_AFC_CTRL05                     ,OV2715_AFC_CTRL05_DEFAULT                          ,"AFC_CTRL05"                               ,eReadWrite    },
    { OV2715_AFC_CTRL06                     ,OV2715_AFC_CTRL06_DEFAULT                          ,"AFC_CTRL06"                               ,eReadWrite    },
    { OV2715_AFC_CTRL07                     ,OV2715_AFC_CTRL07_DEFAULT                          ,"AFC_CTRL07"                               ,eReadWrite    },
    { OV2715_AFC_CTRL08                     ,OV2715_AFC_CTRL08_DEFAULT                          ,"AFC_CTRL08"                               ,eReadWrite    },
    { OV2715_AFC_CTRL09                     ,OV2715_AFC_CTRL09_DEFAULT                          ,"AFC_CTRL09"                               ,eReadWrite    },
    { OV2715_AFC_CTRL10                     ,OV2715_AFC_CTRL10_DEFAULT                          ,"AFC_CTRL10"                               ,eReadWrite    },
    { OV2715_AFC_CTRL11                     ,OV2715_AFC_CTRL11_DEFAULT                          ,"AFC_CTRL11"                               ,eReadWrite    },
    { OV2715_AFC_CTRL12                     ,OV2715_AFC_CTRL12_DEFAULT                          ,"AFC_CTRL12"                               ,eReadWrite    },
    { OV2715_AFC_CTRL13                     ,OV2715_AFC_CTRL13_DEFAULT                          ,"AFC_CTRL13"                               ,eReadWrite    },
    { OV2715_AFC_CTRL14                     ,OV2715_AFC_CTRL14_DEFAULT                          ,"AFC_CTRL14"                               ,eReadWrite    },
    { OV2715_AFC_CTRL15                     ,OV2715_AFC_CTRL15_DEFAULT                          ,"AFC_CTRL15"                               ,eReadWrite    },
    { OV2715_AFC_CTRL16                     ,OV2715_AFC_CTRL16_DEFAULT                          ,"AFC_CTRL16"                               ,eReadWrite    },
    { OV2715_AFC_CTRL17                     ,OV2715_AFC_CTRL17_DEFAULT                          ,"AFC_CTRL17"                               ,eReadWrite    },
    { OV2715_AFC_CTRL18                     ,OV2715_AFC_CTRL18_DEFAULT                          ,"AFC_CTRL18"                               ,eReadWrite    },
    { OV2715_AFC_CTRL19                     ,OV2715_AFC_CTRL19_DEFAULT                          ,"AFC_CTRL19"                               ,eReadWrite    },
    { OV2715_AFC_CTRL20                     ,OV2715_AFC_CTRL20_DEFAULT                          ,"AFC_CTRL20"                               ,eReadWrite    },
    { OV2715_302D                           ,OV2715_302D_DEFAULT                                ,"OV2715_302D"                              ,eReadWrite    },
    { OV2715_3600                           ,OV2715_3600_DEFAULT                                ,"OV2715_3600"                              ,eReadWrite    },
    { OV2715_3601                           ,OV2715_3601_DEFAULT                                ,"OV2715_3601"                              ,eReadWrite    },
    { OV2715_3602                           ,OV2715_3602_DEFAULT                                ,"OV2715_3602"                              ,eReadWrite    },
    { OV2715_3603                           ,OV2715_3603_DEFAULT                                ,"OV2715_3603"                              ,eReadWrite    },
    { OV2715_3604                           ,OV2715_3604_DEFAULT                                ,"OV2715_3604"                              ,eReadWrite    },
    { OV2715_3605                           ,OV2715_3605_DEFAULT                                ,"OV2715_3605"                              ,eReadWrite    },
    { OV2715_3606                           ,OV2715_3606_DEFAULT                                ,"OV2715_3606"                              ,eReadWrite    },
    { OV2715_3620                           ,OV2715_3620_DEFAULT                                ,"OV2715_3620"                              ,eReadWrite    },
    { OV2715_3623                           ,OV2715_3623_DEFAULT                                ,"OV2715_3623"                              ,eReadWrite    },
    { OV2715_3630                           ,OV2715_3630_DEFAULT                                ,"OV2715_3630"                              ,eReadWrite    },
    { OV2715_3631                           ,OV2715_3631_DEFAULT                                ,"OV2715_3631"                              ,eReadWrite    },
    { OV2715_3A1A                           ,OV2715_3A1A_DEFAULT                                ,"OV2715_3A1A"                              ,eReadWrite    },
    { OV2715_3702                           ,OV2715_3702_DEFAULT                                ,"OV2715_3702"                              ,eReadWrite    },
    { OV2715_3703                           ,OV2715_3703_DEFAULT                                ,"OV2715_3703"                              ,eReadWrite    },
    { OV2715_3704                           ,OV2715_3704_DEFAULT                                ,"OV2715_3704"                              ,eReadWrite    },
    { OV2715_3706                           ,OV2715_3706_DEFAULT                                ,"OV2715_3706"                              ,eReadWrite    },
    { OV2715_370B                           ,OV2715_370B_DEFAULT                                ,"OV2715_370B"                              ,eReadWrite    },
    { OV2715_3710                           ,OV2715_3710_DEFAULT                                ,"OV2715_3710"                              ,eReadWrite    },
    { OV2715_3712                           ,OV2715_3712_DEFAULT                                ,"OV2715_3712"                              ,eReadWrite    },
    { OV2715_3713                           ,OV2715_3713_DEFAULT                                ,"OV2715_3713"                              ,eReadWrite    },
    { OV2715_3714                           ,OV2715_3714_DEFAULT                                ,"OV2715_3714"                              ,eReadWrite    },
    { OV2715_3811                           ,OV2715_3811_DEFAULT                                ,"OV2715_3811"                              ,eReadWrite    },
    { OV2715_381C                           ,OV2715_381C_DEFAULT                                ,"OV2715_381C"                              ,eReadWrite    },
    { OV2715_381D                           ,OV2715_381D_DEFAULT                                ,"OV2715_381D"                              ,eReadWrite    },
    { OV2715_381E                           ,OV2715_381E_DEFAULT                                ,"OV2715_381E"                              ,eReadWrite    },
    { OV2715_381F                           ,OV2715_381F_DEFAULT                                ,"OV2715_381F"                              ,eReadWrite    },
    { OV2715_3820                           ,OV2715_3820_DEFAULT                                ,"OV2715_3820"                              ,eReadWrite    },
    { OV2715_3821                           ,OV2715_3821_DEFAULT                                ,"OV2715_3821"                              ,eReadWrite    },
    { OV2715_401C                           ,OV2715_401C_DEFAULT                                ,"OV2715_401C"                              ,eReadWrite    },
    { OV2715_4301                           ,OV2715_4301_DEFAULT                                ,"OV2715_4301"                              ,eReadWrite    },
    { OV2715_4303                           ,OV2715_4303_DEFAULT                                ,"OV2715_4303"                              ,eReadWrite    },
    { OV2715_5688                           ,OV2715_5688_DEFAULT                                ,"OV2715_5688"                              ,eReadWrite    },
    { 0x0000                                ,0x00                                               ,"TableEnd"                                 ,eTableEnd     }
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
const Isi1x1FloatMatrix_t OV2715_KFactor =
{
    { 6.838349f }   // or 3.94f (to be checked)
};


// PCA matrix
const Isi3x2FloatMatrix_t OV2715_PCAMatrix =
{
    {
        -0.62791f, -0.13803f,  0.76595f,
        -0.52191f,  0.80474f, -0.28283f
    }
};


// mean values from SVD
const Isi3x1FloatMatrix_t OV2715_SVDMeanValue =
{
    {
        0.34165f,  0.37876f,  0.27959f
    }
};



/*****************************************************************************
 * Rg/Bg color space (clipping and out of range)
 *****************************************************************************/
// Center line of polygons {f_N0_Rg, f_N0_Bg, f_d}
const IsiLine_t OV2715_CenterLine =
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
const IsiAwbClipParm_t OV2715_AwbClipParm =
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
const IsiAwbGlobalFadeParm_t OV2715_AwbGlobalFadeParm =
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
const IsiAwbFade2Parm_t OV2715_AwbFade2Parm =
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


