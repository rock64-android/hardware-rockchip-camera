#ifndef __CAMERIC_ISP_FLASH_DRV_API_H__
#define __CAMERIC_ISP_FLASH_DRV_API_H__


#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef enum CamerIcIspFlashMode_e
{
    CAMERIC_ISP_FLASH_OFF = 0x00,
    CAMERIC_ISP_FLASH_AUTO = 0x01,
    CAMERIC_ISP_FLASH_ON = 0x02,
    CAMERIC_ISP_FLASH_RED_EYE = 0x03,
    CAMERIC_ISP_FLASH_TORCH = 0x05
} CamerIcIspFlashMode_t;


typedef struct CamerIcIspFlashCfg_s 
{
    CamerIcIspFlashMode_t mode;   
    
} CamerIcIspFlashCfg_t;



extern RESULT CamerIcIspFlashConfigure
(
    CamerIcDrvHandle_t  handle,
    CamerIcIspFlashCfg_t *flash_cfg
);

extern RESULT CamerIcIspFlashStart
(
    CamerIcDrvHandle_t  handle,
    bool_t operate_now
);

extern RESULT CamerIcIspFlashStop
(
    CamerIcDrvHandle_t  handle,
    bool_t operate_now
);

#ifdef __cplusplus
}
#endif

#endif
