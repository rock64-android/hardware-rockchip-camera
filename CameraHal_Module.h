#ifndef ANDROID_HARDWARE_CAMERA_HARDWARE_MODULE_H
#define ANDROID_HARDWARE_CAMERA_HARDWARE_MODULE_H
#include <linux/videodev2.h>

#define CONFIG_AUTO_DETECT_FRAMERATE    0

#define CAMERAS_SUPPORT_MAX             2
#define CAMERAS_SUPPORTED_SIMUL_MAX     1

typedef struct rk_cam_info_s {
    char device_path[30];
    char driver[16];
    unsigned int version;
    struct camera_info facing_info;
    struct v4l2_frmivalenum fival_list[10];   // default preview framerate, dc preview framerate, dv preview framerate(highe quality/low quality)   
}rk_cam_info_t;


typedef struct rk_camera_device {
    camera_device_t base;   
    int cameraid;
} rk_camera_device_t;
#endif