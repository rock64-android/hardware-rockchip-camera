/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
* @file CameraHal.cpp
*
* This file maps the Camera Hardware Interface to V4L2.
*
*/

#define LOG_TAG "CameraHal_Module"

#include <utils/threads.h>

#include "CameraHal.h"
#include "CameraHal_Module.h"



rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];
static android::CameraHal* gCameraHals[CAMERAS_SUPPORT_MAX];
//static sp<CameraFpsDetectThread> gCameraFpsDetectThread;
static unsigned int gCamerasOpen = 0;
static signed int gCamerasNumber = -1;
static android::Mutex gCameraHalDeviceLock;

static int camera_device_open(const hw_module_t* module, const char* name,
                hw_device_t** device);
static int camera_device_close(hw_device_t* device);
static int camera_get_number_of_cameras(void);
static int camera_get_camera_info(int camera_id, struct camera_info *info);

static struct hw_module_methods_t camera_module_methods = {
        open: camera_device_open
};

camera_module_t HAL_MODULE_INFO_SYM = {
    common: {
         tag: HARDWARE_MODULE_TAG,
         version_major: ((CONFIG_CAMERAHAL_VERSION&0xff00)>>8),
         version_minor: CONFIG_CAMERAHAL_VERSION&0xff,
         id: CAMERA_HARDWARE_MODULE_ID,
         name: CAMERA_MODULE_NAME,
         author: "RockChip",
         methods: &camera_module_methods,
         dso: NULL, /* remove compilation warnings */
         reserved: {0}, /* remove compilation warnings */
    },
    get_number_of_cameras: camera_get_number_of_cameras,
    get_camera_info: camera_get_camera_info,
};




/*******************************************************************
 * implementation of camera_device_ops functions
 *******************************************************************/

int camera_set_preview_window(struct camera_device * device,
        struct preview_stream_ops *window)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->setPreviewWindow(window);

    return rv;
}

void camera_set_callbacks(struct camera_device * device,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->setCallbacks(notify_cb, data_cb, data_cb_timestamp, get_memory, user);
}

void camera_enable_msg_type(struct camera_device * device, int32_t msg_type)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->enableMsgType(msg_type);
}

void camera_disable_msg_type(struct camera_device * device, int32_t msg_type)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->disableMsgType(msg_type);
}

int camera_msg_type_enabled(struct camera_device * device, int32_t msg_type)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return 0;

    rk_dev = (rk_camera_device_t*) device;

    return gCameraHals[rk_dev->cameraid]->msgTypeEnabled(msg_type);
}

int camera_start_preview(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->startPreview();

    return rv;
}

void camera_stop_preview(struct camera_device * device)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->stopPreview();
}

int camera_preview_enabled(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->previewEnabled();
    return rv;
}

int camera_store_meta_data_in_buffers(struct camera_device * device, int enable)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    //  TODO: meta data buffer not current supported
    rv = gCameraHals[rk_dev->cameraid]->storeMetaDataInBuffers(enable);
    return rv;
    //return enable ? android::INVALID_OPERATION: android::OK;
}

int camera_start_recording(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->startRecording();
    return rv;
}

void camera_stop_recording(struct camera_device * device)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->stopRecording();
}

int camera_recording_enabled(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->recordingEnabled();
    return rv;
}

void camera_release_recording_frame(struct camera_device * device,
                const void *opaque)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->releaseRecordingFrame(opaque);
}

int camera_auto_focus(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->autoFocus();
    return rv;
}

int camera_cancel_auto_focus(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->cancelAutoFocus();
    return rv;
}

int camera_take_picture(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->takePicture();
    return rv;
}

int camera_cancel_picture(struct camera_device * device)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->cancelPicture();
    return rv;
}

int camera_set_parameters(struct camera_device * device, const char *params)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->setParameters(params);
    return rv;
}

char* camera_get_parameters(struct camera_device * device)
{
    char* param = NULL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return NULL;

    rk_dev = (rk_camera_device_t*) device;

    param = gCameraHals[rk_dev->cameraid]->getParameters();

    return param;
}

static void camera_put_parameters(struct camera_device *device, char *parms)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->putParameters(parms);
}

int camera_send_command(struct camera_device * device,
            int32_t cmd, int32_t arg1, int32_t arg2)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->sendCommand(cmd, arg1, arg2);
    return rv;
}

void camera_release(struct camera_device * device)
{
    rk_camera_device_t* rk_dev = NULL;

    LOGV("%s", __FUNCTION__);

    if(!device)
        return;

    rk_dev = (rk_camera_device_t*) device;

    gCameraHals[rk_dev->cameraid]->release();
}

int camera_dump(struct camera_device * device, int fd)
{
    int rv = -EINVAL;
    rk_camera_device_t* rk_dev = NULL;

    if(!device)
        return rv;

    rk_dev = (rk_camera_device_t*) device;

    rv = gCameraHals[rk_dev->cameraid]->dump(fd);
    return rv;
}
int camera_device_close(hw_device_t* device)
{
    int ret = 0;
    rk_camera_device_t* rk_dev = NULL;

    LOGD("%s", __FUNCTION__);

    android::Mutex::Autolock lock(gCameraHalDeviceLock);
    LOGD("%s: device:0x%x", __FUNCTION__,device);
    if (!device) {
        ret = -EINVAL;
        goto done;
    }

    rk_dev = (rk_camera_device_t*) device;

    if (rk_dev) {
        if (gCameraHals[rk_dev->cameraid]) {
            delete gCameraHals[rk_dev->cameraid];
            gCameraHals[rk_dev->cameraid] = NULL;
            gCamerasOpen--;
        }

        if (rk_dev->base.ops) {
            free(rk_dev->base.ops);
        }
        free(rk_dev);
    }
done:

    return ret;
}

/*******************************************************************
 * implementation of camera_module functions
 *******************************************************************/

/* open device handle to one of the cameras
 *
 * assume camera service will keep singleton of each camera
 * so this function will always only be called once per camera instance
 */

int camera_device_open(const hw_module_t* module, const char* name,
                hw_device_t** device)
{
    int rv = 0;
    int cameraid;
    rk_camera_device_t* camera_device = NULL;
    camera_device_ops_t* camera_ops = NULL;
    android::CameraHal* camera = NULL;

    android::Mutex::Autolock lock(gCameraHalDeviceLock);

    LOGI("camera_device open");

    if (name != NULL) {
        cameraid = atoi(name);

        if(cameraid > gCamerasNumber) {
            LOGE("camera service provided cameraid out of bounds, "
                    "cameraid = %d, num supported = %d",
                    cameraid, gCamerasNumber);
            rv = -EINVAL;
            goto fail;
        }

        if(gCamerasOpen >= CAMERAS_SUPPORTED_SIMUL_MAX) {
            LOGE("maximum number(%d) of cameras already open",gCamerasOpen);
            rv = -ENOMEM;
            goto fail;
        }

        camera_device = (rk_camera_device_t*)malloc(sizeof(*camera_device));
        if(!camera_device) {
            LOGE("camera_device allocation fail");
            rv = -ENOMEM;
            goto fail;
        }

        camera_ops = (camera_device_ops_t*)malloc(sizeof(*camera_ops));
        if(!camera_ops) {
            LOGE("camera_ops allocation fail");
            rv = -ENOMEM;
            goto fail;
        }

        memset(camera_device, 0, sizeof(*camera_device));
        memset(camera_ops, 0, sizeof(*camera_ops));

        camera_device->base.common.tag = HARDWARE_DEVICE_TAG;
        camera_device->base.common.version = 0;
        camera_device->base.common.module = (hw_module_t *)(module);
        camera_device->base.common.close = camera_device_close;
        camera_device->base.ops = camera_ops;

        camera_ops->set_preview_window = camera_set_preview_window;
        camera_ops->set_callbacks = camera_set_callbacks;
        camera_ops->enable_msg_type = camera_enable_msg_type;
        camera_ops->disable_msg_type = camera_disable_msg_type;
        camera_ops->msg_type_enabled = camera_msg_type_enabled;
        camera_ops->start_preview = camera_start_preview;
        camera_ops->stop_preview = camera_stop_preview;
        camera_ops->preview_enabled = camera_preview_enabled;
        camera_ops->store_meta_data_in_buffers = camera_store_meta_data_in_buffers;
        camera_ops->start_recording = camera_start_recording;
        camera_ops->stop_recording = camera_stop_recording;
        camera_ops->recording_enabled = camera_recording_enabled;
        camera_ops->release_recording_frame = camera_release_recording_frame;
        camera_ops->auto_focus = camera_auto_focus;
        camera_ops->cancel_auto_focus = camera_cancel_auto_focus;
        camera_ops->take_picture = camera_take_picture;
        camera_ops->cancel_picture = camera_cancel_picture;
        camera_ops->set_parameters = camera_set_parameters;
        camera_ops->get_parameters = camera_get_parameters;
        camera_ops->put_parameters = camera_put_parameters;
        camera_ops->send_command = camera_send_command;
        camera_ops->release = camera_release;
        camera_ops->dump = camera_dump;

        *device = &camera_device->base.common;

        // -------- RockChip specific stuff --------

        camera_device->cameraid = cameraid;
        
        camera = new android::CameraHal(cameraid);

        if(!camera) {
            LOGE("Couldn't create instance of CameraHal class");
            rv = -ENOMEM;
            goto fail;
        }

        gCameraHals[cameraid] = camera;
        gCamerasOpen++;
    }

    return rv;

fail:
    if(camera_device) {
        free(camera_device);
        camera_device = NULL;
    }
    if(camera_ops) {
        free(camera_ops);
        camera_ops = NULL;
    }
    if(camera) {
        delete camera;
        camera = NULL;
    }
    *device = NULL;
    return rv;
}

int camera_get_number_of_cameras(void)
{
    char cam_path[20];
    char cam_num[3],i;
    int cam_cnt=0,fd=-1,rk29_cam[CAMERAS_SUPPORT_MAX];
    struct v4l2_capability capability;
    rk_cam_info_t camInfoTmp[CAMERAS_SUPPORT_MAX];
    char *ptr,**ptrr;

    if (gCamerasNumber > 0)
        goto camera_get_number_of_cameras_end;

    memset(&camInfoTmp[0],0x00,sizeof(rk_cam_info_t));
    memset(&camInfoTmp[1],0x00,sizeof(rk_cam_info_t));
    
    for (i=0; i<10; i++) {
        cam_path[0] = 0x00;
        strcat(cam_path, CAMERA_DEVICE_NAME);
        sprintf(cam_num, "%d", i);
        strcat(cam_path,cam_num);
        fd = open(cam_path, O_RDONLY);
        if (fd < 0)
            break;

        memset(&capability, 0, sizeof(struct v4l2_capability));
        if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
        	LOGE("Video device(%s): query capability not supported.\n",cam_path);
            goto loop_continue;
        }
        
        if ((capability.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING)) != (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING)) {
    	    LOGD("Video device(%s): video capture not supported.\n",cam_path);
        } else {
            memset(camInfoTmp[cam_cnt&0x01].device_path,0x00, sizeof(camInfoTmp[cam_cnt&0x01].device_path));
            strcat(camInfoTmp[cam_cnt&0x01].device_path,cam_path);
            memset(camInfoTmp[cam_cnt&0x01].fival_list,0x00, sizeof(camInfoTmp[cam_cnt&0x01].fival_list));
            memcpy(camInfoTmp[cam_cnt&0x01].driver,capability.driver, sizeof(camInfoTmp[cam_cnt&0x01].driver));
            camInfoTmp[cam_cnt&0x01].version = capability.version;
            if (strstr((char*)&capability.card[0], "front") != NULL) {
                camInfoTmp[cam_cnt&0x01].facing_info.facing = CAMERA_FACING_FRONT;
            } else {
                camInfoTmp[cam_cnt&0x01].facing_info.facing = CAMERA_FACING_BACK;
            }  
            ptr = strstr((char*)&capability.card[0],"-");
            if (ptr != NULL) {
                ptr++;
                camInfoTmp[cam_cnt&0x01].facing_info.orientation = atoi(ptr);
            } else {
                camInfoTmp[cam_cnt&0x01].facing_info.orientation = 0;
            }
            cam_cnt++;
            if (cam_cnt >= CAMERAS_SUPPORT_MAX)
                i = 10;
        }
loop_continue:
        if (fd > 0) {
            close(fd);
            fd = -1;
        }
        continue;    
    }

    gCamerasNumber = cam_cnt;

#if CONFIG_AUTO_DETECT_FRAMERATE
    rk29_cam[0] = 0xff;
    rk29_cam[1] = 0xff;
    for (i=0; i<cam_cnt; i++) {
        if (strcmp((char*)&camInfoTmp[i].driver[0],"rk29xx-camera") == 0) {
            if (strcmp((char*)&camInfoTmp[i].driver[0],(char*)&gCamInfos[i].driver[0]) != 0) {
                if (camInfoTmp[i].version >= KERNEL_VERSION(0, 0,3)) {
                    rk29_cam[i] = i; 
                } else { 
                    rk29_cam[i] = 0xff;
                    LOGD("%s catch driver(%s) version is %d.%d.%d, this version is not support detect framerate",
                        camInfoTmp[i].device_path, camInfoTmp[i].driver,(camInfoTmp[i].version>>16)&0xff,
                        (camInfoTmp[i].version>>8)&0xff,camInfoTmp[i].version&0xff);
                }
            }
        } else {
            rk29_cam[i] = 0xff;
        }
    }

    if ((rk29_cam[0] != 0xff) || (rk29_cam[1] != 0xff)) {
        if (gCameraFpsDetectThread == NULL) {
            gCameraFpsDetectThread = new CameraFpsDetectThread();
            LOGD("%s create CameraFpsDetectThread for enum camera framerate!!",__FUNCTION__);
            gCameraFpsDetectThread->run("CameraFpsDetectThread", ANDROID_PRIORITY_AUDIO);
        }
    }
#endif

    memcpy(&gCamInfos[0], &camInfoTmp[0], sizeof(rk_cam_info_t));
    memcpy(&gCamInfos[1], &camInfoTmp[1], sizeof(rk_cam_info_t));
    
camera_get_number_of_cameras_end:
    LOGD("%s(%d): Current board have %d cameras attached.",__FUNCTION__, __LINE__, gCamerasNumber);
    return gCamerasNumber;
}

int camera_get_camera_info(int camera_id, struct camera_info *info)
{
    int rv = 0;
    int face_value = CAMERA_FACING_BACK;
    int orientation = 0;

    if(camera_id > gCamerasNumber) {
        LOGE("%s camera_id out of bounds, camera_id = %d, num supported = %d",__FUNCTION__,
                camera_id, gCamerasNumber);
        rv = -EINVAL;
        goto end;
    }

    info->facing = gCamInfos[camera_id].facing_info.facing;
    info->orientation = gCamInfos[camera_id].facing_info.orientation;    

    LOGD("%s(%d): camera_%d facing(%d), orientation(%d)",__FUNCTION__,__LINE__,camera_id,info->facing,info->orientation);
end:
    return rv;
}
#if 0
int enumFrameIntervals(struct v4l2_frmivalenum *fival, int count)
{
	int ret,i;

    i = 0;
    fival->index = 0;
    fival->pixel_format = mCamDriverPreviewFmt;
    fival->width = mPreviewWidth;
    fival->height = mPreviewHeight;
	while (count && (ret = ioctl(iCamFd, VIDIOC_ENUM_FRAMEINTERVALS, fival)) == 0) {
            	 
		if (fival->type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
		    ret = -1;
			break;
		} else if (fival->type == V4L2_FRMIVAL_TYPE_STEPWISE) {				
		    ret = -1;
            break;
		}
		fival++;     
        i++;
        count--;

        if (count) {
            fival->index = i;
            fival->pixel_format = mCamDriverPreviewFmt;
            fival->width = mPreviewWidth;
            fival->height = mPreviewHeight;
        }
	}

    if (fival->index && fival->width && (fival->discrete.denominator == 0)) 
        memset(fival,0x00, sizeof(struct v4l2_frmivalenum));
    
	return i;
}
#endif
int camera_famerate_detect_loop(void)
{
#if 0
    Camrea_Interface *camera;
    Vector<Size> preview_sizes;
    Size preview_size;
    struct CameraSize size_preview;
    CameraParameters parameters;
    struct v4l2_frmivalenum *fival;
    int i,j,w,h,count,ret;

    LOGD("%s Enter..",__func__);
#if CONFIG_AUTO_DETECT_FRAMERATE    
    for (i=0; i<2; i++) {        
        if ((strstr(camInfo[i].device_path, CAMERA_DEVICE_NAME) != NULL) && (strcmp((char*)&camInfo[i].driver[0],"rk29xx-camera") == 0)) {
            camera = openCameraHardwareEx(i);
            parameters = camera->getParam();  
            preview_sizes.clear();
            parameters.getSupportedPreviewSizes(preview_sizes);            
            fival = camInfo[i].fival_list;
            count = 10;            
            for (j=0; (j<preview_sizes.size()) && (count>0); j++) {
                preview_size = preview_sizes.itemAt(j);
                size_preview.w = preview_size.width;
                size_preview.h = preview_size.height;                                
                camera->setParam(CamCmd_SetPreviewSize,(void*)&size_preview);
                camera->startPreview();
                sleep(2);
                camera->stopPreview();
                ret = camera->enumFrameIntervals(fival,count);                
                if (ret > 0) {
                    count -= ret;
                    fival += ret;
                }                
            }

            fival = camInfo[i].fival_list;
            while (fival->width) {
                LOGD("Camera_%d %dx%d framerate is %d/%d", i,fival->width,fival->height,fival->discrete.denominator,fival->discrete.numerator);
                fival++;
            }
            
            delete camera;           
        }
    }
#endif
#endif
    LOGD("%s Exit..",__func__);
    return false;
}

