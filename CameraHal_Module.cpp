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
#include <binder/IPCThreadState.h>
#include "CameraHal.h"
#include "CameraHal_Module.h"
#include "CameraHal_MediaProfile.cpp"
#include <time.h>

rk_cam_info_t gCamInfos[CAMERAS_SUPPORT_MAX];
static android::CameraHal* gCameraHals[CAMERAS_SUPPORT_MAX];
#if CONFIG_AUTO_DETECT_FRAMERATE 
static sp<CameraFpsDetectThread> gCameraFpsDetectThread;
#endif
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
    char version[PROPERTY_VALUE_MAX];
    char property[PROPERTY_VALUE_MAX];
    int hwrotation = 0;

	//oyyf@rock-chips.com:  modify /data/media_profiles.xml
	struct timeval t0, t1;
    ::gettimeofday(&t0, NULL);
	media_profiles_xml_control();
	::gettimeofday(&t1, NULL);
	LOGD("meida_profiles_xml_control time (%ld)us\n", (t1.tv_sec*1000000 + t1.tv_usec) - (t0.tv_sec*1000000 + t0.tv_usec));

	
    if (gCamerasNumber > 0)
        goto camera_get_number_of_cameras_end;
    
    memset(version,0x00,sizeof(version));
    sprintf(version,"%d.%d.%d",((CONFIG_CAMERAHAL_VERSION&0xff0000)>>16),
        ((CONFIG_CAMERAHAL_VERSION&0xff00)>>8),CONFIG_CAMERAHAL_VERSION&0xff);
    property_set(CAMERAHAL_VERSION_PROPERTY_KEY,version);
    
    memset(&camInfoTmp[0],0x00,sizeof(rk_cam_info_t));
    memset(&camInfoTmp[1],0x00,sizeof(rk_cam_info_t));
    
    for (i=0; i<10; i++) {
        cam_path[0] = 0x00;
        strcat(cam_path, CAMERA_DEVICE_NAME);
        sprintf(cam_num, "%d", i);
        strcat(cam_path,cam_num);
        fd = open(cam_path, O_RDONLY);
        if (fd < 0) {
            LOGE("Open %s failed! strr: %s",cam_path,strerror(errno));
            break;
        }

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

            memset(version,0x00,sizeof(version));
            sprintf(version,"%d.%d.%d",((capability.version&0xff0000)>>16),
                ((capability.version&0xff00)>>8),capability.version&0xff);
            property_set(CAMERADRIVER_VERSION_PROPERTY_KEY,version);

            LOGD("%s(%d): %s:%s",__FUNCTION__,__LINE__,CAMERADRIVER_VERSION_PROPERTY_KEY,version);
            
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
    //zyc , change the camera infomation if there is a usb camera
    if((strcmp(camInfoTmp[0].driver,"uvcvideo") == 0)) {
        if (cam_cnt == 1) {
        	camInfoTmp[0].facing_info.facing = CAMERA_FACING_BACK;
        } else {
            camInfoTmp[0].facing_info.facing = (camInfoTmp[1].facing_info.facing == CAMERA_FACING_FRONT) ? CAMERA_FACING_BACK:CAMERA_FACING_FRONT;
        }
        camInfoTmp[0].facing_info.orientation = (camInfoTmp[0].facing_info.facing == CAMERA_FACING_FRONT)?270:90;
    } else if((strcmp(camInfoTmp[1].driver,"uvcvideo") == 0)) {
    	camInfoTmp[1].facing_info.facing = (camInfoTmp[0].facing_info.facing == CAMERA_FACING_FRONT) ? CAMERA_FACING_BACK:CAMERA_FACING_FRONT;
    	camInfoTmp[1].facing_info.orientation = (camInfoTmp[1].facing_info.facing == CAMERA_FACING_FRONT)?270:90;
    }
    gCamerasNumber = cam_cnt;

#if CONFIG_AUTO_DETECT_FRAMERATE
    rk29_cam[0] = 0xff;
    rk29_cam[1] = 0xff;
    for (i=0; i<cam_cnt; i++) {
        if (strcmp((char*)&camInfoTmp[i].driver[0],"rk29xx-camera") == 0) {
            if (strcmp((char*)&camInfoTmp[i].driver[0],(char*)&gCamInfos[i].driver[0]) != 0) {
                rk29_cam[i] = i; 
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
    #if CONFIG_CAMERA_SINGLE_SENSOR_FORCE_BACK_FOR_CTS
    if ((gCamerasNumber==1) && (camInfoTmp[0].facing_info.facing==CAMERA_FACING_FRONT)) {
        gCamerasNumber = 2;
        memcpy(&camInfoTmp[1],&camInfoTmp[0], sizeof(rk_cam_info_t));
        camInfoTmp[1].facing_info.facing = CAMERA_FACING_BACK;
    }
    #endif
    
    memcpy(&gCamInfos[0], &camInfoTmp[0], sizeof(rk_cam_info_t));
    memcpy(&gCamInfos[1], &camInfoTmp[1], sizeof(rk_cam_info_t));

#if 0
    property_get("ro.sf.hwrotation", property, "0");
    hwrotation = strtol(property,0,0);

    if (hwrotation == 0) {
        gCamInfos[0].facing_info.orientation = 0;    /* ddl@rock-chips.com: v0.4.17 */ 
        gCamInfos[1].facing_info.orientation = 0;
    }
#endif    
camera_get_number_of_cameras_end:
    LOGD("%s(%d): Current board have %d cameras attached.",__FUNCTION__, __LINE__, gCamerasNumber);
    return gCamerasNumber;
}
static int getCallingPid() {
    return IPCThreadState::self()->getCallingPid();
}
int camera_get_camera_info(int camera_id, struct camera_info *info)
{
    int rv = 0,fp;
    int face_value = CAMERA_FACING_BACK;
    int orientation = 0;
    char process_name[30];
        
    if(camera_id > gCamerasNumber) {
        LOGE("%s camera_id out of bounds, camera_id = %d, num supported = %d",__FUNCTION__,
                camera_id, gCamerasNumber);
        rv = -EINVAL;
        goto end;
    }
    
#if CONFIG_CAMERA_ORIENTATION_SKYPE
    process_name[0] = 0x00; 
    sprintf(process_name,"/proc/%d/cmdline",getCallingPid());
    fp = open(process_name, O_RDONLY);
    if (fp < 0) {
        memset(process_name,0x00,sizeof(process_name));
        LOGE("%s(%d): Obtain calling process info failed",__FUNCTION__,__LINE__);
    } else {
        memset(process_name,0x00,sizeof(process_name));
        read(fp, process_name, 30);
        close(fp);
        fp = -1;
    }

    info->facing = gCamInfos[camera_id].facing_info.facing;
    if (strstr(process_name,"com.skype.rover")) {
        info->orientation = (info->facing == CAMERA_FACING_BACK)? CONFIG_CAMERA_BACK_ORIENTATION_SKYPE : CONFIG_CAMERA_FRONT_ORIENTATION_SKYPE;       
    } else {        
        info->orientation = gCamInfos[camera_id].facing_info.orientation;       
    }
#else
    info->facing = gCamInfos[camera_id].facing_info.facing;
    info->orientation = gCamInfos[camera_id].facing_info.orientation;       
#endif
end:
    LOGD("%s(%d): camera_%d facing(%d), orientation(%d)",__FUNCTION__,__LINE__,camera_id,info->facing,info->orientation);
    return rv;
}
#if CONFIG_AUTO_DETECT_FRAMERATE 
int camera_framerate_enum(int fd, struct v4l2_frmivalenum *fival,unsigned int w,unsigned int h, unsigned int fmt,int count)
{
	int ret,i;

    i = 0;
    fival->index = 0;
    fival->pixel_format = fmt;
    fival->width = w;
    fival->height = h;
	while (count && (ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, fival)) == 0) {
            	 
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
            fival->pixel_format = fmt;
            fival->width = w;
            fival->height = h;
        }
	}

    if (fival->index && fival->width && (fival->discrete.denominator == 0)) 
        memset(fival,0x00, sizeof(struct v4l2_frmivalenum));
    
	return i;
}

int camera_famerate_detect_loop(void)
{
    camera_device_t *camera_dev;
    Vector<Size> preview_sizes;
    Size preview_size;    
    CameraParameters parameters;
    struct v4l2_frmivalenum *fival;
    unsigned int i,j,w,h,count,ret;
    char camera_device_name[10]; 
    char *temp;

    LOGD("%s Enter..",__func__);
   
    for (i=0; i<2; i++) {        
        if ((strstr(gCamInfos[i].device_path, CAMERA_DEVICE_NAME) != NULL) && (strcmp((char*)&gCamInfos[i].driver[0],"rk29xx-camera") == 0)) {
            snprintf(camera_device_name, sizeof(camera_device_name), "%d", i);
            if (camera_device_open(NULL, camera_device_name, (hw_device_t**)&camera_dev) == 0) {
                temp = camera_dev->ops->get_parameters(camera_dev);
                String8 str_parms(temp);
                camera_dev->ops->put_parameters(camera_dev, temp);
                parameters.unflatten(str_parms);  
                preview_sizes.clear();
                parameters.getSupportedPreviewSizes(preview_sizes);  
                camera_dev->ops->set_parameters(camera_dev,parameters.flatten().string());
                fival = gCamInfos[i].fival_list;
                count = 10;            
                for (j=0; (j<preview_sizes.size()) && (count>0); j++) {
                    preview_size = preview_sizes.itemAt(j);
                    parameters.setPreviewSize(preview_size.width, preview_size.height);
                    camera_dev->ops->start_preview(camera_dev);
                    sleep(2);
                    camera_dev->ops->stop_preview(camera_dev);
                    ret = camera_framerate_enum(gCameraHals[i]->getCameraFd(),fival,preview_size.width, preview_size.height,V4L2_PIX_FMT_NV12,count);                
                    if (ret > 0) {
                        count -= ret;
                        fival += ret;
                    }                
                }
                
                fival = gCamInfos[i].fival_list;
                while (fival->width) {
                    LOGD("Camera_%d %dx%d framerate is %d/%d", i,fival->width,fival->height,fival->discrete.denominator,fival->discrete.numerator);
                    fival++;
                }

                camera_dev->ops->release(camera_dev);
                camera_dev->common.close(&camera_dev->common);
            }
        }
    }

    LOGD("%s Exit..",__func__);
    return false;
}
#endif

