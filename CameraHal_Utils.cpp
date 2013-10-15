/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
/**
* @file CameraHal_Utils.cpp
*
* This file maps the Camera Hardware Interface to V4L2.
*
*/
#include <math.h>
#include "CameraHal.h" 

namespace android {
#define LOG_TAG "CameraHal_Util"

/* Values */
#define EXIF_DEF_MAKER          "rockchip"
#define EXIF_DEF_MODEL          "rk29sdk"


static char ExifMaker[32];
static char ExifModel[32];


static MemManagerBase* cachMem =NULL;
extern "C" int capturePicture_cacheflush(int buf_type, int offset, int len)
{
	int ret = 0;

    /* ddl@rock-chips.com notes: 
     *                     0 : input buffer index for jpeg encoder
     *                     1 : output buffer index for jpeg encoder
     */

    if (buf_type == 0) {            
        buf_type = RAWBUFFER;
    } else if (buf_type == 1) {
        buf_type = JPEGBUFFER;
    }

    cachMem->flushCacheMem((buffer_type_enum)buf_type,offset,len);

	return ret;
}
extern "C"  int YData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w)
{
    int i;

    for (i=0; i<(w>>2); i++) {
        *pdst = ((*psrc>>24)&0x000000ff) | ((*psrc>>8)&0x0000ff00)
                | ((*psrc<<8)&0x00ff0000) | ((*psrc<<24)&0xff000000);
        psrc++;
        pdst--;
    }

    return 0;
}
extern "C"  int UVData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w)
{
    int i;

    for (i=0; i<(w>>2); i++) {
        *pdst = ((*psrc>>16)&0x0000ffff) | ((*psrc<<16)&0xffff0000);                
        psrc++;
        pdst--;
    }

    return 0;
}
extern "C"  int YuvData_Mirror_Flip(int v4l2_fmt_src, char *pdata, char *pline_tmp, int w, int h)
{
    int *pdata_tmp = NULL;
    int *ptop, *pbottom;
    int err = 0,i,j;

    pdata_tmp = (int*)pline_tmp;
    
    // Y mirror and flip
    ptop = (int*)pdata;
    pbottom = (int*)(pdata+w*(h-1));    
    for (j=0; j<(h>>1); j++) {
        YData_Mirror_Line(v4l2_fmt_src, ptop, pdata_tmp+((w>>2)-1),w);
        YData_Mirror_Line(v4l2_fmt_src, pbottom, ptop+((w>>2)-1), w);
        memcpy(pbottom, pdata_tmp, w);
        ptop += (w>>2);
        pbottom -= (w>>2);
    }
    // UV mirror and flip
    ptop = (int*)(pdata+w*h);
    pbottom = (int*)(pdata+w*(h*3/2-1));    
    for (j=0; j<(h>>2); j++) {
        UVData_Mirror_Line(v4l2_fmt_src, ptop, pdata_tmp+((w>>2)-1),w);
        UVData_Mirror_Line(v4l2_fmt_src, pbottom, ptop+((w>>2)-1), w);
        memcpy(pbottom, pdata_tmp, w);
        ptop += (w>>2);
        pbottom -= (w>>2);
    }
YuvData_Mirror_Flip_end:
    return err;
}
extern "C" int YUV420_rotate(const unsigned char* srcy, int src_stride,  unsigned char* srcuv,
                   unsigned char* dsty, int dst_stride, unsigned char* dstuv,
                   int width, int height,int rotate_angle){
   int i = 0,j = 0;
	// 90 , y plane
  if(rotate_angle == 90){
      srcy += src_stride * (height - 1);
  	  srcuv += src_stride * ((height >> 1)- 1); 
  	  src_stride = -src_stride;
	}else if(rotate_angle == 270){
      dsty += dst_stride * (width - 1);
      dstuv += dst_stride * ((width>>1) - 1);
	  dst_stride = -dst_stride;
  }

  for (i = 0; i < width; ++i)
    for (j = 0; j < height; ++j)
      *(dsty+i * dst_stride + j) = *(srcy+j * src_stride + i); 
  
  //uv 
  unsigned char av_u0,av_v0;
  for (i = 0; i < width; i += 2)
    for (j = 0; j < (height>>1); ++j) {
		av_u0 = *(srcuv+i + (j * src_stride));
		av_v0 = *(srcuv+i + (j * src_stride)+1);
      *(dstuv+((j<<1) + ((i >> 1) * dst_stride)))= av_u0;
      *(dstuv+((j<<1) + ((i >> 1) * dst_stride)+1)) = av_v0;
	}
   
  return 0;
 }
/*fill in jpeg gps information*/  
int CameraHal::Jpegfillgpsinfo(RkGPSInfo *gpsInfo,CameraParameters &params)
{
	char* gpsprocessmethod = NULL;
    double latitude,longtitude,altitude;
    double deg,min,sec;
    double fract;
    long timestamp,num; 
    int year,month,day,hour_t,min_t,sec_t;
    char date[12];
    
    if(gpsInfo==NULL) {    
        LOGE( "%s(%d): gpsInfo is NULL ",__FUNCTION__,__LINE__);
        return 0;
    }

    altitude = mGps_altitude;
    latitude = mGps_latitude;
    longtitude = mGps_longitude;
    timestamp = mGps_timestamp; 
    gpsprocessmethod = (char*)params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);
    
    if(latitude >= 0){
    	gpsInfo->GPSLatitudeRef[0] = 'N';
    	gpsInfo->GPSLatitudeRef[1] = '\0';
    }else if((latitude <0)&&(latitude!=-1)){
    	gpsInfo->GPSLatitudeRef[0] = 'S';
    	gpsInfo->GPSLatitudeRef[1] = '\0';
    }else{
    	gpsInfo->GPSLatitudeRef[0] = '\0';
    	gpsInfo->GPSLatitudeRef[1] = '\0';
    }
    
   if(latitude!= -1)
   {
        latitude = fabs(latitude);
        fract = modf(latitude,&deg);
        fract = modf(fract*60,&min);
        fract = modf(fract*60,&sec);
        if(fract >= 0.5)sec+=1;
        
        //LOGD("latitude: deg = %f;min = %f;sec =%f",deg,min,sec);

    	gpsInfo->GPSLatitude[0].num = (uint32_t)deg;
    	gpsInfo->GPSLatitude[0].denom = 1;
    	gpsInfo->GPSLatitude[1].num =  (uint32_t)min;
    	gpsInfo->GPSLatitude[1].denom = 1;
    	gpsInfo->GPSLatitude[2].num =  (uint32_t)sec;
    	gpsInfo->GPSLatitude[2].denom = 1;
   }
  
   if(longtitude >= 0){
    	gpsInfo->GPSLongitudeRef[0] = 'E';
    	gpsInfo->GPSLongitudeRef[1] = '\0';
    }else if((longtitude < 0)&&(longtitude!=-1)){
    	gpsInfo->GPSLongitudeRef[0] = 'W';
    	gpsInfo->GPSLongitudeRef[1] = '\0';
    }else{
    	gpsInfo->GPSLongitudeRef[0] = '\0';
    	gpsInfo->GPSLongitudeRef[1] = '\0';
    }

    if(longtitude!=-1)
    {
        longtitude = fabs(longtitude);
        fract = modf(longtitude,&deg);
        fract = modf(fract*60,&min);
        modf(fract*60,&sec);
        
        //LOGD("longtitude: deg = %f;min = %f;sec =%f",deg,min,sec);
    	gpsInfo->GPSLongitude[0].num = (uint32_t)deg;
    	gpsInfo->GPSLongitude[0].denom = 1;
    	gpsInfo->GPSLongitude[1].num = (uint32_t)min;
    	gpsInfo->GPSLongitude[1].denom = 1;
    	gpsInfo->GPSLongitude[2].num = (uint32_t)sec;
    	gpsInfo->GPSLongitude[2].denom = 1;
    }
    
    if(altitude >= 0){
        gpsInfo->GPSAltitudeRef = 0;
    }else if((altitude <0 )&&(altitude!=-1)) {
        gpsInfo->GPSAltitudeRef = 1;
    } 
    
    if(altitude!=-1)
    {
        altitude = fabs(altitude);
    	gpsInfo->GPSAltitude.num =(uint32_t)altitude;
    	gpsInfo->GPSAltitude.denom = 0x1;
        //LOGD("altitude =%f  GPSAltitudeRef: %d",altitude,gpsInfo->GPSAltitudeRef);        
    }
    
    if(timestamp!=-1)
    {
       /*timestamp,has no meaning,only for passing cts*/
        //LOGD("timestamp =%d",timestamp);
        gpsInfo->GpsTimeStamp[0].num =0;
    	gpsInfo->GpsTimeStamp[0].denom = 1;
    	gpsInfo->GpsTimeStamp[1].num = 0;
    	gpsInfo->GpsTimeStamp[1].denom = 1;
    	gpsInfo->GpsTimeStamp[2].num = timestamp&0x03;
    	gpsInfo->GpsTimeStamp[2].denom = 1;         
    	memcpy(gpsInfo->GpsDateStamp,"2008:01:01\0",11);//"YYYY:MM:DD\0"
    }    
    return 1;
}


int CameraHal::Jpegfillexifinfo(RkExifInfo *exifInfo,CameraParameters &params)
{
    char property[PROPERTY_VALUE_MAX];
    int jpeg_w,jpeg_h;
    int focalen;
    
    if(exifInfo==NULL){
        LOGE( "..%s..%d..argument error ! ",__FUNCTION__,__LINE__);
        return 0;
    }

    /*get some current relavant  parameters*/
    params.getPictureSize(&jpeg_w, &jpeg_h);
    focalen = strtol(params.get(CameraParameters::KEY_FOCAL_LENGTH),0,0);
    
    /*fill in jpeg exif tag*/  
    property_get("ro.product.brand", property, EXIF_DEF_MAKER);
    strncpy((char *)ExifMaker, property,sizeof(ExifMaker) - 1);
    ExifMaker[sizeof(ExifMaker) - 1] = '\0';
 	exifInfo->maker = ExifMaker;
	exifInfo->makerchars = strlen(ExifMaker);
    
    property_get("ro.product.model", property, EXIF_DEF_MODEL);
    strncpy((char *)ExifModel, property,sizeof(ExifModel) - 1);
    ExifModel[sizeof(ExifModel) - 1] = '\0';
	exifInfo->modelstr = ExifModel;
	exifInfo->modelchars = strlen(ExifModel);  
    
	exifInfo->Orientation = 1;

    //3 Date time
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime((char *)exifInfo->DateTime, 20, "%Y:%m:%d %H:%M:%S", timeinfo);
    
	exifInfo->ExposureTime.num = 1;
	exifInfo->ExposureTime.denom = 100;
	exifInfo->ApertureFNumber.num = 0x118;
	exifInfo->ApertureFNumber.denom = 0x64;
	exifInfo->ISOSpeedRatings = 0x59;
	exifInfo->CompressedBitsPerPixel.num = 0x4;
	exifInfo->CompressedBitsPerPixel.denom = 0x1;
	exifInfo->ShutterSpeedValue.num = 0x452;
	exifInfo->ShutterSpeedValue.denom = 0x100;
	exifInfo->ApertureValue.num = 0x2f8;
	exifInfo->ApertureValue.denom = 0x100;
	exifInfo->ExposureBiasValue.num = 0;
	exifInfo->ExposureBiasValue.denom = 0x100;
	exifInfo->MaxApertureValue.num = 0x02f8;
	exifInfo->MaxApertureValue.denom = 0x100;
	exifInfo->MeteringMode = 02;

    if (params.get(CameraParameters::KEY_SUPPORTED_FLASH_MODES)) {
        if (!strcmp(CameraParameters::FLASH_MODE_OFF, params.get(CameraParameters::KEY_FLASH_MODE))) {
	        exifInfo->Flash = 0;
        } else {
            exifInfo->Flash = 0;
        }    
    } else {
        exifInfo->Flash = 0;
    }
    
	exifInfo->FocalLength.num = (uint32_t)focalen;
	exifInfo->FocalLength.denom = 0x1;
    
	exifInfo->FocalPlaneXResolution.num = 0x8383;
	exifInfo->FocalPlaneXResolution.denom = 0x67;
	exifInfo->FocalPlaneYResolution.num = 0x7878;
	exifInfo->FocalPlaneYResolution.denom = 0x76;
	exifInfo->SensingMethod = 2;
	exifInfo->FileSource = 3;
	exifInfo->CustomRendered = 1;
	exifInfo->ExposureMode = 0;
    if (params.get(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE)) {
        if (!strcmp(CameraParameters::WHITE_BALANCE_AUTO, params.get(CameraParameters::KEY_WHITE_BALANCE))) {
	        exifInfo->WhiteBalance = 0;
        } else {
            exifInfo->WhiteBalance = 1;
        }    
    } else {
        exifInfo->WhiteBalance = 0;
    }
	exifInfo->DigitalZoomRatio.num = jpeg_w;
	exifInfo->DigitalZoomRatio.denom = jpeg_w;
	exifInfo->SceneCaptureType = 0x01;   
    return 1;
}

int CameraHal::copyAndSendRawImage(void *raw_image, int size)
{
    camera_memory_t* picture = NULL;
    void *dest = NULL, *src = NULL;

    if(mMsgEnabled & CAMERA_MSG_RAW_IMAGE) { 
        picture = mRequestMemory(-1, size, 1, NULL);
        if (NULL != picture) {
            dest = picture->data;
            if (NULL != dest) {
                memcpy(dest, raw_image, size);
                mDataCb(CAMERA_MSG_RAW_IMAGE, picture, 0, NULL, mCallbackCookie);
            }
            picture->release(picture);
        } else if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY) {
            mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
        }
    } else if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY) {
        mNotifyCb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, mCallbackCookie);
    }
    return 0;
}

int CameraHal::copyAndSendCompressedImage(void *compressed_image, int size)
{
    camera_memory_t* picture = NULL;
    void *dest = NULL, *src = NULL;

    if(mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) { 
        picture = mRequestMemory(-1, size, 1, NULL);
        if (NULL != picture) {
            dest = picture->data;
            if (NULL != dest) {
                memcpy(dest, compressed_image, size);
                mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, picture, 0, NULL, mCallbackCookie);
            }
            picture->release(picture);
        } 
    }
    return 0;
}

int CameraHal::capturePicture(struct CamCaptureInfo_s *capture)
{
    int jpeg_w,jpeg_h,i;
    unsigned int pictureSize;
    unsigned long base, offset, picture_format;
    struct v4l2_buffer buffer;
    struct v4l2_format format;
    struct v4l2_buffer cfilledbuffer;
    struct v4l2_requestbuffers creqbuf;
    int jpegSize;
    int quality;
    int thumbquality = 0;
    int thumbwidth  = 0;
    int thumbheight = 0;
	int err = 0;
	int rotation = 0;
    char *camDriverV4l2Buffer = NULL;
    JpegEncInInfo JpegInInfo;
    JpegEncOutInfo JpegOutInfo;  
    
	RkExifInfo exifInfo;
	RkGPSInfo gpsInfo;
    char ExifAsciiPrefix[8] = {'A', 'S', 'C', 'I', 'I', '\0', '\0', '\0'};
    char gpsprocessmethod[45];
    char *getMethod = NULL;
    double latitude,longtitude,altitude;
    long timestamp;
    bool driver_mirror_fail = false;
    struct v4l2_control control;
    JpegEncType encodetype;
    CameraParameters params;

    cameraParametersGet(params);             /* ddl@rock-chips.com: v0.4.5 */      
     /*get jpeg and thumbnail information*/
    params.getPictureSize(&jpeg_w, &jpeg_h);                
    quality = params.getInt("jpeg-quality");
    rotation = strtol(params.get(CameraParameters::KEY_ROTATION),0,0);
    thumbquality = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY),0,0);
    thumbwidth = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH),0,0);
    thumbheight = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT),0,0);
    /*get gps information*/
    altitude = mGps_altitude;
    latitude = mGps_latitude;
    longtitude = mGps_longitude;
    timestamp = mGps_timestamp;    
    getMethod = (char*)params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);//getMethod : len <= 32
    if (pictureSize & 0xfff) {
        pictureSize = (pictureSize & 0xfffff000) + 0x1000;
    }
	
	cachMem = this->mCamBuffer;
    if (pictureSize > mCamBuffer->getRawBufInfo().mBufferSizes) {
        LOGE("%s(%d): mRawBuffer(size:0x%x) is not enough for this resolution picture(%dx%d, size:0x%x)",
            __FUNCTION__, __LINE__,mCamBuffer->getRawBufInfo().mBufferSizes, jpeg_w, jpeg_h, pictureSize);
        err = -1;
        goto exit;
    } else {
        LOGD("%s(%d): %dx%d quality(%d) rotation(%d)",__FUNCTION__,__LINE__, jpeg_w, jpeg_h,quality,rotation);
    }
    i = 0;
    while (mCamDriverSupportFmt[i]) {
        if (mCamDriverSupportFmt[i] == mCamDriverPictureFmt)
            break;
        i++;
    }

    if (mCamDriverSupportFmt[i] == 0) {
        picture_format = mCamDriverPreviewFmt;        
    } else {
        picture_format = mCamDriverPictureFmt;
    }
	if(mCamDriverPictureFmt ==V4L2_PIX_FMT_RGB565){
		encodetype = HWJPEGENC_RGB565;
		pictureSize = jpeg_w * jpeg_h *2;
		}
	else if(mCamDriverPictureFmt ==V4L2_PIX_FMT_RGB24){
		encodetype = JPEGENC_YUV420_SP;
		pictureSize = jpeg_w * jpeg_h * 3/2;
		}
	else{
		encodetype = JPEGENC_YUV420_SP;
		pictureSize = jpeg_w * jpeg_h * 3/2;
		}
	
    mPictureLock.lock();
    if (mPictureRunning != STA_PICTURE_RUN) {
        mPictureLock.unlock();
        LOGD("%s(%d): capture cancel, because mPictureRunning(0x%x) dosen't suit capture",
            __FUNCTION__,__LINE__, mPictureRunning);
        err = -1;
        goto exit;
    }
    mPictureLock.unlock();

    err = cameraSetSize(jpeg_w, jpeg_h, picture_format,true);
    if (err < 0) {
		LOGE ("CapturePicture failed to set VIDIOC_S_FMT.");
		goto exit;
	}

    /* Check if the camera driver can accept 1 buffer */    
    creqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    creqbuf.memory = mCamDriverV4l2MemType;
    creqbuf.count  = 1;
    if (ioctl(iCamFd, VIDIOC_REQBUFS, &creqbuf) < 0) {
        LOGE ("VIDIOC_REQBUFS Failed. errno = %d", errno);
        err = -1;
        goto exit;
    }

    buffer.type = creqbuf.type;
    buffer.memory = creqbuf.memory;
    buffer.index = 0;
	err = ioctl(iCamFd, VIDIOC_QUERYBUF, &buffer);
    if (err < 0) {
        LOGE("VIDIOC_QUERYBUF Failed");
        goto exit;
    }

    if (buffer.memory == V4L2_MEMORY_OVERLAY) {
        buffer.m.offset = capture->input_phy_addr;    /* phy address */ 
        camDriverV4l2Buffer = (char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir);
    } else if (buffer.memory == V4L2_MEMORY_MMAP) {
        camDriverV4l2Buffer = (char*)mmap(0 /* start anywhere */ ,
                            buffer.length, PROT_READ, MAP_SHARED, iCamFd,
                            buffer.m.offset);
        if (camDriverV4l2Buffer == MAP_FAILED) {
            LOGE("%s(%d): %s(err:%d), unable to map buffer(length:0x%x offset:0x%x)",__FUNCTION__,__LINE__,strerror(errno),errno, buffer.length,buffer.m.offset);
            err = -1;
            goto exit;
        } else {
            LOGD("%s(%d): camDriverV4l2Buffer:0x%x",__FUNCTION__,__LINE__,(int)camDriverV4l2Buffer);
        }
    }
    
    err = ioctl(iCamFd, VIDIOC_QBUF, &buffer);
    if (err < 0) {
        LOGE("CapturePicture VIDIOC_QBUF Failed");
        goto exit;
    }
    
    if (rotation == 180) {
        if (mDriverFlipSupport && mDriverMirrorSupport) {
            
            control.id = V4L2_CID_HFLIP;
			control.value = true;
			err = ioctl(iCamFd, VIDIOC_S_CTRL, &control);
            if (!err) {
                control.id = V4L2_CID_VFLIP;
                control.value = true;
    			err = ioctl(iCamFd, VIDIOC_S_CTRL, &control); 
            }
            
            if (err) {
                driver_mirror_fail = true;
                LOGE("%s(%d): Mirror and flip picture in driver failed!", __FUNCTION__,__LINE__);
            } else {
                driver_mirror_fail = false;
            }
        } else {
            driver_mirror_fail = true;
        }
    }

    /* turn on streaming */
	err = ioctl(iCamFd, VIDIOC_STREAMON, &creqbuf.type);
    if (err < 0) {
        LOGE("CapturePicture VIDIOC_STREAMON Failed");
        goto exit;
    }

    mPictureLock.lock();
    if (mPictureRunning != STA_PICTURE_RUN) {
        mPictureLock.unlock();
        LOGD("%s(%d): capture cancel, because mPictureRunning(0x%x) dosen't suit capture",
            __FUNCTION__,__LINE__, mPictureRunning);
        goto capturePicture_streamoff;
    }
    mPictureLock.unlock();

    if (CAMERA_IS_UVC_CAMERA()) {
        if (strcmp(cameraCallProcess,"com.android.cts.stub")) {
            for (i=0; i<CONFIG_CAMERA_UVC_INVAL_FRAMECNT; i++) {
                ioctl(iCamFd, VIDIOC_DQBUF, &buffer);
                ioctl(iCamFd, VIDIOC_QBUF, &buffer);
            }
        }
    }

    /* De-queue the next avaliable buffer */
    err = ioctl(iCamFd, VIDIOC_DQBUF, &buffer);
    if (err < 0) {
        LOGE("CapturePicture VIDIOC_DQBUF Failed");
        goto exit;
    } 

    if (mMsgEnabled & CAMERA_MSG_SHUTTER)
        mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
    
capturePicture_streamoff:
    /* turn off streaming */
	err = ioctl(iCamFd, VIDIOC_STREAMOFF, &creqbuf.type);
    if (err < 0) {
        LOGE("CapturePicture VIDIOC_STREAMON Failed \n");
        goto exit;
    }

    mPictureLock.lock();
    if (mPictureRunning != STA_PICTURE_RUN) {
        mPictureLock.unlock();
        LOGD("%s(%d): capture cancel, because mPictureRunning(0x%x) dosen't suit capture",
            __FUNCTION__,__LINE__, mPictureRunning);
        err = -1;
        goto exit;
    }
    mPictureLock.unlock();
    
    if (cameraFormatConvert(picture_format, mCamDriverPictureFmt, NULL,
        (char*)camDriverV4l2Buffer,(char*)(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir),0,0,buffer.bytesused,
        jpeg_w, jpeg_h,jpeg_w,
        jpeg_w, jpeg_h,jpeg_w,
        false) == 0) {
        mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);  /* ddl@rock-chips.com: v0.4.0x11 */
    }

    if (mCamDriverV4l2MemType == V4L2_MEMORY_MMAP) {
        if (camDriverV4l2Buffer != NULL) {
            if (munmap((void*)camDriverV4l2Buffer, buffer.length) < 0)
                LOGE("%s camDriverV4l2Buffer munmap failed : %s",__FUNCTION__,strerror(errno));
            camDriverV4l2Buffer = NULL;
        }
    }
    
    if (rotation == 180) {
        if (driver_mirror_fail == true) {
            YuvData_Mirror_Flip(mCamDriverPictureFmt,(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir), 
                (char*)mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_vir), jpeg_w,jpeg_h);
            mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);
        } else {
            if (mDriverFlipSupport && mDriverMirrorSupport) {  
                control.id = V4L2_CID_HFLIP;
    			control.value = false;
    			ioctl(iCamFd, VIDIOC_S_CTRL, &control);                
                control.id = V4L2_CID_VFLIP;
                control.value = false;
                ioctl(iCamFd, VIDIOC_S_CTRL, &control);
            }
        } 
    }

    copyAndSendRawImage((void*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir), pictureSize);

    JpegInInfo.frameHeader = 1;
    
    if ((rotation == 0) || (rotation == 180)) {
        JpegInInfo.rotateDegree = DEGREE_0;        
    } else if (rotation == 90) {
        if(jpeg_w %16 != 0 || jpeg_h %16 != 0){
			YuvData_Mirror_Flip(mCamDriverPictureFmt,(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir), 
					(char*)mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_vir), jpeg_w, jpeg_h);
            mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);
			JpegInInfo.rotateDegree = DEGREE_270;
		}else{
			JpegInInfo.rotateDegree = DEGREE_90;
		}
    } else if (rotation == 270) {
        JpegInInfo.rotateDegree = DEGREE_270; 
    }

    JpegInInfo.yuvaddrfor180 = NULL;
    JpegInInfo.type = encodetype;
	JpegInInfo.y_rgb_addr = capture->input_phy_addr;
	JpegInInfo.uv_addr = capture->input_phy_addr + jpeg_w*jpeg_h;
/* ddl@rock-chips.con : v0.4.1 fix rk2928 rotate 90 and 270 */
    JpegInInfo.inputW = jpeg_w;
    JpegInInfo.inputH = jpeg_h;
    
    JpegInInfo.qLvl = quality/10;
    if (JpegInInfo.qLvl < 5) {
        JpegInInfo.qLvl = 5;
    }

    JpegInInfo.thumbqLvl = thumbquality /10;
    if (JpegInInfo.thumbqLvl < 5) {
        JpegInInfo.thumbqLvl = 5;
    }
    if(JpegInInfo.thumbqLvl  >10) {
        JpegInInfo.thumbqLvl = 9;
    }
    
    if(thumbwidth !=0 && thumbheight !=0) {
        JpegInInfo.doThumbNail = 1;          //insert thumbnail at APP0 extension
    	JpegInInfo.thumbData = NULL;         //if thumbData is NULL, do scale, the type above can not be 420_P or 422_UYVY
    	JpegInInfo.thumbDataLen = -1;
    	JpegInInfo.thumbW = thumbwidth;
    	JpegInInfo.thumbH = thumbheight;
		JpegInInfo.y_vir_addr = (unsigned char*)capture->input_vir_addr;
		JpegInInfo.uv_vir_addr = (unsigned char*)capture->input_vir_addr+jpeg_w*jpeg_h;
    }else{    
        JpegInInfo.doThumbNail = 0;          //insert thumbnail at APP0 extension   
    }
    
    Jpegfillexifinfo(&exifInfo,params);
    JpegInInfo.exifInfo =&exifInfo;
    if((longtitude!=-1)&& (latitude!=-1)&&(timestamp!=-1)&&(getMethod!=NULL)) {    
        Jpegfillgpsinfo(&gpsInfo,params);  
        memset(gpsprocessmethod,0,45);   
        memcpy(gpsprocessmethod,ExifAsciiPrefix,8);   
        memcpy(gpsprocessmethod+8,getMethod,strlen(getMethod)+1);          
        gpsInfo.GpsProcessingMethodchars = strlen(getMethod)+1+8;
        gpsInfo.GPSProcessingMethod  = gpsprocessmethod;
        //LOGD("\nGpsProcessingMethodchars =%d",gpsInfo.GpsProcessingMethodchars);
        JpegInInfo.gpsInfo = &gpsInfo;
    } else {
        JpegInInfo.gpsInfo = NULL;
    }

    JpegOutInfo.outBufPhyAddr = capture->output_phy_addr;
    JpegOutInfo.outBufVirAddr = (unsigned char*)capture->output_vir_addr;
    JpegOutInfo.outBuflen = capture->output_buflen;
    JpegOutInfo.jpegFileLen = 0x00;
    JpegOutInfo.cacheflush= &capturePicture_cacheflush;

    mPictureLock.lock();
    if (mPictureRunning != STA_PICTURE_RUN) {
        mPictureLock.unlock();
        LOGD("%s(%d): capture cancel, because mPictureRunning(0x%x) dosen't suit capture",
            __FUNCTION__,__LINE__, mPictureRunning);
        err = -1;
        goto exit;
    }
    mPictureLock.unlock();

    err = hw_jpeg_encode(&JpegInInfo, &JpegOutInfo);
    
    if ((err < 0) || (JpegOutInfo.jpegFileLen <=0x00)) {
        LOGE("hw_jpeg_encode Failed \n");
        goto exit;
    } else {          
        copyAndSendCompressedImage((void*)JpegOutInfo.outBufVirAddr,JpegOutInfo.jpegFileLen);
    }

exit:   
    if(err < 0) {
        LOGE("%s(%d) take picture erro!!!,",__FUNCTION__,__LINE__);
        if (mNotifyCb && (mMsgEnabled & CAMERA_MSG_ERROR)) {                        
            mNotifyCb(CAMERA_MSG_ERROR, CAMERA_ERROR_SERVER_DIED,0,mCallbackCookie);
        }
    }

    /* ddl@rock-chips.com: v0.4.11 */
    /* ddl@rock-chips.com: Release v4l2 buffer must by close device, buffer isn't release in VIDIOC_STREAMOFF ioctl */
    if (CAMERA_IS_UVC_CAMERA()) {
        close(iCamFd);
        iCamFd = open(cameraDevicePathCur, O_RDWR);
        if (iCamFd < 0) {
            LOGE ("%s[%d]-Could not open the camera device(%s): %s",__FUNCTION__,__LINE__, cameraDevicePathCur, strerror(errno) );
            err = -1;
            goto exit;
        }
    }
    
    return err;
}

int CameraHal::captureVideoPicture(struct CamCaptureInfo_s *capture, int index)
{
    int jpeg_w,jpeg_h,i;
    unsigned int pictureSize;
    unsigned long base, offset;
    int jpegSize;
    int quality;
    int thumbquality = 0;
    int thumbwidth  = 0;
    int thumbheight = 0;
	int err = 0;
	int rotation = 0;
    char *camDriverV4l2Buffer = NULL;
    JpegEncInInfo JpegInInfo;
    JpegEncOutInfo JpegOutInfo;  
    
	RkExifInfo exifInfo;
	RkGPSInfo gpsInfo;
    char ExifAsciiPrefix[8] = {'A', 'S', 'C', 'I', 'I', '\0', '\0', '\0'};
    char gpsprocessmethod[45];
    char *getMethod = NULL;
    double latitude,longtitude,altitude;
    long timestamp;
    struct Message msg;
	JpegEncType encodetype;
    CameraParameters params;

    cameraParametersGet(params); 
    
    if (mMsgEnabled & CAMERA_MSG_SHUTTER)
        mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
    
     /*get jpeg and thumbnail information*/
    params.getPreviewSize(&jpeg_w, &jpeg_h); 
    quality = params.getInt("jpeg-quality");
    rotation = strtol(params.get(CameraParameters::KEY_ROTATION),0,0);
    thumbquality = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY),0,0);
    thumbwidth = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH),0,0);
    thumbheight = strtol(params.get(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT),0,0);
    /*get gps information*/
    altitude = mGps_altitude;
    latitude = mGps_latitude;
    longtitude = mGps_longitude;
    timestamp = mGps_timestamp;    
    getMethod = (char*)params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);//getMethod : len <= 32
	
	cachMem = this->mCamBuffer;
    if (pictureSize & 0xfff) {
        pictureSize = (pictureSize & 0xfffff000) + 0x1000;
    }
	if(mCamDriverPictureFmt ==V4L2_PIX_FMT_RGB565){
		encodetype = HWJPEGENC_RGB565;
		pictureSize = jpeg_w * jpeg_h *2;
		}
	else{
		encodetype = JPEGENC_YUV420_SP;
		pictureSize = jpeg_w * jpeg_h * 3/2;
		}
    /*ddl@rock-chips.com: v0.4.7*/    
    if ((mCamDriverPreviewFmt != mCamDriverPictureFmt)||(rotation == 180)) {
        if (CAMERA_IS_RKSOC_CAMERA()) {
            if (cameraFormatConvert(mCamDriverPreviewFmt, mCamDriverPictureFmt, NULL,
                (char*)capture->input_vir_addr,(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir),0,0, mPreviewFrameSize,
                jpeg_w, jpeg_h,jpeg_w, 
                jpeg_w, jpeg_h,jpeg_w,
                false) == 0)
                mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);
        } else if (CAMERA_IS_UVC_CAMERA()) {            
            if (V4L2_PIX_FMT_NV12!= mCamDriverPreviewFmt) {  /* ddl@rock-chips.com: v0.4.15 */                
                if (cameraFormatConvert(V4L2_PIX_FMT_NV12, mCamDriverPictureFmt, NULL,
                    (char*)capture->input_vir_addr,(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir),0,0,mPreviewFrameSize, 
                    jpeg_w, jpeg_h,jpeg_w, 
                    jpeg_w, jpeg_h,jpeg_w,
                    false)==0)
                    mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);
            }
        }
        capture->input_phy_addr = mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_phy);
        capture->input_vir_addr = (int)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir);
    }
    copyAndSendRawImage((void*)capture->input_vir_addr, pictureSize);

    JpegInInfo.frameHeader = 1;
	JpegInInfo.yuvaddrfor180 = NULL;
    if ((rotation == 0) ){
	    JpegInInfo.rotateDegree = DEGREE_0;    
    }else if(rotation == 180){
	    JpegInInfo.rotateDegree = DEGREE_0;
        YuvData_Mirror_Flip(mCamDriverPictureFmt,(char*)mCamBuffer->getBufferAddr(RAWBUFFER, 0, buffer_addr_vir), 
            (char*)mCamBuffer->getBufferAddr(JPEGBUFFER, 0, buffer_addr_vir), jpeg_w,jpeg_h);
        mCamBuffer->flushCacheMem(RAWBUFFER,0,mCamBuffer->getRawBufInfo().mBufferSizes);
    }else if (rotation == 90) {
        JpegInInfo.rotateDegree = DEGREE_90;
    } else if (rotation == 270) {
        JpegInInfo.rotateDegree = DEGREE_270; 
    }

    JpegInInfo.type = encodetype;
    JpegInInfo.y_rgb_addr = capture->input_phy_addr;
    JpegInInfo.uv_addr = capture->input_phy_addr + jpeg_w*jpeg_h;    
    JpegInInfo.inputW = jpeg_w;
    JpegInInfo.inputH = jpeg_h;
    
	JpegInInfo.qLvl = quality/10;
    if (JpegInInfo.qLvl < 5) {
        JpegInInfo.qLvl = 5;
    }
    JpegInInfo.thumbqLvl = thumbquality /10;
    if (JpegInInfo.thumbqLvl < 5) {
        JpegInInfo.thumbqLvl = 5;
    }
    if(JpegInInfo.thumbqLvl  >10) {
        JpegInInfo.thumbqLvl = 9;
    }

    if(thumbwidth !=0 && thumbheight !=0) {
        JpegInInfo.doThumbNail = 1;          //insert thumbnail at APP0 extension
    	JpegInInfo.thumbData = NULL;         //if thumbData is NULL, do scale, the type above can not be 420_P or 422_UYVY
    	JpegInInfo.thumbDataLen = -1;
    	JpegInInfo.thumbW = thumbwidth;
    	JpegInInfo.thumbH = thumbheight;
        JpegInInfo.y_vir_addr = (unsigned char*)capture->input_vir_addr;
        JpegInInfo.uv_vir_addr = (unsigned char*)capture->input_vir_addr+jpeg_w*jpeg_h;
    }else{    
        JpegInInfo.doThumbNail = 0;          //insert thumbnail at APP0 extension   
    }
    
    Jpegfillexifinfo(&exifInfo,params);
    JpegInInfo.exifInfo =&exifInfo;
    if((longtitude!=-1)&& (latitude!=-1)&&(timestamp!=-1)&&(getMethod!=NULL)) {    
        Jpegfillgpsinfo(&gpsInfo,params);  
        memset(gpsprocessmethod,0,45);   
        memcpy(gpsprocessmethod,ExifAsciiPrefix,8);   
        memcpy(gpsprocessmethod+8,getMethod,strlen(getMethod)+1);          
        gpsInfo.GpsProcessingMethodchars = strlen(getMethod)+1+8;
        gpsInfo.GPSProcessingMethod  = gpsprocessmethod;
        LOGD("\nGpsProcessingMethodchars =%d",gpsInfo.GpsProcessingMethodchars);
        JpegInInfo.gpsInfo = &gpsInfo;
    } else {
        JpegInInfo.gpsInfo = NULL;
    }

    JpegOutInfo.outBufPhyAddr = capture->output_phy_addr;
    JpegOutInfo.outBufVirAddr = (unsigned char*)capture->output_vir_addr;
    JpegOutInfo.outBuflen = capture->output_buflen;
    JpegOutInfo.jpegFileLen = 0x00;
    JpegOutInfo.cacheflush= &capturePicture_cacheflush;
    
    err = hw_jpeg_encode(&JpegInInfo, &JpegOutInfo);

    cameraPreviewBufferSetSta(mPreviewBufferMap[index], CMD_PREVIEWBUF_SNAPSHOT_ENCING, 0);    
    if (mPreviewRunning == STA_PREVIEW_RUN) {	     
        msg.command = CMD_PREVIEW_QBUF;     
        msg.arg1 = (void*)index;
        msg.arg2 = (void*)CMD_PREVIEWBUF_SNAPSHOT_ENCING;
        msg.arg3 = (void*)mPreviewStartTimes;
        commandThreadCommandQ.put(&msg); 
    }
    
    if ((err < 0) || (JpegOutInfo.jpegFileLen <=0x00)) {
        LOGE("%s(%d): hw_jpeg_encode Failed, err: %d  JpegOutInfo.jpegFileLen:0x%x\n",__FUNCTION__,__LINE__,
            err, JpegOutInfo.jpegFileLen);

        LOGE("%s(%d): JpegOutInfo.outBuflen:0x%x",__FUNCTION__,__LINE__,JpegOutInfo.outBuflen);
        goto exit;
    } else { 
        copyAndSendCompressedImage((void*)JpegOutInfo.outBufVirAddr,JpegOutInfo.jpegFileLen);       
    }
exit:  
    if(err < 0) {
        LOGE("%s(%d) take picture erro!!!,",__FUNCTION__,__LINE__);
        if (mNotifyCb && (mMsgEnabled & CAMERA_MSG_ERROR)) {                        
            mNotifyCb(CAMERA_MSG_ERROR, CAMERA_ERROR_SERVER_DIED,0,mCallbackCookie);
        }
    } 
return err;

}

};




