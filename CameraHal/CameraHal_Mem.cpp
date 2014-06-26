/*
*Author: zyc@rock-chips.co
*/
#include <sys/stat.h>
#include <unistd.h>
#include <utils/CallStack.h>
#include "CameraHal.h"


namespace android {


/******************ION BUFFER START*******************/
MemManagerBase::MemManagerBase()
{
	mPreviewBufferInfo = NULL;
	mRawBufferInfo = NULL;
	mJpegBufferInfo = NULL;
	mJpegBufferInfo = NULL;
}
MemManagerBase::~MemManagerBase()
{
	mPreviewBufferInfo = NULL;
	mRawBufferInfo = NULL;
	mJpegBufferInfo = NULL;
	mJpegBufferInfo = NULL;
}

unsigned int MemManagerBase::getBufferAddr(enum buffer_type_enum buf_type, unsigned int buf_idx, buffer_addr_t addr_type)
{
    unsigned int addr = 0x00;
    struct bufferinfo_s *buf_info;
   
    switch(buf_type)
    {
		case PREVIEWBUFFER:
			buf_info = mPreviewBufferInfo;
			break;
		case RAWBUFFER:
			buf_info = mRawBufferInfo;
			break;
		case JPEGBUFFER:
			buf_info = mJpegBufferInfo;
			break;
		case VIDEOENCBUFFER:
			buf_info = mVideoEncBufferInfo;
			break;
        default:
            LOGD("%s(%d): Buffer type(0x%x) is invaildate",__FUNCTION__,__LINE__,buf_type);
            goto getVirAddr_end;
    }

    if (buf_idx > buf_info->mNumBffers) {
        LOGE("%s(%d): Buffer index(0x%x) is invalidate, Total buffer is 0x%x",__FUNCTION__,__LINE__,
            buf_idx,buf_info->mNumBffers);
        goto getVirAddr_end;
    }

    if (addr_type == buffer_addr_vir) {
        addr = (buf_info+buf_idx)->mVirBaseAddr;
    } else if (addr_type == buffer_addr_phy) {
        addr = (buf_info+buf_idx)->mPhyBaseAddr;
    }
getVirAddr_end:
    return addr;
}

int MemManagerBase::dump()
{
    
    return 0;
}

#if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)

IonMemManager::IonMemManager()
			     :MemManagerBase(),
			      mPreviewData(NULL),
			      mRawData(NULL),
			      mJpegData(NULL),
				mVideoEncData(NULL),
				mIonMemMgr(NULL)
{
	mIonMemMgr = new IonAlloc(PAGE_SIZE, ION_MODULE_CAM);
}

IonMemManager::~IonMemManager()
{
	if (mPreviewData) {
		destroyPreviewBuffer();
		free(mPreviewData);
        mPreviewData = NULL;
	}
	if(mRawData) {
		destroyRawBuffer();
		free(mRawData);
        mRawData = NULL;
	}
	if(mJpegData) {
		destroyJpegBuffer();
		free(mJpegData);
        mJpegData = NULL;
	}
	if(mVideoEncData) {
		destroyVideoEncBuffer();
		free(mVideoEncData);
		mVideoEncData = NULL;
	}
	if(mIonMemMgr)
		delete mIonMemMgr;
	mIonMemMgr = NULL;

}

int IonMemManager::createIonBuffer(struct bufferinfo_s* ionbuf)
{
	int ret =0;
	int numBufs;
	int frame_size;
	struct ion_buffer_t* tmpalloc = NULL;
	struct bufferinfo_s* tmp_buf = NULL;
    
	if (!ionbuf || !mIonMemMgr < 0) {
		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
		goto null_fail;
	}
    
	numBufs = ionbuf->mNumBffers;
	frame_size = ionbuf->mPerBuffersize;
	ionbuf->mBufferSizes = numBufs*PAGE_ALIGN(frame_size);
	switch(ionbuf->mBufType)
	{
		case PREVIEWBUFFER:
			tmpalloc = mPreviewData;
			tmp_buf = &mPreviewBufferInfo;
			break;
		case RAWBUFFER:
			tmpalloc = mRawData;
			tmp_buf = &mRawBufferInfo;
			break;
		case JPEGBUFFER:
			tmpalloc = mJpegData;
			tmp_buf = &mJpegBufferInfo;
			break;
		case VIDEOENCBUFFER:
			tmpalloc = mVideoEncData;
			tmp_buf = &mVideoEncBufferInfo;
			break;
        default:
            goto null_fail;
    }
    
	memset(tmpalloc,0,sizeof(tmpalloc));
	ret = mIonMemMgr->alloc(ionbuf->mBufferSizes, _ION_HEAP_RESERVE, tmpalloc);
	if(ret != 0) {
		LOGE("%s(%d): ion_alloc malloc buffer failed , type is = %d ",__FUNCTION__,__LINE__,ionbuf->mBufType);
		goto alloc_fail;
	}
    
	ionbuf->mPhyBaseAddr = (unsigned long)tmpalloc->phys;
	ionbuf->mVirBaseAddr = (unsigned long)tmpalloc->virt;
	ionbuf->mPerBuffersize = PAGE_ALIGN(frame_size);
	*tmp_buf = *ionbuf;

    return 0;

alloc_fail:
null_fail:
    memset(tmp_buf,0,sizeof(struct bufferinfo_s));
    memset(tmpalloc,0,sizeof(ion_buffer_t));
    return ret;
}

void IonMemManager::destroyIonBuffer(buffer_type_enum buftype)
{
	struct ion_buffer_t* tmpalloc = NULL;
    
	switch(buftype)
	{
		case PREVIEWBUFFER:
			tmpalloc = mPreviewData;
			if(tmpalloc && tmpalloc->virt) {
				LOGD("%s(%d): free preview buffer success!",__FUNCTION__,__LINE__);
				mIonMemMgr->free(*tmpalloc);
				memset(tmpalloc,0,sizeof(ion_buffer_t));
 			} else {
 			    if (mPreviewData == NULL) {
				  //  LOGE("%s(%d): mPreviewData is NULL",__FUNCTION__,__LINE__);
                } else {
				    LOGE("%s(%d): mPreviewData->virt:0x%x mPreviewBufferInfo.mVirBaseAddr:0x%x",__FUNCTION__,__LINE__,(int)mPreviewData->virt,mPreviewBufferInfo.mVirBaseAddr);
                }
			}
			memset(&mPreviewBufferInfo,0,sizeof(mPreviewBufferInfo));
			break;
		case RAWBUFFER:
			tmpalloc = mRawData;
			if(tmpalloc && tmpalloc->virt) {
				LOGD("%s(%d): free RAWBUFFER buffer success!",__FUNCTION__,__LINE__);
				mIonMemMgr->free(*tmpalloc);
				memset(tmpalloc,0,sizeof(ion_buffer_t));
			} else {
				if (mRawData == NULL) {
				 //   LOGE("%s(%d): mRawData is NULL",__FUNCTION__,__LINE__);
                } else {
				    LOGE("%s(%d): mRawData->virt:0x%x mRawBufferInfo.mVirBaseAddr:0x%x",__FUNCTION__,__LINE__,(int)mRawData->virt,mRawBufferInfo.mVirBaseAddr);
                }
			}
			memset(&mRawBufferInfo,0,sizeof(mRawBufferInfo));
			break;
		case JPEGBUFFER:
			tmpalloc = mJpegData;
			if(tmpalloc && tmpalloc->virt) {
				LOGD("%s(%d): free RAWBUFFER buffer success!",__FUNCTION__,__LINE__);
				mIonMemMgr->free(*tmpalloc);
				memset(tmpalloc,0,sizeof(ion_buffer_t));
            } else {
				if (mJpegData == NULL) {
				//    LOGE("%s(%d): mJpegData is NULL",__FUNCTION__,__LINE__);
                } else {
				    LOGE("%s(%d): mJpegData->virt:0x%x mRawBufferInfo.mVirBaseAddr:0x%x",__FUNCTION__,__LINE__,(int)mJpegData->virt,mJpegBufferInfo.mVirBaseAddr);
                }
			}
			memset(&mJpegBufferInfo,0,sizeof(mJpegBufferInfo));
			break;
		case VIDEOENCBUFFER:
			tmpalloc = mVideoEncData;
			if(tmpalloc && tmpalloc->virt) {
				LOGD("%s(%d): free VIDEOENCBUFFER buffer success!",__FUNCTION__,__LINE__);
				mIonMemMgr->free(*tmpalloc);
				memset(tmpalloc,0,sizeof(ion_buffer_t));
			} else {
				if (mVideoEncData == NULL) {
				//	LOGE("%s(%d): mVideoEncData is NULL",__FUNCTION__,__LINE__);
				} else {
					LOGE("%s(%d): mVideoEncData->virt:0x%x mVideoEncBufferInfo.mVirBaseAddr:0x%x",__FUNCTION__,__LINE__,(int)mVideoEncData->virt,mVideoEncBufferInfo.mVirBaseAddr);
				}
			}
			memset(&mVideoEncBufferInfo,0,sizeof(mVideoEncBufferInfo));
			break;

        default:
		   	LOGE("%s(%d): buffer type is wrong !",__FUNCTION__,__LINE__);
            break;
	}
}

int IonMemManager::createVideoEncBuffer(struct bufferinfo_s* videoencbuf)
{
	LOG_FUNCTION_NAME
	int ret;
	Mutex::Autolock lock(mLock);
	
	if(videoencbuf->mBufType != VIDEOENCBUFFER)
		LOGE("%s(%d): the type is not VIDEOENCBUFFER",__FUNCTION__,__LINE__);
	
	if(!mVideoEncData) {
		mVideoEncData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t));
	} else if(mVideoEncData->virt) {
		LOGD("%s(%d): FREE the video buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyVideoEncBuffer();
	}
	
	memset(mVideoEncData,0,sizeof(ion_buffer_t));
	
	ret = createIonBuffer(videoencbuf);
	if (ret == 0) {
		LOGD("%s(%d): Video buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
			mVideoEncBufferInfo.mPhyBaseAddr,mVideoEncBufferInfo.mVirBaseAddr,mVideoEncBufferInfo.mBufferSizes);
	} else {
		LOGE("%s(%d): Video buffer alloc failed",__FUNCTION__,__LINE__);
	}
	LOG_FUNCTION_NAME_EXIT
	return ret;
}
int IonMemManager::destroyVideoEncBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);

	destroyIonBuffer(VIDEOENCBUFFER);

	LOG_FUNCTION_NAME_EXIT
	return 0;

}

int IonMemManager::createPreviewBuffer(struct bufferinfo_s* previewbuf)
{
	LOG_FUNCTION_NAME
    int ret;
	Mutex::Autolock lock(mLock);
    
	if(previewbuf->mBufType != PREVIEWBUFFER)
		LOGE("%s(%d): the type is not PREVIEWBUFFER",__FUNCTION__,__LINE__);
    
	if(!mPreviewData) {
		mPreviewData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t));
	} else if(mPreviewData->virt) {
		LOGD("%s(%d): FREE the preview buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyPreviewBuffer();
	}
    
	memset(mPreviewData,0,sizeof(ion_buffer_t));
    
    ret = createIonBuffer(previewbuf);
    if (ret == 0) {
        LOGD("%s(%d): Preview buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mPreviewBufferInfo.mPhyBaseAddr,mPreviewBufferInfo.mVirBaseAddr,mPreviewBufferInfo.mBufferSizes);
    } else {
        LOGE("%s(%d): Preview buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;
}
int IonMemManager::destroyPreviewBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);

	destroyIonBuffer(PREVIEWBUFFER);

	LOG_FUNCTION_NAME_EXIT
	return 0;

}
int IonMemManager::createRawBuffer(struct bufferinfo_s* rawbuf)
{
	LOG_FUNCTION_NAME
    int ret;
	Mutex::Autolock lock(mLock);
    
	if (rawbuf->mBufType != RAWBUFFER)
		LOGE("%s(%d): the type is not RAWBUFFER",__FUNCTION__,__LINE__);

    if (!mRawData) {
		mRawData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t));
	} else if(mRawData->virt) {
		LOGD("%s(%d): FREE the raw buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyRawBuffer();
	}
	memset(mRawData,0,sizeof(ion_buffer_t));

    ret = createIonBuffer(rawbuf);
    if (ret == 0) {
        LOGD("%s(%d): Raw buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mRawBufferInfo.mPhyBaseAddr,mRawBufferInfo.mVirBaseAddr,mRawBufferInfo.mBufferSizes);
    } else {
        LOGE("%s(%d): Raw buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;

}
int IonMemManager::destroyRawBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);
	destroyIonBuffer(RAWBUFFER);
	LOG_FUNCTION_NAME_EXIT
	return 0;
}
 int IonMemManager::createJpegBuffer(struct bufferinfo_s* jpegbuf)
 {
    LOG_FUNCTION_NAME
    int ret;
    Mutex::Autolock lock(mLock);

    if(jpegbuf->mBufType != JPEGBUFFER)
        LOGE("%s(%d): the type is not JPEGBUFFER",__FUNCTION__,__LINE__);

    if(!mJpegData) {
        mJpegData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t));
    } else if(mJpegData->virt) {
        LOGD("%s(%d): FREE the jpeg buffer alloced before firstly",__FUNCTION__,__LINE__);
        destroyJpegBuffer();
    }
    memset(mJpegData,0,sizeof(ion_buffer_t));
    
    ret = createIonBuffer(jpegbuf);
    if (ret == 0) {
        LOGD("%s(%d): Jpeg buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mJpegBufferInfo.mPhyBaseAddr,mJpegBufferInfo.mVirBaseAddr,mJpegBufferInfo.mBufferSizes);
    } else {
        LOGE("%s(%d): Jpeg buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;

 }
int IonMemManager::destroyJpegBuffer()
{
	 LOG_FUNCTION_NAME
	 Mutex::Autolock lock(mLock);
	 destroyIonBuffer(JPEGBUFFER);
	 LOG_FUNCTION_NAME_EXIT
	 return 0;

}
int IonMemManager::flushCacheMem(buffer_type_enum buftype,unsigned int offset, unsigned int len)
{
    Mutex::Autolock lock(mLock);
    ion_buffer_t data;
    
    switch(buftype)
	{
		case PREVIEWBUFFER:
			data = *mPreviewData;
			break;
		case RAWBUFFER:
			data = *mRawData;
            
			break;
		case JPEGBUFFER:
			data = *mJpegData;
			break;
		case VIDEOENCBUFFER:
			data = *mVideoEncData;
			break;
		default:
			break;
	}
    
    mIonMemMgr->cache_op(data, ION_FLUSH_CACHE);

    return 0;
}
#endif

#if (CONFIG_CAMERA_MEM == CAMERA_MEM_IONDMA)
IonDmaMemManager::IonDmaMemManager()
			     :MemManagerBase(),
			      mPreviewData(NULL),
			      mRawData(NULL),
			      mJpegData(NULL),
				mVideoEncData(NULL),
				client_fd(-1)
{
	client_fd = ion_open();
}

IonDmaMemManager::~IonDmaMemManager()
{
	if (mPreviewData) {
		destroyPreviewBuffer();
		free(mPreviewData);
        mPreviewData = NULL;
	}
	if(mRawData) {
		destroyRawBuffer();
		free(mRawData);
        mRawData = NULL;
	}
	if(mJpegData) {
		destroyJpegBuffer();
		free(mJpegData);
        mJpegData = NULL;
	}
	if(mVideoEncData) {
		destroyVideoEncBuffer();
		free(mVideoEncData);
		mVideoEncData = NULL;
	}
	if(client_fd != -1)
         ion_close(client_fd);
		

}

int IonDmaMemManager::createIonBuffer(struct bufferinfo_s* ionbuf)
{
	int ret =0,i = 0;
	int numBufs;
	int frame_size;
	camera_ionbuf_t* tmpalloc = NULL;
	struct bufferinfo_s* tmp_buf = NULL;
    struct ion_handle* handle = NULL;
    int map_fd;
    unsigned long vir_addr = 0;

    
	if (!ionbuf) {
		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
		return -1;
	}
    
	numBufs = ionbuf->mNumBffers;
	frame_size = ionbuf->mPerBuffersize;
	ionbuf->mBufferSizes = numBufs*PAGE_ALIGN(frame_size);
	switch(ionbuf->mBufType)
	{
		case PREVIEWBUFFER:
            tmpalloc = mPreviewData ;
			if((tmp_buf  = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s))) != NULL){
                mPreviewBufferInfo = tmp_buf;
            }else{
        		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
        		return -1;
            }
			break;
		case RAWBUFFER:
            tmpalloc =  mRawData;
               
			if((tmp_buf = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s))) != NULL){
                mRawBufferInfo = tmp_buf;
            }else{
        		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
        		return -1;
            }
			break;
		case JPEGBUFFER:
            tmpalloc = mJpegData;
            
			if((tmp_buf  = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s))) != NULL ){
                mJpegBufferInfo = tmp_buf;
            }else{
        		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
        		return -1;
            }
			break;
		case VIDEOENCBUFFER:
            tmpalloc =  mVideoEncData ;

            if((tmp_buf = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s))) != NULL){
                mVideoEncBufferInfo = tmp_buf;
            }else{
        		LOGE("%s(%d): ion_alloc malloc buffer failed",__FUNCTION__,__LINE__);
        		return -1;
            }
			break;
        default:
            return -1;
    }

    for(i = 0;i < numBufs;i++){
    	memset(tmpalloc,0,sizeof(struct bufferinfo_s));

        #if (USE_ION_VMALLOC_BUF == 0)
            ret = ion_alloc(client_fd, ionbuf->mPerBuffersize, PAGE_SIZE, 2, 0, &handle);
        #else //vmalloc
            ret = ion_alloc(client_fd, ionbuf->mPerBuffersize, PAGE_SIZE, 8, 0, &handle);
        #endif
        if (ret) {
            LOGE("ion alloc failed\n");
            break;
        }

        LOG1("handle %p\n", handle);

        ret = ion_share(client_fd,handle,&map_fd);
        if (ret) {
            LOGE("ion map failed\n");
            ion_free(client_fd,handle);
            break;
        }

            vir_addr = (unsigned int )mmap(NULL, ionbuf->mPerBuffersize, PROT_READ | PROT_WRITE, MAP_SHARED, map_fd, 0);
        if (vir_addr == 0) {
            LOGE("ion mmap failed\n");
            ret = -1;
            ion_free(client_fd,handle);
            break;
        }
        
        #if (USE_ION_VMALLOC_BUF == 0)
            ion_get_phys(client_fd,handle,&(tmpalloc->phy_addr));
        #else //vmalloc
        {
            tmpalloc->phy_addr = map_fd;
        }
        #endif
        tmpalloc->size = ionbuf->mPerBuffersize;
        tmpalloc->vir_addr = vir_addr;
        tmpalloc->ion_hdl = (void*)handle;
        tmpalloc->map_fd    =   map_fd;

        
    	ionbuf->mPhyBaseAddr = (unsigned long)tmpalloc->phy_addr;
    	ionbuf->mVirBaseAddr = (unsigned long)tmpalloc->vir_addr;
    	ionbuf->mPerBuffersize = PAGE_ALIGN(frame_size);
    	*tmp_buf = *ionbuf;
        tmp_buf++;
        tmpalloc++;
        
    }

    if(ret < 0){
        LOGE("%s:%d failed !",__func__,__LINE__);
        while(--i >= 0){
            --tmpalloc;
            --tmp_buf;
            munmap((void *)tmpalloc->vir_addr, tmpalloc->size);
            ion_free(client_fd, (struct ion_handle*)(tmpalloc->ion_hdl));
        }
        delete tmpalloc;
        delete tmp_buf;
    }
    return ret;
}

void IonDmaMemManager::destroyIonBuffer(buffer_type_enum buftype)
{
	camera_ionbuf_t* tmpalloc = NULL;
    int err = 0;
	struct bufferinfo_s* tmp_buf = NULL;

   
	switch(buftype)
	{
		case PREVIEWBUFFER:
			tmpalloc = mPreviewData;
            tmp_buf = mPreviewBufferInfo;
			break;
		case RAWBUFFER:
			tmpalloc = mRawData;
            tmp_buf = mRawBufferInfo;
			break;
		case JPEGBUFFER:
			tmpalloc = mJpegData;
            tmp_buf = mJpegBufferInfo;
			break;
		case VIDEOENCBUFFER:
			tmpalloc = mVideoEncData;
            tmp_buf = mVideoEncBufferInfo;
			break;

        default:
		   	LOGE("%s(%d): buffer type is wrong !",__FUNCTION__,__LINE__);
            break;
	}


    for(unsigned int i = 0;(tmp_buf && (i < tmp_buf->mNumBffers));i++){
    	if(tmpalloc && tmpalloc->vir_addr) {
                err = munmap((void *)tmpalloc->vir_addr, tmpalloc->size);
            if (err) {
                LOGE("%s(%d):munmap failed\n",__FUNCTION__,__LINE__);
                    return;
            }

            close(tmpalloc->map_fd);

            err = ion_free(client_fd, (struct ion_handle*)(tmpalloc->ion_hdl));

        }
        tmpalloc++;
    }

	switch(buftype)
	{
		case PREVIEWBUFFER:
			delete mPreviewData;
            mPreviewData = NULL;
            delete mPreviewBufferInfo;
            mPreviewBufferInfo = NULL;
			break;
		case RAWBUFFER:
			delete mRawData;
            mRawData = NULL;
            delete mRawBufferInfo;
            mRawBufferInfo = NULL;
			break;
		case JPEGBUFFER:
			delete mJpegData;
            mJpegData = NULL;
            delete mJpegBufferInfo;
            mJpegBufferInfo = NULL;
			break;
		case VIDEOENCBUFFER:
			delete mVideoEncData;
            mVideoEncData = NULL;
            delete mVideoEncBufferInfo;
            mVideoEncBufferInfo = NULL;
			break;

        default:
		   	LOGE("%s(%d): buffer type is wrong !",__FUNCTION__,__LINE__);
            break;
	}

    
}


int IonDmaMemManager::createVideoEncBuffer(struct bufferinfo_s* videoencbuf)
{
	LOG_FUNCTION_NAME
	int ret;
	Mutex::Autolock lock(mLock);
	
	if(videoencbuf->mBufType != VIDEOENCBUFFER)
		LOGE("%s(%d): the type is not VIDEOENCBUFFER",__FUNCTION__,__LINE__);
	
	if(!mVideoEncData) {
		mVideoEncData = (camera_ionbuf_t*)malloc(sizeof(camera_ionbuf_t) * videoencbuf->mNumBffers);
	} else if(mVideoEncData->vir_addr) {
		LOGD("%s(%d): FREE the video buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyVideoEncBuffer();
	}
	
	memset(mVideoEncData,0,sizeof(camera_ionbuf_t)* videoencbuf->mNumBffers);
	
	ret = createIonBuffer(videoencbuf);
	if (ret == 0) {
		LOGD("%s(%d): Video buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
			mVideoEncBufferInfo->mPhyBaseAddr,mVideoEncBufferInfo->mVirBaseAddr,mVideoEncBufferInfo->mBufferSizes);
	} else {
		LOGE("%s(%d): Video buffer alloc failed",__FUNCTION__,__LINE__);
	}
	LOG_FUNCTION_NAME_EXIT
	return ret;
}
int IonDmaMemManager::destroyVideoEncBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);

	destroyIonBuffer(VIDEOENCBUFFER);

	LOG_FUNCTION_NAME_EXIT
	return 0;

}

int IonDmaMemManager::createPreviewBuffer(struct bufferinfo_s* previewbuf)
{
	LOG_FUNCTION_NAME
    int ret;
	Mutex::Autolock lock(mLock);
    
	if(previewbuf->mBufType != PREVIEWBUFFER)
		LOGE("%s(%d): the type is not PREVIEWBUFFER",__FUNCTION__,__LINE__);
    
	if(!mPreviewData) {
		mPreviewData = (camera_ionbuf_t*)malloc(sizeof(camera_ionbuf_t) * previewbuf->mNumBffers);
	} else if(mPreviewData->vir_addr) {
		LOGD("%s(%d): FREE the preview buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyPreviewBuffer();
	}
    
	memset(mPreviewData,0,sizeof(camera_ionbuf_t)* previewbuf->mNumBffers);
    
    ret = createIonBuffer(previewbuf);
    if (ret == 0) {
        LOGD("%s(%d): Preview buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mPreviewBufferInfo->mPhyBaseAddr,mPreviewBufferInfo->mVirBaseAddr,mPreviewBufferInfo->mBufferSizes);
    } else {
        LOGE("%s(%d): Preview buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;
}
int IonDmaMemManager::destroyPreviewBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);

	destroyIonBuffer(PREVIEWBUFFER);

	LOG_FUNCTION_NAME_EXIT
	return 0;

}
int IonDmaMemManager::createRawBuffer(struct bufferinfo_s* rawbuf)
{
	LOG_FUNCTION_NAME
    int ret;
	Mutex::Autolock lock(mLock);
    
	if (rawbuf->mBufType != RAWBUFFER)
		LOGE("%s(%d): the type is not RAWBUFFER",__FUNCTION__,__LINE__);

    if (!mRawData) {
		mRawData = (camera_ionbuf_t*)malloc(sizeof(camera_ionbuf_t) * rawbuf->mNumBffers);
	} else if(mRawData->vir_addr) {
		LOGD("%s(%d): FREE the raw buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyRawBuffer();
	}
	memset(mRawData,0,sizeof(camera_ionbuf_t)* rawbuf->mNumBffers);

    ret = createIonBuffer(rawbuf);
    if (ret == 0) {
        LOGD("%s(%d): Raw buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mRawBufferInfo->mPhyBaseAddr,mRawBufferInfo->mVirBaseAddr,mRawBufferInfo->mBufferSizes);
    } else {
        LOGE("%s(%d): Raw buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;

}
int IonDmaMemManager::destroyRawBuffer()
{
	LOG_FUNCTION_NAME
	Mutex::Autolock lock(mLock);
	destroyIonBuffer(RAWBUFFER);
	LOG_FUNCTION_NAME_EXIT
	return 0;
}

 int IonDmaMemManager::createJpegBuffer(struct bufferinfo_s* jpegbuf)
 {
    LOG_FUNCTION_NAME
    int ret;
    Mutex::Autolock lock(mLock);

    if(jpegbuf->mBufType != JPEGBUFFER)
        LOGE("%s(%d): the type is not JPEGBUFFER",__FUNCTION__,__LINE__);

    if(!mJpegData) {
        mJpegData = (camera_ionbuf_t*)malloc(sizeof(camera_ionbuf_t) * jpegbuf->mNumBffers);
    } else if(mJpegData->vir_addr) {
        LOGD("%s(%d): FREE the jpeg buffer alloced before firstly",__FUNCTION__,__LINE__);
        destroyJpegBuffer();
    }
    memset(mJpegData,0,sizeof(camera_ionbuf_t)* jpegbuf->mNumBffers);
    
    ret = createIonBuffer(jpegbuf);
    if (ret == 0) {
        LOGD("%s(%d): Jpeg buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
            mJpegBufferInfo->mPhyBaseAddr,mJpegBufferInfo->mVirBaseAddr,mJpegBufferInfo->mBufferSizes);
    } else {
        LOGE("%s(%d): Jpeg buffer alloc failed",__FUNCTION__,__LINE__);
    }
    LOG_FUNCTION_NAME_EXIT
	return ret;

 }
int IonDmaMemManager::destroyJpegBuffer()
{
	 LOG_FUNCTION_NAME
	 Mutex::Autolock lock(mLock);
	 destroyIonBuffer(JPEGBUFFER);
	 LOG_FUNCTION_NAME_EXIT
	 return 0;

}
int IonDmaMemManager::flushCacheMem(buffer_type_enum buftype,unsigned int offset, unsigned int len)
{
    Mutex::Autolock lock(mLock);

    return 0;
}


#endif


/******************ION BUFFER END*******************/

/*****************pmem buffer start*******************/
#if (CONFIG_CAMERA_MEM == CAMERA_MEM_PMEM)
PmemManager::PmemManager(char* devpath)
			:MemManagerBase(),
			mPmemFd(-1),
			mPmemSize(0),
			mPmemHeapPhyBase(0),
			mMemHeap(NULL),
			mMemHeapPmem(NULL),
			mJpegBuffer(NULL),
			mRawBuffer(NULL),
			mPreviewBuffer(NULL)
{
    initPmem(devpath);
}
PmemManager::~PmemManager()
{
	deinitPmem();
}
int PmemManager::initPmem(char* devpath)
{
	int pmem_fd;
	struct pmem_region sub;
	int err = 0;
	Mutex::Autolock lock(mLock);
    
	if(mPmemFd > 0) {
		LOGD("%s(%d): PMEM has been initialized",__FUNCTION__,__LINE__);
		return err;
	}
    
	pmem_fd = open(devpath, O_RDWR);
	if (pmem_fd < 0) {
		LOGE("%s(%d): Open the PMEM device(%s): %s",__FUNCTION__,__LINE__, devpath, strerror(errno));
        err = -1;
        goto exit;
	}

	ioctl(pmem_fd, PMEM_GET_TOTAL_SIZE, &sub);
	mPmemSize = sub.len;

	if (pmem_fd > 0) {
		close(pmem_fd);
		pmem_fd = -1;
	}

	mMemHeap = new MemoryHeapBase(devpath,mPmemSize,0);
	mPmemFd = mMemHeap->getHeapID();
	if (mPmemFd < 0) {
		LOGE("%s(%d): allocate mMemHeap from %s failed",__FUNCTION__,__LINE__,devpath);
		err = -1;		
		goto exit;
	}

	if (ioctl(mPmemFd,PMEM_GET_PHYS, &sub)) {
		LOGE("%s(%d): Obtain %s physical address failed",__FUNCTION__,__LINE__,devpath);
		err = -1;
        goto exit;
	} else {
		mPmemHeapPhyBase = sub.offset;
	}

	mMemHeapPmem = new MemoryHeapPmem(mMemHeap,0);

exit:
    if (err < 0) {
        if (mMemHeapPmem != NULL) {
            mMemHeapPmem.clear();
            mMemHeapPmem = NULL;
        }
        
        if (mMemHeap != NULL) {			
			mMemHeap.clear();
			mMemHeap = NULL;
		}
    }
    if (pmem_fd > 0) {
		close(pmem_fd);
		pmem_fd = -1;
	} 
    return err;
}	

int PmemManager::deinitPmem()
{
	Mutex::Autolock lock(mLock);

	if (mMemHeapPmem != NULL) { 		
		mMemHeapPmem.clear();
		mMemHeapPmem = NULL;
	}
	if (mMemHeap != NULL) {		
		mMemHeap.clear();
		mMemHeap = NULL;
	}
	mPmemHeapPhyBase = 0;

	if( mPmemFd > 0 ) {
		close(mPmemFd);
		mPmemFd = -1;
	}
	return 0;
}

int PmemManager::createPreviewBuffer(struct bufferinfo_s* previewbuf)
{
    LOG_FUNCTION_NAME
	int ret =0,i ;
	struct bufferinfo_s* tmp_buf = NULL;
	void * viraddress = NULL;
	int numBufs;
	int frame_size;
	Mutex::Autolock lock(mLock);
    
	if(!previewbuf || mMemHeapPmem == NULL){
		LOGE("%s(%d): Pmem malloc preview buffer failed",__FUNCTION__,__LINE__);
		ret = -1;
		goto null_fail;
	}
    
	numBufs = previewbuf->mNumBffers;
	frame_size = previewbuf->mPerBuffersize;
	previewbuf->mBufferSizes = numBufs*PAGE_ALIGN(frame_size);

    if( mPmemSize < previewbuf->mBufferSizes+mJpegBufferInfo.mBufferSizes+mRawBufferInfo.mBufferSizes){
        LOGE("%s(%d): Pmem is not enough for 0x%x bytes preview buffer!(Pmem:0x%x, Raw:0x%x, Jpeg:0x%x)",
            __FUNCTION__,__LINE__,previewbuf->mBufferSizes,mPmemSize,mRawBufferInfo.mBufferSizes,mJpegBufferInfo.mBufferSizes);
        ret = -1;
        goto null_fail;
    }
    
    mPreviewBuffer = (sp<IMemory>**)malloc(sizeof(sp<IMemory>*)*numBufs);
    if (mPreviewBuffer == NULL) {
        LOGE("%s(%d): mPreviewBuffer malloc failed",__FUNCTION__,__LINE__);
        ret = -1;
        goto null_fail;
    }
    
	for(i = 0; i < numBufs; i++){        
        mPreviewBuffer[i] = (sp<IMemory>*)new (sp<IMemory>);
		*mPreviewBuffer[i] = (static_cast<MemoryHeapPmem*>(mMemHeapPmem.get()))->mapMemory(PAGE_ALIGN(frame_size)*i,PAGE_ALIGN(frame_size));
	}
    
	previewbuf->mPhyBaseAddr = mPmemHeapPhyBase + (*mPreviewBuffer[0])->offset();
	previewbuf->mVirBaseAddr= (unsigned long)(*mPreviewBuffer[0])->pointer();
	previewbuf->mPerBuffersize = PAGE_ALIGN(frame_size);
	mPreviewBufferInfo = *previewbuf;
    LOGD("%s(%d): Preview buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
        mPreviewBufferInfo.mPhyBaseAddr,mPreviewBufferInfo.mVirBaseAddr,mPreviewBufferInfo.mBufferSizes);

    LOG_FUNCTION_NAME_EXIT
null_fail:        
	return ret;
}
int PmemManager::destroyPreviewBuffer()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);
    unsigned int i;

    for(i = 0; i < mPreviewBufferInfo.mNumBffers; i++) {
        if (mPreviewBuffer[i] != NULL) {
            (*mPreviewBuffer[i]).clear();
            delete mPreviewBuffer[i];
            mPreviewBuffer[i] = NULL;
        }
    }
    free((char*)mPreviewBuffer);
    mPreviewBuffer = NULL;
    memset(&mPreviewBufferInfo,0,sizeof(mPreviewBufferInfo));
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int PmemManager::createRawBuffer(struct bufferinfo_s* rawbuf)
{
    LOG_FUNCTION_NAME
    int ret =0;
    int numBufs;
    int frame_size;
    int map_start;
    Mutex::Autolock lock(mLock);

    if(!rawbuf || mMemHeapPmem == NULL ){
        LOGE("%s(%d): Pmem malloc raw buffer failed",__FUNCTION__,__LINE__);
        ret = -1;
        goto null_fail;
    }
    
    numBufs = rawbuf->mNumBffers;
    frame_size = rawbuf->mPerBuffersize;
    rawbuf->mBufferSizes = numBufs*PAGE_ALIGN(frame_size);
    //compute the start address of map
    if( mPmemSize < mPreviewBufferInfo.mBufferSizes + mJpegBufferInfo.mBufferSizes +rawbuf->mBufferSizes){
        LOGE("%s(%d): Pmem is not enough for 0x%x bytes raw buffer!(Pmem:0x%x, Preview:0x%x, Jpeg:0x%x)",
            __FUNCTION__,__LINE__,rawbuf->mBufferSizes,mPmemSize,mPreviewBufferInfo.mBufferSizes,mJpegBufferInfo.mBufferSizes);
        ret = -1;
        goto null_fail;
    } else {
        map_start = mPreviewBufferInfo.mBufferSizes;
    }
    
    mRawBuffer = (static_cast<MemoryHeapPmem*>(mMemHeapPmem.get()))->mapMemory(map_start,PAGE_ALIGN(frame_size)*numBufs);
    rawbuf->mPhyBaseAddr = mPmemHeapPhyBase + mRawBuffer->offset();
    rawbuf->mVirBaseAddr= (unsigned long)mRawBuffer->pointer();
    rawbuf->mPerBuffersize = PAGE_ALIGN(frame_size);
    mRawBufferInfo= *rawbuf;

    LOGD("%s(%d): Raw buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
        mRawBufferInfo.mPhyBaseAddr,mRawBufferInfo.mVirBaseAddr,mRawBufferInfo.mBufferSizes);

    LOG_FUNCTION_NAME_EXIT
    return ret;
null_fail:
    memset(&mRawBufferInfo,0,sizeof(mRawBufferInfo));
    return ret;
}
int PmemManager::destroyRawBuffer()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);

    if (mRawBuffer!= NULL) {        
        mRawBuffer.clear();
        mRawBuffer = NULL;
    }
    memset(&mRawBufferInfo,0,sizeof(mRawBufferInfo));
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int PmemManager::createJpegBuffer(struct bufferinfo_s* jpegbuf)
{
    LOG_FUNCTION_NAME
    int ret =0;
    int numBufs;
    int frame_size;
    int map_start;

    Mutex::Autolock lock(mLock);
    if(!jpegbuf || mMemHeapPmem == NULL ){
        LOGE("%s(%d): Pmem malloc jpeg buffer failed",__FUNCTION__,__LINE__);
        ret = -1;
        goto null_fail;
    }
    
    numBufs = jpegbuf->mNumBffers;
    frame_size = jpegbuf->mPerBuffersize;
    jpegbuf->mBufferSizes = numBufs*PAGE_ALIGN(frame_size);
    
    //compute the start address of map
    if ( mPmemSize < mPreviewBufferInfo.mBufferSizes + mRawBufferInfo.mBufferSizes +jpegbuf->mBufferSizes) {
        LOGE("%s(%d): Pmem is not enough for 0x%x bytes jpeg buffer!(Pmem:0x%x, Preview:0x%x, Raw:0x%x)",
            __FUNCTION__,__LINE__,jpegbuf->mBufferSizes,mPmemSize,mPreviewBufferInfo.mBufferSizes,mRawBufferInfo.mBufferSizes);
        ret = -1;
        goto null_fail;
    } else {
        map_start = mPmemSize - jpegbuf->mBufferSizes;
    }
    mJpegBuffer = (static_cast<MemoryHeapPmem*>(mMemHeapPmem.get()))->mapMemory(map_start,PAGE_ALIGN(frame_size)*numBufs);
    jpegbuf->mPhyBaseAddr = mPmemHeapPhyBase + mJpegBuffer->offset();
    jpegbuf->mVirBaseAddr= (unsigned long)mJpegBuffer->pointer();
    jpegbuf->mPerBuffersize = PAGE_ALIGN(frame_size);
    mJpegBufferInfo= *jpegbuf;

    LOGD("%s(%d): Jpeg buffer information(phy:0x%x vir:0x%x size:0x%x)",__FUNCTION__,__LINE__,
        mJpegBufferInfo.mPhyBaseAddr,mJpegBufferInfo.mVirBaseAddr,mJpegBufferInfo.mBufferSizes);
    LOG_FUNCTION_NAME_EXIT
    return ret;
    
null_fail:
    memset(&mJpegBufferInfo,0,sizeof(mJpegBufferInfo));
    return ret;
}
int PmemManager::destroyJpegBuffer()
{
    LOG_FUNCTION_NAME
    Mutex::Autolock lock(mLock);

    if (mJpegBuffer!= NULL) {
       mJpegBuffer.clear();
       mJpegBuffer = NULL;
    }
    memset(&mJpegBufferInfo,0,sizeof(mJpegBufferInfo));
    LOG_FUNCTION_NAME_EXIT
    return 0;
}
int PmemManager::flushCacheMem(buffer_type_enum buftype,unsigned int offset, unsigned int len)
{
	struct bufferinfo_s* tmpbuf =NULL;
	int ret = 0;
	struct pmem_region region;
	sp<IMemory> tmpmem;
    unsigned int i;
	Mutex::Autolock lock(mLock);    

	switch(buftype)
	{
		case PREVIEWBUFFER:
			tmpmem = (*mPreviewBuffer[0]);
			break;
		case RAWBUFFER:
			tmpmem =mRawBuffer;
            
			break;
		case JPEGBUFFER:
			tmpmem =mJpegBuffer;
			break;
		default:
			break;
	}
    region.offset = tmpmem->offset()+offset;
	region.len = len;
	ret = ioctl(mPmemFd,PMEM_CACHE_FLUSH, &region);
    
    return ret;
}
/******************* pmem buffer end*****************/
#endif
}
