/*
*Author: zyc@rock-chips.co
*/
#include <sys/stat.h>
#include <unistd.h>
#include <utils/CallStack.h>
#include "CameraHal.h"


namespace android {
#define LOG_TAG "CameraHal_Mem"

static volatile int32_t gLogLevel = 0;

#ifdef ALOGD_IF
#define LOG1(...) ALOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) ALOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#else
#define LOG1(...) LOGD_IF(gLogLevel >= 1, __VA_ARGS__);
#define LOG2(...) LOGD_IF(gLogLevel >= 2, __VA_ARGS__);
#endif

#define LOG_FUNCTION_NAME           LOG1("%s Enter", __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT      LOG1("%s Exit ", __FUNCTION__);

/******************ION BUFFER START*******************/
MemManagerBase::MemManagerBase()
{
	memset(&mPreviewBufferInfo,0,sizeof(mPreviewBufferInfo));
	memset(&mRawBufferInfo,0,sizeof(mRawBufferInfo));
	memset(&mJpegBufferInfo,0,sizeof(mJpegBufferInfo));
}
MemManagerBase::~MemManagerBase()
{
	memset(&mPreviewBufferInfo,0,sizeof(mPreviewBufferInfo));
	memset(&mRawBufferInfo,0,sizeof(mRawBufferInfo));
	memset(&mJpegBufferInfo,0,sizeof(mJpegBufferInfo));
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
        default:
            LOGD("%s(%d): Buffer type(0x%x) is invaildate",__FUNCTION__,__LINE__,buf_type);
            goto getVirAddr_end;
    }

    if (buf_idx > buf_info->mNumBffers) {
        LOGE("%s(%d): Buffer index(0x%x) is invalidate, Total buffer is 0x%x",__FUNCTION__,__LINE__,
            buf_idx,buf_info->mNumBffers);
        goto getVirAddr_end;
    }

    buf_info = buf_info + buf_idx;
    
    if (addr_type == buffer_addr_vir) {
        addr = buf_info->mVirBaseAddr;
    } else if (addr_type == buffer_addr_phy) {
        addr = buf_info->mPhyBaseAddr;
    }
getVirAddr_end:
    return addr;
}

int MemManagerBase::dump()
{
    if (gLogLevel < 2) 
        android_atomic_inc(&gLogLevel);
    else
        android_atomic_write(0,&gLogLevel);

    LOGD("Set %s log level to %d",LOG_TAG,gLogLevel);
    return 0;
}

#if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)

IonMemManager::IonMemManager()
			     :MemManagerBase(),
			      mPreviewData(NULL),
			      mRawData(NULL),
			      mJpegData(NULL),
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

        if (mPreviewBufferInfo) {
            free(mPreviewBufferInfo);
            mPreviewBufferInfo = NULL;
        }
	}
	if(mRawData) {
		destroyRawBuffer();
		free(mRawData);
        mRawData = NULL;

        if (mRawBufferInfo) {
            free(mRawBufferInfo);
            mRawBufferInfo = NULL;
        }
	}
	if(mJpegData) {
		destroyJpegBuffer();
		free(mJpegData);
        mJpegData = NULL;

        if (mJpegBufferInfo) {
            free(mJpegBufferInfo);
            mJpegBufferInfo = NULL;
        }
	}
	if(mIonMemMgr)
		delete mIonMemMgr;
	mIonMemMgr = NULL;

}

int IonMemManager::createIonBuffer(struct bufferinfo_s* ionbuf)
{
	int ret =0,i;
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
	ionbuf->mBufferSizes = PAGE_ALIGN(frame_size);
	switch(ionbuf->mBufType)
	{
		case PREVIEWBUFFER:
			tmpalloc = mPreviewData;
			tmp_buf = mPreviewBufferInfo = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s));
            if (tmp_buf == NULL) {
                LOGE("%s(%d): mPreviewBufferInfo malloc failed!",__FUNCTION__,__LINE__);
                ret = -1;
                goto null_fail;
            }
			break;
		case RAWBUFFER:
			tmpalloc = mRawData;
			tmp_buf = mRawBufferInfo = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s));
            if (tmp_buf == NULL) {
                LOGE("%s(%d): mRawBufferInfo malloc failed!",__FUNCTION__,__LINE__);
                ret = -1;
                goto null_fail;
            }
			break;
		case JPEGBUFFER:
			tmpalloc = mJpegData;
			tmp_buf = mJpegBufferInfo = (struct bufferinfo_s*)malloc(numBufs*sizeof(struct bufferinfo_s));
            if (tmp_buf == NULL) {
                LOGE("%s(%d): mJpegBufferInfo malloc failed!",__FUNCTION__,__LINE__);
                ret = -1;
                goto null_fail;
            }
			break;
        default:
            goto null_fail;
    }
    
    for (i=0; i<numBufs; i++) {
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
        tmpalloc++;
        tmp_buf++;
    }

    return 0;

alloc_fail:
null_fail:
    if (tmp_buf != NULL)
        memset(tmp_buf,0,sizeof(struct bufferinfo_s));
    if (tmp_buf != NULL)
        memset(tmpalloc,0,sizeof(ion_buffer_t));
    return ret;
}

void IonMemManager::destroyIonBuffer(buffer_type_enum buftype)
{
    int i,numBufs;
	struct ion_buffer_t* tmpalloc = NULL;
    struct bufferinfo_s *bufinfo;
    
    
	switch(buftype)
	{
		case PREVIEWBUFFER:
            tmpalloc = mPreviewData;
            bufinfo = mPreviewBufferInfo;
			break;
		case RAWBUFFER:
			tmpalloc = mRawData;
            bufinfo = mRawBufferInfo;
			break;
		case JPEGBUFFER:
			tmpalloc = mJpegData;
			bufinfo = mJpegBufferInfo;
			break;

        default:
		   	LOGE("%s(%d): buffer type is wrong !",__FUNCTION__,__LINE__);
            break;
	}

    if ((tmpalloc == NULL) || (bufinfo == NULL)) {
	    LOGE("%s(%d): tmpalloc %p, bufinfo %p is error",__FUNCTION__,__LINE__,
           tmpalloc,bufinfo);
    } else {        
        for (i=0; i<bufinfo->mNumBffers; i++) {    			
			if(tmpalloc && tmpalloc->virt) {
				LOGD("%s(%d): free preview buffer %d success!",__FUNCTION__,__LINE__,i);
				mIonMemMgr->free(*tmpalloc);
				memset(tmpalloc,0,sizeof(ion_buffer_t));
 			}  
            tmpalloc++;
        }
    }
}

int IonMemManager::createPreviewBuffer(struct bufferinfo_s* previewbuf)
{
	LOG_FUNCTION_NAME
    int ret;
	Mutex::Autolock lock(mLock);
    
	if(previewbuf->mBufType != PREVIEWBUFFER)
		LOGE("%s(%d): the type is not PREVIEWBUFFER",__FUNCTION__,__LINE__);
    
	if(!mPreviewData) {
		mPreviewData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t)*previewbuf->mNumBffers);
	} else if(mPreviewData->virt) {
		LOGD("%s(%d): FREE the preview buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyPreviewBuffer();
	}
    
	memset(mPreviewData,0,sizeof(ion_buffer_t)*previewbuf->mNumBffers);
    
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
		mRawData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t)*rawbuf->mNumBffers);
	} else if(mRawData->virt) {
		LOGD("%s(%d): FREE the raw buffer alloced before firstly",__FUNCTION__,__LINE__);
		destroyRawBuffer();
	}
	memset(mRawData,0,sizeof(ion_buffer_t)*rawbuf->mNumBffers);

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
        mJpegData = (ion_buffer_t*)malloc(sizeof(ion_buffer_t)*jpegbuf->mNumBffers);
    } else if(mJpegData->virt) {
        LOGD("%s(%d): FREE the jpeg buffer alloced before firstly",__FUNCTION__,__LINE__);
        destroyJpegBuffer();
    }
    memset(mJpegData,0,sizeof(ion_buffer_t)*jpegbuf->mNumBffers);
    
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
int IonMemManager::destroyJpegBuffer()
{
	 LOG_FUNCTION_NAME
	 Mutex::Autolock lock(mLock);
	 destroyIonBuffer(JPEGBUFFER);
	 LOG_FUNCTION_NAME_EXIT
	 return 0;

}
int IonMemManager::flushCacheMem(buffer_type_enum buftype,unsigned int buf_idx)
{
    int ret = 0;
    Mutex::Autolock lock(mLock);
    ion_buffer_t data;
    
    switch(buftype)
	{
		case PREVIEWBUFFER:
            if (buf_idx > mPreviewBufferInfo->mNumBffers) {
                LOGE("%s(%d): Buffer index %d is invalidate",__FUNCTION__,__LINE__, buf_idx);
                ret = -1;
                goto end;
            }
			data = *(mPreviewData + buf_idx);
			break;
		case RAWBUFFER:
            if (buf_idx > mRawBufferInfo->mNumBffers) {
                LOGE("%s(%d): Buffer index %d is invalidate",__FUNCTION__,__LINE__, buf_idx);
                ret = -1;
                goto end;
            }
			data = *(mRawData + buf_idx);	
			break;
		case JPEGBUFFER:
			data = *mJpegData;
			break;
		default:
			break;
	}    
    mIonMemMgr->cache_op(data, ION_FLUSH_CACHE);
end:
    return ret;
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
int PmemManager::flushCacheMem(buffer_type_enum buftype,unsigned int buf_idx)
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
