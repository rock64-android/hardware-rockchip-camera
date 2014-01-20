/*
*Author: zyc@rock-chips.com
*/
#define CAMERA_MEM_PMEM 0
#define CAMERA_MEM_ION  1
/* 
*NOTE: 
*   configuration macro 
*      
*/
#ifdef  TARGET_RK30
#define CONFIG_CAMERA_MEM               CAMERA_MEM_ION
#else
#define CONFIG_CAMERA_MEM               CAMERA_MEM_PMEM
#endif
#if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)
#include <ion/IonAlloc.h>
#elif (CONFIG_CAMERA_MEM == CAMERA_MEM_PMEM)
#ifdef HAVE_ANDROID_OS 
#include <linux/android_pmem.h>
#include <binder/MemoryHeapPmem.h>
#endif
#endif
#include <binder/IMemory.h>
namespace android {

/*****************for unified memory manager  start*******************/
enum buffer_type_enum{
	PREVIEWBUFFER,
	RAWBUFFER,
	JPEGBUFFER,

};
struct bufferinfo_s{
	unsigned int mNumBffers; //invaild if this value is 0
	size_t mPerBuffersize;
	size_t mBufferSizes;
	unsigned int mPhyBaseAddr;
	unsigned int mVirBaseAddr;
	buffer_type_enum mBufType;
};

typedef enum buffer_addr_e {
    buffer_addr_phy,
    buffer_addr_vir    
}buffer_addr_t;

class MemManagerBase{
public :
	MemManagerBase();
	virtual ~MemManagerBase();
	virtual int createPreviewBuffer(struct bufferinfo_s* previewbuf) = 0;
	virtual int createRawBuffer(struct bufferinfo_s* rawbuf) = 0;
	virtual int createJpegBuffer(struct bufferinfo_s* jpegbuf) = 0;

	virtual int destroyPreviewBuffer() = 0;
	virtual int destroyRawBuffer() = 0;
	virtual int destroyJpegBuffer() = 0;
	virtual int flushCacheMem(buffer_type_enum buftype,unsigned int buf_idx) = 0;
	struct bufferinfo_s* getPreviewBufInfo(){
		return mPreviewBufferInfo;}
	struct bufferinfo_s* getRawBufInfo(){
		return mRawBufferInfo;}
	struct bufferinfo_s* getJpegBufInfo(){
		return mJpegBufferInfo;}
    unsigned int getBufferAddr(enum buffer_type_enum buf_type, unsigned int buf_idx, buffer_addr_t addr_type);
    int dump();
protected:
	struct bufferinfo_s *mPreviewBufferInfo;
	struct bufferinfo_s *mRawBufferInfo;
	struct bufferinfo_s *mJpegBufferInfo;
	mutable Mutex mLock;
};
#if (CONFIG_CAMERA_MEM == CAMERA_MEM_PMEM)
class PmemManager:public MemManagerBase{
	public :
		PmemManager(char* devpath);
		~PmemManager();
		virtual int createPreviewBuffer(struct bufferinfo_s* previewbuf);
		virtual int createRawBuffer(struct bufferinfo_s* rawbuf);
		virtual int createJpegBuffer(struct bufferinfo_s* jpegbuf);

		virtual int destroyPreviewBuffer();
		virtual int destroyRawBuffer();
		virtual int destroyJpegBuffer();
		virtual int flushCacheMem(buffer_type_enum buftype,unsigned int buf_idx);
		int initPmem(char* devpath);
		int deinitPmem();
	private:
		int mPmemFd;
		unsigned int mPmemSize; 
		unsigned int mPmemHeapPhyBase;
		sp<MemoryHeapBase> mMemHeap;
		sp<MemoryHeapBase> mMemHeapPmem;
		sp<IMemory> mJpegBuffer; 
		sp<IMemory> mRawBuffer;  
        sp<IMemory> **mPreviewBuffer;
};
#endif
#if (CONFIG_CAMERA_MEM == CAMERA_MEM_ION)
class IonMemManager:public MemManagerBase{
	public :
		IonMemManager();
		~IonMemManager();

		virtual int createPreviewBuffer(struct bufferinfo_s* previewbuf);
		virtual int createRawBuffer(struct bufferinfo_s* rawbuf);
		virtual int createJpegBuffer(struct bufferinfo_s* jpegbuf);

		virtual int destroyPreviewBuffer();
		virtual int destroyRawBuffer();
		virtual int destroyJpegBuffer();
		virtual int flushCacheMem(buffer_type_enum buftype,unsigned int buf_idx);
	private:
		int createIonBuffer(struct bufferinfo_s* ionbuf);
		void destroyIonBuffer(buffer_type_enum buftype);
		ion_buffer_t *mPreviewData;
		ion_buffer_t *mRawData;
		ion_buffer_t *mJpegData;
		IonAlloc *mIonMemMgr;
};
#endif

/*****************for unified memory manager  end*******************/
}
