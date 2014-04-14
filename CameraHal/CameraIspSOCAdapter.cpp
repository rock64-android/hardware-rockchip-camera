#include "CameraIspAdapter.h"

namespace android{
#define LOG_TAG "CameraHal_CameraIspSOCAdapter"
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

/*
 * zyh neon optimize
 */
#define HAVE_ARM_NEON 1

void CameraIspSOCAdapter::setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h)
{
    CamEngineWindow_t dcWin;
    dcWin.width = width_sensor*2;
    dcWin.height = height_sensor;
    dcWin.hOffset = 0;
    dcWin.vOffset = 0;
    m_camDevice->previewSetup_ex( dcWin, width_sensor*2, height_sensor,
                                CAMERIC_MI_DATAMODE_RAW12,CAMERIC_MI_DATASTORAGE_INTERLEAVED,(bool_t)false);
}

//for soc camera test
extern "C" void arm_isp_yuyv_12bit_to_8bit (int src_w, int src_h,char *srcbuf,uint32_t ycSequence){
    int  *srcint;
    int i = 0;
    int y_size = 0;
    unsigned int *dst_buf,*debug_buf;
    unsigned int tmp = 0;
    //for test 

    y_size = src_w*src_h;
    srcint = ( int*)srcbuf;
    dst_buf = ( int*)srcbuf;
    debug_buf = ( int*)srcbuf;
#if HAVE_ARM_NEON
    /*LOGE("---------arm neon %s-----------",__FUNCTION__);
    for(i=0;i<16;i++) {
           LOGE("before:0x%x,",*(debug_buf+i));
    }*/
    if(ycSequence == ISI_YCSEQ_CBYCRY){
		asm volatile (
				"   pld [%[src], %[src_stride], lsl #2]                         \n\t"
				"   cmp %[n], #8                                                \n\t"
				"   blt 5f                                                      \n\t"
				"0: @ 16 pixel swap                                             \n\t"
				"   vld2.32 {q0,q1} , [%[src]]!  @ q0 = dat0 q1 = dat1          \n\t"
				"   vshr.u32 q0,q0,#6            @ now q0  -> y1 00 v0 00       \n\t"
				"   vshr.u32 q1,q1,#6            @ now q1  -> y0 00 u0 00       \n\t"
				"   vswp q0, q1                  @ now q0 = q1 q1 = q0          \n\t"
				"   vuzp.u8 q0,q4                @ now d0 = y0u0.. d8  = 00..   \n\t"
				"   vuzp.u8 q1,q5                @ now d2 = y1v0.. d10 = 00..   \n\t"
				"   vst2.16 {d0,d2},[%[dst_buf]]!@ now q0  -> dst               \n\t"
				"   sub %[n], %[n], #8                                          \n\t"
				"   cmp %[n], #8                                                \n\t"
				"   bge 0b                                                      \n\t"
				"5: @ end                                                       \n\t"
				: [dst_buf] "+r" (dst_buf),[src] "+r" (srcint), [n] "+r" (y_size)
				: [src_stride] "r" (y_size)
				: "cc", "memory", "q0", "q1", "q2","q3"
				);
		/*for(i=0;i<16;i++) {
			   LOGE("after:0x%x,",*(debug_buf+i));
		}*/
    }
#else
    for(i=0;i<(y_size>>1);i++) {

        if(ycSequence == ISI_YCSEQ_CBYCRY){
           //dst : YUYV
           *dst_buf++= (((*(srcint+1) >> 6) & 0xff ) << 0)| /* Y0 */
                        ((((*(srcint+1) >> 22) & 0xff)) << 8) | /* U*/
                        (((*(srcint) >> 6) & 0xff) << 16) | /*Y1*/
                        ((((*(srcint) >> 22) & 0xff)) << 24) /*V*/
                        ; 
        }
       srcint += 2;

    }
#endif
}

void CameraIspSOCAdapter::bufferCb( MediaBuffer_t* pMediaBuffer )
{
    static int writeoneframe = 0;
    uint32_t y_addr,uv_addr;
    void* y_addr_vir = NULL,*uv_addr_vir = NULL ;
    int width,height;
    int fmt = 0;

	Mutex::Autolock lock(mLock);
    // get & check buffer meta data
    PicBufMetaData_t *pPicBufMetaData = (PicBufMetaData_t *)(pMediaBuffer->pMetaData);
    HalHandle_t  tmpHandle = m_camDevice->getHalHandle();
    //
    if(pPicBufMetaData->Type == PIC_BUF_TYPE_RAW16){
                //get sensor fmt
                //convert to yuyv 8 bit
                fmt = V4L2_PIX_FMT_YUYV;
                y_addr = (uint32_t)(pPicBufMetaData->Data.raw.pBuffer );
                width = pPicBufMetaData->Data.raw.PicWidthPixel >> 1;
                height = pPicBufMetaData->Data.raw.PicHeightPixel;
                HalMapMemory( tmpHandle, y_addr, 100, HAL_MAPMEM_READWRITE, &y_addr_vir );
                m_camDevice->getYCSequence();
                arm_isp_yuyv_12bit_to_8bit(width,height,y_addr_vir,m_camDevice->getYCSequence());
      }else{
           LOGE("not support this type(%dx%d)  ,just support  yuv20 now",width,height);
           return;
    }
    

    if ( pMediaBuffer->pNext != NULL )
    {
        MediaBufLockBuffer( (MediaBuffer_t*)pMediaBuffer->pNext );
    }
#if 1
	//need to display ?
	if(mRefDisplayAdapter->isNeedSendToDisplay()){  
	    MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
      }
      //add to vector
      tmpFrame->frame_index = (int)tmpFrame; 
      tmpFrame->phy_addr = (int)y_addr;
      tmpFrame->frame_width = width;
      tmpFrame->frame_height= height;
      tmpFrame->vir_addr = y_addr_vir;
      tmpFrame->frame_fmt = fmt;
      mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      mRefDisplayAdapter->notifyNewFrame(tmpFrame);
    }

	//video enc ?
	if(mRefEventNotifier->isNeedSendToVideo()){
	    MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
      }
      //add to vector
      tmpFrame->frame_index = (int)tmpFrame; 
      tmpFrame->phy_addr = (int)y_addr;
      tmpFrame->frame_width = width;
      tmpFrame->frame_height= height;
      tmpFrame->vir_addr = y_addr_vir;
      tmpFrame->frame_fmt = fmt;
      mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
      mRefEventNotifier->notifyNewVideoFrame(tmpFrame);		
	}
	//picture ?
	if(mRefEventNotifier->isNeedSendToPicture()){
		MediaBufLockBuffer( pMediaBuffer );
		//new frames
		FramInfo_s *tmpFrame=(FramInfo_s *)malloc(sizeof(FramInfo_s));
		if(!tmpFrame){
			MediaBufUnlockBuffer( pMediaBuffer );
			return;
		}
	  //add to vector
	  tmpFrame->frame_index = (int)tmpFrame; 
	  tmpFrame->phy_addr = (int)y_addr;
	  tmpFrame->frame_width = width;
	  tmpFrame->frame_height= height;
	  tmpFrame->vir_addr = y_addr_vir;
	  tmpFrame->frame_fmt = fmt;
	  mFrameInfoArray.add((void*)tmpFrame,(void*)pMediaBuffer);
	  mRefEventNotifier->notifyNewPicFrame(tmpFrame);	
	}

	//preview data callback ?
	if(mRefEventNotifier->isNeedSendToDataCB()){
	}
	#endif
}

}

