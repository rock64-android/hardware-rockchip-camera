#ifndef ANDROID_HARDWARE_CAMERA_ISP_HARDWARE_H
#define ANDROID_HARDWARE_CAMERA_ISP_HARDWARE_H

//usb camera adapter
#include "CameraHal.h"
#include "cam_api/camdevice.h"
#include "oslayer/oslayer.h"
#include <string>
#include <utils/KeyedVector.h>



namespace android{


class CameraIspAdapter: public CameraAdapter,public BufferCb
{
public:
	static int preview_frame_inval;
    static int DEFAULTPREVIEWWIDTH;
    static int DEFAULTPREVIEWHEIGHT;
    CameraIspAdapter(int cameraId);
    virtual ~CameraIspAdapter();
    virtual status_t startPreview(int preview_w,int preview_h,int w, int h, int fmt,bool is_capture);
    virtual status_t stopPreview();
    virtual int setParameters(const CameraParameters &params_set);
    virtual void initDefaultParameters(int camFd);
    virtual status_t autoFocus();
    virtual int getCurPreviewState(int *drv_w,int *drv_h);
    void AfpsResChangeCb();
    virtual void bufferCb( MediaBuffer_t* pMediaBuffer );

    virtual void setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h);

	virtual void dump(int cameraId);
    
private:
    //talk to driver
    virtual int cameraCreate(int cameraId);
    virtual int cameraDestroy();
    virtual int adapterReturnFrame(int index,int cmd);


    //for isp
    void setScenarioMode(CamEngineModeType_t newScenarioMode);

    void setSensorItf(int newSensorItf);
    void enableAfps( bool enable = false );

    void loadSensor(int cameraId =-1 );
    void loadCalibData(const char* fileName = NULL);
    void openImage( const char* fileName = NULL);

    bool connectCamera();
    void disconnectCamera();

    int start();
    int pause();
    int stop();

    int afListenerThread(void);
    int cameraConfig(const CameraParameters &tmpparams,bool isInit);
protected:
    CamDevice       *m_camDevice;
    KeyedVector<void *, void *> mFrameInfoArray;
    Mutex  mFrameArrayLock;     
    void clearFrameArray();
	mutable Mutex mLock;

    std::string mSensorDriverFile[3];
    int mSensorItfCur;


    class CameraAfThread :public Thread
    {
        //deque 到帧后根据需要分发给DisplayAdapter类及EventNotifier类。
        CameraIspAdapter* mCameraAdapter;
    public:
        CameraAfThread(CameraIspAdapter* adapter)
            : Thread(false), mCameraAdapter(adapter) { }

        virtual bool threadLoop() {
            mCameraAdapter->afListenerThread();

            return false;
        }
    };
    
    CamEngineAfEvtQue_t  mAfListenerQue; 
    sp<CameraAfThread>   mAfListenerThread;
    
};

class CameraIspSOCAdapter: public CameraIspAdapter
{
public:

    CameraIspSOCAdapter(int cameraId);
    virtual ~CameraIspSOCAdapter();
#if 0
    virtual int setParameters(const CameraParameters &params_set);
    virtual void initDefaultParameters()
    {
        mParameters.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "10,15,30");  

    };
   virtual status_t autoFocus();
#endif
    virtual void setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h);
    virtual void bufferCb( MediaBuffer_t* pMediaBuffer );

private:
    bool    mIs10bit0To0;

};



}
#endif
