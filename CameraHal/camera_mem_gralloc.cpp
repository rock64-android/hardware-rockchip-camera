#include <utils/threads.h>
#include <utils/Log.h>

#include "camera_mem.h"
#include "camsys_head.h"
#include "hal/hal_api.h"
#include "ebase/types.h"
#include "hal/hal_mockup.h"
#include "CameraHal_Tracer.h"

using namespace android;

#if 0 //gralloc device alloc memory directly 
#include <gralloc_drm.h>
#include <gralloc_drm_handle.h>
static gralloc_module_t *g_mGrallocModule = NULL;
static struct alloc_device_t *g_mGrallocAllocDev = NULL;
static int g_alloc_open_cnt = 0;

typedef struct camera_gralloc_dev_s
{
    gralloc_module_t *mGrallocModule;
    struct alloc_device_t *mGrallocAllocDev;
	int iommu_enabled;
	int phy_continuous;
}camera_gralloc_dev_t;

static cam_mem_handle_t*  cam_mem_gralloc_ops_init(int iommu_enabled,int phy_continuos)
{
    int ret = 0;
	gralloc_module_t *mGrallocModule = NULL;
    struct alloc_device_t *mGrallocAllocDev = NULL;
	camera_gralloc_dev_t* gralloc_dev = NULL;
	
	cam_mem_handle_t* handle = (cam_mem_handle_t*)malloc(sizeof(cam_mem_handle_t));
	if (!handle) {
		TRACE_E("%s:can't alloc handle!",__FUNCTION__);
		goto init_error;
	}
	
	gralloc_dev = (camera_gralloc_dev_t*)malloc(sizeof(camera_gralloc_dev_t));
	if (!gralloc_dev) {
        TRACE_E("%s: Unable to alloc camera_gralloc_dev_t", __func__);
		goto init_error;
	}
	if (g_alloc_open_cnt++ == 0) {
	    ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&mGrallocModule);
	    if (ret < 0) {
	        TRACE_E("%s: Unable to get gralloc module (error %d)", __func__, ret);
			goto init_error;
	    } else {
	        ret = gralloc_open((const struct hw_module_t*)mGrallocModule, &mGrallocAllocDev);
	        if (ret < 0) {
	            TRACE_E("%s: Unable to open gralloc alloc device (error %d)", __func__, ret);
				goto init_error;
	        }
	    }
		g_mGrallocModule = mGrallocModule;
		g_mGrallocAllocDev = mGrallocAllocDev;
	}
	mGrallocModule = g_mGrallocModule ;
	mGrallocAllocDev = g_mGrallocAllocDev;
	gralloc_dev->iommu_enabled = iommu_enabled;
	gralloc_dev->phy_continuous = phy_continuos;
	gralloc_dev->mGrallocAllocDev = mGrallocAllocDev;
	gralloc_dev->mGrallocModule = mGrallocModule;
	handle->mem_type = CAM_MEM_TYPE_GRALLOC;
	handle->iommu_enabled = iommu_enabled;
	handle->phy_continuos = phy_continuos;
	handle->priv = (void*)gralloc_dev;
	
	return handle;
init_error:
	if (gralloc_dev)
		free(gralloc_dev);
	if (!handle)
		free(handle);
	return NULL;
}

//alloc
static cam_mem_info_t* cam_mem_gralloc_ops_alloc(cam_mem_handle_t* handle,size_t size)
{
    int ret;
    buffer_handle_t buffHandle;
    int stride;
    unsigned int grallocFlags = 0;
    unsigned int halPixFmt;
	void* mem_addr = NULL;
	
	camera_gralloc_dev_t* gralloc_dev = NULL;
	cam_mem_info_t* mem = NULL;

	if (!handle) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return NULL; 
	}
	gralloc_dev = (camera_gralloc_dev_t*)(handle->priv);
	
	mem = (cam_mem_info_t*)malloc(sizeof(cam_mem_info_t));
	if (!mem) {
		TRACE_E("%s:can't alloc cam_mem_info_t!",__FUNCTION__); 
		goto  error_alloc;
	} 
    halPixFmt = HAL_PIXEL_FORMAT_RGB_565;
	// use rgb565 format to alloce buffer size, so size should be divided 2

    grallocFlags |= GRALLOC_USAGE_HW_CAMERA_WRITE;
    grallocFlags |= GRALLOC_USAGE_HW_CAMERA_READ;
    grallocFlags |= GRALLOC_USAGE_SW_READ_OFTEN; //GRALLOC_USAGE_SW_READ_OFTEN is used by RK gralloc, GRALLOC_USAGE_PRIVATE_3 is used by INTEL
	grallocFlags |= GRALLOC_USAGE_PRIVATE_1;
    ret = gralloc_dev->mGrallocAllocDev->alloc(gralloc_dev->mGrallocAllocDev,
			                                  size / 2 ,
			                                  1,
			                                  halPixFmt,
			                                  grallocFlags,
			                                  (buffer_handle_t*)&buffHandle,
			                                  &stride);

    if (ret < 0) {
        TRACE_E("%s: gralloc buffer allocation failed (error %d)", __func__, ret);
		goto  error_alloc;
    }

    if (gralloc_dev->mGrallocModule->perform)
        gralloc_dev->mGrallocModule->perform(
        	gralloc_dev->mGrallocModule, 
        	GRALLOC_MODULE_PERFORM_GET_HADNLE_PRIME_FD, 
        	buffHandle, &mem->fd);
	
    if (gralloc_dev->mGrallocModule->lock)
        gralloc_dev->mGrallocModule->lock(
        	gralloc_dev->mGrallocModule, 
        	(buffer_handle_t)buffHandle, 
        	grallocFlags, 
        	0, 0, 0, 0, 
        	&mem_addr);
	mem->vir_addr = (unsigned long)mem_addr;
	mem->handlle = handle;
	//mem->fd = buffHandle->share_fd;
	mem->iommu_maped = 0;
	mem->mmu_addr = 0;
	mem->phy_addr = 0;
	//mem->vir_addr = buffHandle->base;
	mem->size = size;
	mem->priv = (void*)(buffHandle);

	TRACE_D(0,"alloc sucess,mem %p, vir_addr 0x%lx,fd %d ,buffHandle %p",mem,mem_addr,mem->fd,buffHandle);
	return mem;
error_alloc:
	if (!mem)
		free(mem);
	return NULL;
}

//free
static int cam_mem_gralloc_ops_free(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{

	int ret = 0;
	camera_gralloc_dev_t* gralloc_dev = NULL;
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}

	if (mem->iommu_maped) {
		TRACE_E("%s:ion mem is mmumaped,should be unmapped firstly!",__FUNCTION__); 
		return -1; 
	}
	gralloc_dev = (camera_gralloc_dev_t*)(handle->priv);
	gralloc_dev->mGrallocModule->unlock(gralloc_dev->mGrallocModule, (buffer_handle_t)(mem->priv));
	ret = gralloc_dev->mGrallocAllocDev->free(gralloc_dev->mGrallocAllocDev,(buffer_handle_t)(mem->priv));
	free(mem);
	return ret;
}

static int cam_mem_gralloc_ops_iommu_map(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
    int err = 0;
	static int i = 0;
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}
	TRACE_D(0,"mem %p, fd %d",mem,mem->fd);

	//get iommu or phy addr
	if(1/*handle->iommu_enabled*/){
		camsys_iommu_t *iommu;
		camsys_sysctrl_t sysctrl;
		
		memset(&sysctrl,0,sizeof(camsys_sysctrl_t));
		iommu = (camsys_iommu_t *)(sysctrl.rev);
		//use this field to identify wheather gralloc or ion iommu
		iommu->client_fd = -1;
		iommu->map_fd = mem->fd;
		sysctrl.dev_mask = HAL_DEVID_MARVIN;
		sysctrl.on = 1;
		sysctrl.ops = CamSys_IOMMU;
		err = ioctl(handle->camsys_fd, CAMSYS_SYSCTRL, &sysctrl);
		if((err < 0) /*|| (iommu->linear_addr < 0)*/){
			TRACE_E("gralloc IOMMU map failed,err %d linear_addr %ul,\n",err,iommu->linear_addr);
			return -1;
		}
		mem->mmu_addr = iommu->linear_addr;
		mem->iommu_maped = 1;
		TRACE_D(1,"%s:gralloc iommu addr (0x%x) error %d",__FUNCTION__,iommu->linear_addr,err);
	} else
		err = -1;
	
	return err;
}

static int cam_mem_gralloc_ops_iommu_unmap(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
    int err = 0;
	
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}
    if(1/*handle->iommu_enabled*/){
        camsys_iommu_t *iommu;
        camsys_sysctrl_t sysctrl;
        memset(&sysctrl,0,sizeof(camsys_sysctrl_t));
        iommu = (camsys_iommu_t *)(sysctrl.rev);
        iommu->client_fd = -1;
        iommu->map_fd = mem->fd;
        sysctrl.dev_mask = HAL_DEVID_MARVIN;
        sysctrl.on = 0;
        sysctrl.ops = CamSys_IOMMU;
        err = ioctl(handle->camsys_fd, CAMSYS_SYSCTRL, &sysctrl);
        if(err < 0){
            TRACE_E("gralloc IOMMU unmap failed\n");
            return -1;
        }
		mem->iommu_maped = 0;
    } else
    	err = -1;
	return err;
}

//flush cache
static int cam_mem_gralloc_ops_flush_cache(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
	LOGW("%s: havn't been implemented!",__FUNCTION__);
	return 0;
}

//deinit
static int cam_mem_gralloc_ops_deInit(cam_mem_handle_t* handle)
{
    int ret = 0;
	camera_gralloc_dev_t* gralloc_dev = NULL;
	
	if (!handle) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}	
	gralloc_dev = (camera_gralloc_dev_t*)(handle->priv);
	//if (g_alloc_open_cnt > 0)
	//	g_alloc_open_cnt--;
	if (g_alloc_open_cnt == 0) {
		//ret = gralloc_close(gralloc_dev->mGrallocAllocDev);
		//g_mGrallocModule = NULL;
		//g_mGrallocAllocDev = NULL;
	}
	if (ret < 0)
		TRACE_E("%s: gralloc_close failed (error %d)", __func__, ret);
	else {
		free(gralloc_dev);
		free(handle);
	}

	return ret;
}

#else // use GraphicBuffer to alloc memory
#include <ui/GraphicBufferAllocator.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/GraphicBuffer.h>
#if defined(RK_DRM_GRALLOC)
#include <gralloc_drm.h>
#endif

static cam_mem_handle_t*  cam_mem_gralloc_ops_init(int iommu_enabled,unsigned int mem_flag,int phy_continuos)
{
	int ret = 0;
	const hw_module_t *allocMod = NULL;
	gralloc_module_t const* gm; 
	cam_mem_handle_t* handle = NULL;
	ret= hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &allocMod);
	if (ret == 0)
    	gm = reinterpret_cast<gralloc_module_t const *>(allocMod);
	else 
		goto init_error;
    handle = (cam_mem_handle_t*)malloc(sizeof(cam_mem_handle_t));
    if (!handle) {
        TRACE_E("%s:can't alloc handle!",__FUNCTION__);
        goto init_error;
    }
    handle->mem_type = CAM_MEM_TYPE_GRALLOC;
    handle->iommu_enabled = iommu_enabled;
    handle->phy_continuos = phy_continuos;
	handle->flag =  (mem_flag & CAM_MEM_FLAG_HW_WRITE) ? handle->flag |= GRALLOC_USAGE_HW_CAMERA_WRITE :
					(mem_flag & CAM_MEM_FLAG_HW_READ) ? handle->flag |= GRALLOC_USAGE_HW_CAMERA_READ :
					(mem_flag & CAM_MEM_FLAG_SW_WRITE) ? handle->flag |= GRALLOC_USAGE_SW_WRITE_OFTEN :
					(mem_flag & CAM_MEM_FLAG_SW_READ) ? handle->flag |= GRALLOC_USAGE_SW_READ_OFTEN : 0;
    handle->priv = (void*)gm;
    
    return handle;
init_error:
    if (!handle)
        free(handle);
    return NULL;
}

//alloc
static cam_mem_info_t* cam_mem_gralloc_ops_alloc(cam_mem_handle_t* handle,size_t size)
{
	int ret;
	int stride;
	unsigned int grallocFlags = 0;
	unsigned int halPixFmt;
    void* mem_addr = NULL;
    GraphicBuffer* mgraphicbuf;
    cam_mem_info_t* mem = NULL;
	gralloc_module_t* mGrallocModule;

    if (!handle) { 
        TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
        return NULL; 
    }
	mGrallocModule = (gralloc_module_t*)(handle->priv);
    
    mem = (cam_mem_info_t*)malloc(sizeof(cam_mem_info_t));
    if (!mem) {
        TRACE_E("%s:can't alloc cam_mem_info_t!",__FUNCTION__); 
        goto  error_alloc;
    } 
	halPixFmt = HAL_PIXEL_FORMAT_RGB_565;
	// use rgb565 format to alloce buffer size, so size should be divided 2

	grallocFlags = handle->flag;
	mgraphicbuf = new GraphicBuffer(size/2,1,halPixFmt,grallocFlags);
	mgraphicbuf->incStrong(mgraphicbuf);
	if (mgraphicbuf->initCheck()) {
		TRACE_E("GraphicBuffer error : %s\n",strerror(errno));
		goto error_alloc;
	} 

	ret = mgraphicbuf->lock(grallocFlags, (void**)&mem_addr);

	if (ret) {
		TRACE_E("lock buffer error : %s\n",strerror(errno));
		goto lock_error;
	}
	mgraphicbuf->unlock();
	
#if defined(RK_DRM_GRALLOC)
	mGrallocModule->perform(
		mGrallocModule, 
		GRALLOC_MODULE_PERFORM_GET_HADNLE_PRIME_FD, 
		mgraphicbuf->handle, 
		&mem->fd); 
#else
	mem->fd = mgraphicbuf->handle->share_fd;
#endif
    mem->vir_addr = (unsigned long)mem_addr;
    mem->handlle = handle;
    mem->iommu_maped = 0;
    mem->mmu_addr = 0;
    mem->phy_addr = 0;
    mem->size = size;
    mem->priv = (void*)(mgraphicbuf);

    TRACE_D(0,"alloc graphic buffer sucess,mem %p, vir_addr %p,fd %d",mem,mem_addr,mem->fd);
    return mem;
lock_error:
	//delete mgraphicbuf;
	mgraphicbuf->decStrong(mgraphicbuf);
error_alloc:
    if (!mem)
        free(mem);
    return NULL;
}

//free
static int cam_mem_gralloc_ops_free(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{

	int ret = 0;
    GraphicBuffer* mgraphicbuf;
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}

	if (mem->iommu_maped) {
		TRACE_E("%s:ion mem is mmumaped,should be unmapped firstly!",__FUNCTION__); 
		return -1; 
	}
	mgraphicbuf = (GraphicBuffer*)(mem->priv);
	mgraphicbuf->decStrong(mgraphicbuf);
	free(mem);
	return ret;
}

static int cam_mem_gralloc_ops_iommu_map(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
    int err = 0;
	static int i = 0;
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}
	TRACE_D(0,"mem %p, fd %d",mem,mem->fd);

	//get iommu or phy addr
	if(1/*handle->iommu_enabled*/){
		camsys_iommu_t *iommu;
		camsys_sysctrl_t sysctrl;
		
		memset(&sysctrl,0,sizeof(camsys_sysctrl_t));
		iommu = (camsys_iommu_t *)(sysctrl.rev);
		//use this field to identify wheather gralloc or ion iommu
		iommu->client_fd = -1;
		iommu->map_fd = mem->fd;
		sysctrl.dev_mask = HAL_DEVID_MARVIN;
		sysctrl.on = 1;
		sysctrl.ops = CamSys_IOMMU;
		err = ioctl(handle->camsys_fd, CAMSYS_SYSCTRL, &sysctrl);
		if((err < 0) /*|| (iommu->linear_addr < 0)*/){
			TRACE_E("gralloc IOMMU map failed,err %d linear_addr 0x%lx,\n",err,iommu->linear_addr);
			return -1;
		}
		mem->mmu_addr = iommu->linear_addr;
		mem->iommu_maped = 1;
		TRACE_D(1,"%s:gralloc iommu addr (0x%lx) error %d",__FUNCTION__,iommu->linear_addr,err);
	} else
		err = -1;
	
	return err;
}

static int cam_mem_gralloc_ops_iommu_unmap(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
    int err = 0;
	
	if (!handle || !mem) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}
    if(1/*handle->iommu_enabled*/){
        camsys_iommu_t *iommu;
        camsys_sysctrl_t sysctrl;
        memset(&sysctrl,0,sizeof(camsys_sysctrl_t));
        iommu = (camsys_iommu_t *)(sysctrl.rev);
        iommu->client_fd = -1;
        iommu->map_fd = mem->fd;
        sysctrl.dev_mask = HAL_DEVID_MARVIN;
        sysctrl.on = 0;
        sysctrl.ops = CamSys_IOMMU;
        err = ioctl(handle->camsys_fd, CAMSYS_SYSCTRL, &sysctrl);
        if(err < 0){
            TRACE_E("gralloc IOMMU unmap failed\n");
            return -1;
        }
		mem->iommu_maped = 0;
    } else
    	err = -1;
	return err;
}

//flush cache
static int cam_mem_gralloc_ops_flush_cache(cam_mem_handle_t* handle,cam_mem_info_t* mem)
{
    unsigned int grallocFlags = 0;
	int ret = 0;
	void* mem_addr;
	//LOGW("%s: havn't been implemented!",__FUNCTION__);
    GraphicBuffer* mgraphicbuf = (GraphicBuffer*)(mem->priv);
	ret = mgraphicbuf->lock(handle->flag, (void**)&mem_addr);

	if (ret) {
		TRACE_E("lock buffer error : %s\n",strerror(errno));
		return -1;
	}
	mgraphicbuf->unlock();
	return 0;
}

//deinit
static int cam_mem_gralloc_ops_deInit(cam_mem_handle_t* handle)
{
    int ret = 0;
	
	if (!handle) { 
		TRACE_E("%s:invalid ion mem handle!",__FUNCTION__); 
		return -1; 
	}	
	free(handle);
	return ret;
}

#endif
cam_mem_ops_t g_rk_gralloc_mem_ops {
	//init
	.init = cam_mem_gralloc_ops_init,
	//alloc
	.alloc = cam_mem_gralloc_ops_alloc,
	//free
	.free = cam_mem_gralloc_ops_free,
	.iommu_map = cam_mem_gralloc_ops_iommu_map,
	.iommu_unmap = cam_mem_gralloc_ops_iommu_unmap,
	//flush cache
	.flush_cache = cam_mem_gralloc_ops_flush_cache,
	//deinit
	.deInit = cam_mem_gralloc_ops_deInit,
};

