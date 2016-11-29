#include "camera_mem.h"

struct cam_mem_ops_des_s{
	const char* name;
	enum cam_mem_type_e mem_type;
	cam_mem_ops_t* ops;
};

//extern cam_mem_ops_t g_rk_ion_mem_ops;
//extern cam_mem_ops_t g_rk_ionDma_mem_ops;
extern cam_mem_ops_t g_rk_gralloc_mem_ops;

static struct cam_mem_ops_des_s cam_mem_ops_array[] = {
	{"ion",CAM_MEM_TYPE_ION,NULL},
	{"ionDma",CAM_MEM_TYPE_IONDMA,NULL},
	{"gralloc",CAM_MEM_TYPE_GRALLOC,&g_rk_gralloc_mem_ops},
};

cam_mem_ops_t* get_cam_ops(enum cam_mem_type_e mem_type)
{
	int ops_index = -1;
	switch(mem_type) {

		case CAM_MEM_TYPE_ION:
			ops_index = 0;
			break;
		case CAM_MEM_TYPE_IONDMA:
			ops_index = 1;
			break;
		case CAM_MEM_TYPE_GRALLOC:
			ops_index = 2;
			break;
		default:
			ops_index = -1;
	}

	if (ops_index != -1)
		return cam_mem_ops_array[ops_index].ops;
	else
		return NULL;
}

