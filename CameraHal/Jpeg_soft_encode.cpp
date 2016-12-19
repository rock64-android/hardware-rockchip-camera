/****************************************************************************
 * Copyright (C)
 * 2015 - RogerZheng
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *****************************************************************************/

/*
 * @file Jpeg_soft_encode.cpp
 * @brief
 *                 ALSA Audio Git Log
 * - V0.1.0:add alsa audio hal,just support 312x now.
 * - V0.1.0:add alsa audio hal,just support 312x now.
 * - V0.1.0:add alsa audio hal,just support 312x now.
 *
 * @author  RogerZheng
 * @version 1.0
 * @date 2015-08-23
 */


#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <utils/Log.h>
#include "jerror.h"
#include "CameraHal_Tracer.h"
extern "C" {
#include "jpeglib.h"
}
//#define rgb_data 1
#define true 1
#define false 0
typedef uint8_t BYTE;


/**
* @brief nv21_to_yuv
* 用于NV21到yuv的转换函数
*
* @param dst 输出yuv地址
* @param y 输入NV21的y地址
* @param uv 输入NV21的UV地址
* @param width 输入宽的大小
*/
void nv21_to_yuv(uint8_t* dst, uint8_t* y, uint8_t* uv, int width) {
    if (!dst || !y || !uv) {
        return;
    }
	while ((width--) > 0) {
        uint8_t y0 = y[0];
        uint8_t v0 = uv[0];
        uint8_t u0 = *(uv+1);
        dst[0] = y0;
        dst[1] = v0;
        dst[2] = u0;
        dst += 3;
        y++;
        if(!(width % 2)) uv+=2;
    }
}


/**
 * @brief generateJPEG
 *
 * @param data
 * @param w
 * @param h
 * @param outfilename
 */
extern "C" void generateJPEG(uint8_t* data,int w, int h,unsigned char* outbuf,int* outSize)
{
    int nComponent;
    struct jpeg_compress_struct jcs;
    struct jpeg_error_mgr jem;
    uint8_t row_tmp[w*3];
    uint8_t* row_src = NULL;
    uint8_t* row_uv = NULL; // used only for NV12
    memset(row_tmp,0,w*3);
    jcs.err = jpeg_std_error(&jem);
    jpeg_create_compress(&jcs);
    jpeg_stdio_buf(&jcs,(char*)outbuf,(size_t*)outSize);
    jcs.image_width = w;
    jcs.image_height = h;

    nComponent = 3;
#ifdef rgb_data
    jcs.input_components = nComponent;
    if (nComponent==1)
        jcs.in_color_space = JCS_GRAYSCALE;
    else
        jcs.in_color_space = JCS_RGB;
#else
    jcs.input_components = nComponent;
    jcs.in_color_space = JCS_YCbCr;
    jcs.input_gamma = 1;
    nComponent = 1;
#endif
    jpeg_set_defaults(&jcs);
	jpeg_set_quality (&jcs, 100, true);

    jcs.dct_method = JDCT_IFAST;
    jpeg_start_compress(&jcs, true);

    JSAMPROW row_pointer[1];
    int row_stride;
    row_stride = jcs.image_width*nComponent;
    //row_tmp = (uint8_t*)malloc(w * 3);
	row_src = &data[0];
    row_uv = &data[w*h];

	while (jcs.next_scanline < jcs.image_height) {
#ifdef rgb_data
		row_pointer[0] = & data[jcs.next_scanline*row_stride];
#else
        nv21_to_yuv(row_tmp,row_src, row_uv, w);
        row_pointer[0] = row_tmp;
#endif
        jpeg_write_scanlines(&jcs, row_pointer, 1);
	    row_src = row_src + w * nComponent;
	    if (!(jcs.next_scanline % 2))
                row_uv = row_uv +  w * nComponent;
    }
    jpeg_finish_compress(&jcs);
    jpeg_destroy_compress(&jcs);
    LOGD("%s(%d) jpeg soft encode success:jpeg length = %d\n",__FUNCTION__,__LINE__,*outSize);
}

/**
 * @brief generateRGB24Data
 *
 * @returns
 */
BYTE*  generateRGB24Data()
{
    struct {
        BYTE r;
        BYTE g;
        BYTE b;
    } pRGB[100][199];

    memset( pRGB, 0, sizeof(pRGB) );

    int i=0, j=0;

    for(  i=50;i<70;i++ ){
        for( j=70;j<140;j++ ){
            pRGB[i][j].b = 0xff;
        }
    }


    for(  i=0;i<10;i++ ){
        for( j=0;j<199;j++ ){
            pRGB[i][j].r = 0xff;
        }
    }

    BYTE* ret = (BYTE*)malloc(sizeof(BYTE)*100*199*3);
    memcpy(ret, (BYTE*)pRGB, sizeof(pRGB));
    return ret;
}

