/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) 2016 Hyejung Kwon <<cjscld15@nexell.co.kr>>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

#include <mm_types.h>

#include "gstnxscaler.h"
#include "nx-scaler.h"

#ifdef USE_NATIVE_DRM_BUFFER
#include "nx-drm-allocator.h"
#endif

GST_DEBUG_CATEGORY_STATIC (gst_nx_scaler_debug);
#define GST_CAT_DEFAULT gst_nx_scaler_debug


#ifndef ALIGN
#define ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#endif

#define DEF_BUFFER_COUNT 8
#define MAX_RESOLUTION_X 1920
#define MAX_RESOLUTION_Y 1080

/* Filter signals and args */
enum
{
	ARG_0,
	/* scaler */
	ARG_SCALER_CROP_X,
	ARG_SCALER_CROP_Y,
	ARG_SCALER_CROP_WIDTH,
	ARG_SCALER_CROP_HEIGHT,

	ARG_SCALER_DST_WIDTH,
	ARG_SCALER_DST_HEIGHT,

	ARG_NUM,
};

enum
{
	/* FILL ME */
	LAST_SIGNAL
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
	GST_PAD_SINK,
	GST_PAD_ALWAYS,
	GST_STATIC_CAPS("ANY")
);

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
	GST_PAD_SRC,
	GST_PAD_ALWAYS,
	GST_STATIC_CAPS("ANY")
);

#define gst_nx_scaler_parent_class parent_class
G_DEFINE_TYPE (GstNxScaler, gst_nx_scaler, GST_TYPE_BASE_TRANSFORM);

static void gst_nx_scaler_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_nx_scaler_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

/* GObject vmethod implementations */
static guint32
_get_source_handle(GstNxScaler *scaler, guint32 handle, guint32 index)
{
	guint32 dma_fd = -1, gem_fd = -1;
#ifdef USE_NATIVE_DRM_BUFFER	
	if (scaler->src_gem_fds[index] < 0) {
		gem_fd = import_gem_from_flink(scaler->drm_fd, handle);
		if (gem_fd < 0) {
			GST_ERROR_OBJECT(scaler, "failed to import gem from flink(%d)", (int)handle);
			return gem_fd;
		}
		dma_fd = gem_to_dmafd(scaler->drm_fd, gem_fd);
		if (dma_fd < 0) {
			GST_ERROR_OBJECT(scaler, "failed to get drm fd from gem fd(%d)", (int)gem_fd);
			free_gem(scaler->drm_fd, gem_fd);
			return dma_fd;
		}
        	GST_DEBUG_OBJECT(scaler,"flink:%d, src_gem_fd:%d, src_dma_fd :%d", handle, gem_fd, dma_fd);
		scaler->src_gem_fds[index] = gem_fd;
		scaler->src_dma_fds[index] = dma_fd;
	}
#endif
	return scaler->src_dma_fds[index];
}
static GstFlowReturn
_init_scale_context(GstNxScaler *scaler, MMVideoBuffer *mm_buf, struct nx_scaler_context *scaler_ctx)
{
        guint32 src_y_stride = ALIGN(mm_buf->width[0], 32);
        guint32 src_c_stride = ALIGN(src_y_stride >> 1, 16);
	guint32 dst_stride = ALIGN(scaler->dst_width,32);
        guint32 dst_y_stride = ALIGN(dst_stride, 8);
        guint32 dst_c_stride = ALIGN(dst_stride >> 1, 4);
	GstFlowReturn ret = GST_FLOW_ERROR;

	GST_DEBUG_OBJECT(scaler,"_init_scale_context \n ");

#ifdef USE_NATIVE_DRM_BUFFER
	GST_DEBUG_OBJECT(scaler, "type: 0x%x, width: %d, height: %d, plane_num: %d, handle_num: %d, flink_handle: %d, index: %d",
                         mm_buf->type, mm_buf->width[0],
                         mm_buf->height[0], mm_buf->plane_num,
                         mm_buf->handle_num, mm_buf->handle.gem[0], mm_buf->buffer_index);
        GST_DEBUG_OBJECT(scaler, "display fb %d", mm_buf->buffer_index);

	scaler_ctx->dst_fds[0] = scaler->dma_fds[scaler->buffer_index];
	scaler->src_fd = _get_source_handle(scaler, mm_buf->handle.gem[0], mm_buf->buffer_index);
	if (scaler->src_fd < 0)
		GST_ERROR_OBJECT(scaler,"failed to get src dma fd ");
	scaler_ctx->src_fds[0] = scaler->src_fd;
#endif

	scaler_ctx->crop.x = scaler->crop_x;
        scaler_ctx->crop.y = scaler->crop_y;
        scaler_ctx->crop.width = scaler->crop_width;
        scaler_ctx->crop.height = scaler->crop_height;

        scaler_ctx->src_plane_num = 1;
        scaler_ctx->src_width = mm_buf->width[0];
        scaler_ctx->src_height = mm_buf->height[0];
        scaler_ctx->src_code = mm_buf->format;

        scaler_ctx->src_stride[0] = src_y_stride;
        scaler_ctx->src_stride[1] = src_c_stride;
        scaler_ctx->src_stride[2] = src_c_stride;

        scaler_ctx->dst_plane_num = 1;
        scaler_ctx->dst_width = scaler->dst_width;
	scaler_ctx->dst_height = scaler->dst_height;
        scaler_ctx->dst_code = mm_buf->format;

        scaler_ctx->dst_stride[0] = dst_y_stride;
        scaler_ctx->dst_stride[1] = dst_c_stride;
        scaler_ctx->dst_stride[2] = dst_c_stride;
        GST_DEBUG_OBJECT(scaler, "src_dma_fd:%d, dst_dma_fd:%d, crop_x:%d, crop_y:%d, crop_width:%d, crop_height: %d, src_width: %d, src_height: %d, plane_num: %d, dst_width: %d, dst_height: %d",
			scaler_ctx->src_fds[0], scaler_ctx->dst_fds[0],
			scaler_ctx->crop.x, scaler_ctx->crop.y,
			scaler_ctx->crop.width, scaler_ctx->crop.height,
			scaler_ctx->src_width, scaler_ctx->src_height, scaler_ctx->src_plane_num,
			scaler_ctx->dst_width, scaler_ctx->dst_height);

	ret = GST_FLOW_OK;
	return ret;
}

static guint32
_calc_alloc_size(guint32 w, guint32 h)
{
	guint32 y_stride = ALIGN(w, 32);
        guint32 y_size = y_stride * ALIGN(h, 16);
        guint32 size = 0;

	size = y_size + 2 * (ALIGN(y_stride >> 1, 16) * ALIGN(h >> 1, 16));

        return size;
}

static gboolean
_create_buffer(GstNxScaler *scaler)
{
#ifdef USE_NATIVE_DRM_BUFFER
	int drm_fd;
	int gem_fd;
	int dma_fd;
	//void *vaddr;
	int i;
	GST_DEBUG_OBJECT(scaler, "_create_buffer \n");
	drm_fd = open_drm_device();
	if(drm_fd < 0) {
		GST_ERROR_OBJECT(scaler, "failed to open drm device\n");
		return FALSE;
	}
	GST_DEBUG_OBJECT(scaler,"open_drm_device : %d \n", drm_fd);
	scaler->drm_fd = drm_fd;
	for (i=0; i < scaler->buffer_count; i++) {
		gem_fd = alloc_gem(scaler->drm_fd, scaler->buffer_size, 0);
		if (gem_fd < 0)
		{
			GST_ERROR_OBJECT(scaler, "failed to alloc gem %d", i);
			return FALSE;
		}
		dma_fd = gem_to_dmafd(scaler->drm_fd, gem_fd);
		if (dma_fd < 0) {
			GST_ERROR_OBJECT(scaler,"failed to gem to dma %d", i);
			return FALSE;
		}
		#if 0
		if (get_vaddr(scaler->drm_fd, gem_fd, scaler->buffer_size, &vaddr)) {
			GST_ERROR_OBJECT(scaler,"failed to get_vaddr %d", i);
			return FALSE;
		}
		#endif
		scaler->gem_fds[i] = gem_fd;
		scaler->dma_fds[i] = dma_fd;
		//scaler->vaddrs[i] = vaddr;
	}
	return TRUE;
#endif
}

static gboolean
_destroy_buffer(GstNxScaler *scaler)
{
#ifdef USE_NATIVE_DRM_BUFFER
	int i;

	GST_DEBUG_OBJECT(scaler,"_destory_buffer");

	for (i = 0; i < scaler->buffer_count; i++) {
		if(scaler->dma_fds[i] >= 0) {
			close(scaler->dma_fds[i]);
			scaler->dma_fds[i] = -1;
		}
		if(scaler->gem_fds[i] >= 0) {
			close(scaler->gem_fds[i]);
			scaler->gem_fds[i] = -1;
		}
	}

	for (i = 0; i < MAX_SRC_BUFFER_COUNT; i++) {
		if(scaler->src_gem_fds[i] >= 0) {
			close(scaler->src_gem_fds[i]);
			scaler->src_gem_fds[i] = -1;
			scaler->src_dma_fds[i] = -1;
		}
	}
	if (scaler->drm_fd) {
		close(scaler->drm_fd);
		scaler->drm_fd = -1;
	}

	GST_DEBUG_OBJECT(scaler,"End _destory_buffer");
	return TRUE;
#endif
}

static GstFlowReturn
gst_nxscaler_prepare_output_buffer(GstBaseTransform *trans, GstBuffer *inbuf, GstBuffer **outbuf)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);
	GstFlowReturn ret = GST_FLOW_OK;
	MMVideoBuffer *mm_buf = NULL;
	GstMemory *meta_data = NULL, *meta_block = NULL;
	GstBuffer *buffer = NULL;
	GstMapInfo info;
	struct nx_scaler_context s_ctx;
	if (gst_base_transform_is_passthrough (trans)) {
		GST_DEBUG_OBJECT(scaler,"Passthrough, no need to do anything");
		*outbuf = inbuf;
		goto beach;
	}

	memset(&info, 0, sizeof(GstMapInfo));
	meta_block = gst_buffer_peek_memory(inbuf, 0);
	gst_memory_map(meta_block, &info, GST_MAP_READ);
	mm_buf = (MMVideoBuffer *)malloc(sizeof(*mm_buf));
	if (!mm_buf) {
		GST_ERROR_OBJECT(scaler, "failed to alloc MMVideoBuffer");
		goto beach;
	}
	memset(mm_buf, 0, sizeof(*mm_buf));

	memcpy(mm_buf,(MMVideoBuffer*)info.data,info.size);
	if (!mm_buf) {
		GST_ERROR_OBJECT(scaler, "failed to get MMVideoBuffer !");
		goto beach;
	} else {
                GST_DEBUG_OBJECT(scaler, "get MMVideoBuffer");
	}

	ret = _init_scale_context(scaler,mm_buf,&s_ctx);
	if (ret != GST_FLOW_OK) {
		GST_ERROR_OBJECT(scaler, "set scale context fail");
		goto beach;
	}
	gst_memory_unmap(meta_block, &info);
	ret = nx_scaler_run(scaler->scaler_fd, &s_ctx);

#ifdef USE_NATIVE_DRM_BUFFER
	mm_buf->type = MM_VIDEO_BUFFER_TYPE_GEM;
	mm_buf->data[0] = scaler->vaddrs[scaler->buffer_index];
	if (scaler->flinks[scaler->buffer_index] < 0) {
		scaler->flinks[scaler->buffer_index] = get_flink_name(scaler->drm_fd,
							  scaler->gem_fds[scaler->buffer_index]);
	}
	mm_buf->handle.gem[0] = scaler->flinks[scaler->buffer_index];
	mm_buf->handle_num = 1;
	mm_buf->buffer_index = scaler->buffer_index;
	if (scaler->buffer_index < scaler->buffer_count - 1)
		scaler->buffer_index++;
	else
		scaler->buffer_index = 0;
#endif

	buffer = gst_buffer_new();
	meta_data = gst_memory_new_wrapped(GST_MEMORY_FLAG_READONLY,
		mm_buf,
		sizeof(MMVideoBuffer),
		0,
		sizeof(MMVideoBuffer),
		mm_buf,
		free);
	if (!meta_data) {
		GST_ERROR_OBJECT(scaler, "failed to get copy data ");
		goto beach;
	} else {
		gst_buffer_append_memory(buffer,meta_data);
	}
	*outbuf = buffer;
	return ret;
beach:

	gst_memory_unmap(meta_block, &info);
	if (buffer) {
		gst_buffer_unref((GstBuffer*)buffer);
	}
	ret = GST_FLOW_ERROR;
	return ret;
}

static GstFlowReturn
gst_nxscaler_transform_transform (GstBaseTransform *trans, GstBuffer *inbuf, GstBuffer *outbut)
{
	GST_DEBUG("gst_nxscaler_transfrom  \n");
	return GST_FLOW_OK;
}

static gboolean
gst_nxscaler_transform_start (GstBaseTransform *trans)
{

	GstNxScaler *scaler = GST_NXSCALER(trans);
	gboolean ret = TRUE;
	GST_DEBUG_OBJECT(scaler,"gst_nxscaler_transform_start \n");
	scaler->scaler_fd = scaler_open();
	if (scaler->scaler_fd < 0)
		GST_ERROR_OBJECT(scaler, "failed to open scaler");
	scaler->buffer_count = DEF_BUFFER_COUNT;
	scaler->buffer_size = _calc_alloc_size(scaler->dst_width, scaler->dst_height);
	GST_DEBUG_OBJECT(scaler,"buffer_size = %d \n", scaler->buffer_size);
	ret = _create_buffer(scaler);
	if (ret == FALSE)
		GST_ERROR_OBJECT(scaler, "failed to create buffer");
	return ret;
}

static gboolean
gst_nxscaler_transform_stop (GstBaseTransform *trans)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);
	GST_DEBUG_OBJECT(scaler,"gst_nxscaler_transform_stop \n");

	_destroy_buffer(scaler);
	nx_scaler_close(scaler->scaler_fd);
	return TRUE;
}


/* initialize the scaler's class */
static void
gst_nx_scaler_class_init (GstNxScalerClass * klass)

{
	GObjectClass *gobject_class;
	GstElementClass *gstelement_class;
	GstBaseTransformClass *base_transform_class;

	gobject_class = (GObjectClass *) klass;
	gstelement_class = (GstElementClass *) klass;
	base_transform_class = (GstBaseTransformClass *) klass;
	gobject_class->set_property = gst_nx_scaler_set_property;
	gobject_class->get_property = gst_nx_scaler_get_property;

	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_X,
					g_param_spec_uint("scaler_crop_x",
							  "Crop X",
							  "X value for crop",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_Y,
					g_param_spec_uint("scaler_crop_y",
							  "Crop Y",
							  "Y value for crop",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));
	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_WIDTH,
					g_param_spec_uint("scaler_crop_width",
							  "Crop WIDTH",
							  "WIDTH value for crop",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_HEIGHT,
					g_param_spec_uint("scaler_crop_height",
							  "Crop HEIGHT",
							  "HEIGHT value for crop",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_DST_WIDTH,
					g_param_spec_uint("scaler_dst_width",
							  "dst WIDTH",
							  "WIDTH value for display",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_DST_HEIGHT,
					g_param_spec_uint("scaler_dst_height",
							  "dst HEIGHT",
							  "HEIGHT value for display",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));

	gst_element_class_set_details_simple(gstelement_class,
	"Nexell Scaler GStreamer Plug-in",
	"Filter/Video/Scaler",
	"scale the video stream based GStreamer Plug-in",
	"Hyejung Kwon <<cjscld15@nexell.co.kr>>");

	gst_element_class_add_pad_template (gstelement_class,
	gst_static_pad_template_get (&src_factory));
	gst_element_class_add_pad_template (gstelement_class,
	gst_static_pad_template_get (&sink_factory));

	base_transform_class->prepare_output_buffer = GST_DEBUG_FUNCPTR(gst_nxscaler_prepare_output_buffer);
	base_transform_class->transform = GST_DEBUG_FUNCPTR(gst_nxscaler_transform_transform);
	base_transform_class->start = GST_DEBUG_FUNCPTR(gst_nxscaler_transform_start);
	base_transform_class->stop = GST_DEBUG_FUNCPTR(gst_nxscaler_transform_stop);
	base_transform_class->passthrough_on_same_caps = FALSE;
	base_transform_class->transform_ip_on_passthrough = FALSE;
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_nx_scaler_init (GstNxScaler * scaler)
{
	int i;

	GST_DEBUG_OBJECT(scaler, "gst_nx_scaler_init \n");

	scaler->silent = FALSE;

	scaler->dst_width = 1280;
	scaler->dst_height = 720;

	scaler->crop_x = 0;
	scaler->crop_y = 0;

	scaler->crop_width = scaler->dst_width;
	scaler->crop_height = scaler->dst_height;

	scaler->scaler_fd = -1;
	scaler->buffer_size = 0;
	scaler->buffer_count = 0;
	scaler->buffer_index = 0;
	scaler->src_fd = -1;

#ifdef USE_NATIVE_DRM_BUFFER
	scaler->drm_fd = -1;
	for (i = 0; i < MAX_BUFFER_COUNT; i++) {
		scaler->gem_fds[i] = -1;
		scaler->dma_fds[i] = -1;
		scaler->flinks[i] = -1;
		scaler->src_gem_fds[i] = -1;
		scaler->src_dma_fds[i] = -1;
	}
	for (i = 0; i < MAX_SRC_BUFFER_COUNT; i++) {
		scaler->src_gem_fds[i] = -1;
		scaler->src_dma_fds[i] = -1;
	}
#endif
}

static void
gst_nx_scaler_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
	GstNxScaler *scaler = GST_NXSCALER (object);

	GST_INFO_OBJECT(scaler, "gst_nx_scaler_set_property ");

	switch (prop_id) {
	case ARG_SCALER_CROP_X:
		scaler->crop_x = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_CROP_X: %u",
				scaler->crop_x);
		break;
	case ARG_SCALER_CROP_Y:
		scaler->crop_y = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_CROP_Y: %u",
				scaler->crop_y);
		break;
	case ARG_SCALER_CROP_WIDTH:
		scaler->crop_width = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_CROP_WIDTH: %u",
				scaler->crop_width);
		break;
	case ARG_SCALER_CROP_HEIGHT:
		scaler->crop_height = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_CROP_HEIGHT: %u",
				scaler->crop_height);
		break;
	case ARG_SCALER_DST_WIDTH:
		scaler->dst_width = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_DST_WIDTH: %u",
				scaler->dst_width);
		break;
	case ARG_SCALER_DST_HEIGHT:
		scaler->dst_height = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set SCALER_DST_HEIGHT: %u",
				scaler->dst_height);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
		break;
	}
}

static void
gst_nx_scaler_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
	switch (prop_id) {
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
		break;
	}
}

/* GstElement vmethod implementations */



/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
nxscaler_init (GstPlugin * nxscaler)
{
  /* debug category for fltering log messages
   *
   * exchange the string 'Template scaler' with your description
   */

  GST_DEBUG_CATEGORY_INIT (gst_nx_scaler_debug, "nxscaler",
      0, "Template nxscaler");

  GST_DEBUG_OBJECT(nxscaler,"nxscaler_init\n");
  return gst_element_register (nxscaler, "nxscaler", GST_RANK_PRIMARY+102,
      GST_TYPE_NXSCALER);

}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "nexell.gst.scaler"
#endif

/* gstreamer looks for this structure to register scalers
 *
 * exchange the string 'Template scaler' with your scaler description
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    nxscaler,
    "Nexell Scaler plug-in",
    nxscaler_init,
    "0.0.1",
    "LGPL",
    "Nexell Co",
    "http://www.nexell.co.kr/"
)
