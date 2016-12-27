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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <gst/gst.h>
#include <gst/video/video-info.h>
#include <gst/base/gstbasetransform.h>
#include <linux/media-bus-format.h>
#include <gstmmvideobuffermeta.h>

#include "mm_types.h"

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
#define DEFAULT_RESOLUTION_X 1280
#define DEFAULT_RESOLUTION_Y 720
#define MINIMUM_CROP_X 320
#define MINIMUM_CROP_Y 240
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

	/* buffer type */
	ARG_BUFFER_TYPE,

	ARG_NUM,
};

enum
{
	BUFFER_TYPE_NORMAL,
	BUFFER_TYPE_GEM,
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
	GST_STATIC_CAPS("video/x-raw, "
	"format = (string) { I420 }, "
	"width = (int)[1, 1920],"
	"height = (int) [1, 1080]; ")
);

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
	GST_PAD_SRC,
	GST_PAD_ALWAYS,
	GST_STATIC_CAPS("video/x-raw, "
	"format = (string) { I420 }, "
	"width = (int)[1, 1920],"
	"height = (int) [1, 1080]; ")
);

#define gst_nx_scaler_parent_class parent_class
G_DEFINE_TYPE (GstNxScaler, gst_nx_scaler, GST_TYPE_BASE_TRANSFORM);

static void gst_nx_scaler_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_nx_scaler_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

/* GObject vmethod implementations */
static gint32
_get_source_handle(GstNxScaler *scaler, guint32 handle, guint32 index)
{
	gint32 dma_fd = -1, gem_fd = -1;
#ifdef USE_NATIVE_DRM_BUFFER
	if(index<MAX_IN_BUFFER_COUNT) {
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
	} else {
		GST_ERROR_OBJECT(scaler, "index is over the size of in buffer (%d)", (int)index);
		return -1;
	}
#endif
	return scaler->src_dma_fds[index];
}
static GstFlowReturn
_init_scale_context(GstNxScaler *scaler, MMVideoBuffer *mm_buf, struct nx_scaler_context *scaler_ctx)
{
        guint32 src_y_stride = ALIGN(mm_buf->width[0], 32);
        guint32 src_c_stride = ALIGN(src_y_stride >> 1, 16);
        guint32 dst_y_stride = ALIGN(scaler->dst_width, 8);
        guint32 dst_c_stride = ALIGN(dst_y_stride >> 1, 4);
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
	scaler_ctx->src_code = MEDIA_BUS_FMT_YUYV8_2X8;

	scaler_ctx->src_stride[0] = src_y_stride;
	scaler_ctx->src_stride[1] = src_c_stride;
	scaler_ctx->src_stride[2] = src_c_stride;

	scaler_ctx->dst_plane_num = 1;
	scaler_ctx->dst_width = scaler->dst_width;
	scaler_ctx->dst_height = scaler->dst_height;
	scaler_ctx->dst_code = MEDIA_BUS_FMT_YUYV8_2X8;
	scaler_ctx->dst_stride[0] = dst_y_stride;
	scaler_ctx->dst_stride[1] = dst_c_stride;
	scaler_ctx->dst_stride[2] = dst_c_stride;
    GST_DEBUG_OBJECT(scaler, "src_dma_fd:%d, dst_dma_fd:%d,crop_x:%d, crop_y:%d, crop_width:%d, crop_height: %d, src_width: %d, src_height: %d, plane_num: %d, dst_width: %d, dst_height: %d",
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
		if (scaler->buffer_type == BUFFER_TYPE_NORMAL) {
			void *vaddr = mmap(NULL,
					   scaler->buffer_size,
					   PROT_READ,
					   MAP_SHARED,
					   dma_fd,
					   0);
			if (vaddr == MAP_FAILED) {
				GST_ERROR_OBJECT(scaler, "failed to mmap");
				return FALSE;
			}
			scaler->vaddrs[i] = vaddr;
		}
		scaler->gem_fds[i] = gem_fd;
		scaler->dma_fds[i] = dma_fd;
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
			free_gem(scaler->drm_fd, scaler->gem_fds[i]);
			scaler->gem_fds[i] = -1;
		}
	}

	for (i = 0; i < MAX_IN_BUFFER_COUNT; i++) {
		if(scaler->src_gem_fds[i] >= 0) {
			close(scaler->src_dma_fds[i]);
			free_gem(scaler->drm_fd, scaler->src_gem_fds[i]);
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

static void _unmap_buffer(GstNxScaler *scaler)
{
	int i;

	GST_DEBUG_OBJECT(scaler, "ENTERED");
	for (i = 0; i < scaler->buffer_count; i++) {
		if (scaler->vaddrs[i]) {
			munmap(scaler->vaddrs[i], scaler->buffer_size);
			scaler->vaddrs[i] = NULL;
		}
	}
        GST_DEBUG_OBJECT(scaler, "LEAVED");
}

static GstCaps*
_set_caps_init(GstNxScaler *scaler)
{
	GstCaps *caps = NULL;

	GST_DEBUG_OBJECT(scaler, "ENTERED");

        caps =  gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, "I420",
		"framerate", GST_TYPE_FRACTION, 1, 1,
                "buffer-type", G_TYPE_INT, MM_VIDEO_BUFFER_TYPE_GEM,
                "width", G_TYPE_INT, scaler->dst_width,
                "height", G_TYPE_INT, scaler->dst_height,
                NULL);
        if(caps) {
		scaler->src_caps = gst_caps_copy(caps);
        	GST_DEBUG_OBJECT(scaler, "new caps %" GST_PTR_FORMAT, scaler->src_caps);
	}
        GST_DEBUG_OBJECT(scaler, "LEAVED");
        return caps;
}

static GstCaps*
gst_nxscaler_fixate_caps (GstBaseTransform *trans, GstPadDirection direction,
GstCaps *caps, GstCaps *othercaps)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);
	GstStructure *s_src, *s_sink = NULL;
	guint32 w = 0, h = 0;
	GstPad *pad = NULL;
	const GValue *framerate;

	GST_DEBUG_OBJECT(scaler, "Direction : %s[%d] ",
		(direction == GST_PAD_SINK) ? "sink" : "sorce", direction);
	pad =
	(direction ==
	GST_PAD_SINK) ? GST_BASE_TRANSFORM_SINK_PAD (trans) :
	GST_BASE_TRANSFORM_SRC_PAD (trans);	
	GST_DEBUG_OBJECT(scaler, "caps %" GST_PTR_FORMAT, caps);
	GST_DEBUG_OBJECT(scaler, "other caps %" GST_PTR_FORMAT ,othercaps);
	if(direction == GST_PAD_SINK) {
        	s_sink = gst_caps_get_structure(caps, 0);
        	if(!s_sink)
                	GST_ERROR_OBJECT(scaler, "failed to get structure from caps %" GST_PTR_FORMAT, caps);
        	s_src = gst_caps_get_structure(othercaps, 0);
        	if(!s_src)
                	GST_ERROR_OBJECT(scaler, "failed to get structure from caps %" GST_PTR_FORMAT, othercaps);
		framerate = gst_structure_get_value(s_sink, "framerate");
		if(!framerate)
			GST_ERROR_OBJECT(scaler, "failed to get structure from caps %" GST_PTR_FORMAT, othercaps);
		gst_structure_set_value(s_src, "framerate", framerate);
        	if (!gst_structure_get_int(s_src, "width", &w))
			GST_ERROR_OBJECT(scaler, "failed to get width from structure(%p)", s_src);
        	if (!gst_structure_get_int(s_src, "height", &w))
			GST_ERROR_OBJECT(scaler, "failed to get height from structure(%p)", s_src);
		if((w != scaler->dst_width) || (h != scaler->dst_height)) {
                	GST_ERROR_OBJECT(scaler, "fixed caps of src pad isan't matched with the caps of orignal srcpad");
                	gst_structure_set(s_src, "width", G_TYPE_INT, scaler->dst_width, NULL);
                	gst_structure_set(s_src, "height", G_TYPE_INT, scaler->dst_height, NULL);
		}
		GST_DEBUG_OBJECT(scaler, "other caps is changed %" GST_PTR_FORMAT, othercaps);
	}
  	othercaps = gst_caps_fixate (othercaps);
 	GST_DEBUG_OBJECT (scaler, "fixated to %" GST_PTR_FORMAT, othercaps);
	return othercaps;
}
static gboolean
_check_valid_crop_size(GstNxScaler *scaler, guint32 width, guint32 height)
{
	guint32 crop_width = scaler->crop_width + scaler->crop_x;
	guint32 crop_height = scaler->crop_height + scaler->crop_y;
	gboolean ret = TRUE;

	GST_INFO_OBJECT(scaler, "input - width[%d] height[%d], scaler - width[%d] height[%d]",
				width, height, crop_width, crop_height);
	if( (crop_width <= 0) || (crop_width > width) || (crop_height <= 0) || (crop_height > height)) {
		ret = FALSE;
		GST_ERROR_OBJECT(scaler, "crop width and height is unvalid: x[%d] y[%d]",
				scaler->crop_x, scaler->crop_y);
	}
	return ret;
}

static gboolean
gst_nxscaler_accept_caps (GstBaseTransform *trans, GstPadDirection direction,
	GstCaps *caps)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);
	GstPad *pad, *otherpad;
	GstCaps *templ, *otempl, *ocaps = NULL;
	gboolean ret = TRUE;
	GstStructure *s = NULL;
	guint32 w = 0, h = 0;

	GST_DEBUG_OBJECT(scaler, "Entered ");

	GST_DEBUG_OBJECT(scaler, "Direction : %s[%d] ",
		(direction == GST_PAD_SINK) ? "sink" : "sorce", direction);
	pad =
	(direction ==
	GST_PAD_SINK) ? GST_BASE_TRANSFORM_SINK_PAD (trans) :
	GST_BASE_TRANSFORM_SRC_PAD (trans);
	otherpad =
	(direction ==
	GST_PAD_SINK) ? GST_BASE_TRANSFORM_SRC_PAD (trans) :
	GST_BASE_TRANSFORM_SINK_PAD (trans);
	GST_DEBUG_OBJECT (scaler, "accept caps %" GST_PTR_FORMAT, caps);
	GST_DEBUG_OBJECT (scaler, "sink pad caps: %" GST_PTR_FORMAT,
	gst_pad_get_pad_template_caps (GST_BASE_TRANSFORM_SINK_PAD(trans)));
	GST_DEBUG_OBJECT (scaler, "src pad caps: %" GST_PTR_FORMAT,
	scaler->src_caps);
	if(direction == GST_PAD_SRC) {
		if((scaler->src_caps != NULL) || (!gst_caps_is_empty(scaler->src_caps))) {
			templ = gst_caps_copy(scaler->src_caps);
		} else {
			GST_DEBUG_OBJECT (scaler, "scaler->src_caps is null or empty  %" GST_PTR_FORMAT
			, scaler->src_caps);
			templ = gst_pad_get_pad_template_caps (pad);
		}
		otempl = gst_pad_get_pad_template_caps (otherpad);
	} else {
		if((scaler->src_caps != NULL) || (!gst_caps_is_empty(scaler->src_caps))) {
			otempl = gst_caps_copy(scaler->src_caps);
		} else {
			GST_DEBUG_OBJECT (scaler, "scaler->src_caps is null or empty  %" GST_PTR_FORMAT
			, scaler->src_caps);
			otempl = gst_pad_get_pad_template_caps (otherpad);
		}
		templ = gst_pad_get_pad_template_caps (pad);
	}

	/* get all the formats we can handle on this pad */
	GST_DEBUG_OBJECT (scaler, "intersect with pad template: %" GST_PTR_FORMAT,
	templ);
	GST_DEBUG_OBJECT (scaler, "intersect with other pad template: %" GST_PTR_FORMAT,
	otempl);

	if (!gst_caps_can_intersect (caps, templ)) {
		GST_DEBUG_OBJECT (scaler,
			"the caps that is intersected with pad template is empty : %"
			GST_PTR_FORMAT, caps);
		goto reject_caps;
	}
	s = gst_caps_get_structure(caps, 0);
	if (!gst_structure_get_int(s, "width", &w) || !gst_structure_get_int(s, "height", &h))
		GST_ERROR_OBJECT(scaler, "failed to get width from structure(%p)", s);
	if(!_check_valid_crop_size(scaler, w, h))
		goto reject_caps;
	done:
	{
		GST_DEBUG_OBJECT (scaler, "accept-caps result: %d", ret);
		gst_caps_unref (templ);
		gst_caps_unref (otempl);
		return ret;
	}
	/* ERRORS */
	reject_caps:
	{
		GST_DEBUG_OBJECT (scaler, "caps can't intersect with the template");
		ret = FALSE;
		goto done;
	}
	no_transform_possible:
	{
		GST_DEBUG_OBJECT (scaler,
		"transform could not transform %" GST_PTR_FORMAT
		" in anything we support", caps);
		ret = FALSE;
		goto done;
	}
	return TRUE;
}

static gboolean
gst_nxscaler_sink_event (GstBaseTransform *parent, GstEvent *event)
{
	gboolean ret=TRUE;
	GstNxScaler *scaler = GST_NXSCALER(parent);
	switch (GST_EVENT_TYPE (event)) {
		case GST_EVENT_CAPS:
		{
			GstCaps *caps;
			GST_DEBUG_OBJECT(scaler, "caps ");
			gst_event_parse_caps (event, &caps);
			GST_DEBUG_OBJECT (scaler, "in(sinkpad) caps  %" GST_PTR_FORMAT, caps);
			break;
		}
		default:
		GST_DEBUG_OBJECT(scaler,"event = %s",GST_EVENT_TYPE_NAME(event));
		break;
	}
	GST_DEBUG_OBJECT(scaler,"ret[%d]",ret);
	return ret;
}

static gboolean
gst_nxscaler_src_event (GstBaseTransform *parent, GstEvent *event)
{
	gchar *caps_string;
	GstNxScaler *scaler = GST_NXSCALER(parent);

	switch (GST_EVENT_TYPE (event)) {
		case GST_EVENT_CAPS:
		{
			GstCaps *caps;
			GST_DEBUG_OBJECT(scaler,"event = %s",GST_EVENT_TYPE_NAME(event));
			gst_event_parse_caps (event, &caps);
			break;
		}
		default:
		GST_DEBUG_OBJECT(scaler,"event = %s",GST_EVENT_TYPE_NAME(event));
		break;
	}
	return TRUE;
}
static GstFlowReturn
output_buffer_accel(GstNxScaler *scaler, GstBuffer *inbuf,
		    GstBuffer **outbuf)
{
	GstFlowReturn ret = GST_FLOW_OK;
	MMVideoBuffer *mm_buf = NULL;
	GstMemory *meta_data = NULL, *meta_block = NULL;
	GstBuffer *buffer = NULL;
	GstMapInfo info;
	struct nx_scaler_context s_ctx;

	memset(&info, 0, sizeof(GstMapInfo));
	meta_block = gst_buffer_peek_memory(inbuf, 0);
	gst_memory_map(meta_block, &info, GST_MAP_READ);
	mm_buf = (MMVideoBuffer *)malloc(sizeof(*mm_buf));
	if (!mm_buf) {
		GST_ERROR_OBJECT(scaler, "failed to alloc MMVideoBuffer");
		goto beach;
	}
	memset(mm_buf, 0, sizeof(*mm_buf));
	memcpy(mm_buf, (MMVideoBuffer*)info.data, info.size);
	if (!mm_buf) {
		GST_ERROR_OBJECT(scaler, "failed to get MMVideoBuffer !");
		goto beach;
	} else {
                GST_DEBUG_OBJECT(scaler, "get MMVideoBuffer");
	}

	ret = _init_scale_context(scaler, mm_buf, &s_ctx);
	if (ret != GST_FLOW_OK) {
		GST_ERROR_OBJECT(scaler, "set scale context fail");
		goto beach;
	}

	GST_DEBUG_OBJECT(scaler, "NX_SCALER_RUN \n");
	nx_scaler_run(scaler->scaler_fd, &s_ctx);
#ifdef USE_NATIVE_DRM_BUFFER
	mm_buf->type = MM_VIDEO_BUFFER_TYPE_GEM;
	mm_buf->data[0] = scaler->vaddrs[scaler->buffer_index];
	if (scaler->flinks[scaler->buffer_index] < 0) {
		scaler->flinks[scaler->buffer_index] = get_flink_name(scaler->drm_fd,
							  scaler->gem_fds[scaler->buffer_index]);
	}
	mm_buf->width[0] = scaler->dst_width;
	mm_buf->height[0] = scaler->dst_height;
        mm_buf->size[0] = scaler->buffer_size;
        /* FIXME: currently test only YUV420 format */
        mm_buf->plane_num = 3;
        mm_buf->format = MM_PIXEL_FORMAT_I420;
        mm_buf->stride_width[0] = s_ctx.dst_stride[0];
        mm_buf->stride_width[1] = s_ctx.dst_stride[1];
        mm_buf->stride_width[2] = s_ctx.dst_stride[2];
        mm_buf->stride_height[0] = GST_ROUND_UP_16(scaler->dst_height);
        mm_buf->stride_height[1] = GST_ROUND_UP_16(scaler->dst_height >> 1);
        mm_buf->stride_height[2] = mm_buf->stride_height[1];
	mm_buf->handle.gem[0] = scaler->flinks[scaler->buffer_index];
	mm_buf->handle_num = 1;
	mm_buf->buffer_index = scaler->buffer_index;
	if (scaler->buffer_index < scaler->buffer_count - 1)
		scaler->buffer_index++;
	else
		scaler->buffer_index = 0;
#endif

	buffer = gst_buffer_new();
	if (buffer == NULL) {
		GST_ERROR_OBJECT(scaler, "failed to get gst buffer ");
		goto beach;
	}

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
		GST_DEBUG_OBJECT(scaler, "gst_buffer = 0x%x, meta_data = 0x%x, meta_data_size = %d \n",
		buffer, meta_data, sizeof(MMVideoBuffer));
		gst_buffer_append_memory(buffer,meta_data);
	}
	// set time info
	GST_BUFFER_PTS(buffer) = GST_BUFFER_PTS(inbuf);
	gst_buffer_add_mmvideobuffer_meta(buffer, 0);
	*outbuf = buffer;
	GST_DEBUG_OBJECT(scaler, "Input Memory Buffer Unmap \n");
	gst_memory_unmap(meta_block, &info);
	return ret;
beach:

	GST_ERROR_OBJECT(scaler, "ERROR: \n ");
	gst_memory_unmap(meta_block, &info);
	if (buffer) {
		gst_buffer_unref((GstBuffer*)buffer);
	}
	if (mm_buf)
		free(mm_buf);
	ret = GST_FLOW_ERROR;
	return ret;
}

static GstFlowReturn
output_buffer_normal(GstNxScaler *scaler, GstBuffer *inbuf,
		     GstBuffer **outbuf)
{
	GstFlowReturn ret = GST_FLOW_OK;
	MMVideoBuffer *mm_buf = NULL;
	GstMemory *meta_data = NULL, *meta_block = NULL;
	GstBuffer *buffer = NULL;
	GstMapInfo info;
	GstVideoMeta *video_meta = NULL;
	unsigned char *data = NULL;
	gsize offset[3] = {0, };
	gint stride[3] = {0, };
	struct nx_scaler_context s_ctx;

	memset(&info, 0, sizeof(GstMapInfo));
	meta_block = gst_buffer_peek_memory(inbuf, 0);
	gst_memory_map(meta_block, &info, GST_MAP_READ);

	mm_buf = (MMVideoBuffer *)info.data;
	if (!mm_buf) {
		GST_ERROR_OBJECT(scaler, "failed to get MMVideoBuffer !");
		goto beach;
	} else {
                GST_DEBUG_OBJECT(scaler, "get MMVideoBuffer");
	}

	ret = _init_scale_context(scaler, mm_buf, &s_ctx);
	if (ret != GST_FLOW_OK) {
		GST_ERROR_OBJECT(scaler, "set scale context fail");
		goto beach;
	}

	GST_DEBUG_OBJECT(scaler, "NX_SCALER_RUN \n");
	nx_scaler_run(scaler->scaler_fd, &s_ctx);

	buffer = gst_buffer_new();
	if (buffer == NULL) {
		GST_ERROR_OBJECT(scaler, "failed to get gst buffer ");
		goto beach;
	}
	data = (unsigned char *)scaler->vaddrs[scaler->buffer_index];
	if (scaler->buffer_index < scaler->buffer_count - 1)
		scaler->buffer_index++;
	else
		scaler->buffer_index = 0;

	meta_data = gst_memory_new_wrapped(GST_MEMORY_FLAG_READONLY,
					   data,
					   scaler->buffer_size,
					   0,
					   scaler->buffer_size,
					   NULL,
					   NULL);

	if (!meta_data) {
		GST_ERROR_OBJECT(scaler, "failed to get gstmem ");
		goto beach;
	} else {
		GST_DEBUG_OBJECT(scaler, "gst_buffer = 0x%x, meta_data = 0x%x, meta_data_size = %d \n",
		buffer, meta_data, scaler->buffer_size);
		gst_buffer_append_memory(buffer, meta_data);
	}

        mm_buf->stride_width[0] = s_ctx.dst_stride[0];
        mm_buf->stride_width[1] = s_ctx.dst_stride[1];
        mm_buf->stride_width[2] = s_ctx.dst_stride[2];
        mm_buf->stride_height[0] = GST_ROUND_UP_16(scaler->dst_height);
        mm_buf->stride_height[1] = GST_ROUND_UP_16(scaler->dst_height >> 1);
        mm_buf->stride_height[2] = mm_buf->stride_height[1];
	stride[0] = s_ctx.dst_stride[0];
	stride[1] = s_ctx.dst_stride[1];
	stride[2] = s_ctx.dst_stride[2];
	offset[0] = 0;
	offset[1] = offset[0] + (stride[0] * GST_ROUND_UP_16(scaler->dst_height));
	offset[2] = offset[1] + (stride[1] * GST_ROUND_UP_16(scaler->dst_height >> 1));
	GST_DEBUG_OBJECT(scaler, "stride : %d, %d, %d , offset : %d, %d, %d",
		stride[0], stride[1], stride[2],
		offset[0], offset[1], offset[2]);
	video_meta = gst_buffer_add_video_meta_full(buffer,
						    GST_VIDEO_FRAME_FLAG_NONE,
						    GST_VIDEO_FORMAT_I420,
						    scaler->dst_width,
						    scaler->dst_height,
						    3,
						    offset,
						    stride);
	if (!video_meta) {
		GST_ERROR_OBJECT(scaler, "failed to gst_buffer_add_video_meta_full");
		goto beach;
	}
	// set time info
	GST_BUFFER_PTS(buffer) = GST_BUFFER_PTS(inbuf);
	*outbuf = buffer;

	GST_DEBUG_OBJECT(scaler, "Input Memory Buffer Unmap \n");
	gst_memory_unmap(meta_block, &info);

	return ret;

beach:
	GST_ERROR_OBJECT(scaler, "ERROR: \n ");
	gst_memory_unmap(meta_block, &info);
	if (buffer) {
		gst_buffer_unref((GstBuffer*)buffer);
	}
	ret = GST_FLOW_ERROR;
	return ret;
}

static GstFlowReturn
gst_nxscaler_prepare_output_buffer(GstBaseTransform *trans, GstBuffer *inbuf, GstBuffer **outbuf)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);

	if (gst_base_transform_is_passthrough (trans)) {
		GST_DEBUG_OBJECT(scaler,"Passthrough, no need to do anything");
		*outbuf = inbuf;
		return GST_FLOW_OK;
	}

	if (scaler->buffer_type == BUFFER_TYPE_GEM)
		return output_buffer_accel(scaler, inbuf, outbuf);
	else
		return output_buffer_normal(scaler, inbuf, outbuf);
}


static GstFlowReturn
gst_nxscaler_transform (GstBaseTransform *trans, GstBuffer *inbuf, GstBuffer *outbut)
{
	GST_DEBUG("gst_nxscaler_transfrom  \n");
	return GST_FLOW_OK;
}

static gboolean
gst_nxscaler_start (GstBaseTransform *trans)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);
	gboolean ret = TRUE;
	GST_DEBUG_OBJECT(scaler,"gst_nxscaler_start \n");
	_set_caps_init(scaler);
	scaler->scaler_fd = scaler_open();
	if (scaler->scaler_fd < 0) {
		GST_ERROR_OBJECT(scaler, "failed to open scaler");
		return FALSE;
	}
	scaler->buffer_count = DEF_BUFFER_COUNT;
	scaler->buffer_size = _calc_alloc_size(scaler->dst_width, scaler->dst_height);
	GST_DEBUG_OBJECT(scaler,"buffer_size = %d \n", scaler->buffer_size);
	ret = _create_buffer(scaler);
	if (ret == FALSE)
		GST_ERROR_OBJECT(scaler, "failed to create buffer");

	return ret;
}

static gboolean
gst_nxscaler_stop (GstBaseTransform *trans)
{
	GstNxScaler *scaler = GST_NXSCALER(trans);

	GST_DEBUG_OBJECT(scaler,"ENTERED\n");
	if(scaler->src_caps)
		gst_caps_unref(scaler->src_caps);
	if (scaler->buffer_type == BUFFER_TYPE_NORMAL)
		_unmap_buffer(scaler);
	_destroy_buffer(scaler);
	nx_scaler_close(scaler->scaler_fd);
	GST_DEBUG_OBJECT(scaler,"LEAVED\n");
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

	gst_element_class_set_details_simple(gstelement_class,
	"Nexell Scaler GStreamer Plug-in",
	"Filter/Video/Scaler",
	"scale the video stream based GStreamer Plug-in",
	"Hyejung Kwon <<cjscld15@nexell.co.kr>>");

	gst_element_class_add_pad_template (gstelement_class,
	gst_static_pad_template_get (&src_factory));
	gst_element_class_add_pad_template (gstelement_class,
	gst_static_pad_template_get (&sink_factory));
	gobject_class->set_property = gst_nx_scaler_set_property;
	gobject_class->get_property = gst_nx_scaler_get_property;
	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_X,
					g_param_spec_uint("scaler-crop-x",
							  "Crop X",
							  "X value for crop",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_Y,
					g_param_spec_uint("scaler-crop-y",
							  "Crop Y",
							  "Y value for crop",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));
	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_WIDTH,
					g_param_spec_uint("scaler-crop-width",
							  "Crop WIDTH",
							  "WIDTH value for crop",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_CROP_HEIGHT,
					g_param_spec_uint("scaler-crop-height",
							  "Crop HEIGHT",
							  "HEIGHT value for crop",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_DST_WIDTH,
					g_param_spec_uint("scaler-dst-width",
							  "dst WIDTH",
							  "WIDTH value for display",
							  0,
							  MAX_RESOLUTION_X,
							  0,
							  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_SCALER_DST_HEIGHT,
					g_param_spec_uint("scaler-dst-height",
							  "dst HEIGHT",
							  "HEIGHT value for display",
							  0,
							  MAX_RESOLUTION_Y,
							  0,
							  G_PARAM_READWRITE));
	g_object_class_install_property(gobject_class,
					ARG_BUFFER_TYPE,
					g_param_spec_uint("buffer-type",
							  "buffer-type",
							  "Buffer Type(0:NORMAL 1:MM_VIDEO_BUFFER_TYPE_GEM)",
							  0,
							  1,
							  BUFFER_TYPE_GEM,
							  G_PARAM_READWRITE));

	base_transform_class->start = GST_DEBUG_FUNCPTR(gst_nxscaler_start);
	base_transform_class->stop = GST_DEBUG_FUNCPTR(gst_nxscaler_stop);
	base_transform_class->accept_caps = GST_DEBUG_FUNCPTR(gst_nxscaler_accept_caps);
	base_transform_class->fixate_caps = GST_DEBUG_FUNCPTR(gst_nxscaler_fixate_caps);
	base_transform_class->prepare_output_buffer = GST_DEBUG_FUNCPTR(gst_nxscaler_prepare_output_buffer);
	base_transform_class->transform = GST_DEBUG_FUNCPTR(gst_nxscaler_transform);
	base_transform_class->passthrough_on_same_caps = FALSE;
	base_transform_class->transform_ip_on_passthrough = FALSE;
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure:
 */
static void
gst_nx_scaler_init (GstNxScaler * scaler)
{
	int i;

	GST_DEBUG_OBJECT(scaler, "gst_nx_scaler_init \n");

	scaler->silent = FALSE;

	scaler->dst_width = DEFAULT_RESOLUTION_X;
	scaler->dst_height = DEFAULT_RESOLUTION_Y;

	scaler->crop_x = 0;
	scaler->crop_y = 0;

	scaler->crop_width = MINIMUM_CROP_X;
	scaler->crop_height = MINIMUM_CROP_Y;

	scaler->src_caps = NULL;

	scaler->scaler_fd = -1;
	scaler->buffer_size = 0;
	scaler->buffer_count = 0;
	scaler->buffer_index = 0;
	scaler->src_fd = -1;

	scaler->buffer_type = BUFFER_TYPE_GEM;

#ifdef USE_NATIVE_DRM_BUFFER
	scaler->drm_fd = -1;
	for (i = 0; i < MAX_OUT_BUFFER_COUNT; i++) {
		scaler->gem_fds[i] = -1;
		scaler->dma_fds[i] = -1;
		scaler->flinks[i] = -1;
	}
	for (i = 0; i < MAX_IN_BUFFER_COUNT; i++) {
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
	case ARG_BUFFER_TYPE:
		scaler->buffer_type = g_value_get_uint(value);
		GST_INFO_OBJECT(scaler, "Set buffer_type: %u",
				scaler->buffer_type);
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
	GstNxScaler *scaler = GST_NXSCALER (object);

	GST_INFO_OBJECT(scaler, "gst_nx_scaler_get_property ");

	switch (prop_id) {
        case ARG_SCALER_CROP_X:
               g_value_set_uint(value, scaler->crop_x);
                break;
        case ARG_SCALER_CROP_Y:
               g_value_set_uint(value, scaler->crop_y);
                break;
        case ARG_SCALER_CROP_WIDTH:
               g_value_set_uint(value, scaler->crop_width);
                break;
        case ARG_SCALER_CROP_HEIGHT:
               g_value_set_uint(value, scaler->crop_height);
                break;
        case ARG_SCALER_DST_WIDTH:
               g_value_set_uint(value, scaler->dst_width);
                break;
        case ARG_SCALER_DST_HEIGHT:
               g_value_set_uint(value, scaler->dst_height);
                break;
	case ARG_BUFFER_TYPE:
		g_value_set_uint(value, scaler->buffer_type);
		break;
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
