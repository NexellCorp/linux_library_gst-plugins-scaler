/*
 * libmm-common
 *
 * Copyright (c) 2000 - 2011 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * Contact: Jonghyuk Choi <jhchoi.choi@samsung.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/**
 * This file declares common data structure of multimedia framework.
 *
 * @file		mm_types.h
 * @author
 * @version		1.0
 * @brief		This file declares common data structure of multimedia framework.
 */

/**
        @addtogroup COMMON
        @{
 */

#ifndef __MM_TYPES_H__
#define	__MM_TYPES_H__

#include <stdbool.h>


#ifdef __cplusplus
	extern "C" {
#endif


#define MM_MAX_FILENAME_LEN	256			/**< Maximum length of the file name */
#define MM_MAX_URL_LEN		2048	/**< Maximum length of the maximum URL */

/**
 * @def Defines range of logical volume factor.
 * It's for logical volume control in player.
 * So, don't use this one for media volume.
 * range : 0 ~ 1.0 (1.0 = 100%)
 */
#define MM_VOLUME_LEVEL_MIN	0	/**< Minimum factor of volume */
#define MM_VOLUME_LEVEL_MAX	1.0	/**< Maximum factor of volume */

/**
 * Multimedia Framework handle type.
 */
typedef void *MMHandleType;		/**< Handle type */


/**
 * Enumerations of video capture devices.
 */
enum MMVideoDeviceType {
	MM_VIDEO_DEVICE_NONE = -1,	/**< No camera */
	MM_VIDEO_DEVICE_CAMERA0,	/**< Primary camera */
	MM_VIDEO_DEVICE_CAMERA1,	/**< Secondary camera */
	MM_VIDEO_DEVICE_NUM,		/**< Number of video capture devices */
};

/**
 * Enumerations of audio capture devices.
 */
enum MMAudioDeviceType {
	MM_AUDIO_DEVICE_MIC,	/**< Mic device */
	MM_AUDIO_DEVICE_MODEM,	/**< Modem */
	MM_AUDIO_DEVICE_RADIO,	/**< Radio */
	MM_AUDIO_DEVICE_NUM,	/**< Number of audio capture devices */
};

/**
 * Enumerations of display surfaces.
 */
typedef enum {
	MM_DISPLAY_SURFACE_OVERLAY,            /**< Overlay surface - default */
	MM_DISPLAY_SURFACE_EVAS,               /**< Evas object surface */
	MM_DISPLAY_SURFACE_GL,                 /**< GL surface */
	MM_DISPLAY_SURFACE_NULL,               /**< This just disposes of buffers */
	MM_DISPLAY_SURFACE_REMOTE,             /**< Daemon client surface */
	MM_DISPLAY_SURFACE_NUM,                /**< number of enum */
} MMDisplaySurfaceType;

/**
 * Enumerations of display modes.
 */
enum MMDisplayModeType {
	MM_DISPLAY_MODE_DEFAULT,                                        /**< default - primary display(UI+Video) + secondary display(UI+Video) if available */
	MM_DISPLAY_MODE_PRI_VIDEO_ON_AND_SEC_VIDEO_FULL_SCREEN,         /**< primary display(UI+Video) + secondary display(Video Full Screen) if available */
	MM_DISPLAY_MODE_PRI_VIDEO_OFF_AND_SEC_VIDEO_FULL_SCREEN,        /**< primary display(UI only) + secondary display(Video Full Screen) if available */
	MM_DISPLAY_MODE_NUM,                                            /**< number of enum */
};

/**
 * Enumerations of display devices.
 */
enum MMDisplayDeviceType {
	MM_DISPLAY_DEVICE_MAINLCD,             /**< Main LCD - default */
	MM_DISPLAY_DEVICE_SUBLCD,              /**< Sub LCD */
	MM_DISPLAY_DEVICE_TVOUT,               /**< TV out */
	MM_DISPLAY_DEVICE_MAINLCD_AND_SUBLCD,  /**< Main LCD and Sub LCD */
	MM_DISPLAY_DEVICE_MAINLCD_AND_TVOUT,   /**< Main LCD and TV */
	MM_DISPLAY_DEVICE_NUM,                 /**< number of enum */
};

/**
 * Enumerations of display scales.
 */
enum MMDisplayScaleType {
	MM_DISPLAY_SCALE_DEFAULT,		/**< Default size */
	MM_DISPLAY_SCALE_DOUBLE_LENGTH,	/**< Expand width, height to double size */
	MM_DISPLAY_SCALE_TRIPLE_LENGTH,	/**< Expand width, height to triple size */
	MM_DISPLAY_SCALE_NUM,			/**< Number of display scales */
};

/**
 * Enumerations of storage.
 */
enum MMStorageType {
	MM_STORAGE_INTERNAL,			/**< Internal storage */
	MM_STORAGE_EXTERNAL,			/**< External storage such as memory card */
	MM_STORAGE_NUM,					/**< Number of storage type */
};

/**
 * Enumerations for quality.
 */
enum MMQualityType {
	MM_QUALITY_LOW,		/**< Low quality */
	MM_QUALITY_HIGH,	/**< High quality */
	MM_QUALITY_NUM,		/**< Number of quality type */
};

/**
 * Enumerations of channel type.
 */
enum MMVolumeChannelType {
	MM_VOLUME_CHANNEL_LEFT,		/**< Left channel */
	MM_VOLUME_CHANNEL_RIGHT,		/**< Right channel */
	MM_VOLUME_CHANNEL_NUM,		/**< Numbers of channels */
	MM_CHANNEL_LEFT = MM_VOLUME_CHANNEL_LEFT,		/**< Left channel */
	MM_CHANNEL_RIGHT = MM_VOLUME_CHANNEL_RIGHT,		/**< Right channel */
	MM_CHANNEL_NUM = MM_VOLUME_CHANNEL_NUM,		/**< Numbers of channels */
};

/*
 * Enumerations of SW Volume Table Type.
 */
enum MMSoftwareVolumeTableType {
	MM_SOUND_VOLUME_TYPE_SYSTEM,		/**< Volume table for System */
	MM_SOUND_VOLUME_TYPE_NOTIFICATION,	/**< Volume table for Notification */
	MM_SOUND_VOLUME_TYPE_ALARM,			/**< Volume table for Alarm */
	MM_SOUND_VOLUME_TYPE_RINGTONE,		/**< Volume table for Ringtone */
	MM_SOUND_VOLUME_TYPE_MEDIA,			/**< Volume table for Multimedia */
	MM_SOUND_VOLUME_TYPE_CALL,			/**< Volume table for Call */
	MM_SOUND_VOLUME_TYPE_VOIP,			/**< Volume table for VOIP */
	MM_SOUND_VOLUME_TYPE_VOICE,			/**< Volume table for VOICE */
	MM_SOUND_VOLUME_TYPE_SVOICE,			/**< Volume table for SVOICE */
	MM_SOUND_VOLUME_TYPE_EMERGENCY,		/**< Volume table for Emergency (FIXED) */
	MM_SOUND_VOLUME_TYPE_NUM,
	MM_SOUND_VOLUME_TABLE_NUM = MM_SOUND_VOLUME_TYPE_NUM,
};

/*
 * Enumerations of SW Volume Gain.
 */
enum MMSoftwareVolumeGainType {
	MM_SOUND_VOLUME_GAIN_DEFAULT	= 0,		/**< Volume gain for Default */
	MM_SOUND_VOLUME_GAIN_DIALER		= 1<<8,		/**< Volume gain for Dialer */
	MM_SOUND_VOLUME_GAIN_TOUCH		= 2<<8,		/**< Volume gain for Touch */
	MM_SOUND_VOLUME_GAIN_AF			= 3<<8,		/**< Volume gain for AF */
	MM_SOUND_VOLUME_GAIN_SHUTTER1	= 4<<8,		/**< Volume gain for Shutter1 */
	MM_SOUND_VOLUME_GAIN_SHUTTER2	= 5<<8,		/**< Volume gain for Shutter2 */
	MM_SOUND_VOLUME_GAIN_CAMCORDING	= 6<<8,		/**< Volume gain for Camcording */
	MM_SOUND_VOLUME_GAIN_MIDI		= 7<<8,		/**< Volume gain for Midi */
	MM_SOUND_VOLUME_GAIN_BOOTING	= 8<<8,		/**< Volume gain for Booting */
	MM_SOUND_VOLUME_GAIN_VIDEO		= 9<<8,		/**< Volume gain for Video */
	MM_SOUND_VOLUME_GAIN_TTS		= 10<<8,	/**< Volume gain for TTS */
	MM_SOUND_VOLUME_GAIN_NUM
};

#define MM_SOUND_PRIORITY_ALLOW_MIX	(0x80) /**< This define has deprecated */
/*
 * Enumerations of Sound Priority.
 */
enum MMSoundPriorityType {
	MM_SOUND_PRIORITY_0,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_1,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_2,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_3,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_4,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_5,		/**< This enum has deprecated */
	MM_SOUND_PRIORITY_NUM,
};

/**
 * Enumerations of video codec.
 */
enum MMVideoCodecType {
	MM_VIDEO_CODEC_INVALID = -1,	/**< Invalid codec type */
	MM_VIDEO_CODEC_NONE,			/**< None (will be deprecated) */
	MM_VIDEO_CODEC_H263,			/**< H263 codec		*/
	MM_VIDEO_CODEC_H264,			/**< H264 codec		*/
	MM_VIDEO_CODEC_H26L,			/**< H26L codec		*/
	MM_VIDEO_CODEC_MPEG4,			/**< MPEG4 codec	*/
	MM_VIDEO_CODEC_MPEG1,			/**< MPEG1 codec	*/
	MM_VIDEO_CODEC_WMV,			/**< WMV codec		*/
	MM_VIDEO_CODEC_DIVX,			/**< DIVX codec		*/
	MM_VIDEO_CODEC_XVID,			/**< XVID codec		*/
	MM_VIDEO_CODEC_H261,				/**< H261 codec										*/
	MM_VIDEO_CODEC_H262,				/**< H262/MPEG2-part2 codec							*/
	MM_VIDEO_CODEC_H263V2,				/**< H263v2 codec									*/
	MM_VIDEO_CODEC_H263V3,				/**< H263v3 codec									*/
	MM_VIDEO_CODEC_MJPEG,				/**< Motion JPEG Video codec						*/
	MM_VIDEO_CODEC_MPEG2,				/**< MPEG2 codec									*/
	MM_VIDEO_CODEC_MPEG4_SIMPLE,        /**< MPEG4 part-2 Simple profile codec				*/
	MM_VIDEO_CODEC_MPEG4_ADV_SIMPLE,    /**< MPEG4 part-2 Advanced Simple profile codec		*/
	MM_VIDEO_CODEC_MPEG4_MAIN,			/**< MPEG4 part-2 Main profile codec				*/
	MM_VIDEO_CODEC_MPEG4_CORE,			/**< MPEG4 part-2 Core profile codec				*/
	MM_VIDEO_CODEC_MPEG4_ACE,			/**< MPEG4 part-2 Adv Coding Eff profile codec		*/
	MM_VIDEO_CODEC_MPEG4_ARTS,			/**< MPEG4 part-2 Adv RealTime Simple profile codec	*/
	MM_VIDEO_CODEC_MPEG4_AVC,			/**< MPEG4 part-10 (h.264) codec					*/
	MM_VIDEO_CODEC_REAL,				/**< Real video										*/
	MM_VIDEO_CODEC_VC1,					/**< VC-1 video										*/
	MM_VIDEO_CODEC_AVS,					/**< AVS video										*/
	MM_VIDEO_CODEC_CINEPAK,				/**< Cinepak video	codec							*/
	MM_VIDEO_CODEC_INDEO,				/**< Indeo video	codec							*/
	MM_VIDEO_CODEC_THEORA,				/**< Theora video	codec							*/
	MM_VIDEO_CODEC_FLV,					/**< Adobe Flash Video	codec						*/
	MM_VIDEO_CODEC_NUM,			/**< Number of video codec type	*/
};

/**
 * Enumerations of audio codec.
 */
enum MMAudioCodecType {
	MM_AUDIO_CODEC_INVALID = -1,	/**< Invalid codec type */
	MM_AUDIO_CODEC_AMR,			/**< AMR codec		*/
	MM_AUDIO_CODEC_G723_1,			/**< G723.1 codec	*/
	MM_AUDIO_CODEC_MP3,			/**< MP3 codec		*/
	MM_AUDIO_CODEC_OGG,			/**< OGG codec		*/
	MM_AUDIO_CODEC_AAC,			/**< AAC codec		*/
	MM_AUDIO_CODEC_WMA,			/**< WMA codec		*/
	MM_AUDIO_CODEC_MMF,			/**< MMF codec		*/
	MM_AUDIO_CODEC_ADPCM,			/**< ADPCM codec	*/
	MM_AUDIO_CODEC_WAVE,			/**< WAVE codec		*/
	MM_AUDIO_CODEC_WAVE_NEW,			/**< WAVE codec		*/
	MM_AUDIO_CODEC_MIDI,			/**< MIDI codec		*/
	MM_AUDIO_CODEC_IMELODY,		/**< IMELODY codec	*/
	MM_AUDIO_CODEC_MXMF,
	MM_AUDIO_CODEC_MPA,			/**< MPEG1-Layer1 codec						*/
	MM_AUDIO_CODEC_MP2,			/**< MPEG1-Layer2 codec						*/
	MM_AUDIO_CODEC_G711,		/**< G711 codec								*/
	MM_AUDIO_CODEC_G722,		/**< G722 wideband speech codec				*/
	MM_AUDIO_CODEC_G722_1,		/**< G722.1 codec							*/
	MM_AUDIO_CODEC_G722_2,		/**< G722.2  (AMR-WB) codec					*/
	MM_AUDIO_CODEC_G723,		/**< G723 wideband speech codec				*/
	MM_AUDIO_CODEC_G726,		/**< G726 (ADPCM) codec						*/
	MM_AUDIO_CODEC_G728,		/**< G728 speech codec						*/
	MM_AUDIO_CODEC_G729,		/**< G729 codec								*/
	MM_AUDIO_CODEC_G729A,		/**< G729a codec							*/
	MM_AUDIO_CODEC_G729_1,		/**< G729.1 codec							*/
	MM_AUDIO_CODEC_REAL,        /**< Real audio 							*/
	MM_AUDIO_CODEC_AAC_LC,		/**< AAC-Low complexity codec				*/
	MM_AUDIO_CODEC_AAC_MAIN,	/**< AAC-Main profile codec					*/
	MM_AUDIO_CODEC_AAC_SRS,		/**< AAC-Scalable sample rate codec			*/
	MM_AUDIO_CODEC_AAC_LTP,		/**< AAC-Long term prediction codec			*/
	MM_AUDIO_CODEC_AAC_HE_V1,	/**< AAC-High Efficiency v1 codec			*/
	MM_AUDIO_CODEC_AAC_HE_V2,	/**< AAC-High efficiency v2 codec			*/
	MM_AUDIO_CODEC_AC3,			/**< DolbyDigital codec						*/
	MM_AUDIO_CODEC_ALAC,		/**< Apple Lossless audio codec				*/
	MM_AUDIO_CODEC_ATRAC,		/**< Sony proprietary audio codec			*/
	MM_AUDIO_CODEC_SPEEX,		/**< SPEEX audio codec						*/
	MM_AUDIO_CODEC_VORBIS,		/**< Vorbis audio codec						*/
	MM_AUDIO_CODEC_AIFF,        /**< AIFF codec								*/
	MM_AUDIO_CODEC_AU,	        /**< AU codec								*/
	MM_AUDIO_CODEC_NONE,			/**< None (will be deprecated) */
	MM_AUDIO_CODEC_PCM,			/**< PCM codec */
	MM_AUDIO_CODEC_ALAW,			/**< ALAW codec */
	MM_AUDIO_CODEC_MULAW,			/**< MULAW codec */
	MM_AUDIO_CODEC_MS_ADPCM,	/**< MS ADPCM codec */
	MM_AUDIO_CODEC_FLAC,			/**< Free Lossless audio codec				*/
	MM_AUDIO_CODEC_NUM,			/**< Number of audio codec type	*/
};

/**
 * Enumerations of image codec.
 */
enum MMImageCodecType {
	MM_IMAGE_CODEC_INVALID = -1,/**< Invalid codec type */
	MM_IMAGE_CODEC_JPEG,        /**< JPEG codec */
	MM_IMAGE_CODEC_PNG,         /**< PNG codec		*/
	MM_IMAGE_CODEC_BMP,         /**< BMP codec		*/
	MM_IMAGE_CODEC_WBMP,        /**< WBMP codec		*/
	MM_IMAGE_CODEC_TIFF,        /**< TIFF codec		*/
	MM_IMAGE_CODEC_PCX,         /**< PCX codec		*/
	MM_IMAGE_CODEC_GIF,         /**< GIF codec		*/
	MM_IMAGE_CODEC_ICO,         /**< ICO codec		*/
	MM_IMAGE_CODEC_RAS,         /**< RAS codec		*/
	MM_IMAGE_CODEC_TGA,         /**< TGA codec		*/
	MM_IMAGE_CODEC_XBM,         /**< XBM codec		*/
	MM_IMAGE_CODEC_XPM,         /**< XPM codec		*/
	MM_IMAGE_CODEC_SRW,         /**< SRW (Samsung standard RAW) */
	MM_IMAGE_CODEC_JPEG_SRW,    /**< JPEG + SRW */
	MM_IMAGE_CODEC_NUM,         /**< Number of image codecs */
};

/**
 * Enumerations of file container format.
 */
enum MMFileFormatType {
        MM_FILE_FORMAT_INVALID = -1,/**< Invalid file format */
        MM_FILE_FORMAT_3GP,                     /**< 3GP file format */
        MM_FILE_FORMAT_ASF,                     /**< Advanced Systems File file format */
        MM_FILE_FORMAT_AVI,                     /**< Audio Video Interleaved file format */
        MM_FILE_FORMAT_MATROSKA,        /**< MATROSAK file format */
        MM_FILE_FORMAT_MP4,                     /**< MP4 file format */
        MM_FILE_FORMAT_OGG,                     /**< OGG file format */
        MM_FILE_FORMAT_NUT,                     /**< NUT file format */
        MM_FILE_FORMAT_QT,                      /**< MOV file format */
        MM_FILE_FORMAT_REAL,            /**< RealMedia file format */
        MM_FILE_FORMAT_AMR,                     /**< AMR file format */
        MM_FILE_FORMAT_AAC,                     /**< AAC file format */
        MM_FILE_FORMAT_MP3,                     /**< MP3 file format */
        MM_FILE_FORMAT_AIFF,            /**< AIFF file format */
        MM_FILE_FORMAT_AU,                      /**< Audio file format */
        MM_FILE_FORMAT_WAV,                     /**< WAV file format */
        MM_FILE_FORMAT_MID,                     /**< MID file format */
        MM_FILE_FORMAT_MMF,                     /**< MMF file format */
        MM_FILE_FORMAT_DIVX,            /**< DivX file format */
        MM_FILE_FORMAT_FLV,                     /**< Flash video file format */
        MM_FILE_FORMAT_VOB,                     /**< DVD-Video Object file format */
        MM_FILE_FORMAT_IMELODY,         /**< IMelody file format */
        MM_FILE_FORMAT_WMA,             /**< WMA file format */
        MM_FILE_FORMAT_WMV,             /**< WMV file format */
        MM_FILE_FORMAT_JPG,             /**< JPEG file format */
        MM_FILE_FORMAT_FLAC,            /**< FLAC file format */
        MM_FILE_FORMAT_M2TS,            /**< MPEG2-Transport Stream file format */
        MM_FILE_FORMAT_M2PS,			/**< MPEG2-Program Stream file format */
        MM_FILE_FORMAT_M1VIDEO,		/**< MPEG1-Video file format */
        MM_FILE_FORMAT_M1AUDIO,		/**< MPEG1-Audio file format */
        MM_FILE_FORMAT_NUM,             /**< Number of file format type */
};

/**
 * Enumerations of display layers.
 */
typedef enum {
	MM_VIDEO_LAYER_PRIMARY,		/**< Primary of the video layer */
	MM_VIDEO_LAYER_OVERLAY,		/**< Overlay of the video layer */
	MM_VIDEO_LAYER_NUM		/**< Number of the video layer */
} MMVideoLayerType;

/**
 * Enumerations of Pixel formats
 */
typedef enum {
	MM_PIXEL_FORMAT_INVALID	= -1,   /**< Invalid pixel format */
	MM_PIXEL_FORMAT_NV12,           /**< NV12 pixel format */
	MM_PIXEL_FORMAT_NV12T,          /**< NV12 Tiled pixel format */
	MM_PIXEL_FORMAT_NV16,           /**< NV16 pixel format */
	MM_PIXEL_FORMAT_NV21,           /**< NV21 pixel format */
	MM_PIXEL_FORMAT_YUYV,           /**< YUYV(YUY2) pixel format */
	MM_PIXEL_FORMAT_UYVY,           /**< UYVY pixel format */
	MM_PIXEL_FORMAT_422P,           /**< YUV422(Y:U:V) planar pixel format */
	MM_PIXEL_FORMAT_I420,           /**< I420 pixel format */
	MM_PIXEL_FORMAT_YV12,           /**< YV12 pixel format */
	MM_PIXEL_FORMAT_RGB565,         /**< RGB565 pixel format */
	MM_PIXEL_FORMAT_RGB888,         /**< RGB888 pixel format */
	MM_PIXEL_FORMAT_RGBA,           /**< RGBA pixel format */
	MM_PIXEL_FORMAT_ARGB,           /**< ARGB pixel format */
	MM_PIXEL_FORMAT_ENCODED,        /**< Encoded pixel format : JPEG */
	MM_PIXEL_FORMAT_ITLV_JPEG_UYVY,	/**< FIXME: JPEG+UYVY Interleaved format */
	MM_PIXEL_FORMAT_ENCODED_H264,	/**< Encoded pixel format : H.264 */
	MM_PIXEL_FORMAT_NUM		/**< Number of the pixel format */
} MMPixelFormatType;

/**
 * Enumerations of video input rotation type.
 */
typedef enum {
	MM_VIDEO_INPUT_ROTATION_NONE,	/**< No rotation of the display */
	MM_VIDEO_INPUT_ROTATION_90,		/**< 90 degree rotation */
	MM_VIDEO_INPUT_ROTATION_180,	/**< 180 degree rotation */
	MM_VIDEO_INPUT_ROTATION_270,	/**< 270 degree rotation */
	MM_VIDEO_INPUT_ROTATION_NUM		/**< Number of the rotation */
} MMVideoInputRotationType;

/**
 * Enumerations of display rotation type.
 */
typedef enum {
	MM_DISPLAY_ROTATION_NONE,	/**< No rotation of the display */
	MM_DISPLAY_ROTATION_90,		/**< 90 degree rotation */
	MM_DISPLAY_ROTATION_180,	/**< 180 degree rotation */
	MM_DISPLAY_ROTATION_270,	/**< 270 degree rotation */
	MM_DISPLAY_ROTATION_NUM		/**< Number of the rotation */
} MMDisplayRotationType;

/**
 * Enumerations of flip type.
 */
typedef enum {
	MM_FLIP_NONE,                   /**< No Flip */
	MM_FLIP_HORIZONTAL,             /**< Horizontal flip */
	MM_FLIP_VERTICAL,               /**< Vertical flip */
	MM_FLIP_BOTH,                   /**< Horizontal and Vertical flip */
	MM_FLIP_NUM                     /**< Number of flip */
} MMFlipType;

/**
 * Enumerations of streaming type.
 */
typedef enum {
	STREAMING_SERVICE_VOD,			/**< Streaming is vod */
	STREAMING_SERVICE_LIVE,			/**< Streaming is live stream */
	STREAMING_SERVICE_NONE,			/**< Not an streaming */
	STREAMING_SERVICE_NUM,			/**< Number of the streaming type */
} MMStreamingType;

/**
 * Resolution
 */
typedef struct {
	int width;		/**< width */
	int height;		/**< height */
} MMResolutionType;

/**
 * Type definition of rectangle.
 * This will be deprecated.
 */
typedef struct {
	unsigned int x;		/**< Start x point */
	unsigned int y;		/**< Start y point */
	unsigned int width;	/**< width */
	unsigned int height;	/**< Height */
} MMRectType;

#define BT_ADDR_LEN 18		/**< Length of BT address */
/**
 * bluetooth information.
 */
typedef struct {
	int mode;						/**< BT enable/disable */
	char addr[BT_ADDR_LEN];			/**< BT device address */
} MMBluetoothType;


/**
 * Enumerations of bluetooth mode
 */
enum MMBluetoothMode {
	MM_BLUETOOTH_DISABLE = 0,
	MM_BLUETOOTH_ENABLE
};

/*
 * Enumerations of sound path policy
 */
enum MMAudioRoutePolicy {
	MM_AUDIOROUTE_USE_EXTERNAL_SETTING = -1,
	MM_AUDIOROUTE_PLAYBACK_NORMAL,
	MM_AUDIOROUTE_PLAYBACK_ALERT,
	MM_AUDIOROUTE_PLAYBACK_HEADSET_ONLY,
	MM_AUDIOROUTE_CAPTURE_NORMAL = 0,
	MM_AUDIOROUTE_CAPTURE_MAINMIC_ONLY,
	MM_AUDIOROUTE_CAPTURE_STEREOMIC_ONLY,
};

/*
 * Enumerations of display geometry method
 */
typedef enum  {
	MM_DISPLAY_METHOD_LETTER_BOX = 0,
	MM_DISPLAY_METHOD_ORIGIN_SIZE,
	MM_DISPLAY_METHOD_FULL_SCREEN,
	MM_DISPLAY_METHOD_CROPPED_FULL,
	MM_DISPLAY_METHOD_ORIGIN_OR_LETTER,
	MM_DISPLAY_METHOD_CUSTOM_ROI,
}MMDisplayGeometryMethod;

/*
 * Enumerations of ROI mode of display geometry method
 */
typedef enum  {
	MM_DISPLAY_METHOD_CUSTOM_ROI_FULL_SCREEN = 0,
	MM_DISPLAY_METHOD_CUSTOM_ROI_LETER_BOX
}MMDisplayGeometryMethodRoiMode;


#define MM_VIDEO_BUFFER_PLANE_MAX   4	/**< Max num of video buffer plane */
/*
 * Enumerations of multimedia video buffer type
 */
typedef enum {
	MM_VIDEO_BUFFER_TYPE_PHYSICAL_ADDRESS = 0,
	MM_VIDEO_BUFFER_TYPE_DMABUF_FD,
	MM_VIDEO_BUFFER_TYPE_TBM_BO,
	MM_VIDEO_BUFFER_TYPE_GEM,
} MMVideoBufferType;

typedef struct {
	void *paddr[MM_VIDEO_BUFFER_PLANE_MAX];         /**< physical address */
	int dmabuf_fd[MM_VIDEO_BUFFER_PLANE_MAX];       /**< dmabuf fd */
	void *bo[MM_VIDEO_BUFFER_PLANE_MAX];            /**< TBM bo */
	int gem[MM_VIDEO_BUFFER_PLANE_MAX];		/**< GEM handle */
} MMVideoBufferHandle;

/*
 * Type definition of multimedia video buffer
 */
typedef struct {
	MMVideoBufferType type;                                 /**< buffer type
                                                                     - The field of handle that type indicates should be filled,
                                                                       and other fields of handle are optional. */
	MMPixelFormatType format;                               /**< buffer type */
	int plane_num;                                          /**< number of planes */
	int width[MM_VIDEO_BUFFER_PLANE_MAX];                   /**< width of buffer */
	int height[MM_VIDEO_BUFFER_PLANE_MAX];                  /**< height of buffer */
	int stride_width[MM_VIDEO_BUFFER_PLANE_MAX];            /**< stride width of buffer */
	int stride_height[MM_VIDEO_BUFFER_PLANE_MAX];           /**< stride height of buffer */
	int size[MM_VIDEO_BUFFER_PLANE_MAX];                    /**< size of planes */
	void *data[MM_VIDEO_BUFFER_PLANE_MAX];                  /**< data pointer(user address) of planes */
	int handle_num;                                         /**< number of buffer handle */
	int handle_size[MM_VIDEO_BUFFER_PLANE_MAX];             /**< size of handles */
	MMVideoBufferHandle handle;                             /**< handle of buffer */
	int is_secured;                                         /**< secured buffer flag. ex) TrustZone memory, user can not access it. */
	int flush_request;                                      /**< flush request flag
                                                                     - If this flag is TRUE, sink element will make copy of last buffer,
                                                                       and it will return all buffers from src element.
                                                                       Then, src element can restart without changing pipeline state. */
	int buffer_index;
} MMVideoBuffer;


#ifdef __cplusplus
	}
#endif


#endif	/* __MM_TYPES_H__ */
