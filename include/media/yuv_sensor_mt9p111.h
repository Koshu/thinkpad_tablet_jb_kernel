#ifndef ___YUV5_SENSOR_H__
#define ___YUV5_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/*-------------------------------------------Important---------------------------------------
 * for changing the SENSOR_NAME, you must need to change the owner of the device. For example
 * Please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * -------------------------------------------Important--------------------------------------
*/

#define SENSOR_5M_NAME  "mt9p111"
#define DEV_5M(x)          "/dev/"x
#define SENSOR_5M_PATH     DEV_5M(SENSOR_5M_NAME)
#define LOG_5M_NAME(x)     "ImagerODM-"x
#define LOG_TAG_5M         LOG_5M_NAME(SENSOR_5M_NAME)

#define SENSOR_5M_WAIT_MS       0   /* special number to indicate this is wait time require */
#define SENSOR_5M_TABLE_END     1   /* special number to indicate this is end of table */
#define WRITE_REG_DATA8         2   /* special the data width as one byte */
#define WRITE_REG_DATA16        3   /* special the data width as one byte */
#define POLL_REG_BIT            4   /* poll the bit set */

#define SENSOR_5M_MAX_RETRIES   3      /* max counter for retry I2C access */
#define SENSOR_5M_POLL_RETRIES  20000  /* max poll retry */
#define SENSOR_5M_POLL_WAITMS   1      /* poll wait time */

#define SENSOR_5M_IOCTL_SET_MODE           _IOW('o', 1, struct sensor_5m_mode)
#define SENSOR_5M_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define SENSOR_5M_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define SENSOR_5M_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define SENSOR_5M_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define SENSOR_5M_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define SENSOR_5M_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)
#define SENSOR_5M_IOCTL_SET_EXPOSURE       _IOW('o', 8, __u8)
#define SENSOR_5M_IOCTL_SET_FLASH_MODE     _IOW('o', 9, __u8)
#define SENSOR_5M_IOCTL_CAPTURE_CMD        _IOW('o', 10, __u8)
#define SENSOR_5M_IOCTL_GET_BRIGHTNESS     _IOW('o', 11, __u16)
#define SENSOR_5M_IOCTL_SET_OVERRIDE_TABLE _IOW('o',21, struct sensor_override_table)
#define SENSOR_5M_IOCTL_GET_CAPTURE_FRAME_RATE _IOR('o',22, __u8)
/* Compal Indigo-Carl ++ */
// focus window for touch focus
#define SENSOR_5M_IOCTL_SET_FOCUS_WINDOW   _IOW('o', 23, struct yuv5_focus_rect)
/* Compal Indigo-Carl -- */

//Sandow++ 0412 for new NV release
enum {
      IsYUVSensor = 1,
      Brightness,
} ;

enum {
	YUV_5M_FlashControlOn = 0,
	YUV_5M_FlashControlOff,
	YUV_5M_FlashControlAuto,
	YUV_5M_FlashControlRedEyeReduction,
	YUV_5M_FlashControlFillin,
	YUV_5M_FlashControlTorch,
};

enum {
	YUV_5M_ColorEffect = 0,
	YUV_5M_Whitebalance,
	YUV_5M_SceneMode,
	YUV_5M_Exposure,
	YUV_5M_FlashMode
};

enum {
	YUV_5M_ColorEffect_Invalid = 0,
	YUV_5M_ColorEffect_Aqua,
	YUV_5M_ColorEffect_Blackboard,
	YUV_5M_ColorEffect_Mono,
	YUV_5M_ColorEffect_Negative,
	YUV_5M_ColorEffect_None,
	YUV_5M_ColorEffect_Posterize,
	YUV_5M_ColorEffect_Sepia,
	YUV_5M_ColorEffect_Solarize,
	YUV_5M_ColorEffect_Whiteboard
};

enum {
	YUV_5M_Whitebalance_Invalid = 0,
	YUV_5M_Whitebalance_Auto,
	YUV_5M_Whitebalance_Incandescent,
	YUV_5M_Whitebalance_Fluorescent,
	YUV_5M_Whitebalance_WarmFluorescent,
	YUV_5M_Whitebalance_Daylight,
	YUV_5M_Whitebalance_CloudyDaylight,
	YUV_5M_Whitebalance_Shade,
	YUV_5M_Whitebalance_Twilight,
	YUV_5M_Whitebalance_Custom
};

enum {
	YUV_5M_SceneMode_Invalid = 0,
	YUV_5M_SceneMode_Auto,
	YUV_5M_SceneMode_Action,
	YUV_5M_SceneMode_Portrait,
	YUV_5M_SceneMode_Landscape,
	YUV_5M_SceneMode_Beach,
	YUV_5M_SceneMode_Candlelight,
	YUV_5M_SceneMode_Fireworks,
	YUV_5M_SceneMode_Night,
	YUV_5M_SceneMode_NightPortrait,
	YUV_5M_SceneMode_Party,
	YUV_5M_SceneMode_Snow,
	YUV_5M_SceneMode_Sports,
	YUV_5M_SceneMode_SteadyPhoto,
	YUV_5M_SceneMode_Sunset,
	YUV_5M_SceneMode_Theatre,
	YUV_5M_SceneMode_Barcode
};



struct sensor_5m_mode {
	int xres;
	int yres;
};

//#ifdef __KERNEL__
struct sensor_override_table {
	int table[3][1000];
};
// COMPAL Shiva --

//#ifdef __KERNEL__
struct yuv5_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
//#endif /* __KERNEL__ */

/* Compal Indigo-Carl ++ */
// focus window for touch focus
struct yuv5_focus_rect {
	int start_x;
	int start_y;
	int size_x;
	int size_y;
};
/* Compal Indigo-Carl -- */

#endif  /* __YUV5_SENSOR_H__ */
