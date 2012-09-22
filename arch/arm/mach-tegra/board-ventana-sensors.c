/*
 * arch/arm/mach-tegra/board-ventana-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/akm8975.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
//Eric  0825 begin
#include <linux/kxtf9.h>
//Eric	 end
#include <mach/gpio.h>
/* Compal Indigo-Carl begin */
//#include <media/ov5650.h>
//#include <media/ov2710.h>
//#include <media/sh532u.h>
//#include <media/ssl3250a.h>
#include <media/yuv_sensor_mt9d115.h>
#include <media/yuv_sensor_mt9p111.h>
/* Compal Indigo-Carl end */
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-ventana.h"
#include "cpu-tegra.h"

#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4

/* Compal Indigo-Carl begin */
//#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
//#define CAMERA_FLASH_ACT_GPIO	TEGRA_GPIO_PD2
#define YUV_SENSOR_OE_GPIO      TEGRA_GPIO_PBB5
#define YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PD2
#define YUV5_PWR_DN_GPIO        TEGRA_GPIO_PBB4
#define YUV5_RST_GPIO           TEGRA_GPIO_PA0
/* Compal Indigo-Carl end */

#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PN6

/* Compal Indigo-Carl begin */
struct camera_gpios {
	const char *name;
	int gpio;
	int enabled;
        int milliseconds;
};

#define CAMERA_GPIO(_name, _gpio, _enabled, _milliseconds)		        \
	{						                        \
		.name = _name,				                        \
		.gpio = _gpio,				                        \
		.enabled = _enabled,			                        \
		.milliseconds = _milliseconds,				        \
	}
/* Compal Indigo-Carl end */

#if 0  // Compal Indigo-Carl ++
static int ventana_camera_init(void)
{
	int err;

	tegra_gpio_enable(CAMERA_POWER_GPIO);
	gpio_request(CAMERA_POWER_GPIO, "camera_power_en");
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	gpio_export(CAMERA_POWER_GPIO, false);

	tegra_gpio_enable(CAMERA_CSI_MUX_SEL_GPIO);
	gpio_request(CAMERA_CSI_MUX_SEL_GPIO, "camera_csi_sel");
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	gpio_export(CAMERA_CSI_MUX_SEL_GPIO, false);

	err = gpio_request(CAMERA_FLASH_ACT_GPIO, "torch_gpio_act");
	if (err < 0) {
		pr_err("gpio_request failed for gpio %d\n",
					CAMERA_FLASH_ACT_GPIO);
	} else {
		tegra_gpio_enable(CAMERA_FLASH_ACT_GPIO);
		gpio_direction_output(CAMERA_FLASH_ACT_GPIO, 0);
		gpio_export(CAMERA_FLASH_ACT_GPIO, false);
	}
	return 0;
}
#endif  // Compal Indigo-Carl --

#if 0  // Compal Indigo-Carl ++
/* left ov5650 is CAM2 which is on csi_a */
static int ventana_left_ov5650_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM2_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM2_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM2_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_left_ov5650_power_off(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM2_RST_L_GPIO, 0);
	gpio_direction_output(CAM2_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 0);
	return 0;
}

struct ov5650_platform_data ventana_left_ov5650_data = {
	.power_on = ventana_left_ov5650_power_on,
	.power_off = ventana_left_ov5650_power_off,
};

/* right ov5650 is CAM1 which is on csi_b */
static int ventana_right_ov5650_power_on(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM1_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM1_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM1_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM1_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_right_ov5650_power_off(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM1_RST_L_GPIO, 0);
	gpio_direction_output(CAM1_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM1_LDO_SHUTDN_L_GPIO, 0);
	return 0;
}

struct ov5650_platform_data ventana_right_ov5650_data = {
	.power_on = ventana_right_ov5650_power_on,
	.power_off = ventana_right_ov5650_power_off,
};

static int ventana_ov2710_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM3_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM3_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_ov2710_power_off(void)
{
	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	gpio_direction_output(CAM3_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	return 0;
}

struct ov2710_platform_data ventana_ov2710_data = {
	.power_on = ventana_ov2710_power_on,
	.power_off = ventana_ov2710_power_off,
};
#endif  // Compal Indigo-Carl --

/* Compal Indigo-Carl begin */
// MT9D115 (2M camera)
static struct camera_gpios yuv_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0),
	[1] = CAMERA_GPIO("yuv_sensor_oe", YUV_SENSOR_OE_GPIO, 0, 0),
	[2] = CAMERA_GPIO("yuv_sensor_rst", YUV_SENSOR_RST_GPIO, 1, 0),
};

static int yuv_sensor_power_on(void)
{
	int i, ret;

	pr_info("[ MT9D115 ] _____  %s  _____\n", __func__);  // Carl test

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_sensor_power_off(void)
{
        int i;
	pr_info("[ MT9D115 ] _____  %s  _____\n", __func__);  // Carl test
	gpio_direction_output(YUV_SENSOR_OE_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	i = ARRAY_SIZE(yuv_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return 0;
}

struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};

// MT9P111 (5M camera)
static struct camera_gpios yuv5_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0),
	[1] = CAMERA_GPIO("yuv5_sensor_pwdn", YUV5_PWR_DN_GPIO, 0, 0),
	[2] = CAMERA_GPIO("yuv5_sensor_rst", YUV5_RST_GPIO, 1, 0),
};

static int yuv5_sensor_power_on(void)
{
	int i, ret;

	pr_info("[ MT9P111 ] _____  %s  _____\n", __func__);  // Carl test

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);
	gpio_direction_output(YUV5_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV5_RST_GPIO, 1);
	msleep(1);

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		gpio_export(yuv5_sensor_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv5_sensor_power_off(void)
{
	int i;
	pr_info("[ MT9P111 ] _____  %s  _____\n", __func__);  // Carl test
	gpio_direction_output(YUV5_RST_GPIO, 0);
	//msleep(1);
	udelay(500);   // Compal 0729
	gpio_direction_output(YUV5_PWR_DN_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);

	i = ARRAY_SIZE(yuv5_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return 0;
};

struct yuv5_sensor_platform_data yuv5_sensor_data = {
	.power_on = yuv5_sensor_power_on,
	.power_off = yuv5_sensor_power_off,
};

static int ventana_camera_init(void)
{
	int i, ret;

	pr_info("[ Camera ] _____  %s  _____\n", __func__);  // Carl test

	// initialize MT9D115
	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail2;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
		gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
	}

	// initialize MT9P111
	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail3;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);
	gpio_direction_output(YUV5_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
		gpio_export(yuv5_sensor_gpio_keys[i].gpio, false);
	}

	return 0;

fail2:
        while (i--)
                gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;

fail3:
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
}
/* Compal Indigo-Carl end */

#if 0  // Compal Indigo-Carl ++
static struct sh532u_platform_data sh532u_left_pdata = {
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_reset	= CAM2_RST_L_GPIO,
	.gpio_en	= CAM2_LDO_SHUTDN_L_GPIO,
};

static struct sh532u_platform_data sh532u_right_pdata = {
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_reset	= CAM1_RST_L_GPIO,
	.gpio_en	= CAM1_LDO_SHUTDN_L_GPIO,
};


static struct nvc_torch_pin_state ventana_ssl3250a_pinstate = {
	.mask		= 0x0040, /* VGP6 */
	.values		= 0x0040,
};

static struct ssl3250a_platform_data ventana_ssl3250a_pdata = {
	.dev_name	= "torch",
	.pinstate	= &ventana_ssl3250a_pinstate,
	.gpio_act	= CAMERA_FLASH_ACT_GPIO,
};
#endif  // Compal Indigo-Carl --

//Eric-0825 begin
//add light sensor driver
static void ventana_al3000a_ls_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "al3000a_ls");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}
//#ifdef CONFIG_SENSORS_AK8975
static void ventana_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
//#endif
static void ventana_kxtf9_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PN4);
	gpio_request(TEGRA_GPIO_PN4, "kxtf9");
	gpio_direction_input(TEGRA_GPIO_PN4);
}
//Eric-0825 end

static void ventana_ECBat_init(void)
{
	tegra_gpio_enable(AC_PRESENT_GPIO);
	gpio_request(AC_PRESENT_GPIO, "ac_present");
	gpio_direction_input(AC_PRESENT_GPIO);
	/* compal indigo-Howard Chang 20110513 begin */
	/* enable DOCK_ON pin  */
	tegra_gpio_enable(TEGRA_GPIO_PS7);
      /* compal indigo-Howard Chang 20110513 end */	
}

static void ventana_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data ventana_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info ventana_i2c0_board_info[] = {
//Eric-0825 begin
//add light sensor driver
	{
		I2C_BOARD_INFO("al3000a_ls", 0x1C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
//Eric-0825 end
};

/*static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z75", 0x0B),
	},
};*/

static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("EC_Battery", 0x58),
		.irq = TEGRA_GPIO_TO_IRQ(AC_PRESENT_GPIO),
	},
};

#if 0  // Compal Indigo-Carl ++
static struct pca953x_platform_data ventana_tca6416_data = {
	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
};

static struct pca954x_platform_mode ventana_pca9546_modes[] = {
	{ .adap_id = 6, .deselect_on_exit = 1 }, /* REAR CAM1 */
	{ .adap_id = 7, .deselect_on_exit = 1 }, /* REAR CAM2 */
	{ .adap_id = 8, .deselect_on_exit = 1 }, /* FRONT CAM3 */
};

static struct pca954x_platform_data ventana_pca9546_data = {
	.modes	  = ventana_pca9546_modes,
	.num_modes      = ARRAY_SIZE(ventana_pca9546_modes),
};

static const struct i2c_board_info ventana_i2c3_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &ventana_tca6416_data,
	},
};

static const struct i2c_board_info ventana_i2c3_board_info_pca9546[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &ventana_pca9546_data,
	},
};

static const struct i2c_board_info ventana_i2c3_board_info_ssl3250a[] = {
	{
		I2C_BOARD_INFO("ssl3250a", 0x30),
		.platform_data = &ventana_ssl3250a_pdata,
	},
};
#endif  // Compal Indigo-Carl --

///Eric 0825 begin
struct kxtf9_platform_data kxtf9_pdata = {

                .min_interval           = 1,
                .poll_interval          = 10,
                .g_range                = KXTF9_G_2G,
                .shift_adj              = SHIFT_ADJ_2G,
                .axis_map_x             = 1,
                .axis_map_y             = 0,
                .axis_map_z             = 2,
                .negate_x               = 0,
                .negate_y               = 0,
                .negate_z               = 1,
                .data_odr_init          = ODR100F,
                .ctrl_reg1_init         = ( KXTF9_G_2G | RES_12BIT ) & ~TDTE & ~TPE & ~WUFE,
                .int_ctrl_init          = 0x00,
                .tilt_timer_init        = 0x00,
                .engine_odr_init        = OTP1_6 | OWUF25 | OTDT50,
                .wuf_timer_init         = 0x00,
                .wuf_thresh_init        = 0x08,
                .tdt_timer_init         = 0x78,
                .tdt_h_thresh_init      = 0xB6,
                .tdt_l_thresh_init      = 0x1A,
                .tdt_tap_timer_init     = 0xA2,
                .tdt_total_timer_init   = 0x24,
                .tdt_latency_timer_init = 0x28,
                .tdt_window_timer_init  = 0xA0,
                .gpio = TEGRA_GPIO_PN4, // Replace with appropriate GPIO pin number 
};
///Eric 0825 end

static struct i2c_board_info ventana_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &ventana_nct1008_pdata,
	},
///Eric 0825 begin
///add compass g-sensor driver
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
	{ 
		I2C_BOARD_INFO("kxtf9", 0x0F),
              .platform_data = &kxtf9_pdata,
        },
};

/* Compal Indigo-Carl begin */
static struct i2c_board_info ventana_i2c3_board_info[] = {
#ifdef CONFIG_VIDEO_MT9D115
	// I2C slave addr : 0x3D
	{
		I2C_BOARD_INFO("mt9d115", 0x3D),
		.platform_data = &yuv_sensor_data,
 	},
#endif
#ifdef CONFIG_VIDEO_MT9P111
	// I2C slave addr : 0x3C
	{
		I2C_BOARD_INFO("mt9p111", 0x3C),
		.platform_data = &yuv5_sensor_data,
	},
#endif
};
/* Compal Indigo-Carl end */

#if 0  // Compal Indigo-Carl ++
static struct i2c_board_info ventana_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &ventana_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_right_pdata,
	},
};

static struct i2c_board_info ventana_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &ventana_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_left_pdata,
	},
};

static struct i2c_board_info ventana_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &ventana_ov2710_data,
	},
};
#endif  // Compal Indigo-Carl --

#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu3050_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if	MPU_ACCEL_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
#endif
		.platform_data = &mpu3050_accel_data,
	},
};

static struct i2c_board_info __initdata inv_mpu_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if	MPU_COMPASS_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
#endif
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

#if	MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif

	/* MPU-IRQ assignment */
	tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
	i2c_register_board_info(MPU_COMPASS_BUS_NUM, inv_mpu_i2c4_board_info,
		ARRAY_SIZE(inv_mpu_i2c4_board_info));
}
#endif

int __init ventana_sensors_init(void)
{
	struct board_info BoardInfo;

//Eric-0825 begin
//add sensor irq init
	ventana_al3000a_ls_init();
	ventana_akm8975_init();
	ventana_kxtf9_init();
//Eric-0825 end

#ifdef CONFIG_MPU_SENSORS_MPU3050
	mpuirq_init();
#endif
	ventana_camera_init();
	ventana_nct1008_init();

	i2c_register_board_info(0, ventana_i2c0_board_info,
		ARRAY_SIZE(ventana_i2c0_board_info));

//ec init
#if 0
	tegra_get_board_info(&BoardInfo);

	/*
	 * battery driver is supported on FAB.D boards and above only,
	 * since they have the necessary hardware rework
	 */
	if (BoardInfo.sku > 0) {
		i2c_register_board_info(2, ventana_i2c2_board_info,
			ARRAY_SIZE(ventana_i2c2_board_info));
	}
#else
	i2c_register_board_info(2, ventana_i2c2_board_info,
	                        ARRAY_SIZE(ventana_i2c2_board_info));
	ventana_ECBat_init();
#endif

    /* Compal Indigo-Carl begin */
	//i2c_register_board_info(3, ventana_i2c3_board_info_ssl3250a,
	//	ARRAY_SIZE(ventana_i2c3_board_info_ssl3250a));
	i2c_register_board_info(3, ventana_i2c3_board_info,
		ARRAY_SIZE(ventana_i2c3_board_info));
    /* Compal Indigo-Carl end */

	i2c_register_board_info(4, ventana_i2c4_board_info,
		ARRAY_SIZE(ventana_i2c4_board_info));

	//i2c_register_board_info(6, ventana_i2c6_board_info,
	//	ARRAY_SIZE(ventana_i2c6_board_info));

	//i2c_register_board_info(7, ventana_i2c7_board_info,
	//	ARRAY_SIZE(ventana_i2c7_board_info));

	//i2c_register_board_info(8, ventana_i2c8_board_info,
	//	ARRAY_SIZE(ventana_i2c8_board_info));

	return 0;
}

#if 0  // Compal Indigo-Carl ++
#ifdef CONFIG_TEGRA_CAMERA

struct tegra_camera_gpios {
	const char *name;
	int gpio;
	int enabled;
};

#define TEGRA_CAMERA_GPIO(_name, _gpio, _enabled)		\
	{						\
		.name = _name,				\
		.gpio = _gpio,				\
		.enabled = _enabled,			\
	}

static struct tegra_camera_gpios ventana_camera_gpio_keys[] = {
	[0] = TEGRA_CAMERA_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 1),
	[1] = TEGRA_CAMERA_GPIO("cam_i2c_mux_rst_lo", CAM_I2C_MUX_RST_GPIO, 1),

	[2] = TEGRA_CAMERA_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 0),
	[3] = TEGRA_CAMERA_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
	[4] = TEGRA_CAMERA_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
	[5] = TEGRA_CAMERA_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),

	[6] = TEGRA_CAMERA_GPIO("cam3_ldo_shdn_lo", CAM3_LDO_SHUTDN_L_GPIO, 0),
	[7] = TEGRA_CAMERA_GPIO("cam3_af_pwdn_lo", CAM3_AF_PWR_DN_L_GPIO, 0),
	[8] = TEGRA_CAMERA_GPIO("cam3_pwdn", CAM3_PWR_DN_GPIO, 0),
	[9] = TEGRA_CAMERA_GPIO("cam3_rst_lo", CAM3_RST_L_GPIO, 1),

	[10] = TEGRA_CAMERA_GPIO("cam1_ldo_shdn_lo", CAM1_LDO_SHUTDN_L_GPIO, 0),
	[11] = TEGRA_CAMERA_GPIO("cam1_af_pwdn_lo", CAM1_AF_PWR_DN_L_GPIO, 0),
	[12] = TEGRA_CAMERA_GPIO("cam1_pwdn", CAM1_PWR_DN_GPIO, 0),
	[13] = TEGRA_CAMERA_GPIO("cam1_rst_lo", CAM1_RST_L_GPIO, 1),
};

int __init ventana_camera_late_init(void)
{
	int ret;
	int i;
	struct regulator *cam_ldo6 = NULL;

	if (!machine_is_ventana())
		return 0;

	cam_ldo6 = regulator_get(NULL, "vdd_ldo6");
	if (IS_ERR_OR_NULL(cam_ldo6)) {
		pr_err("%s: Couldn't get regulator ldo6\n", __func__);
		return PTR_ERR(cam_ldo6);
	}

	ret = regulator_enable(cam_ldo6);
	if (ret){
		pr_err("%s: Failed to enable ldo6\n", __func__);
		goto fail_put_regulator;
	}

	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_tca6416);

	for (i = 0; i < ARRAY_SIZE(ventana_camera_gpio_keys); i++) {
		ret = gpio_request(ventana_camera_gpio_keys[i].gpio,
			ventana_camera_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(ventana_camera_gpio_keys[i].gpio,
			ventana_camera_gpio_keys[i].enabled);
		gpio_export(ventana_camera_gpio_keys[i].gpio, false);
	}

	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_pca9546);

	ventana_ov2710_power_off();
	ventana_left_ov5650_power_off();
	ventana_right_ov5650_power_off();

	ret = regulator_disable(cam_ldo6);
	if (ret){
		pr_err("%s: Failed to disable ldo6\n", __func__);
		goto fail_free_gpio;
	}

	regulator_put(cam_ldo6);
	return 0;

fail_free_gpio:
	while (i--)
		gpio_free(ventana_camera_gpio_keys[i].gpio);

fail_put_regulator:
	regulator_put(cam_ldo6);
	return ret;
}

late_initcall(ventana_camera_late_init);

#endif /* CONFIG_TEGRA_CAMERA */
#endif  // Compal Indigo-Carl --
