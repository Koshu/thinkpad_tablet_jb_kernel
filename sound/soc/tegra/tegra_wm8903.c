/*
 * tegra_wm8903.c - Tegra machine ASoC driver for boards using WM8903 codec.
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
/*fandy 1207 start*/
#include "tegra_wired_jack.h"
/*fandy 1207 end*/
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_wm8903_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
/*fandy 1130 start*/
#include <sound/wm8903.h>
#include <linux/delay.h>
/*fandy 1130 end*/
#include "../codecs/wm8903.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#endif

#define DRV_NAME "tegra-snd-wm8903"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)
/*fandy 1125 start*/
#define GPIO_DOCKING_MIC_DET  BIT(5)
/*fandy 1125 end*/

/*fandy 1209 start*/
static struct kobject *audio_kobj;
struct snd_soc_codec *PublicCodec;
/* Compal Fandy 20120421 begin */
//#define INDIGO_ALLOW_CHANGE_REGISTER
#ifdef INDIGO_ALLOW_CHANGE_REGISTER
s32	CodeRegisterReadAddress;
#endif
/* Compal Fandy 20120421 end */
bool Mute_Enable = false;
//fandy CTS 0416
bool CTS_Enable = false;

/* Compal Fandy 20120421 begin */
#ifdef INDIGO_ALLOW_CHANGE_REGISTER
static int CharHextoInt(const char *hex,int length)
{
	int result = 0,i;
	
	printk("length : %d\n",length);
	
	for(i = 0; i < length;i++)
	{
		printk("!%d\n",hex[i]);
		switch(hex[i])
		{
			case 0:
				break;
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				result <<= 4;
				result += hex[i] - 48;
				break;
			case 'a':
			case 'A':
				result <<= 4;
				result += 10;
				break;
			case 'b':
			case 'B':
				result <<= 4;
				result += 11;
				break;
			case 'c':
			case 'C':
				result <<= 4;
				result += 12;
				break;
			case 'd':
			case 'D':
				result <<= 4;
				result += 13;
				break;
			case 'e':
			case 'E':
				result <<= 4;
				result += 14;
				break;
			case 'f':
			case 'F':
				result <<= 4;
				result += 15;
				break;
			default:
				return -1;
				break;
		}
	}
	return result;
}

int IntToCharHex(char *buf,unsigned int value,int size)
{
	int i,j;
	char cbuff;
	
	//int to char array
	for(i =0;i  < size;i++)
	{
		buf[i] = value % 16;
		value /= 16;
		if(value == 0)
			break;
	}
	if(value != 0 || size < 1)
             return -1;
	 // convert to ascii
	for(j = 0;j <= i;j++)
	{
		if(buf[j] < 10)
			buf[j] += 48;
		else
			buf[j] += 55;
	}
 
	//reverse array 
	for(j = (i + 1) / 2;j > 0;j--)
	{
		cbuff = buf[i +1-j];
		buf[i+1-j] = buf[j - 1];
		buf[j-1]=cbuff;
	}
	return i+1;
}

static ssize_t CodecRegisterRead_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int result = CharHextoInt(buf,n-1);
	if(n < 0)
	{
		printk("CodecRegisterRead_store get wrong argument\n");
		return n;
	}
	CodeRegisterReadAddress = result;
	printk("CodeRegisterReadAddress 0x%x\n",CodeRegisterReadAddress);
	return n;
}

static ssize_t CodecRegisterRead_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int CtrlReg1;
	if(PublicCodec == NULL)
	{
		printk("CodecRegisterRead_show : PublicCodec is NULL\n");
		return 0;
	}
	CtrlReg1 = snd_soc_read(PublicCodec, CodeRegisterReadAddress);
	
	if(CtrlReg1 < 0)
	{
		printk("CodecRegisterRead_show : error snd_soc_read return %d\n",CtrlReg1);
		return 0;
	}
	
	printk("CodecRegisterRead_show : register 0x%x value : 0x%x\n",CodeRegisterReadAddress,CtrlReg1);
	
        return sprintf(buf,"%x\n",CtrlReg1);
}
static ssize_t CodecRegisterWrite_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int result1,result2;
	if(n < 7)
	{
		printk("CodecRegisterWrite_store get wrong argument\n");
		return n;
	}
	
	result1 = CharHextoInt(buf,2);
	result2 = CharHextoInt(buf+2,4);

	if(PublicCodec == NULL)
	{
		printk("CodecRegisterWrite_store : PublicCodec is NULL\n");
		return n;
	}
	result1 = snd_soc_write(PublicCodec, result1,result2);
	printk("CodecRegisterWrite_store : snd_soc_write return 0x%x\n",result1);
	
	return n;
}

static ssize_t CodecRegisterWrite_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        return 0;
}
#endif
/* Compal Fandy 20120421 end */

int a_to_i(const char *a)
{
    int s = 0;
    while(*a >= '0' && *a <= '9')
        s = (s << 3) + (s << 1) + *a++ - '0';
    return s;
}

static ssize_t Mic_Mute_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
	buffer = a_to_i(buf);
	int MicVolumeCtrlReg;
	MicVolumeCtrlReg = snd_soc_read(PublicCodec, WM8903_ANALOGUE_LEFT_INPUT_0);
	if (buffer == 1)
	{
		Mute_Enable = true;

		MicVolumeCtrlReg |= WM8903_LINMUTE;
		snd_soc_write(PublicCodec, WM8903_ANALOGUE_LEFT_INPUT_0,
				MicVolumeCtrlReg);
		snd_soc_write(PublicCodec, WM8903_ANALOGUE_RIGHT_INPUT_0,
				MicVolumeCtrlReg);
	}
	else
	{
		Mute_Enable = false;

		MicVolumeCtrlReg &= ~ WM8903_LINMUTE;
		snd_soc_write(PublicCodec, WM8903_ANALOGUE_LEFT_INPUT_0,
				MicVolumeCtrlReg);
		snd_soc_write(PublicCodec, WM8903_ANALOGUE_RIGHT_INPUT_0,
				MicVolumeCtrlReg);
	}

	return n;
}

static ssize_t Mic_Mute_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int state = 0;

	if(Mute_Enable)
		state = 1;

        return sprintf(buf,"%d\n",state);
}

//fandy CTS 0416
static ssize_t SNSD_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
	buffer = a_to_i(buf);
	
	if (buffer == 1)
	{
		CTS_Enable = true;
		 	snd_soc_write(PublicCodec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ca);
              	snd_soc_write(PublicCodec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ca);	
	}
	else
	{
		CTS_Enable = false;
		 	snd_soc_write(PublicCodec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ef);
            		snd_soc_write(PublicCodec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ef);
	}

	return n;
}

static ssize_t SNSD_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int state = 0;

	if(CTS_Enable)
		state = 1;

        return sprintf(buf,"%d\n",state);
}

#define debug_attr(_name) \
        static struct kobj_attribute _name##_attr = { \
        .attr = { \
        .name = __stringify(_name), \
        .mode = 0660, \
        }, \
        .show = _name##_show, \
        .store = _name##_store, \
        }

debug_attr(Mic_Mute);
//fandy CTS 0416
debug_attr(SNSD);
/* Compal Fandy 20120421 begin */
#ifdef INDIGO_ALLOW_CHANGE_REGISTER
debug_attr(CodecRegisterRead);
debug_attr(CodecRegisterWrite);
#endif
/* Compal Fandy 20120421 end */

static struct attribute *attr_item[] = 
{
/* Compal Fandy 20120421 begin */
#ifdef INDIGO_ALLOW_CHANGE_REGISTER
	&CodecRegisterRead_attr.attr,
	&CodecRegisterWrite_attr.attr,
#endif
/* Compal Fandy 20120421 end */
	&Mic_Mute_attr.attr,
//fandy CTS 0416
   	&SNSD_attr.attr,
	NULL,
};

static struct attribute_group attr_group =
{
        .attrs = attr_item,
};
/*fandy 1209 end*/

struct tegra_wm8903 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_wm8903_platform_data *pdata;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	int gpio_requested;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	enum snd_soc_bias_level bias_level;
};

int hp_irq_enb = 1;

/*fandy 1202 start*/
int jack_state;
#define SET_REG_VAL(r,m,l,v) (((r)&(~((m)<<(l))))|(((v)&(m))<<(l)))
/*fandy 1202 end*/

#ifdef CONFIG_SWITCH
/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
/*fandy 1128 start*/
	BIT_DOCKING_MIC = (1 << 2),
	BIT_DOCKING_MIC_HP = (1 << 3),
/*fandy 1128 end*/
};

/*fandy 1128 start*/
struct tegra_wm8903_platform_data 
gpio_tegra_wm8903_platform_data = {-1,-1,-1,-1,-1,-1};

/* polling external mic */
struct snd_soc_codec* g_codec;
/*fandy 1128 end*/

static void configure_dmic(struct snd_soc_codec *codec)
{
	u16 test4, reg;

		/* Disable DIG_MIC */
	test4 = snd_soc_read(codec, WM8903_CLOCK_RATE_TEST_4);
	test4 &= ~WM8903_ADC_DIG_MIC;

	reg = snd_soc_read(codec, 0x81);
	snd_soc_write(codec, 0x81,
			 reg | 0x0001);
	snd_soc_write(codec, WM8903_CLOCK_RATE_TEST_4, test4);
	snd_soc_write(codec, 0x81, reg);

}

static int tegra_wm8903_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, i2s_daifmt;
	int err;
	struct clk *clk_m;
	int rate;

/*fandy 1202 start*/
	int CtrlReg = 0;
	int VolumeCtrlReg = 0;

    PublicCodec = codec;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}



	clk_m = clk_get_sys(NULL, "clk_m");
	if (IS_ERR(clk_m)) {
		dev_err(card->dev, "Can't retrieve clk clk_m\n");
		err = PTR_ERR(clk_m);
		return err;
	}
	rate = clk_get_rate(clk_m);
	printk("extern1 rate=%d\n",rate);

#if TEGRA30_I2S_MASTER_PLAYBACK
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBS_CFS;
#else
	mclk = rate;

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBM_CFM;
#endif


	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2) &&
	    (machine_is_ventana() || machine_is_harmony() ||
	    machine_is_kaen() || machine_is_aebl()))
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
	else
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif

/*fandy 1202 start*/
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {

		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0, 0X7);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0, 0X7);
		/* Mic Bias enable */
		CtrlReg = (0x1<<B00_MICBIAS_ENA) | (0x1<<B01_MICDET_ENA);
		snd_soc_write(codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
		/* Enable DRC */

/* fix internal mic function */
		if(jack_state == BIT_HEADSET)
		{
			printk("\nhw Record device : headset\n");
			CtrlReg = (0x0<<B06_IN_CM_ENA) |(0x1<<B04_IP_SEL_N) | (0x0<<B02_IP_SEL_P) | (0x0<<B00_MODE);
			VolumeCtrlReg = (0x1C << B00_IN_VOL);
/* mic gain tunning */                 
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1c0);
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1c0);
			
		}else if( (jack_state == BIT_HEADSET_NO_MIC) ||  (jack_state == BIT_NO_HEADSET) )
		{
			printk("hw Record device : int mic\n");
			CtrlReg = 0x52;
			VolumeCtrlReg = (0x03 << B00_IN_VOL);
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ef);
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ef);
//fandy CTS 0416		
		if(CTS_Enable){
		 	snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ca);
            		snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ca);
		 }    

		}else if( (jack_state == BIT_DOCKING_MIC) ||(jack_state == BIT_DOCKING_MIC_HP) )
		{
			printk("\nhw Record device : docking mic\n");
			CtrlReg = (0x0<<B06_IN_CM_ENA) |(0x2<<B04_IP_SEL_N) | (0x2<<B02_IP_SEL_P) | (0x0<<B00_MODE);
			VolumeCtrlReg = (0x1C << B00_IN_VOL);
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1c0);
            snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1c0);                     

	    }
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);

//Enable mute
	    if(Mute_Enable)
			VolumeCtrlReg |= WM8903_LINMUTE;
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0,
				VolumeCtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
				VolumeCtrlReg);
		CtrlReg = snd_soc_read(codec, WM8903_AUDIO_INTERFACE_0);
		
		if( (jack_state == BIT_HEADSET_NO_MIC) ||  (jack_state == BIT_NO_HEADSET) )
		{
			CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCR, 0x1);
			CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCL, 0x1);
		}
		else
		{
			CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCR, 0x0);
			CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, B06_AIF_ADCL, 0x0);
		}
		
		snd_soc_write(codec, WM8903_AUDIO_INTERFACE_0, CtrlReg);
		/* Enable analog inputs */
		CtrlReg = (0x1<<B01_INL_ENA) | (0x1<<B00_INR_ENA);
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_0, CtrlReg);
		/* ADC Settings */
		CtrlReg = snd_soc_read(codec, WM8903_ADC_DIGITAL_0);
		CtrlReg |= (0x1<<B04_ADC_HPF_ENA);
		snd_soc_write(codec, WM8903_ADC_DIGITAL_0, CtrlReg);
		/* Disable sidetone */
		CtrlReg = 0;
		snd_soc_write(codec, R20_SIDETONE_CTRL, CtrlReg);
		/* Enable ADC */
		CtrlReg = snd_soc_read(codec, WM8903_POWER_MANAGEMENT_6);
		CtrlReg |= (0x1<<B01_ADCL_ENA);
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_6, CtrlReg);
		CtrlReg = snd_soc_read(codec, R29_DRC_1);
		CtrlReg |= 0x3; /*mic volume 18 db */
		snd_soc_write(codec, R29_DRC_1, CtrlReg);

		configure_dmic(codec);
	}
	//tuning speaker volume
//fandy 0419 
	CtrlReg = 0xBC;//0xBB 
	snd_soc_write(codec, WM8903_ANALOGUE_OUT3_LEFT, CtrlReg);
	CtrlReg = 0x1BE;//0x1B0
	snd_soc_write(codec, WM8903_DAC_DIGITAL_VOLUME_LEFT, CtrlReg);
	snd_soc_write(codec, WM8903_DAC_DIGITAL_VOLUME_RIGHT, CtrlReg);
	CtrlReg = 0xB8;
	snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, CtrlReg);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, CtrlReg);
	CtrlReg = 0xB4;
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, CtrlReg);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, CtrlReg);
/*fandy 1202 end*/
	return 0;
}

static int tegra_bt_sco_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_wm8903_ops = {
	.hw_params = tegra_wm8903_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_wm8903_bt_sco_ops = {
	.hw_params = tegra_bt_sco_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_jack tegra_wm8903_hp_jack;
static struct snd_soc_jack tegra_wm8903_docking_mic_jack;

/*fandy 1125 start*/
#if 1
static struct snd_soc_jack_gpio tegra_wm8903_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};
static struct snd_soc_jack_gpio tegra_wm8903_mic_jack_gpio = {
	.name = "docking mic detect",
	.report = SND_JACK_DOCKING_MIC,
	.debounce_time = 150,
	.invert = 1,
};

#else
static struct snd_soc_jack_gpio tegra_wm8903_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};
#endif
/*fandy 1125 end*/

/* fandy 1201 begin*/
/* Based on hp_gpio and mic_gpio, hp_gpio is active low */
enum {
	ERROR_0 = 				0x0,
	ERROR_1 = 				0x1,
	DOCKING_MIC_HP = 		0x2,
	DOCKING_MIC = 			0x3,
	HEADSET_WITHOUT_MIC = 	0x4,
	NO_DEVICE = 			0x5,
	HEADSET_WITH_MIC = 		0x6,
	ERROR_3 = 				0x7,
};
/* fandy 1201 end*/

static struct switch_dev tegra_wm8903_headset_switch = {
	.name = "h2w",
};

/*fandy 1130 start*/
/* polling external mic */
static int wired_jack_detect(void)
{
	int i;
	int withMic = 0;
	int withoutMic = 0;
	int MICDET_EINT_14 = 0;
	int MICSHRT_EINT_15 = 0;
	int irqStatus;
	int irq_mask = 0x3fff;
	int all_mask = 0xffff;
	int CtrlReg = 0;

/* fix the issue that device can't switch path from
 * internal MIC to cradle MIC on recording mode*/
	int backup_micbias_register;
	CtrlReg = snd_soc_read(g_codec, WM8903_VMID_CONTROL_0);
	CtrlReg &= ~(WM8903_VMID_RES_MASK);
	CtrlReg |= WM8903_VMID_RES_50K;
	snd_soc_write(g_codec, WM8903_VMID_CONTROL_0, CtrlReg);
	snd_soc_write(g_codec, WM8903_BIAS_CONTROL_0, 0xB);
	CtrlReg = snd_soc_read(g_codec, WM8903_CLOCK_RATES_2);
	CtrlReg |= (WM8903_CLK_DSP_ENA | WM8903_CLK_SYS_ENA | WM8903_TO_ENA);
	snd_soc_write(g_codec, WM8903_CLOCK_RATES_2, CtrlReg);
	snd_soc_write(g_codec, WM8903_INTERRUPT_STATUS_1_MASK, irq_mask);
/* fix the issue that device can't switch path from internal
 *  MIC to cradle MIC on recording mode*/
	backup_micbias_register = snd_soc_read(g_codec, WM8903_MIC_BIAS_CONTROL_0);
	snd_soc_write(g_codec, WM8903_MIC_BIAS_CONTROL_0, WM8903_MICDET_ENA | WM8903_MICBIAS_ENA);
	/* debounce */
	msleep(100);
	for(i = 0; i <= 15; i++)
	{
		msleep(1);
		irqStatus = snd_soc_read(g_codec, WM8903_INTERRUPT_STATUS_1);

		MICDET_EINT_14 = (irqStatus >> 14) & 0x1;
		MICSHRT_EINT_15 = (irqStatus >> 15) & 0x1;
		if(MICDET_EINT_14 == MICSHRT_EINT_15)
			withoutMic++;
		else
			withMic++;
		if(i%2 == 0)
			snd_soc_write(g_codec, WM8903_INTERRUPT_POLARITY_1, irq_mask);
		else
			snd_soc_write(g_codec, WM8903_INTERRUPT_POLARITY_1, all_mask);
	}
/* fix the issue that device can't switch path from internal
 * MIC to cradle MIC on recording mode*/
	snd_soc_write(g_codec, WM8903_MIC_BIAS_CONTROL_0, backup_micbias_register);
	if (withMic > withoutMic)
		return 1;
	else
		return 0;
}
/* fandy 1130 end*/

/* for hot plug when recording */
static void select_mic_input(int state)
{
	int CtrlReg = 0,VolumeCtrlReg = 0;

	switch (state) {
		case BIT_HEADSET:
		{
			printk("\nRecord device : headset mic\n");
			snd_soc_write(g_codec, WM8903_AUDIO_INTERFACE_0,0x10);
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x1<<B04_IP_SEL_N) | (0x0<<B02_IP_SEL_P) | (0x0<<B00_MODE);
			VolumeCtrlReg = (0x1C << B00_IN_VOL);	             
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1c0);
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1c0);                     						
		}
		break;

		case BIT_NO_HEADSET:
		case BIT_HEADSET_NO_MIC:
		{
			printk("\nRecord device : int mic\n");
			snd_soc_write(g_codec, WM8903_AUDIO_INTERFACE_0,0xD0);
			CtrlReg = 0x52;
			VolumeCtrlReg = (0x03 << B00_IN_VOL);
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ef);
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ef);                    
//fandy CTS 0416		
			if(CTS_Enable){
		 		snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1ca);
            			snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1ca);
		 	} 		
		}
		break;

		case BIT_DOCKING_MIC:
		case BIT_DOCKING_MIC_HP:
		{
			printk("\nRecord device : docking mic\n");
			snd_soc_write(g_codec, WM8903_AUDIO_INTERFACE_0,0x10);
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x2<<B04_IP_SEL_N) | (0x2<<B02_IP_SEL_P) | (0x0<<B00_MODE);
			VolumeCtrlReg = (0x1C << B00_IN_VOL);
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, 0x1c0);
            snd_soc_write(g_codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, 0x1c0);
		}
		break;
	}
	snd_soc_write(g_codec, WM8903_ANALOGUE_LEFT_INPUT_0,VolumeCtrlReg);
	snd_soc_write(g_codec, WM8903_ANALOGUE_RIGHT_INPUT_0,VolumeCtrlReg);
	snd_soc_write(g_codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
	snd_soc_write(g_codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);
}

static int tegra_wm8903_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;

	int mic_gpio = wired_jack_detect();

	if (jack == &tegra_wm8903_hp_jack) {
		machine->jack_status &= ~SND_JACK_HEADPHONE;
		machine->jack_status |= (action & SND_JACK_HEADPHONE);
	} else if (jack == &tegra_wm8903_docking_mic_jack){
		machine->jack_status &= ~SND_JACK_DOCKING_MIC;
		machine->jack_status |= (action & SND_JACK_DOCKING_MIC);	
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
	    if (mic_gpio) {
			state = BIT_HEADSET;
			jack_state = BIT_HEADSET;
	    } else {
			state = BIT_HEADSET_NO_MIC;
			jack_state = BIT_HEADSET_NO_MIC;
	     }
		break;
	case SND_JACK_DOCKING_MIC_HP:
		state = BIT_HEADSET_NO_MIC;
		jack_state = BIT_DOCKING_MIC_HP;
		break;
	case SND_JACK_DOCKING_MIC:
		state = BIT_NO_HEADSET;
		jack_state = BIT_DOCKING_MIC;
		break;
	default:
		state = BIT_NO_HEADSET;
		jack_state = BIT_NO_HEADSET;
	}

	select_mic_input(jack_state);
	switch_set_state(&tegra_wm8903_headset_switch, state);

	return NOTIFY_OK;
}

static struct notifier_block tegra_wm8903_jack_detect_nb = {
	.notifier_call = tegra_wm8903_jack_notifier,
};
#else
static struct snd_soc_jack_pin tegra_wm8903_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_pin tegra_wm8903_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};
#endif

static int tegra_wm8903_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (machine->dmic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->dmic_reg);
		else
			regulator_disable(machine->dmic_reg);
	}

	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget cardhu_dapm_widgets[] = {
/*fadny 1123 start*/
#if 1
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
	SND_SOC_DAPM_LINE("Headset", NULL),
#else
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
	SND_SOC_DAPM_LINE("Line In", NULL),
#endif
/*fandy 1123 end*/
};

static const struct snd_soc_dapm_widget tegra_wm8903_default_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route harmony_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route cardhu_audio_map[] = {
/*fandy 1123 begin*/
#if 1
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},

	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},

	{"Line Out", NULL, "LINEOUTL"},
	{"Line Out", NULL, "LINEOUTR"},

	{"Mic Bias", NULL, "Mic Jack"},
	{"IN3L", NULL, "Mic Bias"},
	{"IN3R", NULL, "Mic Bias"},

	{"Headset", NULL, "HPOUTR"},
	{"Headset", NULL, "HPOUTL"},
	{"Mic Bias", NULL, "Headset"},
	{"IN2L", NULL, "Mic Bias"},

	{"Mic Bias", NULL, "Int Mic"},
	{"IN1R", NULL, "Mic Bias"},
	{"IN2R", NULL, "Mic Bias"},
#else
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Line Out", NULL, "LINEOUTL"},
	{"Line Out", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
	{"IN1L", NULL, "Mic Bias"},
	{"IN1R", NULL, "Mic Bias"},
	{"IN3L", NULL, "Line In"},
	{"IN3R", NULL, "Line In"},
#endif
/*fandy 1123 end*/
};

static const struct snd_soc_dapm_route seaboard_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route kaen_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN2R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route aebl_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "LINEOUTR"},
	{"Int Spk", NULL, "LINEOUTL"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_kcontrol_new cardhu_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("LineOut"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("LineIn"),
};

static const struct snd_kcontrol_new tegra_wm8903_default_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int tegra_wm8903_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;
	int ret;
/*fandy 1128 start*/
	int gpio_hp_det;
	int gpio_docking_mic_det;
/*fandy 1128 end*/

	jack_state = BIT_NO_HEADSET;

/*fandy 1130 start*/
    g_codec = rtd->codec;
/*fandy 1130 end*/

/*fandy 1209 start*/
   PublicCodec = NULL;
	//get codec register

	audio_kobj = kobject_create_and_add("AudioCodec", NULL);
	if (audio_kobj == NULL)
		printk("tegra_soc_wm8903 kobject_create_and_add failed\n");
	
	if(sysfs_create_group(audio_kobj, &attr_group))
		printk("tegra_soc_wm8903 sysfs_create_group failed\n");
/*fandy 1209 end*/

	machine->bias_level = SND_SOC_BIAS_STANDBY;

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}

	if (machine_is_cardhu() || machine_is_ventana()) {
		ret = snd_soc_add_controls(codec, cardhu_controls,
				ARRAY_SIZE(cardhu_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm, cardhu_dapm_widgets,
				ARRAY_SIZE(cardhu_dapm_widgets));
	}
	else {
		ret = snd_soc_add_controls(codec,
				tegra_wm8903_default_controls,
				ARRAY_SIZE(tegra_wm8903_default_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm,
				tegra_wm8903_default_dapm_widgets,
				ARRAY_SIZE(tegra_wm8903_default_dapm_widgets));
	}

	if (machine_is_harmony()) {
		snd_soc_dapm_add_routes(dapm, harmony_audio_map,
					ARRAY_SIZE(harmony_audio_map));
	} else if (machine_is_cardhu() || machine_is_ventana()) {
		snd_soc_dapm_add_routes(dapm, cardhu_audio_map,
					ARRAY_SIZE(cardhu_audio_map));
	} else if (machine_is_seaboard()) {
		snd_soc_dapm_add_routes(dapm, seaboard_audio_map,
					ARRAY_SIZE(seaboard_audio_map));
	} else if (machine_is_kaen()) {
		snd_soc_dapm_add_routes(dapm, kaen_audio_map,
					ARRAY_SIZE(kaen_audio_map));
	} else {
		snd_soc_dapm_add_routes(dapm, aebl_audio_map,
					ARRAY_SIZE(aebl_audio_map));
	}

/*fandy 1125 start*/
#if 1
if (gpio_is_valid(pdata->gpio_hp_det)) {
		gpio_hp_det=pdata->gpio_hp_det;
		tegra_wm8903_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_wm8903_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_hp_jack,
					ARRAY_SIZE(tegra_wm8903_hp_jack_pins),
					tegra_wm8903_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_hp_jack,
					&tegra_wm8903_jack_detect_nb);
#endif
		ret=snd_soc_jack_add_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
		if(ret){
		printk("Cannot set hp gpio\n");
		}
		machine->gpio_requested |= GPIO_HP_DET;
		gpio_tegra_wm8903_platform_data.gpio_hp_det = gpio_hp_det;
}

if (gpio_is_valid(pdata->gpio_docking_mic_det))	{					
		gpio_docking_mic_det = pdata->gpio_docking_mic_det;
		tegra_wm8903_mic_jack_gpio.gpio = pdata->gpio_docking_mic_det;
		snd_soc_jack_new(codec, "Mic Jack", SND_JACK_DOCKING_MIC,
				&tegra_wm8903_docking_mic_jack);	
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_hp_jack,
					ARRAY_SIZE(tegra_wm8903_hp_jack_pins),
					&tegra_wm8903_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_docking_mic_jack,
				&tegra_wm8903_jack_detect_nb);
#endif
		ret=snd_soc_jack_add_gpios(&tegra_wm8903_docking_mic_jack,
					1,
					&tegra_wm8903_mic_jack_gpio);
		if(ret){
			printk("Cannot set docking mic gpio.\n");
			}
		machine->gpio_requested |= GPIO_DOCKING_MIC_DET;
		gpio_tegra_wm8903_platform_data.gpio_docking_mic_det= gpio_docking_mic_det;
}

#else
	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_wm8903_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_wm8903_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_hp_jack,
					ARRAY_SIZE(tegra_wm8903_hp_jack_pins),
					tegra_wm8903_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_hp_jack,
					&tegra_wm8903_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;
	}

	snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			 &tegra_wm8903_mic_jack);
#ifndef CONFIG_SWITCH
	snd_soc_jack_add_pins(&tegra_wm8903_mic_jack,
			      ARRAY_SIZE(tegra_wm8903_mic_jack_pins),
			      tegra_wm8903_mic_jack_pins);
#else
	snd_soc_jack_notifier_register(&tegra_wm8903_mic_jack,
				&tegra_wm8903_jack_detect_nb);
#endif

#endif
/*fandy 1125 end*/

/*fandy 1201 start*/
//	wm8903_mic_detect(codec, &tegra_wm8903_mic_jack, SND_JACK_MICROPHONE,
//				0);
/*fandy 1201 end*/

	snd_soc_dapm_force_enable_pin(dapm, "Mic Bias");

	/* FIXME: Calculate automatically based on DAPM routes? */
	if (!machine_is_harmony() && !machine_is_ventana() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1L");
	if (!machine_is_seaboard() && !machine_is_aebl() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1R");
	snd_soc_dapm_nc_pin(dapm, "IN2L");
	if (!machine_is_kaen())
		snd_soc_dapm_nc_pin(dapm, "IN2R");
	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");

	if (machine_is_aebl()) {
		snd_soc_dapm_nc_pin(dapm, "LON");
		snd_soc_dapm_nc_pin(dapm, "RON");
		snd_soc_dapm_nc_pin(dapm, "ROP");
		snd_soc_dapm_nc_pin(dapm, "LOP");
	} else {
		snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
		snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	}

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int tegra30_soc_set_bias_level(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF &&
		level != SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_enable(&machine->util_data);

	return 0;
}

static int tegra30_soc_set_bias_level_post(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF &&
		level == SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_disable(&machine->util_data);

	machine->bias_level = level;

	return 0 ;
}

static struct snd_soc_dai_link tegra_wm8903_dai[] = {
	{
		.name = "WM8903",
		.stream_name = "WM8903 PCM",
		.codec_name = "wm8903.0-001a",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.0",
		.codec_dai_name = "wm8903-hifi",
		.init = tegra_wm8903_init,
		.ops = &tegra_wm8903_ops,
	},
	{
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-spdif",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_spdif_ops,
	},
	{
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.1",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_wm8903_bt_sco_ops,
	},
};

void tegra_wm8903_suspend_post(struct snd_soc_card *card)
{
	struct snd_soc_jack_gpio *gpio = &tegra_wm8903_hp_jack_gpio;

       if (gpio_is_valid(gpio->gpio)){
               if(hp_irq_enb){
                       disable_irq(gpio_to_irq(gpio->gpio));
                       hp_irq_enb = 0;
               }
       }
}

void tegra_wm8903_resume_pre(struct snd_soc_card *card)
{
	int val;
	struct snd_soc_jack_gpio *gpio = &tegra_wm8903_hp_jack_gpio;

	if (gpio_is_valid(gpio->gpio)) {
		val = gpio_get_value(gpio->gpio);
		val = gpio->invert ? !val : val;
		snd_soc_jack_report(gpio->jack, val, gpio->report);
               if(!hp_irq_enb){
                       enable_irq(gpio_to_irq(gpio->gpio));
                       hp_irq_enb = 1;
               }
	}
}

static struct snd_soc_card snd_soc_tegra_wm8903 = {
	.name = "tegra-wm8903",
	.dai_link = tegra_wm8903_dai,
	.num_links = ARRAY_SIZE(tegra_wm8903_dai),
	.suspend_post = tegra_wm8903_suspend_post,
	.resume_pre = tegra_wm8903_resume_pre,
	//.set_bias_level = tegra30_soc_set_bias_level,
	//.set_bias_level_post = tegra30_soc_set_bias_level_post,
};

static __devinit int tegra_wm8903_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine;
	struct tegra_wm8903_platform_data *pdata;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_wm8903), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_wm8903 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		goto err_free_machine;

	machine->spk_reg = regulator_get(&pdev->dev, "vdd_spk_amp");
	if (IS_ERR(machine->spk_reg)) {
		dev_info(&pdev->dev, "No speaker regulator found\n");
		machine->spk_reg = 0;
	}

	machine->dmic_reg = regulator_get(&pdev->dev, "vdd_dmic");
	if (IS_ERR(machine->dmic_reg)) {
		dev_info(&pdev->dev, "No digital mic regulator found\n");
		machine->dmic_reg = 0;
	}

	if (machine_is_cardhu()) {
		tegra_wm8903_dai[0].codec_name = "wm8903.4-001a",
		tegra_wm8903_dai[0].cpu_dai_name = "tegra30-i2s.1";

		tegra_wm8903_dai[1].cpu_dai_name = "tegra30-spdif";

		tegra_wm8903_dai[2].cpu_dai_name = "tegra30-i2s.3";
	}

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&tegra_wm8903_headset_switch);
	if (ret < 0)
		goto err_fini_utils;
#endif

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_switch;
	}

	if (!card->instantiated) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_card;
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_headset_switch);
err_fini_utils:
#endif
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_wm8903_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);

/*fandy 1129 start*/
	if (machine->gpio_requested & GPIO_DOCKING_MIC_DET)
		snd_soc_jack_free_gpios(&tegra_wm8903_docking_mic_jack,
					1,
					&tegra_wm8903_mic_jack_gpio);
/*fandy 1129 end*/

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	machine->gpio_requested = 0;

	if (machine->spk_reg)
		regulator_put(machine->spk_reg);
	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_headset_switch);
#endif
	kfree(machine);

	return 0;
}

static struct platform_driver tegra_wm8903_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_wm8903_driver_probe,
	.remove = __devexit_p(tegra_wm8903_driver_remove),
};

static int __init tegra_wm8903_modinit(void)
{
	return platform_driver_register(&tegra_wm8903_driver);
}
module_init(tegra_wm8903_modinit);

static void __exit tegra_wm8903_modexit(void)
{
	platform_driver_unregister(&tegra_wm8903_driver);
}
module_exit(tegra_wm8903_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra+WM8903 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
