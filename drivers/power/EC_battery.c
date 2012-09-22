/*
 * drivers/power/EC_Bat.c
 *
 * Gas Gauge driver for TI's BQ20Z75
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>  
#include <linux/gpio.h>   
#include <linux/timer.h>  
#include <linux/interrupt.h>  

/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
#include <linux/io.h>
/* compal indigo-Howard Chang 20100310 end */

#define NVBATTERY_POLLING_INTERVAL 30000 /* 30 Seconds */   
/* compal indigo-Ryan 20110315 begin */
#define DOCK_ON		151 //GPIO_PS7
/* compal indigo-Ryan 20110315 end */

// #define FACTORY_BUILD_VERSION

static enum power_supply_property EC_Bat_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};


static enum power_supply_property EC_Bat_charge_properties[] = {
        POWER_SUPPLY_PROP_ONLINE,
};

static int EC_Bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);



static int EC_Bat_get_charge_property(struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val);

/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
static int bq20z75_get_usb_charge_property(struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val);
/* compal indigo-Howard Chang 20100310 end */

/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
void __iomem		*USB1_reg = NULL;
/* compal indigo-Howard Chang 20100310 end */

static char *supply_list[] = {
        "battery",
};

static struct power_supply EC_Bat_supply[] = {
{
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = EC_Bat_properties,
        .num_properties = ARRAY_SIZE(EC_Bat_properties),
        .get_property = EC_Bat_get_property,
},
{
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = EC_Bat_charge_properties,
        .num_properties = ARRAY_SIZE(EC_Bat_charge_properties),
        .get_property = EC_Bat_get_charge_property,
},
/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
{
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = EC_Bat_charge_properties,
        .num_properties = ARRAY_SIZE(EC_Bat_charge_properties),
        .get_property = bq20z75_get_usb_charge_property,
},
/* compal indigo-Howard Chang 20100310 end */
};

struct EC_Bat_device_info {
	struct i2c_client	*client;
} *EC_Bat_device;

struct timer_list poll_timer;    
unsigned int batt_status_poll_period;    
int gpio;       
s16 ThreeGPower_val = 1;
s16 MicSwitch_val = 2;
s16 Coldboot_val = 0;
s16 Resume_val = 0;
s16 GPS_val = 0;
s16 PsensorPower_val = 2;
bool ECflashMode = 0;    //for EC flash
SYMBOL_EXPORT(ThreeGPower_val);

typedef enum
{
        NvCharger_Type_Battery = 0,
        NvCharger_Type_AC,
/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
        NvCharger_Type_USB,
/* compal indigo-Howard Chang 20100310 end */
} NvCharger_Type;

/* compal indigo-Howard Chang 20100412 begin*/
u8 ECNameSpaceReadAddress = 0;
/* compal indigo-Howard Chang 20100412 end */

static void tegra_battery_poll_timer_func(unsigned long unused)
{
        power_supply_changed(&EC_Bat_supply[NvCharger_Type_Battery]);
        mod_timer(&poll_timer, jiffies + msecs_to_jiffies(batt_status_poll_period));
}





static irqreturn_t ac_interrupt(int irq, void *dev_id)
{
        if(ECflashMode == 0)       //disable polling EC for battery info when ECflash
        {
//        	printk("[Power] detect ac-usb plug \n");  
              power_supply_changed(&EC_Bat_supply[NvCharger_Type_Battery]);
              power_supply_changed(&EC_Bat_supply[NvCharger_Type_AC]);
/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
              power_supply_changed(&EC_Bat_supply[NvCharger_Type_USB]);
/* compal indigo-Howard Chang 20100310 end */
          }
        return IRQ_HANDLED;
}



//SYS_START
static struct kobject *Ecdebug_kobj;

#ifdef FACTORY_BUILD_VERSION
#define debug_attr(_name) \
        static struct kobj_attribute _name##_attr = { \
        .attr = { \
        .name = __stringify(_name), \
        .mode = 0666, \
        }, \
        .show = _name##_show, \
        .store = _name##_store, \
        }
#else
#define debug_attr(_name) \
        static struct kobj_attribute _name##_attr = { \
        .attr = { \
        .name = __stringify(_name), \
        .mode = 0660, \
        }, \
        .show = _name##_show, \
        .store = _name##_store, \
        }
#endif

int atoi(const char *a)
{
int s = 0;
    while(*a >= '0' && *a <= '9')
        s = (s << 3) + (s << 1) + *a++ - '0';
    return s;
}
//Eric 0607 begin
//EEPROM 
void Clear_Index()
{
	i2c_smbus_write_word_data(EC_Bat_device->client,0x49,1);
	return ;
}

int Wait_For_Ready ()
{
        s32 ret;
	 s32 ready;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x48);	 
	 ready = ret & 0x00000008;	 
	 return ready;	
}
//Eric 0607 end

static ssize_t PowerLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "PowerLED");
        return (s - buf);
}

static ssize_t PowerLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x42,0);
        msleep(100);
        return n;
}

static ssize_t ChargeLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "ChargeLED");
        return (s - buf);
}

static ssize_t ChargeLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x43,0);
        msleep(100);
        return n;
}
static ssize_t OriStsLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "OriStsLED");
        return (s - buf);
}

static ssize_t OriStsLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x40,0);
        msleep(100);
        return n;
}
static ssize_t OffLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "OffLED");
        return (s - buf);
}

static ssize_t OffLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x41,0);
        msleep(100);
        return n;
}
static ssize_t BatCtlEnable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        s8 byteret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x5b);
        byteret = ret & 0x000000FF;
        s += sprintf(s, "%d\n",byteret);
        return (s - buf);
}
static ssize_t BatCtlEnable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        int buffer;
        s16 BatCtlEnable_val;
        buffer = atoi(buf);
        BatCtlEnable_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x50,BatCtlEnable_val);
        msleep(100);
        return n;
}
static ssize_t BatCtlDisable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "BatCtlDisable");
        return (s - buf);
}
static ssize_t BatCtlDisable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x51,0);
        msleep(100);
        return n;
}

static ssize_t EcVer_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 PjIDMaVer, MiVerTestVer, ret;
        PjIDMaVer = i2c_smbus_read_word_data(EC_Bat_device->client,0x30);
        msleep(100);
        MiVerTestVer = i2c_smbus_read_word_data(EC_Bat_device->client,0x31);
        msleep(100);
        ret = PjIDMaVer;
        ret = ret << 16 | (MiVerTestVer & 0xFFFF);
        s += sprintf(s, "%x\n", ret);
        return (s - buf);
}

static ssize_t EcVer_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

static void TransformToByte(u16 val16, u8 *val8_0, u8 *val8_1)
{
     *val8_0 = val16 & 0x00ff;
     *val8_1 = val16 >> 8 & 0x00ff;


}

static ssize_t UUID_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

        int i;
        char * s = buf;
        u8 val8[32] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "UUID_show EEPROM not Ready\n");
       	return (s - buf);
	}

        
        for(i=0;i<16;i++)
           {
               val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x60);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }
        s += sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
val8[31],val8[30],val8[29],val8[28],val8[27],val8[26],val8[25],val8[24],val8[23],val8[22],val8[21],val8[20],val8[19],val8[18],val8[17],val8[16],
val8[15],val8[14],val8[13],val8[12],val8[11],val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);
         

        return (s - buf);
}

static u8 TransformCharToByte(const char *buf, int index)
{


       u8  val8;
       char locbuf[2] = {0};
       int buffer = 0, j;

 
      for(j=0;j<=1;j++)
           {
                  locbuf[j] = buf[index+j];
           }

               if(locbuf[1] == 'a' || locbuf[1] == 'A')
                  buffer = 10;
               else if(locbuf[1] == 'b' || locbuf[1] == 'B')
                       buffer = 11;
               else if(locbuf[1] == 'c' || locbuf[1] == 'C')
                       buffer = 12;
               else if(locbuf[1] == 'd' || locbuf[1] == 'D')
                       buffer = 13;
               else if(locbuf[1] == 'e' || locbuf[1] == 'E')
                       buffer = 14;
               else if(locbuf[1] == 'f' || locbuf[1] == 'F')
                       buffer = 15;
               else if(locbuf[1] == '0')
                       buffer = 0;
               else if(locbuf[1] == '1')
                       buffer = 1;
               else if(locbuf[1] == '2')
                       buffer = 2;
               else if(locbuf[1] == '3')
                       buffer = 3;
               else if(locbuf[1] == '4')
                       buffer = 4;
               else if(locbuf[1] == '5')
                       buffer = 5;
               else if(locbuf[1] == '6')
                       buffer = 6;
               else if(locbuf[1] == '7')
                       buffer = 7;
               else if(locbuf[1] == '8')
                       buffer = 8;
               else if(locbuf[1] == '9')
                       buffer = 9;


               if(locbuf[0] == 'a' || locbuf[0] == 'A')
                  buffer += 10*16;
               else if(locbuf[0] == 'b' || locbuf[0] == 'B')
                       buffer += 11*16;
               else if(locbuf[0] == 'c' || locbuf[0] == 'C')
                       buffer += 12*16;
               else if(locbuf[0] == 'd' || locbuf[0] == 'D')
                       buffer += 13*16;
               else if(locbuf[0] == 'e' || locbuf[0] == 'E')
                       buffer += 14*16;
               else if(locbuf[0] == 'f' || locbuf[0] == 'F')
                       buffer += 15*16;
               else if(locbuf[0] == '0')
                       buffer += 0*16;
               else if(locbuf[0] == '1')
                       buffer += 1*16;
               else if(locbuf[0] == '2')
                       buffer += 2*16;
               else if(locbuf[0] == '3')
                       buffer += 3*16;
               else if(locbuf[0] == '4')
                       buffer += 4*16;
               else if(locbuf[0] == '5')
                       buffer += 5*16;
               else if(locbuf[0] == '6')
                       buffer += 6*16;
               else if(locbuf[0] == '7')
                       buffer += 7*16;
               else if(locbuf[0] == '8')
                       buffer += 8*16;
               else if(locbuf[0] == '9')
                       buffer += 9*16;

                  val8 = buffer & 0x000000ff;


          return val8;

}

static u16 TransformToWord(u8 val8_0, u8 val8_1)
{

      u16 val16;
      val16 = val8_1;
      val16 = val16 << 8 | val8_0;

      return val16;

}

static ssize_t UUID_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{


        u8  val8_0, val8_1;
        u16 val16;
        int i;
	printk("UUID recevier lenth is %d, content is %s\n",strlen(buf),buf);
	 Clear_Index();
        
        for(i=0;i<16;i++)
           {
                val8_0 = TransformCharToByte(buf,62-4*i);
                val8_1 = TransformCharToByte(buf,60-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x61,val16);
                msleep(10);
           }
 

      return n;
}

static ssize_t BatCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x00);   
        msleep(10);
        s += sprintf(s, "%d\n", ret);   
        return (s - buf);
}

static ssize_t BatCapacity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

/* gary-20110613 begin */
/* for read battery info */
static ssize_t BatFullCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x07);
        msleep(100);
        s += sprintf(s, "%d\n", ret);
        return (s - buf);
}

static ssize_t BatFullCapacity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}
/* gary-20110613 end */
static ssize_t BatDesignCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);
        msleep(100);
        s += sprintf(s, "%d\n", ret);
        return (s - buf);
}

static ssize_t BatDesignCapacity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}
 
static ssize_t BTMAC_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[6] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "BTMAC_show EEPROM not Ready\n");
       	return (s - buf);
	}

        for(i=0;i<=2;i++)
           {
               val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x62);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }

        s += sprintf(s, "%02x%02x%02x%02x%02x%02x\n",val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

}

static ssize_t BTMAC_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;

	 Clear_Index();

        for(i=0;i<=2;i++)
           {
                val8_0 = TransformCharToByte(buf,10-4*i);
                val8_1 = TransformCharToByte(buf,8-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x63,val16);
                msleep(10);
           }

        return n;
}
static ssize_t WIFIMAC_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[6] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "WIFIMAC_show EEPROM not Ready\n");
       	return (s - buf);
	}

        for(i=0;i<=2;i++)
           {
               val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x64);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }

        s += sprintf(s, "%02x%02x%02x%02x%02x%02x\n",val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

}

static ssize_t WIFIMAC_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{       
        u8  val8_0, val8_1;
        u16 val16;
        int i;

	 Clear_Index();

        for(i=0;i<=2;i++)
           {
                val8_0 = TransformCharToByte(buf,10-4*i);
                val8_1 = TransformCharToByte(buf,8-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x65,val16);
                msleep(10);
           }

        return n;
}


static ssize_t BatStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        int ACStatus;
        s32 Capacity, Present;
        Present = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);   //read designcapacity to judge present
        msleep(100);
        Capacity = i2c_smbus_read_word_data(EC_Bat_device->client,0x00);  //read capacity
        ACStatus = gpio_get_value(gpio);                                   //read ac exist gpio, 0 is exist

        if(Present == 0)
          {
                  s += sprintf(s, "Absence\n");
          }
        else
          {
                  if(Capacity < 100)
                    {
               		if(ACStatus == 1)
                 	 s += sprintf(s, "Discharging\n");
              		else
                  	 s += sprintf(s, "Charging\n");
                    }
                  else
                    {
                        s += sprintf(s, "Full\n");
                    }
          }
        msleep(100);
        return (s - buf);
}

static ssize_t BatStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

/* compal indigo-Ryan 20110315 begin */
static ssize_t ChargerStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
		char * s = buf;
		s32 ret;
		ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x0b);
		msleep(100);
		s += sprintf(s, "%d\n", ret);
		return (s - buf);
}
static ssize_t ChargerStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
		return n;
}
static ssize_t UsbStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
		char * s = buf;
		s32 ret;
		ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x5b);
		msleep(100);
		s += sprintf(s, "%d\n", ret);
		return (s - buf);
}
static ssize_t UsbStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
		return n;
}
/* compal indigo-Ryan 20110315 end */

static ssize_t ECRead_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        u16 val16;
        s32 val32;


        val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0xF1);
        val16 = val32 & 0x0000ffff;

        s += sprintf(s, "0x%04x\n",val16);

        return (s - buf);

}

//echo addresshbyte_addresslbyte > ECRead
static ssize_t ECRead_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);

        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data(EC_Bat_device->client,0xF0,val16);

        return n;
}

static ssize_t ECWrite_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "ECWrite");
        return (s - buf);

}

//echo addresshbyte_addresslbyte_val > ECWrite
static ssize_t ECWrite_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

      
        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);
        val16 = TransformToWord(val8_0, val8_1);
   
        i2c_smbus_write_word_data(EC_Bat_device->client,0xF0,val16);   //write address
        
        val8_1 = 0;
        val8_0 = TransformCharToByte(buf,4);
        val16 = TransformToWord(val8_0, val8_1);
    
        i2c_smbus_write_word_data(EC_Bat_device->client,0xF2,val16);   //write value 00xx
        
        return n;
}


static ssize_t Shutdown_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "Shutdown");
        return (s - buf);
}

static ssize_t Shutdown_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x52,0);
        msleep(100);
        return n;
}

static ssize_t Suspend_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "Suspend");
        return (s - buf);
}

static ssize_t Suspend_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x53,0);
        msleep(100);
        return n;
}

static ssize_t Coldboot_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",Coldboot_val);
        return (s - buf);
}

static ssize_t Coldboot_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        Coldboot_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x55,Coldboot_val);
        msleep(100);
        return n;

}

static ssize_t RebootAfterEcUpdate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "RebootAfterEcUpdate");
        return (s - buf);
}

static ssize_t RebootAfterEcUpdate_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        int buffer;
        buffer = atoi(buf) & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0xBA, buffer);
        msleep(100);
        return n;
}

static ssize_t Resume_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",Resume_val);
        return (s - buf);
}

static ssize_t Resume_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
 
        int buffer;
        buffer = atoi(buf);
        Resume_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x56,Resume_val);
        msleep(100);
        return n;

}


static ssize_t RecoveryMode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "RecoveryMode");
        return (s - buf);
}

static ssize_t RecoveryMode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        s16 val;
        buffer = atoi(buf);
        val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x58,val);
        msleep(100);
        return n;

}




static ssize_t ECflashwrite_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
       
        char * s = buf;
        s += sprintf(s, "ECflashwrite");
        return (s - buf);

}

//echo cmd_val16 > ECflashwrite
static ssize_t ECflashwrite_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        u16 val16;
        val16 = TransformToWord(buf[2], buf[1]);
        i2c_smbus_write_word_data(EC_Bat_device->client, buf[0], val16);   

        return n;

}


char cmd;

static ssize_t ECflashread_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

        s32 ret;


        ret = i2c_smbus_read_word_data(EC_Bat_device->client,cmd);

        buf[0] = (ret & 0x0000ff00) >> 8; 
        buf[1] = ret & 0x000000ff;
        
        return 2; 

}

static ssize_t ECflashread_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        cmd = buf[0];
        return n;

}

static ssize_t MicSwitch_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",MicSwitch_val);
        return (s - buf);
}


//echo 0 > MicSwitch(front) ,  echo 1 > MicSwitch(back), echo 2 > MicSwitch(normal)
//echo 3 > MicSwitch(echo cancellation test), echo 4 > MicSwitch(disable echo cancellation)
static ssize_t MicSwitch_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        MicSwitch_val = buffer & 0x0000FFFF;
        if(MicSwitch_val < 0 || MicSwitch_val > 4)
           {
               printk("MicSwitch: echo value should be 0 to 4\n");
               goto fail;
           }
        i2c_smbus_write_word_data(EC_Bat_device->client,0x44,MicSwitch_val);
        msleep(100);
fail:
        return n;
}



static ssize_t ThreeGPower_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",ThreeGPower_val);
        return (s - buf);
}


//echo 1 > ThreeGPower(poweron) ,  echo 0 > ThreeGPower(poweroff), echo 2 > ThreeGPower(backtoOriginstate)
static ssize_t ThreeGPower_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        ThreeGPower_val = buffer & 0x0000FFFF;
        if(ThreeGPower_val != 0 && ThreeGPower_val != 1 && ThreeGPower_val != 2)
           {
               printk("ThreeGPower: echo value should be 0 or 1 or 2\n");
               goto fail;
           }
        i2c_smbus_write_word_data(EC_Bat_device->client,0x45,ThreeGPower_val);
        msleep(100);
fail:
        return n;
}

void enable_ThreeGPower(void) {

	ThreeGPower_val = 1;
	i2c_smbus_write_word_data(EC_Bat_device->client,0x45,ThreeGPower_val);
	msleep(100);
}

SYMBOL_EXPORT(enable_ThreeGPower);

void disable_ThreeGPower(void) {

	ThreeGPower_val = 0;
	i2c_smbus_write_word_data(EC_Bat_device->client,0x45,ThreeGPower_val);
	msleep(100);
}

SYMBOL_EXPORT(disable_ThreeGPower);

static ssize_t SerialNumber_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++) {
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10) {
		s += sprintf(s, "ffffffffffffffffffffffffffffffff\n");
		return (s - buf);
	}

        for(i=0;i<=7;i++) {
		val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x66);
		val16 = val32 & 0x0000ffff;
		TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
		msleep(10);
	}


        s += sprintf(s,
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		val8[15],val8[14],val8[13],val8[12],val8[11],val8[10],
		val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

}

static ssize_t SerialNumber_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;

	 Clear_Index();

        for(i=0;i<=7;i++) {
		val8_0 = TransformCharToByte(buf,30-4*i);
		val8_1 = TransformCharToByte(buf,28-4*i);

		val16 = TransformToWord(val8_0, val8_1);
		i2c_smbus_write_word_data(EC_Bat_device->client,0x67,val16);
		msleep(10);
	}

	//val8_0 = TransformCharToByte(buf,0);
	//val8_1 = 0;
	
	//val16 = TransformToWord(val8_0, val8_1);
	//i2c_smbus_write_word_data(EC_Bat_device->client,0x67,val16);
	//msleep(10);

        return n;
}

static ssize_t ECflashMode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n", ECflashMode);
        return (s - buf);
}

//echo 1 > ECflashMode (ecflash on)
//echo 0 > ECflashMode (ecflash off)
static ssize_t ECflashMode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        ECflashMode = buffer & 0x00000001;
        if(ECflashMode == 0)
          {
               setup_timer(&poll_timer, tegra_battery_poll_timer_func, 0);
               mod_timer(&poll_timer, jiffies + msecs_to_jiffies(batt_status_poll_period));
          }
        else
          {
               del_timer_sync(&poll_timer);
          }
        return n;

}



static ssize_t SkuNumber_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        
        char * s = buf;
        u8 val8[2] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "SkuNumber_show EEPROM not Ready\n");
       	return (s - buf);
	}

        val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x68);
        val16 = val32 & 0x0000ffff;
        TransformToByte(val16, &val8[0], &val8[1]);
        msleep(10);

        s += sprintf(s, "%02x%02x\n",val8[1],val8[0]);

        return (s - buf);

}



static ssize_t SkuNumber_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        u8  val8_0, val8_1;
        u16 val16;

	 Clear_Index();

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);

        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data(EC_Bat_device->client,0x69,val16);
        msleep(10);

        return n;
}



//echo 1 > LEDAndroidOff
static ssize_t LEDAndroidOff_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "LEDAndroidOff");
        return (s - buf);
}

static ssize_t LEDAndroidOff_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data(EC_Bat_device->client,0x5a,0);
        msleep(100);
        return n;
}

static ssize_t ManufactureDate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i;
     	char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "ManufactureDate_show EEPROM not Ready\n");
       	return (s - buf);
	}
 
        for(i=0;i<=1;i++)
        {
        	val32 = i2c_smbus_read_word_data(EC_Bat_device->client, 0x6e);
                val16 = val32 & 0x0000ffff;
                TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
                msleep(10);
        }

        s += sprintf(s, "%02x%02x-%02x-%02x\n",val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);
}

static ssize_t ManufactureDate_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

	 Clear_Index();

        val8_0 = TransformCharToByte(buf,6);
        val8_1 = TransformCharToByte(buf,4);
        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data(EC_Bat_device->client,0x6f,val16);
        msleep(10);

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);
        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data(EC_Bat_device->client,0x6f,val16);
        msleep(10);

	return n;
}

static ssize_t SerialNumberwithoutBarcode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i;
     	char * s = buf;
	u32 val32;
	u16 val16;
	u8 val8[22] = {0};

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "SerialNumberwithoutBarcode_show EEPROM not Ready\n");
       	return (s - buf);
	}
 
        for(i=10;i>=0;i--)
        {
		val32 = i2c_smbus_read_word_data(EC_Bat_device->client, 0x6a);
		val16 = val32 & 0x0000ffff;
                msleep(10);
		val8[2*i+1] = val16 & 0x000000ff;
		val8[2*i] = (val16 >> 8) & 0x000000ff;
        }

        s += sprintf(s, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", val8[21],val8[20],val8[19],val8[18],val8[17],val8[16],val8[15],val8[14],val8[13],val8[12],val8[11],val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

	return (s - buf);
}
static ssize_t SerialNumberwithoutBarcode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

	u8  val8_0, val8_1;
        u16 val16;
        int i;

	Clear_Index();

        for(i=0;i<11;i++)
        {
        	val8_0 = buf[2*i];
                val8_1 = buf[2*i+1];

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x6b,val16);

                msleep(10);
        }

	return n;
}

static ssize_t IMEIwithBarcode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i,j=0;
        char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "IMEIwithBarcode_show EEPROM not Ready\n");
       	return (s - buf);
	}

        for(i=0;i<=3;i++)
        {
                val32 = i2c_smbus_read_word_data(EC_Bat_device->client, 0x6c);
                val16 = val32 & 0x0000ffff;
                TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
                msleep(10);
        }

	if(val8[0]&0x80)
		j+=8;
	if(val8[0]&0x40)
		j+=4;
	if(val8[0]&0x20)
		j+=2;
	if(val8[0]&0x10)
		j+=1;
	
        s += sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%d\n",val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],j);

	return (s - buf);
}
static ssize_t IMEIwithBarcode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u8  val8_0, val8_1;
        u16 val16;
        int i;

	Clear_Index();

        for(i=0;i<=3;i++)
        {
        	val8_0 = TransformCharToByte(buf,14-4*i);
                val8_1 = TransformCharToByte(buf,12-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x6d,val16);
                msleep(10);
        }

	return n;
}



static ssize_t Reset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "Reset");
        return (s - buf);
}

static ssize_t Reset_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        i2c_smbus_write_word_data(EC_Bat_device->client,0x54,0);
        return n;

}


static ssize_t BatCurrent_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        s16 cur;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x03);  
        cur = ret & 0x0000ffff;
        msleep(10);
        s += sprintf(s, "%d\n", cur);   
        return (s - buf);
}

static ssize_t BatCurrent_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

/* compal indigo-Howard Chang 20100412 begin*/
// the path for application updates gas gauage firmware
static ssize_t ECNameSpaceReadAddress_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	u16 tempCommand;
	s32 result;
	
	tempCommand = ECNameSpaceReadAddress;
	if(i2c_smbus_write_word_data(EC_Bat_device->client,0x82,tempCommand) != 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x80);
	result >>= 8;
	//printk(KERN_ERR "%s read result %x\n", __FUNCTION__,result);
	
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_read_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
	{
		*buf = result;
		return 1;
	}
	
}

static ssize_t ECNameSpaceReadAddress_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	ECNameSpaceReadAddress = *buf;
	//printk(KERN_ERR "%s get address %x\n", __FUNCTION__,ECNameSpaceReadAddress);
	return n;
}

static ssize_t ECNameSpaceWriteAddress_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return 0;
}


static ssize_t ECNameSpaceWriteAddress_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u16 writeCommand = ((*buf) << 8) + *(buf + 1);
	udelay(750);
	//printk(KERN_ERR "%s writeCommand %x\n", __FUNCTION__,writeCommand);
	if(i2c_smbus_write_word_data(EC_Bat_device->client,0x81,writeCommand) != 0)
		printk(KERN_ERR "%s error\n", __FUNCTION__);
	return n;
}
/* compal indigo-Howard Chang 20100412 end */
/* compal indigo-Howard Chang 20100415 begin*/
static ssize_t PassCode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	s32 result;
       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "PassCode_show EEPROM not Ready\n");
       	return (s - buf);
	}

	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x72);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
	{
		*buf = (result & 0xff00) >> 8;
		*(buf+1) = result & 0xff;
	}

	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x72);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
	{
		*(buf+2) = (result & 0xff00) >> 8;
		*(buf+3) = result & 0xff;
	}
	return 4;
}


static ssize_t PassCode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u16 writeCommand;
	
	if(n < 4)//too short, size should be 4 or greater tahn 4 at least.
	{
		printk(KERN_ERR "%s error, buf size less than 4\n", __FUNCTION__);
		return n;
	}
	
	Clear_Index();

	writeCommand = (*buf << 8) + *(buf + 1);
	//printk(KERN_ERR "%s writeCommand %x\n", __FUNCTION__,writeCommand);
       if(i2c_smbus_write_word_data(EC_Bat_device->client,0x73,writeCommand) != 0)
       {
		printk(KERN_ERR "%s error\n", __FUNCTION__);
		return n;
       }
	
	writeCommand = (*(buf+2) << 8) + *(buf + 3);
	//printk(KERN_ERR "%s writeCommand %x\n", __FUNCTION__,writeCommand);
       if(i2c_smbus_write_word_data(EC_Bat_device->client,0x73,writeCommand) != 0)
       {
		printk(KERN_ERR "%s error\n", __FUNCTION__);
		return n;
       }   
	return n;
}
/* compal indigo-Howard Chang 20100415 end */


/* compal indigo-Howard Chang 20100415 begin*/
static ssize_t BatLife_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u16 writeCommand;
	
	if(*buf == '0')
		writeCommand = 0;
	else if(*buf == '1')
		writeCommand = 1;
	else
		return n;


       if(i2c_smbus_write_word_data(EC_Bat_device->client,0x59,writeCommand) != 0)
       {
		printk(KERN_ERR "%s error\n", __FUNCTION__);
		return n;
       }

	return n;
}

static ssize_t BatLife_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return 0;
}

static ssize_t BatLifeTime_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

static ssize_t BatLifeTime_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	s32 result,count;
	int i,j;
	char cbuff;
	char * s = buf;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "BatLifeTime_show EEPROM not Ready\n");
       	return (s - buf);
	}

	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x70);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error 1\n", __FUNCTION__);
		return 0;
	}
	count = result;
	
	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x70);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error 2\n", __FUNCTION__);
		return 0;
	}
	count += result << 16;
	
	//int to decimal ascii
	for(i =0;i  < 100;i++)
	{
		buf[i] = (count % 10) + 48;
		count /= 10;
		if(count == 0)
			break;
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

/* compal indigo-Howard Chang 20100422 end */

static ssize_t BoardID_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        s16 cur;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x32);
        cur = ret & 0x0000ffff;
        msleep(10);
        s += sprintf(s, "%d\n", cur);
        return (s - buf);
}

static ssize_t BoardID_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}


static ssize_t GPSPower_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",GPS_val);
        return (s - buf);
}

//echo 0 > GPS(off), echo 1 > GPS(on)
static ssize_t GPSPower_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
        buffer = atoi(buf);
        GPS_val = buffer & 0x0000FFFF;
        if(GPS_val == 0)
           {
        	i2c_smbus_write_word_data(EC_Bat_device->client,0x47,0);
           }
        else if(GPS_val == 1)
           {
        	i2c_smbus_write_word_data(EC_Bat_device->client,0x4a,0);
           }
        else
           {
               printk("GPS: echo value should be 0 or 1\n");
               goto fail;
           }
fail:
        return n;

}

/*
Bit0: If sim card plugs in/out, it will be 1
Bit1: GPS status, 1 is 3g/gps power on, 0 is power off
Bit2: Echo cancellation
remaining bits wait to be defined
*/
static ssize_t DeviceStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        char binary[20] = {0};
        int i;
        s32 ret;
        u16 cur;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x48);
        cur = ret & 0x0000ffff;

        for(i=15 ; i>=0; i--)
           {
                binary[i] = cur & 0x1 ? '1' : '0';
                cur >>= 1;
           }

        s += sprintf(s, "%s\n", binary);
        return (s - buf);
}

static ssize_t DeviceStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
        u16 val;
        buffer = atoi(buf);
        val = buffer & 0x0000FFFF;
	i2c_smbus_write_word_data(EC_Bat_device->client,0x49,val);
        return n;
}


static ssize_t PsensorPower_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n", PsensorPower_val);
        return (s - buf);
}

//echo 0 > PsensorPower(off), echo 1 > PsensorPower(on), echo 2 > PsensorPower(normal control)
static ssize_t PsensorPower_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        int buffer;
        buffer = atoi(buf);
        PsensorPower_val = buffer & 0x0000FFFF;
        if(PsensorPower_val < 0 || PsensorPower_val > 2)
           {
               printk("PsensorPower: echo value should be 0 to 2\n");
               goto fail;
           }
        i2c_smbus_write_word_data(EC_Bat_device->client,0x4b, PsensorPower_val);
        msleep(100);
fail:
        return n;

}
//Eric 0520 start
//Battery lock
static ssize_t BatLock_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
/*        s32 ret;
        s8 byteret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x5D);
        byteret = ret & 0x000000FF;
        s += sprintf(s, "%d\n",byteret);*/
	 s += sprintf(s, "%s\n","ok\n");
        return (s - buf);
}
static ssize_t BatLock_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        int buffer;
        s16 BatCtlEnable_val;
        buffer = atoi(buf);
        BatCtlEnable_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x5D,BatCtlEnable_val);
        msleep(100);
        return n;
}
//Eric 0520 end

/* compal indigo-Howard Chang 20100524 begin*/
static ssize_t NBUsbChargingCurrent_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	s32 result;
	
	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x5e);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
		return sprintf(buf, "%d\n", result);
}


static ssize_t NBUsbChargingCurrent_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u16 writeCommand;
	
	if(*buf == '0')
		writeCommand = 0;
	else if(*buf == '1')
		writeCommand = 1;
	else
		return n;
	
	if(i2c_smbus_write_word_data(EC_Bat_device->client,0x5e,writeCommand) != 0)
		printk(KERN_ERR "%s error\n", __FUNCTION__);
	return n;
}
/* compal indigo-Howard Chang 20100524 end */

/* compal indigo-Howard Chang 20100527 begin*/
static ssize_t SysStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	s32 result;
       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "SysStatus_show EEPROM not Ready\n");
       	return (s - buf);
	}

	result = i2c_smbus_read_word_data(EC_Bat_device->client,0x7E);
	if(result < 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
		return sprintf(buf, "%d\n", result);
}


static ssize_t SysStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int value = atoi(buf);
	s16 writeValue = value & 0x0000FFFF;
	Clear_Index();

	if(i2c_smbus_write_word_data(EC_Bat_device->client,0x7F,writeValue) != 0)
		printk(KERN_ERR "%s error\n", __FUNCTION__);
	return n;
}
/* compal indigo-Howard Chang 20100527 end */

/* Compal Earvin 20110615 begin*/
static ssize_t BatteryUpdateMode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        s8 byteret;
        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x0C);
        byteret = ret & 0x000000FF;
        s += sprintf(s, "%d\n",byteret);
        return (s - buf);
}


static ssize_t BatteryUpdateMode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
        s16 BatCtlEnable_val;
        buffer = atoi(buf);
        BatCtlEnable_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data(EC_Bat_device->client,0x0C,BatCtlEnable_val);
        msleep(100);
        return n;
}
/* Compal Earvin 20110615 end */

/* compal indigo-Howard Chang 20100603 begin*/
int GetMultiByteECValue(char * buf,int length,int address)
{
	s32 count,x,flag = length % 2;
	s32 result;
	char *cBuf = buf;

	if(flag == 1)
		count = length / 2 + 1;
	else
		count = length / 2;

	for(x = 0;x < count;x++)
	{
		result = i2c_smbus_read_word_data(EC_Bat_device->client,address);
                msleep(10);
		if(result < 0)
		{
			printk(KERN_ERR "%s i2c_smbus_read_word_data error\n", __FUNCTION__);
			return -1;
		}
		*cBuf++ = result & 0xff;
		if(x == count - 1 && flag == 1)
			break;
		*cBuf++ = (result & 0xff00) >> 8;
	}

	return 0;
}

int PutMultiByteECValue(const char * buf,int length,int address)
{
	s32 flag =  length % 2;
	s32 x,count;
	u16 writeCommand;
	char *cBuf = (char *)buf;

	if(flag == 1)
		count = length / 2 + 1;
	else
		count = length / 2;


	for(x = 0;x < count;x++)
	{
		if(x == count - 1 && flag == 1)
			writeCommand = *cBuf;
		else
			writeCommand = *cBuf | *(cBuf + 1) << 8;

		cBuf += 2;
		if(i2c_smbus_write_word_data(EC_Bat_device->client,address,writeCommand) != 0)
		{
			printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
			return -1;
		}
                msleep(10);
	}
	return 0;
}


static ssize_t MTMSN_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "MTMSN_show EEPROM not Ready\n");
       	return (s - buf);
	}

	if(GetMultiByteECValue( buf,16,0x74) != 0)
	{
		printk(KERN_ERR "%s GetMultiByteECValue error\n", __FUNCTION__);
		return 0;
	}
	else
		return 16;
}


static ssize_t MTMSN_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	if(n < 16)
	{
		printk(KERN_ERR "%s length is too short\n", __FUNCTION__);
		return n;
	}

	Clear_Index();

	if(PutMultiByteECValue(buf,16,0x75) != 0)
		printk(KERN_ERR "%s PutMultiByteECValue error\n", __FUNCTION__);

	return n;
}
static ssize_t PlanarSN_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "PlanarSN_show EEPROM not Ready\n");
       	return (s - buf);
	}

	if(GetMultiByteECValue( buf,22,0x76) != 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
		return 22;
}


static ssize_t PlanarSN_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	if(n < 22)
	{
		printk(KERN_ERR "%s length is too short\n", __FUNCTION__);
		return n;
	}	

	Clear_Index();

	if(PutMultiByteECValue(buf,22,0x77) != 0)
		printk(KERN_ERR "%s PutMultiByteECValue error\n", __FUNCTION__);

	return n;
}

static ssize_t Brandname_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "Brandname_show EEPROM not Ready\n");
       	return (s - buf);
	}

	if(GetMultiByteECValue( buf,15,0x78) != 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
		return 15;
}


static ssize_t Brandname_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	if(n < 15)
	{
		printk(KERN_ERR "%s length is too short\n", __FUNCTION__);
		return n;
	}	

	Clear_Index();

	if(PutMultiByteECValue(buf,15,0x79) != 0)
		printk(KERN_ERR "%s PutMultiByteECValue error\n", __FUNCTION__);

	return n;
}

static ssize_t MFGBuildDate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

       char * s = buf;
	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "MFGBuildDate_show EEPROM not Ready\n");
       	return (s - buf);
	}

	if(GetMultiByteECValue( buf,10,0x7a) != 0)
	{
		printk(KERN_ERR "%s i2c_smbus_write_word_data error\n", __FUNCTION__);
		return 0;
	}
	else
		return 10;
}


static ssize_t MFGBuildDate_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

	printk("BuildDate recevier lenth is %d, content is %s\n",strlen(buf),buf);
	if(n < 10)
	{
		printk(KERN_ERR "%s length is too short\n", __FUNCTION__);
		return n;
	}	

	Clear_Index();

	if(PutMultiByteECValue(buf,10,0x7b) != 0)
		printk(KERN_ERR "%s PutMultiByteECValue error\n", __FUNCTION__);

	return n;
}
static ssize_t AssetNumber_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

       char * s = buf;
	int count,i;
       u8 val8[30] = {0};
       u16 val16;
       s32 val32;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "AssetNumber_show EEPROM not Ready\n");
       	return (s - buf);
	}

        for(i=0;i<15;i++)
        {
               val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x7c);
               val16 = val32 & 0x0000ffff;
               memmove(&val8[i*2],&val16,2);
               msleep(10);
        }
        s += sprintf(s, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
                     val8[0],val8[1],val8[2],val8[3],val8[4],val8[5],val8[6],val8[7],val8[8],val8[9],val8[10],val8[11],val8[12],val8[13],val8[14],
                     val8[15],val8[16],val8[17],val8[18],val8[19],val8[20],val8[21],val8[22],val8[23],val8[24],val8[25],val8[26],val8[27],val8[28],val8[29]);
        return (s - buf);
}


static ssize_t AssetNumber_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;
	if(n < 30)
	{
		printk(KERN_ERR "%s length is too short\n", __FUNCTION__);
		return n;
	}
	Clear_Index();

        for(i=0;i<15;i++)
        {
                memmove(&val16,(buf+i*2),2);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x7d,val16);
                msleep(10);
        }

	return n;
}
/* compal indigo-Howard Chang 20100603 end */
/* sacdar-20110622 begin */
/* send a command to disable EC watchdog if boot finish, to fix Coldboot fail issue */
static ssize_t EcWatchDogDisable_show(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int i = 0;

	for(i=0;i<10;i++) {
		if(!i2c_smbus_write_word_data(EC_Bat_device->client,0x46,0))
			return n;
		msleep(100);
	}
	return n;
}
static ssize_t EcWatchDogDisable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int i = 0;

	for(i=0;i<10;i++) {
		if(!i2c_smbus_write_word_data(EC_Bat_device->client,0x46,0))
			return n;
		msleep(100);
	}
	return n;
}
/* sacdar-20110622 end */

static ssize_t FactoryStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[8] = {0};
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
 		s += sprintf(s, "FactoryStatus EEPROM not Ready\n");
       	return (s - buf);
	}

        
        for(i=0;i<4;i++)
           {
               val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0x83);
               val16 = val32 & 0x0000ffff;
		 memmove(&val8[i*2],&val16,2);
//               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }
        s += sprintf(s, "%c%c%c%c%c%c%c%c\n",
				val8[0],val8[1],val8[2],val8[3],val8[4],val8[5],val8[6],val8[7]);         

        return (s - buf);
}
static ssize_t FactoryStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;

	 Clear_Index();
        
        for(i=0;i<4;i++)
           {
//                val8_0 = TransformCharToByte(buf,14-4*i);
//                val8_1 = TransformCharToByte(buf,12-4*i);
		  memmove(&val16,(buf+i*2),2);
//                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data(EC_Bat_device->client,0x84,val16);
                msleep(10);
           }

      return n;
}

static ssize_t ECFwStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        u8 val8;
        u16 val16;
        s32 val32;

	int count;
	Clear_Index();
	for(count=0 ;count < 10; count++)
	{
		if(Wait_For_Ready ())
			break;
		mdelay(500);
	}
	if(count == 10)
	{
		s += sprintf(s, "ECFwStatus EEPROM not Ready\n");
		return (s - buf);
	}

       val32 = i2c_smbus_read_word_data(EC_Bat_device->client,0xb0);
       val16 = val32 & 0x0000ff00;
	val8 = val16 >> 8 & 0x00ff;

       s += sprintf(s, "%02x\n", val8);

       return (s - buf);
}
static ssize_t ECFwStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

      return n;
}


debug_attr(EcVer);
debug_attr(UUID);
debug_attr(PowerLED);
debug_attr(ChargeLED);
debug_attr(OriStsLED);
debug_attr(OffLED);
debug_attr(BatCtlEnable);
debug_attr(BatCtlDisable);
debug_attr(BatCapacity);
debug_attr(BatDesignCapacity);
debug_attr(BatStatus);
/* compal indigo-Ryan 20110315 begin */
debug_attr(ChargerStatus);
debug_attr(UsbStatus);
debug_attr(BatLock);
/* compal indigo-Ryan 20110315 end */
/* gary-20110613 begin */
debug_attr(BatFullCapacity);
/* gary-20110613 end */
debug_attr(BTMAC);
debug_attr(WIFIMAC);
debug_attr(ECRead);
debug_attr(ECWrite);
debug_attr(Shutdown);
debug_attr(Suspend);
debug_attr(Coldboot);
debug_attr(RebootAfterEcUpdate);
debug_attr(Resume);
debug_attr(RecoveryMode);
debug_attr(ECflashwrite);
debug_attr(ECflashread);
debug_attr(MicSwitch);
debug_attr(ThreeGPower);
debug_attr(SerialNumber);
debug_attr(ECflashMode);
debug_attr(SkuNumber);
debug_attr(LEDAndroidOff);
debug_attr(ManufactureDate);
debug_attr(SerialNumberwithoutBarcode);
debug_attr(IMEIwithBarcode);
debug_attr(Reset);
debug_attr(BatCurrent);
/* compal indigo-Howard Chang 20100412 begin*/
debug_attr(ECNameSpaceReadAddress);
debug_attr(ECNameSpaceWriteAddress);
/* compal indigo-Howard Chang 20100412 end */
/* compal indigo-Howard Chang 20100415 begin*/
debug_attr(PassCode);
/* compal indigo-Howard Chang 20100415 end */
/* compal indigo-Howard Chang 20100415 begin*/
debug_attr(BatLife);
debug_attr(BatLifeTime);
/* compal indigo-Howard Chang 20100422 end */
debug_attr(BoardID);
debug_attr(GPSPower);
debug_attr(DeviceStatus);
debug_attr(PsensorPower);
/* compal indigo-Howard Chang 20100524 begin*/
debug_attr(NBUsbChargingCurrent);
/* compal indigo-Howard Chang 20100524 end */
/* compal indigo-Howard Chang 20100527 begin*/
debug_attr(SysStatus);
/* compal indigo-Howard Chang 20100527 end */
/* Compal Earvin 20110615 begin*/
debug_attr(BatteryUpdateMode);
/* Compal Earvin 20110615 end*/
/* compal indigo-Howard Chang 20100603 begin*/
debug_attr(MTMSN);
debug_attr(PlanarSN);
debug_attr(Brandname);
debug_attr(MFGBuildDate);
debug_attr(AssetNumber);
/* compal indigo-Howard Chang 20100603 end */
/* sacdar-20110622 begin */
/* send a command to disable EC watchdog if boot finish, to fix Coldboot fail issue */
debug_attr(EcWatchDogDisable);
/* sacdar-20110622 end */
debug_attr(FactoryStatus);
debug_attr(ECFwStatus);

static struct attribute * g[] = {
        &EcVer_attr.attr,
	&UUID_attr.attr,
	&PowerLED_attr.attr,
	&ChargeLED_attr.attr,
	&OriStsLED_attr.attr,
	&OffLED_attr.attr,
	&BatCtlEnable_attr.attr,
	&BatCtlDisable_attr.attr,
	&BatCapacity_attr.attr,
	&BatDesignCapacity_attr.attr,
	&BatStatus_attr.attr,
/* compal indigo-Ryan 20110315 begin */
	&ChargerStatus_attr.attr,
	&UsbStatus_attr.attr,
	&BatLock_attr.attr,
/* compal indigo-Ryan 20110315 end */
/* gary-20110613 begin */
	&BatFullCapacity_attr.attr,
/* gary-20110613 end */
	&BTMAC_attr.attr,
	&WIFIMAC_attr.attr,
	&ECRead_attr.attr,
	&ECWrite_attr.attr,
	&Shutdown_attr.attr,
	&Suspend_attr.attr,
	&Coldboot_attr.attr,
	&RebootAfterEcUpdate_attr.attr,
	&Resume_attr.attr,
	&ECflashwrite_attr.attr,
	&ECflashread_attr.attr,
	&RecoveryMode_attr.attr,
	&MicSwitch_attr.attr,
	&ThreeGPower_attr.attr,
        &SerialNumber_attr.attr,
	&ECflashMode_attr.attr,
	&SkuNumber_attr.attr,
	&LEDAndroidOff_attr.attr,
	&ManufactureDate_attr.attr,
	&SerialNumberwithoutBarcode_attr.attr,
	&IMEIwithBarcode_attr.attr,
	&Reset_attr.attr,
	&BatCurrent_attr.attr,
/* compal indigo-Howard Chang 20100412 begin*/
	&ECNameSpaceReadAddress_attr.attr,
	&ECNameSpaceWriteAddress_attr.attr,
/* compal indigo-Howard Chang 20100412 end */
/* compal indigo-Howard Chang 20100415 begin*/
	&PassCode_attr.attr,
/* compal indigo-Howard Chang 20100415 end */
/* compal indigo-Howard Chang 20100415 begin*/
	&BatLife_attr.attr,
	&BatLifeTime_attr.attr,
/* compal indigo-Howard Chang 20100415 end */
	&BoardID_attr.attr,
	&GPSPower_attr.attr,
	&DeviceStatus_attr.attr,
	&PsensorPower_attr.attr,
/* compal indigo-Howard Chang 20100524 begin*/
	&NBUsbChargingCurrent_attr.attr,
/* compal indigo-Howard Chang 20100524 end */
/* compal indigo-Howard Chang 20100527 begin*/
	&SysStatus_attr.attr,
/* compal indigo-Howard Chang 20100527 end */
/* Compal Earvin 20110615 begin*/
	&BatteryUpdateMode_attr.attr,
/* Compal Earvin 20110615 end*/
/* compal indigo-Howard Chang 20100603 begin*/
	&MTMSN_attr.attr,
	&PlanarSN_attr.attr,
	&Brandname_attr.attr,
	&MFGBuildDate_attr.attr,
	&AssetNumber_attr.attr,
/* compal indigo-Howard Chang 20100603 end */
/* sacdar-20110622 begin */
/* send a command to disable EC watchdog if boot finish, to fix Coldboot fail issue */
	&EcWatchDogDisable_attr.attr,
/* sacdar-20110622 end */
	&FactoryStatus_attr.attr,
	&ECFwStatus_attr.attr,
        NULL,
};


static struct attribute_group attr_group = {
        .attrs = g,
};
//SYS_END




void SysShutdown(void )
{
     printk("SysShutdown\n");
     int i;
     for(i=0;i<10;i++)
      {		
     	if(!i2c_smbus_write_word_data(EC_Bat_device->client,0x52,0))
		return;
	mdelay(200);
      }
	return ;
}
SYMBOL_EXPORT(SysShutdown);


void SysRestart(void )
{	
     int i;
     for(i=0;i<10;i++)
      {		
     	if(!i2c_smbus_write_word_data(EC_Bat_device->client,0x55,1))
		return;
	mdelay(200);
      }
	return ;
}
SYMBOL_EXPORT(SysRestart);

u16 BoardID(void)
{

     s32 ret;
     u16 cur;
     ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x32);
     cur = ret & 0x0000ffff;
     
     return cur;
}
SYMBOL_EXPORT(BoardID);

//cloud-20111013start
//for the usb self-connect issue
bool IsIndigo = false;
void Block_The_Usb_Charge(bool block)
{
	IsIndigo = block;
}
SYMBOL_EXPORT(Block_The_Usb_Charge);
//cloud-20111013end


static int EC_Bat_get_battery_presence_and_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

        if (psp == POWER_SUPPLY_PROP_PRESENT) 
           {
                   //read designcapacity to judge present
         	ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
		if(ret == 0)          //no battery
                   {
			dev_err(&EC_Bat_device->client->dev, "%s: No battery, read again\n", __func__);
                      	msleep(500);
                      	ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
                      	if(ret == 0)
                          {
		      		dev_err(&EC_Bat_device->client->dev, "%s: No battery\n", __func__);	
				val->intval = 0;	
				return 0;	  
                          }
		        else if(ret < 0)
                          {
	                      	dev_err(&EC_Bat_device->client->dev, "%s: i2c read for BatPresent failed\n", __func__);
        	              	return -EINVAL;
                	  }
                   }
        	else if (ret < 0) 
                   {
	        	dev_err(&EC_Bat_device->client->dev,
		        	"%s: i2c read for battery presence failed\n", __func__);
	        	return -EINVAL;
	           }
              
                val->intval = 1;


           }
        else if (psp == POWER_SUPPLY_PROP_HEALTH) 
               {
      			//read designcapacity to judge present
         		ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
			if(ret == 0)          //no battery
                   	  {
				dev_err(&EC_Bat_device->client->dev, "%s: No battery, read again\n", __func__);
                      		msleep(500);
                      		ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
                      		if(ret == 0)
                          	  {
		      			dev_err(&EC_Bat_device->client->dev, "%s: No battery\n", __func__);	
					val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;	
					return 0;	  
                          	  }
		        	else if(ret < 0)
                          	  {
	                      		dev_err(&EC_Bat_device->client->dev, "%s: i2c read for BatPresent failed\n",
						 	 __func__);
        	              		return -EINVAL;
                	  	  }	
                   	  }
        		else if (ret < 0) 
                   	  {
	        		dev_err(&EC_Bat_device->client->dev,
		        	"%s: i2c read for battery presence failed\n", __func__);
	        		return -EINVAL;
	           	  }
              
	                val->intval = POWER_SUPPLY_HEALTH_GOOD;

               } 

	return 0;
}

static int EC_Bat_get_battery_property(int reg_offset,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	int BatPresent, ACStatus;
	s32 Capacity;
	s32 EC_status;
	if (psp == POWER_SUPPLY_PROP_STATUS) 
           {
                    //read designcapacity to judge present
                    //POWER_SUPPLY_PROP_PRESENT is after POWER_SUPPLY_PROP_STATUS
                 BatPresent = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
                           
                 if(BatPresent >= 0)   
                   {     
			if(BatPresent == 0)          //no battery
                   	  {
		      		dev_err(&EC_Bat_device->client->dev, "%s: No battery, read again\n", __func__);
                      		msleep(500);
                      		BatPresent = i2c_smbus_read_word_data(EC_Bat_device->client,0x08);  
                      		if(BatPresent == 0)
                        	  {
		      			dev_err(&EC_Bat_device->client->dev, "%s: No battery\n", __func__);		
					val->intval = POWER_SUPPLY_STATUS_UNKNOWN;  
                                        return 0; 
                        	  }
		        	else if(BatPresent < 0)
                          	  {
	                      		dev_err(&EC_Bat_device->client->dev, "%s: i2c read for BatPresent failed\n", 
							__func__);
        	              		return -EINVAL;
                	  	  }
                   	  }

			//read capacity
            	       Capacity = i2c_smbus_read_word_data(EC_Bat_device->client,0x00);
                      
		       if(Capacity == 0)
		         {
                                dev_err(&EC_Bat_device->client->dev, "%s: No battery capacity, read again\n", __func__);
                                msleep(500);
                                Capacity = i2c_smbus_read_word_data(EC_Bat_device->client,0x00); 
                                if(Capacity == 0)
                                  {
		      		   	dev_err(&EC_Bat_device->client->dev, "%s: No battery capacity\n", __func__);
                                  }
		      	 	else if(Capacity < 0)
                        	  {
	                      		dev_err(&EC_Bat_device->client->dev, "%s: i2c read for capacity failed\n", __func__);
        	              		return -EINVAL;
                		  }
                         }
                       else if(Capacity < 0) 
                         {
		      		dev_err(&EC_Bat_device->client->dev, "%s: i2c read for capacity failed\n", __func__);
	                	return -EINVAL;
                         }

                         //read ac exist gpio, 0 is exist
        	       ACStatus = gpio_get_value(gpio);

			/* compal indigo-Howard Chang 20110513 begin */
			/* get charging status from EC */
			EC_status = i2c_smbus_read_word_data(EC_Bat_device->client,0x0B);

      		       if(Capacity < 100)
         	  	 {
				if(ACStatus == 1)
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				else
				{
					if(EC_status == 2)
						val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
					else if(EC_status == 1)
					{
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
						//cloud-20111013start
						//for the usb self-connect issue
						if(IsIndigo)
							val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
						//cloud-20111013end	
					}
					else if(EC_status == 0)
						val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					else
						val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				}
			}
			else
			{
				if(ACStatus == 1)
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				else
				{
					if(EC_status == 2)
						val->intval = POWER_SUPPLY_STATUS_FULL;
					else if(EC_status == 1)
					{
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
						//cloud-20111013start
						//for the usb self-connect issue
						if(IsIndigo)
							val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
						//cloud-20111013end	
					}
					else if(EC_status == 0)
						val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					else
						val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				}
                   	}
      			/* compal indigo-Howard Chang 20110513 end */

                 }
                 else if(BatPresent < 0)
                 {
		      dev_err(&EC_Bat_device->client->dev, "%s: i2c read for BatPresent failed\n", __func__);
	              return -EINVAL;
                   }
           }
        else if (psp == POWER_SUPPLY_PROP_TEMP)
               {
                    ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x0a);    

		    if(ret == 0)
                      {
			   dev_err(&EC_Bat_device->client->dev, "%s: No battery temperature, read again\n", __func__);
                           msleep(500);
			   ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x0a);
                           if(ret == 0)
                             {
				  dev_err(&EC_Bat_device->client->dev, "%s: No battery temperature\n", __func__);	
				  val->intval = 0;	
 				  return 0;
                             }
		           else if(ret < 0)
                             {
	                          dev_err(&EC_Bat_device->client->dev, "%s: i2c read for temperature failed\n", __func__);
        	              	  return -EINVAL;
                	     }
		      }
                    else if(ret < 0)
                      {
		      	   dev_err(&EC_Bat_device->client->dev, "%s: i2c read for temperature failed\n", __func__);
	                   return -EINVAL;
                      }
		    val->intval = ret - 2731;   

               }
        else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW)
               {
                    ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x01);    

		    if(ret == 0)
                      {
			   dev_err(&EC_Bat_device->client->dev, "%s: No battery voltage, read again\n", __func__);
                           msleep(500);
			   ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x01);   
                           if(ret == 0)
                             {
				dev_err(&EC_Bat_device->client->dev, "%s: No battery voltage\n", __func__);
                             }
		           else if(ret < 0)
                             {
	                        dev_err(&EC_Bat_device->client->dev, "%s: i2c read for voltage failed\n", __func__);
        	              	return -EINVAL;
                	     }
		      }
                    else if(ret < 0)
                      {
		      	   dev_err(&EC_Bat_device->client->dev, "%s: i2c read for voltage failed\n", __func__);
	                   return -EINVAL;
                      }
		    val->intval = ret * 1000;    
               }
        
	return 0;
}

static int EC_Bat_get_battery_capacity(union power_supply_propval *val)
{
	s32 ret;

        ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x00);

	if(ret == 0)
          {
		dev_err(&EC_Bat_device->client->dev, "%s: No battery capacity, read again\n", __func__);
                msleep(500);
		ret = i2c_smbus_read_word_data(EC_Bat_device->client,0x00);   
                if(ret == 0)
                  {
			dev_err(&EC_Bat_device->client->dev, "%s: No battery capacity\n", __func__);
                  }
		else if(ret < 0)
                  {
	                dev_err(&EC_Bat_device->client->dev, "%s: i2c read for capacity failed\n", __func__);
        	        return -EINVAL;
                  }
	  }
	else if (ret < 0) 
          {
		dev_err(&EC_Bat_device->client->dev, "%s: i2c read for capacity failed\n", __func__);
		return -EINVAL;
	  }
	/* EC_Bat spec says that this can be >100 %
	 * even if max value is 100 % */
	val->intval = ( (ret >= 100) ? 100 : ret);

	printk("battery capacity: %d\n", val->intval);
	return 0;
}
/* carry-0617 begin */
/* check Dock state */
int Get_Dock_Status()
{
  return gpio_get_value(DOCK_ON); 
}
SYMBOL_EXPORT(Get_Dock_Status);
/* carry-0617 end */

/* compal indigo-Howard Chang 20110513 begin */
/* add correct cradle detection & code flow & timing */
static int EC_Bat_get_ac_status(union power_supply_propval *val)
{
	int ACStatus;
	unsigned int pUSB_STATUS;

	msleep(100);
	
	//read ac exist gpio, 0 is exist
	//no AC IN, report ac doesn't present
	if(gpio_get_value(gpio) == 1)
	{
		val->intval = 0;
		return 0;
	}
	//Dock in. report AC present
	if(gpio_get_value(DOCK_ON) == 1)
	{
		val->intval = 1;
		i2c_smbus_write_word_data(EC_Bat_device->client,0x5b,0x0001);
		return 0;
	}
	
	//can't identify AC/USB, report AC default
	if(USB1_reg == NULL)
	{
		if(ACStatus == 0)
		{	
			val->intval = 1;
			i2c_smbus_write_word_data(EC_Bat_device->client,0x5b,0x0001);
		}
		else
			val->intval = 0;
		return 0;
	}
	
	//get D+ D- status
	pUSB_STATUS= readl(USB1_reg+0x184);
	//printk(KERN_ERR "bq20z75_get_ac_status AC : %d USB register :%x\n",(ACStatus == 0?1:0),pUSB_STATUS);

	if((pUSB_STATUS & 0x00000c00) == 0x00000c00)
	{
		val->intval = 1;
		i2c_smbus_write_word_data(EC_Bat_device->client,0x5b,0x0001);
	}
	else
		val->intval = 0;

	return 0;
}

static int bq20z75_get_usb_status(union power_supply_propval *val)
{
	unsigned int pUSB_STATUS;
	
	msleep(100);
	

	//read ac exist gpio, 0 is exist
	//no AC IN, report usb doesn't present
	if(gpio_get_value(gpio) == 1)
	{
		val->intval = 0;
		return 0;
	}
	
	//Dock in. report USB doesn't present
	if(gpio_get_value(DOCK_ON) == 1)
	{
		val->intval = 0;
		return 0;
	}

	//can't identify AC/USB, report AC default
	if(USB1_reg == NULL)
	{
		val->intval = 0;
		return 0;
	}
	
	//get D+ D- status
	pUSB_STATUS= readl(USB1_reg+0x184);
	//printk(KERN_ERR "bq20z75_get_usb_status AC : %d USB register :%x\n",(ACStatus == 0?1:0),pUSB_STATUS);

	if((pUSB_STATUS & 0x00000c00) != 0x00000c00)
	{
		val->intval = 1;
		i2c_smbus_write_word_data(EC_Bat_device->client,0x5b,0x0000);
	}
	else
	{
		val->intval = 0;
	}

	//cloud-20111013start
	//for the usb self-connect issue
	if(IsIndigo)
		val->intval = 0;
	//cloud-20111013end
	return 0;
}
/* compal indigo-Howard Chang 20110513 end */

static int EC_Bat_get_charge_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
       switch (psp) {
              case POWER_SUPPLY_PROP_ONLINE:
                  if (EC_Bat_get_ac_status(val))
                      return -EINVAL;

                   break;
              default:
                        dev_err(&EC_Bat_device->client->dev,
                                "%s: INVALID property\n", __func__);
                        return -EINVAL;
        }
       return 0;
}

/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism */
static int bq20z75_get_usb_charge_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
       switch (psp) {
              case POWER_SUPPLY_PROP_ONLINE:
                  if (bq20z75_get_usb_status(val))
                      return -EINVAL;
                   break;
              default:
                        dev_err(&EC_Bat_device->client->dev,
                                "%s: INVALID property\n", __func__);
                        return -EINVAL;
        }
       return 0;
}
/* compal indigo-Howard Chang 20100310 end */

static int EC_Bat_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{

	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:


			if (EC_Bat_get_battery_presence_and_health(psp, val))
				return -EINVAL;

			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (EC_Bat_get_battery_capacity(val))
				return -EINVAL;
                 
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_TEMP:

			if (EC_Bat_get_battery_property(count, psp, val))
				return -EINVAL;

			break;

		default:
			dev_err(&EC_Bat_device->client->dev,
				"%s: INVALID property\n", __func__);
			return -EINVAL;
	}





	return 0;
}


extern tegra_charging;

static int EC_Bat_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc, err, i, ret;
	/* compal indigo-Ryan 20110315 begin */
	ret = gpio_request(DOCK_ON, "DOCKDetect");
	if (ret) {
		dev_err(&EC_Bat_device->client->dev,
				"%s: DOCKDetect request fail\n", __func__);
		return -EINVAL;
	}
	/* compal indigo-Howard Chang 20110513 begin */
	/* enable DOCK_ON pin */
	if(gpio_direction_input(DOCK_ON) != 0)
	{
		printk("[Power] fail to setup DOCK_ON direct in");
		return -EINVAL;
	}
	/* compal indigo-Howard Chang 20110513 end */
	/* compal indigo-Ryan 20110315 end */

        gpio = irq_to_gpio(client->irq);      //AC detect gpio
	EC_Bat_device = kzalloc(sizeof(*EC_Bat_device), GFP_KERNEL);
	if (!EC_Bat_device) {
                goto fail1;
	}
	memset(EC_Bat_device, 0, sizeof(*EC_Bat_device));

/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism. PtoV */
	USB1_reg = ioremap(0xC5000000, 0x00004000);
/* compal indigo-Howard Chang 20100310 end */

        ret = request_irq(client->irq, ac_interrupt, IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                 client->name, EC_Bat_device);
        if( ret )
        {
                dev_err(&EC_Bat_device->client->dev,
                                "%s: request_irq failed\n", __func__);
                goto fail2;
        }

	EC_Bat_device->client = client;
	i2c_set_clientdata(client, EC_Bat_device);


        
        for (i = 0; i < ARRAY_SIZE(EC_Bat_supply); i++)
            {
	         rc = power_supply_register(&client->dev, &EC_Bat_supply[i]);
	         if (rc) {
	        	dev_err(&EC_Bat_device->client->dev,
		        	"%s: Failed to register power supply\n", __func__);
	        	kfree(EC_Bat_device);
		       return rc;
                        	}
            }
	dev_info(&EC_Bat_device->client->dev,
		"%s: battery driver registered\n", client->name);

      
        batt_status_poll_period = NVBATTERY_POLLING_INTERVAL;
        setup_timer(&poll_timer, tegra_battery_poll_timer_func, 0);
        mod_timer(&poll_timer,
                jiffies + msecs_to_jiffies(batt_status_poll_period));

//SYS_START
        Ecdebug_kobj = kobject_create_and_add("EcControl", NULL);
        if (Ecdebug_kobj == NULL) {
            printk("%s: subsystem_register failed\n", __FUNCTION__);
          }
        err = sysfs_create_group(Ecdebug_kobj, &attr_group);
        if(err) {
            printk("%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
          }

//SYS_END

	if(tegra_charging)
		SysShutdown();

	return 0;


fail2:
        free_irq(client->irq, EC_Bat_device);
fail1:
        i2c_set_clientdata(client, NULL);
        kfree(EC_Bat_device);
        EC_Bat_device = NULL;

	return ret;

}

static int EC_Bat_remove(struct i2c_client *client)
{
	struct EC_Bat_device_info *EC_Bat_device = i2c_get_clientdata(client);
        int i;

        
        del_timer_sync(&poll_timer);
        if(client->irq)
           {
                free_irq(client->irq, EC_Bat_device);
           }

        for (i = 0; i < ARRAY_SIZE(EC_Bat_supply); i++)
            {
	         power_supply_unregister(&EC_Bat_supply[i]);
            }
	if (EC_Bat_device) {
		kfree(EC_Bat_device);
		EC_Bat_device = NULL;
	}
/* compal indigo-Howard Chang 20100310 begin */
/* add detecting USB charging source mechanism, release virtual address */
	iounmap(USB1_reg);
	USB1_reg = NULL;
/* compal indigo-Howard Chang 20100310 end */

	return 0;
}

static int EC_Bat_suspend(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;
	struct EC_Bat_device_info *EC_Bat_device = i2c_get_clientdata(client);


        del_timer_sync(&poll_timer);
        ret = i2c_smbus_write_word_data(EC_Bat_device->client,0x53,0);
        if (ret < 0) {
                dev_err(&EC_Bat_device->client->dev,
                        "i2c write error\n");
                return -EINVAL;
        }
        
	return 0;
}

/* any smbus transaction will wake up EC_Bat */
static int EC_Bat_resume(struct i2c_client *client)
{
        setup_timer(&poll_timer, tegra_battery_poll_timer_func, 0);
        mod_timer(&poll_timer,
                jiffies + msecs_to_jiffies(batt_status_poll_period));
      
	return 0;
}

static const struct i2c_device_id EC_Bat_id[] = {
	{ "EC_Battery", 0 },
	{},
};

static struct i2c_driver EC_Bat_battery_driver = {
	.probe		= EC_Bat_probe,
	.remove 	= EC_Bat_remove,
	.suspend	= EC_Bat_suspend,
	.resume 	= EC_Bat_resume,
	.id_table	= EC_Bat_id,
	.driver = {
		.name	= "EC_Battery",
	},
};

static int __init EC_Bat_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&EC_Bat_battery_driver);
	if (ret)
		dev_err(&EC_Bat_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);
  /* carry-0705 begin */
  /* create gpio node for Dock : /sys/devices/virtual/gpio/gpio151/value */
  printk("create gpio node for Dock : /sys/devices/virtual/gpio/gpio%d/value\n",DOCK_ON);
  gpio_export(DOCK_ON, 0);
  printk("create gpio node for Dock is finished !\n");
  /* carry-0705 end */
	return ret;
}
module_init(EC_Bat_battery_init);

static void __exit EC_Bat_battery_exit(void)
{
	i2c_del_driver(&EC_Bat_battery_driver);
}
module_exit(EC_Bat_battery_exit);

MODULE_AUTHOR("NVIDIA Graphics Pvt Ltd");
MODULE_DESCRIPTION("BQ20z75 battery monitor driver");
MODULE_LICENSE("GPL");
