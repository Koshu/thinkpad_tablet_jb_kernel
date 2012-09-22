#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define TRACE_LCD 1

#if TRACE_LCD
#define LCD(x...) printk(KERN_INFO "[phj00_lcd] " x)
#else
#define LCD(x...) do {} while (0)
#endif

static struct kobject *lcd_kobj;

struct phj00_lcd_info {
	struct i2c_client *i2c_client;
};

struct phj00_lcd_info *info_lcd = NULL;

char Edid[128] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

static ssize_t edid_store(struct kobject*, struct kobj_attribute*, char*);
static int edid_show(struct kobject*, struct kobj_attribute*, char*);

#define LCD_ATTR(_name) \
         static struct kobj_attribute _name##_attr = { \
         .attr = { .name = __stringify(_name), .mode = 0660}, \
         .show = _name##_show, \
         .store = _name##_store, \
         }

LCD_ATTR(edid);

static struct attribute * attributes[] = {
        &edid_attr.attr,
        NULL,
};

static struct attribute_group attribute_group = {
        .attrs = attributes,
};

int read_edid(char reg, char *edid, int len)
{
        int err;
        struct i2c_msg msg[2];

        if (!info_lcd->i2c_client->adapter)
                return -ENODEV;

        msg[0].addr = info_lcd->i2c_client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &reg;

        msg[1].addr = info_lcd->i2c_client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 128;
        msg[1].buf = edid;

        err = i2c_transfer(info_lcd->i2c_client->adapter, msg, 2);

        if (err != 2)
                return -EINVAL;

        return 0;
}

static ssize_t edid_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf)
{
	return 0;
}

static int edid_show(struct kobject* kobj,struct kobj_attribute* kobj_attr, char* buf)
{
	int i = 0;
	int reg = 0;
	int len = 128;
   	char dispStr[512];
   	char tmpStr[3];

   	*tmpStr = 0;
   	*dispStr = 0;

   	read_edid(reg, Edid, len);

   	for ( i = 0; i < 128; i++ )
   	{
      		sprintf( tmpStr, "%02x ", Edid[i] );
      		strcat( dispStr, tmpStr );

      		if ( i % 16 == 15 )
         	strcat( dispStr, "\n" );
   	}

   	len += sprintf(buf + i, dispStr);

   	return len;
}

static int phj00_lcd_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
        printk("phj00_lcd: probing sensor.\n");

        info_lcd = kzalloc(sizeof(struct phj00_lcd_info), GFP_KERNEL);

        if (!info_lcd)
	{
                LCD("phj00_lcd: Unable to allocate memory!\n");
                return -ENOMEM;
        }

        info_lcd->i2c_client = client;
        return 0;
}

static int phj00_lcd_remove(struct i2c_client *client)
{
        struct phj00_lcd_info *info_lcd;
        info_lcd = i2c_get_clientdata(client);
        kfree(info_lcd);

        return 0;
}

static const struct i2c_device_id phj00_lcd_id[] = {
	{ "phj00_lcd", 0 },
        { },
};

static struct i2c_driver phj00_lcd_i2c_driver = {
        .driver = {
                .name = "phj00_lcd",
                .owner = THIS_MODULE,
        },
        .probe = phj00_lcd_probe,
        .remove = phj00_lcd_remove,
	.id_table = phj00_lcd_id,
};

static int __init phj00_lcd_init(void)
{
    int err;

    lcd_kobj = kobject_create_and_add("lcd", NULL);
    if (lcd_kobj == NULL){
        LCD("lcd_edid subsystem_register failed\n");
        goto err_diag;
    }

    err = sysfs_create_group(lcd_kobj, &attribute_group);
    if(err){
        LCD("sysfs_create failed, \n");
        goto err_diag;
    }

    return i2c_add_driver(&phj00_lcd_i2c_driver);

err_diag:
    kobject_del(lcd_kobj);
    return -ENODEV;

}

static void __exit phj00_lcd_exit(void)
{
         i2c_del_driver(&phj00_lcd_i2c_driver);
}
module_init(phj00_lcd_init);
module_exit(phj00_lcd_exit);
