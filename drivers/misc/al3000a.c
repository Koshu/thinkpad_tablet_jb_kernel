/*
 *  Generic Light Sensor Driver
 *
 *  Copyright (c) 2004-2008 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
//COMPAL_START
#include <linux/init.h>
#include <linux/string.h>
//COMPAL_END
#define DRIVER_NAME    "al3000a_ls"

#define DEBUG 1
#include <linux/device.h>
#include <linux/earlysuspend.h>


struct al_data
{
//COMPAL_START
    struct work_struct work;
//COMPAL_END
    struct input_dev   *input_dev;
    struct i2c_client  *client;
    atomic_t           l_flag;   //report input flag
    struct early_suspend early_suspend;
};
static struct al_data *ls;
/***** ls control functions ***************************************/
//COMPAL_START
struct work_struct        workq;

void sample_workqueue(struct work_struct *work)
{
	struct al_data *data = container_of(work, struct al_data, work);
	__u8 level;
       int ret;

       ret = i2c_smbus_read_i2c_block_data(data->client, 0x05, 1, &level);
       if (WARN_ON(ret < 0)) {
            dev_err(&data->client->dev, "error %d reading event data\n", ret);
        return ;
    	}

       input_report_abs(data->input_dev, ABS_MISC, level);
       input_sync(data->input_dev);
	
	return ;	
}

static ssize_t data_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{	
	char data;
	i2c_smbus_read_i2c_block_data(ls->client, 0x05, 1, &data);
	return sprintf(buf, " light =%x\n", data);
}

static ssize_t data_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{	
	return count;
}

static ssize_t light_lux_show(struct kobject *, struct kobj_attribute *, char *);

static ssize_t light_lux_store(struct kobject *,
			       struct kobj_attribute *,
			       const char *,
			       size_t);

static ssize_t light_mode_show(struct kobject *, struct kobj_attribute *, char *);

static ssize_t light_mode_store(struct kobject *,
			        struct kobj_attribute *,
			        const char *,
			        size_t);
static void ls_enable(int);

static ssize_t light_lux_show(struct kobject *kobj, 
			      struct kobj_attribute *attr,
			      char *buf)
{
	char lux ;
	i2c_smbus_read_i2c_block_data(ls->client, 0x05, 1, &lux);
	return sprintf(buf, "%x\n", lux);
}

static ssize_t light_lux_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{	
	return count;
}

static ssize_t light_mode_show(struct kobject *kobj, 
			      struct kobj_attribute *attr,
			      char *buf)
{
       return sprintf(buf, "real light sensor status : %d\n(not echo 1/0 > /sys/light/light_mode status)\n",
				atomic_read(&ls->l_flag));
}

static ssize_t light_mode_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{	
	int xxx = 0;
	if ( *buf == '1' && !atomic_read(&ls->l_flag)){
                ls_enable(1);
                xxx = strlen(buf);
        }
        else
        if( !atomic_read(&ls->l_flag) ){
                ls_enable(0);
                xxx = strlen(buf);
        }
        else
                xxx = -EINVAL;

        return xxx;
}

static struct kobj_attribute light_data_attribute =
	__ATTR(light, 0660, data_show, data_store);

static struct kobj_attribute light_lux_light_lux_attribute =

	__ATTR(light_lux, 0660, light_lux_show, light_lux_store);

static struct kobj_attribute light_mode_light_mode_attribute =
	__ATTR(light_mode, 0660, light_mode_show, light_mode_store);

static struct attribute *attrs[] = {
	&light_data_attribute.attr,	
	&light_lux_light_lux_attribute.attr,
	&light_mode_light_mode_attribute.attr,
	NULL,	
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};
static struct kobject *light_kobj;
//COMPAL_END

static void ls_enable(int enable)
{
    struct i2c_client *client = ls->client;
    __u8 data = 0;

    if (enable) {
    	printk(KERN_INFO "AL3000A Light sensor enabled --->\n");
    	//Read Data to clear INT Flag
	i2c_smbus_read_i2c_block_data(client, 0x05, 1, &data);
	//Power Up & Enable ALS
	i2c_smbus_write_byte_data(client, 0x00, 0x00);
    } else {
    	printk(KERN_INFO "AL3000A Light sensor disabled --->\n");
    	//Power Down & Idle
	i2c_smbus_write_byte_data(client, 0x00, 0x0B);
    }
}

static int light_misc_open(struct inode *inode, struct file *file)
{

    return 0;
}

static int light_misc_release(struct inode *inode, struct file *file)
{
 
    return 0;
}

static int
light_misc_ioctl(struct inode *inode, struct file *file,
                 unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag = 0;

    switch (cmd) {
    case LIGHTSENSOR_IOCTL_ENABLE:
        if (copy_from_user(&flag, argp, sizeof(flag)))
            return -EFAULT;
        if (flag < 0 || flag > 1)
            return -EINVAL;
    default:
        break;
    }

    switch (cmd) {
    case LIGHTSENSOR_IOCTL_ENABLE:
        ls_enable(flag);
        atomic_set(&ls->l_flag, flag);
        printk(KERN_INFO "light_misc_ioctl: LFLAG is set to %d\n", flag);
        break;
    case LIGHTSENSOR_IOCTL_GET_ENABLED:
        flag = atomic_read(&ls->l_flag);
        printk(KERN_INFO "light_misc_ioctl: LFLAG is %d ----->>>\n", flag);
        break;
    }

    switch (cmd) {
    case LIGHTSENSOR_IOCTL_GET_ENABLED:
        if (copy_to_user(argp, &flag, sizeof(flag))) {
            return -EFAULT;
        }
        break;
    default:
        break;
    }

    return 0;
}

/*********************************************/
static struct file_operations light_misc_fops = {
        .owner = THIS_MODULE,
        .open = light_misc_open,
        .release = light_misc_release,
        .unlocked_ioctl = light_misc_ioctl,
};

static struct miscdevice light_misc_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "lightsensor",
        .fops = &light_misc_fops,
};
/*********************************************/

static int al_init(struct i2c_client *client)
{
    __u8 data;
    int ret = 0;	
	
    //F/W Initial Flow
    //Power Down & Idle
    ret = i2c_smbus_write_byte_data(client, 0x00, 0x0B);
    if (ret < 0) {
        printk(KERN_ERR "Init AL300A failed - 1\n");
        return ret;
    }

    // Integration Cycle = 4; Integration Time = 100ms;
    // Interrupt trigger when lux detection has changed 4 times 
    // at 100ms intervals. 
    ret = i2c_smbus_write_byte_data(client, 0x01, 0x11);
    if (ret < 0) {
        printk(KERN_ERR "Init AL300A failed - 2\n");
        return ret;
    }
	
    //AL3000A ADC resolution = 64 levels; Low lux threshold = 0
    ret = i2c_smbus_write_byte_data(client, 0x02, 0xA0);
    if (ret < 0) {
        printk(KERN_ERR "Init AL300A failed - 3\n");
        return ret;
    }
	
    //ALS Window Loss = 0
    //It isn't covered by shell so no window loss, need to modify at DVT
    ret = i2c_smbus_write_byte_data(client, 0x08, 0x00);
    if (ret < 0) {
        printk(KERN_ERR "Init AL300A failed - 4\n");
        return ret;
    }

    //Read Data to clear INT Flag
    ret = i2c_smbus_read_i2c_block_data(client, 0x05, 1, &data);
     if (ret < 0) {
        printk(KERN_ERR "Init AL300A failed - 5\n");
        return ret;
    }
    return ret;
}

static irqreturn_t al3000a_irq(int irq, void *dev_id)
{	
    struct al_data *data = dev_id;    
	schedule_work(&data->work);
    return IRQ_HANDLED;
}

static int al3000a_suspend(struct i2c_client *client, pm_message_t state)
{
    struct al_data *data = i2c_get_clientdata(client);
    int ret;

    if (WARN_ON(!data))
        return -EINVAL;

    disable_irq(client->irq);

    //Power Up & Enable ALS
    ret = i2c_smbus_write_byte_data(client, 0x00, 0x0B);
    if (ret < 0) {
        printk(KERN_ERR "%s: suspend failed\n", __func__);
        return ret;
    }
    else{
        printk(KERN_ERR "%s: suspend\n", __func__);
    }

    return 0;
}

static int al3000a_resume(struct i2c_client *client)
{
    struct al_data *data = i2c_get_clientdata(client);
    int ret = 0;

    if (WARN_ON(!data))
        return -EINVAL;

    ret = i2c_smbus_write_byte_data(client, 0x00, 0x00);
    if (ret < 0) {
        printk(KERN_ERR "%s: resume failed\n", __func__);
        return ret;
    }
    else{
        printk(KERN_ERR "%s: resume\n", __func__);
    }

    enable_irq(client->irq);

    return 0;
}

static void al3000a_early_suspend(struct early_suspend *handler)
{
        struct al_data *data;
        data = container_of(handler, struct al_data, early_suspend);

        pr_info("%s\n", __func__);
        al3000a_suspend(data->client, PMSG_SUSPEND);
}


static void al3000a_late_resume(struct early_suspend *handler)
{
        struct al_data *data;
        data = container_of(handler, struct al_data, early_suspend);

        pr_info("%s\n", __func__);
        al3000a_resume(data->client);
}

static int al3000a_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    struct al_data *als = NULL;
    int ret = 0;

    if (al_init(client) < 0)
	return -EINVAL;

    als = kzalloc(sizeof(struct al_data), GFP_KERNEL);
    if (!als) {
        printk(KERN_ERR "%s: no memory\n", __func__);
        return -ENOMEM;
    }

    /* Declare input device */
    als->input_dev = input_allocate_device();
    if (!als->input_dev) {
        printk(KERN_ERR "%s: failed to allocate input device\n", __func__);
        kfree(als);
	return -ENOMEM;
    }

    /* Setup input device */
    set_bit(EV_ABS, als->input_dev->evbit);
    input_set_abs_params(als->input_dev, ABS_MISC, 0, 0xFF, 0, 0);

    als->client = client;
    i2c_set_clientdata(client, als);

    /* Set name */
    als->input_dev->name = "lightsensor-level";
    /* Register */
    ret = input_register_device(als->input_dev);
    if (ret) {
        printk(KERN_ERR "%s: input_register_device failed\n", __func__);
	goto fail_input_device;
    }
    
    /* Setup misc device */
    ret = misc_register(&light_misc_device);
    if (ret) {
        printk(KERN_ERR "%s: misc_register failed\n", __func__);
	goto fail_misc_device;
    }

    /* As default, doesn't report input event */
    atomic_set(&als->l_flag, 0);
   
    ls = als;

    INIT_WORK(&als->work,sample_workqueue);
    
    /* get the irq */
//COMPAL_START
    //ret = request_irq(als->client->irq, al3000a_irq,
    //                            IRQF_ONESHOT| IRQF_TRIGGER_LOW,
    //                           DRIVER_NAME, als);
      ret = request_irq(client->irq,al3000a_irq,IRQF_TRIGGER_FALLING,
                               DRIVER_NAME, als);
      //ls_enable(1);
      schedule_work(&als->work);
//COMPAL_END
    if (ret) {
            printk(KERN_ERR "%s: request_irq(%d) failed\n",
                    __func__, als->client->irq);
            goto fail_request_irq;
    }
    
//COMPAL_END
	light_kobj = kobject_create_and_add("light", NULL);
	if (!light_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(light_kobj, &attr_group);
	if (ret)
		kobject_put(light_kobj);
//COMPAL_END

	als->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        als->early_suspend.suspend = al3000a_early_suspend;
        als->early_suspend.resume = al3000a_late_resume;
        register_early_suspend(&als->early_suspend);

    printk(KERN_INFO "%s: Light Sensor Driver Initialized", __func__);
    return 0;

fail_request_irq:
    misc_deregister(&light_misc_device);
fail_misc_device:
    input_unregister_device(als->input_dev);
fail_input_device:
    input_free_device(als->input_dev);
    kfree(als);
    return ret;
}

static int al3000a_remove(struct i2c_client *client)
{
    struct al_data *data = i2c_get_clientdata(client);

    printk(KERN_INFO "AL3000A Light Sensor Unloaded\n");

    if (!data)
        return -EINVAL;
 //COMPAL_START
    cancel_work_sync(&data->work);
    free_irq(data->client->irq, data);
 //COMPAL_END
    misc_deregister(&light_misc_device);
    input_unregister_device(data->input_dev);
    input_free_device(data->input_dev);
    kfree(data);

    return 0;
}

/*
static int al3000a_suspend(struct i2c_client *client, pm_message_t state)
{
    struct al_data *data = i2c_get_clientdata(client);
    int ret;

    if (WARN_ON(!data))
        return -EINVAL;

    disable_irq(client->irq);

    //Power Up & Enable ALS
    ret = i2c_smbus_write_byte_data(client, 0x00, 0x0B);
    if (ret < 0) {
        printk(KERN_ERR "%s: suspend failed\n", __func__);
        return ret;
    }

    return 0;
}

static int al3000a_resume(struct i2c_client *client)
{
    struct al_data *data = i2c_get_clientdata(client);
    int ret = 0;

    if (WARN_ON(!data))
        return -EINVAL;
    
    ret = i2c_smbus_write_byte_data(client, 0x00, 0x00);
    if (ret < 0) {
        printk(KERN_ERR "%s: resume failed\n", __func__);
        return ret;
    }

    enable_irq(client->irq);

    return 0;
}

static void al3000a_early_suspend(struct early_suspend *handler)
{
	struct al_data *data;
	data = container_of(handler, struct al_data, early_suspend);

        pr_info("%s\n", __func__);
        al3000a_suspend(data->client);
}


static void al3000a_late_suspend(struct early_suspend *handler)
{
        struct al_data *data;
        data = container_of(handler, struct al_data, early_suspend);

        pr_info("%s\n", __func__);
        al3000a_resume(data->client);
}
*/

static const struct i2c_device_id al3000a_id[] = {
    { DRIVER_NAME, 0 },
    { },
};

static struct i2c_driver al3000a_driver = {
    .probe	= al3000a_probe,
    .remove	= al3000a_remove,
    .suspend    = al3000a_suspend,
    .resume     = al3000a_resume,
    .id_table   = al3000a_id,
    .driver	= {
	    .name = DRIVER_NAME,
    },
};

static int __devinit al3000a_init(void)
{
    int e;

    e = i2c_add_driver(&al3000a_driver);
    if (e != 0) {
            pr_err("%s: failed to register with I2C bus with "
                   "error: 0x%x\n", __func__, e);
    }
    return e;
}

static void __exit al3000a_exit(void)
{
//COMPAL_START
	kobject_put(light_kobj);	
//COMPAL_END
    	i2c_del_driver(&al3000a_driver);
}

module_init(al3000a_init);
module_exit(al3000a_exit);
MODULE_DESCRIPTION("AL3000A Light Sensor Driver");
