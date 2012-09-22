#include <linux/module.h>
#include <linux/fs.h>
#include <linux/dmi.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>


#define TRACE_DIAG 1
/*Joe Lee-0517 begin*/
/*Sku pin*/
#define GPIO_PR0 136	//sku pin
/*Joe Lee-0517 end*/

#if TRACE_DIAG
#define DIAG(x...) printk(KERN_INFO "[phj00_diag] " x)
#else
#define DIAG(x...) do {} while (0)
#endif

/*Joe Lee-0517 begin*/
/*Sku attribute*/

static struct kobject *diag_kobj;

static ssize_t sku_show(struct kobject*, struct kobj_attribute*, char*);
static ssize_t sku_store(struct kobject*, struct kobj_attribute*, char*);

static ssize_t wp_show(struct kobject*, struct kobj_attribute*, char*);
static ssize_t wp_store(struct kobject*, struct kobj_attribute*, char*);

#define DIAG_ATTR(_name) \
         static struct kobj_attribute _name##_attr = { \
         .attr = { .name = __stringify(_name), .mode = 0660}, \
         .show = _name##_show, \
         .store = _name##_store, \
         }

DIAG_ATTR(sku);
DIAG_ATTR(wp);

static struct attribute * attributes[] = {
	&sku_attr.attr,
	&wp_attr.attr,
        NULL,
};

static struct attribute_group attribute_group = {
        .attrs = attributes,
};

//sku start
static ssize_t sku_show(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf)
{

	int pin_value;

	tegra_gpio_enable(GPIO_PR0);
	gpio_request(GPIO_PR0,"PIN0");
	gpio_direction_input(GPIO_PR0);
	pin_value = gpio_get_value(GPIO_PR0);
	gpio_free(GPIO_PR0);
	tegra_gpio_disable(GPIO_PR0);

    return sprintf(buf, "%d\n", pin_value);


}

static ssize_t sku_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf)
{
	return 0;
}
//sku end
/*Joe Lee-0517 end*/
static ssize_t wp_show(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf)
{
    return sprintf(buf, "%d\n", gpio_get_value(57));
}

static ssize_t wp_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf)
{
	return 0;
}

extern void SysShutdown(void);
extern void SysRestart(void);

static void phj00_pm_power_off(void)
{
    DIAG("SysShutdown\n");
    SysShutdown();
}

static void phj00_pm_reset(void)
{
    DIAG("SysRestart\n");
    SysRestart();
}

static int __init phj00_diag_init(void)
{
/*Joe Lee-0517 begin*/
/*create attribute group*/
	int err;

    diag_kobj = kobject_create_and_add("diag", NULL);
    if (diag_kobj == NULL){
        DIAG("kobj create failed\n");
		return -ENODEV;
    }

    err = sysfs_create_group(diag_kobj, &attribute_group);
    if(err){
        DIAG("/fs_create failed, \n");
		return -ENODEV;
    }
/*Joe Lee-0517 end*/

    DIAG("PHJ00 diag Driver Initialized\n");

    return 0;
}

static void __exit phj00_diag_exit(void)
{
}

module_init(phj00_diag_init);
module_exit(phj00_diag_exit);
