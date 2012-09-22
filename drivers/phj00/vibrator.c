#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include "../arch/arm/mach-tegra/gpio-names.h"
#include "../../drivers/staging/android/timed_output.h"
#include "../../drivers/staging/android/timed_gpio.h"


/*
enum Version
{
    DVT2_BEFORE = 0,
    DVT3,
    PVT_AFTER,
};

enum Version BoardVersion;

extern u16 BoardID(void);
extern unsigned long g_sku_id;
*/
static struct timed_gpio indigo_timed_gpios[] = {
        {
                .name = "vibrator",
                .gpio = TEGRA_GPIO_PV5,
                .max_timeout = 10000,    //10 seconds!
                .active_low = 0,
        },
};

static struct timed_gpio_platform_data indigo_timed_gpio_platform_data = {
        .num_gpios      = ARRAY_SIZE(indigo_timed_gpios),
        .gpios          = indigo_timed_gpios,
};

static struct platform_device indigo_timed_gpio_device = {
        .name   = "timed-gpio",
        .id     = 0,
        .dev    = {
                .platform_data  = &indigo_timed_gpio_platform_data,
        },
};


/*
void CheckVersion(void)
{
   u16 boardid;
   if(g_sku_id == 7)
     {
           BoardVersion = DVT2_BEFORE;
     }
   else
     {
           boardid = BoardID();

           if(boardid == 0xfefe)
             {
                  BoardVersion = DVT3;
                  printk( "Vibrator: EC firmware is not supported, now use DVT3 gpio setting\n");
                                  
             }
           else
             {
                  if(boardid == 2)
                     BoardVersion = DVT3;
                  else if(boardid > 2)
                      BoardVersion = PVT_AFTER;
             }
     }


}
*/



static int __init vibrator_init(void)
{
	struct platform_device *timed_gpio_device[1];
	printk("Vibrator initialize\n");
	//CheckVersion();
	//printk("Vibrator: BoardVersion is %d\n",BoardVersion);
/*
#ifdef CONFIG_I_LOVE_PBJ30
	BoardVersion = PVT_AFTER;
#endif

	if(BoardVersion == DVT2_BEFORE || BoardVersion == DVT3)
  	  {
		picasso_timed_gpios[0].gpio = TEGRA_GPIO_PW1;
        	timed_gpio_device[0] = &picasso_timed_gpio_device;
		platform_add_devices(timed_gpio_device, 1);
		tegra_gpio_enable(TEGRA_GPIO_PW1);
          }
        else if(BoardVersion == PVT_AFTER)
  	       {*/
			indigo_timed_gpios[0].gpio = TEGRA_GPIO_PV5;
        		timed_gpio_device[0] = &indigo_timed_gpio_device;
			platform_add_devices(timed_gpio_device, 1);
			tegra_gpio_enable(TEGRA_GPIO_PV5);
              // }

        return 0;
}


static void __exit vibrator_cleanup(void)
{

}




module_init(vibrator_init);
module_exit(vibrator_cleanup);

