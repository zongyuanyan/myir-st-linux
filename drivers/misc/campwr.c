#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/phy.h> 

struct gpio_desc *campwr;

static int gpio_init_probe(struct platform_device *pdev)
{
   int ret = -1;
   campwr = devm_gpiod_get(&pdev->dev, "cam", GPIOD_OUT_HIGH);
   if (IS_ERR(campwr)) {
                ret = PTR_ERR(campwr);
               // printk("cannot reset %d\n", ret);
                return ret;
   }
   ssleep(1);
   gpiod_set_value_cansleep(campwr, 1);
   printk("Initializing CAM successfully\n");

   return(0);
}

static int gpio_exit_remove(struct platform_device *pdev)
{
   printk("GPIO phy exit\n");
   
   return(0);
}

/* this structure does the matching with the device tree */
/* if it does not match the compatible field of DT, nothing happens */
static struct of_device_id dummy_match[] = {
    {.compatible = "myir,cam_pwr"},
    {/* end node */}
};

static struct platform_driver dummy_driver = {
    .probe = gpio_init_probe,
    .remove = gpio_exit_remove,
    .driver = {
        .name = "cam_driver",
                .owner = THIS_MODULE,
                .of_match_table = dummy_match,
    }
};
 
module_platform_driver(dummy_driver);

MODULE_AUTHOR("licy");
MODULE_DESCRIPTION("Gpio cam init");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cam_driver");
