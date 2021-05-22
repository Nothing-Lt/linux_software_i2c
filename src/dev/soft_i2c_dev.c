#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/platform_device.h>

static struct resource pin_resources[] = {
    
    // For SCL pin
    {
        .start = 66,
        .end = 66,
        .flags= IORESOURCE_IRQ,
        .name = "scl"
    },

    // For SDA pin
    {
        .start = 69,
        .end = 69,
        .flags= IORESOURCE_IRQ,
        .name = "sda"
    }
};

struct platform_device soft_i2c_bus_dev = {
    .name = "soft_i2c_bus",
    .num_resources = ARRAY_SIZE(pin_resources),
    .resource = pin_resources
};

static int __init device_init(void)
{
    int err = 0;

    printk(KERN_INFO "[sw_iic] softiic_dev: staring...\n");

    err = platform_device_register(&soft_i2c_bus_dev);

    return err;
}

static void __exit device_exit(void)
{
    platform_device_unregister(&soft_i2c_bus_dev);
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");
