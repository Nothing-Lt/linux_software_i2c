#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include "sw_i2c.h"
#include "lib_soft_i2c.h"

MODULE_LICENSE("GPL");

#define DEVICE_NAME "soft_i2c"
#define CLASS_NAME "softi2ccla"

struct mod_ctrl_s
{
    unsigned occupied_flag;
    pid_t    pid;
};

//#define DEBUG_MOD
static int major_num;
static struct class* cl;
static struct device* dev;

struct mod_ctrl_s mod_ctrl;
uint8_t dev_addr; // device i2c address
uint8_t reg_addr; // device i2c register
size_t len;     // size of the data will be transfered
void* buf = NULL; // buffer

extern unsigned _scl_pin;
extern unsigned _sda_pin;

// Determine a lock for the spinlock,
// It is needed for prenventing the preemption when 
// generating the scl and sda signal.
spinlock_t wire_lock;

static int __init soft_i2c_bus_driver_init(void);
static void __exit soft_i2c_bus_driver_exit(void);
static int soft_i2c_bus_driver_probe(struct platform_device* dev);
static int soft_i2c_bus_driver_remove(struct platform_device* dev);

static int soft_i2c_bus_open(struct inode* node, struct file* file);
static int soft_i2c_bus_release(struct inode*, struct file* );
static loff_t soft_i2c_bus_lseek(struct file*, loff_t, int);
static ssize_t soft_i2c_bus_read(struct file*, char* ,size_t,loff_t*);
static ssize_t soft_i2c_bus_write(struct file*, const char*, size_t, loff_t*);
long soft_i2c_bus_ioctl (struct file *, unsigned int, unsigned long);

static struct file_operations file_ops = {
    .llseek  = soft_i2c_bus_lseek,
    .read    = soft_i2c_bus_read,
    .write   = soft_i2c_bus_write,
    .unlocked_ioctl = soft_i2c_bus_ioctl,
    .open    = soft_i2c_bus_open,
    .release = soft_i2c_bus_release
};

struct platform_driver soft_i2c_bus_drv = {
    .probe = soft_i2c_bus_driver_probe,
    .remove = soft_i2c_bus_driver_remove,
    .driver = {
        .name = "soft_i2c_bus"
    }
};

static int soft_i2c_bus_driver_probe(struct platform_device* pdev)
{
    struct resource* res = NULL;

    //Gettng the scl pin
    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "scl");
    _scl_pin = (unsigned)(res->start);

    // Getting the sda pin
    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "sda");
    _sda_pin = (unsigned)(res->start);

    // Initialize the soft i2c bus
    if(0 != i2c_scl_request(_scl_pin)){
        goto i2c_scl_failed_l;
    }

    if(0 != i2c_sda_request(_sda_pin)){
        goto i2c_sda_failed_l;
    }

    i2c_reset();

    mod_ctrl.occupied_flag = 0;
    mod_ctrl.pid = 0;

    spin_lock_init(&wire_lock);

    // device_create(...,"hello"), the hello here might means after this function, the name for this driver under /dev/ is hello,
    // so we will have /dev/hello
    dev = device_create(cl,NULL,MKDEV(major_num,0),NULL,DEVICE_NAME);
    if(IS_ERR(dev)){
        printk(KERN_ALERT "[sw_i2c] failed device create\n");
        goto device_create_failed_l;
    }

    printk(KERN_INFO "[sw_i2c_drv] probe starting done\n");

    return 0;

device_create_failed_l:
    i2c_sda_free();
i2c_sda_failed_l:
    i2c_scl_free();
i2c_scl_failed_l:
    return -1;
}

static int soft_i2c_bus_driver_remove(struct platform_device* dev)
{
    i2c_sda_free();
    i2c_scl_free();

    device_destroy(cl, MKDEV(major_num,0));

    return 0;
}

static int __init soft_i2c_bus_driver_init(void)
{
    printk(KERN_INFO "[sw_i2c] softi2c_drv: starting...\n");

    // Find a place in kernel char-device-array for keeping and allocate major-id for this driver.
    // major-id is the index for this driver in char-device-array.
    // After this step, we can find this device from /proc/devices
    major_num = register_chrdev(0,DEVICE_NAME,&file_ops);
    if(0 > major_num){
    printk(KERN_ALERT "[sw_i2c] failed alloc\n");
        goto finish_l;
    }

    // Create a class under /sys/class, so that later we can call device_create() to create a node under /dev/
    cl = class_create(THIS_MODULE,CLASS_NAME);
    if(IS_ERR(cl)){
        printk(KERN_ALERT "[sw_i2c] failed create class\n");
        goto unregister_chrdev_region_l;
    } 
    
    if(NULL == (buf = vmalloc(4096))){
        goto class_destroy_l;
    }

    if(platform_driver_register(&soft_i2c_bus_drv)){
        goto platform_driver_register_failed_l;
    }

    printk(KERN_INFO "[sw_i2c_drv] staring done.\n");
    
    return 0;

platform_driver_register_failed_l:
    vfree(buf);
class_destroy_l:
    class_destroy(cl);
unregister_chrdev_region_l:
    unregister_chrdev(major_num, DEVICE_NAME);
finish_l:
    return -1;
}

static void __exit soft_i2c_bus_driver_exit(void)
{
    printk(KERN_INFO "[sw_i2c]: stopping...\n");

    platform_driver_unregister(&soft_i2c_bus_drv);
    vfree(buf);
    class_unregister(cl);
    class_destroy(cl);
    unregister_chrdev(major_num, DEVICE_NAME);
    printk(KERN_INFO "[sw_i2c]: stopping done.\n");
}

static int soft_i2c_bus_open(struct inode* inode, struct file* file)
{
    printk(KERN_INFO " %d device openning, scl %u sda %u\n", current->pid, _scl_pin, _sda_pin);

    return 0;
}

static int soft_i2c_bus_release(struct inode* node, struct file* file)
{
    printk(KERN_INFO "%d device released\n", current->pid);

    if(mod_ctrl.occupied_flag && (mod_ctrl.pid == current->pid)){
        mod_ctrl.occupied_flag = 0;
    }

    return 0;
}

long soft_i2c_bus_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = -ENOSYS;

    switch(cmd)
    {
        case IOCTL_CMD_TAKE:
            spin_lock_irq(&wire_lock);
            if(!mod_ctrl.occupied_flag){
                mod_ctrl.occupied_flag = 1;
                mod_ctrl.pid = current->pid;
                ret = 0;
            }
            spin_unlock_irq(&wire_lock);
        break;

        case IOCTL_CMD_RELEASE:
            spin_lock_irq(&wire_lock);
            if(mod_ctrl.occupied_flag && (mod_ctrl.pid == current->pid)){
                mod_ctrl.occupied_flag = 0;
                ret = 0;
            }
            spin_unlock_irq(&wire_lock);
        break;

        // case IOCTL_CMD_SCL_PIN_SET:
        //     i2c_scl_pin_set(arg);
        //     i2c_reset();
        // break;
        // case IOCTL_CMD_SDA_PIN_SET:
        //     i2c_sda_pin_set(arg);
        //     i2c_reset();
        // break;
        
        case IOCTL_CMD_CLK_FRQ_SET:
            return i2c_clock_rate_set(arg);
        break;
        default:
            return EPERM;
    }

    return 0;
}


static loff_t soft_i2c_bus_lseek(struct file* file, loff_t offset, int orig)
{
    if(mod_ctrl.occupied_flag && (mod_ctrl.pid != current->pid)){
        return -1;
    }

    dev_addr = (uint8_t)((offset >> 16) & 0xFF);
    reg_addr = (uint8_t)(offset & 0xFF);

    printk(KERN_INFO "[sw_i2c] %d lseek %d %llx %d %d\n",current->pid, sizeof(loff_t), offset, dev_addr, reg_addr);

    return 0;
}

/**
 * size => len
 */
static ssize_t soft_i2c_bus_read(struct file* file, char* str,size_t size,loff_t* offset)
{ 
    len = size;
    ((uint8_t*)buf)[0] = 0;
    
    if(mod_ctrl.occupied_flag && (mod_ctrl.pid != current->pid)){
        return -1;
    }

    if(soft_i2c_read(dev_addr << 1,reg_addr,buf,len)){
        printk(KERN_INFO "[sw_i2c] no ack!!\n");
        return -1;
    }

#ifdef DEBUG_MOD
    int i;
    printk(KERN_INFO "[sw_i2c] read : ");
    for (i = 0; i < len; i++)
    {
        printk(KERN_INFO "%d,",((char*)buf)[i]);
    }
    printk(KERN_INFO "\n");
#endif

    copy_to_user(str,buf,len);
    
    return len;
}

/**
 *
 */
static ssize_t soft_i2c_bus_write(struct file* file, const char* str, size_t size, loff_t* offset)
{    
    len = size;
    
    if((!mod_ctrl.occupied_flag) || (mod_ctrl.pid != current->pid)){
        return -1;
    }
    
    copy_from_user(buf,str,size);

    printk(KERN_INFO "[sw_i2c] %d Going to write addr %d reg %d\n",current->pid,dev_addr, reg_addr);

#ifdef DEBUG_MOD
    int i;
    for (i = 0  ; i < len ; i++)
    {
        printk(KERN_INFO "%d,",((char*)buf)[i]);
    }
    printk(KERN_INFO "\n");
#endif

    if(soft_i2c_write(dev_addr << 1,reg_addr,buf,len)){
        printk(KERN_INFO "no ack from slave\n");
        return -1;
    }
    
    printk(KERN_INFO "write done\n");
    
    return size;
}

module_init(soft_i2c_bus_driver_init);
module_exit(soft_i2c_bus_driver_exit);
