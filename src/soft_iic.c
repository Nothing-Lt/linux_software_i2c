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
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "sw_iic.h"
#include "lib_soft_iic.h"

MODULE_LICENSE("GPL");

#define DEVICE_NAME "soft_iic"
#define CLASS_NAME "softiiccla"

#define DFLT_SCL 66
#define DFLT_SDA 69

static int major_num;
static struct class* cl;
static struct device* dev;
static struct task_struct* ts = NULL;

uint8_t dev_addr; // device iic address
uint8_t reg_addr; // device iic register
size_t len;     // size of the data will be transfered
void* buf = NULL; // buffer

extern unsigned scl_pin;
extern unsigned sda_pin;

// Determine a lock for the spinlock,
// It is needed for prenventing the preemption when 
// generating the scl and sda signal.
spinlock_t wire_lock;

static int __init device_init(void);
static void __exit device_exit(void);
static int device_open(struct inode* node, struct file* file);
static int device_release(struct inode*, struct file* );
static loff_t device_lseek(struct file*, loff_t, int);
static ssize_t device_read(struct file*, char* ,size_t,loff_t*);
static ssize_t device_write(struct file*, const char*, size_t, loff_t*);
long device_ioctl (struct file *, unsigned int, unsigned long);

static struct file_operations file_ops = {
    .llseek  = device_lseek, 
    .read    = device_read,
    .write   = device_write,
    .unlocked_ioctl = device_ioctl,
    .open    = device_open,
    .release = device_release
};

static int hello_thread(void* data)
{
    while(!kthread_should_stop()){

        gpio_set_value(66,1);
        msleep(50);
        gpio_set_value(66,0);
        msleep(50);
    }

    return 0;
}

static int __init device_init(void)
{
    printk(KERN_ALERT "softiic: staring...\n");

    // Find a place in kernel char-device-array for keeping and allocate major-id for this driver.
    // major-id is the index for this driver in char-device-array.
    // After this step, we can find this device from /proc/devices
    major_num = register_chrdev(0,DEVICE_NAME,&file_ops);
    if(0 > major_num){
    printk(KERN_ALERT "failed alloc\n");
        goto finish_l;
    }

    // Create a class under /sys/class, so that later we can call device_create() to create a node under /dev/
    cl = class_create(THIS_MODULE,CLASS_NAME);
    if(IS_ERR(cl)){
        printk(KERN_ALERT "failed create class\n");
        goto unregister_chrdev_region_l;
    } 
    
    // device_create(...,"hello"), the hello here might means after this function, the name for this driver under /dev/ is hello,
    // so we will have /dev/hello
    dev = device_create(cl,NULL,MKDEV(major_num,0),NULL,DEVICE_NAME);
    if(IS_ERR(dev)){
        printk(KERN_ALERT "failed device create\n");
        goto class_destroy_l;
    }

    if(NULL == (buf = vmalloc(4096))){
        goto device_destroy_l;
    }

/*
    ts = kthread_run(hello_thread,NULL,"softiic_thread");
    if(IS_ERR(ts)){
        goto free_buf_l;
    }
*/
    spin_lock_init(&wire_lock);

    printk(KERN_ALERT "staring done.\n");
    return 0;

device_destroy_l:
    device_destroy(cl, MKDEV(major_num,0));
class_destroy_l:
    class_destroy(cl);
unregister_chrdev_region_l:
    unregister_chrdev(major_num, DEVICE_NAME);
finish_l:
    return -1;
}

static void __exit device_exit(void)
{
    printk(KERN_ALERT "hello: stopping...\n");
/*
    kthread_stop(ts);
    ts = NULL;
*/

    device_destroy(cl, MKDEV(major_num,0));
    class_unregister(cl);
    class_destroy(cl);
    unregister_chrdev(major_num, DEVICE_NAME);
    printk(KERN_ALERT "hello: stopping done.\n");
}

static int device_open(struct inode* inode, struct file* file)
{
    printk(KERN_ALERT "device openning\n");

    if((-ENOSYS == i2c_scl_request(scl_pin)) || \
       (-ENOSYS == i2c_sda_request(sda_pin))){
        return -ENOSYS;
    }

    return 0;
}

static int device_release(struct inode* node, struct file* file)
{
    printk(KERN_ALERT "device released\n");

    i2c_scl_free();
    i2c_sda_free();

    return 0;
}

long device_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case IOCTL_CMD_SCL_PIN_SET:
            return i2c_scl_pin_set(arg);
        break;
        case IOCTL_CMD_SDA_PIN_SET:
            return i2c_sda_pin_set(arg);
        break;
        case IOCTL_CMD_CLK_FRQ_SET:
            return i2c_clock_rate_set(arg);
        break;
        default:
            return EPERM;
    }

    return 0;
}


static loff_t device_lseek(struct file* file, loff_t offset, int orig)
{

    dev_addr = (uint8_t)((offset >> 16) & 0xFF);
    reg_addr = (uint8_t)(offset & 0xFF);

    printk(KERN_ALERT "%d %llx %d %d\n",sizeof(loff_t), offset, dev_addr, reg_addr);

    return 0;
}

/**
 * size => len
 */
static ssize_t device_read(struct file* file, char* str,size_t size,loff_t* offset)
{
    len = size;
    ((uint8_t*)buf)[0] = 0;

    // iic read to buf
    spin_lock_irq(&wire_lock);
    if(soft_i2c_read(dev_addr << 1,reg_addr,buf,len)){
        return 0;
    }
    spin_unlock_irq(&wire_lock);

    copy_to_user(str,buf,len);

    return 0;
}

/**
 *
 */
static ssize_t device_write(struct file* file, const char* str, size_t size, loff_t* offset)
{
    len = size;

    copy_from_user(buf,str,size);

    printk(KERN_ALERT "Going to write %d %d %s\n",dev_addr, reg_addr, buf);

    // iic write
    spin_lock_irq(&wire_lock);
    if(soft_i2c_write(dev_addr << 1,reg_addr,buf,len)){
        return 0;
    }
    spin_unlock_irq(&wire_lock);
    
    printk(KERN_ALERT "write down\n");

    return 0;
}


module_init(device_init);
module_exit(device_exit);
