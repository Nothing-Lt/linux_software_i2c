#include "lib_soft_iic.h"

#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/errno.h>
#include <linux/spinlock.h>

#define _I2C_100KHZ 1e9/400000/4

extern spinlock_t wire_lock;

unsigned long t_delay = _I2C_100KHZ;
void (*_i2c_delay)(unsigned long secs) = ndelay;

unsigned _scl_pin = DFLT_SCL;
unsigned _sda_pin = DFLT_SDA;

#define SCL_HIGH() gpio_direction_input(_scl_pin); \
                   _i2c_delay(t_delay)
#define SCL_LOW()  gpio_direction_output(_scl_pin,0); \
                   _i2c_delay(t_delay)

#define SDA_SET(val) 1 == (val) ? gpio_direction_input(_sda_pin) : gpio_direction_output(_sda_pin,0); \
                     _i2c_delay(t_delay)
#define SDA_GET()    gpio_get_value(_sda_pin)

/**
 * For the implementation of this delay,
 * parameter is in nanoseconds
 * my devices is 1.0GHz, so this definition is ok for me.
 */
static void _lib_i2c_delay(unsigned long secs)
{
    unsigned long i;
    
    for(i = 0 ; i < secs ; i++){

    }
}

static void _i2c_release_wait(unsigned pin)
{
    int cnt = 0;

    gpio_direction_input(pin);
    _i2c_delay(t_delay);

    while(!gpio_get_value(pin)){
        cnt++;
        if(100 <= cnt){
            return;
        }
        msleep(100);
        gpio_direction_input(pin);
    }
    _i2c_delay(t_delay);
}

static int _i2c_start(void)
{
    SDA_SET(1);

    if(!SDA_GET()){
        i2c_reset();
    }
    _i2c_release_wait(_scl_pin);

    SDA_SET(0);

    SCL_LOW();

    return 0;
}

static int _i2c_stop(void)
{
    _i2c_release_wait(_scl_pin);
    
    SDA_SET(1);
    if(!SDA_GET()){
        i2c_reset();
    }

    return 0;
}

static void _i2c_bit_write(uint8_t data)
{
    SDA_SET(data);
    
    _i2c_release_wait(_scl_pin);
    
    SCL_LOW();
    SDA_SET(0);
}

static int _i2c_bit_read(void)
{
    int res;

    SDA_SET(1);
    
    _i2c_release_wait(_scl_pin);
    
    res = SDA_GET();
    
    SCL_LOW();
    SDA_SET(0);

    return res;
}

static int _i2c_byte_write(uint8_t data)
{
    int i;
    int res = 1;

    spin_lock_irq(&wire_lock);

    for(i = 7 ; i >= 0 ; i--){
        _i2c_bit_write((data >> i) & 1);
    }
    res = _i2c_bit_read();

    spin_unlock_irq(&wire_lock);

    // For wating for the ack signal
    return res; 
}

static uint8_t _i2c_byte_read(void)
{    
    int i;
    uint8_t data = 0;

    spin_lock_irq(&wire_lock);

    for(i = 7 ; i >= 0 ; i--){
        data = (data << 1) | _i2c_bit_read();
    }

    // For generating the ack signal
    _i2c_bit_write(0);

    spin_unlock_irq(&wire_lock);

    return data;
}

int  i2c_scl_request(unsigned long scl_pin)
{
    if(-ENOSYS == gpio_request(scl_pin, NULL)){
        return -ENOSYS;
    }
    gpio_direction_input(scl_pin);

    _scl_pin = scl_pin;

    return 0;
}

void i2c_scl_free()
{
    gpio_direction_input(_scl_pin);
    gpio_free(_scl_pin);
}

int i2c_sda_request(unsigned long sda_pin)
{
    if(-ENOSYS == gpio_request(sda_pin, NULL)){
        return -ENOSYS;
    }
    gpio_direction_input(sda_pin);

    _sda_pin = sda_pin;

    return 0;
}

void i2c_sda_free()
{
    gpio_direction_input(_sda_pin);
    gpio_free(_sda_pin);
}

int i2c_clock_rate_set(unsigned long clk_rate)
{
    switch(clk_rate)
    {
        case I2C_CLK_FRQ_100KHZ:
            t_delay = _I2C_100KHZ;
            _i2c_delay = ndelay;
        break;
        case I2C_CLK_FRQ_200KHZ:
            t_delay = 500;
            _i2c_delay = _lib_i2c_delay;
        break;
        case I2C_CLK_FRQ_400KHZ:
            t_delay = 250; // Later to change the delay function. use ndelay 2500
            _i2c_delay = _lib_i2c_delay;
        break;
        case I2C_CLK_FRQ_1MHZ:
        break;
        case I2C_CLK_FRQ_3D2MHZ:
        break;
        default:
            return -EPERM;
        break;
    }

    return 0;
}

void i2c_reset(void)
{
    int i;

    SDA_SET(1);

    do{
        for(i = 0 ; i < 10 ; i++){
            SCL_LOW();
            SCL_HIGH();
        }

        msleep(10);
    }while(!SDA_GET());

    SCL_LOW();
    SDA_SET(0);

    _i2c_stop();
}

int soft_i2c_read(uint8_t dev_addr, uint8_t reg_addr, void* buf, size_t len)
{
    int i;
    uint8_t* ptr = NULL;

    if(NULL == buf){
        return -EPERM;
    }

    ptr = (uint8_t*)buf;

    _i2c_start();
    if(_i2c_byte_write(dev_addr & (~0x1))){
        i2c_reset();
        printk(KERN_INFO "addr %x\n",dev_addr);
        return 1;
    } 
    if(_i2c_byte_write(reg_addr)){
        i2c_reset();
        printk(KERN_INFO "reg %x\n",reg_addr);
        return 1;
    }
    
    _i2c_start();             // Generate restart signal

    if(_i2c_byte_write(dev_addr | 0x1)){
        i2c_reset();
        printk(KERN_INFO "read dev_addr | 1\n");
        return -1;
    } 

    // For reading the data from slave.
    for(i = 0 ; i < len ; i++){
        ptr[i] = _i2c_byte_read();
    }
    _i2c_byte_read();
    
    _i2c_stop();

    return 0;
}

int soft_i2c_write(const uint8_t dev_addr, const uint8_t reg_addr, const void* buf, const size_t len)
{
    int i;
    uint8_t* ptr;

    if(NULL == buf){
        return -EPERM;
    }
    ptr = (uint8_t*)buf;
    
    _i2c_start();
    if(_i2c_byte_write(dev_addr & (~0x1))){
        i2c_reset();
        printk(KERN_INFO "addr %x\n",dev_addr);
        return 1;
    } 
    if(_i2c_byte_write(reg_addr)){
        i2c_reset();
        printk(KERN_INFO "reg %x\n",reg_addr);
        return 1;
    }

    for(i = 0 ; i < len ; i++){
        if(_i2c_byte_write(ptr[i])){
            i2c_reset();
            printk(KERN_INFO "no ack from slave %x\n",ptr[i]);
        }
    }

    _i2c_stop();

    return 0;
}
