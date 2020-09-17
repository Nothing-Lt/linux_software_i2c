#include "lib_soft_iic.h"

#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/errno.h>

extern unsigned scl_pin;
extern unsigned sda_pin;

#define SCL_HIGH() gpio_set_value(scl_pin,1)
#define SCL_LOW()  gpio_set_value(scl_pin,0)

#define SDA_OUT()    gpio_direction_output(sda_pin,1)
#define SDA_INPUT()  gpio_direction_input(sda_pin)
#define SDA_SET(val) gpio_set_value(sda_pin,val)
#define SDA_GET()    gpio_get_value(sda_pin)

static int _i2c_start(void)
{
    SCL_LOW();
    SDA_SET(1);
    //udelay(20);

    SCL_HIGH();
    SDA_SET(0);
    udelay(10);

    return 0;
}

static int _i2c_stop(void)
{
    SCL_LOW();
    SDA_SET(1);

    return 0;
}

static int _i2c_byte_write(uint8_t data)
{
    int i;

    SCL_LOW();
    for(i = 7 ; i >= 0 ; i--){
        SDA_SET((data >> i) & 1);
        SCL_HIGH();
        udelay(10);
        SCL_LOW();
        udelay(10);
    }

    // For wating for the ack signal
    SDA_SET(1);
    SCL_HIGH();
    udelay(10);
    SCL_LOW();
    udelay(10);

    return 0;
}

static uint8_t _i2c_byte_read(void)
{    
    int i;
    uint8_t data = 0;

    SCL_LOW();
    for(i = 7 ; i >= 0 ; i--){
        SCL_HIGH();
        data = (data << 1) | SDA_GET();
        udelay(10);
        SCL_LOW();
        udelay(10);
    }

    // For generating the ack signal
    SDA_SET(0);
    SCL_HIGH();
    udelay(10);
    SCL_LOW();
    udelay(10);

    return data;
}

int soft_i2c_read(uint8_t dev_addr, uint8_t reg_addr, void* buf, size_t len)
{
    int i;
    uint8_t* ptr = NULL;

    if(NULL == buf){
        return EPERM;
    }

    ptr = (uint8_t*)buf;

    // read
    SDA_OUT();    
    _i2c_start();
    _i2c_byte_write(dev_addr & (~(0x1))); // For sending the device address, so use write bit here.
    _i2c_byte_write(reg_addr);
    _i2c_start();             // Generate restart signal
    _i2c_byte_write(dev_addr | 0x1); // For reading the data from slave.

    SDA_INPUT();
    for(i = 0 ; i < len ; i++){
        ptr[i] = _i2c_byte_read();
    }
    SDA_OUT();

    _i2c_stop();


    return 0;
}

int soft_i2c_write(const uint8_t dev_addr, const uint8_t reg_addr, const void* buf, const size_t len)
{
    int i;
    uint8_t* ptr;

    if(NULL == buf){
        return EPERM;
    }
    ptr = (uint8_t*)buf;
    
    SDA_OUT();
    _i2c_start();
    _i2c_byte_write(dev_addr & (~(0x1))); // 
    _i2c_byte_write(reg_addr);

    for(i = 0 ; i < len ; i++){
        _i2c_byte_write(ptr[i]);
    }

    _i2c_stop();

    return 0;
}