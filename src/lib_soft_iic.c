#include "lib_soft_iic.h"

#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/errno.h>

unsigned scl_pin = DFLT_SCL;
unsigned sda_pin = DFLT_SDA;
unsigned long t_delay = 5000; 
void (*_i2c_delay)(unsigned long secs) = ndelay;

#define SCL_HIGH() gpio_set_value(scl_pin,1)
#define SCL_LOW()  gpio_set_value(scl_pin,0)

#define SDA_OUT()    gpio_direction_output(sda_pin,1)
#define SDA_INPUT()  gpio_direction_input(sda_pin)
#define SDA_SET(val) gpio_set_value(sda_pin,val)
#define SDA_GET()    gpio_get_value(sda_pin)

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

static int _i2c_start(void)
{
    SCL_LOW();
    SDA_SET(1);
    //udelay(20);

    SCL_HIGH();
    SDA_SET(0);
    //udelay(10);
    _i2c_delay(t_delay);
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
        _i2c_delay(t_delay);
        SCL_LOW();
        _i2c_delay(t_delay);
    }

    // For wating for the ack signal
    SDA_SET(1);
    SCL_HIGH();
    _i2c_delay(t_delay);
    SCL_LOW();
    _i2c_delay(t_delay);

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
        _i2c_delay(t_delay);
        SCL_LOW();
        _i2c_delay(t_delay);
    }

    // For generating the ack signal
    SDA_SET(0);
    SCL_HIGH();
    _i2c_delay(t_delay);
    SCL_LOW();
    _i2c_delay(t_delay);

    return data;
}

int  i2c_scl_request(unsigned long scl_pin)
{
    if(-ENOSYS == gpio_request(scl_pin, NULL)){
        return -ENOSYS;
    }
    gpio_direction_output(scl_pin,0);

    return 0;
}

int i2c_sda_request(unsigned long sda_pin)
{
    if(-ENOSYS == gpio_request(sda_pin, NULL)){
        return -ENOSYS;
    }
    gpio_direction_output(sda_pin,1);

    return 0;
}

int i2c_clock_rate_set(unsigned long clk_rate)
{
    switch(clk_rate)
    {
        case I2C_CLK_FRQ_100KHZ:
            t_delay = 5000;
            _i2c_delay = ndelay;
        break;
        case I2C_CLK_FRQ_400KHZ:
            t_delay = 1250; // Later to change the delay function. use ndelay 2500
            _i2c_delay = ndelay;
        break;
        case I2C_CLK_FRQ_1MHZ:
            t_delay = 500;
            _i2c_delay = _lib_i2c_delay; // Linux only give the microsecond level of delay, this frequency beyond this, so I defined this function.
        break;
        case I2C_CLK_FRQ_3D2MHZ:
            t_delay = 166;
            _i2c_delay = _lib_i2c_delay;
        break;
        default:
            return -EPERM;
        break;
    }

    return 0;
}

int i2c_scl_pin_set(unsigned long new_scl_pin)
{
    if((scl_pin != new_scl_pin) && (0 != new_scl_pin)){
        gpio_free(scl_pin);

        scl_pin = new_scl_pin;
        return i2c_scl_request(scl_pin);
    }

    return -1;
}

int i2c_sda_pin_set(unsigned long new_sda_pin)
{
    if((sda_pin != new_sda_pin) && (0 != new_sda_pin)){
        gpio_free(sda_pin);

        sda_pin = new_sda_pin;
        return i2c_sda_request(sda_pin);
    }

    return -1;
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
