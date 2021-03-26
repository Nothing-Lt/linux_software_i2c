#ifndef _LIB_SOFT_IIC_H_
#define _LIB_SOFT_IIC_H_

#include <linux/types.h>
#include <linux/gpio.h>

#define DFLT_SCL 66
#define DFLT_SDA 69

#define I2C_CLK_FRQ_100KHZ    0x1
#define I2C_CLK_FRQ_200KHZ    0x2
#define I2C_CLK_FRQ_400KHZ    0x4
#define I2C_CLK_FRQ_1MHZ      0x8
#define I2C_CLK_FRQ_3D2MHZ    0x10

int i2c_scl_request(unsigned long scl_pin);

int i2c_sda_request(unsigned long sda_pin);

void i2c_scl_free(void);

void i2c_sda_free(void);

int i2c_clock_rate_set(unsigned long clk_rate);

int i2c_scl_pin_set(unsigned long new_scl_pin);

int i2c_sda_pin_set(unsigned long new_sda_pin);

void i2c_reset(void);

// reading operation
int soft_i2c_read(uint8_t dev_addr, uint8_t reg_addr, void* buf, size_t len);


// writing operation
int soft_i2c_write(const uint8_t dev_addr, const uint8_t reg_addr, const void* buf, const size_t len);



#endif
