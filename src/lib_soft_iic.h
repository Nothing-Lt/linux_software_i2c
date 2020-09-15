#ifndef LIB_SOFT_IIC_H
#define LIB_SOFT_IIC_H

#include <linux/types.h>

// reading operation
int soft_i2c_read(uint8_t dev_addr, uint8_t reg_addr, void* buf, size_t len);


// writing operation
int soft_i2c_write(const uint8_t dev_addr, const uint8_t reg_addr, const void* buf, const size_t len);



#endif