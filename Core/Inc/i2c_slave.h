#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void i2c_init_slave();
void i2c_set_slave_addr(uint8_t addr);
uint8_t i2c_get_slave_addr();

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */