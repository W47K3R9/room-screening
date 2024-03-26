#ifndef I2C_TRANSMISSION
#define I2C_TRANSMISSION

#include <stdint.h>
#define COND_START 0x08
#define COND_ADDR_WRITE_ACK 0x18
#define COND_ADDR_READ_ACK 0x40
#define COND_DATA_WRITE_ACK 0x28
#define COND_DATA_READ_ACK 0x50
#define COND_DATA_READ_NACK 0x58

// All functions return 0 for success.
uint8_t init_i2c(uint8_t speed, uint8_t prescale);
uint8_t master_transmit_read_reg(uint8_t address, uint8_t data);
uint8_t master_transmit_write_to_reg(uint8_t address, uint8_t reg,
                                     uint8_t value);
uint8_t master_receive_byte(uint8_t address, uint8_t *storage);
uint8_t master_receive_nbytes(uint8_t address, uint8_t *storage,
                              uint8_t num_bytes);
#endif // I2C_TRANSMISSION
