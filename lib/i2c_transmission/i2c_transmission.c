#include "i2c_transmission.h"
#include "avr/io.h"
#include <stdint.h>

uint8_t init_i2c(uint8_t speed, uint8_t prescale) {
  if (prescale > 3) {
    // Prescaler has to be 0, 1 or 2.
    return 1;
  }
  // Analog In 4 and 5 are SDA and SCL Out.
  // Pullup Resistor is needed.
  DDRC = 0b0;
  PORTC = (1 << DDC4) | (1 << DDC5);
  // Clear Prescaler Bits.
  TWSR &= ~(0b11);
  // Set speed bits to given number.
  TWBR = speed;
  return 0;
}

uint8_t master_transmit_read_reg(uint8_t address, uint8_t data) {
  // Prepare for Transmission and set start bit.
  TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
  // In case that the bus is busy,
  // wait until TWINT is set to 1.
  while (!(TWCR & (1 << TWINT)))
    ;
  // Check correct status.
  if ((TWSR & ~(0b11)) != COND_START) {
    // Error: Start failed!
    return 1;
  }

  // Determine Address along with write bit (0 for last bit).
  TWDR = (address << 1) & ~(0b1);
  // Transmit address with write command.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_ADDR_WRITE_ACK) {
    // Error: No acknowledgement for address!
    return 22;
  }

  // Now the byte of data can be sent.
  TWDR = data;
  // Transmit data.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_DATA_WRITE_ACK) {
    // Error: No acknowledgement for data!
    return 3;
  }

  // Stop Transfer!
  TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
  return 0;
}

uint8_t master_transmit_write_to_reg(uint8_t address, uint8_t reg,
                                     uint8_t value) {
  // Prepare for Transmission and set start bit.
  TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
  // In case that the bus is busy,
  // wait until TWINT is set to 1.
  while (!(TWCR & (1 << TWINT)))
    ;
  // Check correct status.
  if ((TWSR & ~(0b11)) != COND_START) {
    // Error: Start failed!
    return 1;
  }

  // Determine Address along with write bit (0 for last bit).
  TWDR = (address << 1) & ~(0b1);
  // Transmit address with write command.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_ADDR_WRITE_ACK) {
    // Error: No acknowledgement for address!
    return 2;
  }

  // Now the first byte of data can be sent.
  TWDR = reg;
  // Transmit data.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_DATA_WRITE_ACK) {
    // Error: No acknowledgement for data!
    return 3;
  }

  // Now the second byte of data can be sent.
  TWDR = value;
  // Transmit data.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_DATA_WRITE_ACK) {
    // Error: No acknowledgement for data!
    return 4;
  }

  // Stop Transfer!
  TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
  return 0;
}

uint8_t master_receive_byte(uint8_t address, uint8_t *storage) {
  // Prepare to receive and set Start Bit.
  TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
  // In case that the bus is busy,
  // wait until TWINT is set to 1.
  while (!(TWCR & (1 << TWINT)))
    ;
  // Check correct status.
  if ((TWSR & ~(0b11)) != COND_START) {
    // Error: Start failed!
    return 1;
  }
  // Determine Address along with read bit (1 for last bit).
  TWDR = (address << 1) | 0b1;
  // Transmit address with write command.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_ADDR_READ_ACK) {
    // Error: No acknowledgement for address!
    return 2;
  }

  // Enable the first and last data package and deactivate acknowledge.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_DATA_READ_NACK) {
    // Error: Master nack not received!
    return 4;
  }
  *storage = TWDR;

  // Stop reading!
  TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
  return 0;
}

uint8_t master_receive_nbytes(uint8_t address, uint8_t *storage,
                              uint8_t num_bytes) {
  if (num_bytes < 2) {
    master_receive_byte(address, storage);
    return 0;
  }
  // Prepare to receive and set Start Bit.
  TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
  // In case that the bus is busy,
  // wait until TWINT is set to 1.
  while (!(TWCR & (1 << TWINT)))
    ;

  // Check correct status.
  if ((TWSR & ~(0b11)) != COND_START) {
    // Error: Start failed!
    return 1;
  }

  // Determine Address along with read bit (1 for last bit).
  TWDR = (address << 1) | 0b1;
  // Transmit address with write command.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_ADDR_READ_ACK) {
    // Error: No acknowledgement for address!
    return 2;
  }

  // Now the reading process begins.
  for (int i = 0; i < num_bytes - 1; i++) {
    // Enable the first data package.
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)))
      ;
    if ((TWSR & ~(0b11)) != COND_DATA_READ_ACK) {
      // Error: Master ack not received!
      return 3;
    }
    *(storage + i) = TWDR;
  }
  // Enable the last data package and deactivate acknowledge.
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  if ((TWSR & ~(0b11)) != COND_DATA_READ_NACK) {
    // Error: Master nack not received!
    return 4;
  }
  *(storage + (num_bytes - 1)) = TWDR;

  // Stop reading!
  TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
  return 0;
}
