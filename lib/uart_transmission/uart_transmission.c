#include "uart_transmission.h"

void init_uart_transmission(uint16_t baud_rate) {
  // Configure value for register containing baud rate.
  uint16_t reg_rate = SYS_CLK / 16 / baud_rate - 1;

  // reg_rate = 103;
  // USART_BAUD_RATE_16 = reg_rate;
  UBRR0L = (uint8_t)(reg_rate & 0xFF);
  UBRR0H = (uint8_t)(reg_rate >> 8);

  // Enable Transmission (Receiving is not needed).
  // Let other default values as they are.
  UCSR0B |= (1 << TXEN0);
  UCSR0C = 0b00000110;
}

void send_char(const char to_send) {
  // Wait for emptied out data register
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = to_send;
}

void send_string(const char *to_send) {
  // Call send_char for the whole string
  while (*to_send) {
    send_char(*to_send);
    to_send++;
  }
}

void send_unsigned_decimal(uint64_t to_send) {
  // Max. Number has 20 digits.
  // Additional "-" sign and end of string leads to 22 chars.
  char buffer[22 * sizeof(char)] = {};
  // Let the resulting string begin at max. bufferposition.
  char *decimal_string = (buffer + sizeof(buffer) - 1);

  // Make sure to end the string!
  *decimal_string = '\0';
  do {
    char digit = to_send % 10;
    to_send /= 10;
    *--decimal_string = digit + '0';
  } while (to_send > 0);

  send_string(decimal_string);
}

void send_signed_decimal(int64_t to_send) {
  // Only difference is the minus sign.
  // Send it as a char and invert value to positive.
  if (to_send < 0) {
    to_send = -to_send;
    send_char('-');
    send_unsigned_decimal(to_send);
  } else {
    send_unsigned_decimal(to_send);
  }
}

void send_float(float to_send, uint8_t precision) {
  // minus sign stuff
  if (to_send < 0.0) {
    to_send = -to_send;
    send_char('-');
  }

  // Extract the number before the decimal point and print it.
  uint64_t whole = (uint64_t)to_send;
  float frac = to_send - whole;

  // Extract the number after the decimal point.
  send_unsigned_decimal(whole);
  if (frac != 0) {
    send_char('.');
    while (precision > 0) {
      frac *= 10;
      char digit = (uint8_t)frac;
      frac -= digit;
      send_char(digit + '0');
      --precision;
    }
  }
}
