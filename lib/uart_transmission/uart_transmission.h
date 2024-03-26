#ifndef UART_TRANSMISSION_H
#define UART_TRANSMISSION_H
#include <avr/io.h>
#include <stdint.h>

#define SYS_CLK 16000000UL

void init_uart_transmission(uint16_t baud_rate);
void send_char(const char to_send);
void send_string(const char *to_send);
void send_unsigned_decimal(uint64_t to_send);
void send_signed_decimal(int64_t to_send);
void send_float(float to_send, uint8_t precision);

#endif // UART_TRANSMISSION_H
