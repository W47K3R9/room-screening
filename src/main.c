#include "bme280_measure.h"
#include "i2c_transmission.h"
#include "uart_transmission.h"
#include <stdint.h>
#include <util/delay.h>

int main() {
  TransmitStatus bme_transmit_status;
  // Max Speed for BME280 Sensor is 3.4 MHz
  // Arduino runs at 16 MHz
  // For I2C Clock of 400 kHz speed,
  // set value of 12 without prescaling.
  init_i2c(12, 0);
  init_uart_transmission((uint16_t)9600);
  bme_init(&bme_transmit_status);
  send_string("\r\nInitialization status:\r\n");
  send_string(bme_transmit_status.status_msg);
  send_string("\r\n");
  SensorConstants bme280_comp_vals;
  bme_load_comp_vals(&bme280_comp_vals, &bme_transmit_status);
  send_string("\r\nLoading Values status:\r\n");
  send_string(bme_transmit_status.status_msg);
  send_string("\r\n");
  uint32_t raw_temperature = 0;
  while (1) {
    _delay_ms(1000);
    raw_temperature = bme_get_temp_raw(16, &bme_transmit_status);
    send_string("\r\nTemperature read status:\r\n");
    send_string(bme_transmit_status.status_msg);
    send_string("\r\nRaw Temperature: ");
    send_signed_decimal(raw_temperature);
    send_string("\r\nIn degrees: ");
    int32_t real_temp = compensate_temp(raw_temperature, &bme280_comp_vals);
    float formatted_temp = (float) real_temp * 0.01;
    send_float(formatted_temp, 2);
    send_string("\r\n");
  }
  return 0;
}
