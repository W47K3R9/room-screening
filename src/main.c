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
  float temperature = 0;
  float humidity = 0;
  while (1) {
    _delay_ms(5000);

    temperature =
        bme_get_temp_degrees(&bme280_comp_vals, 16, &bme_transmit_status);
    send_string("\r\n\r\nTemperature read status:\r\n");
    send_string(bme_transmit_status.status_msg);
    send_string("\r\nTemperature in degrees: ");
    send_float(temperature, 2);
    send_string(" °C\r\n");
    humidity = bme_get_hum_percent(&bme280_comp_vals, 16, &bme_transmit_status);
    send_string("\r\nHumidity read status:\r\n");
    send_string(bme_transmit_status.status_msg);
    send_string("\r\nHumidity in percent: ");
    send_float(humidity, 2);
    send_string(" %\r\n");
  }
  return 0;
}
