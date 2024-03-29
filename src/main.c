#include "bme280_measure.h"
#include "i2c_transmission.h"
#include "uart_transmission.h"
#include <util/delay.h>

int main() {
  TransmitStatus bme_transmit_status;
  SensorConstants bme_sensor_constants;
  // Max Speed for BME280 Sensor is 3.4 MHz
  // Arduino runs at 16 MHz
  // For I2C Clock of 400 kHz speed,
  // set value of 12 without prescaling.
  init_i2c(12, 0);
  bme_init(&bme_transmit_status);
  send_string("\r\nInitialization status:\r\n");
  send_string(bme_transmit_status.status_msg);
  send_string("\r\n");
  bme_load_comp_vals(&bme_sensor_constants, &bme_transmit_status);
  send_string("\r\nLoading Values status:\r\n");
  send_string(bme_transmit_status.status_msg);
  send_string("\r\n");
  float temperature = 0;
  float humidity = 0;
  while (1) {
    _delay_ms(5000);
    temperature =
        bme_get_temp_degrees(&bme_sensor_constants, 16, &bme_transmit_status);
    send_string("\r\n\r\nTemperature read status:\r\n");
    send_string(bme_transmit_status.status_msg);
    send_string("\r\nTemperature in degrees: ");
    send_float(temperature, 2);
    send_string(" °C\r\n");
    humidity =
        bme_get_hum_percent(&bme_sensor_constants, 16, &bme_transmit_status);
    // humidity = bme_get_hum_raw(16, &bme_transmit_status);
    // uint32_t hum_comp = compensate_hum(humidity, &bme_sensor_constants);
    send_string("\r\nHumidity read status:\r\n");
    send_string(bme_transmit_status.status_msg);
    send_string("\r\nHumidity in percent: ");
    send_float(humidity, 2);
    send_string(" %\r\n");
  }
  return 0;
}
