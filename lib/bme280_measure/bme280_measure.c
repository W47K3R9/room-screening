#include "bme280_measure.h"
#include "i2c_transmission.h"
#include "uart_transmission.h"
#include "util/delay.h"
#include <stdint.h>
#include <string.h>

// Compensation Function for raw temperature readings.
// raw_temp: int32_t containing the raw temperature data of the sensor.
int32_t compensate_temp(uint32_t raw_temp, SensorConstants *scp) {
  int32_t t_fine, var1, var2, temperature;
  int32_t temperature_min = -4000;
  int32_t temperature_max = 8500;

  var1 = ((((raw_temp >> 3) - ((int32_t)scp->dig_T1 << 1))) *
          ((int32_t)scp->dig_T2)) >>
         11;
  var2 = (((((raw_temp >> 4) - ((int32_t)scp->dig_T1)) *
            ((raw_temp >> 4) - ((int32_t)scp->dig_T1))) >>
           12) *
          ((int32_t)scp->dig_T3)) >>
         14;
  t_fine = var1 + var2;
  temperature = (t_fine * 5 + 128) >> 8;
  if (temperature < temperature_min) {
    temperature = temperature_min;
  } else if (temperature > temperature_max) {
    temperature = temperature_max;
  }

  return temperature;
}

// Initialize the sensor, will apply a soft reset and deactivate filters.
// tsp: Pointer to transmission status codes.
void bme_init(TransmitStatus *tsp) {
  uint8_t check_status =
      master_transmit_read_reg(BME280_ADDRESS_GND, BME280_CHIP_ID_REG);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "ID_REG_ERR");
    return;
  }
  uint8_t returned_id = 0;
  check_status = master_receive_byte(BME280_ADDRESS_GND, &returned_id);
  if ((check_status != 0) || (returned_id != BME280_CHIP_ID)) {
    strcpy(tsp->status_msg, "ID_READ_ERR");
    return;
  }
  check_status = master_transmit_write_to_reg(
      BME280_ADDRESS_GND, BME280_RESET_REG, BME280_RESET_VAL);
  if ((check_status != 0) || (returned_id != BME280_CHIP_ID)) {
    strcpy(tsp->status_msg, "RESET_ERR");
    return;
  }
  check_status =
      master_transmit_write_to_reg(BME280_ADDRESS_GND, BME280_CONFIG_REG, 0x00);
  if ((check_status != 0) || (returned_id != BME280_CHIP_ID)) {
    strcpy(tsp->status_msg, "CONFIG_ERR");
    return;
  }
  tsp->chip_id = returned_id;
  strcpy(tsp->status_msg, "INIT_SUCC");
}

// Function to load compensation values stored within the sensor.
// scp: Pointer to buffer in which the constants should be stored.
// tsp: Pointer to transmission status codes.
void bme_load_comp_vals(SensorConstants *scp, TransmitStatus *tsp) {
  uint8_t comp_buffer[BME280_COMPENSATE_REG_1_LEN];
  uint8_t check_status =
      master_transmit_read_reg(BME280_ADDRESS_GND, BME280_COMPENSATE_REG_1);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "COMP1_REG_ERR");
    return;
  }
  check_status = master_receive_nbytes(BME280_ADDRESS_GND, comp_buffer,
                                       sizeof(comp_buffer));
  if (check_status != 0) {
    strcpy(tsp->status_msg, "COMP1_LD_ERR");
    return;
  }
  // read in all temperature compensation values
  scp->dig_T1 = comp_buffer[0] | ((uint16_t)comp_buffer[1] << 8);
  scp->dig_T2 = comp_buffer[2] | ((uint16_t)comp_buffer[3] << 8);
  scp->dig_T3 = comp_buffer[4] | ((uint16_t)comp_buffer[5] << 8);

  // read in all pressure compensation values
  scp->dig_P1 = comp_buffer[6] | ((uint16_t)comp_buffer[7] << 8);
  scp->dig_P2 = comp_buffer[8] | ((uint16_t)comp_buffer[9] << 8);
  scp->dig_P3 = comp_buffer[10] | ((uint16_t)comp_buffer[11] << 8);
  scp->dig_P4 = comp_buffer[12] | ((uint16_t)comp_buffer[13] << 8);
  scp->dig_P5 = comp_buffer[14] | ((uint16_t)comp_buffer[15] << 8);
  scp->dig_P6 = comp_buffer[16] | ((uint16_t)comp_buffer[17] << 8);
  scp->dig_P7 = comp_buffer[18] | ((uint16_t)comp_buffer[19] << 8);
  scp->dig_P8 = comp_buffer[20] | ((uint16_t)comp_buffer[21] << 8);
  scp->dig_P9 = comp_buffer[22] | ((uint16_t)comp_buffer[23] << 8);
  // only the first humidity constant in this buffer
  scp->dig_H1 = comp_buffer[24];

  check_status =
      master_transmit_read_reg(BME280_ADDRESS_GND, BME280_COMPENSATE_REG_2);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "COMP2_REG_ERR");
    return;
  }
  check_status = master_receive_nbytes(BME280_ADDRESS_GND, comp_buffer,
                                       BME280_COMPENSATE_REG_2_LEN);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "COMP2_LD_ERR");
    return;
  }
  scp->dig_H2 = comp_buffer[0] | ((uint16_t)comp_buffer[1] << 8);
  scp->dig_H3 = comp_buffer[2];
  scp->dig_H4 = ((uint16_t)comp_buffer[3] << 4) | (comp_buffer[4] & 0x0F);
  scp->dig_H5 = (comp_buffer[4] & 0xF0) | ((uint16_t)comp_buffer[5] << 4);
  scp->dig_H6 = comp_buffer[6];

  strcpy(tsp->status_msg, "COMP_LD_SUCC");
}

// Get the raw temperature value.
// oversampling: Choose temperature oversempling.
//  Valid values are 1, 2, 4, 8 and 16 other values result in oversampling = 1.
// tsp: Pointer to transmission status codes.
uint32_t bme_get_temp_raw(uint8_t oversampling, TransmitStatus *tsp) {
  // The overgiven parameter has to be checked to determine
  // correct oversampling value.
  uint8_t choose_os;
  switch (oversampling) {
  case 1:
    choose_os = BME280_OST_1;
    break;
  case 2:
    choose_os = BME280_OST_2;
    break;
  case 4:
    choose_os = BME280_OST_4;
    break;
  case 8:
    choose_os = BME280_OST_8;
    break;
  case 16:
    choose_os = BME280_OST_16;
    break;
  default:
    choose_os = BME280_OST_1;
  }
  // Write register value to start measurement and set oversampling.
  uint8_t ost_start_meas = choose_os | BME280_FORCE_MEAS;
  uint8_t check_status = master_transmit_write_to_reg(
      BME280_ADDRESS_GND, BME280_CONTROL_MEAS_REG, ost_start_meas);
  // Worst case time taken for the measurement.
  if (check_status != 0) {
    strcpy(tsp->status_msg, "START_MEAS_ERR");
    return check_status;
  }
  _delay_ms(50);

  // Prepare to read measured data from register.
  check_status = master_transmit_read_reg(BME280_ADDRESS_GND, BME280_TEMP_REG);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "TEMP_REG_ERR");
    return check_status;
  }

  // Create buffer to store the temperature value.
  uint8_t temp_data_buf[BME280_TEMP_LEN] = {};
  uint32_t raw_temp = 0;
  check_status =
      master_receive_nbytes(BME280_ADDRESS_GND, temp_data_buf, BME280_TEMP_LEN);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "TEMP_LD_ERR");
    return check_status;
  }
  // Get the correct value (1. xlsb 7-4; 2. lsb; 3. msb)
  send_string("\r\n");
  send_signed_decimal((int32_t)temp_data_buf[0] << 12);
  send_string("\r\n");
  send_signed_decimal((int32_t)temp_data_buf[1] << 4);
  send_string("\r\n");
  send_signed_decimal((int32_t)temp_data_buf[2] >> 4);
  send_string("\r\n");
  raw_temp = ((int32_t)temp_data_buf[0] << 12) |
             ((int32_t)temp_data_buf[1] << 4) | (temp_data_buf[2] >> 4);
  strcpy(tsp->status_msg, "TEMP_LD_SUCC");

  return raw_temp;
}

// Returns the raw measured temperature in degrees.
// scp: Pointer to sensor constants.
// oversampling: Choose temperature oversempling.
//  Valid values are 1, 2, 4, 8 and 16 other values result in oversampling = 1.
// tsp: Pointer to transmission status codes.
int32_t bme_get_temp_degrees(SensorConstants *scp, uint8_t oversampling,
                             TransmitStatus *tsp) {
  int32_t raw_temp = bme_get_temp_raw(oversampling, tsp);
  int32_t temperature = compensate_temp(raw_temp, scp);
  return temperature;
}
