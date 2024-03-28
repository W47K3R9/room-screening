#include "bme280_measure.h"
#include "i2c_transmission.h"
// #include "uart_transmission.h"
#include "util/delay.h"
#include <stdint.h>
#include <string.h>

// Compensation Function for raw temperature readings.
// raw_temp: int32_t containing the raw temperature data of the sensor.
int32_t compensate_temp(uint32_t raw_temp, SensorConstants *scp) {
  int32_t var1, var2, temperature;
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
  scp->t_fine = var1 + var2;
  temperature = (scp->t_fine * 5 + 128) >> 8;
  if (temperature < temperature_min) {
    temperature = temperature_min;
  } else if (temperature > temperature_max) {
    temperature = temperature_max;
  }
  return temperature;
}

// Compensation Function for raw humidity readings.
// raw_hum: uint32_t containing the raw humidity data of the sensor.
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22
// integer and 10 fractional bits). Output value of “47445” represents
// 47445/1024 = 46.333 %RH
uint32_t compensate_hum(uint32_t raw_hum, SensorConstants *scp) {
  int32_t hum_buffer;
  hum_buffer = (scp->t_fine - ((int32_t)76800));
  hum_buffer = (((((raw_hum << 14) - (((int32_t)scp->dig_H4) << 20) -
                   (((int32_t)scp->dig_H5) * hum_buffer)) +
                  ((int32_t)16384)) >>
                 15) *
                (((((((hum_buffer * ((int32_t)scp->dig_H6)) >> 10) *
                     (((hum_buffer * ((int32_t)scp->dig_H3)) >> 11) +
                      ((int32_t)32768))) >>
                    10) +
                   ((int32_t)2097152)) *
                      ((int32_t)scp->dig_H2) +
                  8192) >>
                 14));
  hum_buffer =
      (hum_buffer - (((((hum_buffer >> 15) * (hum_buffer >> 15)) >> 7) *
                      ((int32_t)scp->dig_H1)) >>
                     4));
  hum_buffer = (hum_buffer < 0 ? 0 : hum_buffer);
  hum_buffer = (hum_buffer > 419430400 ? 419430400 : hum_buffer);
  return (uint32_t)(hum_buffer >> 12);
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
  scp->dig_H1 = comp_buffer[25];
  // start reading of second part of variables
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
  scp->dig_H5 =
      ((comp_buffer[4] & 0xF0) >> 4) | ((uint16_t)comp_buffer[5] << 4);
  scp->dig_H6 = comp_buffer[6];
  strcpy(tsp->status_msg, "COMP_LD_SUCC");
}

// Local function to determine the basic oversampling rate
uint8_t determine_general_ovs(uint8_t oversampling) {
  uint8_t choose_os;
  switch (oversampling) {
  case 0:
    choose_os = 0b000;
    break;
  case 1:
    choose_os = 0b001;
    break;
  case 2:
    choose_os = 0b010;
    break;
  case 4:
    choose_os = 0b011;
    break;
  case 8:
    choose_os = 0b100;
    break;
  case 16:
    choose_os = 0b101;
    break;
  default:
    choose_os = 0b001;
  }
  return choose_os;
}

// Local function to delay the device as long as the measurement takes.
// Since the _delay_ms() function needs a fixed value at compile time,
// three worst case alternatives are made.
// TODO: Find a more elegant solution, e.g. with timers.
void worst_case_delay(uint8_t ovs_t, uint8_t ovs_p, uint8_t ovs_h) {
  uint8_t state_machine = 0;
  uint8_t bit_1 = ovs_t != 0 ? 1 : 0;
  uint8_t bit_2 = ovs_p != 0 ? 1 : 0;
  uint8_t bit_3 = ovs_h != 0 ? 1 : 0;
  // States representing if an oversampling value is given, hence a
  // measurement will be done or not.
  state_machine = bit_1 + bit_2 + bit_3;
  // send_string("\n\rDEBUG!\n\r");
  // send_unsigned_decimal(state_machine);
  // send_string("\n\rDEBUG!\n\r");
  switch (state_machine) {
  case 0:
    break;
  case 1:
    _delay_ms(50);
    break;
  case 2:
    _delay_ms(100);
    break;
  case 3:
    _delay_ms(150);
    break;
  default:
    break;
  }
}

// Local function to transmit start of measurement and provide enough
// delay for the measurement to complete.
uint8_t start_measurement(uint8_t ovs_t, uint8_t ovs_p, uint8_t ovs_h,
                          TransmitStatus *tsp) {
  uint8_t ovs_h_reg_val = determine_general_ovs(ovs_h);
  uint8_t check_status = master_transmit_write_to_reg(
      BME280_ADDRESS_GND, BME280_CONTROL_HUM_REG, ovs_h_reg_val);
  // Worst case time taken for the measurement.
  if (check_status != 0) {
    strcpy(tsp->status_msg, "OVS_H_REG_ERR");
    return check_status;
  }
  uint8_t ovs_t_reg_val = determine_general_ovs(ovs_t);
  uint8_t ovs_p_reg_val = determine_general_ovs(ovs_h);
  uint8_t ctrl_reg_val =
      (ovs_t_reg_val << 5) | (ovs_p_reg_val << 2) | BME280_FORCE_MEAS;
  check_status = master_transmit_write_to_reg(
      BME280_ADDRESS_GND, BME280_CONTROL_MEAS_REG, ctrl_reg_val);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "START_MEAS_ERR");
    return check_status;
  }
  // Max Value here is about 120 ms so uint8_t is enough.
  worst_case_delay(ovs_t, ovs_p, ovs_h);
  return 0;
}

// Get the raw temperature value.
// oversampling: Choose temperature oversempling.
//  Valid values are 1, 2, 4, 8 and 16 other values result in oversampling = 1.
// tsp: Pointer to transmission status codes.
uint32_t bme_get_temp_raw(uint8_t oversampling, TransmitStatus *tsp) {
  // Start the measurement but only for temperature.
  uint8_t check_status = start_measurement(oversampling, 0, 0, tsp);
  if (check_status != 0) {
    return check_status;
  }
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
float bme_get_temp_degrees(SensorConstants *scp, uint8_t oversampling,
                           TransmitStatus *tsp) {
  int32_t raw_temp = bme_get_temp_raw(oversampling, tsp);
  int32_t temperature = compensate_temp(raw_temp, scp);
  float degree_temp = (float)temperature * 0.01;
  return degree_temp;
}

uint32_t bme_get_hum_raw(uint8_t oversampling, TransmitStatus *tsp) {
  // Start the measurement but only for humidity.
  uint8_t check_status = start_measurement(0, 0, oversampling, tsp);
  if (check_status != 0) {
    return check_status;
  }
  // Prepare to read measured data from humidity register.
  check_status = master_transmit_read_reg(BME280_ADDRESS_GND, BME280_HUM_REG);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "HUM_REG_ERR");
    return check_status;
  }
  // Create buffer to store the humidity value.
  uint8_t hum_data_buf[BME280_HUM_LEN] = {};
  uint32_t raw_hum = 0x0;
  check_status =
      master_receive_nbytes(BME280_ADDRESS_GND, hum_data_buf, BME280_HUM_LEN);
  if (check_status != 0) {
    strcpy(tsp->status_msg, "HUM_LD_ERR");
    return check_status;
  }
  // Get the correct value (1. MSB, 2. LSB)
  raw_hum = ((uint16_t)hum_data_buf[0] << 8) | hum_data_buf[1];
  strcpy(tsp->status_msg, "HUM_LD_SUCC");
  return raw_hum;
}

float bme_get_hum_percent(SensorConstants *scp, uint8_t oversampling,
                          TransmitStatus *tsp) {
  uint32_t raw_hum = bme_get_hum_raw(oversampling, tsp);
  uint32_t humidity = compensate_hum(raw_hum, scp);
  float percent_hum = (float)humidity / 1024;
  return percent_hum;
}
