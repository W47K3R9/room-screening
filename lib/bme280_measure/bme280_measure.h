#ifndef BME280_MEASURE_H
#define BME280_MEASURE_H
#include <stdint.h>

// Chip ID
#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID 0x60

// I2C Adresses
// Address if SDO pin set to ground
#define BME280_ADDRESS_GND 0x76
// Address if SDO pin set to VCC
#define BME280_ADDRESS_VCC 0x77

// Compensation values
// Goes until 0xA1 (26 Bytes)
#define BME280_COMPENSATE_REG_1 0x88
#define BME280_COMPENSATE_REG_1_LEN 26
// Goes until 0xE7 (8 Bytes)
#define BME280_COMPENSATE_REG_2 0xE1
#define BME280_COMPENSATE_REG_2_LEN 8

// Soft Reset
#define BME280_RESET_REG 0xE0
#define BME280_RESET_VAL 0xB6

// Measurement control register
#define BME280_CONTROL_MEAS_REG 0xF4
// Start measurments
#define BME280_FORCE_MEAS 0x01
#define BME280_CYCLE_MEAS 0x03
#define BME280_SLEEP 0x00

// Oversampling settings register for humidity
#define BME280_CONTROL_HUM_REG 0xF2

#define BME280_CONFIG_REG 0xF5

// Read registers
#define BME280_TEMP_REG 0xFA
#define BME280_TEMP_LEN 3
#define BME280_HUM_REG 0xFD
#define BME280_HUM_LEN 2

typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;

  int32_t t_fine;
} SensorConstants;

typedef struct {
  uint32_t temperature_raw;
  uint32_t pressure_raw;
  uint32_t humidity_raw;
} RawData;

typedef struct {
  uint32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
} CompensatedData;

typedef struct {
  char status_msg[16];
  uint8_t chip_id;
} TransmitStatus;

// Compensation Formulae
int32_t compensate_temp(uint32_t raw_temp, SensorConstants *scp);
uint32_t compensate_hum(uint32_t raw_hum, SensorConstants *scp);

// initialize sensor, returns Chip-ID
void bme_init(TransmitStatus *tsp);
// read out fixed constants
void bme_load_comp_vals(SensorConstants *scp, TransmitStatus *tsp);
// get temperature
uint32_t bme_get_temp_raw(uint8_t oversampling, TransmitStatus *tsp);
float bme_get_temp_degrees(SensorConstants *scp, uint8_t oversampling,
                           TransmitStatus *tsp);
uint32_t bme_get_hum_raw(uint8_t oversampling, TransmitStatus *tsp);
float bme_get_hum_percent(SensorConstants *scp, uint8_t oversampling,
                             TransmitStatus *tsp);
#endif // BME280_MEASURE_H
