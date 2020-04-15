/*
 * DrvSPI.h
 *
 *  Created on: March 25, 2020
 *      Author: us0jhys
 *      Reference: https://github.com/ij96/Honeywell_RSC/blob/master/src/Honeywell_RSC.h
 */

#ifndef DRIVERS_DRVSPI_DRVSPI_H_
#define DRIVERS_DRVSPI_DRVSPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "RegMap.h"
//#define ADC_CONVERSION_TIMEOUT 3
//
//#define NUMBER_OF_SAMPLES (uint8_t)4
//#define NUMBER_OF_ACIVE_CHANNELS 3
//#define ADC2_CHANNEL_8 0
//#define ADC2_CHANNEL_9 1
//#define ADC1_CHANNEL_5 2


 extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi4;
//extern SPI_HandleTypeDef hspi5;

#define CS_EE_Pin SPI1_CS_EE_Pin
#define CS_EE_Port SPI1_CS_EE_GPIO_Port
#define CS_ADC_Pin SPI1_CS_ADC_Pin
#define CS_ADC_Port SPI1_CS_ADC_GPIO_Port
#define DRDY_Pin SPI1_DRDY_Pin
#define DRDY_Port SPI1_DRDY_GPIO_Port

void DrvSPIInit();

// chip selection
void select_eeprom();
void deselect_eeprom();
void select_adc();
void deselect_adc();

// read number of bytes stored in EEPROM, starting from an address
void eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data);
void get_catalog_listing();
void get_serial_number();
void get_pressure_range();
void get_pressure_minimum();
void get_pressure_unit();
void get_pressure_type();
void get_coefficients();
void get_initial_adc_values(uint8_t *adc_init_values);

// read from ADC
void adc_read(READING_T type, uint8_t *data);
float get_temperature();
float get_pressure();

// write to ADC
void adc_write(uint8_t reg, uint8_t num_bytes, uint8_t *data);

// other ADC related functions
void setup_adc(uint8_t* adc_init_values);
void add_dr_delay();

// setter functions
void set_data_rate(RSC_DATA_RATE dr);
void set_mode(RSC_MODE mode);

// getter functions
/*char* catalog_listing() const {return _catalog_listing;}
char* serial_number() const {return _serial_number;}
float pressure_range() const {return _pressure_range;}
float pressure_minimum() const {return _pressure_minimum;}
char* pressure_unit_name() const {return _pressure_unit_name;}
char* pressure_type_name() const {return _pressure_type_name;}*/

// from EEPROM
unsigned char _catalog_listing[RSC_SENSOR_NAME_LEN];
char _serial_number[RSC_SENSOR_NUMBER_LEN];
float _pressure_range;
float _pressure_minimum;
PRESSURE_U _pressure_unit;
char* _pressure_unit_name;
PRESSURE_T _pressure_type;
char* _pressure_type_name;

// ADC
RSC_DATA_RATE _data_rate;
RSC_MODE _mode;

// for calculating compensated pressure
float _coeff_matrix[RSC_COEFF_T_ROW_NO][RSC_COEFF_T_COL_NO];
int32_t _t_raw;




#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_DRVSPI_DRVSPI_H_ */
