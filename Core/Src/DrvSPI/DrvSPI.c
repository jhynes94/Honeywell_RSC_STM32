/*
 * DrvSPI.c
 *
 *  Created on: March 25, 2020
 *      Author: us0jhys
 */

#include <stdbool.h>
#include <stddef.h>
#include <sys/_stdint.h>
#include <string.h>

//#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_tim.h"
#include "main.h"
//#include "cmsis_os.h"
//#include "AppPipette.h"
//#include "vfu.h"

#include "DrvSPI.h"
#include "RegMap.h"

void DrvSPIInit() {
  // read and store constants from EEPROM
  get_catalog_listing();
  get_serial_number();
  get_pressure_range();
  get_pressure_minimum();
  get_pressure_unit();
  get_pressure_type();

  // setup ADC
  uint8_t adc_init_values[4];
  get_initial_adc_values(adc_init_values);
//  setup_adc(adc_init_values);
//
//  get_coefficients();
//
//  set_data_rate(N_DR_20_SPS);
//  set_mode(NORMAL_MODE);
  HAL_Delay(5);

}

void setup_adc(uint8_t* adc_init_values) {
  // refer to datasheet section 3.4 ADC Programming Sequence – Power Up
  uint8_t command[5] = {RSC_ADC_RESET_COMMAND, adc_init_values[0], adc_init_values[1], adc_init_values[2], adc_init_values[3]};
  adc_write(0, 5, command);
  HAL_Delay(5);
}

void get_initial_adc_values(uint8_t* adc_init_values) {
  eeprom_read(RSC_ADC_CONFIG_00, 1, &adc_init_values[0]);
  HAL_Delay(2);
  eeprom_read(RSC_ADC_CONFIG_01, 1, &adc_init_values[1]);
  HAL_Delay(2);
  eeprom_read(RSC_ADC_CONFIG_02, 1, &adc_init_values[2]);
  HAL_Delay(2);
  eeprom_read(RSC_ADC_CONFIG_03, 1, &adc_init_values[3]);
  HAL_Delay(2);
}

void get_pressure_type() {
  char buf[RSC_SENSOR_TYPE_LEN];
  eeprom_read(RSC_PRESSURE_REFERENCE, RSC_SENSOR_TYPE_LEN, buf);
  switch (buf[0]) {
    case 'D':
      _pressure_type = DIFFERENTIAL;
      _pressure_type_name = "differential";
      break;
    case 'A':
      _pressure_type = ABSOLUTE;
      _pressure_type_name = "absolute";
      break;
    case 'G':
      _pressure_type = GAUGE;
      _pressure_type_name = "gauge";
      break;
    default:
      _pressure_type = DIFFERENTIAL;
      _pressure_type_name = "differential";
  }
}

void get_pressure_unit() {
  char buf[RSC_PRESSURE_UNIT_LEN] = {0};
  eeprom_read(RSC_PRESSURE_UNIT_MSB, RSC_PRESSURE_UNIT_LEN, buf);
  buf[RSC_PRESSURE_UNIT_LEN - 1] = '\0';
  if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'O') {
    _pressure_unit = INH2O;
    _pressure_unit_name = "inH2O";
  } else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'a') {
    if (buf[RSC_PRESSURE_UNIT_LEN - 4] == 'K') {
      _pressure_unit = KPASCAL;
      _pressure_unit_name = "kilopascal";
    } else if (buf[RSC_PRESSURE_UNIT_LEN - 4] == 'M') {
      _pressure_unit = MPASCAL;
      _pressure_unit_name = "megapascal";
    } else {
      _pressure_unit = PASCAL;
      _pressure_unit_name = "pascal";
    }
  } else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'r') {
    if (buf[RSC_PRESSURE_UNIT_LEN - 5] == 'm') {
      _pressure_unit = mBAR;
      _pressure_unit_name = "millibar";
    } else {
      _pressure_unit = BAR;
      _pressure_unit_name = "bar";
    }
  } else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'i') {
    _pressure_unit = PSI;
    _pressure_unit_name = "psi";
  }
}

void get_pressure_minimum() {
  uint8_t buf[RSC_PRESSURE_MINIMUM_LEN];
  eeprom_read(RSC_PRESSURE_MINIMUM_LSB, RSC_PRESSURE_MINIMUM_LEN, buf);
  // convert byte array to float (buf[0] is LSB)
  memcpy(&_pressure_minimum, &buf, sizeof(_pressure_minimum));
}

void get_pressure_range() {
  uint8_t buf[RSC_PRESSURE_RANGE_LEN];
  eeprom_read(RSC_PRESSURE_RANGE_LSB, RSC_PRESSURE_RANGE_LEN, buf);
  // convert byte array to float (buf[0] is LSB)
  memcpy(&_pressure_range, &buf, sizeof(_pressure_range));
}

void get_serial_number() {
  eeprom_read(RSC_SERIAL_NO_YYYY_MSB, RSC_SENSOR_NUMBER_LEN, _serial_number);
}

void get_catalog_listing() {
  eeprom_read(RSC_CATALOG_LISTING_MSB, RSC_SENSOR_NAME_LEN, _catalog_listing);
}

void eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data) {
  // generate command (refer to sensor datasheet section 2.2)
  uint8_t command[2] = {0};
  command[0] = RSC_READ_EEPROM_INSTRUCTION | ((address & RSC_EEPROM_ADDRESS_9TH_BIT_MASK) >> 5);
  command[1] = address & 0xFF;

  // send command
  // select EEPROM
  select_eeprom();
  HAL_StatusTypeDef SPI_Status;
  SPI_Status = HAL_SPI_Transmit(&hspi1, &command[0], 1, HAL_MAX_DELAY); //SPI.transfer(command[0]);
  SPI_Status = HAL_SPI_Transmit(&hspi1, &command[1], 1, HAL_MAX_DELAY); //SPI.transfer(command[1]);

  // receive results
  // - results are transmitted back after the last bit of the command is sent
  // - to get results, just transfer dummy data, as subsequent bytes will not used by sensor
  for (int i = 0; i < num_bytes; i++) {
	  uint8_t a = 0x00;
	  SPI_Status = HAL_SPI_TransmitReceive(&hspi1, &a, &data[i], 1, HAL_MAX_DELAY); //data[i] = SPI.transfer(0x00);
	  //SPI_Status = HAL_SPI_Receive(&hspi1, &data[i], 1, HAL_MAX_DELAY); //data[i] = SPI.transfer(0x00);
  }
  // deselect EEPROM
  // - after command is sent, the sensor will keep sending bytes from EEPROM,
  //   in ascending order of address. Resetting the CS_EE pin at the end of
  //   the function means that when reading from EEPROM next time, the result
  //   would start at the correct address.
  deselect_eeprom();
}


void select_eeprom() {
  // enable CS_EE
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_EE_Pin, GPIO_PIN_RESET);

  // make sure CS_ADC is not active
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_ADC_Pin, GPIO_PIN_SET);

  // the EEPROM interface operates in SPI mode 0 (CPOL = 0, CPHA = 0) or mode 3 (CPOL = 1, CPHA = 1)
//  SPI.beginTransaction(SPISettings(1250000, MSBFIRST, SPI_MODE0));
}

void deselect_eeprom() {
//  SPI.endTransaction();
	  HAL_GPIO_WritePin(GPIOB, SPI1_CS_EE_Pin, GPIO_PIN_SET);
}
/*
void adc_write(uint8_t reg, uint8_t num_bytes, uint8_t* data) {
  // check if the register and the number of bytes are valid
  // The number of bytes to write has to be - 1,2,3,4
  if (num_bytes <= 0 || num_bytes > 4)
    return;

  // the ADC registers are 0,1,2,3
  if (reg > 3)
    return;

  // generate command
  // the ADC REG Write (WREG) command is as follows: 0100 RRNN
  //   RR - Register Number             (0,1,2,3)
  //   NN - Number of Bytes to send - 1 (0,1,2,3)
  uint8_t command[num_bytes + 1];
  command[0] = RSC_ADC_WREG
               | ((reg << 2) & RSC_ADC_REG_MASK)
               | ((num_bytes - 1) & RSC_ADC_NUM_BYTES_MASK);

  for (int i = 0; i < num_bytes; i++) {
    command[i + 1] = data[i];
  }

  // send command
  select_adc();
  for (int i = 0; i < num_bytes + 1; i++) {
    SPI.transfer(command[i]);
  }
  deselect_adc();
}

void select_adc() {
  // enable CS_ADC
  digitalWrite(_cs_adc_pin, LOW);

  // make sure CS_EE is not active
  digitalWrite(_cs_ee_pin, HIGH);

  // the ADC interface operates in SPI mode 1 (CPOL = 0, CPHA = 1)
  SPI.beginTransaction(SPISettings(1250000, MSBFIRST, SPI_MODE1));
}

void deselect_adc() {
  SPI.endTransaction();
  digitalWrite(_cs_adc_pin, HIGH);
}
*/

/*
static void SPI_Init_EEPROM(void)
{
  // SPI1 parameter configuration
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}*/
