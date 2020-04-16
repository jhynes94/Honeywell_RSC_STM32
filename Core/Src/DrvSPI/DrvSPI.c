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
  setup_adc(adc_init_values);

  get_coefficients();

  set_data_rate(N_DR_20_SPS);
  set_mode(NORMAL_MODE);
  HAL_Delay(5);

}

float get_pressure() {
  // reads uncompensated pressure from ADC, then use temperature reading to convert it to compensated pressure
  // refer to datasheet section 3.6 ADC Programming and Read Sequence – Pressure Reading

  // read the 24 bits uncompensated pressure
  uint8_t sec_arr[4]={0};
  adc_read(PRESSURE, sec_arr);


  uint32_t Justins_raw_Pressure = (sec_arr[1]<<8)|(sec_arr[2]);

  uint32_t p_raw = (sec_arr[1]<<16)|(sec_arr[2]<<8)|sec_arr[3];

  uint32_t t_raw = 25;
  float x = (_coeff_matrix[0][3]*t_raw*t_raw*t_raw);
  float y = (_coeff_matrix[0][2]*t_raw*t_raw);
  float z = (_coeff_matrix[0][1]*t_raw);
  float p_int1 = p_raw - (x + y + z + _coeff_matrix[0][0]);

  x = (_coeff_matrix[1][3]*t_raw*t_raw*t_raw);
  y = (_coeff_matrix[1][2]*t_raw*t_raw);
  z = (_coeff_matrix[1][1]*t_raw);
  float p_int2 = p_int1/(x + y + z + _coeff_matrix[1][0]);

  x = (_coeff_matrix[2][3]*p_int2*p_int2*p_int2);
  y = (_coeff_matrix[2][2]*p_int2*p_int2);
  z = (_coeff_matrix[2][1]*p_int2);
  float p_comp_fs = x + y + z + _coeff_matrix[2][0];

  float p_comp = (p_comp_fs*_pressure_range) + _pressure_minimum;

  return p_comp;
}

float get_temperature() {
  // reads temperature from ADC, stores raw value in sensor object, but returns the temperature in Celsius
  // refer to datasheet section 3.5 ADC Programming and Read Sequence – Temperature Reading

  uint8_t sec_arr[4] = {0};

  adc_read(TEMPERATURE, sec_arr);

  // first 14 bits represent temperature
  // following 10 bits are random thus discarded
  _t_raw = (((int32_t)sec_arr[1] << 8) | (int32_t)sec_arr[2]) >> 2;
  float temp = _t_raw * 0.03125;

  return temp;
}

void set_data_rate(RSC_DATA_RATE dr) {
  _data_rate = dr;
  switch (dr) {
    case N_DR_20_SPS:
    case N_DR_45_SPS:
    case N_DR_90_SPS:
    case N_DR_175_SPS:
    case N_DR_330_SPS:
    case N_DR_600_SPS:
    case N_DR_1000_SPS:
      set_mode(NORMAL_MODE);
      break;
    case F_DR_40_SPS:
    case F_DR_90_SPS:
    case F_DR_180_SPS:
    case F_DR_350_SPS:
    case F_DR_660_SPS:
    case F_DR_1200_SPS:
    case F_DR_2000_SPS:
      set_mode(FAST_MODE);
      break;
    default:
      set_mode(NA_MODE);
  }
}

void set_mode(RSC_MODE mode) {
  RSC_MODE l_mode;

  switch (mode) {
    case NORMAL_MODE:
      if (_data_rate < N_DR_20_SPS || _data_rate > N_DR_1000_SPS) {
        //Serial.println("RSC: Normal mode not supported with the current selection of data rate\n");
        //Serial.println("RSC: You will see erronous readings\n");
        l_mode = NA_MODE;
      } else
        l_mode = NORMAL_MODE;
      break;
    case FAST_MODE:
      if (_data_rate < F_DR_40_SPS || _data_rate > F_DR_2000_SPS) {
        //Serial.println("RSC: Fast mode not supported with the current selection of data rate\n");
        //Serial.println("RSC: You will see erronous readings\n");
        l_mode = NA_MODE;
      } else
        l_mode = FAST_MODE;
      break;
    default:
      l_mode = NA_MODE;
  }
  _mode = l_mode;
}

void get_coefficients() {
  int base_address = RSC_OFFSET_COEFFICIENT_0_LSB;
  uint8_t buf[4] = {0};
  int i, j = 0;

  // coeff matrix structure
  // _coeff_matrix[i][j]
  //  i\j   0                     1                     2                     3
  //  0   OffsetCoefficient0    OffsetCoefficient1    OffsetCoefficient2    OffsetCoefficient3
  //  1   SpanCoefficient0      SpanCoefficient1      SpanCoefficient2      SpanCoefficient3
  //  2   ShapeCoefficient0     ShapeCoefficient1     ShapeCoefficient2     ShapeCoefficient3

  // storing all the coefficients
  for (i = 0; i < RSC_COEFF_T_ROW_NO; i++) {
    for (j = 0; j < RSC_COEFF_T_COL_NO; j++) {
      // 80 is the number of bytes that separate the beginning
      // of the address spaces of all the 3 coefficient groups
      // refer the datasheet for more info
      base_address = RSC_OFFSET_COEFFICIENT_0_LSB + i * 80 + j * 4;
      eeprom_read(base_address, 4, buf);
      memcpy(&_coeff_matrix[i][j], (&buf), sizeof(_coeff_matrix[i][j]));
    }
  }
}

void setup_adc(uint8_t* adc_init_values) {
  // refer to datasheet section 3.4 ADC Programming Sequence – Power Up
  uint8_t command[6] = {RSC_ADC_RESET_COMMAND, 0x43, adc_init_values[0], adc_init_values[1], adc_init_values[2], adc_init_values[3]};
  select_adc();
  HAL_StatusTypeDef SPI_Status;
  for (int i = 0; i < 6; i++) {
	  SPI_Status = HAL_SPI_Transmit(&hspi1, &command[i], 1, HAL_MAX_DELAY); //SPI.transfer(command[0]);
  }
  if (SPI_Status != HAL_OK)
  {
    Error_Handler();
  }
  deselect_adc();

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
	  uint8_t a = 0x10;
	  SPI_Status = HAL_SPI_TransmitReceive(&hspi1, &a, &data[i], 1, HAL_MAX_DELAY); //data[i] = SPI.transfer(0x00);
  }
  if (SPI_Status != HAL_OK)
  {
    Error_Handler();
  }

  // deselect EEPROM
  // - after command is sent, the sensor will keep sending bytes from EEPROM,
  //   in ascending order of address. Resetting the CS_EE pin at the end of
  //   the function means that when reading from EEPROM next time, the result
  //   would start at the correct address.
  deselect_eeprom();
}


void select_eeprom() {
  // make sure CS_ADC is not active
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_ADC_Pin, GPIO_PIN_SET);

  // enable CS_EE
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_EE_Pin, GPIO_PIN_RESET);

  // the EEPROM interface operates in SPI mode 0 (CPOL = 0, CPHA = 0) or mode 3 (CPOL = 1, CPHA = 1)
  SPI_Init_EEPROM(); //SPI.beginTransaction(SPISettings(1250000, MSBFIRST, SPI_MODE0));
}

void deselect_eeprom() {
  HAL_SPI_DeInit(&hspi1); //SPI.endTransaction();
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_EE_Pin, GPIO_PIN_SET);
}

void adc_read(READING_T type, uint8_t *data) {
  // refer to datasheet section 3

  // need to configure the ADC to temperature mode first
  // generate command
  uint8_t command[2] = {0};
  // WREG byte
  command[0] = RSC_ADC_WREG
               | ((1 << 2) & RSC_ADC_REG_MASK);
  // configuration byte, which includes DataRate, Mode, Pressure/Temperature choice
  command[1] = (((_data_rate << RSC_DATA_RATE_SHIFT) & RSC_DATA_RATE_MASK)
                | ((_mode << RSC_OPERATING_MODE_SHIFT) & RSC_OPERATING_MODE_MASK)
                | (((type & 0x01) << 1) | RSC_SET_BITS_MASK));
  // send command
  select_adc();
  HAL_StatusTypeDef SPI_Status;
  SPI_Status = HAL_SPI_Transmit(&hspi1, &command[0], 1, HAL_MAX_DELAY); //SPI.transfer(command[0]);
  SPI_Status = HAL_SPI_Transmit(&hspi1, &command[1], 1, HAL_MAX_DELAY); //SPI.transfer(command[1]);

  add_dr_delay();

  // receive results
  // send 0x10 command to start data conversion on ADC
  uint8_t startCommCommand = 0x10; //SPI.transfer(0x10);
  SPI_Status = HAL_SPI_Transmit(&hspi1, &startCommCommand, 1, HAL_MAX_DELAY); //SPI.transfer(command[1]);
  for (int i = 0; i < 4; i++) {
    uint8_t a = 0x00;
    SPI_Status = HAL_SPI_TransmitReceive(&hspi1, &a, &data[i], 1, HAL_MAX_DELAY); //data[i] = SPI.transfer(0x00);
  }
  if (SPI_Status != HAL_OK)
  {
    Error_Handler();
  }
  deselect_adc();
}

void select_adc() {
  // make sure CS_EE is not active
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_EE_Pin, GPIO_PIN_SET); //digitalWrite(_cs_ee_pin, HIGH);

  // enable CS_ADC
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_ADC_Pin, GPIO_PIN_RESET); //digitalWrite(_cs_adc_pin, LOW);

  // the ADC interface operates in SPI mode 1 (CPOL = 0, CPHA = 1)
  SPI_Init_ADC(); //SPI.beginTransaction(SPISettings(1250000, MSBFIRST, SPI_MODE1));
}

void deselect_adc() {
  HAL_SPI_DeInit(&hspi1); //SPI.endTransaction();
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_ADC_Pin, GPIO_PIN_SET); //digitalWrite(_cs_adc_pin, HIGH);
}


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
}

static void SPI_Init_ADC(void)
{
  // SPI1 parameter configuration
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
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

}

void add_dr_delay() {
  float delay_duration = 0;
  // calculating delay based on the Data Rate
  switch (_data_rate) {
    case N_DR_20_SPS:
      delay_duration = MSEC_PER_SEC / 20;
      break;
    case N_DR_45_SPS:
      delay_duration = MSEC_PER_SEC / 45;
      break;
    case N_DR_90_SPS:
      delay_duration = MSEC_PER_SEC / 90;
      break;
    case N_DR_175_SPS:
      delay_duration = MSEC_PER_SEC / 175;
      break;
    case N_DR_330_SPS:
      delay_duration = MSEC_PER_SEC / 330;
      break;
    case N_DR_600_SPS:
      delay_duration = MSEC_PER_SEC / 600;
      break;
    case N_DR_1000_SPS:
      delay_duration = MSEC_PER_SEC / 1000;
      break;
    case F_DR_40_SPS:
      delay_duration = MSEC_PER_SEC / 40;
      break;
    case F_DR_90_SPS:
      delay_duration = MSEC_PER_SEC / 90;
      break;
    case F_DR_180_SPS:
      delay_duration = MSEC_PER_SEC / 180;
      break;
    case F_DR_350_SPS:
      delay_duration = MSEC_PER_SEC / 350;
      break;
    case F_DR_660_SPS:
      delay_duration = MSEC_PER_SEC / 660;
      break;
    case F_DR_1200_SPS:
      delay_duration = MSEC_PER_SEC / 1200;
      break;
    case F_DR_2000_SPS:
      delay_duration = MSEC_PER_SEC / 2000;
      break;
    default:
      delay_duration = 50;
  }
  HAL_Delay((int)delay_duration + 2);
}
