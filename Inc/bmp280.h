#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <main.h>

#define BMP280_DEFAULT_ADDR 0x76

//Options Registers

//SPI Option in register CONFIG (0xF5)
#define BMP280_CONFIG_SPI_OFF	0b0
#define BMP280_CONFIG_SPI_ON	0b1

//Stand By Option in register CONFIG (0xF5)

#define BMP280_STBY_0_5			0b000
#define BMP280_STBY_62_5		0b001
#define BMP280_STBY_125			0b010
#define BMP280_STBY_250 		0b011
#define BMP280_STBY_500			0b100
#define BMP280_STBY_1000		0b101
#define BMP280_STBY_2000		0b110
#define BMP280_STBY_4000		0b111

//Filter Option in register CONFIG (0xF5)

#define BMP280_FILTER_OFF		0b000
#define BMP280_FILTER_2 		0b001
#define BMP280_FILTER_4			0b010
#define BMP280_FILTER_8			0b011
#define BMP280_FILTER_16		0b111

//OverSampling Option in register ctrl_meas (0xF4)

#define BMP280_OSRS_SKIP		0b000
#define BMP280_OSRS_X1			0b001
#define BMP280_OSRS_X2			0b010
#define BMP280_OSRS_X4			0b011
#define BMP280_OSRS_X8			0b100
#define BMP280_OSRS_X16			0b111

//Mode Option in register ctrl_meas

#define BMP280_POWER_SLEEP		0b00
#define BMP280_POWER_FORCE		0b10
#define BMP280_POWER_NORM		0b11

//Data registers (TEMP PRESSURE)

#define BMP280_REG_temp_xlsb	0xFC
#define BMP280_REG_temp_lsb		0xFB
#define BMP280_REG_temp_msb		0xFA
#define BMP280_REG_press_xlsb	0xF9
#define BMP280_REG_press_lsb	0xF8
#define BMP280_REG_press_msb	0xF7

//Config Registers

#define BMP280_REG_CONFIG		0xF5
#define BMP280_REG_ctr_meas		0xF4

//Misc Registers

#define BMP280_REG_STATUS		0xF3
#define BMP280_REG_RESET		0xE0
#define BMP280_REG_ID			0xD0

//Сalibration registers (DIG_PRESSURE DIG_TEMP)

#define BMP280_REG_DIG_T1_lsb	0x88
#define BMP280_REG_DIG_T1_msb	0x89

#define BMP280_REG_DIG_T2_lsb 	0x8A
#define BMP280_REG_DIG_T2_msb	0x8B

#define BMP280_REG_DIG_T3_lsb	0x8C
#define BMP280_REG_DIG_T3_msb	0x8D

#define BMP280_REG_DIG_P1_lsb 	0x8E
#define BMP280_REG_DIG_P1_msb	0x8F

#define BMP280_REG_DIG_P2_lsb	0x90
#define BMP280_REG_DIG_P2_msb	0x91

#define BMP280_REG_DIG_P3_lsb	0x92
#define BMP280_REG_DIG_P3_msb	0x93

#define BMP280_REG_DIG_P4_lsb	0x94
#define BMP280_REG_DIG_P4_msb	0x95

#define BMP280_REG_DIG_P5_lsb	0x96
#define BMP280_REG_DIG_P5_msb	0x97

#define BMP280_REG_DIG_P6_lsb	0x98
#define BMP280_REG_DIG_P6_msb	0x99

#define BMP280_REG_DIG_P7_lsb	0x9A
#define BMP280_REG_DIG_P7_msb	0x9B

#define BMP280_REG_DIG_P8_lsb	0x9C
#define BMP280_REG_DIG_P8_msb	0x9D

#define BMP280_REG_DIG_P9_lsb	0x9E
#define BMP280_REG_DIG_P9_msb	0x9F

//struct for store callibration data

struct calibration_data
{
	uint16_t 	dig_T1;
	int16_t 	dig_T2;
	int16_t		dig_T3;
	uint16_t	dig_P1;
	int16_t		dig_P2;
	int16_t		dig_P3;
	int16_t		dig_P4;
	int16_t		dig_P5;
	int16_t		dig_P6;
	int16_t		dig_P7;
	int16_t		dig_P8;
	int16_t		dig_P9;
};

//BMP280_driver by tdehtyar

class bmp280_driver
{
private:
	//Address of BMP280
	uint8_t _bmp_addr;
	//I2C Handle
	I2C_HandleTypeDef* _I2C_PORT;
	//Calibration data
	calibration_data calib_data;

	HAL_StatusTypeDef read_byte(uint8_t reg_addr, uint8_t* result);
	HAL_StatusTypeDef write_byte(uint8_t data, uint8_t reg_addr);

	HAL_StatusTypeDef reg_ctrl_meas(uint8_t temp_oversample, uint8_t pessure_oversample, uint8_t mode);
	HAL_StatusTypeDef reg_config(uint8_t st_time, uint8_t filter, uint8_t SPI_EN);

	int32_t get_t_fine();

	void update_callibration();

public:

	bmp280_driver(I2C_HandleTypeDef* I2C_PORT , uint8_t bmp_addr);

	HAL_StatusTypeDef config_stanbytime(uint8_t time);
	HAL_StatusTypeDef config_filter(uint8_t filter);
	HAL_StatusTypeDef config_SPI(uint8_t SPI_ON_OFF);

	HAL_StatusTypeDef config_temp_oversample(uint8_t temp_oversample);
	HAL_StatusTypeDef config_pressure_oversample(uint8_t pressure_oversample);
	HAL_StatusTypeDef config_mode(uint8_t mode);

	float getLastTemp();

	float getLastPressure();
};

#endif /* INC_BMP280_H_ */
