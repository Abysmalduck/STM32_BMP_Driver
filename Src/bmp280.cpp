#include "bmp280.h"

bmp280_driver::bmp280_driver(I2C_HandleTypeDef* I2C_PORT , uint8_t bmp_addr)
{
	_bmp_addr = bmp_addr;
	_I2C_PORT = I2C_PORT;
}
uint8_t bmp280_driver::read_byte(uint8_t addr)
{
	uint8_t buff;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, addr, I2C_MEMADD_SIZE_8BIT, &buff, 1, HAL_MAX_DELAY);
	if (status == HAL_OK) return buff;
	else return 0;
}

HAL_StatusTypeDef bmp280_driver::reg_ctrl_meas(uint8_t temp_oversample, uint8_t pessure_oversample, uint8_t mode)
{
	uint8_t ctrl_meas = (temp_oversample << 5) | (pessure_oversample << 2) | mode;
	HAL_StatusTypeDef status;
	status = write_byte(ctrl_meas, BMP280_REG_ctr_meas);
	return status;
}

HAL_StatusTypeDef bmp280_driver::reg_config(uint8_t st_time, uint8_t filter, uint8_t SPI_EN)
{
	uint8_t config_reg = (st_time << 5) | (filter << 3) | SPI_EN;
	HAL_StatusTypeDef status;
	status = write_byte(config_reg, BMP280_REG_CONFIG);
	return status;
}

HAL_StatusTypeDef bmp280_driver::write_byte(uint8_t data, uint8_t addr)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(_I2C_PORT, _bmp_addr << 1, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef bmp280_driver::update_callibration()
{
	uint8_t buffer_lsb;
	uint8_t buffer_msb;

	buffer_lsb = read_byte(BMP280_REG_DIG_T1_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_T1_msb);
	calib_data.dig_T1 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_T2_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_T2_msb);
	calib_data.dig_T2 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_T3_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_T3_msb);
	calib_data.dig_T3 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P1_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P1_msb);
	calib_data.dig_P1 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P2_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P2_msb);
	calib_data.dig_P2 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P3_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P3_msb);
	calib_data.dig_P3 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P4_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P4_msb);
	calib_data.dig_P4 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P5_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P5_msb);
	calib_data.dig_P5 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P6_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P6_msb);
	calib_data.dig_P6 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P7_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P7_msb);
	calib_data.dig_P7 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P8_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P8_msb);
	calib_data.dig_P8 = buffer_msb << 8 | buffer_lsb;

	buffer_lsb = read_byte(BMP280_REG_DIG_P9_lsb);
	buffer_msb = read_byte(BMP280_REG_DIG_P9_msb);
	calib_data.dig_P9 = buffer_msb << 8 | buffer_lsb;
}


float bmp280_driver::getLastTemp()
{
	uint8_t adc_t_xlsb = read_byte(BMP280_REG_temp_xlsb);
	uint8_t adc_t_lsb = read_byte(BMP280_REG_temp_lsb);
	uint8_t adc_t_msb = read_byte(BMP280_REG_temp_msb);

	uint32_t adc_t = adc_t_msb << 12 | adc_t_lsb << 4 | adc_t_xlsb >> 4;

	double var1 = (((double)adc_t / (double)16384) - ((double)calib_data.dig_T1 / (double)1024)) * (double)calib_data.dig_T2;
	double var2 = (((double)adc_t / (double)131072 - (double)calib_data.dig_T1 / (double)8192) * ((double)adc_t / (double)131072 - (double)calib_data.dig_T1 / (double)8192)) * (double)calib_data.dig_T3;

	double result = (var1 + var2) / (double)5120;

	return result;

}
