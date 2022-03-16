#include "bmp280.h"

bmp280_driver::bmp280_driver(I2C_HandleTypeDef* I2C_PORT , uint8_t bmp_addr)
{
	_bmp_addr = bmp_addr;
	_I2C_PORT = I2C_PORT;

	update_callibration();
	reg_ctrl_meas(BMP280_OSRS_X16, BMP280_OSRS_X16, BMP280_POWER_NORM);
	reg_config(BMP280_STBY_1000, BMP280_FILTER_16, BMP280_CONFIG_SPI_OFF);
}

HAL_StatusTypeDef bmp280_driver::read_byte(uint8_t addr, uint8_t* result)
{
	uint8_t buff;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, addr, I2C_MEMADD_SIZE_8BIT, &buff, 1, HAL_MAX_DELAY);

	*result = buff;

	return status;
}

HAL_StatusTypeDef bmp280_driver::write_byte(uint8_t data, uint8_t addr)
{
	HAL_StatusTypeDef status;
	uint8_t buff = data;

	status = HAL_I2C_Mem_Write(_I2C_PORT, _bmp_addr << 1, addr, I2C_MEMADD_SIZE_8BIT, &buff, 1, HAL_MAX_DELAY);
	return status;
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

void bmp280_driver::update_callibration()
{
	uint16_t buff[12];

	HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, BMP280_REG_DIG_T1_lsb, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buff, 24, HAL_MAX_DELAY);

	calib_data.dig_T1 = buff[0];
	calib_data.dig_T2 = buff[1];
	calib_data.dig_T3 = buff[2];
	calib_data.dig_P1 = buff[3];
	calib_data.dig_P2 = buff[4];
	calib_data.dig_P3 = buff[5];
	calib_data.dig_P4 = buff[6];
	calib_data.dig_P5 = buff[7];
	calib_data.dig_P6 = buff[8];
	calib_data.dig_P7 = buff[9];
	calib_data.dig_P8 = buff[10];
	calib_data.dig_P9 = buff[11];
}

HAL_StatusTypeDef bmp280_driver::config_stanbytime(uint8_t time)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_CONFIG, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b111 << 5);
	buff |= time << 5;

	write_byte(buff, BMP280_REG_CONFIG);
	return status;
}

HAL_StatusTypeDef bmp280_driver::config_filter(uint8_t filter)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_CONFIG, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b111 << 2);
	filter &= ~(0xF8);
	buff |= filter << 2;

	write_byte(buff, BMP280_REG_CONFIG);
	return status;
}

HAL_StatusTypeDef bmp280_driver::config_SPI(uint8_t SPI_ON_OFF)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_CONFIG, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b1);
	SPI_ON_OFF &= ~(0xFE);
	buff |= SPI_ON_OFF;

	write_byte(buff, BMP280_REG_CONFIG);
	return status;
}

HAL_StatusTypeDef bmp280_driver::config_temp_oversample(uint8_t temp_oversample)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_ctr_meas, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b111 << 5);
	buff |= temp_oversample << 5;

	write_byte(buff, BMP280_REG_ctr_meas);
	return status;
}

HAL_StatusTypeDef bmp280_driver::config_pressure_oversample(uint8_t pressure_oversample)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_ctr_meas, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b111 << 2);
	pressure_oversample &= ~(0xF8);
	buff |= pressure_oversample << 2;

	write_byte(buff, BMP280_REG_ctr_meas);
	return status;
}

HAL_StatusTypeDef bmp280_driver::config_mode(uint8_t mode)
{
	uint8_t buff;
	HAL_StatusTypeDef status;

	status = read_byte(BMP280_REG_ctr_meas, &buff);
	if (status != HAL_OK) return status;

	buff &= ~(0b11);
	mode &= ~(0xFC);
	buff |= mode;

	write_byte(buff, BMP280_REG_ctr_meas);
	return status;
}

int32_t bmp280_driver::get_t_fine()
{
	uint8_t _adc_t[3];
	HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, BMP280_REG_temp_msb, I2C_MEMADD_SIZE_8BIT, _adc_t, 3, HAL_MAX_DELAY);

	uint32_t adc_t = _adc_t[0] << 12 | _adc_t[1] << 4 | _adc_t[2] >> 4;

	double var1 = (((double)adc_t / 16384) - ((double)calib_data.dig_T1 / 1024)) * (double)calib_data.dig_T2;
	double var2 = (((double)adc_t / 131072 - (double)calib_data.dig_T1 / 8192) * ((double)adc_t / 131072 - (double)calib_data.dig_T1 / 8192));
	var2 = var2 * (double)calib_data.dig_T3;

	int32_t result = (uint32_t)(var1 + var2);

	return result;
}

float bmp280_driver::getLastTemp()
{

	uint8_t _adc_t[3];
	HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, BMP280_REG_temp_msb, I2C_MEMADD_SIZE_8BIT, _adc_t, 3, HAL_MAX_DELAY);

	uint32_t adc_t = _adc_t[0] << 12 | _adc_t[1] << 4 | _adc_t[2] >> 4;

	double var1 = (((double)adc_t / 16384) - ((double)calib_data.dig_T1 / 1024)) * (double)calib_data.dig_T2;
	double var2 = (((double)adc_t / 131072 - (double)calib_data.dig_T1 / 8192) * ((double)adc_t / 131072 - (double)calib_data.dig_T1 / 8192));
	var2 = var2 * (double)calib_data.dig_T3;
	double result = (var1 + var2) / 5120;

	return result;
}

float bmp280_driver::getLastPressure()
{
	uint8_t _adc_p[3];

	HAL_I2C_Mem_Read(_I2C_PORT, _bmp_addr << 1, BMP280_REG_press_msb, I2C_MEMADD_SIZE_8BIT, _adc_p, 3, HAL_MAX_DELAY);

	uint32_t adc_p = _adc_p[0] << 12 | _adc_p[1] << 4 | _adc_p[2] >> 4;

	uint32_t t_fine = get_t_fine();

	double var1 = ((double)t_fine/2.0) - 64000;
	double var2 = var1 * var1 * ((double)calib_data.dig_P6) / 32768;
	var2 = var2 + var1 * ((double)calib_data.dig_P5) * 2.0;
	var2 = (var2/4.0) + (((double)calib_data.dig_P4) * 65536.0);
	var1 = (((double)calib_data.dig_P3) * var1 * var1 / 524288.0 + ((double)calib_data.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1/32768.0) * ((double)calib_data.dig_P1);
	double p = 1048576.0 - (double)adc_p;
	p = (p - (var2/4096.0)) * 6250.0 / var1;
	var1 = ((double)calib_data.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)calib_data.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)calib_data.dig_P7)) / 16.0;

	return p;
}
