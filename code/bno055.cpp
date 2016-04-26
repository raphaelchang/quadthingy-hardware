#include "ch.h"
#include "hal.h"
#include <string.h>
#include "bno055.h"

static const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

bool bno055_init()
{
	i2cStart(&I2CD1, &i2ccfg);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	uint8_t id = bno055_read_addr(BNO055_CHIP_ID_ADDR);
	if(id != BNO055_ID)
	{
	    chThdSleepMilliseconds(1000);
		id = bno055_read_addr(BNO055_CHIP_ID_ADDR);
		if(id != BNO055_ID) {
			return false;
		}
	}
	bno055_write_addr(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
	chThdSleepMilliseconds(30);
	bno055_write_addr(BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (bno055_read_addr(BNO055_CHIP_ID_ADDR) != BNO055_ID)
	{
		chThdSleepMilliseconds(10);
	}
	chThdSleepMilliseconds(50);

	/* Set to normal power mode */
	bno055_write_addr(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	chThdSleepMilliseconds(10);

	bno055_write_addr(BNO055_PAGE_ID_ADDR, 0);
	bno055_write_addr(BNO055_SYS_TRIGGER_ADDR, 0x80);
	chThdSleepMilliseconds(10);
	bno055_write_addr(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
	chThdSleepMilliseconds(50);
	return true;
}

vector3 bno055_get_vector(uint8_t addr)
{
	vector3 vector;
	uint8_t buffer[6];
	memset (buffer, 0, 6);
	bno055_read_len(addr, 6, buffer);
	int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
	switch(addr)
	{
	case VECTOR_MAGNETOMETER:
		/* 1uT = 16 LSB */
		vector.x = ((double)x)/16.0;
		vector.y = ((double)y)/16.0;
		vector.z = ((double)z)/16.0;
		break;
	case VECTOR_GYROSCOPE:
		/* 1rps = 900 LSB */
		vector.x = ((double)x)/900.0;
		vector.y = ((double)y)/900.0;
		vector.z = ((double)z)/900.0;
		break;
	case VECTOR_EULER:
		/* 1 degree = 16 LSB */
		vector.z = ((double)x)/16.0;
		vector.y = ((double)y)/16.0;
		vector.x = ((double)z)/16.0;
		break;
	case VECTOR_ACCELEROMETER:
	case VECTOR_LINEARACCEL:
	case VECTOR_GRAVITY:
		/* 1m/s^2 = 100 LSB */
		vector.x = ((double)x)/100.0;
		vector.y = ((double)y)/100.0;
		vector.z = ((double)z)/100.0;
		break;
	}

	return vector;
}

uint8_t bno055_get_status()
{
	return bno055_read_addr(BNO055_SYS_STAT_ADDR);
}

uint8_t bno055_get_error()
{
	return bno055_read_addr(BNO055_SYS_ERR_ADDR);
}

uint8_t bno055_read_addr(uint8_t addr)
{
 	systime_t tmo = MS2ST(4);
	uint8_t txbuf[1];
	uint8_t rxbuf[1];
	txbuf[0] = addr;
	i2cAcquireBus(&I2CD1);
	uint8_t stat = i2cMasterTransmitTimeout(&I2CD1, BNO055_ADDR, txbuf, 1, rxbuf, 1, tmo);
	i2cReleaseBus(&I2CD1);
	return rxbuf[0];
}

void bno055_read_len(uint8_t addr, uint8_t len, uint8_t *buffer)
{
 	systime_t tmo = MS2ST(4);
	uint8_t txbuf[1];
	txbuf[0] = addr;
	i2cAcquireBus(&I2CD1);
	uint8_t stat = i2cMasterTransmitTimeout(&I2CD1, BNO055_ADDR, txbuf, 1, buffer, len, tmo);
	i2cReleaseBus(&I2CD1);
}

void bno055_write_addr(uint8_t addr, uint8_t value)
{
 	systime_t tmo = MS2ST(4);
	uint8_t txbuf[2];
	uint8_t rxbuf[1];
	txbuf[0] = addr;
	txbuf[1] = value;
	i2cAcquireBus(&I2CD1);
	uint8_t stat = i2cMasterTransmitTimeout(&I2CD1, BNO055_ADDR, txbuf, 2, rxbuf, 0, tmo);
	i2cReleaseBus(&I2CD1);
}