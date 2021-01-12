/**
Source file for Sensirion SHTC3
 */

#include "shtc3.h"

/**
Measurements Commands Identification  */

#define CMD_NORMAL_CLKSTRETCH_TEMPFIRST		0x7CA2	//Normal Mode, Clock Stretching Enabled, Reading Temperature First
#define CMD_NORMAL_CLKSTRETCH_HUMIFIRST		0x5C24	//Normal Mode, Clock Stretching Enabled, Reading Humidity First
#define CMD_NORMAL_NOCLKSTRETCH_TEMPFIRST	0x7866	//Normal Mode, No Clock Stretching Enabled, Reading Temperature First
#define CMD_NORMAL_NOCLKSTRETCH_HUMIFIRST	0x58E0	//Normal Mode, No Clock Stretching Enabled, Reading Humidity First
#define CMD_LPOWER_CLKSTRETCH_TEMPFIRST		0x6458	//Normal Mode, Clock Stretching Enabled, Reading Temperature First
#define CMD_LPOWER_CLKSTRETCH_HUMIFIRST		0x44DE	//Normal Mode, Clock Stretching Enabled, Reading Humidity First
#define CMD_LPOWER_NOCLKSTRETCH_TEMPFIRST	0x609C	//Normal Mode, No Clock Stretching Enabled, Reading Temperature First
#define CMD_LPOWER_NOCLKSTRETCH_HUMIFIRST	0x401A	//Normal Mode, No Clock Stretching Enabled, Reading Humidity First
#define CMD_GETSERIAL						0xEFC8	//ID Register

/**
 * Send a single 16bit command to the SHTC3 device.
 */
static int shtc3_sendcmd(shtc3_driver_t *self_p, uint16_t cmd)
{
    uint8_t i2ccmd[2];
    i2ccmd[0] = (cmd >> 8) & 0xff;
    i2ccmd[1] = cmd & 0xff;
    HAL_I2C_Master_Transmit(self_p->i2c_p, self_p->i2c_addr, i2ccmd, 2, 1000);
    return (0);
}

/**
 * Read 2x 16bit to the SHTC3 device.
 */
static int shtc3_read2x16(shtc3_driver_t *self_p, uint8_t *data_p)
{
    HAL_I2C_Master_Receive(self_p->i2c_p, self_p->i2c_addr, data_p, 6, 1000);
    return (0);
}

int shtc3_init(shtc3_driver_t *self_p, void *i2c_p, int i2c_addr)
{
    self_p->i2c_p = i2c_p;
    self_p->i2c_addr = i2c_addr;
    return (0);
}

/**
 * Calculate the temperature in Celsius from SHTC3 raw temperature.
 */
static float shtc3_calculate_temp_c(uint8_t msb, uint8_t lsb)
{
    uint16_t val;
    float temp;

    val = msb << 8 | lsb;
    temp = 175.0f * ((float)val / 65535.0f) - 45.0f;

    return (temp);
}

/**
 * Calculate the humidity from SHT3xD raw temperature.
 */
static float shtc3_calculate_humid(uint8_t msb, uint8_t lsb)
{
    uint16_t val;
    float humid;

    val = msb << 8 | lsb;
    humid = 100.0f * ((float)val / 65535.0f);

    return (humid);
}

int shtc3_get_temp_humid(shtc3_driver_t *self_p, float *temp_p, float *humid_p)
{
    uint8_t data[6];

    shtc3_sendcmd(self_p, CMD_NORMAL_CLKSTRETCH_TEMPFIRST);

    // We use max duration to avoid having to handle retry.
    HAL_Delay(MEASUREMENT_DURATION_HIGH_MS);

    shtc3_read2x16(self_p, data);

    if (temp_p != 0) {
        *temp_p = shtc3_calculate_temp_c(data[0], data[1]);
    }
    if (humid_p != 0) {
        *humid_p = shtc3_calculate_humid(data[3], data[4]);
    }

    return (0);
}


