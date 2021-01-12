/**
Header file for Sensirion SHTC3
 */

#ifndef __DRIVERS_SHTC3_H__
#define __DRIVERS_SHTC3_H__
#endif

#include <stdint.h>


typedef struct{
	void *i2c_p;
    int i2c_addr;
    uint32_t serial;
}shtc3_driver_t;

//! SHTC3 default I2C address.
#define SHTC3_I2C_ADDR			0xE0

//! Max measurement time for high repeatability.
#define MEASUREMENT_DURATION_HIGH_MS	15

/**
 * Initialize driver object. The driver object will be used for a
 * single sensor.
 *
 * @param[out] self_p Driver object to be initialize.
 * @param[in] i2c_p The I2C driver pointer.
 * @param[in] i2c_addr The address of the SHTC3.
 *                     Probably SHTC3_I2C_ADDR.
 *
 * @return zero(0) or negative error code.
 */
int shtc3_init(shtc3_driver_t *self_p, void *i2c_p, int i2c_addr);

/**
 * Get measurements and return it from the SHTC3 chip.
 *
 * This is a "high level" function which will block for the time it
 * takes the sensor to perform the measurement.
 *
 * @param[in] self_p Driver object.
 * @param[out] temp_p Tempererature in Celsius, or NULL.
 * @param[out] humid_p Relative Humidity, or NULL.
 *
 * @return zero(0) or negative error code.
 */
int shtc3_get_temp_humid(shtc3_driver_t *self_p, float *temp_p, float *humid_p);


