#ifndef _MPU6050_H_
#define _MPU5060_H_

#include <stdbool.h>
#include <stdint.h>

#if __has_include("stm32f4xx_hal.h")
#include "stm32f4xx_hal.h"
#elif __has_include("stm32f1xx_hal.h")
#include "stm32f1xx_hal.h"
#elif __has_include("stm32c0xx_hal.h")
#include "stm32c0xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#include "stm32g4xx_hal.h"
#endif

#define MPU6050_ADDR 0xD0

#define SENSORS_GRAVITY_EARTH (9.80665F)   // Earth's gravity in m/s^2
#define SENSORS_DPS_TO_RADS (0.017453293F) //  Degrees/s to rad/s multiplier

#define MPU6050_LSBFIRST 0
#define MPU6050_MSBFIRST 1

#define MPU6050_I2CADDR_DEFAULT 0x68   // MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID 0x70         // The correct MPU6050_WHO_AM_I value
#define MPU6050_WHO_AM_I 0x75          // Divice ID register
#define MPU6050_PWR_MGMT_1 0x6B        // Primary power/sleep control register
#define MPU6050_SIGNAL_PATH_RESET 0x68 // Signal path reset register
#define MPU6050_SMPLRT_DIV 0x19        // sample rate divisor register
#define MPU6050_CONFIG 0x1A            // General configuration register
#define MPU6050_GYRO_CONFIG 0x1B       // Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C      // Accelerometer specific configration register
#define MPU6050_ACCEL_OUT 0x3B         // base address for sensor data reads

#define MPU6050_MAX_REST_POLL 127         // max 254
#define MPU6050_MAX_READ_CHIP_ID_POLL 254 // max 254
#define MPU6050_MAX_HAL_TIMEOUT 500

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t address;
} mpu6050_handle;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum
{
  MPU6050_BAND_260_HZ, // Docs imply this disables the filter
  MPU6050_BAND_184_HZ, // 184 Hz
  MPU6050_BAND_94_HZ,  // 94 Hz
  MPU6050_BAND_44_HZ,  // 44 Hz
  MPU6050_BAND_21_HZ,  // 21 Hz
  MPU6050_BAND_10_HZ,  // 10 Hz
  MPU6050_BAND_5_HZ,   // 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum
{
  MPU6050_RANGE_250_DEG,  // +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG,  // +/- 500 deg/s
  MPU6050_RANGE_1000_DEG, // +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG, // +/- 2000 deg/s
} mpu6050_gyro_range_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum
{
  MPU6050_RANGE_2_G = 0b00,  // +/- 2g (default value)
  MPU6050_RANGE_4_G = 0b01,  // +/- 4g
  MPU6050_RANGE_8_G = 0b10,  // +/- 8g
  MPU6050_RANGE_16_G = 0b11, // +/- 16g
} mpu6050_accel_range_t;

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address The I2C address to be used.
 *    @param  mpuh Pointer to the mpu6050 handle
 *    @return True if initialization was successful, otherwise false.
 */
bool mpu6050_init(const mpu6050_handle *mpuh);

/*!
 * @brief Resets the MPU6050 device.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return True if the reset was successful, otherwise false.
 */
bool mpu6050_reset(const mpu6050_handle *mpuh);

/*!
 * @brief Sets the sample rate divisor for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param divisor The sample rate divisor value.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setSampleRateDivisor(const mpu6050_handle *mpuh, uint8_t divisor);

/*!
 * @brief Sets the digital low pass filter bandwidth for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param bandwidth The desired bandwidth value.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setFilterBandwidth(const mpu6050_handle *mpuh, mpu6050_bandwidth_t bandwidth);

/*!
 * @brief Sets the gyroscope range for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param new_range The desired gyroscope range.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setGyroRange(const mpu6050_handle *mpuh, mpu6050_gyro_range_t new_range);

/*!
 * @brief Sets the accelerometer range for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param new_range The desired accelerometer range.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setAccelerometerRange(const mpu6050_handle *mpuh, mpu6050_accel_range_t new_range);

/*!
 * @brief Reads data from the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param readbuffer Buffer to store the read data.
 * @param MemAddress Memory address to read from.
 * @param len Number of bytes to read.
 * @return HAL status.
 */
HAL_StatusTypeDef mpu6050_readMem(const mpu6050_handle *mpuh, uint8_t *readbuffer, uint8_t MemAddress, uint8_t len);

/*!
 * @brief Writes data to the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param writebuffer Buffer containing the data to write.
 * @param MemAddress Memory address to write to.
 * @param len Number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef mpu6050_write(const mpu6050_handle *mpuh, uint8_t *writebuffer, uint8_t MemAddress, uint8_t len);

/*!
 * @brief Performs a Register read from the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param MemAddress Memory address to read from.
 * @return Read data as a 8-bit unsigned integer.
 */
uint8_t mpu6050_registerRead(const mpu6050_handle *mpuh, uint8_t MemAddress);

/*!
 * @brief Gets the current accelerometer range setting.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return Current accelerometer range.
 */
mpu6050_accel_range_t mpu6050_getAccelerometerRange(const mpu6050_handle *mpuh);

/*!
 * @brief Gets the current gyroscope range setting.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return Current gyroscope range.
 */
mpu6050_gyro_range_t mpu6050_getGyroRange(const mpu6050_handle *mpuh);

/*!
 * @brief Reads sensor data from the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param temperature Pointer to store the temperature value.
 * @param accX Pointer to store the X-axis accelerometer value.
 * @param accY Pointer to store the Y-axis accelerometer value.
 * @param accZ Pointer to store the Z-axis accelerometer value.
 * @param gyroX Pointer to store the X-axis gyroscope value.
 * @param gyroY Pointer to store the Y-axis gyroscope value.
 * @param gyroZ Pointer to store the Z-axis gyroscope value.
 */
void mpu6050_read(const mpu6050_handle *mpuh, float *temperature, float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ);

#endif
