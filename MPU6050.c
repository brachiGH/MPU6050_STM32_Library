#include "MPU6050.h"

bool _mpu6050_registerWriteBits(const mpu6050_handle *mpuh, uint8_t addr, uint8_t data, uint8_t _bits, uint8_t _shift)
{
  // see register map page 41
  uint8_t val = mpu6050_registerRead(mpuh, addr);
  if (val == -1)
  {
    return false;
  }

  // mask off the data before writing
  uint8_t mask = (1 << (_bits)) - 1;
  data &= mask;

  mask <<= _shift;
  val &= ~mask;          // remove the current data at that spot
  val |= data << _shift; // and add in the new data

  if (mpu6050_write(mpuh, &val, addr, 1) != HAL_OK)
  {
    return false;
  };

  return true;
}

uint8_t _mpu6050_registerReadBits(const mpu6050_handle *mpuh, uint8_t addr, uint8_t _bits, uint8_t _shift)
{
  uint8_t val = mpu6050_registerRead(mpuh, addr);
  if (val == -1)
    return -1;

  val >>= _shift;
  return val & ((1 << (_bits)) - 1);
}

/*!
 * @brief Resets the MPU6050 device.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return True if the reset was successful, otherwise false.
 */
bool mpu6050_reset(const mpu6050_handle *mpuh)
{
  bool status = true;

  // see register map page 41
  status &= _mpu6050_registerWriteBits(mpuh, MPU6050_PWR_MGMT_1, 1, 1, 7);

  uint8_t n = 0;
  uint8_t val = 1;
  while (val == 1 && n < MPU6050_MAX_REST_POLL)
  { // check for the post reset value
    val = _mpu6050_registerReadBits(mpuh, MPU6050_PWR_MGMT_1, 1, 7);
    if (val == -1)
      return false;
    n++;
    HAL_Delay(1);
  }
  HAL_Delay(100);

  val = 0x7;
  if (mpu6050_write(mpuh, &val, MPU6050_SIGNAL_PATH_RESET, 1) != HAL_OK)
  {
    return false;
  };

  HAL_Delay(100);
  return true;
}

/*!
 * @brief Sets the sample rate divisor for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param divisor The sample rate divisor value.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setSampleRateDivisor(const mpu6050_handle *mpuh, uint8_t divisor)
{
  return mpu6050_write(mpuh, &divisor, MPU6050_SMPLRT_DIV, 1) == HAL_OK;
}

/*!
 * @brief Sets the digital low pass filter bandwidth for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param bandwidth The desired bandwidth value.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setFilterBandwidth(const mpu6050_handle *mpuh, mpu6050_bandwidth_t bandwidth)
{
  return _mpu6050_registerWriteBits(mpuh, MPU6050_CONFIG, bandwidth, 3, 0);
}

/*!
 * @brief Sets the gyroscope range for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param new_range The desired gyroscope range.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_setGyroRange(const mpu6050_handle *mpuh, mpu6050_gyro_range_t new_range)
{
  return _mpu6050_registerWriteBits(mpuh, MPU6050_GYRO_CONFIG, new_range, 2, 3);
}

/*!
    @brief Sets the accelerometer measurement range
    @param  new_range The new range to set. Must be a `mpu6050_accel_range_t`
*/
bool mpu6050_setAccelerometerRange(const mpu6050_handle *mpuh, mpu6050_accel_range_t new_range)
{
  return _mpu6050_registerWriteBits(mpuh, MPU6050_ACCEL_CONFIG, new_range, 2, 3);
}

bool _mpu6050_testConnection(const mpu6050_handle *mpuh)
{
  uint8_t n = 0;
  // make sure we're talking to the right chip
  while (mpu6050_registerRead(mpuh, MPU6050_WHO_AM_I) != MPU6050_DEVICE_ID)
  {
    n++;
    if (n >= MPU6050_MAX_READ_CHIP_ID_POLL)
      return false;
  }

  return true;
}

/*!
 * @brief Sets the accelerometer range for the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param new_range The desired accelerometer range.
 * @return True if the operation was successful, otherwise false.
 */
bool mpu6050_init(const mpu6050_handle *mpuh)
{
  bool status = true;

  // For boards with I2C bus power control, may need to delay to allow
  // MPU6050 to come up after initial power.
  HAL_Delay(10);

  // Verify the I2C connection.
  status &= _mpu6050_testConnection(mpuh);

  // reset
  status &= mpu6050_reset(mpuh);

  // Power management register write all 0's to wake up sensor
  status &= mpu6050_setSampleRateDivisor(mpuh, 0);

  status &= mpu6050_setFilterBandwidth(mpuh, MPU6050_BAND_21_HZ);

  status &= mpu6050_setGyroRange(mpuh, MPU6050_RANGE_500_DEG);

  status &= mpu6050_setAccelerometerRange(mpuh, MPU6050_RANGE_8_G);

  uint8_t val = 0x01;
  status &= mpu6050_write(mpuh, &val, MPU6050_PWR_MGMT_1, 1) == HAL_OK; // set clock config to PLL with Gyro X reference

  HAL_Delay(100);
  return status;
}

/*!
 * @brief Reads data from the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param readbuffer Buffer to store the read data.
 * @param MemAddress Memory address to read from.
 * @param len Number of bytes to read.
 * @return HAL status.
 */
HAL_StatusTypeDef mpu6050_readMem(const mpu6050_handle *mpuh, uint8_t *readbuffer, uint8_t MemAddress, uint8_t len)
{
  return HAL_I2C_Mem_Read(mpuh->hi2c, mpuh->address, MemAddress, 1, readbuffer, len, MPU6050_MAX_HAL_TIMEOUT);
}

/*!
 * @brief Writes data to the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param writebuffer Buffer containing the data to write.
 * @param MemAddress Memory address to write to.
 * @param len Number of bytes to write.
 * @return HAL status.
 */
HAL_StatusTypeDef mpu6050_write(const mpu6050_handle *mpuh, uint8_t *writebuffer, uint8_t MemAddress, uint8_t len)
{
  return HAL_I2C_Mem_Write(mpuh->hi2c, mpuh->address, MemAddress, 1, writebuffer, len, MPU6050_MAX_HAL_TIMEOUT);
}

/*!
 * @brief Performs a Register read from the MPU6050.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @param MemAddress Memory address to read from.
 * @return Read data as a 8-bit unsigned integer.
 */
uint8_t mpu6050_registerRead(const mpu6050_handle *mpuh, uint8_t MemAddress)
{
  uint8_t len = 1;
  uint8_t _buffer;

  HAL_StatusTypeDef status = mpu6050_readMem(mpuh, &_buffer, MemAddress, len);
  if (status != HAL_OK)
    return -1;

  return _buffer;
}

/*!
 * @brief Gets the current accelerometer range setting.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return Current accelerometer range.
 */
mpu6050_accel_range_t mpu6050_getAccelerometerRange(const mpu6050_handle *mpuh)
{
  return (mpu6050_accel_range_t)_mpu6050_registerReadBits(mpuh, MPU6050_ACCEL_CONFIG, 2, 3);
}

/*!
 * @brief Gets the current gyroscope range setting.
 *
 * @param mpuh Pointer to the mpu6050 handle.
 * @return Current gyroscope range.
 */
mpu6050_gyro_range_t mpu6050_getGyroRange(const mpu6050_handle *mpuh)
{
  return (mpu6050_gyro_range_t)_mpu6050_registerReadBits(mpuh, MPU6050_GYRO_CONFIG, 2, 3);
}

/**
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
void mpu6050_read(const mpu6050_handle *mpuh, float *temperature, float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ)
{
  uint8_t buffer[14];
  mpu6050_readMem(mpuh, buffer, MPU6050_ACCEL_OUT, 14);

  int16_t rawAccX = buffer[0] << 8 | buffer[1];
  int16_t rawAccY = buffer[2] << 8 | buffer[3];
  int16_t rawAccZ = buffer[4] << 8 | buffer[5];

  int16_t rawTemp = buffer[6] << 8 | buffer[7];

  int16_t rawGyroX = buffer[8] << 8 | buffer[9];
  int16_t rawGyroY = buffer[10] << 8 | buffer[11];
  int16_t rawGyroZ = buffer[12] << 8 | buffer[13];

  *temperature = (rawTemp / 340.0) + 36.53;

  mpu6050_accel_range_t accel_range = mpu6050_getAccelerometerRange(mpuh);

  float accel_scale = 1;
  if (accel_range == MPU6050_RANGE_16_G)
    accel_scale = 2048;
  else if (accel_range == MPU6050_RANGE_8_G)
    accel_scale = 4096;
  else if (accel_range == MPU6050_RANGE_4_G)
    accel_scale = 8192;
  else if (accel_range == MPU6050_RANGE_2_G)
    accel_scale = 16384;

  // setup range dependant scaling
  *accX = ((float)rawAccX) / accel_scale;
  *accY = ((float)rawAccY) / accel_scale;
  *accZ = ((float)rawAccZ) / accel_scale;

  mpu6050_gyro_range_t gyro_range = mpu6050_getGyroRange(mpuh);

  float gyro_scale = 1;
  if (gyro_range == MPU6050_RANGE_250_DEG)
    gyro_scale = 131;
  else if (gyro_range == MPU6050_RANGE_500_DEG)
    gyro_scale = 65.5;
  else if (gyro_range == MPU6050_RANGE_1000_DEG)
    gyro_scale = 32.8;
  else if (gyro_range == MPU6050_RANGE_2000_DEG)
    gyro_scale = 16.4;

  *gyroX = ((float)rawGyroX) / gyro_scale;
  *gyroY = ((float)rawGyroY) / gyro_scale;
  *gyroZ = ((float)rawGyroZ) / gyro_scale;
}