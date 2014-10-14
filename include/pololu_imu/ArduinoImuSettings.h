#ifndef POLOLU_IMU_ARDUINO_IMU_SETTINGS_H
#define POLOLU_IMU_ARDUINO_IMU_SETTINGS_H

#define TEMPERATURE_PIN 3
#define GYRO_DEVICE_TYPE    L3G_DEVICE_AUTO
#define COMPASS_DEVICE_TYPE LSM303::device_auto

#define PARAM_PUBLISH_TOPIC "output"
#define PARAM_FREQUENCY "freq"
#define PARAM_FRAME_ID "freq"

#define DEFAULT_PUBLISH_TOPIC "arduino_imu"
#define DEFAULT_FREQUENCY 200 /*Hz*/
#define DEFAULT_FRAME_ID "/base_imu"

#define ACCELEROMETER (unsigned) 0
#define GYROSCOPE (unsigned) 1
#define MAGNETOMETER (unsigned) 2
#define TEMPERATURE (unsigned) 3
#define DATA_LENGTH (unsigned) 4

#endif
