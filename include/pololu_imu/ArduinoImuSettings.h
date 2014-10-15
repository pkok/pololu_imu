#ifndef POLOLU_IMU_ARDUINO_IMU_SETTINGS_H
#define POLOLU_IMU_ARDUINO_IMU_SETTINGS_H

/* Constants regarding the Arduino's setup */
#define GYRO_DEVICE_TYPE    L3G_DEVICE_AUTO
#define GYRO_CONNECTED      true
#define COMPASS_DEVICE_TYPE LSM303::device_auto
#define COMPASS_CONNECTED   true
#define TEMPERATURE_PIN     3

/* ROS-related constants */
#define PARAM_PUBLISH_TOPIC "output"
#define PARAM_FREQUENCY     "freq"
#define PARAM_FRAME_ID      "freq"

#define DEFAULT_PUBLISH_TOPIC "arduino_imu"
#define DEFAULT_FREQUENCY     200 /*Hz*/
#define DEFAULT_FRAME_ID      "/base_imu"

/* Constants for access to the published Polygon, to reorder the 
 * data into more appropriate datatypes on a bigger device. 
 */
#if GYRO_CONNECTED && COMPASS_CONNECTED
#define GYROSCOPE (unsigned)     0
#define ACCELEROMETER (unsigned) 1
#define MAGNETOMETER (unsigned)  2
#define TEMPERATURE (unsigned)   3
#define DATA_LENGTH (unsigned)   4
#elif GYRO_CONNECTED
#define GYROSCOPE (unsigned)     0
#define DATA_LENGTH (unsigned)   1
#elif COMPASS_CONNECTED
#define ACCELEROMETER (unsigned) 0
#define MAGNETOMETER (unsigned)  1
#define TEMPERATURE (unsigned)   2
#define DATA_LENGTH (unsigned)   3
#else
#define DATA_LENGTH (unsigned)   0
#endif

#endif
