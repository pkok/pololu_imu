cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

## Only use this for an Arduino with native USB (Leonardo)
# add_definitions(-DUSB_CON)

set(l3g_basedir "${PROJECT_SOURCE_DIR}/l3g-arduino/L3G")
set(lsm303_basedir "${PROJECT_SOURCE_DIR}/lsm303-arduino/LSM303")

## Generate the required libraries for the sensors

generate_arduino_library(l3g
  BOARD uno
  SRCS ${l3g_basedir}/L3G.cpp
  HDRS ${l3g_basedir}/L3G.h
)

generate_arduino_library(lsm303
  BOARD uno
  SRCS ${lsm303_basedir}/LSM303.cpp
  HDRS ${lsm303_basedir}/LSM303.h
)

include_directories(
  ${l3g_basedir}
  ${lsm303_basedir}
  ../include
)

## Generate the firmware image for the Arduino board
generate_arduino_firmware(imu
  BOARD uno
  SRCS imu.cpp ${ROS_LIB_DIR}/time.cpp
  LIBS l3g lsm303
  PORT /dev/ttyACM0
)
