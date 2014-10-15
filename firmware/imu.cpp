#include <ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pololu_imu/ArduinoImuSettings.h>

#include <Arduino.h>

#include <Wire.h>

#if GYRO_CONNECTED
#include <L3G.h>
#endif

#if COMPASS_CONNECTED
#include <LSM303.h>
#endif

ros::NodeHandle nh;

geometry_msgs::PolygonStamped msg;
geometry_msgs::Point32 mixed_data[DATA_LENGTH];

ros::Publisher data_pub("", &msg);

#if GYRO_CONNECTED
L3G gyro;
#endif
#if COMPASS_CONNECTED
LSM303 compass;
#endif

unsigned long timer;

int update_frequency;
unsigned long loop_rate;

void setup() {
  Wire.begin();
  
  nh.initNode();

  if (
#if GYRO_CONNECTED
      !gyro.init(GYRO_DEVICE_TYPE) ||
#endif
#if COMPASS_CONNECTED
      !compass.init(COMPASS_DEVICE_TYPE) ||
#endif 
      false) {
    //isFaulty = true;
  }
  //while (isFaulty);
  
#if GYRO_CONNECTED
  gyro.enableDefault();
#endif
#if COMPASS_CONNECTED
  compass.enableDefault();
  analogReference(INTERNAL);
#endif

  msg.polygon.points = mixed_data;
  msg.polygon.points_length = DATA_LENGTH;

  nh.advertise(data_pub);

  const char* publish_topic;
  if (!nh.getParam(PARAM_PUBLISH_TOPIC, (char**)&publish_topic))
    publish_topic = DEFAULT_PUBLISH_TOPIC;
  data_pub = ros::Publisher(publish_topic, &msg);


  if (!nh.getParam(PARAM_FRAME_ID, (char**)&msg.header.frame_id))
    msg.header.frame_id = DEFAULT_FRAME_ID;

  if (!nh.getParam(PARAM_FREQUENCY, &update_frequency))
    update_frequency = DEFAULT_FREQUENCY;
  loop_rate = 1000 / update_frequency;
}


void loop() {
  timer = millis();
  msg.header.stamp = nh.now();

#if GYRO_CONNECTED
  gyro.read();

  msg.polygon.points[GYROSCOPE].x = gyro.g.x;
  msg.polygon.points[GYROSCOPE].y = gyro.g.y;
  msg.polygon.points[GYROSCOPE].z = gyro.g.z;
#endif
#if COMPASS_CONNECTED
  compass.read();

  msg.polygon.points[ACCELEROMETER].x = compass.a.x;
  msg.polygon.points[ACCELEROMETER].y = compass.a.y;
  msg.polygon.points[ACCELEROMETER].z = compass.a.z;

  msg.polygon.points[MAGNETOMETER].x = compass.m.x;
  msg.polygon.points[MAGNETOMETER].y = compass.m.y;
  msg.polygon.points[MAGNETOMETER].z = compass.m.z;

  msg.polygon.points[TEMPERATURE].x = analogRead(TEMPERATURE_PIN);
#endif

  data_pub.publish(&msg);

  nh.spinOnce();

  msg.header.seq++;

  while ((millis() - timer) < loop_rate);
}
