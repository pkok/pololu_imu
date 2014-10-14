#include <ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <Arduino.h>

#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

#include <pololu_imu/ArduinoImuSettings.h>

ros::NodeHandle nh;

geometry_msgs::PolygonStamped msg;
geometry_msgs::Point32 mixed_data[DATA_LENGTH];

ros::Publisher data_pub("", &msg);

L3G gyro;
LSM303 compass;

unsigned long timer;

int update_frequency;
unsigned long loop_rate;

void setup() {
  Wire.begin();
  
  nh.initNode();

  if (!gyro.init(GYRO_DEVICE_TYPE) || !compass.init(COMPASS_DEVICE_TYPE)) {
    //isFaulty = true;
  }
  //while (isFaulty);
  
  gyro.enableDefault();
  compass.enableDefault();
  analogReference(INTERNAL);

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

  gyro.read();
  compass.read();

  msg.polygon.points[ACCELEROMETER].x = compass.a.x;
  msg.polygon.points[ACCELEROMETER].y = compass.a.y;
  msg.polygon.points[ACCELEROMETER].z = compass.a.z;

  msg.polygon.points[GYROSCOPE].x = gyro.g.x;
  msg.polygon.points[GYROSCOPE].y = gyro.g.y;
  msg.polygon.points[GYROSCOPE].z = gyro.g.z;

  msg.polygon.points[MAGNETOMETER].x = compass.m.x;
  msg.polygon.points[MAGNETOMETER].y = compass.m.y;
  msg.polygon.points[MAGNETOMETER].z = compass.m.z;

  msg.polygon.points[TEMPERATURE].x = analogRead(TEMPERATURE_PIN);

  data_pub.publish(&msg);

  nh.spinOnce();

  msg.header.seq++;

  while ((millis() - timer) < loop_rate);
}
