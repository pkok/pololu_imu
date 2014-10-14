#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <pololu_imu/ArduinoImuSettings.h>

ros::Publisher imu_pub;
ros::Publisher magnetic_pub;
ros::Publisher temperature_pub;

ros::Subscriber arduino_sub;

void arduinoCallback(const geometry_msgs::PolygonStampedConstPtr& msg) {
  sensor_msgs::Imu imu_msg;
  sensor_msgs::MagneticField magnetic_msg;
  sensor_msgs::Temperature temperature_msg;

  geometry_msgs::Point32 data;

  imu_msg.header = msg->header;
  magnetic_msg.header = msg->header;
  temperature_msg.header = msg->header;

  imu_msg.orientation_covariance[0] = -1;

  data = msg->polygon.points[ACCELEROMETER];
  imu_msg.linear_acceleration.x = data.x;
  imu_msg.linear_acceleration.y = data.y;
  imu_msg.linear_acceleration.z = data.z;

  data = msg->polygon.points[GYROSCOPE];
  imu_msg.angular_velocity.x = data.x;
  imu_msg.angular_velocity.y = data.y;
  imu_msg.angular_velocity.z = data.z;

  data = msg->polygon.points[MAGNETOMETER];
  magnetic_msg.magnetic_field.x = data.x;
  magnetic_msg.magnetic_field.y = data.y;
  magnetic_msg.magnetic_field.z = data.z;

  data = msg->polygon.points[TEMPERATURE];
  temperature_msg.temperature = data.x;

  imu_pub.publish(imu_msg);
  magnetic_pub.publish(magnetic_msg);
  temperature_pub.publish(temperature_msg);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pololu_imu");
  ros::NodeHandle nh("~");

  imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);
  magnetic_pub = nh.advertise<sensor_msgs::MagneticField>("magnetic", 1000);
  temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 1000);

  std::string arduino_topic;
  nh.param(std::string(PARAM_PUBLISH_TOPIC), arduino_topic, std::string(DEFAULT_PUBLISH_TOPIC));
  arduino_sub = nh.subscribe(arduino_topic, 1, arduinoCallback);

  ros::spin();
  return EXIT_SUCCESS;
}
