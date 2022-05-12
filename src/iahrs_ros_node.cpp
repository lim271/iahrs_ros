#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iahrs_ros/iahrs_ros.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "iahrs");
  ros::NodeHandle nh("~");

  std::string port, frame;
  int baud_rate, hz;
  double comm_recv_timeout;

  nh.param("port", port, std::string("/dev/ttyUSB0"));
  nh.param("frame", frame, std::string("iahrs"));
  nh.param("baud_rate", baud_rate, 115200);
  nh.param("loop_rate", hz, 15);
  nh.param("comm_recv_timeout", comm_recv_timeout, 0.2);

  geometry_msgs::Vector3 linear_acceleration, angular_velocity, magnetic_field;
  geometry_msgs::Quaternion orientation;

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = frame;
  imu_msg.orientation_covariance[0] = 0.013 * M_PI / 180.0;
  imu_msg.orientation_covariance[4] = 0.011 * M_PI / 180.0;
  imu_msg.orientation_covariance[8] = 0.006 * M_PI / 180.0;
  imu_msg.linear_acceleration_covariance[0] = 0.0064;
  imu_msg.linear_acceleration_covariance[4] = 0.0063;
  imu_msg.linear_acceleration_covariance[8] = 0.0064;
  imu_msg.angular_velocity_covariance[0] = 0.032 * M_PI / 180.0;
  imu_msg.angular_velocity_covariance[4] = 0.028 * M_PI / 180.0;
  imu_msg.angular_velocity_covariance[8] = 0.006 * M_PI / 180.0;

  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.frame_id = frame;
  mag_msg.magnetic_field_covariance[0] =
  mag_msg.magnetic_field_covariance[4] =
  mag_msg.magnetic_field_covariance[8] = 1e-8;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);
  ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

  ros::Rate loop_rate(hz);

  iahrs_ros::iAHRSROS sensor(port, baud_rate, comm_recv_timeout);

  if(!sensor.initialize())
  {
    ROS_ERROR("Initialization failed.\n");
    return 0;
  }
  else
  {
    ROS_INFO("Initialization success.\n");
  }


  while(ros::ok())
  {
    ros::Time current_time = ros::Time::now();

    sensor.readLinearAcceleration(linear_acceleration);
    sensor.readAngularVelocity(angular_velocity);
    sensor.readOrientation(orientation);

    imu_msg.header.stamp = current_time;
    imu_msg.orientation = orientation;
    imu_msg.angular_velocity = angular_velocity;
    imu_msg.linear_acceleration = linear_acceleration;
    imu_pub.publish(imu_msg);

    sensor.readMagneticField(magnetic_field);

    mag_msg.header.stamp = current_time;
    mag_msg.magnetic_field = magnetic_field;
    mag_pub.publish(mag_msg);

    loop_rate.sleep();
  }

  return 0;
}
