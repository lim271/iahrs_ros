#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iahrs_ros/iahrs_ros.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "iahrs");
  ros::NodeHandle nh("~");

  std::string port, frame;
  int baud_rate, hz;
  int comm_recv_timeout;
  bool sync_mode;
  double roll_init, pitch_init, yaw_init;

  nh.param("port",  port,  std::string("/dev/ttyUSB0"));
  nh.param("frame", frame, std::string("iahrs"));
  nh.param("baud_rate", baud_rate, 115200);
  nh.param("loop_rate", hz, 40);
  nh.param("comm_recv_timeout", comm_recv_timeout, 30);
  nh.param("sync_mode", sync_mode, true);
  nh.param("roll_init",  roll_init,  0.0);
  nh.param("pitch_init", pitch_init, 0.0);
  nh.param("yaw_init",   yaw_init,   0.0);

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

  sensor.setInitialOrientation(roll_init, pitch_init, yaw_init);
  ros::Subscriber orientation_sub = nh.subscribe("orientation_init", 1, &iahrs_ros::iAHRSROS::initialOrientationCallback, &sensor);

  if (!sensor.initialize())
  {
    ROS_ERROR("Initialization failed.\n");
    return 0;
  }
  else
  {
    ROS_INFO("Initialization success.\n");
  }

  if (sync_mode)
  {
    sensor.setSyncMode(hz);
  }

  while (ros::ok())
  {
    if (sync_mode)
    {
      sensor.readSyncData(linear_acceleration, angular_velocity, magnetic_field, orientation);
      imu_msg.header.stamp = mag_msg.header.stamp = ros::Time::now();
    }
    else{
      sensor.readLinearAcceleration(linear_acceleration);
      sensor.readAngularVelocity(angular_velocity);
      sensor.readOrientation(orientation);
      imu_msg.header.stamp = ros::Time::now();

      sensor.readMagneticField(magnetic_field);
      mag_msg.header.stamp = ros::Time::now();
    }
    imu_msg.orientation = orientation;
    imu_msg.angular_velocity = angular_velocity;
    imu_msg.linear_acceleration = linear_acceleration;
    mag_msg.magnetic_field = magnetic_field;

    imu_pub.publish(imu_msg);
    mag_pub.publish(mag_msg);

    if (!sync_mode)
    {
      loop_rate.sleep();
    }
  }

  return 0;
}
