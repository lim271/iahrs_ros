#ifndef _IAHRS_ROS_H_
#define _IAHRS_ROS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <exception>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iahrs_ros/gp.h>



namespace iahrs_ros
{



class iAHRSROS
{

public:

  iAHRSROS(const std::string& port, const unsigned int& baud_rate, const unsigned long& comm_recv_timeout);


  ~iAHRSROS();


  bool initialize();


  bool restart();


  bool setSyncMode(const int& hz);


  bool setBaudrate(const int& baudrate);


  void setInitialOrientation(const double& roll, const double& pitch, const double& yaw);


  void setInitialOrientation(const geometry_msgs::Quaternion& orientation);


  void setCalibration(const bool& calib);


  void initialOrientationCallback(const geometry_msgs::Quaternion& msg);


  bool readLinearAcceleration(geometry_msgs::Vector3& linear_acceleration);


  bool readAngularVelocity(geometry_msgs::Vector3& angular_velocity);


  bool readMagneticField(geometry_msgs::Vector3& magnetic_field);


  bool readLinearVelocity(geometry_msgs::Vector3& linear_velocity);


  bool readOrientation(geometry_msgs::Quaternion& orientation);


  bool readPosition(geometry_msgs::Point& position);


  bool readSyncData(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& magnetic_field, geometry_msgs::Quaternion& orientation);


  GaussianProcess gp;

private:

  std::string _port;
  int _baudrate;
  int _port_fd;
  //double _comm_recv_timeout;
  unsigned long _comm_recv_timeout;
  bool _isInitialized, _calib;
  char _buffer[1024];
  const int _buff_size = 1024;
  tf2::Quaternion _quat_init;


  bool _Open();


  void _Close();


  int _Send(const char* command);


  int _Recv(double* returned_data, int data_length);


  int _SendRecv(const char* command, double* returned_data, int data_length);


  int _getBaud(const int& baudrate);


  unsigned long _getTickCount();

};



} // namespace iahrs_ros



#endif // _IAHRS_ROS_H_
