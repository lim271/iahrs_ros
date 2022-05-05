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



namespace iahrs_ros
{



class iAHRSROS
{

public:

  iAHRSROS(const std::string& port, const unsigned int& baud_rate, const double& comm_recv_timeout);


  ~iAHRSROS();


  bool initialize();


  bool restart();


  bool flashWrite();


  bool setBaudrate(const int& baudrate);


  bool setBaudrate232(const int& baudrate);


  bool setBaudrateUSB(const int& baudrate);


  bool readLinearAcceleration(geometry_msgs::Vector3& linear_acceleration);


  bool readAngularVelocity(geometry_msgs::Vector3& angular_velocity);


  bool readMagneticField(geometry_msgs::Vector3& magnetic_field);


  bool readLinearVelocity(geometry_msgs::Vector3& linear_velocity);


  bool readOrientation(geometry_msgs::Quaternion& orientation);


  bool readPosition(geometry_msgs::Point& position);

private:

  std::string _port;
  int _baudrate;
  int _port_fd;
  double _comm_recv_timeout;
  bool _isInitialized;


  bool _Open();


  void _Close();


  bool _Send(const char* command);


  int _SendRecv(const char* command, double* returned_data, int data_length);


  int _getBaud(const int& baudrate);

};



} // namespace iahrs_ros



#endif // _IAHRS_ROS_H_
