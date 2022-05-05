#include <iahrs_ros/iahrs_ros.h>



namespace iahrs_ros
{


    
iAHRSROS::iAHRSROS(const std::string& port, const unsigned int& baudrate, const double& comm_recv_timeout): _port(port), _baudrate(baudrate), _port_fd(-1), _comm_recv_timeout(comm_recv_timeout)
{
  _isInitialized = false;
}


iAHRSROS::~iAHRSROS()
{

}


bool iAHRSROS::initialize()
{
  if (!_isInitialized)
  {
    _isInitialized = iAHRSROS::_Open();
  }
  iAHRSROS::restart();
  return _isInitialized;
}


bool iAHRSROS::restart()
{
  std::string command = "rd\n";
  return iAHRSROS::_Send(command.c_str());
}


bool iAHRSROS::flashWrite()
{
  std::string command = "fw\n";
  return iAHRSROS::_Send(command.c_str());
}


bool iAHRSROS::setBaudrate(const int& baudrate)
{
  return iAHRSROS::setBaudrate232(baudrate) && iAHRSROS::setBaudrateUSB(baudrate);
}


bool iAHRSROS::setBaudrate232(const int& baudrate)
{
  std::string command = "b1=" + std::to_string(baudrate) + "\n";
  return iAHRSROS::_Send(command.c_str());
}


bool iAHRSROS::setBaudrateUSB(const int& baudrate)
{
  std::string command = "b2=" + std::to_string(baudrate) + "\n";
  return iAHRSROS::_Send(command.c_str());
}


bool iAHRSROS::readLinearAcceleration(geometry_msgs::Vector3& linear_acceleration)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "a\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    linear_acceleration.x = data[0];
    linear_acceleration.y = data[1];
    linear_acceleration.z = data[2];
    return true;
  }
  return false;
}


bool iAHRSROS::readAngularVelocity(geometry_msgs::Vector3& angular_velocity)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "g\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    angular_velocity.x = data[0];
    angular_velocity.y = data[1];
    angular_velocity.z = data[2];
    return true;
  }
  return false;
}


bool iAHRSROS::readMagneticField(geometry_msgs::Vector3& magnetic_field)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "m\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  magnetic_field.x = data[0];
  magnetic_field.y = data[1];
  magnetic_field.z = data[2];
  return true;
}


bool iAHRSROS::readLinearVelocity(geometry_msgs::Vector3& linear_velocity)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "v\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  linear_velocity.x = data[0];
  linear_velocity.y = data[1];
  linear_velocity.z = data[2];
  return true;
}


bool iAHRSROS::readOrientation(geometry_msgs::Quaternion& orientation)
{
  const int max_data = 4;
  double data[max_data];
  std::string command = "q\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  orientation.w = data[0];
  orientation.x = data[1];
  orientation.y = data[2];
  orientation.z = data[3];
  return true;
}


bool iAHRSROS::readPosition(geometry_msgs::Point& position)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "p\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  position.x = data[0];
  position.y = data[1];
  position.z = data[2];
  return true;
}


bool iAHRSROS::iAHRSROS::_Open()
{
  //ROS_INFO("Try to open serial: %s\n", _port);
  ROS_INFO_STREAM("Try to open serial: "<< _port << "\n");

  _port_fd = open(_port.c_str(), O_RDWR|O_NOCTTY); // |O_NDELAY);
  if (_port_fd < 0) {
    //ROS_ERROR("Error unable to open %s\n", _port);
    ROS_ERROR_STREAM("Error unable to open serial: " << _port << "\n");
    return false;
  }
  //ROS_INFO("%s open success\n", _port);
  ROS_INFO_STREAM(_port << " open success\n");

  struct termios tio;
  tcgetattr(_port_fd, &tio);
  cfmakeraw(&tio);
  tio.c_cflag = CS8|CLOCAL|CREAD;
  tio.c_iflag &= ~(IXON | IXOFF);
  cfsetspeed(&tio, iAHRSROS::_getBaud(_baudrate));
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  int err = tcsetattr(_port_fd, TCSAFLUSH, &tio);
  if (err != 0)
  {
    ROS_ERROR("Error tcsetattr() function return error\n");
    iAHRSROS::_Close();
    return false;
  }
  return true;
}


void iAHRSROS::_Close()
{
  if(_port_fd > 0)
  {
    close(_port_fd);
    _port_fd = -1;
  }
}


bool iAHRSROS::_Send(const char* command)
{
  int command_len = strlen(command);
  int n = write(_port_fd, command, command_len);
  if (n < 0) return false; else return true;
}


int iAHRSROS::_SendRecv(const char* command, double* returned_data, int data_length)
{
  char temp_buff[128];
  read(_port_fd, temp_buff, 128);

  int command_len = strlen(command);
  if (write(_port_fd, command, command_len) < 0) return -1;

  const int buff_size = 128;
  int  recv_len = 0;
  char recv_buff[buff_size + 1]; // buff size + EOS

  double time_start = ros::Time::now().toSec();

  while (recv_len < buff_size)
  {
    int n = read(_port_fd, recv_buff + recv_len, buff_size - recv_len);
    if (n < 0)
    {
      return -1;
    }
    else if (n == 0)
    {
      usleep(1000); // wait 1ms if no data received.
    }
    else if (n > 0)
    {
      recv_len += n;
      if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n')
      {
        break;
      }
    } // check EOL is \r or \n.

    double time_current = ros::Time::now().toSec();
    double dt = time_current - time_start;

    if (dt >= _comm_recv_timeout) break;
  }
  recv_buff[recv_len] = '\0';

  if (recv_len > 0)
  {
    if (recv_buff[0] == '!')
    {
			return -1;
    }
  } // check error returned.

  if (strncmp(command, recv_buff, command_len - 1) == 0)
  {
    if (recv_buff[command_len - 1] == '=')
    {
      int data_count = 0;
      char* p = &recv_buff[command_len];
      char* pp = NULL;

      for (int i = 0; i < data_length; i++)
      {
        if (p[0] == '0' && p[1] == 'x')
        { // hexadecimal
          returned_data[i] = strtol(p+2, &pp, 16);
          data_count++;
        }
        else
        {
          returned_data[i] = strtod(p, &pp);
          data_count++;
        }

        if (*pp == ',')
        {
          p = pp + 1;
        }
        else
        {
          break;
        }
      }
      return data_count;
    }
  } // check received and sent is matched.
  return 0;
}


int iAHRSROS::_getBaud(const int& baudrate)
{
  switch (baudrate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  case 460800:
    return B460800;
  case 500000:
    return B500000;
  case 576000:
    return B576000;
  case 921600:
    return B921600;
  case 1000000:
    return B1000000;
  case 1152000:
    return B1152000;
  case 1500000:
    return B1500000;
  case 2000000:
    return B2000000;
  case 2500000:
    return B2500000;
  case 3000000:
    return B3000000;
  case 3500000:
    return B3500000;
  case 4000000:
    return B4000000;
  default: 
    return -1;
  }
}


} // namespace iahrs_ros
