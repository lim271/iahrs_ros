#include <iahrs_ros/iahrs_ros.h>



namespace iahrs_ros
{


    
iAHRSROS::iAHRSROS(const std::string& port, const unsigned int& baudrate, const unsigned long& comm_recv_timeout): _port(port), _baudrate(baudrate), _port_fd(-1), _comm_recv_timeout(comm_recv_timeout)
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
  const int max_data = 10;
  double data[max_data];
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  usleep(1000000);
  return true;
}


bool iAHRSROS::setSyncMode(const int& hz)
{
  const int max_data = 10;
  double data[max_data];
  std::string command;
  command = "so=1\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  command = "sp=" + std::to_string(int(1000 / hz)) + "\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  command = "sd=0x9C\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) < 0)
  {
    return false;
  }
  return true;
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
    angular_velocity.x = data[0] * M_PI / 180.0;
    angular_velocity.y = data[1] * M_PI / 180.0;
    angular_velocity.z = data[2] * M_PI / 180.0;
    return true;
  }
  return false;
}


bool iAHRSROS::readMagneticField(geometry_msgs::Vector3& magnetic_field)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "m\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    magnetic_field.x = data[0] * 0.000001;
    magnetic_field.y = data[1] * 0.000001;
    magnetic_field.z = data[2] * 0.000001;
    return true;
  }
  return false;
}


bool iAHRSROS::readLinearVelocity(geometry_msgs::Vector3& linear_velocity)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "v\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    linear_velocity.x = data[0];
    linear_velocity.y = data[1];
    linear_velocity.z = data[2];
    return true;
  }
  return false;
}


bool iAHRSROS::readOrientation(geometry_msgs::Quaternion& orientation)
{
  const int max_data = 4;
  double data[max_data];
  std::string command = "q\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    orientation.w = data[0];
    orientation.x = data[1];
    orientation.y = data[2];
    orientation.z = data[3];
    return true;
  }
  return false;
}


bool iAHRSROS::readPosition(geometry_msgs::Point& position)
{
  const int max_data = 3;
  double data[max_data];
  std::string command = "p\n";
  if (iAHRSROS::_SendRecv(command.c_str(), data, max_data) == max_data)
  {
    position.x = data[0];
    position.y = data[1];
    position.z = data[2];
    return true;
  }
  return false;
}


bool iAHRSROS::readSyncData(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& magnetic_field, geometry_msgs::Quaternion& orientation)
{
  const int max_data = 13;
  double data[max_data];
  if (iAHRSROS::_Recv(data, max_data) == max_data)
  {
    linear_acceleration.x = data[0];
    linear_acceleration.y = data[1];
    linear_acceleration.z = data[2];
    angular_velocity.x = data[3] * M_PI / 180.0;
    angular_velocity.y = data[4] * M_PI / 180.0;
    angular_velocity.z = data[5] * M_PI / 180.0;
    magnetic_field.x = data[6] * 0.000001;
    magnetic_field.y = data[7] * 0.000001;
    magnetic_field.z = data[8] * 0.000001;
    orientation.w = data[9];
    orientation.x = data[10];
    orientation.y = data[11];
    orientation.z = data[12];
    return true;
  }
  return false;
}


bool iAHRSROS::_Open()
{
  ROS_INFO_STREAM("Try to open serial: "<< _port << "\n");

  _port_fd = open(_port.c_str(), O_RDWR|O_NOCTTY); // |O_NDELAY);
  if (_port_fd < 0) {
    ROS_ERROR_STREAM("Error unable to open serial: " << _port << "\n");
    return false;
  }
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
  if (_port_fd > 0)
  {
    close(_port_fd);
    _port_fd = -1;
  }
}


int iAHRSROS::_Send(const char* command)
{
  int command_len = strlen(command);
  if (write(_port_fd, command, command_len) < 0) return -1;
}


int iAHRSROS::_Recv(double* returned_data, int data_length)
{
  memset(_buffer, 0, _buff_size);
  int  recv_len = 0;

  unsigned long time_start = iAHRSROS::_getTickCount();

  while (recv_len < _buff_size - 1)
  {
    int n = read(_port_fd, _buffer + recv_len, _buff_size - 1 - recv_len);
    if (n < 0)
    {
      return -1;
    }
    else if (n == 0)
    {
      usleep(100); // wait 100us if no data received.
    }
    else if (n > 0)
    {
      recv_len += n;
      if (_buffer[recv_len - 1] == '\r' || _buffer[recv_len - 1] == '\n')
      {
        break;
      }
    } // check EOL is \r or \n.

    if (iAHRSROS::_getTickCount() - time_start >= _comm_recv_timeout) break;
  }
  _buffer[recv_len] = '\0';

  if (recv_len > 0)
  {
    if (_buffer[0] == '!')
    {
			return -1;
    }
  } // check error returned.

  int data_count = 0;
  char* p = &_buffer[0];
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


int iAHRSROS::_SendRecv(const char* command, double* returned_data, int data_length)
{
  read(_port_fd, _buffer, _buff_size);

  int command_len = strlen(command);
  if (write(_port_fd, command, command_len) < 0) return -1;

  memset(_buffer, 0, _buff_size);
  int  recv_len = 0;

  unsigned long time_start = iAHRSROS::_getTickCount();

  while (recv_len < _buff_size - 1)
  {
    int n = read(_port_fd, _buffer + recv_len, _buff_size - 1 - recv_len);
    if (n < 0)
    {
      return -1;
    }
    else if (n == 0)
    {
      usleep(100); // wait 100us if no data received.
    }
    else if (n > 0)
    {
      recv_len += n;
      if (_buffer[recv_len - 1] == '\r' || _buffer[recv_len - 1] == '\n')
      {
        break;
      }
    } // check EOL is \r or \n.

    if (iAHRSROS::_getTickCount() - time_start >= _comm_recv_timeout) break;
  }
  _buffer[recv_len] = '\0';

  if (recv_len > 0)
  {
    if (_buffer[0] == '!')
    {
			return -1;
    }
  } // check error returned.

  if (strncmp(command, _buffer, command_len - 1) == 0)
  {
    if (_buffer[command_len - 1] == '=')
    {
      int data_count = 0;
      char* p = &_buffer[command_len];
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


unsigned long iAHRSROS::_getTickCount()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);

  return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}


} // namespace iahrs_ros

