// Interface to the Microstrain 3DM-GX3-25
// N. Michael

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <eigen3/Eigen/Geometry> 
// #include "pose_utils.h"

using namespace std;

#define REPLY_LENGTH 4

boost::asio::serial_port* serial_port = 0;
const char stop[3] = {'\xFA','\x75','\xB4'};
char mode[4] = {'\xD4','\xA3','\x47','\x00'};
unsigned char reply[REPLY_LENGTH];
std::string name;

static float extract_float(unsigned char* addr)
{
  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

static int extract_int(unsigned char* addr)
{
  int tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

bool validate_checksum(const unsigned char *data, unsigned short length)
{
  unsigned short chksum = 0;
  unsigned short rchksum = 0;

  for (unsigned short i = 0; i < length - 2; i++)
    chksum += data[i];

  rchksum = data[length - 2] << 8;
  rchksum += data[length - 1];

  return (chksum == rchksum);
}

inline void print_bytes(const unsigned char *data, unsigned short length)
{
  for (unsigned int i = 0; i < length; i++)
    printf("%2x ", data[i]);
  puts("");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_3dm_gx3");
  ros::NodeHandle n("~");

  name = ros::this_node::getName();

  std::string port;
  if (n.hasParam("port"))
    n.getParam("port", port);
  else
    {
      ROS_ERROR("%s: must provide a port", name.c_str());
      return -1;
    }

  boost::asio::io_service io_service;
  serial_port = new boost::asio::serial_port(io_service);

  try
    {
      serial_port->open(port);
    }
  catch (boost::system::system_error &error)
    {
      ROS_ERROR("%s: Failed to open port %s with error %s",
                name.c_str(), port.c_str(), error.what());
      return -1;
    }

  if (!serial_port->is_open())
    {
      ROS_ERROR("%s: failed to open serial port %s",
                name.c_str(), port.c_str());
      return -1;
    }

  int baud;
  n.param("baud", baud, 115200);

  string frame_id;
  n.param("frame_id", frame_id, string("imu"));

  double delay;
  n.param("delay", delay, 0.0);

  typedef boost::asio::serial_port_base sb;

  sb::baud_rate baud_option(baud);
  sb::flow_control flow_control(sb::flow_control::none);
  sb::parity parity(sb::parity::none);
  sb::stop_bits stop_bits(sb::stop_bits::one);

  serial_port->set_option(baud_option);
  serial_port->set_option(flow_control);
  serial_port->set_option(parity);
  serial_port->set_option(stop_bits);

  // Stop continous mode if it is running
  boost::asio::write(*serial_port, boost::asio::buffer(stop, 3));
  ROS_WARN("Wait 0.1s"); 
  ros::Duration(0.1).sleep();

  // Check the mode
  bool reInitFlag = false;
  boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
  boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
  if (!validate_checksum(reply, REPLY_LENGTH))
    {
      ROS_ERROR("%s: failed to get mode", name.c_str());
      if (serial_port->is_open())
        serial_port->close();
      reInitFlag = true;
    }

  if (reInitFlag)
  {
    ROS_WARN("In Re-Init");
    ros::Duration(0.1).sleep();
    try
      {
        serial_port->open(port);
      }
    catch (boost::system::system_error &error)
      {
        ROS_ERROR("%s: Failed to open port %s with error %s",
                  name.c_str(), port.c_str(), error.what());
        return -1;
      }
    if (!serial_port->is_open())
      {
        ROS_ERROR("%s: failed to open serial port %s",
                  name.c_str(), port.c_str());
        return -1;
      }
    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);
    // Check the mode
    boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
    boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
    if (!validate_checksum(reply, REPLY_LENGTH))
      {
        ROS_ERROR("%s: failed to get mode", name.c_str());
        if (serial_port->is_open())
          serial_port->close();
        return -1;
      }    
  }

  // If we are not in active mode, change it
  if (reply[2] != '\x01')
    {
      mode[3] = '\x01';
      boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
      boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
      if (!validate_checksum(reply, REPLY_LENGTH))
        {
          ROS_ERROR("%s: failed to set mode to active", name.c_str());
          if (serial_port->is_open())
            serial_port->close();
          return -1;
        }
    }

  // Set the continous preset mode
  const char preset[4] = {'\xD6','\xC6','\x6B','\xCC'};
  boost::asio::write(*serial_port, boost::asio::buffer(preset, 4));

  boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
  if (!validate_checksum(reply, REPLY_LENGTH))
    {
      ROS_ERROR("%s: failed to set continuous mode preset", name.c_str());
      if (serial_port->is_open())
        serial_port->close();
      return -1;
    }

  // Set the mode to continous output
  mode[3] = '\x02';
  boost::asio::write(*serial_port, boost::asio::buffer(mode, 4));
  boost::asio::read(*serial_port, boost::asio::buffer(reply, REPLY_LENGTH));
  if (!validate_checksum(reply, REPLY_LENGTH))
    {
      ROS_ERROR("%s: failed to set mode to continuous output", name.c_str());
      if (serial_port->is_open())
        serial_port->close();
      return -1;
    }

  // Set Timer
  char set_timer[8] = {'\xD7','\xC1','\x29','\x01','\x00','\x00','\x00','\x00'};
  unsigned char reply_timer[7];
  boost::asio::write(*serial_port, boost::asio::buffer(set_timer, 8));
//  ros::Time t0 = ros::Time::now();  
  boost::asio::read(*serial_port, boost::asio::buffer(reply_timer, 7));
  ros::Time t0 = ros::Time::now();  

  ROS_WARN("Streaming Data...");
  unsigned short data_length = 79;
  unsigned char data[data_length];
  sensor_msgs::Imu msg;
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  while (n.ok())
    {
      boost::asio::read(*serial_port, boost::asio::buffer(data, data_length));
      if (!validate_checksum(data, data_length))
        {
          ROS_ERROR("%s: checksum failed on message", name.c_str());
          continue;
        }

      unsigned int k = 1;
      float acc[3];
      float ang_vel[3];
      float mag[3];
      float M[9];
      double T;
      for (unsigned int i = 0; i < 3; i++, k += 4)
        acc[i] = extract_float(&(data[k]));
      for (unsigned int i = 0; i < 3; i++, k += 4)
        ang_vel[i] = extract_float(&(data[k]));
      for (unsigned int i = 0; i < 3; i++, k += 4)
        mag[i] = extract_float(&(data[k]));
      for (unsigned int i = 0; i < 9; i++, k += 4)
        M[i] = extract_float(&(data[k]));
      T = extract_int(&(data[k])) / 62500.0;

      msg.header.stamp    = t0 + ros::Duration(T) - ros::Duration(delay);
      msg.header.frame_id = frame_id;
      msg.angular_velocity.x = ang_vel[0];
      msg.angular_velocity.y = ang_vel[1];
      msg.angular_velocity.z = ang_vel[2];
      msg.linear_acceleration.x = acc[0] * 9.81;
      msg.linear_acceleration.y = acc[1] * 9.81;
      msg.linear_acceleration.z = acc[2] * 9.81;
      // mat R(3,3);
      // for (unsigned int i = 0; i < 3; i++)
      //   for (unsigned int j = 0; j < 3; j++)
      //     R(i,j) = M[j*3+i];
      // colvec q = R_to_quaternion(R);
      Eigen::Matrix3d R;
      for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
          R(i,j) = M[j*3+i];
      Eigen::Quaternion<double> q(R);
      msg.orientation.w = (double)q.w();// q(0);
      msg.orientation.x = (double)q.x();// q(1);
      msg.orientation.y = (double)q.y();// q(2);
      msg.orientation.z = (double)q.z();// q(3);
      msg.orientation_covariance[0] = mag[0];
      msg.orientation_covariance[1] = mag[1];
      msg.orientation_covariance[2] = mag[2];
      pub.publish(msg);
    }

  // Stop continous and close device
  boost::asio::write(*serial_port, boost::asio::buffer(stop, 3));
  ROS_WARN("Wait 0.1s"); 
  ros::Duration(0.1).sleep();
  serial_port->close(); 

  return 0;
}
