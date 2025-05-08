/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/io/serial_port.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/Setpoints.h>
// #include <cola2_msgs/srv/cam_trigger.hpp>
#include <stm32/Intensitylightcamfrequency.h>
#include <stm32/stm32.h>
//  #include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include "ros/ros.h"

// Assuming STM32 and cola2::io::SPConfig are defined elsewhere
// Replace with actual header includes as needed
class STM32; // Forward declaration
namespace cola2
{
  namespace io
  {
    struct SPConfig;
  }
}
// using namespace std;

class STM32node
{
private:
  // Pointer to the driver that does the work
  std::shared_ptr<STM32> stm32_;

  // Config struct
  struct
  {
    cola2::io::SPConfig sp_config;
    std::string frame_id;
  } config_;

  // Publishers and Subscribers
  ros::Subscriber sub_setpoints_;
  ros::ServiceServer service_camTrigger_;

  std::string fillWithZeros(int number, int width);

  uint8_t normalizeTo255(double value);

  uint8_t normalizeTo255ForMotors(double value);

  std::string intToString(int value);

public:
  STM32node();

  void init();

  bool handle_cam_trigger(stm32::Intensitylightcamfrequency::Request &request,
                          stm32::Intensitylightcamfrequency::Response &response);

  void setpoints_callback(const cola2_msgs::Setpoints &msg);

  void getConfig();

  bool isDeviceConnected();

  void reconnect();
};

STM32node::STM32node()
{
}

void STM32node::init()
{
  std::cout << "Started node.init() initialisation" << std::endl;
  getConfig();

  ros::NodeHandle nh;

  // Define publishers
  sub_setpoints_ = nh.subscribe("/stm32/Setpoints", 10, &STM32node::setpoints_callback, this);

  service_camTrigger_ = nh.advertiseService("cam_trigger_service", &STM32node::handle_cam_trigger, this);

  // Init stm32 connection and send configuration
  bool done = false;
  while (!done && ros::ok())
  {
    try
    {
      stm32_ = std::shared_ptr<STM32>(new STM32());

      stm32_->init(config_.sp_config);
      ROS_INFO("configuring stm32 device...");

      // done
      done = true;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR("Error setting up the serial port: %s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  // Show message
  ROS_INFO("initialized.");
}

bool STM32node::isDeviceConnected()
{
  int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1)
  {
    ROS_ERROR("El dispositivo STM32 no está disponible (no se puede abrir /dev/ttyACM0).");
    return false;
  }

  close(fd);
  return true;
}

void STM32node::reconnect()
{
  try
  {
    // Destruir el objeto anterior y crear uno nuevo
    stm32_.reset(); // Libera el puntero antiguo (destruye la conexión anterior)

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Espera 1 segundo para que el SO detecte el puerto nuevamente

    stm32_ = std::make_shared<STM32>(); // Crea un nuevo objeto STM32
    stm32_->init(config_.sp_config);    // Intenta inicializarlo de nuevo

    ROS_INFO("STM32 reconectado exitosamente.");
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Fallo en la reconexión al STM32: %s", e.what());
  }
}

std::string STM32node::fillWithZeros(int number, int width)
{
  std::ostringstream oss;
  oss << std::setw(width) << std::setfill('0') << number;
  return oss.str();
}

uint8_t STM32node::normalizeTo255(double value)
{
  // Check if the value is within the valid range
  if (value <= 0.0 || value >= 1.0)
  {
    std::cerr << "Error: Value must be between 0 and 1." << std::endl;
    return 0; // Return 0 in case of error
  }

  // Normalize the value to the range 0-255
  return static_cast<uint8_t>(value * 255.0);
}

uint8_t STM32node::normalizeTo255ForMotors(double value)
{
  // Check if the value is within the valid range
  if (value < -1.0 || value > 1.0)
  {
    std::cerr << "Error: Value must be between -1 and 1." << std::endl;
    return 0; // Return 0 in case of error
  }

  // Normalize the value to the range 0-255
  uint8_t normalizedvalue = static_cast<uint8_t>((value + 1) * (255) / 2);

  return normalizedvalue;
}

std::string STM32node::intToString(int value)
{
  return std::to_string(value);
}

bool STM32node::handle_cam_trigger(stm32::Intensitylightcamfrequency::Request &request,
                                   stm32::Intensitylightcamfrequency::Response &response)
{
  if (request.intensity < 0 || request.intensity > 1) // checking that the intensity request for the lights is within bounds
  {
    response.success = false;
    return true;
  }
  else
  {
    std::cout << "Sending the camera and light values..." << std::endl;
    std::string checksum = fillWithZeros((request.camfrec + normalizeTo255(request.intensity)), 3);
    std::string msg = "cc:" + intToString(request.camfrec) + ":";
    msg = msg + fillWithZeros(normalizeTo255(request.intensity), 3) + ":" + checksum + ":ee";
    stm32_->write(msg);
    std::cout << msg << std::endl;
    ROS_INFO("%s", msg.c_str());
    response.success = true;
    return true;
  }
}

void STM32node::setpoints_callback(const cola2_msgs::Setpoints &msg)
{
  std::string string_to_send = "aa:";
  int checksum = 0;

  // Process the setpoints array
  for (const auto &setpoint : msg.setpoints)
  {
    if (setpoint >= -1.0 && setpoint <= 1.0)
    {
      int normalized_setpoint = normalizeTo255ForMotors(setpoint);
      checksum += normalized_setpoint;
      string_to_send = string_to_send + fillWithZeros(normalized_setpoint, 3) + ":";
      std::cout << "Sending Thruster setpoints: "<< normalized_setpoint << std::endl;
    }
    else
    {
      ROS_INFO("Received Setpoints not valid");
      return;
    }
  }
  string_to_send = string_to_send + fillWithZeros(checksum, 4);
  string_to_send = string_to_send + ":ee";
  ROS_INFO("%s", string_to_send.c_str());
  stm32_->write(string_to_send);
}

void STM32node::getConfig()
{
  ros::NodeHandle nh("~");
  std::cout << "Trying to connect to the stm port..." << std::endl;
  // Port for testing do in terminal before run the code: sudo chmod 777 /dev/tty0
  // nh.param<std::string>("serial_port/path", config_.sp_config.sp_path, "/dev/tty0");

  // Official serial port:
  nh.param<std::string>("serial_port/path", config_.sp_config.sp_path, "/dev/ttyACM0");
  nh.param<int>("serial_port/baud_rate", config_.sp_config.sp_baud_rate, 9600);
  nh.param<int>("serial_port/char_size", config_.sp_config.sp_char_size, 8);
  nh.param<int>("serial_port/stop_bits", config_.sp_config.sp_stop_bits, 1);
  nh.param<std::string>("serial_port/parity", config_.sp_config.sp_parity, "NONE");
  nh.param<std::string>("serial_port/flow_control", config_.sp_config.sp_flow_control, "NONE");
  nh.param<int>("serial_port/timeout", config_.sp_config.sp_timeout, 10000);
  nh.param<std::string>("frame_id", config_.frame_id, "STM32");
  std::cout<<"Heyyyy!!!"<<std::endl;
  // Append robot name to frame id to meet the TF tree
  std::string ns = ros::this_node::getNamespace();
  if (ns.length() > 1 && ns[0] == '/')
    ns = ns.substr(1);
  config_.frame_id = ns + "/" + config_.frame_id;
}

int main(int argc, char **argv)
{
  std::cout << "NODE HAS BEEN STARTED!!!!" << std::endl;
  ros::init(argc, argv, "stm32_node");
  STM32node node;
  node.init();
  ros::Rate rate(1);

  while (ros::ok())
  {
    if (node.isDeviceConnected())
    {
      std::cout<<"Device connected!!!"<<std::endl;
      ros::spinOnce();
    }
    else
    {
      ROS_ERROR("STM32 desconectado. Intentando reconectar...");
      node.reconnect();
      ros::Duration(0.1).sleep();
    }
    rate.sleep();
  }

  return 0;
}