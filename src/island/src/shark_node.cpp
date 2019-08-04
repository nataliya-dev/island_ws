#include <iostream>

#include <ros/ros.h>

#include <island/shark.h>

int main(int argc, char** argv) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "shark_node");
  ros::NodeHandle nh;
  island::Shark shark(&nh);
  shark.Swim();
  return 0;
}