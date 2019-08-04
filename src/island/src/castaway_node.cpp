#include <iostream>

#include <ros/console.h>
#include <ros/ros.h>

#include <island/castaway.h>

int main(int argc, char** argv) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "castaway_node");
  ros::NodeHandle nh;
  island::Castaway castaway(&nh);
  castaway.Run();
  return 0;
}