#ifndef SHARK_H_
#define SHARK_H_

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include "island/entity.h"

namespace island {
class Shark : public Entity {
 public:
  Shark(ros::NodeHandle* nodehandle);
  ~Shark();
  void Swim();

 private:
  ros::Subscriber castaway_sub_;
  ros::Publisher shark_pub_;
  geometry_msgs::PointStamped previous_pt_;

  void InitSharkPos();
  void CastawayPositionCallback(
      const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr);
  bool CalculateSharkPosition();
};

}  // namespace island

#endif  // SHARK_H_