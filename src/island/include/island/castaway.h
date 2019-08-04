#ifndef CASTAWAY_H_
#define CASTAWAY_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include "island/entity.h"

namespace island {
class Castaway : public Entity {
 public:
  Castaway(ros::NodeHandle* nodehandle);
  ~Castaway();
  void Run();

 private:
  ros::Subscriber shark_sub_;
  ros::Publisher castaway_pub_;
  geometry_msgs::PointStamped previous_pt_;
  bool is_antipodality_reached_ = false;

  void InitCastawayPos();
  void SharkPositionCallback(
      const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr);
  bool CalculateCastawayPosition();
  void MoveStraightForShore(const double& dist);
  void MoveAntipodal(const double& dist);
  void CheckCastawayAntipodalToShark(double eps = 0.06);
};
}  // namespace island

#endif  // CASTAWAY_H_