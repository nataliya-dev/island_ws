#ifndef ENTITY_H_
#define ENTITY_H_

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include "island/island_constants.h"

namespace island {
class Entity {
 public:
  Entity(ros::NodeHandle *nodehandle);
  virtual ~Entity();

 protected:
  ros::NodeHandle nh_;
  geometry_msgs::PointStamped shark_pos_;
  geometry_msgs::PointStamped castaway_pos_;

  double GetDistanceFromCenter(const geometry_msgs::PointStamped &point);

  int CheckTerminationCondition(double eps = 0.01);

  bool IsPointOnCircleCircumference(const geometry_msgs::PointStamped &point,
                                    double eps = 0.01);

  bool IsPointInsideCircle(const geometry_msgs::PointStamped &point);

  bool IsSharkZeroPosition();

  double CalcTimeDiffFromNow(const ros::Time &t1);

  double GetDistanceBetweenTwoPts(const geometry_msgs::PointStamped &point1,
                                  const geometry_msgs::PointStamped &point2);
};
}  // namespace island

#endif  // ENTITY_H_