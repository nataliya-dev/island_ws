#include "island/entity.h"

namespace island {

Entity::Entity(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {}
Entity::~Entity() {}

double Entity::GetDistanceFromCenter(const geometry_msgs::PointStamped &point) {
  double dist = sqrt(pow(point.point.x, 2) + pow(point.point.y, 2));
  // ROS_DEBUG_STREAM("GetDistanceFromCenter " << dist);
  return dist;
}

int Entity::CheckTerminationCondition(double eps) {
  if (fabs(castaway_pos_.point.x - shark_pos_.point.x) < eps &&
      fabs(castaway_pos_.point.y - shark_pos_.point.y) < eps) {
    ROS_INFO("======= TERMINATION CONDITION =======!");
    ROS_INFO("The shark got dinner!");
    // ROS_DEBUG_STREAM("Shark x,y " << shark_pos_.point.x << ", "
    //                               << shark_pos_.point.y);
    // ROS_DEBUG_STREAM("Castaway x,y " << castaway_pos_.point.x << ", "
    //                                  << castaway_pos_.point.y);
    return 1;
  } else if (!IsPointInsideCircle(castaway_pos_)) {
    ROS_INFO("======= TERMINATION CONDITION =======!");
    ROS_INFO("The castaway has escaped!");
    // ROS_DEBUG_STREAM("Shark x,y " << shark_pos_.point.x << ", "
    //                               << shark_pos_.point.y);
    // ROS_DEBUG_STREAM("Castaway x,y " << castaway_pos_.point.x << ", "
    //                                  << castaway_pos_.point.y);
    return 2;
  }
  return 0;
}

bool Entity::IsPointOnCircleCircumference(
    const geometry_msgs::PointStamped &point, double eps) {
  if (fabs(GetDistanceFromCenter(point) - ISLAND_RADIUS) > eps) {
    ROS_ERROR("IsPointOnCircleCircumference = false ");
    ROS_ERROR_STREAM("Point x,y " << point.point.x << ", " << point.point.y);
    return false;
  }
  return true;
}

bool Entity::IsPointInsideCircle(const geometry_msgs::PointStamped &point) {
  if (GetDistanceFromCenter(point) >= ISLAND_RADIUS) {
    return false;
  }
  return true;
}

bool Entity::IsSharkZeroPosition() {
  if (shark_pos_.point.x == 0 && shark_pos_.point.y == 0) {
    return true;
  }
  return false;
}

double Entity::CalcTimeDiffFromNow(const ros::Time &t1) {
  ros::Time cur_time = ros::Time::now();
  ros::Duration duration = cur_time - t1;
  double d_time = duration.toSec();
  return d_time;
}

double Entity::GetDistanceBetweenTwoPts(
    const geometry_msgs::PointStamped &point1,
    const geometry_msgs::PointStamped &point2) {
  return sqrt(pow(point1.point.x - point2.point.x, 2) +
              pow(point1.point.y - point2.point.y, 2));
}

}  // namespace island