#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>

#include "island/island_constants.h"
#include "island/shark.h"

namespace island {

Shark::Shark(ros::NodeHandle* nodehandle) : Entity(nodehandle) {
  ROS_INFO("Shark constructor.");
  castaway_sub_ = nh_.subscribe("castaway_position", 1,
                                &Shark::CastawayPositionCallback, this);
  shark_pub_ = nh_.advertise<geometry_msgs::PointStamped>("shark_position", 1);
  InitSharkPos();
};

Shark::~Shark(){};

void Shark::Swim() {
  ROS_INFO("Entering Swim()");
  ros::Rate loop_rate(LOOP_RATE);
  previous_pt_.header.stamp = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    if (!CalculateSharkPosition()) {
      ROS_ERROR("Calculation failure. Unable to proceed.");
      break;
    }
    shark_pub_.publish(shark_pos_);
    if (CheckTerminationCondition() != 0) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Exiting Swim(). Goodbye!");
  return;
}

void Shark::InitSharkPos() {
  shark_pos_.point.x = 1.0;
  shark_pos_.point.y = 0.0;
  return;
}

void Shark::CastawayPositionCallback(
    const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr) {
  castaway_pos_ = *pt_msg_ptr;
  return;
}

bool Shark::CalculateSharkPosition() {
  ROS_INFO("Calculating shark position.");

  double theta_castaway = atan2(castaway_pos_.point.y, castaway_pos_.point.x);
  double theta_shark = atan2(shark_pos_.point.y, shark_pos_.point.x);
  double alpha = theta_castaway - theta_shark;
  alpha = std::fmod(alpha + 180, 360) - 180;
  double d_time = CalcTimeDiffFromNow(previous_pt_.header.stamp);
  double d_dist = SHARK_SPEED * d_time;
  if (fabs(alpha) < d_dist) {
    d_dist = ISLAND_RADIUS * alpha;
  }
  std::copysign(d_dist, alpha);
  double d_theta = d_dist / ISLAND_RADIUS;

  shark_pos_.point.x = ISLAND_RADIUS * cos(theta_shark + d_theta);
  shark_pos_.point.y = ISLAND_RADIUS * sin(theta_shark + d_theta);

  shark_pos_.header.stamp = ros::Time::now();
  previous_pt_ = shark_pos_;

  // ROS_DEBUG_STREAM("Shark x,y " << shark_pos_.point.x << ", "
  //                               << shark_pos_.point.y);
  // ROS_DEBUG_STREAM("theta_castaway:" << theta_castaway);
  // ROS_DEBUG_STREAM("theta_shark:" << theta_shark);
  // ROS_DEBUG_STREAM("alpha:" << alpha);
  // ROS_DEBUG_STREAM("d_time:" << d_time);
  // ROS_DEBUG_STREAM("d_theta:" << d_theta);

  if (!IsPointOnCircleCircumference(shark_pos_)) {
    return false;
  }
  return true;
}

}  // namespace island
