#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>

#include "island/castaway.h"
#include "island/island_constants.h"

namespace island {

Castaway::Castaway(ros::NodeHandle* nodehandle) : Entity(nodehandle) {
  ROS_INFO("Castaway constructor.");
  shark_sub_ = nh_.subscribe("shark_position", 1,
                             &Castaway::SharkPositionCallback, this);
  castaway_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("castaway_position", 1);
  InitCastawayPos();
};

Castaway::~Castaway(){};

void Castaway::Run() {
  ROS_INFO("Entering Run()");
  ros::Rate loop_rate(LOOP_RATE);
  previous_pt_.header.stamp = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    if (IsSharkZeroPosition()) {
      castaway_pub_.publish(castaway_pos_);
      continue;
    }
    if (CheckTerminationCondition() != 0) {
      break;
    }
    if (!CalculateCastawayPosition()) {
      ROS_ERROR("Calculation failure. Unable to proceed.");
      break;
    }
    castaway_pub_.publish(castaway_pos_);

    double pt_diff = GetDistanceBetweenTwoPts(shark_pos_, castaway_pos_);
    double dist_from_shore =
        ISLAND_RADIUS - GetDistanceFromCenter(castaway_pos_);

    ROS_DEBUG_STREAM(
        "Current distance between shark and castaway: " << pt_diff);
    ROS_DEBUG_STREAM(
        "Current distance of castaway from shore: " << dist_from_shore);

    loop_rate.sleep();
  }
  ROS_INFO("Exiting Run(). Goodbye!");
  return;
}

void Castaway::InitCastawayPos() {
  castaway_pos_.point.x = 0.0;
  castaway_pos_.point.y = 0.0;
  return;
}

void Castaway::SharkPositionCallback(
    const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr) {
  shark_pos_ = *pt_msg_ptr;
}

void Castaway::MoveStraightForShore(const double& dist) {
  ROS_INFO_STREAM("Castaway heading directly for the shore!");
  double theta = atan2(castaway_pos_.point.y, castaway_pos_.point.x);
  double dx = dist * cos(theta);
  double dy = dist * sin(theta);
  castaway_pos_.point.x = castaway_pos_.point.x + dx;
  castaway_pos_.point.y = castaway_pos_.point.y + dy;
  return;
}
void Castaway::MoveAntipodal(const double& dist) {
  ROS_INFO_STREAM("Castaway moving antipodal to the shark");
  double antipodal_theta = atan2(-shark_pos_.point.y, -shark_pos_.point.x);
  double r_castaway = GetDistanceFromCenter(castaway_pos_);

  if (r_castaway == 0) {
    double dx = dist * cos(antipodal_theta);
    double dy = dist * sin(antipodal_theta);
    castaway_pos_.point.x = castaway_pos_.point.x + dx;
    castaway_pos_.point.y = castaway_pos_.point.y + dy;
    return;
  }

  double x_2 = r_castaway * cos(antipodal_theta);
  double y_2 = r_castaway * sin(antipodal_theta);

  double antipodal_offset = sqrt(pow(x_2 - castaway_pos_.point.x, 2) +
                                 pow(y_2 - castaway_pos_.point.y, 2));
  double d_direction = antipodal_offset / dist;
  double theta_direction = acos(d_direction);

  double y_dist = dist * sin(theta_direction);
  double r_new_castaway = y_dist + r_castaway;

  double dx = r_new_castaway * cos(antipodal_theta);
  double dy = r_new_castaway * sin(antipodal_theta);

  castaway_pos_.point.x = dx;
  castaway_pos_.point.y = dy;
}

void Castaway::CheckCastawayAntipodalToShark(double eps) {
  double theta_castaway = atan2(castaway_pos_.point.y, castaway_pos_.point.x);
  double theta_shark = atan2(shark_pos_.point.y, shark_pos_.point.x);
  double alpha = theta_castaway - theta_shark;
  if (fabs(fabs(alpha) - M_PI) < eps && !is_antipodality_reached_) {
    ROS_INFO("Castaway is now antipodal to shark.");
    is_antipodality_reached_ = true;
  } else if (fabs(fabs(alpha) - M_PI) < eps && is_antipodality_reached_) {
    ROS_INFO("Castaway is still antipodal to shark.");
  } else if (fabs(fabs(alpha) - M_PI) < eps && !is_antipodality_reached_) {
    ROS_WARN("Castaway NOT antipodal to shark.");
    ROS_INFO_STREAM("alpha: " << alpha);
  } else if (fabs(fabs(alpha) - M_PI) < eps && is_antipodality_reached_) {
    ROS_INFO(
        "Castaway reached antipodality and moving with different strategy.");
  }
}

bool Castaway::CalculateCastawayPosition() {
  ROS_INFO("Calculating castaway position.");

  double d_time = CalcTimeDiffFromNow(previous_pt_.header.stamp);
  double d_dist = CASTAWAY_SPEED * d_time;

  if (GetDistanceFromCenter(castaway_pos_) >= 0.25 * ISLAND_RADIUS &&
      is_antipodality_reached_) {
    MoveStraightForShore(d_dist);
  } else {
    MoveAntipodal(d_dist);
  }

  CheckCastawayAntipodalToShark();

  castaway_pos_.header.stamp = ros::Time::now();
  previous_pt_ = castaway_pos_;

  return true;
}

}  // namespace island