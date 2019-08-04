#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void CastawayCallback(const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(pt_msg_ptr->point.x, pt_msg_ptr->point.y, 0.0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "castaway_pos"));
}

void SharkCallback(const geometry_msgs::PointStamped::ConstPtr& pt_msg_ptr) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(pt_msg_ptr->point.x, pt_msg_ptr->point.y, 0.0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "shark_pos"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_node");
  ros::NodeHandle nh;
  ros::Subscriber shark_sub = nh.subscribe("shark_position", 1, &SharkCallback);
  ros::Subscriber castaway_sub =
      nh.subscribe("castaway_position", 1, &CastawayCallback);
  ros::spin();
  return 0;
};
