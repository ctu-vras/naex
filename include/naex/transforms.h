#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace naex {

inline void transform_to_pose(const geometry_msgs::msg::Transform &tf,
                              geometry_msgs::msg::Pose &pose) {
  pose.position.x = tf.translation.x;
  pose.position.y = tf.translation.y;
  pose.position.z = tf.translation.z;
  pose.orientation = tf.rotation;
}

inline void transform_to_pose(const geometry_msgs::msg::TransformStamped &tf,
                              geometry_msgs::msg::PoseStamped &pose) {
  pose.header = tf.header;
  transform_to_pose(tf.transform, pose.pose);
}

} // namespace naex
