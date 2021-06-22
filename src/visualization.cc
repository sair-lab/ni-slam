#include "visualization.h"

#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 

void AddNewPoseToPath(
    Eigen::Vector3d& pose, nav_msgs::Path& path, std::string& frame_id){
  ros::Time current_time = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamped; 
  pose_stamped.pose.position.x = pose(0); 
  pose_stamped.pose.position.y = pose(1); 

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose(2)); 
  pose_stamped.pose.orientation.x = q.x; 
  pose_stamped.pose.orientation.y = q.y; 
  pose_stamped.pose.orientation.z = q.z; 
  pose_stamped.pose.orientation.w = q.w; 

  pose_stamped.header.stamp = current_time; 
  pose_stamped.header.frame_id = frame_id; 
  path.poses.push_back(pose_stamped); 
}