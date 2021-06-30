#include "visualization.h"

#include <algorithm>
#include <numeric>
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 

#include "utils.h"

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

void ConvertMapToOccupancyMsgs(OccupancyData& map, nav_msgs::OccupancyGrid& msgs){
  // const int scale = 2;
  // occupancy_map_msgs.info.resolution = occupancy_map_msgs.info.resolution * scale;

  // fill in metadata
  int max_x = std::numeric_limits<int>::min();
  int max_y = std::numeric_limits<int>::min();
  int min_x = std::numeric_limits<int>::max();
  int min_y = std::numeric_limits<int>::max();

  for(auto& kv : map){
    int x = kv.first.x;
    int y = kv.first.y;
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
  }

  if((min_x > max_x) || (min_y > max_y)) return;

  msgs.info.width = max_x - min_x + 1 + 50;
  msgs.info.height = max_y - min_y + 1 + 50;
  // msgs.info.origin.position.x = 0;
  // msgs.info.origin.position.y = 0;
  // msgs.info.origin.position.z = 0;
  // msgs.info.origin.orientation.x = 0;
  // msgs.info.origin.orientation.y = 0;
  // msgs.info.origin.orientation.z = 0;
  // msgs.info.origin.orientation.w = 1;

  // fill in map_data
  int num_grids = msgs.info.width * msgs.info.height;
  std::vector<int8_t> data(num_grids, -1);

  for(auto& kv : map){
    int x = kv.first.x - min_x;
    int y = kv.first.y - min_y;
    int idx = y * msgs.info.width + x;
    int pixel = std::accumulate(kv.second.begin(), kv.second.end(), 0) / kv.second.size();
    // int pixel = kv.second;
    pixel = std::min(100, std::max(0, pixel));

    data[idx] = 100 - static_cast<int8_t>(pixel);

  }
  msgs.data = data;
}