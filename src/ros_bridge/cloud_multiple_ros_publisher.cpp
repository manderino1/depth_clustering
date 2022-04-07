#include "cloud_multiple_ros_publisher.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/timer.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;

void CloudMultipleRosPublisher::OnNewObjectReceived(const Cloud& cloud, const int id){
  time_utils::Timer total_timer;

  auto cloud_ground_removed = cloud.FromImageLuminar(cloud.projection_ptr()->depth_image());
  cloud_ground_removed->SetFrameId(cloud.frame_id());

  // Set cloud as received
  if(cloud.frame_id() == "luminar_front") {
    _front_received = true;
    _front_cloud = cloud_ground_removed;
  } else if(cloud.frame_id() == "luminar_left") {
    _left_received = true;
    _left_cloud = cloud_ground_removed;
  } else if(cloud.frame_id() == "luminar_right"){
    _right_received = true;
    _right_cloud = cloud_ground_removed;
  }

  // If we got all three clouds (they are synced) publish message and reset status
  if(_front_received && _left_received && _right_received) {
    _front_received = false;
    _left_received = false;
    _right_received = false;

    // Transform and Merge clouds
    PointCloudT pcl_front = *_front_cloud->ToPcl();
    PointCloudT pcl_left = *_left_cloud->ToPcl();
    PointCloudT pcl_right = *_right_cloud->ToPcl();

    pcl_ros::transformPointCloud(_frame_id, pcl_front, pcl_front, tf_buffer_);
    pcl_ros::transformPointCloud(_frame_id, pcl_left, pcl_left, tf_buffer_);
    pcl_ros::transformPointCloud(_frame_id, pcl_right, pcl_right, tf_buffer_);

    // Publish merged clouds
    PublishCloud(pcl_front + (pcl_left + pcl_right));
  }
}


  void CloudMultipleRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = ros::Time::now();
    _cloud_pub.publish(cloud2);
  }

}

