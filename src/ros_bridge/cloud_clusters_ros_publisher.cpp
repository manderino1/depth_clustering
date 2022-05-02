#include "cloud_clusters_ros_publisher.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/timer.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;

void CloudClusterRosPublisher::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int id){
  time_utils::Timer total_timer;
  PointCloudT pcl_cloud;
  ImageToPcl(clouds, pcl_cloud);
  PublishCloud(pcl_cloud);
}

void CloudClusterRosPublisher::ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds, PointCloudT& pcl_cloud) {
  int i = 0;
  for (const auto& kv : clouds) {
    const auto& cluster = kv.second;
    PointCloudT pcl_temp;
    for (const auto& point : cluster.points()) {
      PointT p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      p.label = kv.first; // it was i, but more usefull kv.first for debuging reasons
      pcl_temp.push_back(p);
    }

    pcl_cloud += pcl_temp;
    i++;
  }
}

  void CloudClusterRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = ros::Time::now(); // TODO: Set output time stamp as original cluster stamp
    _cloud_pub.publish(cloud2);
  }

}

