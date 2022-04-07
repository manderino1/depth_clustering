#include "cloud_ros_publisher.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/timer.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;

void CloudRosPublisher::OnNewObjectReceived(const Cloud& cloud, const int id){
  time_utils::Timer total_timer;
  //PointCloudT plc_cloud;
  //ImageToPcl(clouds, plc_cloud);
  PointCloudT pcl_temp;
  auto cloud_ground_removed = cloud.FromImageLuminar(cloud.projection_ptr()->depth_image());
  //PublishCloud(*(cloud.ToPcl()));
  PublishCloud(*cloud_ground_removed->ToPcl());
}


  void CloudRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = ros::Time::now();
    _cloud_pub.publish(cloud2);
  }

}

