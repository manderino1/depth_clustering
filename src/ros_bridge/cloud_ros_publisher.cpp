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
  auto cloud_multiple_returns = cloud.MultiReturnLuminar(cloud.projection_ptr()->depth_image());
  //PublishCloud(*(cloud.ToPcl()));
  auto cloud_pcl = *cloud_ground_removed->ToPcl();
  auto cloud_multiple_returns_pcl = *cloud_multiple_returns->ToPcl();
  time_stamp_ = cloud.time_stamp();
  PublishCloud(cloud_pcl);
  PublishCloudMultiple(cloud_multiple_returns_pcl);
}


  void CloudRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = time_stamp_;
    cloud2.fields[3].name = "idx";
    _cloud_pub.publish(cloud2);
  }

  void CloudRosPublisher::PublishCloudMultiple(const PointCloudT& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = time_stamp_;
    cloud2.fields[3].name = "idx";
    _multi_pub.publish(cloud2);
  }

}

