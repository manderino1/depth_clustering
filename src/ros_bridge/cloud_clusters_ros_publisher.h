#ifndef DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
#define DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "image_labelers/abstract_image_labeler.h"
#include "utils/cloud.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>

namespace depth_clustering {

using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

typedef pcl::PointXYZRGBL PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

class CloudClusterRosPublisher
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
 public:
  CloudClusterRosPublisher(ros::NodeHandle* node_handle,
                        const std::string& frame_id,
                        const std::string& topic_clouds)
      : _node_handle{node_handle},
        _frame_id{frame_id},
        _topic_clouds{topic_clouds},
        _cloud_pub{_node_handle->advertise<PointCloud2>(_topic_clouds, 1)} {}

  ~CloudClusterRosPublisher() override {}
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, int id) override;
  void ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds, PointCloudC& pcl_cloud);
  void PublishCloud(const PointCloudC& pcl_cloud);


 protected:
  ros::NodeHandle* _node_handle;
  std::string _frame_id, _topic_clouds;
  ros::Publisher _cloud_pub;
};

}
#endif  // DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
