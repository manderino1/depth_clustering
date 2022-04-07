#ifndef DEPTH_CLUSTERING_CLOUD_ROS_PUBLISHER_H
#define DEPTH_CLUSTERING_CLOUD_ROS_PUBLISHER_H

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <unordered_map>

namespace depth_clustering {

using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CloudMultipleRosPublisher
    : public AbstractClient<Cloud> {
 public:
  CloudMultipleRosPublisher(ros::NodeHandle* node_handle,
                        const std::string& frame_id,
                        const std::string& topic_clouds)
      : _node_handle{node_handle},
        _frame_id{frame_id},
        _topic_clouds{topic_clouds},
        _cloud_pub{_node_handle->advertise<PointCloud2>(_topic_clouds, 1)} {}

  ~CloudMultipleRosPublisher() override {}
  void OnNewObjectReceived(const Cloud& cloud, int id) override;
  void PublishCloud(const PointCloudT& pcl_cloud);


 protected:
  bool _front_received = false;
  bool _left_received = false;
  bool _right_received = false;

  Cloud::Ptr _front_cloud;
  Cloud::Ptr _left_cloud;
  Cloud::Ptr _right_cloud;

  ros::NodeHandle* _node_handle;
  std::string _frame_id, _topic_clouds;
  ros::Publisher _cloud_pub;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

}
#endif  // DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
