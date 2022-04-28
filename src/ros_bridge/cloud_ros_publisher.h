#ifndef DEPTH_CLUSTERING_CLOUD_ROS_PUBLISHER_H
#define DEPTH_CLUSTERING_CLOUD_ROS_PUBLISHER_H

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>

namespace depth_clustering {

using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CloudRosPublisher
    : public AbstractClient<Cloud> {
 public:
  CloudRosPublisher(ros::NodeHandle* node_handle,
                        const std::string& frame_id,
                        const std::string& topic_clouds)
      : _node_handle{node_handle},
        _frame_id{frame_id},
        _topic_clouds{topic_clouds},
        _cloud_pub{_node_handle->advertise<PointCloud2>(_topic_clouds, 1)},
        _multi_pub{_node_handle->advertise<PointCloud2>(_topic_clouds + "/multiple", 1)}
        {}

  ~CloudRosPublisher() override {}
  void OnNewObjectReceived(const Cloud& cloud, int id) override;
  void PublishCloud(const PointCloudT& pcl_cloud);
  void PublishCloudMultiple(const PointCloudT& pcl_cloud);

 protected:
  ros::NodeHandle* _node_handle;
  std::string _frame_id, _topic_clouds;
  ros::Publisher _cloud_pub;
  ros::Publisher _multi_pub;
  ros::Time time_stamp_;
};

}
#endif  // DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
