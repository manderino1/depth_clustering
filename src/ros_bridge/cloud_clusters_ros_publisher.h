#ifndef DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
#define DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "image_labelers/abstract_image_labeler.h"
#include "utils/cloud.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>

struct PointXYZRGBLI
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;                    //if you want to add colors, you'd better use that

  uint32_t label;
  uint32_t idx;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBLI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float,rgb,rgb)
                                  (uint32_t, label, label)
                                  (uint32_t, idx, idx)
)

namespace depth_clustering {

using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

typedef PointXYZRGBLI PointC;
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
  ros::Time time_stamp_;
};

}
#endif  // DEPTH_CLUSTERING_CLOUD_CLUSTERS_ROS_PUBLISHER_H
