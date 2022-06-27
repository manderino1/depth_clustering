#include "cloud_clusters_ros_publisher.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/timer.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;

void CloudClusterRosPublisher::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int id){
  if(!clouds.empty()) {
    time_stamp_ = clouds.begin()->second.time_stamp();
  }

  time_utils::Timer total_timer;
  PointCloudC pcl_cloud;
  ImageToPcl(clouds, pcl_cloud);
  PublishCloud(pcl_cloud);
}

void CloudClusterRosPublisher::ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds, PointCloudC& pcl_cloud) {
  int i = 0;
  auto random_colors = AbstractImageLabeler::GetRandomColors();

  geometry_msgs::PoseArray dims;
  geometry_msgs::PoseArray sizeps;

  for (const auto& kv : clouds) {
    const auto& cluster = kv.second;
    PointCloudC pcl_temp;
    double dist_sum = 0;
    for (const auto& point : cluster.points()) {
      PointC p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      p.label = kv.first; // it was i, but more usefull kv.first for debuging reasons
      p.idx = point.index();
      dist_sum += sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

      // Set label color
      auto random_color = random_colors[p.label % random_colors.size()];
      p.r = random_color[0];
      p.g = random_color[1];
      p.b = random_color[2];

      pcl_temp.push_back(p);
    }

    PointC min_point, max_point;
    pcl::getMinMax3D(pcl_temp, min_point, max_point);

    geometry_msgs::Pose dim;
    dim.orientation.x = kv.first;
    dim.position.x = abs(max_point.x - min_point.x);
    dim.position.y = abs(max_point.y - min_point.y);
    dim.position.z = abs(max_point.z - min_point.z);
    dims.poses.push_back(dim);

    geometry_msgs::Pose sizep;
    sizep.orientation.x = kv.first;
    sizep.position.x = pcl_temp.size();
    sizeps.poses.push_back(sizep);

    // Show color depending on cluster type
    double dist_avg = dist_sum / cluster.size();
    double volume = dim.position.x * dim.position.y * dim.position.z;
    double area = dim.position.x * dim.position.y;

    // Create feature vector
    std::vector<double> feature_vector;
    feature_vector.push_back(dim.position.x);
    feature_vector.push_back(dim.position.y);
    feature_vector.push_back(dim.position.z);
    feature_vector.push_back(pcl_temp.size());
    feature_vector.push_back(dist_avg);
    feature_vector.push_back(area);
    feature_vector.push_back(volume);

    // Classify cluster
    int target_class = cluster_decision_tree(feature_vector);


    for (auto& p : pcl_temp) {
      // Set label color
      if(target_class == 0) {
        p.r = 255;
        p.g = 0;
        p.b = 0;
      } else if(target_class == 1) {
        p.r = 0;
        p.g = 255;
        p.b = 0;
      }
    }

    pcl_cloud += pcl_temp;
    i++;
  }

  dims.header.stamp = time_stamp_;
  _dim_pub.publish(dims);

  sizeps.header.stamp = time_stamp_;
  _points_pub.publish(sizeps);

  // Fill dimensions
}

  void CloudClusterRosPublisher::PublishCloud(const PointCloudC& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = time_stamp_; // TODO: Set output time stamp as original cluster stamp
    _cloud_pub.publish(cloud2);
  }

}

