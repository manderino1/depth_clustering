#include "cloud_clusters_ros_publisher.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/timer.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;

void CloudClusterRosPublisher::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int id){
  time_utils::Timer total_timer;
  PointCloudC pcl_cloud;
  ImageToPcl(clouds, pcl_cloud);
  PublishCloud(pcl_cloud);
}

void CloudClusterRosPublisher::ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds, PointCloudC& pcl_cloud) {
  auto random_colors = AbstractImageLabeler::GetRandomColors();

  std::vector<PointCloudC::Ptr> pcl_clouds;
  pcl_clouds.reserve(clouds.size());

  // Create set of pcl clouds
  for (const auto& kv : clouds) {
    const auto& cluster = kv.second;
    pcl_clouds.emplace_back(new PointCloudC);
    for (const auto& point : cluster.points()) {
      PointC p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      p.label = kv.first; // it was i, but more usefull kv.first for debuging reasons

      // Set label color
      auto random_color = random_colors[p.label % random_colors.size()];
      p.r = random_color[0];
      p.g = random_color[1];
      p.b = random_color[2];

      pcl_clouds.back()->push_back(p);
    }
  }

  // Build kd-trees for each one
  std::vector<pcl::KdTreeFLANN<PointC>> kdtree_clouds;
  kdtree_clouds.resize(clouds.size());

  for (int i=0; i<pcl_clouds.size(); i++) {
    kdtree_clouds[i].setInputCloud(pcl_clouds[i]);
  }

  // Compare all clusters with all clusters
  std::vector<int> index_j(1);
  std::vector<int> index_i(1);
  std::vector<float> distance_j(1);
  std::vector<float> distance_i(1);

  std::vector<std::pair<int, int>> clusters_to_match;

  for (int i=0; i<pcl_clouds.size(); i++) {
    for (int j=i+1; j<pcl_clouds.size(); j++) {
      // Cycle all points in the first cloud
      for(int k=0; k<pcl_clouds[i]->size(); k++) {
        // Find closest point to this one in cluster in j
        if (kdtree_clouds[j].nearestKSearch ((*pcl_clouds[i])[k], 1, index_j, distance_j) > 0) {
          // Find closest point to the found one in cluster in i
          if (kdtree_clouds[i].nearestKSearch ((*pcl_clouds[j])[index_j[0]], 1, index_i, distance_i) > 0) {
            // If point is the closest point to the one found, these are the closest point between clusters
            if(k == index_i[0]) {
              // Closest point has been found
              // If under threshold merge clusters
              if(distance_i[0] < 5) {
                clusters_to_match.emplace_back(i, j);
              }
            }
          }
        }
      }
    }
  }

  // TODO: free clouds
}

  void CloudClusterRosPublisher::PublishCloud(const PointCloudC& pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.frame_id = _frame_id;
    cloud2.header.stamp = ros::Time::now(); // TODO: Set output time stamp as original cluster stamp
    _cloud_pub.publish(cloud2);
  }

}

