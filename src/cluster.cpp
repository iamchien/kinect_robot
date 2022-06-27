#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include "kinect_robot/SegmentedClustersArray.h"

ros::Publisher cluster_array_pub;
ros::Publisher pose_array_pub;
ros::Publisher marker_array_pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;

void callback(const sensor_msgs::PointCloud2& ros_pc)
{
  // Convert input to PCL format
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(ros_pc, pcl_pc);
  point_cloud_t::Ptr cloud (new point_cloud_t());
  point_cloud_t::Ptr remainPoints (new point_cloud_t());
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);

  // Perform euclidean cluster segmentation to seporate individual objects
  std::vector <point_cloud_t::Ptr, Eigen::aligned_allocator <point_cloud_t::Ptr>> clusters;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.1); // 1 cm
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    point_cloud_t::Ptr cloud_cluster (new point_cloud_t());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }

  kinect_robot::SegmentedClustersArray cluster_array;
  geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;

  for (int i = 0; i < clusters.size(); i++)
  {
    if (cluster_array_pub.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toPCLPointCloud2(*clusters[i], pcl_pc);
      pcl_conversions::fromPCL(pcl_pc, ros_pc2_out);
      cluster_array.clusters.push_back(ros_pc2_out);
    }

    if (pose_array_pub.getNumSubscribers() > 0)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid( * clusters[i], centroid);

      geometry_msgs::Pose pose;
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);

      #ifdef LOG
        Eigen::Vector4f min, max;
        pcl::getMinMax3D( * clusters[i], min, max);
        std::cerr << ros_pc -> header.seq << " " <<
          ros_pc -> header.stamp << " " <<
          min[0] << " " <<
          min[1] << " " <<
          min[2] << " " <<
          max[0] << " " <<
          max[1] << " " <<
          max[2] << " " <<
          std::endl;
      #endif
    }

    if (marker_array_pub.getNumSubscribers() > 0)
    {
      Eigen::Vector4f min, max;
      pcl::getMinMax3D( * clusters[i], min, max);

      visualization_msgs::Marker marker;
      marker.header = ros_pc.header;
      marker.ns = "euclidean_clustering";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;

      geometry_msgs::Point p[24];
      p[0].x = max[0]; p[0].y = max[1]; p[0].z = max[2];
      p[1].x = min[0]; p[1].y = max[1]; p[1].z = max[2];
      p[2].x = max[0]; p[2].y = max[1]; p[2].z = max[2];
      p[3].x = max[0]; p[3].y = min[1]; p[3].z = max[2];
      p[4].x = max[0]; p[4].y = max[1]; p[4].z = max[2];
      p[5].x = max[0]; p[5].y = max[1]; p[5].z = min[2];
      p[6].x = min[0]; p[6].y = min[1]; p[6].z = min[2]; 
      p[7].x = max[0]; p[7].y = min[1]; p[7].z = min[2];
      p[8].x = min[0]; p[8].y = min[1]; p[8].z = min[2];
      p[9].x = min[0]; p[9].y = max[1]; p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
      for (int i = 0; i < 24; i++)
      {
        marker.points.push_back(p[i]);
      }

      marker.scale.x = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.lifetime = ros::Duration(0.1);
      marker_array.markers.push_back(marker);
    }
  }

  if (cluster_array.clusters.size())
  {
    cluster_array.header = ros_pc.header;
    cluster_array_pub.publish(cluster_array);
  }

  if (pose_array.poses.size())
  {
    pose_array.header = ros_pc.header;
    pose_array_pub.publish(pose_array);
  }

  if (marker_array.markers.size())
  {
    marker_array_pub.publish(marker_array);
  }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cluster");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/segmen", 1, callback);

    // Create a ROS publisher for the output point cloud
    cluster_array_pub = nh.advertise<kinect_robot::SegmentedClustersArray> ("/camera/depth/cluster",1);
    pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/camera/depth/poses", 1);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/camera/depth/markers", 1);

    // Spin
    ros::spin ();
}

