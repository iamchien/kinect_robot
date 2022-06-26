#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "kinect_robot/SegmentedClustersArray.h"

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;

void callback(const sensor_msgs::PointCloud2& ros_pc)
{
  // Convert input to PCL format
  pcl::PCLPointCloud2 pcl_pc;

  // Convert ROS data PC to PCL data PC
  pcl_conversions::toPCL(ros_pc, pcl_pc);

  point_cloud_t::Ptr cloud (new point_cloud_t());
  point_cloud_t::Ptr remainPoints (new point_cloud_t());
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);

  // Perform euclidean cluster segmentation to seporate individual objects
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.01); // 1 cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  // int j = 0;
  kinect_robot::SegmentedClustersArray CloudClusters;
  sensor_msgs::PointCloud2 ros_output;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    point_cloud_t::Ptr cloud_cluster (new point_cloud_t());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }

    // cloud_cluster->width = cloud_cluster->size ();
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;
    
    // Covert output to ROS format
    // Publish the data
    pcl::toPCLPointCloud2(*cloud_cluster, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);
    CloudClusters.clusters.push_back(ros_output);
    // j++;
  }

  pub.publish(CloudClusters);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cluster");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/segmen", 1, callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<kinect_robot::SegmentedClustersArray> ("/camera/depth/cluster",1);

    // Spin
    ros::spin ();
}

