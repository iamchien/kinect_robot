#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

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
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::PointIndices::Ptr ground (new pcl::PointIndices());
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.01f);
  pmf.setMaxDistance (0.02f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*remainPoints);

  // Covert output to ROS format
  // Publish the data
  sensor_msgs::PointCloud2 ros_output;
  pcl::toPCLPointCloud2(*remainPoints, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, ros_output);

  pub.publish(ros_output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "identify_ground");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/filtered", 1, callback);

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<pcl_msgs::PointIndices>("/camera/depth/segmen", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/segmen", 1);

    // Spin
    ros::spin ();
}
