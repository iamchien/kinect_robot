#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>

// Filtering a PointCloud using ModelOutlierRemoval
// http://pointclouds.org/documentation/tutorials/model_outlier_removal.php#model-outlier-removal

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

void callback(const sensor_msgs::PointCloud2& ros_pc)
{
  // Convert input to PCL format
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(ros_pc, pcl_pc);

  point_cloud_t::Ptr input_ptr(new point_cloud_t());
  pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  // Setup Segmentation
  pcl::ModelCoefficients plane_coeff;
  plane_coeff.values.resize (4);
  plane_coeff.values[0] = 0;
  plane_coeff.values[1] = 0;
  plane_coeff.values[2] = 0;
  plane_coeff.values[3] = 0;
  
  pcl::ModelOutlierRemoval<pcl::PointXYZ> plane_filter;
  plane_filter.setModelCoefficients (plane_coeff);
  plane_filter.setThreshold (0.05);
  plane_filter.setModelType (pcl::SACMODEL_PLANE);
  plane_filter.setInputCloud (input_ptr);
  
  plane_filter.filter (*output_ptr);

  // Covert output to ROS format
  sensor_msgs::PointCloud2 ros_output;
  pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, ros_output);

  // Publish the data
  pub.publish(ros_output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segmen");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/filtered", 1, callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/segmen", 1);

    // Spin
    ros::spin ();
}
