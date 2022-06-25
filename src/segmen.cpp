#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// PCL specific includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

  // Setup Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  // std::cout << "Size of inliers.indices: " << inliers.indices.size() << std::endl;

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*remainPoints);  

  // pcl_msgs::PointIndices ros_inliers;
  // pcl_conversions::fromPCL(*inliers, ros_inliers);

  // // mark the found inliers in green
  // for (int m=0; m<ros_inliers.indices.size(); ++m)
  // {
  //   cloud->points[ros_inliers.indices[m]].r = 255;
  //   cloud->points[ros_inliers.indices[m]].g = 0;
  //   cloud->points[ros_inliers.indices[m]].b = 0;
  // }

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
    ros::init (argc, argv, "segmen");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/filtered", 1, callback);

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<pcl_msgs::PointIndices>("/camera/depth/segmen", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/segmen", 1);

    // Spin
    ros::spin ();
}

