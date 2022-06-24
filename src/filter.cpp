#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;

void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary

    // Convert ROS data PC to PCL data PC
    pcl_conversions::toPCL(ros_pc, pcl_pc);

    // Convert point cloud to PCL native point cloud
    point_cloud_t::Ptr input_ptr(new point_cloud_t());
    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

    // Set up VoxelGrid filter to bin into 5cm grid
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input_ptr);
    sor.setLeafSize(0.025, 0.025, 0.025);

    // Create output point cloud
    point_cloud_t::Ptr output_ptr(new point_cloud_t());

    // Run filter
    sor.filter(*output_ptr);

    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);

    // Publish the data
    pub.publish(ros_output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "filter");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/filtered", 1);

    // Spin
    ros::spin ();
}
