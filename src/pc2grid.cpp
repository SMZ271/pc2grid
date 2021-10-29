#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "pc2grid/Grid.h"


//  specfic includes 
//#include <grid_map_ros/grid_map_ros.hpp>

using namespace std;
//using namespace grid_map;

ros::Publisher pub_pointcloud;
ros::Publisher pub_gridmap;


void callback(const sensor_msgs::PointCloud2ConstPtr& input){
	// convert pointcloud2 to pcl pointcloud;
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2 cloud_filtered;
	pcl_conversions::toPCL(*input, *cloud);

	// crop roi
	pcl::PassThrough<pcl::PCLPointCloud2> pass_x;
	pass_x.setInputCloud(cloud);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-1.2, 1.2);
	pass_x.filter(*cloud);

	pcl::PassThrough<pcl::PCLPointCloud2> pass_y;
	pass_y.setInputCloud(cloud);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-1.2, 1.2);
	pass_y.filter(cloud_filtered);
	
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);
	
	pub_pointcloud.publish(output);

	pcl::PointCloud<pcl::PointXYZ> f_pc;
	pcl::fromPCLPointCloud2(cloud_filtered, f_pc);
	
	// create mesh map
	nav_msgs::OccupancyGrid grid_msgs;
	grid_msgs.header.stamp = ros::Time::now();
	grid_msgs.header.frame_id = "livox_frame";	
	grid_msgs.info.resolution = 0.12;
        grid_msgs.info.width = 20;
	grid_msgs.info.height = 20;

	for(int i = 0; i < 400; i++){
		grid_msgs.data.push_back(0);
	}	
	
	for(int i = 0; i < f_pc.size(); i++){

		double x = f_pc.points[i].x;
		double y = f_pc.points[i].y;
		int xc = (int)((x + 1.2) / 0.12);
		int yc = (int)((y + 1.2) / 0.12);
		grid_msgs.data[yc * 20 + xc] = 100;  

	}

	// cout << count_grid << endl;
	
	pub_gridmap.publish(grid_msgs);

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_to_grid");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/livox/lidar", 10, callback);

  // Create a ROS publisher for the output point cloud
  pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  
  // create a ros publisher for the output grid
  pub_gridmap = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

  // Spin
  ros::spin ();
}
