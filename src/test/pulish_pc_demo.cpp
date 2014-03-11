// =====================================================================================
//
//       Filename:  pulish_pc_demo.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/09/2014 10:09:47 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================

#include "../include/point_types.h"

#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>


  int
main ( int argc, char *argv[] )
{
  if(argc < 2) {
    ROS_ERROR("Usage: run point_cloud.pcd");
    exit(-1);
  }

  ros::init(argc, argv, "pulish_pc_demo");
  ros::NodeHandle n;

  // read the argv[1].pcd into mem
  sensor_msgs::PointCloud2::Ptr cloud_blob_ptr(new sensor_msgs::PointCloud2);
  if(pcl::io::loadPCDFile(argv[1], *cloud_blob_ptr) == -1) {
    ROS_ERROR("can't load point cloud from %s", argv[1]);
    exit(-1);
  }
  ROS_INFO("Loaded %d data points from %s.", (int)(cloud_blob_ptr->width * cloud_blob_ptr->height), argv[1]);
  ROS_INFO("Press Enter key to publish this point cloud.");

  // publish to /camera/depth_registered/points
  ros::Publisher pc_pub = n.advertise <sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10);
  while (ros::ok()) {
    switch (std::cin.get()) {
      case '\n':
        ROS_INFO("Publishing %s point cloud...", argv[1]);
        pc_pub.publish(*cloud_blob_ptr);
        break;
    }
    ros::spinOnce();
  }
  return 0;
}     // ----------  end of function main  ----------
