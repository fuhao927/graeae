// =====================================================================================
//
//       Filename:  label_viwer.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/11/2014 02:49:25 AM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "../include/label_viwer.h"
#include "../include/point_types.h"
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

typedef pcl::PointXYZRGBCamSL PointT;


boost::shared_ptr<pcl::visualization::PCLVisualizer>
initVisualizer()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(1,1,1);
  viewer->initCameraParameters ();
  return (viewer);
}


void
visualizerCallback(const sensor_msgs::PointCloud2 &cloud_blob)
{
  //convert point cloud to pcl type
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(cloud_blob, *cloud);
  ROS_INFO("Recieving %d data point cloud from Graeae system.", cloud->size());

  //adding point cloud to viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=initVisualizer();
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb, "label cloud");

  //--------------------
  //-----Main loop-----
  //--------------------
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}


  int
main ( int argc, char *argv[] )
{
  ros::init(argc, argv, "label_visualizer");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scene_label/labeled_cloud", 10, visualizerCallback);
  ROS_INFO("Waiting for Graeae labeling system ...");
  ros::spin();
  return 0;
}				// ----------  end of function main  ----------
