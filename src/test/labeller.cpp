// =====================================================================================
//
//       Filename:  labeller.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/15/2014 08:06:56 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "../include/global_define.h"
#include "../include/generic_utils.h"
#include "../include/combine_utils.h"
#include "../include/features.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>


//----------------------------------------------------------------------
//  Global variables list
//----------------------------------------------------------------------
pcl::visualization::PCLVisualizer viewer("3D Viewer");
PointCloudT::Ptr cloud_ptr(new PointCloudT);
PointCloudT::Ptr new_cloud_ptr (new PointCloudT);
std::string pressed_num;
std::vector<std::vector<size_t> > indicesVector;
int currentSegment = 0;
std::string infile;
uint8_t r=255, g=0, b=0;
uint32_t seg_color = ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);

// ===  FUNCTION  ======================================================================
//         Name:  applySegmentFilter
//  Description:
// =====================================================================================
void applySegmentFilter(PointCloudT &cloud, std::vector<std::vector<size_t> > &_indicesVector)
{
  // find the max segment number
  PointCloudT cloud_seg;
  int max_segment_num = 0;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (max_segment_num < (int)cloud.points[i].segment)
      max_segment_num = (int) cloud.points[i].segment;
  }

  // Extract indices by seg_num
  _indicesVector.resize(max_segment_num+1);
  for (int seg_num = 0; seg_num <= max_segment_num; ++seg_num)
    applySegmentFilter(cloud, seg_num, cloud_seg, _indicesVector[seg_num]);
}

// ===  FUNCTION  ======================================================================
//         Name:  markLabel
//  Description:
// =====================================================================================
void markLabel(size_t seg, int label_num)
{
  ROS_INFO("segment %d will be marked as label %d.", seg, label_num);
  for(size_t i = 0; i < indicesVector[currentSegment].size(); i++)
    cloud_ptr->points[indicesVector[currentSegment][i]].label = label_num;
}

// ===  FUNCTION  ======================================================================
//         Name:  nextSegment
//  Description:
// =====================================================================================
void nextSegment(int seg_num)
{
  viewer.removePointCloud("sample cloud");
  *new_cloud_ptr = *cloud_ptr;
  ROS_INFO("Showing next segment: %d of current segment %d", indicesVector.size(), seg_num);
  for(size_t i=0; i < indicesVector[seg_num].size(); i++)
    new_cloud_ptr->points[indicesVector[seg_num][i]].rgb = *reinterpret_cast<float*>(&seg_color);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(new_cloud_ptr);
  viewer.addPointCloud<PointT> (new_cloud_ptr, rgb, "sample cloud");
}

// ===  FUNCTION  ======================================================================
//         Name:  keyboardEventOccurred
//  Description:
// =====================================================================================
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{
  if(event.isCtrlPressed() && event.keyUp()) {
    int label_num = boost::lexical_cast<int>(pressed_num);
    markLabel(currentSegment, label_num);
    pressed_num.clear();
  }else {
    char word = event.getKeyCode();
    if(event.keyUp() && word >= '0' && word <= '9')
      pressed_num += word;
    else if (event.keyUp() && word == 'n') {
      currentSegment = ++currentSegment % indicesVector.size();
      nextSegment(currentSegment);
    }
    else if (event.keyUp() && word == 'p') {
      currentSegment = --currentSegment % indicesVector.size();
      nextSegment(currentSegment);
    }
    else if (event.keyUp() && word == 's') {
      string fn = "labelled_" + infile;
      pcl::io::savePCDFile(fn, *cloud_ptr);
      ROS_INFO("point cloud has saved into %s.", fn.c_str());
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  colorVisual
//  Description:
// =====================================================================================
void colorVisualInWindow(PointCloudT::ConstPtr cloud)
{
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.initCameraParameters();
  viewer.registerKeyboardCallback(keyboardEventOccurred);
}
  int
main ( int argc, char *argv[] )
{
  // Load in point cloud
  if(argc < 2) ROS_INFO("Usage: test.pcd");
  sensor_msgs::PointCloud2 cloud_blob;
  infile = argv[1];
  if(pcl::io::loadPCDFile<PointT>(infile, *cloud_ptr) != 0) {
    ROS_ERROR("can not load into %s.", infile.c_str());
    exit(-1);
  }
  ROS_INFO("%d point cloud has been loaded into mem.", cloud_ptr->size());

  colorVisualInWindow(cloud_ptr);

  // Extract segment indices into vector
  applySegmentFilter(*cloud_ptr, indicesVector);
  nextSegment(currentSegment);

  // Main loop
  viewer.spin();
  return 0;
}				// ----------  end of function main  ----------
