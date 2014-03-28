// =====================================================================================
//
//       Filename:  segmenter_test.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/11/2014 07:16:47 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "../include/point_types.h"
#include "../include/segmenter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>


  int
main ( int argc, char *argv[] )
{
  PointCloudT::Ptr cloud_ptr(new PointCloudT);
  pcl::io::loadPCDFile<PointT>(argv[1], *cloud_ptr);
  for(size_t i=0;i<cloud_ptr->points.size();i++)
    cloud_ptr->points[i].segment=0;
  filterInPlace(cloud_ptr, 0.01, 0.01, 0.01);
  Graeae::Segment::SegmentType method = (atoi(argv[2])==1) ? Graeae::Segment::EUCLIDEAN : Graeae::Segment::REGIONGROW;
  segmentInPlace(cloud_ptr, method);

  int counts[640 * 480];
  // find the max segment number
  int max_segment_num = 0;
  for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
    counts[cloud_ptr->points[i].segment]++;
    if (max_segment_num < (int)cloud_ptr->points[i].segment)
      max_segment_num = (int)cloud_ptr->points[i].segment;
  }
  ROS_INFO("max_seg num: %d of %d", max_segment_num, cloud_ptr->points.size());

  pcl::io::savePCDFile("segmenter_test.pcd", *cloud_ptr);
  return 0;
}				// ----------  end of function main  ----------
