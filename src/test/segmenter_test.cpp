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
  PointCloudT::Ptr cloud_out(new PointCloudT);
  pcl::io::loadPCDFile<PointT>(argv[1], *cloud_ptr);
  for(size_t i=0;i<cloud_ptr->points.size();i++)
    cloud_ptr->points[i].segment=0;
  filterInPlace(cloud_ptr, 0.01, 0.01, 0.01);
  Graeae::Segment::SegmentType method = (atoi(argv[2])==1) ? Graeae::Segment::EUCLIDEAN : Graeae::Segment::REGIONGROW;
  segmentInPlace(*cloud_ptr, *cloud_out, method);
  pcl::io::savePCDFile("segmenter_test.pcd", *cloud_out);
  return 0;
}				// ----------  end of function main  ----------
