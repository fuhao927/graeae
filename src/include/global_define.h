// =====================================================================================
//
//       Filename:  global_define.h
//
//    Description:
//
//        Version:  1.0
//        Created:  03/11/2014 03:34:24 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#ifndef GLOBAL_DEFINE_H
#define GLOBAL_DEFINE_H

#include <ros/ros.h>
#include "point_types.h"
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/organized.hpp>

typedef pcl::PointXYZRGBCamSL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef  pcl::search::KdTree<PointT> KdTree;
typedef  pcl::search::KdTree<PointT>::Ptr KdTreePtr;

#endif
