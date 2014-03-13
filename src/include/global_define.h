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

#include "point_types.h"
#include "combine_utils.h"
#include <ros/ros.h>
#include <eigen/EigenVector>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/organized.hpp>
#include <boost/dynamic_bitset.hpp>

typedef pcl::PointXYZRGBCamSL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::search::KdTree<PointT> KdTree;
typedef pcl::search::KdTree<PointT>::Ptr KdTreePtr;

// GLOBAL DEFINE HERE
std::string ENVIRONMENT = "office";
const int MIN_SEGMENT_SIZE = 300;
ros::Publisher pub;
TransformG globalTransform;

const int NUM_CLASSES=17;
Eigen::Matrix<float, Dynamic, Dynamic> *nodeWeights;
Eigen::Matrix<float, Dynamic, Dynamic> *edgeWeights[NUM_CLASSES];
std::vector<int> nodeFeatIndices;
std::vector<int> edgeFeatIndices;
std::vector<std::string> nodeFeatNames;
std::vector<std::string> edgeFeatNames;
static const string NODEBINFILE = "binStumpsN.txt";
static const string EDGEBINFILE = "binStumpsE.txt";

vector<int> labelsToFind; // list of classes to find
boost::dynamic_bitset<> labelsFound(NUM_CLASSES); // if the class label is found or not
boost::dynamic_bitset<> labelsToFindBitset(NUM_CLASSES);

map<int, int> invLabelMap;
map<int, int> labelMap;

#endif
