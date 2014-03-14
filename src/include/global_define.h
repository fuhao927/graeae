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
#include <Eigen/StdVector>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/organized.hpp>
#include <boost/dynamic_bitset.hpp>

// GLOBAL DEFINE HERE
std::string ENVIRONMENT = "office";
std::string WORKDIR = "./toolkit/";
std::string TMPDIR = "./tmp/";
std::string PCDDIR = "./test_results/";
const int MIN_SEGMENT_SIZE = 300;
ros::Publisher pub;
TransformG globalTransform;
std::map<int, double> sceneToAngleMap;

const int NUM_CLASSES=17;
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> *nodeWeights;
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> *edgeWeights[NUM_CLASSES];
std::vector<int> nodeFeatIndices;
std::vector<int> edgeFeatIndices;
std::vector<std::string> nodeFeatNames;
std::vector<std::string> edgeFeatNames;
static const string NODEBINFILE = "binStumpsN.txt";
static const string EDGEBINFILE = "binStumpsE.txt";

bool foundAny = false;
std::vector<int> labelsToFind; // list of classes to find
boost::dynamic_bitset<> labelsFound(NUM_CLASSES); // if the class label is found or not
boost::dynamic_bitset<> labelsToFindBitset(NUM_CLASSES);

std::map<int, int> invLabelMap;
std::map<int, int> labelMap;

#endif
