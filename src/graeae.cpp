// =====================================================================================
//
//       Filename:  main.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/12/2014 01:00:59 AM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "include/global_define.h"
#include "include/generic_utils.h"
#include "include/combine_utils.h"
#include "include/features.h"

int STEP = 1;

void readWeightVectors(std::vector<int> &node,
                      std::vector<int> &edge,
                      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> *node_weights,
                      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> **edge_weights)
{
  node.push_back(15);
  node.push_back(51);
  edge.push_back(5);
  edge.push_back(6);
  edge.push_back(7);
  edge.push_back(10);
  node_weights = new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > (NUM_CLASSES, 10 * node.size());
  readCSV(WORKDIR+"weights/node_weights.csv", NUM_CLASSES, 10 * node.size(), ",", *node_weights);

  for (int i = 0; i < NUM_CLASSES; i++) {
    edge_weights[i] = new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > (NUM_CLASSES, 10 * edge.size());
    readCSV(WORKDIR+"weights/edge_weights_" + boost::lexical_cast<std::string > (i) + ".csv", NUM_CLASSES, 10 * edge.size(), ",", *edge_weights[i]);
  }
}

void readStumpValues(vector<BinStumps> & featBins, const string & file)
{
  string line;
  ifstream myfile(file.data());

  if (!myfile.is_open()) {
    ROS_ERROR("cound not find the file: %s which stores the ranges for binning the features binStumpsN.txt and binStumpsE.txt", file.c_str());
  }
  assert(myfile.is_open());

  while (myfile.good()) {
    getline(myfile, line);
    if (line.length() > 0)
      featBins.push_back(BinStumps(line));
  }
  myfile.close();
}

// ===  FUNCTION  ======================================================================
//         Name:  setupFrame
//  Description:
// =====================================================================================
void setupFrame(OriginalFrameInfo* &frame, TransformG t, PointCloudT::Ptr cloud_ptr)
{
  frame = new OriginalFrameInfo(cloud_ptr);
  frame->setCameraTrans(t);
}

void pointCloudProCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_blob)
{
  static int callback_counter_ = 0;
  ROS_INFO("Received frame from kinect");
  if (++callback_counter_ % STEP == 0) {
    ROS_INFO("accepted it");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudT::Ptr cloud_seg_ptr(new PointCloudT);
    pcl::fromROSMsg(*cloud_blob, *cloud_ptr);
    convertType(*cloud_ptr, *cloud_seg_ptr, VectorG(0,0,0), 0);
    assert(cloud_seg_ptr->size() == 640 * 480);
    globalTransform.transformPointCloudInPlaceAndSetOrigin(*cloud_seg_ptr);
    setupFrame(originalFrame, globalTransform, cloud_seg_ptr);
    filterInPlace(cloud_seg_ptr, 0.01, 0.01, 0.01);
    for(size_t i=0;i<cloud_seg_ptr->points.size();i++) {
      cloud_seg_ptr->points[i].segment = 0;
      cloud_seg_ptr->points[i].label = 50;
    }
    segmentInPlace(cloud_seg_ptr, Graeae::Segment::REGIONGROW);
    writeFeatures(cloud_seg_ptr, callback_counter_);
    sceneToAngleMap[callback_counter_] = currentAngle;
  }else
    ROS_INFO("rejected it");
}

  int
main ( int argc, char *argv[] )
{
  readWeightVectors(nodeFeatIndices, edgeFeatIndices, nodeWeights, edgeWeights);
  ros::init(argc, argv, "Graeae");
  if (argc > 1) ENVIRONMENT = argv[1];
  std::cout << "using evv= " << ENVIRONMENT << endl;
  ros::NodeHandle n;

  //Instantiate the kinect image listener
  readStumpValues(nodeFeatStumps, WORKDIR + ENVIRONMENT + "/" + NODEBINFILE);
  readStumpValues(edgeFeatStumps, WORKDIR + ENVIRONMENT + "/" + EDGEBINFILE);

  readLabelList(SETTING + ENVIRONMENT + "_to_find.txt", labelsToFind, labelsToFindBitset);
  labelsFound = labelsToFindBitset;
  labelsFound.flip();

  readInvLabelMap(WORKDIR + "svm-python-v204/" + ENVIRONMENT + "_labelmap.txt", invLabelMap, labelMap);
  globalTransform = readTranform(SETTING + "globalTransform.bag");

  pub = n.advertise<sensor_msgs::PointCloud2 > ("/scene_label/labeled_cloud", 10);
  ros::Subscriber cloud_sub_ = n.subscribe("/camera/depth_registered/points", 1, pointCloudProCallback);
  ros::spin();

  return 0;
} // ----------  end of function main  ----------
