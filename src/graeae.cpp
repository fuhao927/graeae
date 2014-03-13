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

void pointCloudProCallback(/*const sensor_msgs::ImageConstPtr& visual_img_msg,
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,*/
                     const sensor_msgs::PointCloud2ConstPtr& cloud_blob)
{
  static int callback_counter_ = 0;
  callback_counter_++;
  ROS_INFO("Received frame from kinect");
  if (++callback_counter_ % STEP == 0) {
    ROS_INFO("accepted it");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudT::Ptr cloud_seg_ptr(new PointCloudT);
    pcl::fromROSMsg(*cloud_blob, *cloud_ptr);
    convertType(cloud, *cloud_seg_ptr, VectorG(0,0,0), 0);
    assert(cloud_seg_ptr->size() == 640 * 480);
    globalTransform.transformPointCloudInPlaceAndSetOrigin(*cloud_seg_ptr);
    setupFrame(originalFrame, globalTransform, cloud_seg_ptr);
    filterInPlace(cloud_seg_ptr, 0.01, 0.01, 0.01);
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
  readStumpValues(nodeFeatStumps, ENVIRONMENT + "/" + NODEBINFILE);
  readStumpValues(edgeFeatStumps, ENVIRONMENT + "/" + EDGEBINFILE);

  readLabelList(ENVIRONMENT, labelsToFind, labelsToFindBitset);
  labelsFound = labelsToFindBitset;
  labelsFound.flip();

  readInvLabelMap("../svm-python-v204/" + ENVIRONMENT + "_labelmap.txt", invLabelMap, labelMap);
  globalTransform = readTranform("globalTransform.bag");

  pub = n.advertise<sensor_msgs::PointCloud2 > ("/scene_label/labeled_cloud", 10);
  ros::Subscriber cloud_sub_ = n.subscribe("/camera/depth_registered/points", 1, pointCloudProCallback);
  ros::spin();

  return 0;
} // ----------  end of function main  ----------
