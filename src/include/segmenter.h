// =====================================================================================
//
//       Filename:  segmenter.h
//
//    Description:
//
//        Version:  1.0
//        Created:  03/11/2014 02:06:08 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#ifndef SEGMENTER_H
#define SEGMENTER_H

#include "global_define.h"
#include "ModelCoefficients.h"
#include "impl/segmenter.hpp"
#include "impl/conditional_euclidean_clustering.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types_conversion.h>

namespace Graeae {
  namespace Segment{
    enum SegmentType{EUCLIDEAN, REGIONGROW};
  }
} //namespace end

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

// ===  FUNCTION  ======================================================================
//         Name:  filterInPlace
//  Description:  filter cloud_ptr in a scale of (size_x, size_y, size_z)
// =====================================================================================
void filterInPlace(PointCloudT::Ptr cloud_ptr, float size_x, float size_y, float size_z)
{
  pcl::PassThrough<PointT> pt_;
  pt_.setInputCloud(cloud_ptr);
  pt_.filter(*cloud_ptr);

  pcl::VoxelGrid<PointT> vg_;
  vg_.setLeafSize(size_x, size_y, size_z);
  vg_.setInputCloud(cloud_ptr);
  vg_.filter(*cloud_ptr);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud_ptr);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_ptr);

  ROS_INFO("Cloud size after filter: %d", cloud_ptr->size());
}

// ===  FUNCTION  ======================================================================
//         Name:  markSegmentNum
//  Description:  mark a segment cloud as labelNum
// =====================================================================================
void markSegmentNum(PointCloudT &cloud, pcl::PointIndices &cluster, int segNum)
{
  for(size_t i=0;i<cluster.indices.size();i++)
    cloud.points[cluster.indices[i]].segment=segNum;
}

// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlace
//  Description:  segment cloud with a minimun size of min_pts_per_cluster
// =====================================================================================
void segmentInPlaceEuclidean (PointCloudT &cloud, PointCloudT &cloud_out, int min_pts_per_cluster = 1000)
{
  pcl::SACSegmentation<PointT> seg_;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (100);
  seg_.setDistanceThreshold (0.02);

  // Set segment label as 1 to a start
  int seg_num=1;
  int orig_points = cloud.points.size();
  PointCloudT::Ptr cloud_ptr (new PointCloudT(cloud));
  PointCloudT::Ptr cloud_tmp_ptr (new PointCloudT);
  PointCloudT::Ptr cloud_plane_ptr (new PointCloudT);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Segment the planar component from the input cloud
  while(cloud_ptr->points.size() > 0.4 * orig_points) {
    seg_.setInputCloud(cloud_ptr);
    seg_.segment (*inliers, *coefficients);
    assert(inliers->indices.size() != 0);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    markSegmentNum(*cloud_ptr, *inliers, seg_num++);
    extract.setNegative(false);
    extract.filter(*cloud_plane_ptr);
    cloud_out += *cloud_plane_ptr;

    // Remove the planar inliers
    extract.setNegative(true);
    extract.filter(*cloud_tmp_ptr);
    *cloud_ptr = *cloud_tmp_ptr;
  }

  // Creating kdtree for the search method of the extraction
  KdTreePtr tree(new KdTree);
  tree->setInputCloud(cloud_ptr);
  std::vector<pcl::PointIndices> cluster_vector;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize (min_pts_per_cluster);
  ec.setMaxClusterSize (cloud_ptr->points.size());
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_vector);
  for(size_t i=0;i<cluster_vector.size();i++)
    markSegmentNum(*cloud_ptr, cluster_vector[i], seg_num++);
  cloud_out += *cloud_ptr;
  ROS_INFO("%d size cloud is seperated into %d pieces.", cloud_out.points.size(), seg_num-1);
}

bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 1e-4) {
    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  } else {
    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}


// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlaceRG
//  Description:
// =====================================================================================
void segmentInPlaceRG(PointCloudT &cloud, PointCloudT &cloud_out)
{
  // Data containers used
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_i_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_normals_ptr (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

  // Load the input point cloud
  pcl::copyPointCloud (cloud, *cloud_rgb_ptr);
  pcl::copyPointCloud (cloud, cloud_out);
  PointCloudXYZRGBtoXYZI(*cloud_rgb_ptr, *cloud_i_ptr);

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  //  ROS_INFO("Computing normals...");
  pcl::copyPointCloud (*cloud_i_ptr, *cloud_normals_ptr);
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_i_ptr);
  ne.setSearchMethod (search_tree);
  ne.setKSearch (30);
  ne.compute (*cloud_normals_ptr);

  // Set up a Conditional Euclidean Clustering class
  //  ROS_INFO("Segmenting to clusters...");
  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
  cec.setInputCloud (cloud_normals_ptr);
  cec.setConditionFunction (&customRegionGrowing);
  cec.setClusterTolerance (0.04);
  cec.setMinClusterSize (cloud_normals_ptr->points.size () / 200);
  cec.setMaxClusterSize (cloud_normals_ptr->points.size () / 5);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);

  ROS_INFO("Specified Segment Number..., small: %d, large: %d, extract: %d", small_clusters->size(), large_clusters->size(), clusters->size());
  int segNum=1;
  // Using the intensity channel for lazy visualization of the output
  for (size_t i = 0; i < clusters->size(); ++i)
    markSegmentNum(cloud_out, (*clusters)[i], segNum++);
  for (size_t i = 0; i < large_clusters->size(); ++i)
    markSegmentNum(cloud_out, (*large_clusters)[i], segNum++);
  if(segNum < 8)
    for (size_t i = 0; i < 8-segNum; ++i)
      markSegmentNum(cloud_out, (*small_clusters)[i], segNum++);
}

// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlace
//  Description:  use specified method to segment the cloud
// =====================================================================================
void segmentInPlace(PointCloudT &cloud, PointCloudT &cloud_out, Graeae::Segment::SegmentType method)
{
  switch (method) {
    case Graeae::Segment::EUCLIDEAN:
             segmentInPlaceEuclidean(cloud, cloud_out);
               break;
    case Graeae::Segment::REGIONGROW:
    default :
             segmentInPlaceRG(cloud, cloud_out);
  }
}

#endif
