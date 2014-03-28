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
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>

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
void markSegmentNum(PointCloudT &cloud, pcl::PointIndices &cluster, int seg_num)
{
  for(size_t i=0;i<cluster.indices.size();i++)
    cloud.points[cluster.indices[i]].segment=seg_num;
}

// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlace
//  Description:  segment cloud with a minimun size of min_pts_per_cluster
// =====================================================================================
void segmentInPlaceEuclidean (PointCloudT::Ptr &cloud_ptr, int min_pts_per_cluster = 1000)
{
  PointCloudT::Ptr _cloud_ptr(new PointCloudT);
  pcl::copyPointCloud (*cloud_ptr, *_cloud_ptr);
  pcl::SACSegmentation<PointT> seg_;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (100);
  seg_.setDistanceThreshold (0.02);

  // Set segment label as 1 to a start
  int seg_num=1;
  int orig_points = _cloud_ptr->points.size();
  PointCloudT::Ptr cloud_tmp_ptr (new PointCloudT);
  PointCloudT::Ptr cloud_plane_ptr (new PointCloudT);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Segment the planar component from the input cloud
  while(_cloud_ptr->points.size() > 0.4 * orig_points) {
    seg_.setInputCloud(_cloud_ptr);
    seg_.segment (*inliers, *coefficients);
    markSegmentNum(*cloud_ptr, *inliers, seg_num++);
    assert(inliers->indices.size() != 0);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_cloud_ptr);
    extract.setIndices(inliers);

    // Remove the planar inliers
    extract.setNegative(true);
    extract.filter(*cloud_tmp_ptr);
    *_cloud_ptr = *cloud_tmp_ptr;
  }

  // Creating kdtree for the search method of the extraction
  KdTreePtr tree(new KdTree);
  tree->setInputCloud(_cloud_ptr);
  std::vector<pcl::PointIndices> cluster_vector;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize (min_pts_per_cluster);
  ec.setMaxClusterSize (_cloud_ptr->points.size());
  ec.setSearchMethod (tree);
  ec.setInputCloud (_cloud_ptr);
  ec.extract (cluster_vector);
  for(size_t i=0;i<cluster_vector.size();i++)
    markSegmentNum(*cloud_ptr, cluster_vector[i], seg_num++);
  ROS_INFO("%d size cloud is seperated into %d pieces.", cloud_ptr->points.size(), seg_num-1);
}

bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 1e-4) {
    if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  } else {
    if (fabs (point_a.intensity - point_b.intensity) < 1.0f)
      return (true);
  }
  return (false);
}

void PointCloudXYZRGBtoXYZH(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<PointTypeIO> &out)
{
  out.width = in.width;
  out.height = in.height;
  for(size_t i = 0; i < in.points.size(); ++i) {
    pcl::PointXYZHSV hsv;
    PointTypeIO tmp;
    PointXYZRGBtoXYZHSV(in.points[i], hsv);
    tmp.x = in.points[i].x; tmp.y = in.points[i].y; tmp.z = in.points[i].z;
    tmp.intensity = hsv.h/360;
    out.points.push_back(tmp);
  }
  ROS_INFO("to hsv point size: %d", out.points.size());
}

// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlaceRG
//  Description:
// =====================================================================================
void segmentInPlaceRG(PointCloudT::Ptr &cloud_ptr)
{
  // Data containers used
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_i_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_normals_ptr (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

  // Load the input point cloud
  pcl::copyPointCloud (*cloud_ptr, *cloud_rgb_ptr);
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
  int seg_num=1;
  // Using the intensity channel for lazy visualization of the output
  for (size_t i = 0; i < clusters->size(); ++i)
    markSegmentNum(*cloud_ptr, (*clusters)[i], seg_num++);
  for (size_t i = 0; i < large_clusters->size(); ++i)
    markSegmentNum(*cloud_ptr, (*large_clusters)[i], seg_num++);
  for (size_t i = 0; i < small_clusters->size(); ++i)
    if(MIN_SEGMENT_SIZE < (*small_clusters)[i].indices.size())
      markSegmentNum(*cloud_ptr, (*small_clusters)[i], seg_num++);
}

// ===  FUNCTION  ======================================================================
//         Name:  segmentInPlace
//  Description:  use specified method to segment the cloud
// =====================================================================================
void segmentInPlace(PointCloudT::Ptr cloud_ptr, Graeae::Segment::SegmentType method)
{
  switch (method) {
    case Graeae::Segment::EUCLIDEAN:
             segmentInPlaceEuclidean(cloud_ptr);
               break;
    case Graeae::Segment::REGIONGROW:
    default :
             segmentInPlaceRG(cloud_ptr);
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  applySegmentFilter
//  Description:
//                [out] cloud_out
//                [out] indices
// =====================================================================================
void applySegmentFilter(PointCloudT &cloud, int seg_num, PointCloudT &cloud_out, std::vector<size_t> &indices)
{
  cloud_out.points.clear();
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if ((int)cloud.points[i].segment == seg_num) {
      PointT tmp = cloud.points[i];
      cloud_out.points.push_back(tmp);
      indices.push_back(i);
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  applySegmentFilter
//  Description:
//                [out] cloud_out
//                [out] indices
// =====================================================================================
void applySegmentFilter(PointCloudT &cloud, int seg_num, PointCloudT &cloud_out)
{
  cloud_out.points.clear();

  int j=0;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if ((int)cloud.points[i].segment == seg_num) {
      PointT tmp = cloud.points[i];
      cloud_out.points.push_back(tmp);
      j++;
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  applyCameraFilter
//  Description:
// =====================================================================================
void applyCameraFilter(const PointCloudT &incloud, PointCloudT &outcloud, int camera)
{
  outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());
  outcloud.header.frame_id = incloud.header.frame_id;
  outcloud.points.resize(incloud.points.size());

  int j = -1;
  for (size_t i = 0; i < incloud.points.size(); ++i) {
    if ((int)incloud.points[i].cameraIndex == camera) {
      j++;
      outcloud.points[j].x = incloud.points[i].x;
      outcloud.points[j].y = incloud.points[i].y;
      outcloud.points[j].z = incloud.points[i].z;
      outcloud.points[j].rgb = incloud.points[i].rgb;
      outcloud.points[j].segment = incloud.points[i].segment;
      outcloud.points[j].label = incloud.points[i].label;
      outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
      outcloud.points[j].distance = incloud.points[i].distance;
    }
  }
  assert(j >= 0);
  outcloud.points.resize(j + 1);
}

void applyNotsegmentFilter(const PointCloudT &incloud, int segment, PointCloudT &outcloud)
{
    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());
    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points.resize(incloud.points.size());

    int j=0;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].segment != segment) {
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;
            j++;
        }
    }
    assert(j > 0);
    outcloud.points.resize(j);
}

// ===  FUNCTION  ======================================================================
//         Name:  getDistanceToBoundary
//  Description:
// =====================================================================================
float getDistanceToBoundary(const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2)
{
  float max_distance = 0;
  for (size_t i = 0; i < cloud1.points.size(); ++i) {
    float pdist = 0;
    for (size_t j = 0; j < cloud2.points.size(); ++j) {
      float point_dist = pow((cloud1.points[i].x - cloud2.points[j].x), 2) + pow((cloud1.points[i].y - cloud2.points[j].y), 2) + pow((cloud1.points[i].z - cloud2.points[j].z), 2);
      float distance = (pow(cloud2.points[j].distance, 2) - pow(cloud1.points[i].distance, 2) - (point_dist)) / (2 * cloud1.points[i].distance);
      if (pdist < distance) pdist = distance;
      if (max_distance < distance) max_distance = distance;
    }
  }
  return max_distance;
}

// ===  FUNCTION  ======================================================================
//         Name:  getSegmentDistanceToBoundary
//  Description:
// =====================================================================================
void getSegmentDistanceToBoundary(const PointCloudT &cloud, std::map<int, float> &segment_boundary_distance)
{
  PointCloudT::Ptr cloud_rest(new PointCloudT ());
  PointCloudT::Ptr cloud_cam(new PointCloudT ());
  PointCloudT::Ptr cloud_seg(new PointCloudT ());
  PointCloudT::Ptr cloud_ptr(new PointCloudT (cloud));

  int cnt = 0;

  std::map<int, int> camera_indices;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    camera_indices[(int) cloud.points[i].cameraIndex] = 1;
  }
  // for every camera index .. apply filter anf get the point cloud
  for (std::map<int, int>::iterator it = camera_indices.begin(); it != camera_indices.end(); it++) {
    int ci = (*it).first;
    applyCameraFilter(*cloud_ptr, *cloud_cam, ci);

    // find the segment list
    std::map<int, int> segments;
    for (size_t i = 0; i < cloud_cam->points.size(); ++i)
      segments[(int) cloud_cam->points[i].segment] = 1;
    for (std::map<int, int>::iterator it2 = segments.begin(); it2 != segments.end(); it2++) {
      cnt++;
      int seg_num = (*it2).first;
      applySegmentFilter(*cloud_cam, seg_num, *cloud_seg);
      applyNotsegmentFilter(*cloud_cam, seg_num, *cloud_rest);
      float bdist = getDistanceToBoundary(*cloud_seg, *cloud_rest);

      std::map<int, float>::iterator segit = segment_boundary_distance.find(seg_num);
      if (segit == segment_boundary_distance.end() || bdist > segment_boundary_distance[seg_num])
        segment_boundary_distance[seg_num] = bdist;
    }
  }
}

#endif
