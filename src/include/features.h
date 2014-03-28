// =====================================================================================
//
//       Filename:  feature.h
//
//    Description:
//
//        Version:  1.0
//        Created:  03/12/2014 01:12:23 AM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#ifndef FEATURE_H
#define FEATURE_H

#include "global_define.h"
#include "generic_utils.h"
#include "covarianceMatrix.h"
#include "segmenter.h"
#include "color.h"
#include "HOG.h"
#include "wall_distance.h"
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_ros/OctomapROS.h>


// =====================================================================================
//        Class:  SpectralProfile
//  Description:
// =====================================================================================
class SpectralProfile
{
  private:
    std::vector<float> eigenValues; // sorted in ascending order

  public:
    pcl::PointCloud<PointT>::Ptr cloudPtr;
    HOGFeaturesOfBlock avgHOGFeatsOfSegment;
    float avgH;
    float avgS;
    float avgV;
    int count;

    geometry_msgs::Point32 centroid;
    Eigen::Matrix<float,4,1> centMatrix;
    Eigen::Vector3f normal;

    void setEigValues(Eigen::Vector3f eigenValues_)
    {
      eigenValues.clear();
      assert(eigenValues_(0) <= eigenValues_(1));
      assert(eigenValues_(1) <= eigenValues_(2));

      for (int i = 0; i < 3; i++)
        eigenValues.push_back(eigenValues_(i));
    }

    float getDescendingLambda(int index) const
    {
      return eigenValues[2 - index];
    }

    void addCentroid(const SpectralProfile & other)
    {
      centroid.x+=other.centroid.x;
      centroid.y+=other.centroid.y;
      centroid.z+=other.centroid.z;
      count++; // will be used for computing average
    }

    void transformCentroid(TransformG & trans)
    {
      trans.transformPointInPlace(centroid);
    }

    void setCentroid(const SpectralProfile & other)
    {
      centroid=other.centroid;
      count=1;
    }

    void setAvgCentroid()
    {
      centroid.x/=count;
      centroid.y/=count;
      centroid.z/=count;
    }

    pcl::PointXYZ getCentroid()
    {
      pcl::PointXYZ ret;
      ret.x = centroid.x;
      ret.y = centroid.y;
      ret.z = centroid.z;
      return ret;
    }

    PointT getCentroidSL()
    {
      PointT ret;
      ret.x = centroid.x;
      ret.y = centroid.y;
      ret.z = centroid.z;
      return ret;
    }

    float getScatter() const
    {
      return getDescendingLambda(0);
    }

    float getLinearNess() const
    {
      return (getDescendingLambda(0) - getDescendingLambda(1));
    }

    float getPlanarNess() const
    {
      return (getDescendingLambda(1) - getDescendingLambda(2));
    }

    float getNormalZComponent() const
    {
      return normal[2];
    }

    float getAngleWithVerticalInRadians() const
    {
      return acos(getNormalZComponent());
    }

    float getHorzDistanceBwCentroids(const SpectralProfile & other) const
    {
      return sqrt(pow(centroid.x - other.centroid.x, 2) + pow(centroid.y - other.centroid.y, 2));
    }

    float getDistanceSqrBwCentroids(const SpectralProfile & other) const
    {
      return pow(centroid.x - other.centroid.x, 2) + pow(centroid.y - other.centroid.y, 2) + pow(centroid.z - other.centroid.z, 2);
    }

    float getVertDispCentroids(const SpectralProfile & other)
    {
      return (centroid.z - other.centroid.z);
    }

    float getHDiffAbs(const SpectralProfile & other)
    {
      return fabs(avgH - other.avgH);
    }

    float getSDiff(const SpectralProfile & other)
    {
      return (avgS - other.avgS);
    }

    float getVDiff(const SpectralProfile & other)
    {
      return (avgV - other.avgV);
    }

    float getAngleDiffInRadians(const SpectralProfile & other)
    {
      return (getAngleWithVerticalInRadians() - other.getAngleWithVerticalInRadians());
    }

    float getNormalDotProduct(const SpectralProfile & other)
    {
      return fabs(normal(0) * other.normal(0) + normal(1) * other.normal(1) + normal(2) * other.normal(2));
    }

    float getInnerness(const SpectralProfile & other)
    {
      float r1 = sqrt(centroid.x * centroid.x + centroid.y * centroid.y);
      float r2 = sqrt(other.centroid.x * other.centroid.x + other.centroid.y * other.centroid.y);
      return r1 - r2;
    }

    void pushHogDiffFeats(const SpectralProfile & other, std::vector<float> & feats)
    {
      avgHOGFeatsOfSegment.pushBackAllDiffFeats(other.avgHOGFeatsOfSegment, feats);
    }

    float getCoplanarity(const SpectralProfile & other)
    {
      float dotproduct = getNormalDotProduct(other);
      // if the segments are coplanar return the displacement between centroids in the direction of the normal
      if (fabs(dotproduct) > 0.9) {
        float distance = (centroid.x - other.centroid.x) * normal[0] + (centroid.y - other.centroid.y) * normal[1] + (centroid.z - other.centroid.z) * normal[2];
        if (distance == 0 || fabs(distance) < (1 / 1000)) {
          return 1000;
        }
        return fabs(1 / distance);
      } else // else return -1
        return -1;
    }

    int getConvexity(const SpectralProfile & other, float mindistance)
    {
      VectorG centroid1(centroid.x, centroid.y, centroid.z);
      VectorG centroid2(other.centroid.x, other.centroid.y, other.centroid.z);

      VectorG c1c2 = centroid2.subtract(centroid1);
      VectorG c2c1 = centroid1.subtract(centroid2);
      VectorG normal1(normal[0], normal[1], normal[2]);
      VectorG normal2(other.normal[0], other.normal[1], other.normal[2]);
      // refer local convexity criterion paper
      if (mindistance < 0.04 && ((normal1.dotProduct(c1c2) <= 0 && normal2.dotProduct(c2c1) <= 0) || fabs(normal1.dotProduct(normal2)) > 0.95))  {
        return 1;
      }
      // else return 0
      return 0;
    }
};

// =====================================================================================
//        Class:  BinStumps
//  Description:
// =====================================================================================
class BinStumps
{
  public:
    static const int NUM_BINS = 10;
    double binStumps[NUM_BINS];

    BinStumps(string line)
    {
      boost::char_separator<char> sep("\t");
      boost::tokenizer<boost::char_separator<char> > tokens(line, sep);
      int count = 0;

      BOOST_FOREACH(string t, tokens) {
        binStumps[count] = (boost::lexical_cast<double>(t.data()));
        count++;
      }
      assert(count == NUM_BINS);
    }

    void writeBinnedValues(double value, std::ofstream & file, int featIndex)
    {
      int binv, bindex;
      for (int i = 0; i < NUM_BINS; i++) {
        binv = 0;
        if (value <= binStumps[i])
          binv = 1;
        bindex = featIndex * NUM_BINS + i + 1;
        file << " " << bindex << ":" << binv;
      }
    }

    void storeBinnedValues(double value, Eigen::Matrix<float, Eigen::Dynamic, 1> & mat, int featIndex)
    {
      int binv, bindex;
      for (int i = 0; i < NUM_BINS; i++) {
        binv = 0;
        if (value <= binStumps[i])
          binv = 1;
        bindex = featIndex * NUM_BINS + i;
        mat(bindex) = binv;
      }
    }

    void print()
    {
        for (int i = 0; i < NUM_BINS; i++)
            cout << "," << binStumps[i];
        cout << endl;
    }
}; // end of class BinStumps

// =====================================================================================
//        Class:  BinningInfo
//  Description:
// =====================================================================================
class BinningInfo {
    float max;
    float min;
    int numBins;
    float binSize;

  public:
    BinningInfo(float min_, float max_, int numBins_)
    {
      max = max_;
      min = min_;
      numBins = numBins_;
      assert(max > min);
      binSize = (max - min) / numBins;
    }

    int getBinIndex(float value)
    {
      assert(value >= min);
      assert(value <= max);

      int bin = ((value - min) / binSize);
      assert(bin <= numBins);

      if (bin == numBins) {
        bin = numBins - 1;
      }
      return bin;
    }

    float GetBinSize() const
    {
      return binSize;
    }

    int GetNumBins() const
    {
      return numBins;
    }

    float GetMin() const
    {
      return min;
    }

    float GetMax() const
    {
        return max;
    }
}; // end of class BinningInfo

// =====================================================================================
//        Class:  OriginalFrameInfo
//  Description:
// =====================================================================================
class OriginalFrameInfo {
  private:
    HOG hogDescriptors;
    TransformG cameraTrans;
    bool cameraTransSet;

  public:
    PointCloudT RGBDSlamFrame; // required to get 2D pixel positions
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void saveImage(int segmentId, int label, std::vector<Point2DAbhishek>points)
    {
      CvSize size;
      size.height = 480;
      size.width = 640;
      IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);

      PointT tmp;
      for (int x = 0; x < size.width; x++) {
        for (int y = 0; y < size.height; y++) {
          int index = x + y * size.width;
          tmp = RGBDSlamFrame.points[index];
          ColorRGB tmpColor(tmp.rgb);
          CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
          CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
          CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;
        }
      }

      ColorRGB tmpColor(0.0, 1.0, 0.0);
      for (size_t i = 0; i < points.size(); i++) {
        int x = points[i].x;
        int y = points[i].y;

        CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
        CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
        CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;
      }

      char filename[30];
      sprintf(filename, "%ss%d_l%d.png", PCDDIR.c_str(), segmentId, label);
      HOG::saveFloatImage(filename, image);
      cvReleaseImage(&image);
    }

    OriginalFrameInfo(PointCloudT::Ptr RGBDSlamFrame_)
    {
      cameraTransSet = false;
      RGBDSlamFrame = *RGBDSlamFrame_;
      CvSize size;
      size.height = 480;
      size.width = 640;
      ROS_INFO("RGBslam size: %d", RGBDSlamFrame.size());
      // can be 0 for dummy pcds of manually transformed
      if (RGBDSlamFrame.size() == 0)
        return;

      // can be 0 for dummy pcds of manually transformed
      assert((int)RGBDSlamFrame.size() == size.width * size.height);

      IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);
      PointT tmp;
      for (int x = 0; x < size.width; x++) {
        for (int y = 0; y < size.height; y++) {
          int index = x + y * size.width;
          tmp = RGBDSlamFrame.points[index];
          ColorRGB tmpColor(tmp.rgb);
          CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
          CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
          CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;
        }
      }

      hogDescriptors.computeHog(image);
      cvReleaseImage(&image);
    }

    static Point2DAbhishek getPixelFromIndex(int index)
    {
      //assuming size is 640*480;
      int width = 640;
      Point2DAbhishek ret;
      ret.y = index / width;
      ret.x = index % width;
      assert(index == ret.x + ret.y * width);
      return ret;
    }

    static void findHog(std::vector<size_t> &pointIndices, pcl::PointCloud<PointT> &incloud, HOGFeaturesOfBlock &hogSegment, OriginalFrameInfo* &targetFrame)
    {
      assert(targetFrame->RGBDSlamFrame.size() > 0);
      assert(targetFrame->cameraTransSet);

      std::vector<Point2DAbhishek> pointsInImageLyingOnSegment;
      for (size_t i = 0; i < pointIndices.size(); i++)
        pointsInImageLyingOnSegment.push_back(getPixelFromIndex(pointIndices[i]));

      assert(pointsInImageLyingOnSegment.size() > 0);
      targetFrame->hogDescriptors.getFeatValForPixels(pointsInImageLyingOnSegment, hogSegment);
    }

    void setCameraTrans(TransformG cameraTrans)
    {
      this->cameraTrans = cameraTrans;
      cameraTransSet = true;
    }

    void applyPostGlobalTrans(TransformG globalTrans)
    {
      if (cameraTransSet)
        cameraTrans = cameraTrans.preMultiply(globalTrans);
    }

    TransformG getCameraTrans() const
    {
      assert(cameraTransSet);
      return cameraTrans;
    }

    void setCameraTransSet(bool cameraTransSet)
    {
      this->cameraTransSet = cameraTransSet;
    }

    bool isCameraTransSet() const
    {
      return cameraTransSet;
    }

    bool isEmpty() const
    {
      return RGBDSlamFrame.size() == 0;
    }

}; // end of class OriginalFrameInfo

// fix me here
//----------------------------------------------------------------------
//  GLOBAL DEFINE
//----------------------------------------------------------------------
std::vector<BinStumps> nodeFeatStumps;
std::vector<BinStumps> edgeFeatStumps;
OriginalFrameInfo *originalFrame;
int NUM_ASSOCIATIVE_FEATS = 4 + 1;
double currentAngle = 0;

bool UseVolFeats = false;
bool BinFeatures = true;
bool addNodeHeader = true;
bool addEdgeHeader = true;

typedef std::vector<std::vector<SpectralProfile, Eigen::aligned_allocator<SpectralProfile> > > SpectralProfileVectorVector;
PointCloudVector cloudVector;
SpectralProfileVectorVector spectralProfilesVector;
std::vector<PointCloudVector > segment_cloudsVector;
std::vector<int> sceneNumVector;
std::vector<std::map<int, int> > segIndex2LabelVector;

// ===  FUNCTION  ======================================================================
//         Name:  buildOctoMap
//  Description:
// =====================================================================================
void buildOctoMap(const pcl::PointCloud<PointT> &cloud, octomap::OcTreeROS & tree)
{
  PointCloudT::Ptr cloud_ptr(new PointCloudT(cloud));
  PointCloudT::Ptr cloud_cam(new PointCloudT());

  // convert to  pointXYZ format
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::toROSMsg(*cloud_ptr, cloud_blob);
  pcl::PointCloud<pcl::PointXYZ> xyzcloud;
  pcl::fromROSMsg(cloud_blob, xyzcloud);

  // find the camera co-ordinate
  VectorG cam_coordinates = originalFrame->getCameraTrans().getOrigin();
  pcl::PointXYZ origin(cam_coordinates.v[0], cam_coordinates.v[1], cam_coordinates.v[2]);
  // insert to the tree
  tree.insertScan(xyzcloud, origin, -1, true);
}

// ===  FUNCTION  ======================================================================
//         Name:  parseAndApplyLabels
//  Description:
// =====================================================================================
void parseAndApplyLabels(std::ifstream &file,
                         PointCloudT &cloud,
                         PointCloudVector &segment_clouds,
                         map<int, int> &segIndex2label)
{
  string tokens[3];
  boost::char_separator<char> sep1(" ");
  boost::char_separator<char> sep2(":");
  std::string line;
  getline(file, line);
  int count;
  boost::tokenizer<boost::char_separator<char> > tokens1(line, sep1);
  map<int, int> segId2label;
  foundAny = false;
  BOOST_FOREACH(string t, tokens1) {
    count = 0;
    boost::tokenizer<boost::char_separator<char> > tokens2(t, sep2);
    BOOST_FOREACH(string t2, tokens2) {
      assert(count < 3);
      tokens[count] = t2;
      count++;
    }

    int segmentIndex = (boost::lexical_cast<int>(tokens[0])) - 1;
    int segmentId = segment_clouds[segmentIndex].points[1].segment;
    int label = boost::lexical_cast<int>(tokens[1]);
    segId2label[segmentId] = label;
    segIndex2label[segmentIndex] = label;
    if (!labelsFound.test(label-1)){
      labelsFound.set(label-1, true);
      foundAny = true;
      ROS_INFO("Found label %d: in scene %d at: %lf.", label-1, sceneNumVector.back(), currentAngle);
    }
  }

  for (size_t i = 0; i < cloud.size(); i++)
    cloud.points[i].label = invLabelMap[segId2label[cloud.points[i].segment]];
}


// ===  FUNCTION  ======================================================================
//         Name:  getFeatureHistogram
//  Description:
// =====================================================================================
void getFeatureHistogram(std::vector<std::vector<float> > &descriptor_results,
                           std::vector< std::vector<float> > &result,
                           std::vector<BinningInfo> &binningInfos)
{
  std::vector<std::vector<float> >::iterator it = descriptor_results.begin();
  int numFeats = it->size();
  // set size of result vector
  result.resize(numFeats);

  std::vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
  for (std::vector<std::vector<float> >::iterator ires = result.begin(); ires < result.end(); ires++, binningInfo++) {
    ires->resize(binningInfo->GetNumBins());
  }

  // fill the histogram
  for (std::vector<std::vector<float> >::iterator it_point = descriptor_results.begin(); it_point < descriptor_results.end(); it_point++) {
    // iterate over points
    std::vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
    std::vector<std::vector<float> >::iterator ires = result.begin();
    //missing features NOT allowed for now.
    assert(numFeats == (int)it_point->size());

    for (std::vector<float>::iterator it_feature = it_point->begin(); it_feature < it_point->end(); it_feature++, binningInfo++, ires++) {
      // iterate over features of the point
      int bin = binningInfo->getBinIndex(*it_feature);
      (*ires)[bin] += 1;
    }
  }

  // normalize and print histogram
  int numPoints = descriptor_results.size();
  int c1 = 0, c2 = 0;
  for (std::vector< std::vector<float> >::iterator i = result.begin(); i < result.end(); i++) {
    c1++;
    for (std::vector<float>::iterator i2 = i->begin(); i2 < i->end(); i2++) {
      c2++;
      *i2 = *i2 / numPoints;
      assert(*i2 <= 1.0);
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  getFeatureAverage
//  Description:
// =====================================================================================
void getFeatureAverage(std::vector<std::vector<float> > &descriptor_results, std::vector<float> &avg_feats)
{
  std::vector<std::vector<float> >::iterator it = descriptor_results.begin();
  while (it->size() == 0) it++;
  avg_feats.resize(it->size());

  int count = 0;
  for (std::vector<std::vector<float> >::iterator it = descriptor_results.begin(); it < descriptor_results.end(); it++) {
    if (it->size() > 0)
      count++;
    std::vector<float>::iterator i = avg_feats.begin();
    for (std::vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++, i++)
      *i = *i + *it2;
  }

  int c = 0;
  for (std::vector<float>::iterator i = avg_feats.begin(); i < avg_feats.end(); i++) {
    c++;
    *i = *i / count;
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  concatFeats
//  Description:  concat feats vector to features vector
// =====================================================================================
void concatFeats(std::vector<float> &features, std::vector<float> &feats)
{
  for (std::vector<float>::iterator it = feats.begin(); it < feats.end(); it++)
    features.push_back(*it);
}

void concatFeats(vector<float> &features, vector<vector<float> > &feats)
{
  for (vector<vector<float> >::iterator it = feats.begin(); it < feats.end(); it++) {
    for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++) {
      features.push_back(*it2);
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  addToNodeHeader
//  Description:
// =====================================================================================
inline void addToNodeHeader(std::string featName, size_t numTimes = 1)
{
  if (addNodeHeader) {
    for (size_t i = 0; i < numTimes; i++) {
      nodeFeatNames.push_back(featName + boost::lexical_cast<std::string > (i));
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  addToEdgeHeader
//  Description:
// =====================================================================================
inline void addToEdgeHeader(std::string featName, size_t numTimes = 1)
{
  if (addEdgeHeader) {
    for (size_t i = 0; i < numTimes; i++) {
      edgeFeatNames.push_back(featName + boost::lexical_cast<std::string > (i));
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  getColorFeatures
//  Description:
// =====================================================================================
void getColorFeatures(const pcl::PointCloud<PointT> &cloud, std::vector<float> &features, SpectralProfile & spectralProfileOfSegment)
{
  int num_bin_H = 6;
  int num_bin_S = 2;
  int num_bin_V = 2;

  // histogram and average of hue and intensity
  std::vector<std::vector<float> > hist_features;
  std::vector<float> avg_features;
  vector < vector <float> > color_features(cloud.points.size());
  vector <vector <float> >::iterator it = color_features.begin();
  for (size_t i = 0; i < cloud.points.size(); ++i, it++) {
    ColorRGB c(cloud.points[i].rgb);
    (*it).push_back(c.H);
    (*it).push_back(c.S);
    (*it).push_back(c.V);
  }

  std::vector<BinningInfo> binnigInfos;
  binnigInfos.push_back(BinningInfo(0, 360, num_bin_H));
  binnigInfos.push_back(BinningInfo(0, 1, num_bin_S));
  binnigInfos.push_back(BinningInfo(0, 1, num_bin_V));
  getFeatureHistogram(color_features, hist_features, binnigInfos);
  getFeatureAverage(color_features, avg_features);

  spectralProfileOfSegment.avgH = avg_features[0];
  spectralProfileOfSegment.avgS = avg_features[1];
  spectralProfileOfSegment.avgV = avg_features[2];

  concatFeats(features, hist_features);
  addToNodeHeader("H_hist", num_bin_H);
  addToNodeHeader("S_hist", num_bin_S);
  addToNodeHeader("V_hist", num_bin_V);

  concatFeats(features, avg_features);
  addToNodeHeader("HAvg");
  addToNodeHeader("SAvg");
  addToNodeHeader("VAvg");
}

// =====================================================================================
//        Class:  getSpectralProfile
//  Description:
// =====================================================================================
void getSpectralProfile(const pcl::PointCloud<PointT> &cloud, SpectralProfile &spectralProfile)
{
  Eigen::Matrix3f eigen_vectors;
  Eigen::Vector3f eigen_values;

  Eigen::Matrix3f covariance_matrix;
  computeCovarianceMatrix(cloud, spectralProfile.centMatrix, covariance_matrix);
  for (unsigned int i = 0 ; i < 3 ; i++)
    for (unsigned int j = 0 ; j < 3 ; j++)
      covariance_matrix(i, j) /= static_cast<double> (cloud.points.size ());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  eigen_values = ei_symm.eigenvalues ();
  eigen_vectors = ei_symm.eigenvectors ();

  spectralProfile.setEigValues(eigen_values);
  float minEigV = FLT_MAX;

  for (int i = 0; i < 3; i++) {
    if (minEigV > eigen_values(i)) {
      minEigV = eigen_values(i);
      spectralProfile.normal = eigen_vectors.col(i);
      VectorG centroid(spectralProfile.centroid.x, spectralProfile.centroid.y, spectralProfile.centroid.z);
      VectorG camera = originalFrame->getCameraTrans().getOrigin();
      VectorG cent2cam = camera.subtract(centroid);
      VectorG normal(spectralProfile.normal[0], spectralProfile.normal[1], spectralProfile.normal[2]);
      if (normal.dotProduct(cent2cam) < 0) {
        spectralProfile.normal[0] = -spectralProfile.normal[0];
        spectralProfile.normal[1] = -spectralProfile.normal[1];
        spectralProfile.normal[2] = -spectralProfile.normal[2];
      }
    }
  }
  assert(minEigV == spectralProfile.getDescendingLambda(2));
}

// =====================================================================================
//        Class:  getGlobalFeatures
//  Description:
// =====================================================================================
void getGlobalFeatures(const pcl::PointCloud<PointT> &cloud,
                       vector<float> &features,
                       SpectralProfile & spectralProfileOfSegment)
{
  Eigen::Vector4f min_p;
  Eigen::Vector4f max_p;

  // get bounding box features
  getSpectralProfile(cloud, spectralProfileOfSegment);

  getMinMax(cloud, min_p, max_p);
  float xExtent = max_p[0] - min_p[0];
  float yExtent = max_p[1] - min_p[1];
  float horizontalExtent = sqrt(xExtent * xExtent + yExtent * yExtent);
  float zExtent = max_p[2] - min_p[2];

  features.push_back(horizontalExtent);
  addToNodeHeader("horizontalExtent");

  features.push_back(zExtent);
  addToNodeHeader("zExtent");

  features.push_back(spectralProfileOfSegment.centroid.z);
  addToNodeHeader("centroid_z");

  features.push_back(spectralProfileOfSegment.getNormalZComponent());
  addToNodeHeader("normal_z");

  //spectral saliency features
  features.push_back((spectralProfileOfSegment.getLinearNess()));
  addToNodeHeader("linearness");
  features.push_back((spectralProfileOfSegment.getPlanarNess()));
  addToNodeHeader("planarness");
  features.push_back((spectralProfileOfSegment.getScatter()));
  addToNodeHeader("scatter");
  spectralProfileOfSegment.avgHOGFeatsOfSegment.pushBackAllFeats(features);
  addToNodeHeader("HOG", 31);
}


// ===  FUNCTION  ======================================================================
//         Name:  addDistanceFeatures
//  Description:
// =====================================================================================
void addDistanceFeatures(const pcl::PointCloud<PointT> &cloud, std::map< int,std::vector<float> > &features)
{
  std::map<int,float> segment_boundary_distance;
  getSegmentDistanceToBoundary(cloud,segment_boundary_distance);
  for(std::map<int,float>::iterator it = segment_boundary_distance.begin(); it != segment_boundary_distance.end(); it++ ) {
    int segid = (*it).first;
    features[segid].push_back((*it).second);
  }
}


// ===  FUNCTION  ======================================================================
//         Name:  getOccupancyFeature
//  Description:
// =====================================================================================
float getOccupancyFeature(const PointCloudT &cloud1, const PointCloudT &cloud2, octomap::OcTreeROS & tree)
{
  octomap::OcTreeROS::NodeType* treeNode;
  int occCount = 0;
  int totalCount = 0;
  int unknownCount = 0;
  for (int i = 0; i < 100; i++) {
    int p1Index = rand() % cloud1.points.size();
    int p2Index = rand() % cloud2.points.size();
    VectorG p1(cloud1.points[p1Index]);
    VectorG p2(cloud2.points[p2Index]);
    double distance = (p2.subtract(p1)).getNorm();
    int count = 0;
    for (double r = 0.05; r <= distance - 0.05; r += 0.05) {
      count++;
      VectorG point = p1.add((p2.subtract(p1).normalizeAndReturn()).multiply(r));
      pcl::PointXYZ pt(point.v[0], point.v[1], point.v[2]);
      treeNode = tree.search(pt);
      if (treeNode) {
        if (treeNode->getOccupancy() > 0.5)
          occCount++;
      }else {
        unknownCount++;
      }
    }

    if (count == 0) {
      VectorG point = p1.add(p2.subtract(p1).multiply(0.5));
      pcl::PointXYZ pt(point.v[0], point.v[1], point.v[2]);
      treeNode = tree.search(pt);
      if (treeNode) {
        if (treeNode->getOccupancy() > 0.5)
          occCount++;
      }else {
        unknownCount++;
      }
      count++;
    }
    totalCount += count;
  }

  ROS_INFO("seg1:%d, label1:%d, seg2:%d, label2:%d.", cloud1.points[1].segment, cloud1.points[1].label, cloud2.points[1].segment, cloud2.points[1].label);
  ROS_INFO("total:%d, unknown:%d, occupied:%d.",totalCount, unknownCount, occCount);
  return (float) unknownCount / (float) totalCount;
}

// ===  FUNCTION  ======================================================================
//         Name:  getPairFeatures
//  Description:
// =====================================================================================
void getPairFeatures(int segment_id,
                    std::vector<int> &neighbor_list,
                    std::map<std::pair <int, int>, float > &distance_matrix,
                    std::map<int, int> &segment_num_index_map,
                    std::vector<SpectralProfile, Eigen::aligned_allocator<SpectralProfile> > &spectralProfiles,
                    std::map <int, std::vector<float> > &edge_features,
                    octomap::OcTreeROS & tree)
{
  SpectralProfile segment1Spectral = spectralProfiles[segment_num_index_map[segment_id]];

  for (std::vector<int>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); it++) {
    int seg2_id = *it;
    SpectralProfile segment2Spectral = spectralProfiles[segment_num_index_map[seg2_id]];

    //here goes the associative features:
    edge_features[seg2_id].push_back(segment1Spectral.getHDiffAbs(segment2Spectral));
    addToEdgeHeader("HDiffAbs");

    edge_features[seg2_id].push_back(segment1Spectral.getSDiff(segment2Spectral));
    addToEdgeHeader("SDiff");

    edge_features[seg2_id].push_back(segment1Spectral.getVDiff(segment2Spectral));
    addToEdgeHeader("Viff");

    edge_features[seg2_id].push_back(segment1Spectral.getCoplanarity(segment2Spectral));
    addToEdgeHeader("Coplanarity");

    edge_features[seg2_id].push_back(segment1Spectral.getConvexity(segment2Spectral, distance_matrix[make_pair(segment_id, seg2_id)]));
    addToEdgeHeader("convexity");

    assert((int)edge_features[seg2_id].size() == NUM_ASSOCIATIVE_FEATS);

    //here goes the non-associative features
    edge_features[seg2_id].push_back(segment1Spectral.getHorzDistanceBwCentroids(segment2Spectral));
    addToEdgeHeader("centroid_horz_diff");
    // difference in z coordinates of the centroids
    edge_features[seg2_id].push_back(segment1Spectral.getVertDispCentroids(segment2Spectral));
    addToEdgeHeader("centroid_z_diff");

    // distance between closest points
    edge_features[seg2_id].push_back(distance_matrix[make_pair(segment_id, seg2_id)]);
    addToEdgeHeader("dist_closest");

    // difference of angles with vertical
    edge_features[seg2_id].push_back(segment1Spectral.getAngleDiffInRadians(segment2Spectral));
    addToEdgeHeader("AngleDiff");

    // dot product of normals
    edge_features[seg2_id].push_back(segment1Spectral.getNormalDotProduct(segment2Spectral));
    addToEdgeHeader("NormalDotProduct");

    // inner product of normals
    edge_features[seg2_id].push_back(segment1Spectral.getInnerness(segment2Spectral));
    addToEdgeHeader("Innerness");

    // occupancy fraction feature
    if (UseVolFeats) {
      edge_features[seg2_id].push_back(getOccupancyFeature(*(segment1Spectral.cloudPtr), *(segment2Spectral.cloudPtr), tree));
      addToEdgeHeader("Occupancy");
    }

    // this line should be in the end
    addEdgeHeader = false;
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  writeFeats
//  Description:
// =====================================================================================
void writeFeatures(PointCloudT::Ptr & cloud_ptr, int scene_num)
{
  size_t counts[640 * 480];
  std::ofstream featfile;
  std::string featfilename = "data_scene_labelling_full_" + boost::lexical_cast<string > (scene_num);
  featfile.open((TMPDIR + featfilename).data());

  octomap::OcTreeROS tree(0.01);
  if (UseVolFeats) {
  //    octomap::OcTreeROS::NodeType* treeNode;
    buildOctoMap(*cloud_ptr, tree);
  }

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());
  pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
  PointCloudVector segment_clouds;
  std::map<int, int> segment_num_index_map;
  std::vector<size_t> segment_indices;

  // find the max segment number
  int max_segment_num = 0;
  for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
    counts[cloud_ptr->points[i].segment]++;
    if (max_segment_num < (int)cloud_ptr->points[i].segment)
      max_segment_num = (int) cloud_ptr->points[i].segment;
  }

  //  pcl::ExtractIndices<PointT> extract;
  int index_ = 0;
  std::vector<SpectralProfile , Eigen::aligned_allocator<SpectralProfile> > spectralProfiles;
  ROS_INFO("max_seg num: %d of %d.", max_segment_num, cloud_ptr->points.size());
  for (int seg = 1; seg <= max_segment_num; seg++) {
    if(counts[seg]<=MIN_SEGMENT_SIZE)
      continue;
    SpectralProfile temp;
    applySegmentFilter(*cloud_ptr, seg, *cloud_seg, segment_indices);
    if(MIN_SEGMENT_SIZE < (int)segment_indices.size())
      OriginalFrameInfo::findHog(segment_indices, *cloud_ptr, temp.avgHOGFeatsOfSegment, originalFrame);
    else cloud_seg->points.clear();

    if (!cloud_seg->points.empty() && (int)cloud_seg->points.size() > MIN_SEGMENT_SIZE) {
      segment_clouds.push_back(*cloud_seg);
      PointCloudT::Ptr tempPtr(new pcl::PointCloud<PointT > (segment_clouds[segment_clouds.size() - 1]));
      temp.cloudPtr = tempPtr;

      spectralProfiles.push_back(temp);
      PointT tmp_point = cloud_seg->points[1];
      segment_num_index_map[tmp_point.segment] = index_;
      index_++;
    }
  }

  std::map<pair<int, int>, float> distance_matrix;
  std::map<int, std::vector<int> > neighbor_map;
  ROS_INFO("computing neighbores");
  clock_t start_time = clock();
  int num_edges = getNeighbors(segment_clouds, distance_matrix, neighbor_map);
  clock_t elapsed = clock() - start_time;
  ROS_INFO("computing neighbores %lf ms.", elapsed/((double)CLOCKS_PER_SEC));

  // for each segment compute node featuers
  std::map <int, std::vector<float> > features;
  bool isFirstFrame = addNodeHeader;
  for (size_t i = 0; i < segment_clouds.size(); i++) {
    int seg_id = segment_clouds[i].points[1].segment;
    // computing color features
    getColorFeatures(segment_clouds[i], features[seg_id], spectralProfiles[i]);
    // get bounding box and centroid point features
    getGlobalFeatures(segment_clouds[i], features[seg_id], spectralProfiles[i]);
    addNodeHeader = false;
  }
  ROS_INFO("adding wall distance features");
  start_time = clock();
  addDistanceFeatures(*cloud_ptr, features, segment_clouds);
  if(isFirstFrame) nodeFeatNames.push_back("distance_from_wall0");
  elapsed = clock() - start_time;
  ROS_INFO("time for computing wall %lf", elapsed/((double) CLOCKS_PER_SEC));
  ROS_INFO("done adding wall distance features");

  int totatAssocFeats = NUM_ASSOCIATIVE_FEATS;
  if (BinFeatures)
    totatAssocFeats = NUM_ASSOCIATIVE_FEATS * BinStumps::NUM_BINS;
  //should not matter ... it is read from modelfile
  featfile << segment_clouds.size() << " " << num_edges << " " << 17 << " " << totatAssocFeats << std::endl;

  for (std::map< int, std::vector<float> >::iterator it = features.begin(); it != features.end(); it++) {
    assert(nodeFeatNames.size() == (*it).second.size());
    featfile << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << " " << (*it).first;
    int featIndex = 0;
    for (std::vector<float>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++) {
      if (BinFeatures) {
        nodeFeatStumps[featIndex].writeBinnedValues(*it2, featfile, featIndex);
      }else {
        featfile << " " << featIndex << ":" << *it2;
      }
      featIndex++;
    }
    featfile << "\n";
  }

  std::map <int, std::vector<float> > edge_features;
  int edgecount = 0;
  for (std::map<int, std::vector<int> >::iterator it = neighbor_map.begin(); it != neighbor_map.end(); it++) {
    edge_features.clear();
    getPairFeatures((*it).first, (*it).second, distance_matrix, segment_num_index_map, spectralProfiles, edge_features, tree);
    edgecount++;

    // print pair-wise features
    for (std::map< int, std::vector<float> >::iterator it2 = edge_features.begin(); it2 != edge_features.end(); it2++) {
      featfile << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << " " << segment_clouds[segment_num_index_map[(*it2).first]].points[1].label << " " << (*it).first << " " << (*it2).first;
      assert(edgeFeatNames.size() == (*it2).second.size());

      int featIndex = 0;
      for (std::vector<float>::iterator it3 = (*it2).second.begin(); it3 != (*it2).second.end(); it3++) {
        if (BinFeatures) {
          edgeFeatStumps[featIndex].writeBinnedValues(*it3, featfile, featIndex);
        }else {
          featfile << " " << featIndex << ":" << *it3;
        }
        featIndex++;
      }
      featfile << std::endl;
    }
  }
  featfile.close();

  featfile.open((TMPDIR + "temp." + featfilename).data());
  featfile << TMPDIR + featfilename;
  featfile.close();
//  std::string command = "./toolkit/svm-python-v204/svm_python_classify --m svmstruct_mrf --l micro --lm nonassoc --cm sumLE1.IP --omf ./toolkit/svm-python-v204/"
//                  + ENVIRONMENT + "_objectMap.txt temp." + featfilename + " ./toolkit/svm-python-v204/"
//                  + ENVIRONMENT + "Model pred." + featfilename + " > out." + featfilename;
  std::string instruct = WORKDIR + "svm-python-v204/svm_python_classify --m svmstruct_mrf --l micro --lm nonassoc --cm sumLE1.IP --omf ";
  std::string tmpfeat = WORKDIR + "svm-python-v204/" + ENVIRONMENT + "_objectMap.txt " + TMPDIR + "temp." + featfilename+" ";
  std::string prefeat = WORKDIR + "svm-python-v204/" + ENVIRONMENT + "Model " + TMPDIR + "pred." + featfilename;
  std::string outfeat = " > " + TMPDIR + "out." + featfilename;
  std::string command = instruct + tmpfeat + prefeat + outfeat;
  int resultVal = system(command.data());
  ROS_INFO("Running svm classifier ... %d", resultVal);

  std::ifstream predLabels;
  predLabels.open((TMPDIR + "pred." + featfilename).data()); // open the file containing predictions
  std::map<int, int> segIndex2Label;

  pcl::PCDWriter writer;
  cloudVector.push_back(*cloud_ptr);
  spectralProfilesVector.push_back(spectralProfiles);
  segment_cloudsVector.push_back(segment_clouds);
  sceneNumVector.push_back(scene_num);
  parseAndApplyLabels(predLabels, *cloud_ptr, segment_clouds, segIndex2Label);
  segIndex2LabelVector.push_back(segIndex2Label);
  predLabels.close();
  writer.write<PointT > (PCDDIR + featfilename + ".pcd", *cloud_ptr, true);
  sensor_msgs::PointCloud2 cloudMsg;
  toROSMsg(*cloud_ptr, cloudMsg);
  pub.publish(cloudMsg);
}

#endif
