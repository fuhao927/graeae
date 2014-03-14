/*
 * File:   wallDistance.h
 * Author: aa755
 *
 * Created on June 20, 2011, 6:18 PM
 */

#ifndef WALLDISTANCE_H
#define WALLDISTANCE_H

class Point2DGeo
{
  public:
    double x;
    double y;
    Point2DGeo(double x_,double y_)
    {
      x=x_;
      y=y_;
    }
    Point2DGeo()
    {
    }

    Point2DGeo subtract(Point2DGeo rhs)
    {
      return Point2DGeo(x-rhs.x,y-rhs.y);
    }

    double toDegress(double rad)
    {
      return rad*180.0/PI;
    }

    double angle()
    {
      double principalAngle=toDegress(atan(y/x));
      double fullAngle;
      if(x>=0)
        fullAngle= principalAngle;
      else
        fullAngle= principalAngle+180.0;

      if(fullAngle<0.0)
        fullAngle+=360.0;

      assert(fullAngle>=0.0);
      assert(fullAngle<360.0);
      return fullAngle;
    }

    double norm()
    {
      return sqrt(x*x+y*y);
    }

    int principleRange(int angle)
    {
      return (angle % 360);
    }

    vector<int>  getAdjacentAngles(int angle,int ticks)
    {
      vector<int> ret;

      for(int i=0;i<ticks;i++)
        ret.push_back(principleRange(angle+i));

      // 0 is already covered => so starting from 1
      for(int i=1;i<ticks;i++)
        ret.push_back(principleRange(angle-i));

      return ret;
    }
};

std::pair<int,double> get2DAngleDegreesAndDistance(PointT  point)
{
    Point2DGeo point2D(point.x,point.y);
    Point2DGeo ray2D=point2D;//-origin2D;
    return pair<int,double>((int)ray2D.angle(),ray2D.norm());
}

void getMaxRanges(double * maxDist, const pcl::PointCloud<PointT> &cloud)
{
  for(int angle=0;angle<360;angle++) {
    maxDist[angle]=0;
  }

  //get the max distance to boundary along each direction
  for(size_t i=1;i<cloud.size();i++) {
    if(isnan( cloud.points[i].x))
      continue;
    std::pair<int,double>angDist= get2DAngleDegreesAndDistance(cloud.points[i]/*,camera*/);
    int angle=angDist.first;
    double distance=angDist.second;
    if(maxDist[angle]<distance)
      maxDist[angle]=distance;
  }

}

double getWallDistanceCent(double * maxDist,PointT  p)
{
  pair<int,double>angDist= get2DAngleDegreesAndDistance(p);
  int angle=angDist.first;
  double dist=maxDist[angle]-angDist.second;
  return dist;
}

void getSegmentDistanceToBoundaryOptimized( const pcl::PointCloud<PointT> &cloud , map<int,float> &segment_boundary_distance,PointCloudVector & segment_clouds)
{
  double maxDist[360];
  getMaxRanges(maxDist,cloud);

  for(size_t i=0;i<segment_clouds.size();i++) {
    double distBoundary=0;
    double dist;
    pcl::PointCloud<PointT> & segment= segment_clouds[i];
    for(size_t j=1;j<segment.size();j++) {
      std::pair<int,double>angDist= get2DAngleDegreesAndDistance(segment.points[j]/*,camera*/);
      int angle=angDist.first;
      dist=maxDist[angle]-angDist.second;
      assert(dist>=0);
      distBoundary+=dist;
      segment.points[j].distance=dist;
    }
    segment_boundary_distance[segment.points[1].segment]=distBoundary/(segment.size()-1);
  }
}

void addDistanceFeatures(PointCloudT &cloud,
                        std::map<int,vector<float> >&features,
                        PointCloudVector& segment_clouds)
{
  bool outputDistancePCDSegmentWise=false;
  bool outputDistancePCDPointWise=false;
  map<int,float> segment_boundary_distance;
  getSegmentDistanceToBoundaryOptimized(cloud,segment_boundary_distance,segment_clouds);
  for(map<int,float>::iterator it = segment_boundary_distance.begin(); it != segment_boundary_distance.end(); it++ ) {
    int segid = (*it).first;
    features[segid].push_back((*it).second);
  }

  pcl::PCDWriter writer;
  if (outputDistancePCDSegmentWise) {
    pcl::PointCloud<PointT> temp;
    temp.header = cloud.header;
    for (size_t i = 0; i < cloud.size(); i++) {
      if (segment_boundary_distance.find(cloud.points[i].segment) != segment_boundary_distance.end()) {
        PointT tp = cloud.points[i];

        tp.distance = segment_boundary_distance[cloud.points[i].segment];
        temp.points.push_back(tp);
      }
    }
    writer.write<pcl::PointXYZRGBCamSL > ("data_scene_distance_seg.pcd", temp, true);
  }
  if (outputDistancePCDPointWise) {
    pcl::PointCloud<PointT> temp;
    temp.header = cloud.header;
    for (size_t i = 0; i < segment_clouds.size(); i++)
      temp+=segment_clouds[i];
    writer.write<pcl::PointXYZRGBCamSL > ("data_scene_distance_point.pcd", temp, true);
  }
}

#endif	/* WALLDISTANCE_H */

