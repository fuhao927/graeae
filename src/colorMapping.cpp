// =====================================================================================
//
//       Filename:  colorMapping.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  04/02/2014 01:14:36 AM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "include/point_types.h"
#include "include/global_define.h"
#include "include/color.h"
#include <fstream>


  int
main ( int argc, char *argv[] )
{
  bool colorLabels = true;
  bool saveColorPCD = true;
  if(argc<3){
    ROS_INFO("usage: run pointCloudFile label2color [NoLabels||SaveColoredPCD]");
    exit(-1);
  }
  if(argc==4&&argv[3][0]=='N')
    colorLabels=false;
  if(argc==4&&argv[3][0]=='S')
    saveColorPCD=false;

  std::map<int, int> color_mapping;
  std::ifstream in;
  in.open(argv[2]);
  if(in.is_open()) {
    int colorNum, labelNum;
    while(in.good()) {
      colorNum = 0;
      in >> colorNum >> labelNum;
      if(colorNum == 0)
        break;
      color_mapping[labelNum] = colorNum;
    }
  }else {
    ROS_INFO("could not open color mapping file...exiting\n");
    exit(-1);
  }

  int NUM_CLASSES_TO_SHOW = 12;
//  std::vector<ColorRGB> labelColors(NUM_CLASSES_TO_SHOW);
//  labelColors.push_back(*(new ColorRGB(1,0,0)));
//  labelColors.push_back(*(new ColorRGB(0,0,1)));
//  labelColors.push_back(*(new ColorRGB(1,1,0)));
//  labelColors.push_back(*(new ColorRGB(0,1,1)));
//  labelColors.push_back(*(new ColorRGB(1,0,1)));
//  labelColors.push_back(*(new ColorRGB(0.5,0,0)));
//  labelColors.push_back(*(new ColorRGB(0,0.5,0)));
//  labelColors.push_back(*(new ColorRGB(0,0,0.5)));
//  labelColors.push_back(*(new ColorRGB(0.5,0.5,0.0)));
//  labelColors.push_back(*(new ColorRGB(0.0,0.5,0.5)));
//  labelColors.push_back(*(new ColorRGB(0.5,0.0,0.5)));
  ColorRGB *labelColors[NUM_CLASSES_TO_SHOW];
  labelColors[0]= new ColorRGB(1,0,0);
  labelColors[1]= new ColorRGB(0,1,0);
  labelColors[2]= new ColorRGB(0,0,1);
  labelColors[3]= new ColorRGB(1,1,0);
  labelColors[4]= new ColorRGB(0,1,1);
  labelColors[5]= new ColorRGB(1,0,1);
  labelColors[6]= new ColorRGB(0.5,0,0);
  labelColors[7]= new ColorRGB(0,0.5,0);
  labelColors[8]= new ColorRGB(0,0,0.5);
  labelColors[9]= new  ColorRGB(0.5,0.5,0.0);
  labelColors[10]= new ColorRGB(0.0,0.5,0.5);
  labelColors[11]= new ColorRGB(0.5,0.0,0.5);


  PointCloudT cloud_labeled, cloud_colored;
  PointT p;
  if (pcl::io::loadPCDFile<PointT>(argv[1], cloud_labeled)) {
    ROS_ERROR("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO("Loaded %d data points from pcd", cloud_labeled.points.size());

  cloud_colored = cloud_labeled;
  for (size_t i = 0; i < cloud_labeled.points.size(); i++) {
    p = cloud_labeled.points[i];
    if(colorLabels && saveColorPCD && p.label>0 && color_mapping[p.label]>0) {
      ColorRGB tmpColor=*labelColors[color_mapping[p.label]-1];
      cloud_colored.points[i].rgb = tmpColor.getFloatRep();
    }
  }

  std::string filename = argv[1];
  if(saveColorPCD)
    pcl::io::savePCDFile(filename+"_color.pcd",cloud_colored);
  return 0;
}				// ----------  end of function main  ----------
