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
#include "global_define.h"
#include "features.h"

  int
main ( int argc, char *argv[] )
{
  readWeightVectors(nodeFeatIndices, edgeFeatIndices, nodeWeights, edgeWeights);
  ros::init(argc, argv, "Graeae");
  ENVIRONMENT = "office";
  if (argc > 1) ENVIRONMENT = argv[1];
  std::cout << "using evv= " << ENVIRONMENT << endl;
  ros::NodeHandle n;

  //Instantiate the kinect image listener
  readStumpValues(nodeFeatStumps, ENVIRONMENT + "/" + NODEBINFILE);
  readStumpValues(edgeFeatStumps, ENVIRONMENT + "/" + EDGEBINFILE);

  readLabelList(ENVIRONMENNT, labelsToFind, labelsToFindBitset);
  labelsFound = labelsToFindBitset;
  labelsFound.flip();


  return 0;
}				// ----------  end of function main  ----------
