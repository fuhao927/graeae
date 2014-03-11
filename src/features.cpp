// =====================================================================================
//
//       Filename:  feature.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/12/2014 01:14:36 AM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "include/features.h"
#include "include/genericUtils.h"

// ===  FUNCTION  ======================================================================
//         Name:  readWeightVectors
//  Description:
// =====================================================================================
void readWeightVectors(std::vector<int> &nodeFeatures,
                       std::vector<int> &edgeFeatures,
                       Eigen::Matrix<float, Dynamic, Dynamic>* nw,
                       Eigen::Matrix<float, Dynamic, Dynamic>** ew)
{
  nodeFeatures.push_back(15);
  nodeFeatures.push_back(51);
  edgeFeatures.push_back(5);
  edgeFeatures.push_back(6);
  edgeFeatures.push_back(7);
  edgeFeatures.push_back(10);

  nw = new Eigen::Matrix<float, Dynamic, Dynamic > (NUM_CLASSES, 10 * nodeFeatures.size());
  readCSV("weights/node_weights.csv", NUM_CLASSES, 10 * nodeFeatures.size(), ",", *nw);
  for (size_t i = 0; i < NUM_CLASSES; i++) {
    ew[i] = new Matrix<float, Dynamic, Dynamic > (NUM_CLASSES, 10 * edgeFeatures.size());
    readCSV("weights/edge_weights_" + boost::lexical_cast<std::string > (i) + ".csv", NUM_CLASSES, 10 * edgeFeatures.size(), ",", *ew[i]);
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  readStumpValues
//  Description:
// =====================================================================================
void readStumpValues(vector<BinStumps> & featBins, const string & file)
{
  //    char lineBuf[1000]; // assuming a line is less than
  string line;
  ifstream myfile(file.data());

  if (!myfile.is_open()) {
    cerr << "cound not find the file:" << file << " which stores the ranges for binning the features .. you should run this program from the folder scene_processing(do roscd scene_processing), or put the binning info files: binStumpsN.txt(for node features) and binStumpsE.txt(for edge features) in current folder and rerun ... exiting with assertion failure" << endl;
  }
  assert(myfile.is_open());

  while (myfile.good()) {
    getline(myfile, line);
    if (line.length() > 0)
      featBins.push_back(BinStumps(line));
  }
  myfile.close();
}

