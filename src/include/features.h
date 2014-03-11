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
      char_separator<char> sep("\t");
      tokenizer<char_separator<char> > tokens(line, sep);
      int count = 0;

      BOOST_FOREACH(string t, tokens) {
        binStumps[count] = (lexical_cast<double>(t.data()));
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

    void storeBinnedValues(double value, Matrix<float, Dynamic, 1 > & mat, int featIndex)
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
};

// need to fix here
vector<BinStumps> nodeFeatStumps;
vector<BinStumps> edgeFeatStumps;


// ===  FUNCTION  ======================================================================
//         Name:  readWeightVectors
//  Description:
// =====================================================================================
void readWeightVectors(std::vector<int> &nodeFeatures,
                       std::vector<int> &edgeFeatures,
                       Eigen::Matrix<float, Dynamic, Dynamic>* nw,
                       Eigen::Matrix<float, Dynamic, Dynamic>** ew);

// ===  FUNCTION  ======================================================================
//         Name:  readStumpValues
//  Description:
// =====================================================================================
void readStumpValues(vector<BinStumps> & featBins, const string & file);


#endif
