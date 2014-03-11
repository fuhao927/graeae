#ifndef GENERICUTILS_H
#define	GENERICUTILS_H
#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/dynamic_bitset.hpp>

// fix me here
template<typename _Scalar>
void readCSV(std::string filename, int height,int width, std::string separator,Eigen::Matrix<_Scalar,Eigen::Dynamic,  Eigen::Dynamic> & mat)
{
  std::ifstream file;
  file.open(filename.data(),std::ios::in);
  std::string line;
  boost::char_separator<char> sep(separator.data());
  for(int r=0;r<height;r++) {
    getline(file, line);
    boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
    int c=0;
    BOOST_FOREACH(std::string t, tokens1) {
      mat(r,c)=boost::lexical_cast<_Scalar>(t);
      c++;
    }
    assert(c==width);
  }
}

template<typename _Scalar>
void replace(Eigen::Matrix<_Scalar,Eigen::Dynamic,  Eigen::Dynamic> & mat, _Scalar find, _Scalar replace)
{
  for(int r=0;r<mat.rows();r++)
    for(int c=0;c<mat.cols();c++)
      if(mat(r,c)==find)
        mat(r,c)=replace;
}

static void saveFloatImage ( const char* filename, const IplImage * image )
{
  IplImage * saveImage = cvCreateImage (cvGetSize(image), IPL_DEPTH_32F, 3 );
  cvConvertScale (image, saveImage, 255, 0);
  cvSaveImage(filename, saveImage);
  cvReleaseImage (&saveImage);
}


template<typename _Scalar>
void writeHeatMap(const char* filename,Eigen::Matrix<_Scalar,Eigen::Dynamic,  Eigen::Dynamic> & mat,_Scalar max,_Scalar min)
{
  CvSize size;
  size.height=mat.rows();
  size.width=mat.cols();
  IplImage * image = cvCreateImage ( size, IPL_DEPTH_32F, 3 );

  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++) {
      float scaledCost=(mat(y,x)-min)/(max-min);
      CV_IMAGE_ELEM ( image, float, y, 3 * x ) = 1.0-scaledCost;
      CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = 0;
      CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = scaledCost;
     }
  saveFloatImage(filename,  image);
}

template<typename _Scalar>
void writeHeatMap(const char* filename,Eigen::Matrix<_Scalar,Eigen::Dynamic,  Eigen::Dynamic> & mat,_Scalar max,_Scalar min, int maxY, int maxX, int rad=3)
{
  CvSize size;
  size.height=mat.rows();
  size.width=mat.cols();
  IplImage * image = cvCreateImage (size, IPL_DEPTH_32F, 3);

  for(int x=0;x<size.width;x++)
    for(int y=0;y<size.height;y++) {
        if(std::abs(x-maxX)<rad && std::abs(y-maxY)<rad) {
          CV_IMAGE_ELEM (image, float, y, 3 * x ) = 0;
          CV_IMAGE_ELEM (image, float, y, 3 * x + 1) = 1;
          CV_IMAGE_ELEM (image, float, y, 3 * x + 2) = 0;
        } else if(mat(y,x)==max) {
          CV_IMAGE_ELEM ( image, float, y, 3 * x ) = 0;
          CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = 0.5;
          CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = 0;
        } else {
          float scaledCost=(mat(y,x)-min)/(max-min);
          CV_IMAGE_ELEM ( image, float, y, 3 * x ) = 1.0-scaledCost;
          CV_IMAGE_ELEM ( image, float, y, 3 * x + 1 ) = 0;
          CV_IMAGE_ELEM ( image, float, y, 3 * x + 2 ) = scaledCost;
        }
      }
  saveFloatImage(filename,  image);
}

void readLabelList(const string & file, std::vector<int> &labels, boost::dynamic_bitset<> &bitmap)
{
  std::string line;
  std::ifstream myfile(file.data());

  if (!myfile.is_open()) {
    cerr << "cound not find the file:" << file << " which stores the ranges for binning the features .. you should run this program from the folder scene_processing(do roscd scene_processing), or put the binning info files: binStumpsN.txt(for node features) and binStumpsE.txt(for edge features) in current folder and rerun ... exiting with assertion failure" << endl;
  }
  assert(myfile.is_open());

  while (myfile.good()) {
    getline(myfile, line);
    if (line.length() > 0) {
      char_separator<char> sep(",");
      tokenizer<char_separator<char> > tokens(line, sep);

      BOOST_FOREACH(string t, tokens) {
        labels.push_back(lexical_cast<int>(t.data()));
        bitmap.set(lexical_cast<int>(t.data()));
      }
    }
  }
  myfile.close();
}

#endif	/* GENERICUTILS_H */

