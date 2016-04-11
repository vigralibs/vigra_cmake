#ifndef _LUTCALIB_SIMPLECLOUD_H
#define _LUTCALIB_SIMPLECLOUD_H

#include "opencv2/core/core.hpp"

class DistPoint
{
public:
  cv::Point2f off;
  cv::Point3f src;
  
  DistPoint();
  DistPoint(cv::Point2f off, cv::Point3f src);
};

class SimpleCloud3d
{
public:
  SimpleCloud3d() {};
  SimpleCloud3d(int w, int h, int d, int iw, int ih, int id);
  void add(DistPoint p);
  std::vector<DistPoint*> getMin(int idx[3], int count);
  std::vector<DistPoint*> getCube(int idx1[3], int idx2[3]);
  cv::Point2f getCubeAvg(int idx1[3], int idx2[3]);
  cv::Point2f getAvgMin(int idx[3], int count, int max_rad = 1000000);
  cv::Point2f getAvgMinZ(int idx[3], int count, int max_rad = 1000000);
  void writePointImgs(const char *fmt);
  void reset(void);
private:
  int dims[3];
  int img_dims[3];
  std::vector<std::vector<DistPoint> > cells;
 
  int apos(const int x, const int y, const int z);
  int apos(const int idx[3]);
};

class SimplePtrCloud3d
{
public:
  SimplePtrCloud3d() {};
  SimplePtrCloud3d(int w, int h, int d, int iw, int ih, int id);
  void add(cv::Point3f *p, void *data);
  std::vector<void*> & operator[](const int idx[3]);
  void reset(void);
private:
  int dims[3];
  int img_dims[3];
  std::vector<std::vector<void*> > cells;
 
  int apos(const int x, const int y, const int z);
  int apos(const int idx[3]);
};

#endif