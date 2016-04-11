#include "simplecloud.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "common.hpp"

using namespace cv;
using namespace std;

DistPoint::DistPoint(Point2f off, Point3f src)
{
  this->off = off;
  this->src = src;
}

void SimpleCloud3d::reset(void)
{
  cells = vector<vector<DistPoint> >(dims[0]*dims[1]*dims[2]);
}

void SimplePtrCloud3d::reset(void)
{
  cells = vector<vector<void*> >(dims[0]*dims[1]*dims[2]);
}

int SimpleCloud3d::apos(const int x, const int y, const int z)
{
  return z*dims[0]*dims[1]+y*dims[0] + x;
}

int SimpleCloud3d::apos(const int idx[3])
{
  return apos(idx[0], idx[1], idx[2]);
}

int SimplePtrCloud3d::apos(const int x, const int y, const int z)
{
  return z*dims[0]*dims[1]+y*dims[0] + x;
}

int SimplePtrCloud3d::apos(const int idx[3])
{
  return apos(idx[0], idx[1], idx[2]);
}

SimpleCloud3d::SimpleCloud3d(int w, int h, int d, int iw, int ih, int id)
{
  dims[0] = w;
  dims[1] = h;
  dims[2] = d;
  img_dims[0] = iw;
  img_dims[1] = ih;
  img_dims[2] = id;
  
  cells.resize(w*h*d);
}

SimplePtrCloud3d::SimplePtrCloud3d(int w, int h, int d, int iw, int ih, int id)
{
  dims[0] = w;
  dims[1] = h;
  dims[2] = d;
  img_dims[0] = iw;
  img_dims[1] = ih;
  img_dims[2] = id;
  
  cells.resize(w*h*d);
}

void SimpleCloud3d::add(DistPoint p)
{
  int idx[3] = {p.src.x*dims[0]/img_dims[0], p.src.y*dims[1]/img_dims[1], clamp<int>(p.src.z*dims[2]/img_dims[2], 0, dims[2]-1)};
  
  cells[apos(idx)].push_back(p);
}

void SimplePtrCloud3d::add(Point3f *p, void *data)
{
  int idx[3] = {p->x*dims[0]/img_dims[0], p->y*dims[1]/img_dims[1], clamp<int>(p->z*dims[2]/img_dims[2], 0, dims[2]-1)};
  
  cells[apos(idx)].push_back(data);
}


//slowly increase coverage, measuring distance in idx space
vector<DistPoint*> SimpleCloud3d::getMin(int idx[3], int count)
{
  int r;
  int sum;
  int idx2[3];
  vector<DistPoint*> points(0);
  
  for (r=0,sum=-1;sum<count;r++) {
    sum = 0;
    for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
      for(idx2[1]=max(idx[1]-r,0);idx2[1]<min(idx[1]+r+1,dims[1]);idx2[1]++)
        for(idx2[0]=max(idx[0]-r,0);idx2[0]<min(idx[0]+r+1,dims[0]);idx2[0]++)
          sum += cells[apos(idx2)].size();
  }
  r--;
  
  for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
    for(idx2[1]=max(idx[1]-r,0);idx2[1]<min(idx[1]+r+1,dims[1]);idx2[1]++)
      for(idx2[0]=max(idx[0]-r,0);idx2[0]<min(idx[0]+r+1,dims[0]);idx2[0]++)
        for(int i=0;i<cells[apos(idx2)].size();i++)
          points.push_back(&(cells[apos(idx2)])[i]);
        
  return points;
}


//slowly increase coverage, measuring distance in idx space
vector<DistPoint*> SimpleCloud3d::getCube(int idx1[3], int idx2[3])
{
  int idx[3];
  vector<DistPoint*> points;
  
  for(idx[2]=max(idx1[2],0);idx[2]<=min(idx2[2],dims[2]-1);idx[2]++)
    for(idx[1]=max(idx1[1],0);idx[1]<=min(idx2[1],dims[1]-1);idx[1]++)
      for(idx[0]=max(idx1[0],0);idx[0]<=min(idx2[0],dims[0]-1);idx[0]++)
        for(int i=0;i<cells[apos(idx)].size();i++)
          points.push_back(&(cells[apos(idx)])[i]);

  return points;
}

//slowly increase coverage, measuring distance in idx space
Point2f SimpleCloud3d::getCubeAvg(int idx1[3], int idx2[3])
{
  Point2f p(0.0,0.0);
  int idx[3];
  int count = 0;
  
  for(idx[2]=max(idx1[2],0);idx[2]<=min(idx2[2],dims[2]-1);idx[2]++)
    for(idx[1]=max(idx1[1],0);idx[1]<=min(idx2[1],dims[1]-1);idx[1]++)
      for(idx[0]=max(idx1[0],0);idx[0]<=min(idx2[0],dims[0]-1);idx[0]++)
        for(int i=0;i<cells[apos(idx)].size();i++) {
          p += (cells[apos(idx)])[i].off;
          count ++;
        }

  if (count)
    return p*(1.0/count);
  else
    return Point2f(NAN, NAN);
}

//slowly increase coverage, measuring distance in idx space
Point2f SimpleCloud3d::getAvgMin(int idx[3], int count, int max_rad)
{
  int r;
  int sum;
  int idx2[3];
  Point2f p(0.0,0.0);
  int last_r = 0;
  
  for (r=0,sum=-1;sum<count&&r<=max_rad;r++) {
    sum = 0;
    last_r = r;
    for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
      for(idx2[1]=max(idx[1]-r,0);idx2[1]<min(idx[1]+r+1,dims[1]);idx2[1]++)
        for(idx2[0]=max(idx[0]-r,0);idx2[0]<min(idx[0]+r+1,dims[0]);idx2[0]++)
          sum += cells[apos(idx2)].size();
  }
  r = last_r;
  
  for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
    for(idx2[1]=max(idx[1]-r,0);idx2[1]<min(idx[1]+r+1,dims[1]);idx2[1]++)
      for(idx2[0]=max(idx[0]-r,0);idx2[0]<min(idx[0]+r+1,dims[0]);idx2[0]++)
        for(int i=0;i<cells[apos(idx2)].size();i++)
          p += (cells[apos(idx2)])[i].off;
  
  //printf("%d at %d (%dx%dx%d)\n",sum, r, idx[0],idx[1],idx[2]);

  if (sum)
    return p*(1.0/sum);
  else
    return Point2f(NAN, NAN);
}


//slowly increase coverage, measuring distance in idx space
Point2f SimpleCloud3d::getAvgMinZ(int idx[3], int count, int max_rad)
{
  int r;
  int sum;
  int idx2[3];
  Point2f p(0.0,0.0);
  int last_r = 0;
  
  for (r=0,sum=-1;sum<count&&r<=max_rad;r++) {
    sum = 0;
    last_r = r;
    for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
      for(idx2[1]=idx[1];idx2[1]<=idx[1];idx2[1]++)
        for(idx2[0]=idx[0];idx2[0]<=idx[0];idx2[0]++)
          sum += cells[apos(idx2)].size();
  }
  r = last_r;
  
  for(idx2[2]=max(idx[2]-r,0);idx2[2]<min(idx[2]+r+1,dims[2]);idx2[2]++)
      for(idx2[1]=idx[1];idx2[1]<=idx[1];idx2[1]++)
        for(idx2[0]=idx[0];idx2[0]<=idx[0];idx2[0]++)
        for(int i=0;i<cells[apos(idx2)].size();i++)
          p += (cells[apos(idx2)])[i].off;

  if (sum)
    return p*(1.0/sum);
  else
    return Point2f(NAN, NAN);
}

void SimpleCloud3d::writePointImgs(const char *fmt)
{
  int idx[3];
  char buf[64];
  Mat img;
  vector<DistPoint*> points;
  Point2f p;
  FILE *f = fopen("debug/points.csv", "w");
  assert(f);
  printf("writePointImgs()...");
  fflush(NULL);

  for(idx[2]=0;idx[2]<dims[2];idx[2]++) {
    img = Mat::zeros(img_dims[1], img_dims[0], CV_8UC3);
    for(idx[1]=0;idx[1]<dims[1];idx[1]++)
      for(idx[0]=0;idx[0]<dims[0];idx[0]++) {
        points = getMin(idx, 0);
        for(int i=0;i<points.size();i++) {
          p = Point2f(points[i]->src.x, points[i]->src.y);
          //line(img, Point2f(0,0), p, Scalar(255,255,255,0), 1, CV_AA);
          line(img, p, p+points[i]->off, Scalar(255,255,255,200), 1, CV_AA);
          fprintf(f, "%f,%f,%f,%f,%f,0.0\n", points[i]->src.x,points[i]->src.y,points[i]->src.z,points[i]->off.x,points[i]->off.y);
        }
      }
    sprintf(buf, fmt, idx[2]);
    imwrite(buf, img);
  }
  
  fclose(f);
  printf("done\n");
  
}

std::vector<void*> & SimplePtrCloud3d::operator[](const int idx[3])
{
  return cells[apos(idx)];
}

/*
void SimpleCloud3d::writePointData(void)
{
  int idx[3];
  char buf[64];
  vector<DistPoint*> points;
  Point2f p;
  FILE *f = fopen("debug/points.csv", "w");
  assert(f);
  printf("writePointData()...");
  fflush(NULL);

  for(idx[2]=0;idx[2]<dims[2];idx[2]++) {
    for(idx[1]=0;idx[1]<dims[1];idx[1]++)
      for(idx[0]=0;idx[0]<dims[0];idx[0]++) {
        points = getMin(idx, 0);
        for(int i=0;i<points.size();i++) {
          fprintf(f, "%f,%f,%f,%f,%f,0.0\n", points[i]->src.x,points[i]->src.y,points[i]->src.z,points[i]->off.x,points[i]->off.y);
        }
      }
  }
  
  fclose(f);
  printf("done\n");
  
}*/