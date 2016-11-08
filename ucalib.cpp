#include "ucalib.hpp"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/shape/shape_transformer.hpp>

#include <cstdarg>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

const double proj_center_size = 0.4;
const double extr_center_size = 0.2;
const double non_center_rest_weigth = 1e-4;
const double strong_proj_constr_weight = 10.0;
const double proj_constr_weight = 0.01;
const double center_constr_weight = 1.0;
const double enforce_cams_line = 10.0;

int _calib_cams_limit = 1000;
int _calib_views_limit = 1000;

#define NON_CENTRAL 1
#define MESH 2
#define RMS_ONLY 4
  
#ifdef UCALIB_WITH_MM_MESH
  #include "mm-mesh/mesh.hpp"
  #include <thread>
#endif

using namespace clif;
using namespace std;

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f)
{
  cv::Point2d l_o(line[0], line[1]);
  cv::Point2d l_d(line[2], line[3]);
  
  return cv::Vec4d(l_o.x*f.x,l_o.y*f.y, l_d.x*f.x, l_d.y*f.y);
}



void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f)
{
  cv::Point2i fdim(lines[1],lines[2]);
  
  cv::Mat_<cv::Point2d> offset(fdim.y, fdim.x);
  cv::Mat_<cv::Point2d> grad(fdim.y, fdim.x);
  
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      if (std::isnan(lines(0,x,y))) {
        offset.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        grad.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        continue;
      }
        
      
      cv::Point2d ip = cv::Point2d((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y);
      
      //project 3d lines in image space
      //line direction becomes constant term in images space
      offset.at<cv::Point2d>(y,x) = cv::Point2d(lines(2,x,y)*f.x, lines(3,x,y)*f.y) - ip;
      
      //line origin becomes z-dependend term in image space
      grad.at<cv::Point2d>(y,x) = cv::Point2d(lines(0,x,y)*f.x, lines(1,x,y)*f.y);
    }
    
  std::vector<cv::Point2f> cv_wpoints, cv_ipoints;
  std::vector<cv::DMatch> matches;
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      
      if (std::isnan(offset.at<cv::Point2d>(y,x).x))
        continue;
      
      
      double wx = (x+0.5)*idim.x/fdim.x + offset.at<cv::Point2d>(y,x).x + (1.0/z)*grad.at<cv::Point2d>(y,x).x;
      double wy = (y+0.5)*idim.y/fdim.y + offset.at<cv::Point2d>(y,x).y + (1.0/z)*grad.at<cv::Point2d>(y,x).y;
      
      matches.push_back(cv::DMatch(cv_wpoints.size(),cv_wpoints.size(), 0));
      cv_wpoints.push_back(cv::Point2f(wx, wy));
      cv_ipoints.push_back(cv::Point2f((x+0.5)*idim.x/fdim.x, (y+0.5)*idim.y/fdim.y));
    }


  cv::Ptr<cv::ThinPlateSplineShapeTransformer> transform = cv::createThinPlateSplineShapeTransformer(0);
  transform->estimateTransformation(cv_wpoints, cv_ipoints, matches);

  int approx_step = 8;
  
  printf("calc ipoints\n");
  int progress = 0;
#pragma omp parallel for schedule(dynamic)
  for(int y=0;y<idim.y;y+=approx_step) {
    std::vector<cv::Point2f> perfect_points(idim.x/approx_step), ipoints(idim.x/approx_step);
    
    for(int x=0;x<idim.x;x+=approx_step)
      perfect_points[x/approx_step] = cv::Point2f(x, y);

    transform->applyTransformation(perfect_points, ipoints);

    for(int x=0;x<idim.x;x+=approx_step) {
      //cout << perfect_points[x/approx_step] << " -> " << ipoints[x/approx_step] << "\n"; 
      map.at<cv::Point2f>(y,x) = ipoints[x/approx_step];
    }
  }
  
  
#ifndef WIN32
#pragma omp parallel for schedule(dynamic, 128) collapse(2)
#else  
#pragma omp parallel for schedule(dynamic)
#endif
  for(int y=0;y<idim.y-approx_step;y++) {
    for(int x=0;x<idim.x-approx_step;x++) {
      if (y % approx_step == 0 && x % approx_step == 0)
        continue;
      
      int x_m = x / approx_step * approx_step;
      int y_m = y / approx_step * approx_step;
      
      float xf, xf_i, yf, yf_i;
      xf_i = (x % approx_step)*1.0/approx_step;
      xf = 1.0-xf_i;
      yf_i = (y % approx_step)*1.0/approx_step;
      yf = 1.0-yf_i;
      
      cv::Point2d tl = map.at<cv::Point2f>(y_m,x_m);
      cv::Point2d tr = map.at<cv::Point2f>(y_m,x_m+approx_step);
      cv::Point2d bl = map.at<cv::Point2f>(y_m+approx_step,x_m);
      cv::Point2d br = map.at<cv::Point2f>(y_m+approx_step,x_m+approx_step);
      
      if (std::isnan(tl.x) || std::isnan(tr.x) || std::isnan(bl.x) || std::isnan(br.x)) {
        map.at<cv::Point2f>(y,x) =  cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
        continue;
      }
        
      if (norm(tl - tr) >= 2.0*approx_step) {
        map.at<cv::Point2f>(y,x) =  cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
        //printf("big step == fail? %f, %dx%d\n", norm(tl - tr),x ,y);
        continue;
      }
      
      cv::Point2d t = tl*xf + tr*xf_i;
      cv::Point2d b = bl*xf + br*xf_i;
      
      map.at<cv::Point2f>(y,x) = t*yf + b*yf_i;
    }
  }
}

namespace ucalib {

void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, clif::Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints)
{  
  cv::Point2d f(cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1));
  
  cv::Point2i fdim(lines[1],lines[2]);
  
  cv::Mat_<cv::Point2d> offset(fdim.y, fdim.x);
  cv::Mat_<cv::Point2d> grad(fdim.y, fdim.x);
  
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      if (std::isnan(lines(0,x,y))) {
        offset.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        grad.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        continue;
      }
        
      
      cv::Point2d ip = cv::Point2d((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y);
      
      //project 3d lines in image space
      //line direction becomes constant term in images space
      offset.at<cv::Point2d>(y,x) = cv::Point2d(lines(2,x,y)*f.x, lines(3,x,y)*f.y) - ip;
      
      //line origin becomes z-dependend term in image space
      grad.at<cv::Point2d>(y,x) = cv::Point2d(lines(0,x,y)*f.x, lines(1,x,y)*f.y);
    }
    
  std::vector<cv::Point2f> cv_wpoints, cv_ipoints, perfect_points;
  std::vector<cv::Point2f> ip_off, ip_grad;
  std::vector<cv::DMatch> matches;
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      
      if (std::isnan(offset.at<cv::Point2d>(y,x).x))
        continue;
      
      double wx = (x+0.5)*idim.x/fdim.x + offset.at<cv::Point2d>(y,x).x + 1/z*grad.at<cv::Point2d>(y,x).x;
      double wy = (y+0.5)*idim.y/fdim.y + offset.at<cv::Point2d>(y,x).y + 1/z*grad.at<cv::Point2d>(y,x).y;
      
      matches.push_back(cv::DMatch(cv_ipoints.size(),cv_ipoints.size(), 0));
      cv_wpoints.push_back(cv::Point2f(wx,wy));
      cv_ipoints.push_back(cv::Point2f((x+0.5)*idim.x/fdim.x, (y+0.5)*idim.y/fdim.y));
    }


  cv::Ptr<cv::ThinPlateSplineShapeTransformer> transform = cv::createThinPlateSplineShapeTransformer(0);
  transform->estimateTransformation(cv_wpoints, cv_ipoints, matches);
  
  
  cv::projectPoints(wpoints, rvec, tvec, cameraMatrix, cv::noArray(), perfect_points);
  
  transform->applyTransformation(perfect_points, ipoints);
}

}

// Generic line error
struct LineZ3GenericCenterLineError {
  LineZ3GenericCenterLineError(double x, double y)
      : x_(x), y_(y) {}
      
  template <typename T>
  bool operator()(const T* const extr,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    //compare with projected pinhole camera ray
    //FIXME without factor the fit might ignore this!
    residuals[0] = p[0]*T(10000);
    residuals[1] = p[1]*T(10000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterLineError, 2, 6>(
                new LineZ3GenericCenterLineError(x, y)));
  }

  double x_,y_;
};

// Generic line error
struct LineZ3GenericCenterLineExtraError {
  LineZ3GenericCenterLineExtraError(double x, double y)
      : x_(x), y_(y) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const extr_rel,
                  T* residuals) const {
                    
                    
    T p[3], p1[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p1);

    // camera[3,4,5] are the translation.
    p1[0] += extr[3];
    p1[1] += extr[4];
    p1[2] += extr[5];
    
    ceres::AngleAxisRotatePoint(extr_rel, p1, p);

    // camera[3,4,5] are the translation.
    p[0] += extr_rel[3];
    p[1] += extr_rel[4];
    p[2] += extr_rel[5];
    
    //compare with projected pinhole camera ray
    //FIXME without factor the fit might ignore this!
    residuals[0] = p[0]*T(10000);
    residuals[1] = p[1]*T(10000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterLineExtraError, 2, 6, 6>(
                new LineZ3GenericCenterLineExtraError(x, y)));
  }

  double x_,y_;
};

// Pinhole line error
struct LineZ3DirPinholeError {
  LineZ3DirPinholeError(double ix, double iy, double x, double y, double w = 1.0)
      : ix_(ix), iy_(iy), x_(x), y_(y), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p);
    
    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - p[2]*line[0])*T(w_);
    residuals[1] = (p[1] - p[2]*line[1])*T(w_);
    //avoid negative residuals
    residuals[2] = (p[2] - abs(p[2]))*T(10000);
    
    /*if (ix_ == 0.0)
      residuals[4] = T(0.0);//p[0]/p[2]*T(w_);
    else
      residuals[4] = sqrt(abs(p[0]/p[2]*T(ix_)/line[0] - T(ix_))+1e-18)*T(w_)*T(0.001);
    
    if (iy_ == 0.0)
      residuals[5] = T(0.0);//p[1]/p[2]*T(w_);
    else
      residuals[5] = sqrt(abs(p[1]/p[2]*T(iy_)/line[1] - T(iy_))+1e-18)*T(w_)*T(0.001);*/
    
    /*if (std::is_same<T,double>::value) {
      if (abs((p[0]/p[2]*T(ix_)/line[0] - T(ix_))) >= 1.0)
        printf("%f == %f : %f\n", T(p[0]/p[2]*T(ix_)/line[0]), T(ix_),(p[0]/p[2]*T(ix_)/line[0] - T(ix_)));
    }*/
    
    
    /*residuals[2] = (p[2]-abs(p[2]));
    residuals[3] = max(T(0.1)-p[2], T(0))*T(10000);*/
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double ix, double iy, double x, double y, double w = 1.0) {
    return (new ceres::AutoDiffCostFunction<LineZ3DirPinholeError, 3, 6, 2>(
                new LineZ3DirPinholeError(ix, iy, x, y, w)));
  }
  
  void print(const double *p, const double *line, const double *residuals, const double *extr) const
  {
    printf("%f x %f x 0 -> %f x %f x %f vs %f x %f e %f %f %f\n", x_, y_, p[0], p[1], p[2], line[0], line[1], extr[0], extr[1], extr[2]);
    if (abs(residuals[0]) > 1000000000000)
      abort();
  }
  void print(const ceres::Jet<double, 8> *p, const ceres::Jet<double, 8> *line, const ceres::Jet<double, 8> *residuals, const ceres::Jet<double, 8> *extr) const
  {
    printf("%f x %f x 0 -> %f x %f x %f vs %f x %f e %f %f %f\n", x_, y_, p[0].a, p[1].a, p[2].a, line[0].a, line[1].a, extr[0].a, extr[1].a, extr[2].a);
    if (abs(residuals[0].a) > 1000000000000)
      abort();
  }

  double x_,y_,w_,ix_,iy_;
};

struct LineZ3DirPinholeErrorIS {
  LineZ3DirPinholeErrorIS(double ix, double iy, double x, double y, double w = 1.0)
      : ix_(ix), iy_(iy), x_(x), y_(y), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    //compare with projected pinhole camera ray
    //residuals[0] = (p[0] - p[2]*line[0])*T(w_);
    //residuals[1] = (p[1] - p[2]*line[1])*T(w_);
    if (ix_ == 0.0)
      //FIXME use neighbours?
      residuals[0] = (p[0]/p[2])*T(w_);
    else
      residuals[0] = /*sqrt(abs*/(p[0]/p[2]*T(ix_)/line[0] - T(ix_))/*+1e-18)*/*T(w_);
    
    if (iy_ == 0.0)
      //FIXME use neighbours?
      residuals[1] = (p[1]/p[2])*T(w_);
    else
      residuals[1] = /*sqrt(abs*/(p[1]/p[2]*T(iy_)/line[1] - T(iy_))/*+1e-18)*/*T(w_);
    
    
    /*if (std::is_same<T,double>::value) {
      if (abs(ix_-(-600+163)) <= 2 && abs(iy_-(-400+147)) <= 2) {
        printf("%fx%f line %fx%f\n", x_, y_, line[0], line[1]);
        printf("%f == %f : %f\n", T(p[0]/p[2]*T(ix_)/line[0]), T(ix_),(p[0]/p[2]*T(ix_)/line[0] - T(ix_)));
        printf("%f == %f : %f\n\n", T(p[0]/p[2]), T(ix_),(p[0]/p[2] - T(ix_)));
      }
    }*/
    
    
    //residuals[2] = (p[2]-abs(p[2]));
    //residuals[3] = max(T(0.1)-p[2], T(0))*T(10000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double ix, double iy, double x, double y, double w = 1.0) {
    return (new ceres::AutoDiffCostFunction<LineZ3DirPinholeErrorIS, 2, 6, 2>(
                new LineZ3DirPinholeErrorIS(ix, iy, x, y, w)));
  }
  
  void print(const double *p, const double *line, const double *residuals, const double *extr) const
  {
    printf("%f x %f x 0 -> %f x %f x %f vs %f x %f e %f %f %f\n", x_, y_, p[0], p[1], p[2], line[0], line[1], extr[0], extr[1], extr[2]);
    if (abs(residuals[0]) > 1000000000000)
      abort();
  }
  void print(const ceres::Jet<double, 8> *p, const ceres::Jet<double, 8> *line, const ceres::Jet<double, 8> *residuals, const ceres::Jet<double, 8> *extr) const
  {
    printf("%f x %f x 0 -> %f x %f x %f vs %f x %f e %f %f %f\n", x_, y_, p[0].a, p[1].a, p[2].a, line[0].a, line[1].a, extr[0].a, extr[1].a, extr[2].a);
    if (abs(residuals[0].a) > 1000000000000)
      abort();
  }

  double x_,y_,w_,ix_,iy_;
};

// Pinhole line error
struct LinExtrError {
  LinExtrError(double idx)
      : idx_(idx) {}
      
  template <typename T>
  bool operator()(const T* const dir, const T* const extr,
                  T* residuals) const {
                    
                    
    T c[3], w_c[3];
    c[0] = extr[3];
    c[1] = extr[4];
    c[2] = extr[5];
    ceres::AngleAxisRotatePoint(extr, c, w_c);
    
    //compare with projected pinhole camera ray
    residuals[0] = (w_c[0]-dir[0]*T(idx_))*T(enforce_cams_line);
    residuals[1] = (w_c[1]-dir[1]*T(idx_))*T(enforce_cams_line);
    residuals[2] = (w_c[2]-dir[2]*T(idx_))*T(enforce_cams_line);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double idx) {
    return (new ceres::AutoDiffCostFunction<LinExtrError, 3, 3, 6>(
                new LinExtrError(idx)));
  }

  double idx_;
};

//center dir error (should be zero!)
struct LineZ3CenterDirError {
  LineZ3CenterDirError() {}
      
  template <typename T>
  bool operator()(const T* const line,
                  T* residuals) const {
    //compare with projected pinhole camera ray
    residuals[0] = line[0]*T(1000);
    residuals[1] = line[1]*T(1000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<LineZ3CenterDirError, 2, 2>(
                new LineZ3CenterDirError()));
  }
};

//center dir and offset error (should be zero!)
struct LineZ3GenericCenterDirError {
  LineZ3GenericCenterDirError() {}
      
  template <typename T>
  bool operator()(const T* const line,
                  T* residuals) const {
    residuals[0] = line[2]*T(100000);
    residuals[1] = line[3]*T(100000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterDirError, 2, 4>(
                new LineZ3GenericCenterDirError()));
  }
};


struct LineZ3GenericCenterDirXOnlyError {
  LineZ3GenericCenterDirXOnlyError() {}
      
  template <typename T>
  bool operator()(const T* const d,
                  T* residuals) const {
    residuals[0] = d[1]*T(100000000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterDirXOnlyError, 1, 2>(
                new LineZ3GenericCenterDirXOnlyError()));
  }
};

struct LineZ3GenericCenterDirYOnlyError {
  LineZ3GenericCenterDirYOnlyError() {}
      
  template <typename T>
  bool operator()(const T* const line,
                  T* residuals) const {
    residuals[0] = line[2]*T(100000);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterDirYOnlyError, 1, 4>(
                new LineZ3GenericCenterDirYOnlyError()));
  }
};


// Pinhole line error
struct LineZ3DirPinholeExtraError {
  LineZ3DirPinholeExtraError(double x, double y, double w = 1.0)
      : x_(x), y_(y), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const extr_rel, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], p1[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p1);

    // camera[3,4,5] are the translation.
    p1[0] += extr[3];
    p1[1] += extr[4];
    p1[2] += extr[5];
    
    
    ceres::AngleAxisRotatePoint(extr_rel, p1, p);

    // camera[3,4,5] are the translation.
    p[0] += extr_rel[3];
    p[1] += extr_rel[4];
    p[2] += extr_rel[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - p[2]*line[0])*T(w_);
    residuals[1] = (p[1] - p[2]*line[1])*T(w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double w = 1.0) {
    return (new ceres::AutoDiffCostFunction<LineZ3DirPinholeExtraError, 2, 6, 6, 2>(
                new LineZ3DirPinholeExtraError(x, y, w)));
  }

  double x_,y_,w_;
};

struct RectProjError {
  RectProjError(cv::Point2d wp, cv::Point2d ip, double weight)
      : w(weight)
      {
        wx = wp.x;
        wy = wp.y;
        ix = ip.x;
        iy = ip.y;
      }

      
  template<typename T> 
  bool operator()(const T* const e, const T* const p,
                  T* residuals) const {
    T pw[3] = {T(wx), T(wy), T(0)};
    T pc[3];
    
    ceres::AngleAxisRotatePoint(e, pw, pc);
    pc[0] += e[3];
    pc[1] += e[4];
    pc[2] += e[5];
    
    residuals[0] = (T(ix)-pc[0]*p[0]/pc[2])*T(w);
    residuals[1] = (T(iy)-pc[1]*p[1]/pc[2])*T(w);
    /*residuals[2] = (pc[2]-abs(pc[2]))*T(w);
    residuals[3] = (p[0]-abs(p[0]));
    residuals[4] = (p[1]-abs(p[1]));
    residuals[5] = (1.0-max(p[0]/p[1],p[1]/p[0]))*T(100);*/
    //cout << pc[2] << "\n";
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Point2d wp, cv::Point2d ip, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjError, 2, 6, 2>(
                new RectProjError(wp, ip, weight)));
  }

  double ix,iy,w;
  double wx,wy;
};


      
template <typename T> void add_mul_vecs(const T* const a, const T* const b, double scale_b, T *res)
{
  res[0] = a[0]+T(scale_b)*b[0];
  res[1] = a[1]+T(scale_b)*b[1];
  res[2] = a[2]+T(scale_b)*b[2];
}
  
struct RectProjMeshError {
  RectProjMeshError(cv::Point2d wp, cv::Point2d ip, double weight, double ws[4])
      : w(weight)
      {
        wx = wp.x;
        wy = wp.y;
        ix = ip.x;
        iy = ip.y;
        for(int i=0;i<4;i++)
          ws_[i] = ws[i];
      }


  template<typename T> 
  bool operator()(const T* const e, const T* const p, const T* const tl, const T* const tr, const T* const bl,  const T* const br,
                  T* residuals) const {
    T pw[3] = {T(wx), T(wy), T(0)};
    T pc[3];
    
    add_mul_vecs(pw, tl, ws_[0], pw);
    add_mul_vecs(pw, tr, ws_[1], pw);
    add_mul_vecs(pw, bl, ws_[2], pw);
    add_mul_vecs(pw, br, ws_[3], pw);
    
    ceres::AngleAxisRotatePoint(e, pw, pc);
    pc[0] += e[3];
    pc[1] += e[4];
    pc[2] += e[5];
    
    residuals[0] = (T(ix)-pc[0]*p[0]/pc[2])*T(w);
    residuals[1] = (T(iy)-pc[1]*p[1]/pc[2])*T(w);
    /*residuals[2] = (pc[2]-abs(pc[2]))*T(w);
    residuals[3] = (p[0]-abs(p[0]));
    residuals[4] = (p[1]-abs(p[1]));
    residuals[5] = (1.0-max(p[0]/p[1],p[1]/p[0]))*T(100);*/
    //cout << pc[2] << "\n";
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Point2d wp, cv::Point2d ip, double weight, double ws[4]) {
    return (new ceres::AutoDiffCostFunction<RectProjMeshError, 2, 6, 2, 3, 3, 3, 3>(
                new RectProjMeshError(wp, ip, weight, ws)));
  }

  double ix,iy,w;
  double wx,wy;
  double ws_[4];
};

struct RectProjExtraError {
  RectProjExtraError(cv::Point2d wp, cv::Point2d ip, double weight)
      : w(weight)
      {
        wx = wp.x;
        wy = wp.y;
        ix = ip.x;
        iy = ip.y;
      }

      
  template<typename T> 
  bool operator()(const T* const e, const T* const e2, const T* const p,
                  T* residuals) const {
    T pw[3] = {T(wx), T(wy), T(0)};
    T pc[3], pc2[3];
    
    ceres::AngleAxisRotatePoint(e, pw, pc);
    pc[0] += e[3];
    pc[1] += e[4];
    pc[2] += e[5];
    
    ceres::AngleAxisRotatePoint(e2, pc, pc2);

    // camera[3,4,5] are the translation.
    pc2[0] += e2[3];
    pc2[1] += e2[4];
    pc2[2] += e2[5];
    
    
    residuals[0] = (T(ix)-pc2[0]*p[0]/pc2[2])*T(w);
    residuals[1] = (T(iy)-pc2[1]*p[1]/pc2[2])*T(w);
    /*residuals[2] = (pc[2]-abs(pc[2]))*T(w);
    residuals[3] = (p[0]-abs(p[0]));
    residuals[4] = (p[1]-abs(p[1]));
    residuals[5] = (1.0-max(p[0]/p[1],p[1]/p[0]))*T(100);*/
    //cout << pc[2] << "\n";
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Point2d wp, cv::Point2d ip, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjExtraError, 2, 6, 6, 2>(
                new RectProjExtraError(wp, ip, weight)));
  }

  double ix,iy,w;
  double wx,wy;
};

struct RectProjDirError4P {
  RectProjDirError4P(cv::Point2d ip, double weight)
      : w(weight)
      {
        ix = ip.x;
        iy = ip.y;
      }

      
  template<typename T> 
  bool operator()(const T* const p, const T* const l,
                  T* residuals) const {
    T wx = l[2]; //dir
    T wy = l[3];
    T px = wx * p[0];    
    T py = wy * p[1];    
    residuals[0] = (T(ix)-px)*T(w);
    residuals[1] = (T(iy)-py)*T(w);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Point2d ip, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjDirError4P, 2, 2, 4>(
                new RectProjDirError4P(ip, weight)));
  }

  double ix,iy,w;
};


// Generic line error
struct LineZ3GenericError {
  LineZ3GenericError(double x, double y)
      : x_(x), y_(y) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - (line[0] + p[2]*line[2]));
    residuals[1] = (p[1] - (line[1] + p[2]*line[3]));
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericError, 2, 6, 4>(
                new LineZ3GenericError(x, y)));
  }

  double x_,y_;
};

// Generic line error, with target deformation
struct LineZ3GenericFreeMeshError {
  LineZ3GenericFreeMeshError(double x, double y, double ws[4], cv::Mat_<float> j)
      : x_(x), y_(y) { for(int i=0;i<4;i++) ws_[i] = ws[i]; j_ = j.clone(); }
      
  template <typename T>
  bool operator()(const T* const extr, const T* const o, const T* const d, const T* const tl, const T* const tr, const T* const bl,  const T* const br,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    
    add_mul_vecs(w_p, tl, ws_[0], w_p);
    add_mul_vecs(w_p, tr, ws_[1], w_p);
    add_mul_vecs(w_p, bl, ws_[2], w_p);
    add_mul_vecs(w_p, br, ws_[3], w_p);
    
    T mesh_d[3] = {w_p[0]-T(x_),w_p[1]-T(y_),w_p[2]};
    
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    T p_c[3]; //res in cam coordinates
    T p_t[3]; //res in target coordinates
    T r_neg[3] = {-extr[0],-extr[1],-extr[2]};
    
    //compare with projected pinhole camera ray
    p_c[0] = (p[0] - (o[0] + p[2]*d[0]));
    p_c[1] = (p[1] - (o[1] + p[2]*d[1]));
    p_c[2] = T(0);
    
    ceres::AngleAxisRotatePoint(r_neg, p_c, p_t);
    
    //TODO check p_t[2] == 0 -> no-not the case...
    residuals[0] = T(j_(0, 0)) * p_t[0] + T(j_(0, 1)) * p_t[1];
    residuals[1] = T(j_(1, 0)) * p_t[0] + T(j_(1, 1)) * p_t[1];
    
    //residuals[0] = p_c[0];
    //residuals[1] = p_c[1];
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double ws[4], cv::Mat_<float> j) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericFreeMeshError, 2, 6, 2, 2, 3, 3, 3, 3>(
                new LineZ3GenericFreeMeshError(x, y, ws, j)));
  }

  double x_,y_;
  double ws_[4];
  double wx_, wy_;
  cv::Mat_<float> j_;
};



// Generic line error, with target deformation
struct MeshSmoothError {
  MeshSmoothError(double w)
      : w_(w) {}
      
  template <typename T>
  bool operator()(const T* const m1, const T* const m2,
                  T* residuals) const {
    //compare with projected pinhole camera ray
    residuals[0] = (m1[0]-m2[0])*T(w_);
    residuals[1] = (m1[1]-m2[1])*T(w_);
    residuals[2] = (m1[2]-m2[2])*T(w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double w) {
    return (new ceres::AutoDiffCostFunction<MeshSmoothError, 3, 3, 3>(
                new MeshSmoothError(w)));
  }

  double w_;
};

// Generic line error, with target deformation
struct LineZ3GenericMeshError {
  LineZ3GenericMeshError(double x, double y, double ws[4], double m_w)
      : x_(x), y_(y), m_w_(m_w) { for(int i=0;i<4;i++) ws_[i] = ws[i]; }
      
  template <typename T>
  bool operator()(const T* const extr, const T* const line, const T* const tl, const T* const tr, const T* const bl,  const T* const br,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    
    add_mul_vecs(w_p, tl, ws_[0], w_p);
    add_mul_vecs(w_p, tr, ws_[1], w_p);
    add_mul_vecs(w_p, bl, ws_[2], w_p);
    add_mul_vecs(w_p, br, ws_[3], w_p);
    
    T mesh_d[3] = {w_p[0]-T(x_),w_p[1]-T(y_),w_p[2]};
    
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - (line[0] + p[2]*line[2]));
    residuals[1] = (p[1] - (line[1] + p[2]*line[3]));
    
    residuals[2] = sqrt(abs(mesh_d[0]+T(1e-18)))*T(m_w_);
    residuals[3] = sqrt(abs(mesh_d[1]+T(1e-18)))*T(m_w_);
    residuals[4] = sqrt(abs(mesh_d[2]+T(1e-18)))*T(m_w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double ws[4], double m_w) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericMeshError, 5, 6, 4, 3, 3, 3, 3>(
                new LineZ3GenericMeshError(x, y, ws, m_w)));
  }

  double x_,y_;
  double ws_[4];
  double m_w_;
};

// Generic line error
struct LineZ3GenericExtraError {
  LineZ3GenericExtraError(double x, double y)
      : x_(x), y_(y) {}
      
  template <typename T>
  bool operator()(const T* const extr, const T* const extr_rel, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], p1[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr, w_p, p1);

    // camera[3,4,5] are the translation.
    p1[0] += extr[3];
    p1[1] += extr[4];
    p1[2] += extr[5];
    
    ceres::AngleAxisRotatePoint(extr_rel, p1, p);

    // camera[3,4,5] are the translation.
    p[0] += extr_rel[3];
    p[1] += extr_rel[4];
    p[2] += extr_rel[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - (line[0] + p[2]*line[2]));
    residuals[1] = (p[1] - (line[1] + p[2]*line[3]));
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericExtraError, 2, 6, 6, 4>(
                new LineZ3GenericExtraError(x, y)));
  }

  double x_,y_;
};

// try to (lightly) enforece pinhole model
struct LineZ3GenericCenterError {
  LineZ3GenericCenterError(double w) : w_(w) {}
      
  template <typename T>
  bool operator()(const T* const line,
                  T* residuals) const {
    residuals[0] = sqrt(abs(line[0])+1e-18)*T(w_);
    residuals[1] = sqrt(abs(line[1])+1e-18)*T(w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double w) {
    //should reduce this to 2 but ceres complains about the same pointer being added with four params
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterError, 2, 4>(
                new LineZ3GenericCenterError(w)));
  }
  
  double w_;
};

// try to (lightly) enforece pinhole model
struct FindCenterError {
  FindCenterError(const double* const line) : line_(line) {}
      
  template <typename T>
  bool operator()(const T* const offset,
                  T* residuals) const {
    residuals[0] = T(line_[0])+offset[0]*T(line_[2]);
    residuals[1] = T(line_[1])+offset[0]*T(line_[3]);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double* const line) {
    //should reduce this to 2 but ceres complains about the same pointer being added with four params
    return (new ceres::AutoDiffCostFunction<FindCenterError, 2, 1>(
                new FindCenterError(line)));
  }
  
  const double* const line_;
};

double calc_line_pos_weight(cv::Point2i pos, cv::Point2i proxy_size, double r, double min_weight = non_center_rest_weigth)
{
  int pmax = std::max(proxy_size.x, proxy_size.y);
  
  double w = norm(cv::Point2d((pos.x+0.5-proxy_size.x*0.5)/pmax,(pos.y+0.5-proxy_size.y*0.5)/pmax));
  
  return std::max(std::max(r-w, 0.0)*(1.0/r), min_weight);
}

static void _zline_problem_add_proj_error(ceres::Problem &problem, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &proj, double proj_constraint, double min_weight)
{
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*proj_center_size*proj_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
      
  for(auto pos : Idx_It_Dims(lines, "x", "cams")) {        
    int x = pos["x"];
    int y = pos["y"];
    
    if (pos["cams"] > _calib_cams_limit)
      continue;
    
    cv::Point2d ip = cv::Point2d((x+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(y+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);
      
    double w = calc_line_pos_weight(cv::Point2i(x, y), proxy_size, center_constr_weight, min_weight);
    
    w *= proj_constraint;
    
    if (w == 0.0)
      continue;
    
    /*ceres::CostFunction* cost_function =
        RectProjError::Create(wp, ip, w);
        
    problem.AddResidualBlock(cost_function,
                            NULL,
                            &proj(0,pos.r("channels","cams")),
                            &lines({2,pos.r("x","cams")}));*/
  }
}

static void _zline_problem_add_lin_views(ceres::Problem &problem, Mat_<double> &extrinsics_rel, double *dir)
{      
  for(auto pos : Idx_It_Dims(extrinsics_rel, "channels", "cams")) {
    if (!pos["cams"])
      continue;
    if (pos["cams"] > _calib_cams_limit)
      continue;
    
    ceres::CostFunction* cost_function =
        LinExtrError::Create(pos["cams"]);
        
    problem.AddResidualBlock(cost_function,
                            NULL,
                            dir,
                            &extrinsics_rel(pos));
  }
}



static void _zline_problem_add_proj_error_4P(ceres::Problem &problem, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &proj, double proj_constraint)
{
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*proj_center_size*proj_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
      
  for(auto pos : Idx_It_Dims(lines, "x", "cams")) {        
    int x = pos["x"];
    int y = pos["y"];
    
    cv::Point2d ip = cv::Point2d((x+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(y+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);
      
    double w = exp(-((ip.x*ip.x+ip.y*ip.y)/two_sigma_squared));
    //w_sum += w;
    w = sqrt(w);
    w *= proj_constraint;
    
    
    ceres::CostFunction* cost_function =
        RectProjDirError4P::Create(ip, w);
        
    problem.AddResidualBlock(cost_function,
                            NULL,
                            &proj(0,pos.r("channels","cams")),
                            &lines({0,pos.r("x","cams")}));
  }
}

static void _zline_problem_add_pinhole_lines(ceres::Problem &problem, cv::Point2i img_size, const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &lines, Mat_<double> &proj, ceres::LossFunction *loss = NULL, bool falloff = true, double min_weight = non_center_rest_weigth, double proj_weight = 1.0)
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*extr_center_size*extr_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  //channels == 0 && cams == 0
  int last_view = 0;
  int ray_count = 0;
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    if (ray["cams"] > _calib_cams_limit)
      continue;
    if (ray["views"] > _calib_views_limit)
      continue;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    
    cv::Point2d ip = cv::Point2d((ray["x"]+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(ray["y"]+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);

    if (isnan(p.x) || isnan(p.y))
      continue;
    
    double w;
    
    /*if (falloff)
      w = calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, extr_center_size, min_weight);
    else*/
      w = 1.0;
 
    if (ray["views"] != last_view) {
      printf("view %d: %d rays\n", last_view, ray_count);
      last_view = ray["views"];
      ray_count = 0;
    }
    
    //keep center looking straight (in local ref system)
    /*if (ray["y"] == center.y && ray["x"] == center.x) {
      ceres::CostFunction* cost_function =
      LineZ3CenterDirError::Create();
      problem.AddResidualBlock(cost_function, loss, 
                              &lines({2,ray.r("x","cams")}));
      ray_count++;
    }*/
    
    if (w > 0.0) {
      if (ref_cam) {
        //std::cout << "process: " << ray << p << "\n";
        //regular line error (in x/y world direction)
        ceres::CostFunction* cost_function = 
        LineZ3DirPinholeError::Create(ip.x, ip.y, p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));
        
        if (proj_weight) {
          double w_proj;
          //FIXME should be extra opt. step...
          if (_calib_cams_limit)
            w_proj = proj_weight*calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, proj_center_size, 0.0);
          else
            w_proj = proj_weight;
          
          if (w_proj) {
            cost_function = RectProjError::Create(p, ip, w_proj);
            problem.AddResidualBlock(cost_function,
                                    NULL,
                                    &extrinsics({0,ray["views"]}),
                                    &proj(0,ray.r("channels","cams")));
            
            /*problem.SetParameterLowerBound(&proj(0,ray.r("channels","cams")), 0, 100);
            problem.SetParameterLowerBound(&proj(0,ray.r("channels","cams")), 1, 100);
            problem.SetParameterUpperBound(&proj(0,ray.r("channels","cams")), 0, 10000);
            problem.SetParameterUpperBound(&proj(0,ray.r("channels","cams")), 1, 10000);*/
          }
        }
      }
      else {
        ceres::CostFunction* cost_function = 
        LineZ3DirPinholeExtraError::Create(p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),&extrinsics_rel({0,ray.r("channels","cams")}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));
        
        if (proj_weight) {
          double w_proj;
          w_proj = proj_weight*calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, proj_center_size, min_weight);
          if (w_proj) {
            cost_function = RectProjExtraError::Create(p, ip, w_proj);
            problem.AddResidualBlock(cost_function,
                                      NULL,
                                      &extrinsics({0,ray["views"]}),&extrinsics_rel({0,ray.r("channels","cams")}),
                                      &proj(0,ray.r("channels","cams")));
          
            problem.SetParameterLowerBound(&proj(0,ray.r("channels","cams")), 0, 100);
            problem.SetParameterLowerBound(&proj(0,ray.r("channels","cams")), 1, 100);
            problem.SetParameterUpperBound(&proj(0,ray.r("channels","cams")), 0, 10000);
            problem.SetParameterUpperBound(&proj(0,ray.r("channels","cams")), 1, 10000);
          }
        }
        
        /*ceres::CostFunction* cost_function = 
        LineZ3DirPinholeError::Create(ip.x, ip.y, p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics_rel({0,ray.r("channels","cams")}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));*/
      }
      ray_count++;
    }
  }
  printf("view %d: %d rays\n", last_view, ray_count);
}


static void _zline_problem_add_pinhole_lines_is(ceres::Problem &problem, cv::Point2i img_size, const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &lines, ceres::LossFunction *loss = NULL, bool falloff = true, double min_weight = non_center_rest_weigth)
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*extr_center_size*extr_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  //channels == 0 && cams == 0
  int last_view = 0;
  int ray_count = 0;
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    if (ray["cams"] > _calib_cams_limit)
      continue;
    if (ray["views"] > _calib_views_limit)
      continue;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    
    cv::Point2d ip = cv::Point2d((ray["x"]+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(ray["y"]+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);

    if (isnan(p.x) || isnan(p.y))
      continue;
    
    double w;
    
    /*if (falloff)
      w = calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, extr_center_size, min_weight);
    else*/
      w = 1.0;
 
    if (ray["views"] != last_view) {
      printf("view %d: %d rays\n", last_view, ray_count);
      last_view = ray["views"];
      ray_count = 0;
    }
    
    //keep center looking straight (in local ref system)
    if (ray["y"] == center.y && ray["x"] == center.x) {
      ceres::CostFunction* cost_function =
      LineZ3CenterDirError::Create();
      problem.AddResidualBlock(cost_function, loss, 
                              &lines({2,ray.r("x","cams")}));
      ray_count++;
    }
    
    if (w > 0.0) {
      if (ref_cam) {
        
        //std::cout << "process: " << ray << p << "\n";
        //regular line error (in x/y world direction)
        ceres::CostFunction* cost_function = 
        LineZ3DirPinholeErrorIS::Create(ip.x, ip.y, p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));
      }
      else {
        abort();
        /*ceres::CostFunction* cost_function = 
        LineZ3DirPinholeExtraError::Create(p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),&extrinsics_rel({0,ray.r("channels","cams")}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));
        */
        ceres::CostFunction* cost_function = 
        LineZ3DirPinholeError::Create(ip.x, ip.y, p.x, p.y, w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics_rel({0,ray.r("channels","cams")}),
                                //use only direction part!
                                &lines({2,ray.r("x","cams")}));
      }
      ray_count++;
    }
  }
  printf("view %d: %d rays\n", last_view, ray_count);
}

class ApxMedian
{
public:
  ApxMedian(double step, double max)
  {
    _inv_step = 1.0/step;
    _lin_buckets = int(M_E*_inv_step)+1;
    _max_idx = _lin_buckets+log(max*100.0)*_inv_step;
    _buckets.resize(_max_idx);
    _max_idx--;
  };
  
  
  void add(double val)
  {
    val = abs(val)*100.0;
    
    if (val >= M_E)
      _buckets[min(int(_lin_buckets+log(val)*_inv_step), _max_idx)]++;
    else
      _buckets[val*_inv_step]++;
    
    _count++;
  };
  
  double median()
  {
    int sum = 0;
    for(int i=0;i<_max_idx;i++) {
      sum += _buckets[i];
      if (sum >= _count/2) {
        printf("bucket %d sum %d count/2: %d\n", i, sum, _count/2);
        if (i < _lin_buckets)
          return i/_inv_step*0.01;
        else
          return exp(i - _lin_buckets)*0.01;
      }
    }
  }
  
private:
  std::vector<int> _buckets;
  int _max_idx;
  int _lin_buckets;
  double _inv_step;
  int _count = 0;
};

static int _zline_problem_filter_pinhole_lines(const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &lines, double threshold)
{
  return 0;
  /*double step = 0.0001;
  //ApxMedian med(step, 1.0);
  std::vector<float> scores;
  int count = 0, remain = 0;
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  //channels == 0 && cams == 0
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    if (isnan(p.x) || isnan(p.y))
      continue;
    
    double residuals[3];
    
    if (ref_cam) {
      LineZ3DirPinholeError err(p.x, p.y);
      err.operator()<double>(&extrinsics({0,ray["views"]}), &lines({2,ray.r("x","cams")}), residuals);
    }
    else {
      LineZ3DirPinholeExtraError err(p.x, p.y);
      err.operator()<double>(&extrinsics({0,ray["views"]}), &extrinsics_rel({0,ray.r("channels","cams")}), &lines({2,ray.r("x","cams")}), residuals);
    }
    
    double err = sqrt(residuals[0]*residuals[0]+residuals[1]*residuals[1]);
    scores.push_back(err);
    //med.add(err);
    //if (err >= 0.1) {
    //  proxy({0,ray.r("x",-1)}) = std::numeric_limits<double>::quiet_NaN();
    //  proxy({1,ray.r("x",-1)}) = std::numeric_limits<double>::quiet_NaN();
    //}
  }
  
  std::sort(scores.begin(), scores.end());
  
  double th = std::max(scores[scores.size()*99/100],scores[scores.size()]*2);// med.median()*4.0 + step;
  
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    if (isnan(p.x) || isnan(p.y))
      continue;
    
    double residuals[3];
    
    if (ref_cam) {
      LineZ3DirPinholeError err(p.x, p.y);
      err.operator()<double>(&extrinsics({0,ray["views"]}), &lines({2,ray.r("x","cams")}), residuals);
    }
    else {
      LineZ3DirPinholeExtraError err(p.x, p.y);
      err.operator()<double>(&extrinsics({0,ray["views"]}), &extrinsics_rel({0,ray.r("channels","cams")}), &lines({2,ray.r("x","cams")}), residuals);
    }
    
    double err = sqrt(residuals[0]*residuals[0]+residuals[1]*residuals[1]);
    //med.add(err);
    if (err >= th) {
      count++;
      proxy({0,ray.r("x",-1)}) = std::numeric_limits<double>::quiet_NaN();
      proxy({1,ray.r("x",-1)}) = std::numeric_limits<double>::quiet_NaN();
    }
    else
      remain++;
  }
  
  printf("median: %f\n", scores[scores.size()/2]);
  printf("removed %d data points, %d remain\n", count, remain);
  return count;*/
}

static void _zline_problem_add_generic_lines(ceres::Problem &problem, const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &lines, bool reproj_error_calc_only = false)
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  //channels == 0 && cams == 0
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    if (isnan(p.x) || isnan(p.y))
      continue;
    
    //keep center looking straight
    if (ray["y"] == center.y && ray["x"] == center.x && !reproj_error_calc_only) {
      ceres::CostFunction* cost_function =
      LineZ3GenericCenterDirError::Create();
      problem.AddResidualBlock(cost_function, NULL, 
                              &lines({0,ray.r("x","cams")}));
    }
    
    if (ray["y"] == center.y && (ray["x"] == center.x-1 || ray["x"] == center.x+1) && !reproj_error_calc_only) {
      ceres::CostFunction* cost_function = LineZ3GenericCenterError::Create(1000000000000);
      problem.AddResidualBlock(cost_function, NULL,
                              &lines({0,ray.r("x","cams")}));
    }
    
  
    if (ref_cam) {
      //std::cout << "process: " << ray << p << "\n";
      

      //regular line error (in x/y world direction)
      ceres::CostFunction* cost_function = 
      LineZ3GenericError::Create(p.x, p.y);
      problem.AddResidualBlock(cost_function,
                              NULL,
                              &extrinsics({0,ray["views"]}),
                              &lines({0,ray.r("x","cams")}));
    }
    else {
        ceres::CostFunction* cost_function = 
        LineZ3GenericExtraError::Create(p.x, p.y);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),
                                &extrinsics_rel({0,ray.r("channels","cams")}),
                                &lines({0,ray.r("x","cams")}));
    }
  }
}


static void _zline_problem_add_generic_lines_mesh(ceres::Problem &problem, const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &proj, Mat_<double> &lines, cv::Point2i img_size, cv::Point2d target_size, Mat_<double> &mesh, bool reproj_error_calc_only = false, const Mat_<float> &proxy_j = Mat_<float>(), int flags = 0)
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  cv::Point2d mesh_size(mesh[1],mesh[2]);
  
  Mat_<float> mesh_coverage({mesh[1],mesh[2]}); 
  
  cvMat(mesh_coverage).setTo(0);
  
  //channels == 0 && cams == 0
  for(auto ray : Idx_It_Dims(proxy, "x", "views")) {
    bool ref_cam = true;
    
    for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
      if (ray[i])
        ref_cam = false;
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    if (isnan(p.x) || isnan(p.y))
      continue;
  
    
    if (ray["y"] == center.y && (ray["x"] == center.x+1) && !reproj_error_calc_only) {
      ceres::CostFunction* cost_function = LineZ3GenericCenterDirXOnlyError::Create();
      problem.AddResidualBlock(cost_function, NULL,
                              &lines({2,ray.r("x","cams")}));
    }
    
    /*if (ray["y"] == center.y+1 && ray["x"] == center.x && !reproj_error_calc_only) {
      ceres::CostFunction* cost_function = LineZ3GenericCenterDirYOnlyError::Create();
      problem.AddResidualBlock(cost_function, NULL,
                              &lines({0,ray.r("x","cams")}));
    }*/
    
    
  
    if (ref_cam) {
      //std::cout << "process: " << ray << p << "\n";
      

      double ws[4];
      double *points[4];
      
      cv::Point2d mesh_f = cv::Point2d(p.x / target_size.x * (mesh_size.x-2), p.y / target_size.y * (mesh_size.y-2));
      cv::Point2i mesh_i(floor(mesh_f.x),floor(mesh_f.y));
      mesh_f -= cv::Point2d(mesh_i);
      
      ws[0] = (1-mesh_f.x)*(1-mesh_f.y);
      ws[1] = (mesh_f.x)*(1-mesh_f.y);
      ws[2] = (1-mesh_f.x)*(mesh_f.y);
      ws[3] = (mesh_f.x)*(mesh_f.y);
      
      if (mesh_i.x+1 >= mesh[1] || mesh_i.y+1 >= mesh[2] || mesh_i.x < 0 || mesh_i.y < 0) {
        printf("WARNING: very wrong target coordinate:"); 
        cout << mesh_f << "/" << p << "\n";
        continue;
      }
      
      mesh_coverage(mesh_i.x, mesh_i.y) += ws[0];
      mesh_coverage(mesh_i.x+1, mesh_i.y) += ws[1];
      mesh_coverage(mesh_i.x, mesh_i.y+1) += ws[2];
      mesh_coverage(mesh_i.x+1, mesh_i.y+1) += ws[3];
      
      /*if (!reproj_error_calc_only)
        for(int c=0;c<3;c++) {
          mesh(c,mesh_i.x, mesh_i.y) = 0;
          mesh(c,mesh_i.x+1, mesh_i.y) = 0;
          mesh(c,mesh_i.x, mesh_i.y+1) = 0;
          mesh(c,mesh_i.x+1, mesh_i.y+1) = 0;
        }*/
      
      bool hold_gridpoint = true;
      
    
      //oly-9-18 dataset
      if (mesh_i.x == 4 && mesh_i.y == 4)
         hold_gridpoint = true;
      if (mesh_i.x == 4 && mesh_i.y == 9)
         hold_gridpoint = true;
      if (mesh_i.x == 9 && mesh_i.y == 4)
         hold_gridpoint = true;
      
      /*if (mesh_i.x == 5 && mesh_i.y == 6)
         hold_gridpoint = true;
      if (mesh_i.x == 7 && mesh_i.y == 6)
         hold_gridpoint = true;
      if (mesh_i.x == 5 && mesh_i.y == 12)
         hold_gridpoint = true;*/
      
      
       /*if (mesh_i.x <= 7 && mesh_i.y <= 8)
         hold_gridpoint = true;*/
      
      /*if (mesh_i.y <= 5)
        hold_gridpoint = true;
      if (mesh_i.y >= 16)
         hold_gridpoint = true;
      if (mesh_i.x <= 1)
        hold_gridpoint = true;
       if (mesh_i.x >= 16)
         hold_gridpoint = true;*/
      
      double wx = 1.0, wy = 1.0;
      
      cv::Mat j;
      if (proxy_j.total()) {
        j = cvMat(proxy_j.bindAll(-1, -1, ray["x"], ray["y"], ray["channels"], ray["cams"], ray["views"]));
        j = j.clone();
        invert(j, j);
      }
      
      if (!reproj_error_calc_only) {
        //push model away from pinhole!
        lines({0,ray.r("x","cams")}) += (rand() / double(RAND_MAX)-0.5)*0.01;
        lines({1,ray.r("x","cams")}) += (rand() / double(RAND_MAX)-0.5)*0.01;
      }
      
      /*if (ray["y"] == center.y && ray["x"] == center.x && !reproj_error_calc_only) {
        //printf("setting pblock constant %p!\n", &lines({0,ray.r("x","cams")}));
        //cout << lines({0,ray.r("x","cams")}) << lines({1,ray.r("x","cams")}) << lines({2,ray.r("x","cams")}) << lines({3,ray.r("x","cams")}) << "\n";
        lines({0,ray.r("x","cams")}) = 0;
        lines({1,ray.r("x","cams")}) = 0;
        lines({2,ray.r("x","cams")}) = 0;
        lines({3,ray.r("x","cams")}) = 0;
      }*/
      
      
      //regular line error (in x/y world direction)
      ceres::CostFunction* cost_function = 
      LineZ3GenericFreeMeshError::Create(p.x, p.y, ws, j);
      problem.AddResidualBlock(cost_function,
                              NULL,
                              &extrinsics({0,ray["views"]}),
                              &lines({0,ray.r("x","cams")}),
                              &lines({2,ray.r("x","cams")}),
                              &mesh(0,mesh_i.x, mesh_i.y),
                              &mesh(0,mesh_i.x+1, mesh_i.y),
                              &mesh(0,mesh_i.x, mesh_i.y+1),
                              &mesh(0,mesh_i.x+1, mesh_i.y+1)
                              );
      

      /*for(int i=0;i<3;i++) {
        mesh(i,mesh_i.x, mesh_i.y) = 0;
        mesh(i,mesh_i.x+1, mesh_i.y) = 0;
        mesh(i,mesh_i.x, mesh_i.y+1) = 0;
        mesh(i,mesh_i.x+1, mesh_i.y+1) = 0;
      }*/
            
      if (hold_gridpoint) {     
        for(int i=0;i<3;i++) {
          mesh(i,mesh_i.x, mesh_i.y) = 0;
          mesh(i,mesh_i.x+1, mesh_i.y) = 0;
          mesh(i,mesh_i.x, mesh_i.y+1) = 0;
          mesh(i,mesh_i.x+1, mesh_i.y+1) = 0;
        }
        
        problem.SetParameterBlockConstant(&mesh(0,mesh_i.x, mesh_i.y));
        problem.SetParameterBlockConstant(&mesh(0,mesh_i.x+1, mesh_i.y));
        problem.SetParameterBlockConstant(&mesh(0,mesh_i.x, mesh_i.y+1));
        problem.SetParameterBlockConstant(&mesh(0,mesh_i.x+1, mesh_i.y+1));
        
      }
      
      if ((flags & NON_CENTRAL) == 0) {
        lines({0,ray.r("x","cams")}) = 0;
        lines({1,ray.r("x","cams")}) = 0;
        problem.SetParameterBlockConstant(&lines({0,ray.r("x","cams")}));
      }
      
      
      //keep center looking straight
      //TODO add mesh mod to center line functions...
      if (ray["y"] == center.y && ray["x"] == center.x && !reproj_error_calc_only) {
        //printf("setting pblock constant %p!\n", &lines({0,ray.r("x","cams")}));
        //cout << lines({0,ray.r("x","cams")}) << lines({1,ray.r("x","cams")}) << lines({2,ray.r("x","cams")}) << lines({3,ray.r("x","cams")}) << "\n";
        lines({0,ray.r("x","cams")}) = 0;
        lines({1,ray.r("x","cams")}) = 0;
        lines({2,ray.r("x","cams")}) = 0;
        lines({3,ray.r("x","cams")}) = 0;
        problem.SetParameterBlockConstant(&lines({0,ray.r("x","cams")}));
        problem.SetParameterBlockConstant(&lines({2,ray.r("x","cams")}));
      }
      
      if (ray["y"] == center.y && ray["x"] == center.x+1 && !reproj_error_calc_only) {
        lines({0,ray.r("x","cams")}) = 0;
        lines({1,ray.r("x","cams")}) = 0;
        problem.SetParameterBlockConstant(&lines({0,ray.r("x","cams")}));
      }
      
      /*double proj_weight = 10.0;
      
      double w_proj;
      if (_calib_cams_limit)
        w_proj = proj_weight*calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, proj_center_size, 0.0);
      else
        w_proj = proj_weight;
      
      if (w_proj) {
        cv::Point2d ip = cv::Point2d((ray["x"]+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(ray["y"]+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);
        
        cost_function = RectProjMeshError::Create(p, ip, w_proj, ws);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &extrinsics({0,ray["views"]}),
                                 &proj(0,ray.r("channels","cams")),
                                  &mesh(0,mesh_i.x, mesh_i.y),
                                  &mesh(0,mesh_i.x+1, mesh_i.y),
                                  &mesh(0,mesh_i.x, mesh_i.y+1),
                                  &mesh(0,mesh_i.x+1, mesh_i.y+1));
      }*/
    }
    else {
        /*ceres::CostFunction* cost_function = 
        LineZ3GenericExtraError::Create(p.x, p.y);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &extrinsics({0,ray["views"]}),
                                &extrinsics_rel({0,ray.r("channels","cams")}),
                                &lines({0,ray.r("x","cams")}));*/
    }
  }
  
  if (!reproj_error_calc_only)
    for(auto pos : Idx_It_Dims(mesh, 1, 2)) {
      double w = 1;
      if (mesh_coverage(pos[1],pos[2]) > 0.0 && mesh_coverage(pos[1]+1,pos[2]) > 0.0 && pos[1]+1 < mesh[1]) {
        //printf("restrict %dx%d\n", pos]});
        w = 10.0/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1]+1,pos[2]));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &mesh(0,pos[1], pos[2]),
                                &mesh(0,pos[1]+1, pos[2]));
      }
      if (mesh_coverage(pos[1],pos[2]) > 0.0 && mesh_coverage(pos[1],pos[2]+1) > 0.0 && pos[2]+1 < mesh[2]) {
        w = 10.0/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1],pos[2]+1));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                NULL,
                                &mesh(0,pos[1], pos[2]),
                                &mesh(0,pos[1], pos[2]+1));
      }
    }
}

double calc_line_residuals(const Mat_<float>& proxy, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &proj, Mat_<double> &lines, cv::Point2i img_size, cv::Point2d target_size, Mat_<double> &mesh, const Mat_<float> &proxy_j = Mat_<float>(), double filter = 0, int *filtered_r = NULL, Mat_<std::vector<cv::Point2f>> *residuals = NULL)
{
  
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  cv::Point2d mesh_size(mesh[1],mesh[2]);
  
  double avg;
  double rms;
  int count = 0;
  int filtered = 0;
  int ray_supports;
  
  std::vector<float> med;
  
  //channels == 0 && cams == 0
  for(auto ray : Idx_It_Dims(proxy, "x", "y")) {
    
    ceres::Problem problem;
    
    ray_supports = 0;
    
    for(auto view : Idx_It_Dims(proxy, "channels", "views")) {
      bool ref_cam = true;
      for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++)
        if (view[i])
          ref_cam = false;
        
        cv::Point2f p(proxy({0,ray.r("x","y"),view.r("channels",-1)}),
                      proxy({1,ray.r("x","y"),view.r("channels",-1)}));
        
        if (isnan(p.x) || isnan(p.y))
          continue;
        
        
        if (ref_cam) {
          double ws[4];
          double *points[4];
          
          cv::Point2d mesh_f = cv::Point2d(p.x / target_size.x * (mesh_size.x-2), p.y / target_size.y * (mesh_size.y-2));
          cv::Point2i mesh_i(floor(mesh_f.x),floor(mesh_f.y));
          mesh_f -= cv::Point2d(mesh_i);
          
          ws[0] = (1-mesh_f.x)*(1-mesh_f.y);
          ws[1] = (mesh_f.x)*(1-mesh_f.y);
          ws[2] = (1-mesh_f.x)*(mesh_f.y);
          ws[3] = (mesh_f.x)*(mesh_f.y);
          
          assert(mesh_i.x+1<mesh[1] && mesh_i.y+1<mesh[2]);
          
          cv::Mat j;
          if (proxy_j.total()) {
            j = cvMat(proxy_j.bindAll(-1, -1, ray["x"], ray["y"], view["channels"], view["cams"], view["views"]));
            j = j.clone();
            invert(j, j);
          }
          
          //regular line error (in x/y world direction)
          ceres::CostFunction* cost_function = 
          LineZ3GenericFreeMeshError::Create(p.x, p.y, ws, j);
          double res[2];
          
          double *params[] = {&extrinsics({0,view["views"]}),
                                  &lines({0,ray.r("x","y"),view.r("channels","cams")}),
                                  &lines({2,ray.r("x","y"),view.r("channels","cams")}),
                                  &mesh(0,mesh_i.x, mesh_i.y),
                                  &mesh(0,mesh_i.x+1, mesh_i.y),
                                  &mesh(0,mesh_i.x, mesh_i.y+1),
                                  &mesh(0,mesh_i.x+1, mesh_i.y+1)};
          
          cost_function->Evaluate(params, res, NULL);
          
          double v = res[0]*res[0] +res[1]*res[1];
          
          if (residuals)
            (*residuals)(ray["x"],ray["y"]).push_back(cv::Point2f(res[0],res[1]));
  
          if (v == 0.0 && ray["x"] == 4 && ray["y"] == 4) {
            LineZ3GenericFreeMeshError err(p.x, p.y, ws, j);
            err(&extrinsics({0,view["views"]}),
                &lines({0,ray.r("x","y"),view.r("channels","cams")}),
                &lines({2,ray.r("x","y"),view.r("channels","cams")}),
                &mesh(0,mesh_i.x, mesh_i.y),
                &mesh(0,mesh_i.x+1, mesh_i.y),
                &mesh(0,mesh_i.x, mesh_i.y+1),
                &mesh(0,mesh_i.x+1, mesh_i.y+1), res);
            printf("%f %f from %fx%f over %fx%fx%f\n", res[0], res[1],p.x, p.y, extrinsics({0,view["views"]}),extrinsics({1,view["views"]}),extrinsics({2,view["views"]}));
            cout << j << "\n";
            j = cvMat(proxy_j.bindAll(-1, -1, ray["x"], ray["y"], view["channels"], view["cams"], view["views"]));
            cout << j << "\n";
          }
          
          
          if ((ray["x"] != center.x || ray["y"] != center.y) && (filter != 0.0 && sqrt(v) > filter)) {
            //cout << res[0] << "x"<< res[1] << ray << center << "\n";
            proxy({0,ray.r("x","y"),view.r("channels",-1)}) = std::numeric_limits<float>::quiet_NaN();
            proxy({1,ray.r("x","y"),view.r("channels",-1)}) = std::numeric_limits<float>::quiet_NaN();
            filtered++;
          }
          else {
            rms += v;
            v = sqrt(v);
            med.push_back(v);
            ray_supports++;
            avg += v;
            count++;
          }
        }
    }
    
     if (ray_supports < 5) {
      for(auto view : Idx_It_Dims(proxy, "channels", "views")) {
        for(int i=proxy.dim("channels");i<=proxy.dim("cams");i++) {
          proxy({0,ray.r("x","y"),view.r("channels",-1)}) = std::numeric_limits<float>::quiet_NaN();
          proxy({1,ray.r("x","y"),view.r("channels",-1)}) = std::numeric_limits<float>::quiet_NaN();
        }
        
        lines({0,ray.r("x","y"),view.r("channels","cams")}) = std::numeric_limits<double>::quiet_NaN();
      }
    }
      
    //printf("ray_supports: %d\n", ray_supports);
  }

  avg /= count;
  
  if (filter != 0.00)
    printf("filtered %d of %d (> %f)\n", filtered,filtered+count, filter);
  
  if (filtered_r)
    *filtered_r = filtered;
  
  std::sort(med.begin(), med.end());
  
  cout << "avg: " << avg << "rms: " << sqrt(rms/count) << " med: " << med[count/2] << "\n";
  cout << " 1/10st: " << med[count*99/100] << "\n";
  return max(med[count/2]*5,med[count*99/100]);
}

static void _zline_problem_add_center_errors(ceres::Problem &problem, Mat_<double> &lines, const Mat_<float> &proxy)
{
  if (center_constr_weight != 0.0)
    for(auto pos : Idx_It_Dims(lines, "x", "cams")) {
      if (!isnan(lines({0,pos.r("x","cams")}))) {
        ceres::CostFunction* cost_function = LineZ3GenericCenterError::Create(center_constr_weight*sqrt(proxy["views"]));
        problem.AddResidualBlock(cost_function, NULL,
                                &lines({0,pos.r("x","cams")}));
      }
    }
}

static void _add_find_center_errors(ceres::Problem &problem, Mat_<double> &lines, double *offset)
{  
  for(auto pos : Idx_It_Dims(lines, "x", "cams"))
    if (!isnan(lines({0,pos.r("x","cams")}))) {
      ceres::CostFunction* cost_function = FindCenterError::Create(&lines({0,pos.r("x","cams")}));
      problem.AddResidualBlock(cost_function, NULL,
                            offset);
    }
}

#ifdef MM_MESH_WITH_VIEWER
void update_cams_mesh(Mesh &cams, Mat_<double> extrinsics, Mat_<double> extrinsics_rel, Mat_<double> lines, Mat_<double> proj)
{
  cams = mesh_cam().scale(20);
  
  for(auto pos : Idx_It_Dims(extrinsics_rel,"channels","cams")) {
      //cv::Vec3b col(0,0,0);
      //col[ch%3]=255;
      Eigen::Vector3d trans(extrinsics_rel(3,pos.r("channels","cams")),extrinsics_rel(4,pos.r("channels","cams")),extrinsics_rel(5,pos.r("channels","cams")));
      Eigen::Vector3d rot(extrinsics_rel(0,pos.r("channels","cams")),extrinsics_rel(1,pos.r("channels","cams")),extrinsics_rel(2,pos.r("channels","cams")));
      
      Mesh cam = mesh_cam().scale(20);
      cam += trans;
      cam.rotate(rot);
      cams.merge(cam);
      //cam_writer.add(cam_left,col);
      //printf("extr:\n");
      //std::cout << -rot << "\n trans:\n" << -trans << std::endl;
      //printf("proj %f %f\n", proj(0, pos.r(1,2)), proj(1,pos.r(1,2)));
      //printf("%.3f ", trans.norm());
      
      Mesh line_mesh;
      
      for(auto line_pos : Idx_It_Dims(lines,"x","y")) {
        /*if (line_pos["x"] % 4 != 0)
          continue;
        if (line_pos["y"] % 4 != 0)
          continue;*/
        Eigen::Vector3d origin(lines({0,line_pos.r("x","y"),pos.r("channels","cams")}),lines({1,line_pos.r("x","y"),pos.r("channels","cams")}),0.0);
        Eigen::Vector3d dir(lines({2,line_pos.r("x","y"),pos.r("channels","cams")}),lines({3,line_pos.r("x","y"),pos.r("channels","cams")}),-1.0);
        
      
        Mesh line = mesh_line(origin,origin+dir*1000.0);
        
        line_mesh.merge(line);
      }
      
      line_mesh += trans;
      line_mesh.rotate(rot);
      
      cams.merge(line_mesh);
      //viewer.data.add_edges(P1,P2,Eigen::RowVector3d(r,g,b);
    }
  //printf("\n");
  
  for(auto pos : Idx_It_Dim(extrinsics, "views")) {
      Eigen::Vector3d trans(extrinsics(3,pos["views"]),extrinsics(4,pos["views"]),extrinsics(5,pos["views"]));
      Eigen::Vector3d rot(extrinsics(0,pos["views"]),extrinsics(1,pos["views"]),extrinsics(2,pos["views"]));
      
      Mesh plane = mesh_plane().scale(2000);
      
      //printf("trans %d %fx%fx%f\n", pos["views"], trans(0), trans(1), trans(2));
      
      plane -= trans;
      plane.rotate(-rot);
      
      cams.merge(plane);
  }
  
  //printf("proj %f %f\n", proj(0, 0), proj(1,0));
}

class _mesh_updater : public ceres::IterationCallback {
public:
explicit _mesh_updater(Mesh *mesh, Mat_<double> *extr, Mat_<double> *extr_rel, Mat_<double> *lines, Mat_<double> *proj) :
_mesh(mesh), _extr(extr), _extr_rel(extr_rel), _lines(lines), _proj(proj)
{};

~_mesh_updater() {}

virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
  Mesh mesh_new;
  update_cams_mesh(mesh_new, *_extr, *_extr_rel, *_lines, *_proj);
  //trigger single viewer update
  *_mesh = mesh_new;
  
  return ceres::SOLVER_CONTINUE;
}

private:
  Mat_<double> *_extr = NULL;
  Mat_<double> *_extr_rel = NULL;
  Mat_<double> *_lines = NULL;
  Mat_<double> *_proj = NULL;
  Mesh *_mesh = NULL;
};
#endif


double solve_pinhole(const ceres::Solver::Options &options, const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> proj, double proj_weight, double min_weight = non_center_rest_weigth)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  double dir[3] = {0,0,0};
  
  printf("target rot: %fx%fx%f\n", extrinsics(0), extrinsics(1), extrinsics(2));
  _zline_problem_add_pinhole_lines(problem, img_size, proxy, extrinsics, extrinsics_rel, lines, proj, NULL, true, min_weight, proj_weight);
  //if (proj_weight > 0.0)
    //_zline_problem_add_proj_error(problem, lines, img_size, proj, proj_weight, 0.0);
  //_zline_problem_add_lin_views(problem, extrinsics_rel, dir);
  
  printf("solving pinhole problem (proj w = %f...\n", proj_weight);
  ceres::Solve(options, &problem, &summary);
  printf("target rot: %fx%fx%f (%p)\n", extrinsics(0), extrinsics(1), extrinsics(2), &extrinsics(0));
  //std::cout << summary.FullReport() << "\n";
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double solve_non_central(const ceres::Solver::Options &options, const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  double dir[3] = {0,0,0};
  
  _zline_problem_add_generic_lines(problem, proxy, extrinsics, extrinsics_rel, lines);
  _zline_problem_add_center_errors(problem, lines, proxy);
  
  printf("solving unconstrained problem\n");
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  printf("\nunconstrained rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double solve_non_central_mesh(ceres::Solver::Options options, const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> proj, cv::Point2i target_size, Mat_<double> &mesh, bool reproj_error_calc_only = false, const Mat_<float> &scales = Mat_<float>(), int flags = 0)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  double dir[3] = {0,0,0};
  
  _zline_problem_add_generic_lines_mesh(problem, proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, mesh, reproj_error_calc_only, scales, flags);
  
  if (reproj_error_calc_only)
    options.max_num_iterations = 0;
  
  options.minimizer_progress_to_stdout = false;
  
  printf("solving deformation problem\n");
  ceres::Solve(options, &problem, &summary);
  //if (!reproj_error_calc_only)
    //std::cout << summary.FullReport() << "\n";
  printf("\nunconstrained rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double correct_cam_center(ceres::Solver::Options options, const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> proj, cv::Point2i target_size, Mat_<double> &mesh, bool reproj_error_calc_only = false, const Mat_<float> &scales = Mat_<float>())
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  double dir[3] = {0,0,0};
  
  double offset;
  
  _add_find_center_errors(problem, lines, &offset);
  
  printf("search cam center\n");
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  
  printf("offset: %f\n", offset);
  
  double *line;
  for(auto pos : Idx_It_Dims(lines, "x", "cams")) {
    line = &lines({0,pos.r("x","cams")});
    line[0] += offset*line[2];
    line[1] += offset*line[3];
  }
  
  //FIXME more views/cams
  for(auto pos : Idx_It_Dim(extrinsics, "views"))
    extrinsics({5,pos[1]}) -= offset;
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}




double refine_pinhole_ispace(const ceres::Solver::Options &options, const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> proj, double proj_weight, double min_weight = non_center_rest_weigth)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  double dir[3] = {0,0,0};
  
  _zline_problem_add_pinhole_lines_is(problem, img_size, proxy, extrinsics, extrinsics_rel, lines, NULL, true, min_weight);
  
  printf("solving pinhole problem (proj w = %f...\n", proj_weight);
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

int filter_pinhole(const Mat_<float>& proxy, Mat_<double> &lines, cv::Point2i img_size, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> proj, double threshold)
{
  return _zline_problem_filter_pinhole_lines(proxy, extrinsics, extrinsics_rel, lines, threshold);
}

/*
 * optimize together:
 *  - per image camera movement (world rotation and translation)
 *  - individual lines (which do not need to converge in an optical center
 */
double fit_cams_lines_multi(Mat_<float> &proxy, cv::Point2i img_size, Mat_<double> &lines, Mat_<double> &extrinsics, Mat_<double> &extrinsics_rel, Mat_<double> &proj, bool vis, const Mat_<float>& j)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  //options.function_tolerance = 1e-10;
  options.parameter_tolerance = 1e-20;
  options.minimizer_progress_to_stdout = false;
  //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  
  options.minimizer_progress_to_stdout = true;
  
  lines.create({IR(4, "line"), proxy.r("x", "cams")});
  extrinsics.create({IR(6, "extrinsics"), IR(proxy["views"], "views")});
  cout << "extrinsics: " << extrinsics << "\n";
  extrinsics_rel.create({IR(6, "extrinsics"), proxy.r("channels","cams")});
  
  //for threading!
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE)) {
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  }
  
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  
  Mat_<double> r({proxy.r("channels","cams")});
  Mat_<double> m({2, proxy.r("channels","cams")});
  
  proj.create({2,proxy.r("channels","cams")});
  
  for(auto pos : Idx_It_Dims(r, 0, -1))
    r(pos) = 0;
  for(auto pos : Idx_It_Dims(m, 0, -1))
    m(pos) = 0;
  for(auto pos : Idx_It_Dims(proj, 0, -1))
    proj(pos) = 1000;
  
  for(auto cam_pos : Idx_It_Dim(extrinsics, "views")) {
    for(int i=0;i<5;i++)
      extrinsics({i, cam_pos["views"]}) = 0;
    extrinsics({5, cam_pos["views"]}) = 1000;
    //extrinsics({2, cam_pos["views"]}) = M_PI*0.5;
  }

  for(auto pos : Idx_It_Dims(extrinsics_rel, 0, -1))
    extrinsics_rel(pos) = 0;
  
  //for(auto pos : Idx_It_Dims(extrinsics_rel, 1, -1))
    //extrinsics_rel({3,pos.r(1,-1)}) = -50*pos["cams"];
  
  for(auto line_pos : Idx_It_Dims(lines, 1, -1)) {
    lines({0, line_pos.r(1,-1)}) = 0;
    lines({1, line_pos.r(1,-1)}) = 0;
    lines({2, line_pos.r(1,-1)}) = 0.01;
    lines({3, line_pos.r(1,-1)}) = 0.01;
  }
  
  if (proxy["views"] > 1) {
    printf("%f x %f \n", proxy(0, 0, 0, 0, 0, 0),proxy(1, 0, 0, 0, 0, 0));
    printf("%f x %f \n", proxy(0, 0, 0, 0, 0, 1),proxy(1, 0, 0, 0, 0, 1));
  }
  
  //mesh display
#ifdef MM_MESH_WITH_VIEWER 
  Mesh *mesh = new Mesh();
  _mesh_updater callback(mesh, &extrinsics, &extrinsics_rel, &lines, &proj);
    
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;
  
  update_cams_mesh(*mesh, extrinsics, extrinsics_rel, lines, proj);
    
  if (vis)
    mesh->show(false);
  
#endif

  
  
  /*ceres::Solver::Summary summary;
  ceres::Problem problem;
  
  _zline_problem_add_pinhole_lines(problem, proxy, extrinsics, extrinsics_rel, lines);
  _zline_problem_add_proj_error(problem, lines, img_size, proj, strong_proj_constr_weight);
  
  printf("solving pinhole problem (strong projection)...\n");
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));*/
  
  /*int filtered = 1;
  while (filtered) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight);
    filtered = filter_pinhole(proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.2);
  }
  filtered = 1;
  while (filtered) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, proj_constr_weight);
    filtered = filter_pinhole(proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.2);
  }
  filtered = 1;
  while (filtered) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.0);
    filtered = filter_pinhole(proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.2);
  }*/
  
  /*_calib_views_limit = 0;
  _calib_cams_limit = 0;
  solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
  return 0.0;*/
  
  //FIXME first selection of views MUST contain center line(s)
  
  options.max_num_iterations = 100;
  _calib_cams_limit = 0;
  for(_calib_views_limit=proxy["views"]-1;_calib_views_limit<proxy["views"];_calib_views_limit=std::min(int(_calib_views_limit)+1,proxy["views"])) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
  }
  if (proxy["views"] > 1) 
    options.max_num_iterations = 5000;
  solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
  solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, proj_constr_weight, non_center_rest_weigth);
  
  //solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 1.0, non_center_rest_weigth);
  
  //solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, proj_constr_weight, non_center_rest_weigth);
  
  /*options.max_num_iterations = 5000;
  _calib_cams_limit=0;
  for(_calib_cams_limit=0;_calib_cams_limit<proxy["cams"];_calib_cams_limit=std::min(int(_calib_cams_limit*1.5)+1,proxy["cams"])) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
  }*/
  //solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
  //solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.0, 0.0);
  
  /*int filtered = 1;
  while (filtered) {
    solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, strong_proj_constr_weight, non_center_rest_weigth);
    filtered = filter_pinhole(proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.2);
  }*/
  //solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.0, non_center_rest_weigth);
  //options.function_tolerance = 1e-20;
  //options.minimizer_progress_to_stdout = true;
  //FIXME disable until we port extrinsics constraint (center ray == 0) back to it
  options.max_num_iterations = 5000;
  //refine_pinhole_ispace(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0.0, non_center_rest_weigth);
  
  
  Mat_<double> target_mesh({3, 20, 20});
  //FIXME target dimension * marker_size!
  cv::Point2i target_size(1200,1200);
  
  cvMat(target_mesh).setTo(0);
  
  
  solve_pinhole(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, 0, 0);
  printf("pinhole rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, true, j);
  
  solve_non_central(options, proxy, lines, img_size, extrinsics, extrinsics_rel);
  printf("non-central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, true, j);
  
printf("\nsolving central camera with deformation ------------------------------------\n\n");
  
  int solve_flags = 0;

  Mat_<float> proxy_cp;
  Mat_<double> lines_cp;
  
  proxy_cp.create(proxy);
  lines_cp.create(lines);
  
  proxy.copyTo(proxy_cp);
  lines.copyTo(lines_cp);
  

  //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, false, j, solve_flags);
  printf("deformation rms:\n");
  
  double res_avg;
  int filtered = 1;
  
  while (filtered) {
    res_avg = calc_line_residuals(proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, target_mesh, j);
    calc_line_residuals(proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, target_mesh, j, res_avg, &filtered);
    if (filtered) {
      //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
      solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, false, j, solve_flags);
    }
  }
  
  printf("final central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, true, j, solve_flags);
  
  
  
  
  
  printf("\nsolving non-central camera with deformation ------------------------------------\n\n");
  
  solve_flags = NON_CENTRAL;
  
  proxy_cp.copyTo(proxy);
  lines_cp.copyTo(lines);
  

  //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, false, j, solve_flags);
  printf("deformation rms:\n");
  
  filtered = 1;
  
  while (filtered) {
    res_avg = calc_line_residuals(proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, target_mesh, j);
    calc_line_residuals(proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, target_mesh, j, res_avg, &filtered);
    if (filtered) {
      //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
      solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, false, j, solve_flags);
    }
  }
  
  printf("final central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, true, j, solve_flags);
  
  Mat_<std::vector<cv::Point2f>> res_m({proxy.r("x","y")});
  calc_line_residuals(proxy, extrinsics, extrinsics_rel, proj, lines, img_size, target_size, target_mesh, j, 0, NULL, &res_m);
  cv::Mat debug_dirs(img_size, CV_8UC3, cv::Scalar(0));
  
  for(auto pos : Idx_It_Dims(res_m, 0, 1)) {
    cv::Point2i fdim(lines[1],lines[2]);
    
    int x = pos["x"];
    int y = pos["y"];
    cv::Point2f ip = cv::Point2f((x+0.5)/fdim.x*img_size.x, (y+0.5)/fdim.y*img_size.y);
    std::vector<cv::Point2f> &ps = res_m(pos);
    
    /*if (ps.size()) {
      for(auto d : ps) {
        printf("draw: %d %d : %d %fx%f %fx%f\n",x,y,ps.size(),ip.x, ip.y,100*d.x, 100*d.y);
        if (d.x != 0.0)
          break;
      }
    }*/
    
    for(auto d : ps)
      cv::line(debug_dirs, ip, ip+100*d, CV_RGB(255,255,255), 1, CV_AA);
  }
  
  imwrite("debug_reproj_dirs.tif", debug_dirs);
  
  
  correct_cam_center(options, proxy, lines, img_size, extrinsics, extrinsics_rel, proj, target_size, target_mesh, true, j);

  printf("proj last: %f %f\n", proj(0), proj(1));
  
  Mesh target_vis = mesh_subdiv_plane(target_mesh[1],target_mesh[2]);
  
  target_vis.scale(double(target_size.x)/target_mesh[1]);
  
  for(int j=0;j<target_mesh[1];j++) {
    for(int i=0;i<target_mesh[2];i++) {
      printf("[%2.2f %2.2f %2.2f]", target_mesh(0,i,j), target_mesh(1,i,j), target_mesh(2,i,j));
      target_vis.V(j*target_mesh[1]+i, 0) += target_mesh(0,i,j);
      target_vis.V(j*target_mesh[1]+i, 1) += target_mesh(1,i,j);
      target_vis.V(j*target_mesh[1]+i, 2) += target_mesh(2,i,j);
    }
    
    printf("\n");
  }
  delete mesh;
  if (vis)
    target_vis.show(true);
  
  for(auto line_pos : Idx_It_Dims(lines, 1, -1)) {
    if (lines({0, line_pos.r(1,-1)}) == 0 &&
        lines({1, line_pos.r(1,-1)}) == 0 &&
        lines({2, line_pos.r(1,-1)}) == 0.01 &&
        lines({3, line_pos.r(1,-1)}) == 0.01) {
          lines({0, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          lines({1, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          lines({2, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          lines({3, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
        }
  }
  
  Mesh line_mesh;
      
  for(auto line_pos : Idx_It_Dims(lines,"x","y")) {
    Eigen::Vector3d origin(lines({0,line_pos.r("x","y"),0,0}),lines({1,line_pos.r("x","y"),0,0}),0.0);
    Eigen::Vector3d dir(lines({2,line_pos.r("x","y"),0,0}),lines({3,line_pos.r("x","y"),0,0}),-1.0);
    
    if (!isnan(origin(0))) {
      dir *= 5;
      //dir(2) *= 0.1;
      Mesh line = mesh_line(origin-dir,origin+dir);
      //cout << origin << dir << "\n";
      line_mesh.merge(line);
    }
  }
  if (vis)
    line_mesh.show(true);
  
  line_mesh.writeOBJ("center.obj");
  
  
  //_zline_problem_eval_pinhole_lines(problem, proxy, extrinsics, extrinsics_rel, lines);
  
  /*ceres::Problem problem2;
  
  _zline_problem_add_pinhole_lines(problem2, proxy, extrinsics, extrinsics_rel, lines);
  _zline_problem_add_proj_error(problem2, lines, img_size, proj, proj_constr_weight);
  
  printf("solving pinhole problem (weak projection)...\n");
  ceres::Solve(options, &problem2, &summary);
  std::cout << summary.FullReport() << "\n";
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem2.NumResiduals()));
  
  printf("proj %fx%f, move %fx%f\n", proj[0], proj[1], m[0], m[1]);
  
  std::cout << summary.FullReport() << "\n";
  
  printf("rms: %f\n",2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  //return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
  
  ///////////////////////////////////////////////
  //full problem
  ///////////////////////////////////////////////
  options.max_num_iterations = 5000;
  options.function_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  
  ceres::Problem problem_full;
  
  _zline_problem_add_generic_lines(problem_full, proxy, extrinsics, extrinsics_rel, lines);
  _zline_problem_add_center_errors(problem_full, lines);
  _zline_problem_add_proj_error_4P(problem_full, lines, img_size, proj, proj_constr_weight);
  
  printf("solving constrained generic problem...\n");
  ceres::Solve(options, &problem_full, &summary);
  std::cout << summary.FullReport() << "\n";
  printf("\nunconstrained rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem_full.NumResiduals()));
  
  options.max_num_iterations = 0;
  ceres::Problem problem_reproj;
  _zline_problem_add_generic_lines(problem_reproj, proxy, extrinsics, extrinsics_rel, lines, true);
  
  ceres::Solve(options, &problem_reproj, &summary);
  
  printf("\nunconstrained rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem_reproj.NumResiduals()));*/

#ifdef MM_MESH_WITH_VIEWER
  //glfwSetWindowShouldClose(_viewer->window, 1);
  //FIXME add blocking exit method to clif::mesh
  //viewer_thread.join();
#endif
  printf("finished\n");

  return 0.0;
}
