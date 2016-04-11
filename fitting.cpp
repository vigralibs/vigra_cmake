#include "fitting.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

//#include "simplecloud.hpp"
//#include "common.hpp"
//#include "globals.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"

using namespace cv;
using namespace std;

const double proj_center_size = 0.3;
const int pers_fit_min_count = 100;

// origin (p1): (p0,p1,0), dir(p2): (p2,p3,1)
struct LineZSimple3dError {
  LineZSimple3dError(double x, double y, double z)
      : x_(x), y_(y), z_(z) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const { 
    residuals[0] = T(x_) - (p[0]+T(z_)*p[2]);
    residuals[1] = T(y_) - (p[1]+T(z_)*p[3]);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double z) {
    return (new ceres::AutoDiffCostFunction<LineZSimple3dError, 2, 4>(
                new LineZSimple3dError(x, y, z)));
  }

  double x_,y_,z_;
};


template<typename T> static inline T norm(const T v[3])
{
  return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

template<typename T> static inline T norm2(const T v[3])
{
  return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
}

//penalize rotation around x/y
struct NonZRotationError {
  NonZRotationError() {}
  

  template <typename T>
  bool operator()(const T* const r,
                  T* residuals) const {
    
    T x2y2 = r[0]*r[0] + r[1]*r[1];
    T z2 = r[2]*r[2];
    T n2 = x2y2+z2;
    //FIXME nicer solution?
    T ratio = x2y2/(n2+0.00000001);
    T err = ratio/(M_PI/2-n2);
    residuals[0] = err*T(10);
                    
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<NonZRotationError, 1, 6>(
                new NonZRotationError()));
  }

  double x_,y_,z_;
};

struct RegGridPenalty {
  RegGridPenalty() {}
  

  template <typename T>
  bool operator()(const T* const l1, const T* const l2, const T* const l3,
                  T* residuals) const {
    T d1[2];
    d1[0] = l2[0]-l1[0];
    d1[1] = l2[1]-l1[1];
    T d2[2];
    d2[0] = l3[0]-l2[0];
    d2[1] = l3[1]-l2[1];
    
    residuals[0] = (d2[0] - d1[0])*T(100);
    residuals[1] = (d2[1] - d1[1])*T(100);                    
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    return (new ceres::AutoDiffCostFunction<RegGridPenalty, 2, 4,4,4>(
                new RegGridPenalty()));
  }

  double x_,y_,z_;
};

// Pinhole line error
struct LineZ3GenericPinholeError {
  LineZ3GenericPinholeError(double x, double y)
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
    residuals[0] = p[0] - p[2]*line[2];
    residuals[1] = p[1] - p[2]*line[3];
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericPinholeError, 2, 6, 4>(
                new LineZ3GenericPinholeError(x, y)));
  }

  double x_,y_;
};

// Pinhole line error
struct LineZ3GenericPinholeExtraError {
  LineZ3GenericPinholeExtraError(double x, double y)
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
    
    //extra rotation (additional camera)
    ceres::AngleAxisRotatePoint(extr_rel, p1, p);

    //extra translation
    p[0] += extr_rel[3];
    p[1] += extr_rel[4];
    p[2] += extr_rel[5];
    
    //compare with projected pinhole camera ray
    residuals[0] = p[0] - p[2]*line[2];
    residuals[1] = p[1] - p[2]*line[3];
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericPinholeExtraError, 2, 6, 6, 4>(
                new LineZ3GenericPinholeExtraError(x, y)));
  }

  double x_,y_;
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
    
    //extra rotation (additional camera)
    ceres::AngleAxisRotatePoint(extr_rel, p1, p);

    //extra translation
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


// Generic line error
struct LineZ3GenericWeightXYError {
  LineZ3GenericWeightXYError(double x, double y)
      : x_(x), y_(y) {}
      
      
      template<typename T> static inline T norm2(const T v[3])
      {
        return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
      }

  template <typename T>
  bool operator()(const T* const extr, const T* const line, const T* const l1, const T* const l2,
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
    
    T d1[2] = {l1[2]-line[2], l1[3]-line[3]};
    T d2[2] = {l2[2]-line[2], l2[3]-line[3]};
    
    T wx = T(10.0)+norm2(d1);
    T wy = T(10.0)+norm2(d2);
    
    //compare with projected pinhole camera ray
    residuals[0] = (p[0] - (line[0] + p[2]*line[2]))/wx;
    residuals[1] = (p[1] - (line[1] + p[2]*line[3]))/wy;
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericWeightXYError, 2, 6, 4, 4, 4>(
                new LineZ3GenericWeightXYError(x, y)));
  }

  double x_,y_;
};

// Generic line error
struct RotSymmPenalty {
  RotSymmPenalty(double mul[4], double weight) : w(weight)
      {
        for(int i=0;i<4;i++)
          m[i] = mul[i];
      }
      
  template <typename T>
  bool operator()(const T* const l1, const T* const l2,
                  T* residuals) const {
    //compare with projected pinhole camera ray
    residuals[0] = (T(m[0])*l1[2]+T(m[1])*l1[3] - l2[2])*T(100000)*T(w);
    residuals[1] = (T(m[2])*l1[2]+T(m[3])*l1[3] - l2[3])*T(100000)*T(w);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double mul[4], double w) {
    return (new ceres::AutoDiffCostFunction<RotSymmPenalty, 2, 4, 4>(
                new RotSymmPenalty(mul, w)));
  }

  double m[4], w;
};


// Generic line error
struct PersPenalty {
  PersPenalty(int mul[4], double weight) : w(weight)
      {
        for(int i=0;i<4;i++)
          m[i] = mul[i];
      }
      
  template <typename T>
  bool operator()(const T* const l1, const T* const l2,
                  T* residuals) const {
    //compare with projected pinhole camera ray
    residuals[0] = (T(m[0])*l1[2]+T(m[1])*l1[3] - l2[2])*T(10000)*T(w);
    residuals[1] = (T(m[2])*l1[2]+T(m[3])*l1[3] - l2[3])*T(10000)*T(w);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int mul[4], double w) {
    return (new ceres::AutoDiffCostFunction<PersPenalty, 2, 4, 4>(
                new PersPenalty(mul, w)));
  }

  double m[4], w;
};

// try to (lightly) enforece pinhole model
struct LineZ3GenericCenterError {
  LineZ3GenericCenterError() {}
      
  template <typename T>
  bool operator()(const T* const line,
                  T* residuals) const {
    residuals[0] = line[0]*T(1e-4);
    residuals[1] = line[1]*T(1e-4);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create() {
    //should reduce this to 2 but ceres complains about the same pointer being added with four params
    return (new ceres::AutoDiffCostFunction<LineZ3GenericCenterError, 2, 4>(
                new LineZ3GenericCenterError()));
  }
};

// try to (lightly) enforece pinhole model  - this may still let the model detoriate the model to a plane!
struct LineZ3GenericStepError {
  LineZ3GenericStepError(double d) : d_(d) {}
      
  template <typename T>
  bool operator()(const T* const extr1, const T* const extr2,
                  T* residuals) const {
    
    T step[3];
    
    step[0] = extr1[3]-extr2[3];
    step[1] = extr1[4]-extr2[4];
    step[2] = extr1[5]-extr2[5];
    
    residuals[0] = T(d_) - sqrt(step[0]*step[0] + step[1]*step[1] + step[2]*step[2]);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double step) {
    //should reduce this to 2 but ceres complains about the same pointer being added with four params
    return (new ceres::AutoDiffCostFunction<LineZ3GenericStepError, 1, 6, 6>(
                new LineZ3GenericStepError(step)));
  }
  
  double d_;
};


// assumes projection center in negative direction!
struct ProjCenterAngError {
  ProjCenterAngError(cv::Vec4d line, double z_ref)
      : ox(line[0]), oy(line[1]), dx(line[2]), dy(line[3]), zr(z_ref) {}
      
  template<typename T> static inline T norm(const T v[3])
  {
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  }

      
  template<typename T> 
  bool operator()(const T* const center,
                  T* residuals) const {
    //assumes center in neg. z direction
    T z_line = T(zr)+center[2];
    T d_center[3];
    d_center[0] = center[0] - (T(ox) + T(dx)*z_line);
    d_center[1] = center[1] - (T(oy) + T(dy)*z_line);
    d_center[2] = center[2] - z_line;
    T d_line[3]= {T(dx), T(dy), T(1)};
    T cross[3];
    
    ceres::CrossProduct(d_center, d_line, cross);
    
    residuals[0] = norm(cross)/(norm(d_center)*norm(d_line));
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Vec4d line, double z_ref) {
    return (new ceres::AutoDiffCostFunction<ProjCenterAngError, 1, 3>(
                new ProjCenterAngError(line, z_ref)));
  }

  double ox,oy,dx,dy,zr;
};

//normalized image coordinates (img center at 0,0)
struct RectProjError {
  RectProjError(cv::Vec4d line, cv::Point2d ip, double z_ref, double weight)
      : w(weight)
      {
        if (z_ref) {
          wx = line[0]/z_ref+line[2];
          wy = line[1]/z_ref+line[3];
        }
        else {
          wx = line[2];
          wy = line[3];
        }
        ix = ip.x;
        iy = ip.y;
      }

      
  template<typename T> 
  bool operator()(const T* const r, const T* const p, const T* const m,
                  T* residuals) const {
    T px = (cos(r[0])*T(wx) - sin(r[0])*T(wy) + m[0])*p[0];    
    T py = (sin(r[0])*T(wx) + cos(r[0])*T(wy) + m[1])*p[1];    
    residuals[0] = (T(ix)-px)*T(w);
    residuals[1] = (T(iy)-py)*T(w);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Vec4d line, cv::Point2d ip, double z_ref, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjError, 2, 1, 2, 2>(
                new RectProjError(line, ip, z_ref, weight)));
  }

  double wx,wy,ix,iy,w;
};
/*
struct RectProjRotError {
  RectProjRotError(cv::Vec4d line, cv::Point2d ip, double z_ref, double weight, double fx, double fy)
      : w(weight)
      {
        wx = line[0]/z_ref+line[2];
        wy = line[1]/z_ref+line[3];
        ix = ip.x;
        iy = ip.y;
        _fx = fx;
        _fy = fy;
      }

      
  template<typename T> 
  bool operator()(const T* const p, const T* const r, const T* const m,
                  T* residuals) const {
    T px = (cos(r[0])*T(wx) - sin(r[0])*T(wy) + m[0])*_fx;    
    T py = (sin(r[0])*T(wx) + cos(r[0])*T(wy) + m[1])*_fy;    
    residuals[0] = (T(ix)-px)*T(w);
    residuals[1] = (T(iy)-py)*T(w);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Vec4d line, cv::Point2d ip, double z_ref, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjRotError, 2, 2, 1, 2>(
                new RectProjRotError(line, ip, z_ref, weight)));
  }

  double wx,wy,ix,iy,w;
  double _fx, _fy;
};*/

struct RectProjDynError {
  RectProjDynError(cv::Point2d ip, double weight)
      : w(weight)
      {
        ix = ip.x;
        iy = ip.y;
      }

      
  template<typename T> 
  bool operator()(const T* const p, const T* const r, const T* const m, const T* const l,
                  T* residuals) const {
    T wx = l[2]; //in infinity l[0] and l[1] don't matter
    T wy = l[3];
    T px = (cos(r[0])*wx - sin(r[0])*wy + m[0])*p[0];    
    T py = (sin(r[0])*wx + cos(r[0])*wy + m[1])*p[1];    
    residuals[0] = (T(ix)-px)*T(w)*T(1);
    residuals[1] = (T(iy)-py)*T(w)*T(1);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(cv::Point2d ip, double weight) {
    return (new ceres::AutoDiffCostFunction<RectProjDynError, 2, 2, 1, 2, 4>(
                new RectProjDynError(ip, weight)));
  }

  double ix,iy,w;
};

// origin (p1): (p0,p1,0), dir(p2): (p2,p3,1)
struct LineZ3dError {
  LineZ3dError(double x, double y, double z, double sx, double sy)
      : x_(x), y_(y), z_(z), sx_(sx), sy_(sy) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const { 
    residuals[0] = (T(x_) - (p[0]+T(z_)*p[2]))*T(sx_);
    residuals[1] = (T(y_) - (p[1]+T(z_)*p[3]))*T(sy_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double z, double sx, double sy) {
    return (new ceres::AutoDiffCostFunction<LineZ3dError, 2, 4>(
                new LineZ3dError(x, y, z, sx, sy)));
  }

  double x_,y_,z_, sx_, sy_;
};

// origin (p1): (p0,p1,0), dir(p2): (p2,p3,1)
struct LineZ3dUwError {
  LineZ3dUwError(double x, double y, double z, double sx, double sy, double w)
      : x_(x), y_(y), z_(z), sx_(sx), sy_(sy), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const { 
    residuals[0] = (T(x_) - (p[0]+T(z_)*p[2]))*T(sx_)*T(w_);
    residuals[1] = (T(y_) - (p[1]+T(z_)*p[3]))*T(sy_)*T(w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double z, double sx, double sy, double w) {
    return (new ceres::AutoDiffCostFunction<LineZ3dUwError, 2, 4>(
                new LineZ3dUwError(x, y, z, sx, sy, w)));
  }

  double x_,y_,z_, sx_, sy_, w_;
};


double fit_line_s(vector<Point3f> &points, vector<Point2f> &scales, double *line)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.minimizer_progress_to_stdout = true;

  //options.num_threads = 2;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;*/
  
  line[0] = points[0].x;
  line[1] = points[0].y;
  line[2] = 0.0;
  line[3] = 0.0;
  
  ceres::Solver::Summary summary;
  
  ceres::Problem problem;
  for(int i=0;i<points.size();i++) {
    ceres::CostFunction* cost_function =
        LineZ3dError::Create(points[i].x,points[i].y,points[i].z, scales[i].x, scales[i].y);
        
    problem.AddResidualBlock(cost_function,
                            NULL,
                            line);
  }
  
  ceres::Solve(options, &problem, &summary);
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double fit_line(vector<Point3f> &points, double *line)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.minimizer_progress_to_stdout = true;
  
  line[0] = points[0].x;
  line[1] = points[0].y;
  line[2] = 0.0;
  line[3] = 0.0;
  
  ceres::Solver::Summary summary;

  
  ceres::Problem problem;
  for(int i=0;i<points.size();i++) {
    ceres::CostFunction* cost_function =
        LineZSimple3dError::Create(points[i].x,points[i].y,points[i].z);
        
    problem.AddResidualBlock(cost_function,
                            NULL,
                            line);
  }
  
  ceres::Solve(options, &problem, &summary);
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

static void _zline_problem_add_pinhole_lines(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  for(int i=0;i<proxy_backwards.size();i+=i_step)
    for(int y=0;y<proxy_size.y;y+=y_step)
      for(int x=0;x<proxy_size.x;x+=x_step) {
        Point2f p = proxy_backwards[i][y*proxy_size.x+x];
        
        if (isnan(p.x) || isnan(p.y))
          continue;
        
        //keep center looking straight
        if (y == proxy_size.y/2 && x == proxy_size.x/2) {
          ceres::CostFunction* cost_function =
              LineZ3GenericCenterLineError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function, NULL, 
                                   extrinsics+i*6);
        }
        //regular line error (in x/y world direction)
        else {
          ceres::CostFunction* cost_function = 
              LineZ3GenericPinholeError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  extrinsics+i*6,
                                  lines + (y*proxy_size.x+x)*4);
        }
      }
}

static void _zline_problem_add_pinhole_lines_extra(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step, double *rel_cam_extrinsics)
{
  for(int i=0;i<proxy_backwards.size();i+=i_step)
    for(int y=0;y<proxy_size.y;y+=y_step)
      for(int x=0;x<proxy_size.x;x+=x_step) {
        Point2f p = proxy_backwards[i][y*proxy_size.x+x];
        
        if (isnan(p.x) || isnan(p.y))
          continue;
        
        //keep center looking straight
        if (y == proxy_size.y/2 && x == proxy_size.x/2) {
          ceres::CostFunction* cost_function =
              LineZ3GenericCenterLineExtraError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function, NULL, rel_cam_extrinsics, 
                                   extrinsics+i*6);
        }
        //regular line error (in x/y world direction)
        else {
          ceres::CostFunction* cost_function = 
              LineZ3GenericPinholeExtraError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  extrinsics+i*6, rel_cam_extrinsics, 
                                  lines + (y*proxy_size.x+x)*4);
        }
      }
}


static void _zline_problem_add_generic_lines(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  ceres::CostFunction* cost_function;
  
  for(int i=0;i<proxy_backwards.size();i+=i_step)
    for(int y=0;y<proxy_size.y;y+=y_step)
      for(int x=0;x<proxy_size.x;x+=x_step) {
        Point2f p = proxy_backwards[i][y*proxy_size.x+x];
        
        if (isnan(p.x) || isnan(p.y))
          continue;
        
        //keep center looking straight
        if (y==proxy_size.y/2 && x == proxy_size.x/2) {
          cost_function = LineZ3GenericCenterLineError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   extrinsics+i*6);
        }
        else {
          cost_function = LineZ3GenericError::Create(p.x, p.y);
          problem.AddResidualBlock(cost_function,
                                   NULL,
                                   extrinsics+i*6,
                                   lines + (y*proxy_size.x+x)*4);
        }
      }
}


static void _zline_problem_add_generic_xy_w(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  for(int i=0;i<proxy_backwards.size();i+=i_step)
    for(int y=0;y<proxy_size.y;y+=y_step)
      for(int x=0;x<proxy_size.x;x+=x_step) 
        //keep center looking straight
        if (y==proxy_size.y/2 && x == proxy_size.x/2) {
          ceres::CostFunction* cost_function =
              LineZ3GenericCenterLineError::Create(proxy_backwards[i][y*proxy_size.x+x].x,proxy_backwards[i][y*proxy_size.x+x].y);
              
          problem.AddResidualBlock(cost_function,
                                NULL,
                                extrinsics+i*6);
        }
        else if (proxy_size.y-y_step == y || proxy_size.x-x_step == x) {
        ceres::CostFunction* cost_function =
            LineZ3GenericError::Create(proxy_backwards[i][y*proxy_size.x+x].x,proxy_backwards[i][y*proxy_size.x+x].y);
            
        problem.AddResidualBlock(cost_function,
                              NULL,
                              extrinsics+i*6,
                              lines + (y*proxy_size.x+x)*4);
        }
        else 
        {
        ceres::CostFunction* cost_function =
            LineZ3GenericWeightXYError::Create(proxy_backwards[i][y*proxy_size.x+x].x,proxy_backwards[i][y*proxy_size.x+x].y);
            
        problem.AddResidualBlock(cost_function,
                              NULL,
                              extrinsics+i*6,
                              lines + (y*proxy_size.x+x)*4,
                              lines + (y*proxy_size.x+(x+1))*4,
                              lines + ((y+1)*proxy_size.x+x)*4);
        }
        
}

static void _zline_problem_add_center_errors(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  //enforce pinhole model
  for(int y=0;y<proxy_size.y;y+=y_step)
    for(int x=0;x<proxy_size.x;x+=x_step)  {
      ceres::CostFunction* cost_function = LineZ3GenericCenterError::Create();
      problem.AddResidualBlock(cost_function, NULL,
                              lines + (y*proxy_size.x+x)*4);
    }
}

static void _zline_problem_add_step_errors(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  for(int i=0;i<proxy_backwards.size()-i_step;i+=i_step) {
    //penalize deviation from pinhole model
    ceres::CostFunction* cost_function = LineZ3GenericStepError::Create(z_step*i_step);
        
    problem.AddResidualBlock(cost_function,
                          NULL,
                          extrinsics+i*6,
                          extrinsics+(i+i_step)*6);
  }
}


static void _zline_problem_add_non_z_rotation_penalty(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  for(int i=0;i<proxy_backwards.size();i+=i_step) {
    ceres::CostFunction* cost_function = NonZRotationError::Create();
    problem.AddResidualBlock(cost_function,
                             NULL,
                             extrinsics+i*6);
  }
  
}

static void _zline_problem_add_proj_error(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, Point2i img_size, double z_step, int i_step, int y_step, int x_step, double *proj, double *r, double *m)
{
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*proj_center_size*proj_center_size;
  
  for(int y=0;y<proxy_size.y;y++)
    for(int x=0;x<proxy_size.x;x++) {
      Vec4d line = lines[y*proxy_size.x+x];
      
        Point2d ip = Point2d((x+0.5-proxy_size.x*0.5)*img_size.x/proxy_size.x,(y+0.5-proxy_size.y*0.5)*img_size.y/proxy_size.y);
        
        double w = exp(-((ip.x*ip.x+ip.y*ip.y)/two_sigma_squared));
        //w_sum += w;
        w = sqrt(w);
        
        
        ceres::CostFunction* cost_function =
            RectProjDynError::Create(ip, w);
            
        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  proj, r, m, &lines[4*(y*proxy_size.x+x)]);
    }
}

//penalty to enforce rotational symmetry (center weighted with a gaussian)
static void _zline_problem_add_4step_rot_symm_penalty(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{
  int half_size = min(proxy_size.x/2, proxy_size.y/2);
  Point2d c, pos, r_pos;
  double w;
  
  c = proxy_size*0.5;
  
  double sin90[5] = {0,1,0,-1,0};
  
  typedef Matx<int, 2, 2 > Matx22s;
  
  //upper left quadrant, include center for x, exclude for y
  for(int y=proxy_size.y/2-half_size;y<proxy_size.y/2;y+=y_step)
    for(int x=proxy_size.x/2-half_size;x<=proxy_size.x/2;x+=x_step)
      if (x != proxy_size.x/2) {
        pos = Point2d(x,y);
        
        Point2d d = (pos-c)*(1.0/norm(Point2d(half_size,half_size)));
        double s2s = 2*0.1*0.1;
        
        w = exp(-(d.x*d.x+d.y*d.y)/s2s);
        
        
        for(int r=2;r<4;r+=2) {
          Matx22d rot(sin90[r+1], -sin90[r],sin90[r],sin90[r+1]);
          r_pos = rot*(pos-c)+c;
          
          //penalize deviation from rotational symmetry (in 90degree steps)
          ceres::CostFunction* cost_function = RotSymmPenalty::Create(rot.val, w);
              
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  lines+(y*proxy_size.x+x)*4,
                                  lines+((int(r_pos.y)*proxy_size.x+int(r_pos.x))*4));

        }
        
        for(int r=1;r<4;r+=2) {
          Matx22d rot(sin90[r+1], -sin90[r],sin90[r],sin90[r+1]);
          r_pos = rot*(pos-c)+c;
          
          double s = (2160.0/25.0)/(2560.0/33.0);
          
          rot.val[1] *= s;
          rot.val[2] *= 1.0/s;
          
          //penalize deviation from rotational symmetry (in 90degree steps)
          ceres::CostFunction* cost_function = RotSymmPenalty::Create(rot.val, w);
              
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  lines+(y*proxy_size.x+x)*4,
                                  lines+(int(r_pos.y)*proxy_size.x+int(r_pos.x))*4);

        }
      }
}


//enforce a low perspective projection component of the fitted lines
static void _zline_problem_add_reg_penalty(ceres::Problem &problem, std::vector<std::vector<cv::Point2f>> &proxy_backwards, double *extrinsics, double *lines, Point2i proxy_size, double z_step, int i_step, int y_step, int x_step)
{  
  for(int y=0;y<proxy_size.y;y+=y_step)
    for(int x=0;x<proxy_size.x-2*x_step;x+=x_step) {
        ceres::CostFunction* cost_function = RegGridPenalty::Create();
              
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  lines+(y*proxy_size.x+x)*4,
                                  lines+(y*proxy_size.x+(x+1))*4,
                                  lines+(y*proxy_size.x+(x+2))*4);
    }
    
  for(int y=0;y<proxy_size.y-2*y_step;y+=y_step)
    for(int x=0;x<proxy_size.x;x+=x_step) {
        ceres::CostFunction* cost_function = RegGridPenalty::Create();
              
          problem.AddResidualBlock(cost_function,
                                  NULL,
                                  lines+(y*proxy_size.x+x)*4,
                                  lines+((y+1)*proxy_size.x+x)*4,
                                  lines+((y+2)*proxy_size.x+x)*4);
    }
}

/*
 * optimize together:
 *  - per image camera movement (world rotation and translation)
 *  - individual lines (which do not need to converge in an optical center
 */
double fit_cams_lines(std::vector<std::vector<cv::Point2f>> &proxy_backwards, std::vector<cv::Vec4d> &linefits, Point2i proxy_size, Point2i img_size, double z_step, vector<double> &extrinsics_v)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 10000;
  options.function_tolerance = 1e-3;
  //options.minimizer_progress_to_stdout = true;
  //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  
  //for threading!
  options.linear_solver_type = ceres::DENSE_SCHUR;
  
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;

  double lines[4*proxy_size.x*proxy_size.y];
  extrinsics_v.resize(6*proxy_backwards.size());
  double *extrinsics = &extrinsics_v[0];
  
  double r = 0;
  double m[2] = {0,0};
  double proj[2] = {1000,1000};
  
  //first step: we assume pinhole model for rough calibration
  
  for(int y=0;y<proxy_size.y;y++)
    for(int x=0;x<proxy_size.x;x++) {
      double *line = &lines[(y*proxy_size.x+x)*4];
      line[0] = 0.0;
      line[1] = 0.0;
      line[2] = 0.0;
      line[3] = 0.0;
    }
    
  for(int i=0;i<proxy_backwards.size();i++) {
    extrinsics[i*6+0] = 0;
    extrinsics[i*6+1] = 0;
    extrinsics[i*6+2] = 0;
    extrinsics[i*6+3] = 0;
    extrinsics[i*6+4] = 0;
    //only translate in z
    //FIXME getting this approximately right seems to be very important! 
    //FIXME random values also work quite well
    //extrinsics[i*6+5] = 400+i*z_step;
    extrinsics[i*6+5] = 500;/// rand() % 1000;
  }
  
  ceres::Solver::Summary summary;

  ceres::Problem problem;
  
  
  _zline_problem_add_pinhole_lines(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_center_errors(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_step_errors(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_reg_penalty(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_non_z_rotation_penalty(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_4step_rot_symm_penalty(problem, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_proj_error(problem, proxy_backwards, extrinsics, lines, proxy_size, img_size, z_step, 1, 1, 1, proj, &r, m);
  
  printf("solving pinhole problem...\n");
  ceres::Solve(options, &problem, &summary);
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
  //std::cout << summary.FullReport() << "\n";
  
  //printf("rms: %f\n",2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  
/////////////////////////////77
  /*for(int y=0;y<proxy_size.y;y++)
    for(int x=0;x<proxy_size.x;x++) {
      double *line = &lines[(y*proxy_size.x+x)*4];
      linefits[y*proxy_size.x+x][0] = line[0];
      linefits[y*proxy_size.x+x][1] = line[1];
      linefits[y*proxy_size.x+x][2] = line[2];
      linefits[y*proxy_size.x+x][3] = line[3];
  }
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());*/
////////////////////////////
  
  options.max_num_iterations = 10000;
  options.function_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  
  ceres::Problem problem_full;
  
  //_zline_problem_add_generic_xy_w(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_generic_lines(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_center_errors(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_step_errors(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_reg_penalty(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_non_z_rotation_penalty(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_4step_rot_symm_penalty(problem_full, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_proj_error(problem_full, proxy_backwards, extrinsics, lines, proxy_size, img_size, z_step, 1, 1, 1, proj, &r, m);
  
  printf("solving constrained generic problem...\n");
  ceres::Solve(options, &problem_full, &summary);
  printf("solved\n");
  //std::cout << summary.FullReport() << "\n";
  //printf("^-- full full problem\n");
  
  
  
  /*ceres::Problem problem_full2;
  
  //FIXME should be zero anyway?!
  lines[(proxy_size.y/2*proxy_size.x+proxy_size.x/2)*4] = 0;
  lines[(proxy_size.y/2*proxy_size.x+proxy_size.x/2)*4+1] = 0;
  lines[(proxy_size.y/2*proxy_size.x+proxy_size.x/2)*4+2] = 0;
  lines[(proxy_size.y/2*proxy_size.x+proxy_size.x/2)*4+3] = 0;
  
  //_zline_problem_add_generic_xy_w(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  _zline_problem_add_generic_lines(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_center_errors(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_step_errors(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_reg_penalty(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_non_z_rotation_penalty(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_4step_rot_symm_penalty(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  //_zline_problem_add_proj_error(problem_full2, proxy_backwards, extrinsics, lines, proxy_size, img_size, z_step, 1, 1, 1, proj, &r, m);
  
  printf("solving full generic problem...\n");
  ceres::Solve(options, &problem_full2, &summary);
  
  std::cout << summary.FullReport() << "\n";*/
  //printf("^-- full2 problem\n");
  
  
  
  for(int y=0;y<proxy_size.y;y++)
    for(int x=0;x<proxy_size.x;x++) {
      double *line = &lines[(y*proxy_size.x+x)*4];
      linefits[y*proxy_size.x+x][0] = line[0];
      linefits[y*proxy_size.x+x][1] = line[1];
      linefits[y*proxy_size.x+x][2] = line[2];
      linefits[y*proxy_size.x+x][3] = line[3];
      
      //printf("%dx%d : %fx%f + z* %fx%f\n",x, y, line[0],line[1],line[2],line[3]);
    }
    
  /*for(int i=0;i<proxy_backwards.size();i++)
    printf("rot %fx%fx%f trans %fx%fx%f\n", extrinsics[i*6+0],
                                            extrinsics[i*6+1],
                                            extrinsics[i*6+2],
                                            extrinsics[i*6+3],
                                            extrinsics[i*6+4],
                                            extrinsics[i*6+5]
    );*/
  
  //reset center line
  /*int y = proxy_size.y/2;
  int x = proxy_size.x/2;
  double *line = &lines[(y*proxy_size.x+x)*4];
  line[0] = 0.0;
  line[1] = 0.0;
  line[2] = 0.0;
  line[3] = 0.0;*/
  
  printf("rms: %f\n",2.0*sqrt(summary.final_cost/problem_full.NumResiduals()));
  
  options.max_num_iterations = 0;
  
  ceres::Problem problem_calc_only;
  
  //options.minimizer_progress_to_stdout = true;
  
  _zline_problem_add_generic_lines(problem_calc_only, proxy_backwards, extrinsics, lines, proxy_size, z_step, 1, 1, 1);
  
  ceres::Solve(options, &problem_calc_only, &summary);
  
  //std::cout << summary.FullReport() << "\n";
  
  return 2.0*sqrt(summary.final_cost/problem_full.NumResiduals());
}

const int max_ransac_tries = 100;

void line_from_2p(Point3f a, Point3f b, double *line)
{
  Point3f dir = b-a;
  
  dir *= 1.0/dir.z;
  
  a = a-dir*a.z;
  
  line[0] = a.x;
  line[1] = a.y;
  line[2] = dir.x;
  line[3] = dir.y;
}


double p_line_dist2_xy(Point3f a, double *line)
{
  double dx = line[0]+line[2]*a.z - a.x;
  double dy = line[1]+line[3]*a.z - a.y;
  
  return dx*dx+dy*dy;
}

//for use within ransac: only use points with max distance
double fit_line_selective(vector<Point3f> &points, double *line, double maxdist, int &count, vector<int> *include)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  
  count = 0;
  ceres::Problem problem;
  for(int i=0;i<points.size();i++)
    if (p_line_dist2_xy(points[i], line) <= maxdist*maxdist) {
      count++;
      if (include)
        (*include)[i] = 1;
      ceres::CostFunction* cost_function =
          LineZSimple3dError::Create(points[i].x,points[i].y,points[i].z);
          
      problem.AddResidualBlock(cost_function,
                              NULL,
                              line);
    }
    else {
      if (include)
        (*include)[i] = 0;
    }
  
  ceres::Solve(options, &problem, &summary);
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double fit_line_ransac(vector<Point3f> &points, double *line, double incl_th, double succ_th, vector<int> *include)
{
  double rms;
  
  for(int i=0;i<100;i++) {
    double initline[4];
    int i1 = rand() % points.size();
    int i2 = i1;
    
    while (i1 == i2)
      i2 = rand() % points.size();
    
    line_from_2p(points[i1], points[i2], initline);
    
    int count = 0;
    int newcount = 2;
    
    while (newcount > count) {
      count = newcount;
      rms = fit_line_selective(points, initline, incl_th, newcount, include);
    }
    
    if (count >= succ_th*points.size()) {
      for(int i=0;i<4;i++)
        line[i] = initline[i];
      return rms;
    }
  }
  
  for(int i=0;i<4;i++)
    line[i] = std::numeric_limits<double>::quiet_NaN();
  
  return FLT_MAX;
}


double fit_line_suw(vector<Point3f> &points, vector<Point2f> &scales, vector<int> &counts, double *line)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.minimizer_progress_to_stdout = true;

  //options.num_threads = 2;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;*/
  
  double w_sum;
  
  line[0] = points[0].x;
  line[1] = points[0].y;
  line[2] = 0.0;
  line[3] = 0.0;
  
  ceres::Solver::Summary summary;
  
  ceres::Problem problem;
  for(int i=0;i<points.size();i++)
    if (counts[i] >= pers_fit_min_count) {
        double w = sqrt(counts[i]);
        w_sum += w;
        w = sqrt(w);
        ceres::CostFunction* cost_function =
            LineZ3dUwError::Create(points[i].x,points[i].y,points[i].z, scales[i].x, scales[i].y, w);
            
        problem.AddResidualBlock(cost_function,
                                 NULL, //new ceres::CauchyLoss(0.5)
                                line);
      }
  
  ceres::Solve(options, &problem, &summary);
  
  return 2.0*sqrt(summary.final_cost/w_sum);
}

//tries to minimize angular difference between center of projection and lines in space for some specific (reference) distance
double fit_center_of_projection(std::vector<cv::Vec4d> &lines, Point3d &center, double z_ref)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  
  //options.minimizer_progress_to_stdout = true;
  
  double c[3] = {0,0,0};
  
  ceres::Solver::Summary summary;
  
  ceres::Problem problem;
  for(int i=0;i<lines.size();i++)
    if (!std::isnan(lines[i][0])) {
      ceres::CostFunction* cost_function =
          ProjCenterAngError::Create(lines[i], z_ref);
          
      problem.AddResidualBlock(cost_function,
                                NULL, //new ceres::CauchyLoss(0.5)
                              c);
    }
  
  ceres::Solve(options, &problem, &summary);
  
  center = Point3d(c[0],c[1],c[2]);
  
  std::cout << summary.FullReport() << "\n";
  
  return sqrt(summary.final_cost/problem.NumResiduals());
}

double fit_projection(std::vector<cv::Vec4d> &lines, Point2i img_size, Point2i fit_size, Point2d &projection, double &rot, Point2d &move, double z_ref)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  
  //printf("start fit proj to %dx%d %dx%d!\n", img_size.x,img_size.y,fit_size.x,fit_size.y);
  
  double proj[2] = {1000,1000};
  double r = 0;
  double m[2] = {0,0};
  
  double two_sigma_squared = 2.0*(img_size.x*img_size.x+img_size.y*img_size.y)*proj_center_size*proj_center_size;
  double w_sum;
  
  ceres::Problem problem;
  for(int y=0;y<fit_size.y;y++)
    for(int x=0;x<fit_size.x;x++) {
      Vec4d line = lines[y*fit_size.x+x];

      if (std::isnan(line[0]))
        continue;
      
      Point2d ip = Point2d((x+0.5-fit_size.x*0.5)*img_size.x/fit_size.x,(y+0.5-fit_size.y*0.5)*img_size.y/fit_size.y);
      
      double w = exp(-((ip.x*ip.x+ip.y*ip.y)/two_sigma_squared));
      w_sum += w;
      w = sqrt(w);
      
      ceres::CostFunction* cost_function =
          RectProjError::Create(line, ip, z_ref, w);
          
      problem.AddResidualBlock(cost_function,
                                NULL,
                              &r, proj, m);
    }
  
  ceres::Solve(options, &problem, &summary);
  
  projection.x = proj[0];
  projection.y = proj[1];
  rot = r;
  //movement of the dir vector (not at depth!)
  move.x = m[0];
  move.y = m[1];
  
  //std::cout << summary.FullReport() << "\n";
  
  return 2.0*sqrt(summary.final_cost/w_sum);
}
