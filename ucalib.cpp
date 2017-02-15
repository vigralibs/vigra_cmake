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

using namespace std;
using namespace MetaMat;

namespace ucalib {

  class RayCalib : public Calib {
  public:
    RayCalib(Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &proj, Mat_<double> &rays, cv::Point2i img_size);
    RayCalib() {};
    RayCalib(std::function<Mat(cpath)> load_mat, std::function<const char *(cpath)> load_string);
    //empty idx specifies first cam or view respectively
    virtual Cam *cam(const Idx &cam) const;
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam, const Idx &view) const;
    virtual const Mat_<double> &extrinsics_cams() const;
    virtual const Mat_<double> &extrinsics_views() const;
    virtual const Mat_<double> &proj();
    virtual const Mat_<double> &rays();
    //virtual cv::Size img_size() const;
    virtual int features() const { return Depth_Required | Rectification; };
    virtual void save(std::function<void(cpath,Mat)> save_mat, std::function<void(cpath,const char *)> save_string);
#ifdef UCALIB_WITH_MM_MESH
    virtual Mesh target_mesh() const;
#endif
    
    virtual void rectify(const cv::Mat &src, cv::Mat &dst, const Idx &view_idx, double z) const;
    
    friend Calib* calibrate_rays(Mat_<float> &proxy, const Mat_<float> &j, cv::Point2i img_size, const Options &opts, const DimSpec &views_dims_start);
    friend double fit_cams_lines_multi(Mat_<float> &proxy, int first_view_dim, cv::Point2i img_size, ucalib::RayCalib &c, bool vis, const Mat_<float>& j, const ucalib::Options &opts);
    friend class RayCam;
  private:
    //cv::Point2i _img_size;
    Mat_<double> _rays; //same size as cam_extrinsics
    Mat_<double> _cams; //same size as cam_extrinsics
    Mat_<double> _views; //same size as cam_extrinsics
    Mat_<double> _proj; //same size as cam_extrinsics
    Mat_<double> _mesh; //same size as cam_extrinsics
  };
  
}

void get_undist_map_for_depth(Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f)
{
  cv::Point2i fdim(lines[1],lines[2]);
  
  cv::Mat_<cv::Point2d> offset(fdim.y, fdim.x);
  cv::Mat_<cv::Point2d> grad(fdim.y, fdim.x);
  
  cv::Ptr<cv::ThinPlateSplineShapeTransformer> transform;
  
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
    
  bool transform_failed = true;
  int skippoints = 0;
  while (transform_failed) {
    std::vector<cv::Point2f> cv_wpoints, cv_ipoints;
    std::vector<cv::DMatch> matches;
    for(int y=0;y<fdim.y;y++)
      for(int x=(skippoints+1)/2;x<fdim.x-skippoints/2;x++) {
        
        if (std::isnan(offset.at<cv::Point2d>(y,x).x))
          continue;
        
        double wx = (x+0.5)*idim.x/fdim.x + offset.at<cv::Point2d>(y,x).x + (1.0/z)*grad.at<cv::Point2d>(y,x).x;
        double wy = (y+0.5)*idim.y/fdim.y + offset.at<cv::Point2d>(y,x).y + (1.0/z)*grad.at<cv::Point2d>(y,x).y;
        

        matches.push_back(cv::DMatch(cv_wpoints.size(),cv_wpoints.size(), 0));
        cv_wpoints.push_back(cv::Point2f(wx, wy));
        cv_ipoints.push_back(cv::Point2f((x+0.5)*idim.x/fdim.x, (y+0.5)*idim.y/fdim.y));
      }


    transform = cv::createThinPlateSplineShapeTransformer(0);
    transform->estimateTransformation(cv_wpoints, cv_ipoints, matches);
    
    //check for failure
    std::vector<cv::Point2f> perfect_points = {{10,10}, {100,100}};
    std::vector<cv::Point2f> ipoints(2);
    transform->applyTransformation(perfect_points, ipoints);
    
    if (ipoints[0] != cv::Point2f(0,0) && ipoints[1] != cv::Point2f(0,0))
      transform_failed = false;
    else
      skippoints++;
  }
  
  if (skippoints) {
    printf("WARNING: needed to skip %d rows of samples due to interpolation issues!\n", skippoints);
  }

  int approx_step = 16;
  
  //printf("calc ipoints\n");
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

void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints)
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

// Pinhole line error
struct LineZ3DirPinholeExtraError {
  LineZ3DirPinholeExtraError(double x, double y, double w = 1.0)
      : x_(x), y_(y), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const extr_cams, const T* const extr_views, const T* const line,
                  T* residuals) const {
                    
                    
    T p[3], p1[3], w_p[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    ceres::AngleAxisRotatePoint(extr_views, w_p, p1);

    // camera[3,4,5] are the translation.
    p1[0] += extr_views[3];
    p1[1] += extr_views[4];
    p1[2] += extr_views[5];
    
    
    ceres::AngleAxisRotatePoint(extr_cams, p1, p);

    // camera[3,4,5] are the translation.
    p[0] += extr_cams[3];
    p[1] += extr_cams[4];
    p[2] += extr_cams[5];
    
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

      
template <typename T> void add_mul_vecs(const T* const a, const T* const b, double scale_b, T *res)
{
  res[0] = a[0]+T(scale_b)*b[0];
  res[1] = a[1]+T(scale_b)*b[1];
  res[2] = a[2]+T(scale_b)*b[2];
}

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
    residuals[0] = (T(j_(0, 0)) * p_t[0] + T(j_(0, 1)) * p_t[1]) * T(1/0.1);
    residuals[1] = (T(j_(1, 0)) * p_t[0] + T(j_(1, 1)) * p_t[1]) * T(1/0.1);
    
//     residuals[0] = p_c[0];
//     residuals[1] = p_c[1];
    
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
struct LineZ3GenericMeshExtraError {
  LineZ3GenericMeshExtraError(double x, double y, double ws[4], cv::Mat_<float> j)
      : x_(x), y_(y) { for(int i=0;i<4;i++) ws_[i] = ws[i]; j_ = j.clone(); }
      
  template <typename T>
  bool operator()(const T* const extr_cams, const T* const extr_views, const T* const o, const T* const d, const T* const tl, const T* const tr, const T* const bl,  const T* const br,
                  T* residuals) const {
                    
                    
    T p[3], w_p[3], p_v[3];
    w_p[0] = T(x_);
    w_p[1] = T(y_);
    w_p[2] = T(0);
    
    add_mul_vecs(w_p, tl, ws_[0], w_p);
    add_mul_vecs(w_p, tr, ws_[1], w_p);
    add_mul_vecs(w_p, bl, ws_[2], w_p);
    add_mul_vecs(w_p, br, ws_[3], w_p);
    
    T mesh_d[3] = {w_p[0]-T(x_),w_p[1]-T(y_),w_p[2]};
    
    ceres::AngleAxisRotatePoint(extr_views, w_p, p_v);

    // camera[3,4,5] are the translation.
    p_v[0] += extr_views[3];
    p_v[1] += extr_views[4];
    p_v[2] += extr_views[5];
    
    ceres::AngleAxisRotatePoint(extr_cams, p_v, p);

    // camera[3,4,5] are the translation.
    p[0] += extr_cams[3];
    p[1] += extr_cams[4];
    p[2] += extr_cams[5];
    
    T p_c[3]; //res in cam coordinates
    T p_t[3]; //res in target coordinates
    T r_neg[3] = {-extr_cams[0],-extr_cams[1],-extr_cams[2]};
    
    //compare with projected pinhole camera ray
    p_c[0] = (p[0] - (o[0] + p[2]*d[0]));
    p_c[1] = (p[1] - (o[1] + p[2]*d[1]));
    p_c[2] = T(0);
    
    ceres::AngleAxisRotatePoint(r_neg, p_c, p_t);
    
    //TODO check p_t[2] == 0 -> no-not the case...
    residuals[0] = (T(j_(0, 0)) * p_t[0] + T(j_(0, 1)) * p_t[1]) ;//* T(1/0.1);
    residuals[1] = (T(j_(1, 0)) * p_t[0] + T(j_(1, 1)) * p_t[1]) ;//* T(1/0.1);
    
//     residuals[0] = p_c[0];
//     residuals[1] = p_c[1];
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double ws[4], cv::Mat_<float> j) {
    return (new ceres::AutoDiffCostFunction<LineZ3GenericMeshExtraError, 2, 6, 6, 2, 2, 3, 3, 3, 3>(
                new LineZ3GenericMeshExtraError(x, y, ws, j)));
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

struct calib_infos
{
  cv::Point2i img_size;
  cv::Point2i target_size;
  //proxy indices
  int cams_min, cams_max;
  int views_min, views_max;
  std::function<void(double sq, const Idx &ray)> sq_callback;
  Mat_<uint8_t> proxy_mask;
};

void _ref_cam_fix_extrinsics(const calib_infos &i, ceres::Problem &problem, const Idx &ray, Mat_<double> &extrinsics_cams)
{
  for(int j=i.cams_min;j<=i.cams_max;j++)
    if (ray[j])
      return;
      
  for(int n=0;n<6;n++)
    extrinsics_cams({n,ray.r(i.cams_min,i.cams_max)}) = 0;
  
  problem.SetParameterBlockConstant(&extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}));
}

static void _zline_problem_add_pinhole_lines(const calib_infos &i, ceres::Problem &problem, const Mat_<float>& proxy, Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &lines, Mat_<double> &proj, ceres::LossFunction *loss = NULL, bool falloff = true, double min_weight = non_center_rest_weigth, double proj_weight = 1.0)
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  double two_sigma_squared = 2.0*(i.img_size.x*i.img_size.x+i.img_size.y*i.img_size.y)*extr_center_size*extr_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  //channels == 0 && cams == 0
  int last_view = 0;
  int ray_count = 0;
  for(auto ray : Idx_It_Dims(proxy, 1, -1)) {
      
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    
    cv::Point2d ip = cv::Point2d((ray["x"]+0.5-proxy_size.x*0.5)*i.img_size.x/proxy_size.x,(ray["y"]+0.5-proxy_size.y*0.5)*i.img_size.y/proxy_size.y);
    
    if (std::isnan(p.x) || std::isnan(p.y))
      continue;
    
        
    //TODO randomize flag?
    //lines({2,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
    //lines({3,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
    
    ceres::CostFunction* cost_function = 
    LineZ3DirPinholeExtraError::Create(p.x, p.y);
    problem.AddResidualBlock(cost_function,
                            NULL,
                            &extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}),
                            &extrinsics_views({0,ray.r(i.views_min,i.views_max)}),
                            //use only direction part!
                            &lines({2,ray.r("x",i.cams_max)}));
    

   _ref_cam_fix_extrinsics(i, problem, ray, extrinsics_cams);
    
    
    if (proj_weight) {
      double w_proj;
      w_proj = proj_weight*calc_line_pos_weight(cv::Point2i(ray["x"], ray["y"]), proxy_size, proj_center_size, min_weight);
      if (w_proj) {
        cost_function = RectProjExtraError::Create(p, ip, w_proj);
        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  &extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}),
                                  &extrinsics_views({0,ray.r(i.views_min,i.views_max)}),
                                  &proj(0,ray.r(i.cams_min,i.cams_max)));
      
        //TODO optional bounds?!
        /*problem.SetParameterLowerBound(&proj(0,ray.r(i.cams_min,i.cams_max)), 0, 10);
        problem.SetParameterLowerBound(&proj(0,ray.r(i.cams_min,i.cams_max)), 1, 10);*/
        problem.SetParameterUpperBound(&proj(0,ray.r(i.cams_min,i.cams_max)), 0, 100000);
        problem.SetParameterUpperBound(&proj(0,ray.r(i.cams_min,i.cams_max)), 1, 100000);
      }
    }
        
    ray_count++;
    
    if (ray["y"] == center.y && ray["x"] == center.x) {
      lines({2,ray.r(1,i.cams_max)}) = 0;
      lines({3,ray.r(1,i.cams_max)}) = 0;
      problem.SetParameterBlockConstant(&lines({2,ray.r("x",i.cams_max)}));
    }
  }
}


static void _zline_problem_add_lines_gen_mesh(const calib_infos &i, ceres::Problem &problem, const Mat_<float>& proxy, Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &lines, Mat_<double> &mesh, bool reproj_error_calc_only = false, const Mat_<float> &proxy_j = Mat_<float>())
{
  cv::Point2i center(proxy["x"]/2, proxy["y"]/2);
  
  double two_sigma_squared = 2.0*(i.img_size.x*i.img_size.x+i.img_size.y*i.img_size.y)*extr_center_size*extr_center_size;
  cv::Point2i proxy_size(lines["x"], lines["y"]);
  
  cv::Point2d mesh_size(mesh[1],mesh[2]);
  
  Mat_<float> mesh_coverage({mesh[1],mesh[2]});
  
  cv::Point2d mesh_center(0,0);
  int mesh_counter = 0;
  
  cvMat(mesh_coverage).setTo(0);
  
  //channels == 0 && cams == 0
  int last_view = 0;
  int ray_count = 0;
  for(auto ray : Idx_It_Dims(proxy, 1, -1)) {
      
      
    cv::Point2f p(proxy({0,ray.r("x",-1)}),
                  proxy({1,ray.r("x",-1)}));
    
    
    cv::Point2d ip = cv::Point2d((ray["x"]+0.5-proxy_size.x*0.5)*i.img_size.x/proxy_size.x,(ray["y"]+0.5-proxy_size.y*0.5)*i.img_size.y/proxy_size.y);
    
    if (std::isnan(p.x) || std::isnan(p.y))
      continue;
    
    if (i.proxy_mask.size())
      if (!i.proxy_mask(ray.r(1,-1)))
        continue;
    
        
    //TODO randomize flag?
    //lines({2,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
    //lines({3,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
    
    
    ///////////////////// calc mesh weights /////////////////////
    
    double ws[4];
    double *points[4];
    
    cv::Point2d mesh_f = cv::Point2d(p.x / i.target_size.x * (mesh_size.x-2), p.y / i.target_size.y * (mesh_size.y-2));
    cv::Point2i mesh_i(floor(mesh_f.x),floor(mesh_f.y));
    
    mesh_center += mesh_f;
    mesh_counter++;
    
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
    
    
    ///////////////////// calc derivatives /////////////////////
    
    cv::Mat j;
    if (proxy_j.total()) {
      //FIXME NAMES!
      Mat j_bind = proxy_j;
      for(int b=ray.size()-1;b>=1;b--)
         j_bind = j_bind.bind(b+1, ray[b]);
      j = cvMat(j_bind).clone();
      invert(j, j);
    }
    else
      j = cv::Mat::eye(2, 2, CV_64F);
    
    /*if (!reproj_error_calc_only) {
      //push model away from pinhole!
      lines({0,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
      lines({1,ray.r("x",i.cams_max)}) += (rand() / double(RAND_MAX)-0.5)*0.1;
      for(int c=0;c<3;c++) {
        mesh(c,mesh_i.x, mesh_i.y) = (rand() / double(RAND_MAX)-0.5)*2;
        mesh(c,mesh_i.x+1, mesh_i.y) = (rand() / double(RAND_MAX)-0.5)*2;
        mesh(c,mesh_i.x, mesh_i.y+1) = (rand() / double(RAND_MAX)-0.5)*2;
        mesh(c,mesh_i.x+1, mesh_i.y+1) = (rand() / double(RAND_MAX)-0.5)*2;
      }
    }*/
    
    
    /*ceres::CostFunction* cost_function = 
    LineZ3DirPinholeExtraError::Create(p.x, p.y);
    problem.AddResidualBlock(cost_function,
                            NULL,
                            &extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}),
                            &extrinsics_views({0,ray.r(i.views_min,i.views_max)}),
                            //use only direction part!
                            &lines({2,ray.r("x",i.cams_max)}));*/
    
    ///////////////////// residual /////////////////////
    
    ceres::CostFunction* cost_function = 
    LineZ3GenericMeshExtraError::Create(p.x, p.y, ws, j);
    problem.AddResidualBlock(cost_function,
                            NULL,
                            &extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}),
                            &extrinsics_views({0,ray.r(i.views_min,i.views_max)}),
                            &lines({0,ray.r("x",i.cams_max)}),
                            &lines({2,ray.r("x",i.cams_max)}),
                            &mesh(0,mesh_i.x, mesh_i.y),
                            &mesh(0,mesh_i.x+1, mesh_i.y),
                            &mesh(0,mesh_i.x, mesh_i.y+1),
                            &mesh(0,mesh_i.x+1, mesh_i.y+1)
                            );
    
    if (reproj_error_calc_only && i.sq_callback) {
      double res[2];
      
      double *params[] = {&extrinsics_cams({0,ray.r(i.cams_min,i.cams_max)}),
                          &extrinsics_views({0,ray.r(i.views_min,i.views_max)}),
                          &lines({0,ray.r("x",i.cams_max)}),
                          &lines({2,ray.r("x",i.cams_max)}),
                          &mesh(0,mesh_i.x, mesh_i.y),
                          &mesh(0,mesh_i.x+1, mesh_i.y),
                          &mesh(0,mesh_i.x, mesh_i.y+1),
                          &mesh(0,mesh_i.x+1, mesh_i.y+1)};
      
      cost_function->Evaluate(params, res, NULL);
      
      double v = res[0]*res[0] +res[1]*res[1];
      i.sq_callback(v, ray);
    }
    
    /////////// FIXME/HACK
/*
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
*/
    
    
    ///////////////////// fix parameters /////////////////////

    //ref cam extrinsics = 0
   _ref_cam_fix_extrinsics(i, problem, ray, extrinsics_cams);
   
   if (ray["y"] == center.y && ray["x"] == center.x && !reproj_error_calc_only) {
     //printf("setting pblock constant %p!\n", &lines({0,ray.r("x","cams")}));
     //cout << lines({0,ray.r("x","cams")}) << lines({1,ray.r("x","cams")}) << lines({2,ray.r("x","cams")}) << lines({3,ray.r("x","cams")}) << "\n";
     lines({0,ray.r("x",i.cams_max)}) = 0;
     lines({1,ray.r("x",i.cams_max)}) = 0;
     lines({2,ray.r("x",i.cams_max)}) = 0;
     lines({3,ray.r("x",i.cams_max)}) = 0;
     problem.SetParameterBlockConstant(&lines({0,ray.r("x",i.cams_max)}));
     problem.SetParameterBlockConstant(&lines({2,ray.r("x",i.cams_max)}));
   }
   
   //ParameterBlockLocalSize: check for already set parametrization
    if (ray["y"] == center.y && ray["x"] == center.x+1 && !reproj_error_calc_only &&
      problem.ParameterBlockLocalSize(&lines({2,ray.r("x",i.cams_max)})) == 2) {
     lines({0,ray.r("x",i.cams_max)}) = 0;
     lines({1,ray.r("x",i.cams_max)}) = 0;
     //x dir may vary
     lines({3,ray.r("x",i.cams_max)}) = 0;
     //origin fixed
     problem.SetParameterBlockConstant(&lines({0,ray.r("x",i.cams_max)}));
     problem.SetParameterization(&lines({2,ray.r("x",i.cams_max)}), new ceres::SubsetParameterization(2,{1}));
   }
   
   //ParameterBlockLocalSize: check for already set parametrization
    if (ray["y"] == center.y+1 && ray["x"] == center.x && !reproj_error_calc_only && problem.ParameterBlockLocalSize(&lines({2,ray.r("x",i.cams_max)})) == 2) {
     lines({0,ray.r("x",i.cams_max)}) = 0;
     lines({1,ray.r("x",i.cams_max)}) = 0;
     lines({2,ray.r("x",i.cams_max)}) = 0;
     //y dir may vary
     //origin fixed
     problem.SetParameterBlockConstant(&lines({0,ray.r("x",i.cams_max)}));
     problem.SetParameterization(&lines({2,ray.r("x",i.cams_max)}), new ceres::SubsetParameterization(2,{0}));
   }
        
    ray_count++;
    
    /*if (ray["y"] == center.y && ray["x"] == center.x) {
      lines({2,ray.r(1,i.cams_max)}) = 0;
      lines({3,ray.r(1,i.cams_max)}) = 0;
      problem.SetParameterBlockConstant(&lines({2,ray.r("x",i.cams_max)}));
    }*/
  }
  
  //fixate mesh at 3 points
  
  mesh_center *= 1.0/mesh_counter;
  cv::Point2i mesh_i = mesh_center;  
  
  for(int i=0;i<3;i++)
    mesh(i,mesh_i.x-2, mesh_i.y-2) = 0;
  //printf("set constant mesh point: %dx%d (%f samples)\n", mesh_i.x-2, mesh_i.y-2,mesh_coverage(mesh_i.x-2, mesh_i.y-2));
  problem.SetParameterBlockConstant(&mesh(0,mesh_i.x-2, mesh_i.y-2));
  
  for(int i=0;i<3;i++)
    mesh(i,mesh_i.x+2, mesh_i.y-2) = 0;
  //printf("set constant mesh point: %dx%d (%f samples)\n", mesh_i.x+2, mesh_i.y-2,mesh_coverage(mesh_i.x+2, mesh_i.y-2));
  problem.SetParameterBlockConstant(&mesh(0,mesh_i.x+2, mesh_i.y-2));
  
  for(int i=0;i<3;i++)
    mesh(i,mesh_i.x-2, mesh_i.y+2) = 0;
  //printf("set constant mesh point: %dx%d (%f samples)\n", mesh_i.x-2, mesh_i.y+2,mesh_coverage(mesh_i.x-2, mesh_i.y+2));
  problem.SetParameterBlockConstant(&mesh(0,mesh_i.x-2, mesh_i.y+2));
  
  
  float min_coverage = 4;
  float mesh_smoothness = 100;
  
  if (!reproj_error_calc_only)
    for(auto pos : Idx_It_Dims(mesh, 1, 2)) {
      double w = 1;
      /*if (!pos[1])
        printf("\n");
      if (mesh_coverage(pos[1],pos[2]) > 0.0)
        printf("%3.1f ", mesh_coverage(pos[1],pos[2]));*/
      if (mesh_coverage(pos[1],pos[2]) < min_coverage && mesh_coverage(pos[1],pos[2]) > 0.0 && pos[1]+1 < mesh[1] && mesh_coverage(pos[1]+1,pos[2]) > 0.0) {
        //printf("restrict %dx%d\n", pos]});
        w = mesh_smoothness/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1]+1,pos[2]));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &mesh(0,pos[1], pos[2]),
                                 &mesh(0,pos[1]+1, pos[2]));
        //printf("add mesh constr to %d x %d\n", pos[1], pos[2]);
      }
      
      if (mesh_coverage(pos[1],pos[2]) < min_coverage && mesh_coverage(pos[1],pos[2]) > 0.0 && pos[1]-1 >= 0 && mesh_coverage(pos[1]-1,pos[2]) > 0.0) {
        //printf("restrict %dx%d\n", pos]});
        w = mesh_smoothness/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1]-1,pos[2]));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &mesh(0,pos[1], pos[2]),
                                 &mesh(0,pos[1]-1, pos[2]));
        //printf("add mesh constr to %d x %d\n", pos[1], pos[2]);
      }
      
      if (mesh_coverage(pos[1],pos[2]) < min_coverage && mesh_coverage(pos[1],pos[2]) > 0.0 && pos[2]+1 < mesh[2] && mesh_coverage(pos[1],pos[2]+1) > 0.0) {
        w = mesh_smoothness/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1],pos[2]+1));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &mesh(0,pos[1], pos[2]),
                                 &mesh(0,pos[1], pos[2]+1));
        //printf("add mesh constr to %d x %d\n", pos[1], pos[2]);
      }
      
      
      if (mesh_coverage(pos[1],pos[2]) < min_coverage && mesh_coverage(pos[1],pos[2]) > 0.0 && pos[2]-1 >= 0 && mesh_coverage(pos[1],pos[2]-1) > 0.0) {
        w = mesh_smoothness/(mesh_coverage(pos[1],pos[2])+mesh_coverage(pos[1],pos[2]-1));
        ceres::CostFunction* cost_function = 
        MeshSmoothError::Create(w);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &mesh(0,pos[1], pos[2]),
                                 &mesh(0,pos[1], pos[2]-1));
        //printf("add mesh constr to %d x %d\n", pos[1], pos[2]);
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
        
        if (std::isnan(p.x) || std::isnan(p.y))
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
  
          /*if (v == 0.0 && ray["x"] == 4 && ray["y"] == 4) {
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
          }*/
          
          
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
        lines({1,ray.r("x","y"),view.r("channels","cams")}) = std::numeric_limits<double>::quiet_NaN();
        lines({2,ray.r("x","y"),view.r("channels","cams")}) = std::numeric_limits<double>::quiet_NaN();
        lines({3,ray.r("x","y"),view.r("channels","cams")}) = std::numeric_limits<double>::quiet_NaN();
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
  return 0;//max(/*sqrt(rms/count)*5*/ (double)med[count/2]*5,(double)med[count*99/100]);
}

static void _add_find_center_errors(ceres::Problem &problem, Mat_<double> &lines, double *offset)
{  
  for(auto pos : Idx_It_Dims(lines, "x", "cams"))
    if (!std::isnan(lines({0,pos.r("x","cams")}))) {
      ceres::CostFunction* cost_function = FindCenterError::Create(&lines({0,pos.r("x","cams")}));
      problem.AddResidualBlock(cost_function, NULL,
                            offset);
    }
}

#ifdef MM_MESH_WITH_VIEWER
void update_cams_mesh(Mesh &cams, Mat_<double> extrinsics_cams, Mat_<double> extrinsics_views, Mat_<double> lines, Mat_<double> proj)
{
  cams = mesh_cam().scale(20);
  
  for(auto pos : Idx_It_Dims(extrinsics_cams,1,-1)) {
      //cv::Vec3b col(0,0,0);
      //col[ch%3]=255;
      Eigen::Vector3d trans(extrinsics_cams(3,pos.r(1,-1)),extrinsics_cams(4,pos.r(1,-1)),extrinsics_cams(5,pos.r(1,-1)));
      Eigen::Vector3d rot(extrinsics_cams(0,pos.r(1,-1)),extrinsics_cams(1,pos.r(1,-1)),extrinsics_cams(2,pos.r(1,-1)));
      
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
        Eigen::Vector3d origin(lines({0,line_pos.r("x","y"),pos.r(1,-1)}),lines({1,line_pos.r("x","y"),pos.r(1,-1)}),0.0);
        Eigen::Vector3d dir(lines({2,line_pos.r("x","y"),pos.r(1,-1)}),lines({3,line_pos.r("x","y"),pos.r(1,-1)}),-1.0);
        
      
        Mesh line = mesh_line(origin,origin+dir*1000.0);
        
        line_mesh.merge(line);
      }
      
      line_mesh += trans;
      line_mesh.rotate(rot);
      
      cams.merge(line_mesh);
      //viewer.data.add_edges(P1,P2,Eigen::RowVector3d(r,g,b);
    }
  //printf("\n");
  
  for(auto pos : Idx_It_Dims(extrinsics_views, 1, -1)) {
      Eigen::Vector3d trans(extrinsics_views(3,pos.r(1,-1)),extrinsics_views(4,pos.r(1,-1)),extrinsics_views(5,pos.r(1,-1)));
      Eigen::Vector3d rot(extrinsics_views(0,pos.r(1,-1)),extrinsics_views(1,pos.r(1,-1)),extrinsics_views(2,pos.r(1,-1)));
      
      Mesh plane = mesh_plane().scale(2000);
      
      //cout << pos; printf("trans %fx%fx%f\n", trans(0), trans(1), trans(2));
      
      plane -= trans;
      plane.rotate(-rot);
      
      cams.merge(plane);
  }
  
  //printf("proj %f %f\n", proj(0, 0), proj(1,0));
}

class _mesh_updater : public ceres::IterationCallback {
public:
explicit _mesh_updater(Mesh *mesh, Mat_<double> *extr_cams, Mat_<double> *extr_views, Mat_<double> *lines, Mat_<double> *proj) :
_mesh(mesh), _extr_cams(extr_cams), _extr_views(extr_views), _lines(lines), _proj(proj)
{};

~_mesh_updater() {}

virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
  Mesh mesh_new;
  update_cams_mesh(mesh_new, *_extr_cams, *_extr_views, *_lines, *_proj);
  //trigger single viewer update
  *_mesh = mesh_new;
  
  return ceres::SOLVER_CONTINUE;
}

private:
  Mat_<double> *_extr_cams = NULL;
  Mat_<double> *_extr_views = NULL;
  Mat_<double> *_lines = NULL;
  Mat_<double> *_proj = NULL;
  Mesh *_mesh = NULL;
};
#endif


double solve_pinhole(const calib_infos &i, ceres::Solver::Options options, const Mat_<float>& proxy, Mat_<double> &lines, Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &proj, double proj_weight, double min_weight = non_center_rest_weigth)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  
  options.minimizer_progress_to_stdout = true;
  
  _zline_problem_add_pinhole_lines(i, problem, proxy, extrinsics_cams, extrinsics_views, lines, proj, NULL, true, min_weight, proj_weight);
  
  printf("solving pinhole problem (proj w = %f...\n", proj_weight);
  ceres::Solve(options, &problem, &summary);
  printf("\npinhole rms ~%fmm\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));
  printf("proj0: %f x %f\n", proj(0), proj(1));
  
  return 2.0*sqrt(summary.final_cost/problem.NumResiduals());
}

double solve_all(const calib_infos &i, ceres::Solver::Options options, const Mat_<float>& proxy, const Mat_<float> &j, Mat_<double> &lines, Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &proj, Mat_<double> &mesh, bool rms_only = false)
{
  ceres::Solver::Summary summary;
  ceres::Problem problem;
  
  _zline_problem_add_lines_gen_mesh(i, problem, proxy, extrinsics_cams, extrinsics_views, lines, mesh, rms_only, j);
  
  if (rms_only) {
    options.max_num_iterations = 0;
  }
  else {
    options.minimizer_progress_to_stdout = true;
    printf("solving full problem\n");
  }
  ceres::Solve(options, &problem, &summary);
  printf("\ncalib rms ~%fpx\n", 2.0*sqrt(summary.final_cost/problem.NumResiduals()));

  
  printf("center rays: ");
  cout << cvMat(lines.bindAll(-1, lines[1]/2,lines[2]/2)) << "\n";
  
  cout << cvMat(lines.bindAll(-1, lines[1]/2+1,lines[2]/2)) << "\n";
  cout << cvMat(lines.bindAll(-1, lines[1]/2,lines[2]/2+1)) << "\n";
  cout << cvMat(lines.bindAll(-1, lines[1]/2+1,lines[2]/2+1)) << "\n";
  
  cout << cvMat(lines.bindAll(-1, lines[1]/2-1,lines[2]/2)) << "\n";
  cout << cvMat(lines.bindAll(-1, lines[1]/2,lines[2]/2-1)) << "\n";
  cout << cvMat(lines.bindAll(-1, lines[1]/2-1,lines[2]/2-1)) << "\n";
  
  
  //cout << cvMat(lines.bind(0, 0)) << "\n";
  
  
  
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

static void echo_sq(double sq)
{
  printf("%f ", sq);
}

struct _filter_data_worst {
  double worst = -1;
  uint8_t *idx = NULL;
  double sum;
  int count;
};

static void sq_get_worst_entry(double sq, const Idx &ray, _filter_data_worst *data, Mat_<uint8_t> &filter)
{
  data->count++;
  data->sum += sq;
  
  if (sq > data->worst) {
    data->worst = sq;
    data->idx = &filter(ray.r(1,-1));
  }
}

namespace ucalib {

/*
 * optimize together:
 *  - per image camera movement (world rotation and translation)
 *  - individual lines (which do not need to converge in an optical center
 */
double fit_cams_lines_multi(Mat_<float> &proxy, int first_view_dim, cv::Point2i img_size, ucalib::RayCalib &c, bool vis, const Mat_<float>& j, const ucalib::Options &opts)
{
  calib_infos i;
  
  i.img_size = img_size;
  i.cams_min = 3;
  //FIXME REWORK do this with dimspec
  i.cams_max = first_view_dim-1;
  i.views_min = first_view_dim;
  i.views_max = proxy.size()-1;
  
  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  //options.function_tolerance = 1e-10;
  options.parameter_tolerance = 1e-20;
  options.minimizer_progress_to_stdout = false;
  //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  
  //options.minimizer_progress_to_stdout = true;
  
  c._rays.create({IR(4, "line"), proxy.r(1, i.cams_max)});
  c._cams.create({IR(6, "extrinsics"), proxy.r(i.cams_min, i.cams_max)});
  c._views.create({IR(6, "extrinsics"), proxy.r(i.views_min,i.views_max)});
  
  printf("calibrate with %d views of %d cams\n", c._views.total()/6, c._cams.total()/6);
  
  //for threading!
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE)) {
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  }
  
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  
  c._proj.create({2,proxy.r(i.cams_min,i.cams_max)});
  
  for(auto pos : Idx_It_Dims(c._proj, 0, -1))
    c._proj(pos) = 1000;
  
  for(auto cam_pos : Idx_It_Dims(c._views, 1, -1)) {
    for(int i=0;i<5;i++)
      c._views({i, cam_pos.r(1,-1)}) = 0;
    c._views({5, cam_pos.r(1,-1)}) = 1000;
    //extrinsics({2, cam_pos["views"]}) = M_PI*0.5;
  }

  for(auto pos : Idx_It_Dims(c._cams, 0, -1))
    c._cams(pos) = 0;
  
  int cl_x = c._rays[1]/2;
  int cl_y = c._rays[2]/2;
  for(auto line_pos : Idx_It_Dims(c._rays, 1, -1)) {
    c._rays({0, line_pos.r(1,-1)}) = 0;
    c._rays({1, line_pos.r(1,-1)}) = 0;
    c._rays({2, line_pos.r(1,-1)}) = (line_pos[1]-cl_x)*0.01;
    c._rays({3, line_pos.r(1,-1)}) = (line_pos[2]-cl_y)*0.01;
  }
  
  /*if (proxy["views"] > 1) {
    printf("%f x %f \n", proxy(0, 0, 0, 0, 0, 0),proxy(1, 0, 0, 0, 0, 0));
    printf("%f x %f \n", proxy(0, 0, 0, 0, 0, 1),proxy(1, 0, 0, 0, 0, 1));
  }*/
  
  //mesh display
#ifdef MM_MESH_WITH_VIEWER 
  Mesh *mesh = new Mesh();
  _mesh_updater callback(mesh, &c._cams, &c._views, &c._rays, &c._proj);
    
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;
  
  update_cams_mesh(*mesh, c._cams, c._views, c._rays, c._proj);
    
  if (opts.flags() & ucalib::LIVE)
    mesh->show(false);
  
#endif

  options.max_num_iterations = 5000;
  
  solve_pinhole(i, options, proxy, c._rays, c._cams, c._views, c._proj, strong_proj_constr_weight, non_center_rest_weigth);
  solve_pinhole(i, options, proxy, c._rays, c._cams, c._views, c._proj, 0, 0);
  
  
  c._mesh.create({3, 20, 20});
  //FIXME automaticall calc usefule size or get from options?!
  //FIXME target dimension * marker_size!
  cv::Point2i target_size(1200,1200);
  i.target_size = target_size;
  
  cvMat(c._mesh).setTo(0);
  for(auto pos : Idx_It_Dims(c._mesh, 1, 2)) {
    c._mesh({2,pos.r(1,2)}) = target_size.x/10;
  }
  c._mesh(2,0,0) = 0;
  c._mesh(2,1,0) = 0;
  c._mesh(2,2,0) = 0;
  c._mesh(2,0,1) = 0;
  
  //solve_pinhole(i, options, proxy, lines, c._cams, c._views, proj, 0, 0);
  
  solve_all(i, options, proxy, j, c._rays, c._cams, c._views, c._proj, c._mesh);
  
  //i.sq_callback = &echo_sq;
  //solve_all(i, options, proxy, j, c._rays, c._cams, c._views, c._proj, c._mesh,  true);
  
  //filter outliers
  /*_filter_data_worst filter_data;
  i.proxy_mask.create(Idx(proxy.r(1,-1)));
  cvMat(i.proxy_mask).setTo(1);
  i.sq_callback = std::bind(&sq_get_worst_entry, std::placeholders::_1, std::placeholders::_2, &filter_data, i.proxy_mask);
  
  for (int f=0;f<100;f++) {
    filter_data.worst = -1;
    filter_data.sum = 0;
    filter_data.count = 0;
    solve_all(i, options, proxy, j, c._rays, c._cams, c._views, c._proj, c._mesh,  true);
    printf("del %f avg %f\n", sqrt(filter_data.worst), sqrt(filter_data.sum / filter_data.count));
    *filter_data.idx = 0;
    solve_all(i, options, proxy, j, c._rays, c._cams, c._views, c._proj, c._mesh, false);
  }*/
  
#ifdef MM_MESH_WITH_VIEWER
  delete mesh;
#endif
  
  /*for(auto line_pos : Idx_It_Dims(c._rays, 1, -1)) {
    if (c._rays({0, line_pos.r(1,-1)}) == 0 &&
        c._rays({1, line_pos.r(1,-1)}) == 0 &&
        //FIXME TODO detection!
        c._rays({2, line_pos.r(1,-1)}) == (line_pos[1]-cl_x)*0.01 &&
        c._rays({3, line_pos.r(1,-1)}) == (line_pos[2]-cl_y)*0.01) {
          c._rays({0, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({1, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({2, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({3, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
        }
  }*/
  
  /*for(auto line_pos : Idx_It_Dims(c._rays, 1, -1)) {
    if (c._rays({0, line_pos.r(1,-1)}) == 0 &&
        c._rays({1, line_pos.r(1,-1)}) == 0 &&
        c._rays({2, line_pos.r(1,-1)}) == (line_pos[1]-cl_x)*0.01 &&
        c._rays({3, line_pos.r(1,-1)}) == (line_pos[2]-cl_y)*0.01) {
          c._rays({0, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({1, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({2, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
          c._rays({3, line_pos.r(1,-1)}) = std::numeric_limits<double>::quiet_NaN();
        }
  }*/
  
#ifdef UCALIB_WITH_MM_MESH
//TODO FIXME
  Mesh line_mesh;
      
  for(auto line_pos : Idx_It_Dims(c._rays,"x","y")) {
    Eigen::Vector3d origin(c._rays({0,line_pos.r("x","y"),0,0}),c._rays({1,line_pos.r("x","y"),0,0}),0.0);
    Eigen::Vector3d dir(c._rays({2,line_pos.r("x","y"),0,0}),c._rays({3,line_pos.r("x","y"),0,0}),-1.0);
    
    if (!std::isnan(origin(0))) {
      dir *= 5.0;
      //dir(2) *= 0.1;
      Mesh line = mesh_line(origin-dir,origin+dir);
      //cout << origin << dir << "\n";
      line_mesh.merge(line);
    }
  }
  if (vis)
    line_mesh.show(true);
  
  line_mesh.writeOBJ("center.obj");
#endif
  
  return 0.0;
  
  
  /*printf("pinhole rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, true, j);
  
  solve_non_central(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views);
  printf("non-central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, true, j);
  
printf("\nsolving central camera with deformation ------------------------------------\n\n");
  
  int solve_flags = 0;

  Mat_<float> proxy_cp;
  Mat_<double> lines_cp;
  
  proxy_cp.create(proxy);
  lines_cp.create(lines);
  
  proxy.copyTo(proxy_cp);
  lines.copyTo(lines_cp);
  

  //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, false, j, solve_flags);
  printf("deformation rms:\n");
  
  double res_avg;
  int filtered = 1;
  
  while (filtered) {
    res_avg = calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j);
    calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j, res_avg, &filtered);
    if (filtered) {
      //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
      solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, false, j, solve_flags);
    }
  }
  
  printf("final central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, true, j, solve_flags);
  
  //////////7
//   Mat_<std::vector<cv::Point2f>> res_m_central({proxy.r("x","y")});
//   calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j, 0, NULL, &res_m_central);
//   
//   Mat_<std::vector<float>> res_l_c({proxy.r("x","y")});
//   for(auto pos : Idx_It_Dims(res_m, 0, 1)) {
//     
//     int x = pos["x"];
//     int y = pos["y"];
//     std::vector<cv::Point2f> &ps = res_m(pos);
//     
// 
//     
//     float avg = 0;
//     for(auto d : ps)
//       avg += norm(d);
//     res_l_c(x,y) = sqrt(avg/ps.size());
//   }
  //save_mat("err_residuals_central.clif", res_l_c);
  
  
  /////////////7
  
  
  
  printf("\nsolving non-central camera with deformation ------------------------------------\n\n");
  
  solve_flags = NON_CENTRAL;
  
  proxy_cp.copyTo(proxy);
  lines_cp.copyTo(lines);
  

  //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, false, j, solve_flags);
  printf("deformation rms:\n");
  
  filtered = 1;
  
  while (filtered) {
    res_avg = calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j);
    calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j, res_avg, &filtered);
    if (filtered) {
      //cvMat(target_mesh).setTo(double(target_size.x)/target_mesh[1]);
      solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, false, j, solve_flags);
    }
  }
  
  printf("final central rms:\n");
  solve_non_central_mesh(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, true, j, solve_flags);
  
  Mat_<std::vector<cv::Point2f>> res_m({proxy.r("x","y")});
  calc_line_residuals(proxy, extrinsics_cams, extrinsics_views, proj, lines, img_size, target_size, target_mesh, j, 0, NULL, &res_m);
  cv::Mat debug_dirs(img_size, CV_8UC3, cv::Scalar(0));
  
  Mat_<float> res_l({proxy.r("x","y")});
  for(auto pos : Idx_It_Dims(res_m, 0, 1)) {
    cv::Point2i fdim(lines[1],lines[2]);
    
    int x = pos["x"];
    int y = pos["y"];
    cv::Point2f ip = cv::Point2f((x+0.5)/fdim.x*img_size.x, (y+0.5)/fdim.y*img_size.y);
    std::vector<cv::Point2f> &ps = res_m(pos);
    
//     if (ps.size()) {
//       for(auto d : ps) {
//         printf("draw: %d %d : %d %fx%f %fx%f\n",x,y,ps.size(),ip.x, ip.y,100*d.x, 100*d.y);
//         if (d.x != 0.0)
//           break;
//       }
//     }
    
    float avg = 0;
    for(auto d : ps) {
      cv::line(debug_dirs, ip, ip+100*d, CV_RGB(255,255,255), 1, CV_AA);
      avg += d.x*d.x+d.y*d.y;
    }
//     res_l(x,y) = sqrt(avg/ps.size());
//     if (x == 0)
//       printf("\n");
//     printf("%0.3f ", res_l(x,y));
  }
  printf("\n");
  //save_mat("err_residuals.clif", res_l);
  
  
  imwrite("debug_reproj_dirs.tif", debug_dirs);
  
  
  correct_cam_center(options, proxy, lines, img_size, extrinsics_cams, extrinsics_views, proj, target_size, target_mesh, true, j);

  printf("proj last: %f %f\n", proj(0), proj(1));
  
  Mesh target_vis = mesh_subdiv_plane(target_mesh[1],target_mesh[2]);
  
  target_vis.scale(double(target_size.x)/target_mesh[1]);
  
  for(int j=0;j<target_mesh[1];j++) {
    for(int i=0;i<target_mesh[2];i++) {
      if (target_mesh(0,i,j) != 0.0 || target_mesh(1,i,j) != 0.0 ||  target_mesh(2,i,j) != target_size.x/10)
        printf("[%2.4f %2.4f %2.4f]", target_mesh(0,i,j), target_mesh(1,i,j), target_mesh(2,i,j));
      target_vis.V(j*target_mesh[1]+i, 0) += target_mesh(0,i,j);
      target_vis.V(j*target_mesh[1]+i, 1) += target_mesh(1,i,j);
      target_vis.V(j*target_mesh[1]+i, 2) += target_mesh(2,i,j);
    }
  }
  delete mesh;
  target_vis.writeOBJ("target.obj");
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
      dir *= 5.0;
      //dir(2) *= 0.1;
      Mesh line = mesh_line(origin-dir,origin+dir);
      //cout << origin << dir << "\n";
      line_mesh.merge(line);
    }
  }
  if (vis)
    line_mesh.show(true);
  
  line_mesh.writeOBJ("center.obj");*/
  
  
  //_zline_problem_eval_pinhole_lines(problem, proxy, extrinsics, extrinsics_rel, lines);

#ifdef MM_MESH_WITH_VIEWER
  //glfwSetWindowShouldClose(_viewer->window, 1);
  //FIXME add blocking exit method to mesh
  //viewer_thread.join();
#endif
  printf("finished\n");

  return 0.0;
}

}

namespace ucalib {
  
  //TODO hide?
  class RayCam : public Cam {
  public:
    virtual void projectPoints(const std::vector<cv::Point3f> &wpoints, std::vector<cv::Point2f> &ipoints);
    virtual cv::Vec3d r() const;
    virtual cv::Vec3d t() const;
    virtual cv::Point2d f() const;
    virtual void rectify_to_dir(cv::Vec3d dir);
    const Mat_<double> &rays();
    friend class RayCalib;
  private:
    const RayCalib *_calib;
    cv::Vec3d _r, _t;
    cv::Point2d _f;
    Mat_<double> _rays;
  };
  
  Cam *RayCalib::cam(const Idx &cam) const
  {
    Idx view;
    Idx cam_only;
    
    //FIXME use dim_view_start_idx!
    if (cam.size() == _cams.size()) {
      view = {cam.r(_cams.size()-1, -1)};
      cam_only = {cam.r(0, _cams.size()-2)};
      
      return RayCalib::cam(cam_only, view);
    }
    //FIXME use dim_view_start_idx!
    return RayCalib::cam(Idx(_cams.size()-1), Idx(1));
    
  }
  
  Cam *RayCalib::cam(const Idx &cam, const Idx &view) const
  {
    RayCam *c = new RayCam();
    c->_calib = this;
    
    cv::Vec3d r_v(_views({0,view.r(0,-1)}),
                  _views({1,view.r(0,-1)}),
                  _views({2,view.r(0,-1)}));
    cv::Vec3d t_v(_views({3,view.r(0,-1)}),
                  _views({4,view.r(0,-1)}),
                  _views({5,view.r(0,-1)}));
    cv::Matx33d r_v_m;
    cv::Rodrigues(r_v, r_v_m);
      
    cv::Vec3d r_c(_cams({0,cam.r(0,-1)}),
                  _cams({1,cam.r(0,-1)}),
                  _cams({2,cam.r(0,-1)}));
    cv::Vec3d t_c(_cams({3,cam.r(0,-1)}),
                  _cams({4,cam.r(0,-1)}),
                  _cams({5,cam.r(0,-1)}));
    cv::Matx33d r_c_m;
    cv::Rodrigues(r_c, r_c_m);
    
    //calc rotation
    cv::Matx33d r_m = r_c_m*r_v_m;
    cv::Rodrigues(r_m, c->_r);
    
    //calc translation
    c->_t = r_c_m*t_v + t_c;
    
    //retrieve f
    c->_f = cv::Point2d(_proj({0,cam.r(0,-1)}),
                       _proj({0,cam.r(0,-1)}));
    
    c->_rays = _rays;
    for(int dim = _rays.size()-1;dim>=3;dim--)
      c->_rays = c->_rays.bind(dim, cam[dim-3]);
    
    return c;
  }
  
  const Mat_<double> &RayCalib::extrinsics_cams() const
  {
    return _cams;
  }
  
  const Mat_<double> &RayCalib::extrinsics_views() const
  {
    return _views;
  }
  
  const Mat_<double> &RayCalib::proj()
  {
    return _proj;
  }
  
  const Mat_<double> &RayCalib::rays()
  {
    return _rays;
  }
  
  const Mat_<double> &RayCam::rays()
  {
    return _rays;
  }
  
  const Cam *Calib::ref_cam() const
  {
    return _ref_cam;
  }
  void Calib::set_ref_cam(const Cam *cam)
  {
    _ref_cam = cam;
  }
  
  /*cv::Size RayCalib::img_size() const
  {
    return _img_size;
  }*/
  
  cv::Point2d RayCam::f() const
  {
    return _f;
  }
  
  void RayCam::projectPoints(const std::vector<cv::Point3f> &wpoints, std::vector<cv::Point2f> &ipoints)
  {}
  
  cv::Vec3d RayCam::r() const
  {
    return _r;    
  }
  cv::Vec3d RayCam::t() const
  {
    return _t;   
  }

  Calib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const Options &opts, const DimSpec &views_dims_start)
  {
    return calibrate_rays(proxy, Mat_<double>(), img_size, opts, views_dims_start);
  }
  
  Calib* calibrate_rays(Mat_<float> &proxy, const Mat_<float> &j, cv::Point2i img_size, const Options &opts, const DimSpec &views_dims_start)
  {
    RayCalib *c = new RayCalib();
    
    int v_start = views_dims_start.get(proxy);
    
    //FIXME check content first?
    proxy.names({"","x","y"});
    fit_cams_lines_multi(proxy, v_start, img_size, *c, true, j, opts);
    
#ifdef MM_MESH_WITH_VIEWER
  if (opts.flags() & ucalib::SHOW_TARGET) {
    Mesh target_vis = c->target_mesh();
    target_vis.show(true);
  }
#endif
    
    return c;
  }
  
  void RayCam::rectify_to_dir(cv::Vec3d dir)
  {    
    //TODO derive direction from layout of all cams? least squares fit of a line through cam positions
    dir *= 1.0/norm(dir);
  
    //FIXME better last cam selection!?!
    Idx lastview(_calib->extrinsics_views().r(1,-2));
    for(auto &i : lastview)
      i--;
    //cout << "lastview: " << lastview << "\n";
    //FIXME free cam / add refcounting within RayCalib?

    //cout << "ref cams:" << _calib->cam({0},lastview)->t() << _calib->cam({0})->t() << "\n";
    cv::Vec3d trans = _calib->cam({0},lastview)->t() - _calib->cam({0})->t();
    //cout << "dir: "<< trans << "\n";
    trans *= 1.0/norm(trans);
    //cout << "dir: "<< trans << "\n";
    
    cv::Vec3d rot = trans.cross(dir);
    
    printf("scalarprodukt: %f\n", trans.dot(dir));
    double angle = acos(trans.dot(dir));
    rot *= -angle;
    
    printf("rectification angle: %f\n", angle);
    
    cv::Matx33d r_orig, r_new, r_add;
    cv::Rodrigues(_r, r_orig);
    cv::Rodrigues(rot,r_add);
    r_new = r_add*r_orig;
    cv::Rodrigues(r_new,_r);
    
    cv::Rodrigues(rot,r_add);
    _t = r_add*_t;
    
    //FIXME also need to modify t (to keep position within new camera coordiante system)
  }
  
  RayCalib::RayCalib(Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &proj, Mat_<double> &rays, cv::Point2i img_size)
  {
    _cams = extrinsics_cams;
    _views = extrinsics_views;
    _proj = proj;
    _rays = rays;
    //_img_size = img_size;
  }
  
  const char *ucalib_str = "UCALIB";
  
  RayCalib::RayCalib(std::function<Mat(cpath)> load_mat, std::function<const char *(cpath)> load_string)
  {
    const char *type = load_string("type");
    
    assert(!strcmp(ucalib_str, type));
    _rays = load_mat("lines");
    _views = load_mat("extrinsics/views");
    _cams = load_mat("extrinsics/cams");
    _proj = load_mat("projection");
    cout << "loaded calib" << _rays <<  _views <<  _cams << _proj << "\n";
  }
  
  
  void RayCalib::save(std::function<void(cpath,Mat)> save_mat, std::function<void(cpath,const char *)> save_string)
  {
    save_string("type", ucalib_str);
    save_mat("lines", rays());
    save_mat("extrinsics/views", extrinsics_views());
    save_mat("extrinsics/cams", extrinsics_cams());
    save_mat("projection", proj());
  }
  
  Calib* load(std::function<Mat(cpath)> load_mat, std::function<const char *(cpath)> load_string)
  {
    const char *type = load_string("type");
    
    if (!strcmp("UCALIB", type))
      return new RayCalib(load_mat, load_string);
    
    return NULL;
  }
  
  //FIXME correctly handle view idx as position in either/or/both cams,views
  void RayCalib::rectify(const cv::Mat &src, cv::Mat &dst, const Idx &view_idx, double z) const
  {    
    printf("do rectification!"); cout << view_idx << "\n";
    //printf("ref:"); cout << ref->_cam << " x "<< ref->_view <<  "\n";
    
    cv::Size size = src.size();
    
    
    Mat_<double> rect_rays({4,_rays.r(1,2)});
    cv::Mat map;
    
    RayCam *view_cam = (RayCam*)cam(view_idx);
    //printf("view:"); cout << view_cam->_cam << " x " << view_cam->_view <<  "\n";
    
    cv::Matx31d r = -view_cam->r();
    cv::Matx31d t = -view_cam->t();
    
    //this cam rotation
    //FIXME combine cam and view extrinsics!?!
    /*cv::Matx31d r(-_views({0,view_idx.r(1,-1)}),
                  -_views({1,view_idx.r(1,-1)}),
                  -_views({2,view_idx.r(1,-1)}));
    cv::Matx31d t(-_views({3,view_idx.r(1,-1)}),
                  -_views({4,view_idx.r(1,-1)}),
                  -_views({5,view_idx.r(1,-1)}));*/
    
    cv::Matx33d r_m;
    cv::Rodrigues(r, r_m);
    
    cv::Matx31d ref_r = _ref_cam->r();
    cv::Matx31d ref_t = _ref_cam->t();
    cv::Matx33d ref_r_m;
    cv::Rodrigues(ref_r, ref_r_m);
    
    for(auto pos : Idx_It_Dims(_rays, 1, 2)) {
      //STEP 1 move lines into reference cam reference frame
      
      cv::Matx31d offset(view_cam->rays()(0,pos.r(1,2)),
                         view_cam->rays()(1,pos.r(1,2)),
                         0.0);
      cv::Matx31d dir(view_cam->rays()(2,pos.r(1,2)),
                         view_cam->rays()(3,pos.r(1,2)),
                         1.0);
      
      if (pos[1] == _rays[1]/2 && pos[2] == _rays[2]/2) {
        printf("center ray (shold be zero!)\n");
        cout << pos << dir << "\n"; 
      }
    
      //revert cam translation to calibration origin (first cam) 
      offset += t;
      //invert cam rotation and apply center cam rotation
      offset = ref_r_m * r_m * offset;
      
      //apply reference cam translation
      offset += ref_t;
      
      //direction is only rotated
      dir = ref_r_m*r_m*dir;
      
      //normalize
      dir *= 1.0/dir(2);
      offset -= offset(2)*dir;
      
      rect_rays({0, pos.r(1,2)}) = offset(0);
      rect_rays({1, pos.r(1,2)}) = offset(1);
      
      rect_rays({2, pos.r(1,2)}) = dir(0);
      rect_rays({3, pos.r(1,2)}) = dir(1);
    }
    
    cout << "view: "<< r << t << "\ncam:" << ref_r << ref_t << ref_r_m*r_m <<  t+ref_t << "\n";
    
    //STEP 2 calculate undist map for cam at depth
    map.create(size, CV_32FC2);
    printf("load depth %f\n", z);
    get_undist_map_for_depth(rect_rays, map, z, size, _ref_cam->f());
    
    //STEP 3 warp
    remap(src, dst, map, cv::noArray(), cv::INTER_LINEAR);
  }
  
#ifdef UCALIB_WITH_MM_MESH
  Mesh RayCalib::target_mesh() const
  {
    Mesh target_vis = mesh_subdiv_plane(_mesh[1],_mesh[2]);
    
    //FIXME
    //target_vis.scale(double(target_size.x)/_mesh[1]);
    
    for(int j=0;j<_mesh[1];j++) {
      for(int i=0;i<_mesh[2];i++) {
        //if (_mesh(0,i,j) != 0.0 || _mesh(1,i,j) != 0.0 ||  _mesh(2,i,j) != target_size.x/10)
          //printf("[%2.4f %2.4f %2.4f]", _mesh(0,i,j), _mesh(1,i,j), _mesh(2,i,j));
        target_vis.V(j*_mesh[1]+i, 0) += _mesh(0,i,j);
        target_vis.V(j*_mesh[1]+i, 1) += _mesh(1,i,j);
        target_vis.V(j*_mesh[1]+i, 2) += _mesh(2,i,j);
      }
    }
    
    return target_vis;
  }
#endif
}
