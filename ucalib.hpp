#ifndef _UCALIB_UCALIB_H
#define _UCALIB_UCALIB_H

#include <metamat/mat.hpp>

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(MetaMat::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);

//needs!
//proxy.names({"point","x","y","channels","cams","views"});
//lines.names({"line","x","y","channels","cams"})
double fit_cams_lines_multi(MetaMat::Mat_<float>& proxy, cv::Point2i img_size, MetaMat::Mat_<double> &lines, MetaMat::Mat_<double> &extrinsics, MetaMat::Mat_<double> &extrinsics_rel, MetaMat::Mat_<double> &proj, bool vis = false, const MetaMat::Mat_<float>& scales = MetaMat::Mat_<float>());

namespace ucalib {
  
void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, MetaMat::Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints);

}


namespace ucalib {
  
  using namespace MetaMat;
  
  enum Features
  {
    None = 0,
    Depth_Required = 1,
    Rectification = 2
  };

  class Calib;
  
  class Cam {
  public:
    virtual void projectPoints(const std::vector<cv::Point3f> &wpoints, std::vector<cv::Point2f> &ipoints) = 0;
    virtual cv::Vec3d r() const = 0;
    virtual cv::Vec3d t() const = 0;
    virtual cv::Point2d f() const = 0;
  };
  
  class Calib {
  public:
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam = Idx()) = 0;
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam, const Idx &view) = 0;
    virtual const Mat_<double> &extrinsics_cams() = 0;
    virtual const Mat_<double> &extrinsics_views() = 0;
    virtual const Mat_<double> &proj() = 0;
    virtual cv::Size img_size() const = 0;
    virtual int features() const { return 0; };
    
    virtual void rectify(const Mat &src, Mat &&dst, const Idx &view_idx, double z) const = 0;
    
    const Cam *ref_cam() const;
    void set_ref_cam(const Cam *cam);
  protected:
    const Cam * _ref_cam = NULL;
  };
  
  class RayCalib : public Calib {
  public:
    RayCalib(Mat_<double> &extrinsics_cams, Mat_<double> &extrinsics_views, Mat_<double> &proj, Mat_<double> &rays, cv::Point2i img_size);
    RayCalib() {};
    //empty idx specifies first cam or view respectively
    virtual Cam *cam(const Idx &cam);
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam, const Idx &view);
    virtual const Mat_<double> &extrinsics_cams();
    virtual const Mat_<double> &extrinsics_views();
    virtual const Mat_<double> &proj();
    virtual const Mat_<double> &rays();
    virtual cv::Size img_size() const;
    virtual int features() const { return 0;/*Depth_Required | Rectification;*/ };
    
    virtual void rectify(const Mat &src, Mat &&dst, const Idx &view_idx, double z) const;
    
    friend RayCalib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const DimSpec &views_dims_start);
    friend RayCalib* calibrate_rays(Mat_<float> &proxy, Mat_<float> &j, cv::Point2i img_size, const DimSpec &views_dims_start);
    friend class RayCam;
  private:
    cv::Point2i _img_size;
    Mat_<double> _rays; //same size as cam_extrinsics
    Mat_<double> _cams; //same size as cam_extrinsics
    Mat_<double> _views; //same size as cam_extrinsics
    Mat_<double> _proj; //same size as cam_extrinsics
  };

  /*
  * 
  calibration input: proxy with: (xy,viewpoints,channels,cams,...,views)
  */

  RayCalib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const DimSpec &views_dims_start = DimSpec(-1));
  RayCalib* calibrate_rays(Mat_<float> &proxy, Mat_<float> &j, cv::Point2i img_size, const DimSpec &views_dims_start);

}

#endif
