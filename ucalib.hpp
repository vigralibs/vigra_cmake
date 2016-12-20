#ifndef _UCALIB_UCALIB_H
#define _UCALIB_UCALIB_H

#include <metamat/mat.hpp>

#ifdef UCALIB_WITH_MM_MESH
  #include <mm-mesh/mesh.hpp>
#endif

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(MetaMat::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);

//needs!
//proxy.names({"point","x","y","channels","cams","views"});
//lines.names({"line","x","y","channels","cams"})
//double fit_cams_lines_multi(MetaMat::Mat_<float>& proxy, cv::Point2i img_size, MetaMat::Mat_<double> &lines, MetaMat::Mat_<double> &extrinsics, MetaMat::Mat_<double> &extrinsics_rel, MetaMat::Mat_<double> &proj, bool vis = false, const MetaMat::Mat_<float>& scales = MetaMat::Mat_<float>());

namespace ucalib {
  
void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, MetaMat::Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints);

}


namespace ucalib {
  
  using namespace MetaMat;
  
  enum Flags {
    CENTRAL = 1, 
    PLANAR = 2,
    LIVE = 4, //live opengl feedback
    SHOW_TARGET = 8, //live opengl feedback
    SHOW_CENTER = 16, //live opengl feedback
    MAX = 32
  };
  
  enum Features
  {
    None = 0,
    Depth_Required = 1,
    Rectification = 2
  };
  
  class Options {
  public:
    Options(int flags) : _flags(flags) {}
    int flags() const { return _flags; }
  private:
    int _flags;
  };

  class Calib;
  
  class Cam {
  public:
    virtual void projectPoints(const std::vector<cv::Point3f> &wpoints, std::vector<cv::Point2f> &ipoints) = 0;
    virtual cv::Vec3d r() const = 0;
    virtual cv::Vec3d t() const = 0;
    virtual cv::Point2d f() const = 0;
    virtual void rectify_to_dir(cv::Vec3d dir) = 0;
  };
  
  class Calib {
  public:
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam = Idx()) const = 0;
    //empty idx size is extrinsics+views size!
    virtual Cam *cam(const Idx &cam, const Idx &view) const = 0;
    virtual const Mat_<double> &extrinsics_cams() const = 0;
    virtual const Mat_<double> &extrinsics_views() const = 0;
    virtual const Mat_<double> &proj() = 0;
    //virtual cv::Size img_size() const = 0;
    virtual int features() const { return 0; };
    virtual void save(std::function<void(cpath,Mat)> save_mat, std::function<void(cpath,const char *)> save_string) = 0;
#ifdef UCALIB_WITH_MM_MESH
    virtual Mesh target_mesh() const = 0;
#endif
    
    virtual void rectify(const Mat &src, Mat &&dst, const Idx &view_idx, double z) const = 0;
    
    const Cam *ref_cam() const;
    void set_ref_cam(const Cam *cam);
  protected:
    const Cam * _ref_cam = NULL;
  };
  
  Cam *rectified_view_cam(Calib *calib, const Idx &view);

  /*
  * 
  calibration input: proxy with: (xy,viewpoints,channels,cams,...,views)
  */

  Calib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const DimSpec &views_dims_start = DimSpec(-1), const Options &opts = Options(0));
  Calib* calibrate_rays(Mat_<float> &proxy, const Mat_<float> &j, cv::Point2i img_size, const DimSpec &views_dims_start, const Options &opts = Options(0));
  Calib* load(std::function<Mat(cpath)> load_mat, std::function<const char *(cpath)> load_string);

}

#endif
