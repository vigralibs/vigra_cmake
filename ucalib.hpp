#ifndef _UCALIB_UCALIB_H
#define _UCALIB_UCALIB_H

#include <metamat/mat.hpp>

#ifdef UCALIB_WITH_MM_MESH
  #include <mm-mesh/mesh.hpp>
#endif

//cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
//void get_undist_map_for_depth(MetaMat::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);

//needs!
//proxy.names({"point","x","y","channels","cams","views"});
//lines.names({"line","x","y","channels","cams"})
//double fit_cams_lines_multi(MetaMat::Mat_<float>& proxy, cv::Point2i img_size, MetaMat::Mat_<double> &lines, MetaMat::Mat_<double> &extrinsics, MetaMat::Mat_<double> &extrinsics_rel, MetaMat::Mat_<double> &proj, bool vis = false, const MetaMat::Mat_<float>& scales = MetaMat::Mat_<float>());

namespace ucalib {
  
  void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, MetaMat::Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints);
  
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
  
  /** Abstract base representing a single calibration problem.
  * This is a virtual base clase, to provide a common api for different calibration methods.
  * This class stores intrinsics and extrinsic calibration for a set of cameras.
  * Calibration is handled for an arbitrary number of cameras, with a static orientation relative to each other, 
  * the relative pose of the cameras are referenced as camera extrinsics below, and are always given relative to the first camera (extrinsics idx = 0).
  * 
  * In addition all cameras may be moved together, either for calibration purposes (when the target is moved) or because camera movement is required part of the respective camera setup, for example when a camera is moved on a translation stage. These movements are called view extrinsics below, and are given as absolute extrinsics for the respective views, as a transform from target to camera coordinates (of the first camera). Note that for calibration it is almost always required to have multiple views, as the target needs to be observed from several angles and distances.
  * Both the set of cameras and the set of views are handled as n-dimensional matrices, which can be used arbitrarily as seems logical. For example for a setup with two linear camera arrays with N cameras each, it is possible to either address the cameras as a single large 1D array with size 2N or as a 2xN (or Nx2) matrix. The same is true for the calibration views.
  *
  * Calibrations can be created with the respective function like calibrate_rays(), or deserialized with load().
  */
  class Calib {
  public:
    /** retrieve a camera object representing the camera and view given by \a cam. \a cam represents the view and cam index by concatenating them. if cam.size() is smaller that views+cams .size() missing values are interpreted as 0. */
    virtual Cam *cam(const Idx &cam = Idx()) const = 0;
    /** retrieve a camera object representing the camera and view given by \a cam and \a view. This variant addresses cam and view idx separately. */
    virtual Cam *cam(const Idx &cam, const Idx &view) const = 0;
    /** returns a matrix which gives all camera extrinsics. The first dimensions are 3x2 where (*,0) is the Rodriguez rotation vector and (*,1) the translation, from the first camera into the respective cam.  */
    virtual const Mat_<double> &extrinsics_cams() const = 0;
    /** returns a matrix which gives all view extrinsics. The first dimensions are 3x2 where (*,0) is the Rodriguez rotation vector and (*,1) the translation, from the target coordinate system into the respective view. */
    virtual const Mat_<double> &extrinsics_views() const = 0;
    /** get the focal length in pixels, matrix has size (2,cams...). */
    virtual const Mat_<double> &proj() = 0;
    /** get a bitflag of supported features of the calibration (e.g. rectification, mesh deformation, etc... )*/
    virtual int features() const { return 0; };
    /** save the calibration using the supplied \a save_mat and \a save_string functions, directly usable functions are supplied in the ucalib_clif.hpp header. Else std::function objects are often conveniently constructed using std::bind to integrate additional arguments.*/
    virtual void save(std::function<void(cpath,Mat)> save_mat, std::function<void(cpath,const char *)> save_string) = 0;
#ifdef UCALIB_WITH_MM_MESH
    /** retrieve the calibrated mesh, if supported (check features()). */
    virtual Mesh target_mesh() const = 0;
#endif
    
    /** rectify the given image as if it were the cam and view given by view, where \a view_idx is (cams...,views...) to a reference depth \a z. The reference camera to which rectification occours is given by \a set_ref_cam()/\a ref_cam()
     * \param src the distorted source image
     * \param dst destination where the rectified image is stored
     * \param view_idx concatenation of the camera idx and view idx for src. The \a dst camera is set with \a set_ref_cam().  Defaults to cam & view 0
     * \param z reference depth for rectification and for non-central camera undistortion. z = 0 (the default) is interpreted as infinity
     */
    virtual void rectify(const cv::Mat &src, cv::Mat &dst, const Idx &view_idx, double z) const = 0;
    
    /** set a reference camera, used for rectification. The reference cameras is normally a camera retrieved with cam(), though only the pinhole properties (projection and pose) are used for rectification (as we want to simulate a perfect camera). */
    void set_ref_cam(const Cam *cam);
    const Cam *ref_cam() const;
  protected:
    const Cam * _ref_cam = NULL;
  };
  
  //Cam *rectified_view_cam(Calib *calib, const Idx &view);

  /*
  * 
  calibration input: proxy with: (xy,viewpoints,channels,cams,...,views)
  */

  /** Calculate a ray-based camera calibration from a proxy of support points 
   * 
   * The proxy is generated with \a proxy_pers_poly(). As the proxy can contain any number of views and cameras, see TODO, \a views_dims_start refers to the first view dimension, which is the first dimension from the proxy where the images contain only different views (e.g. extrinsics) of the different cameras specified in the lower dimensions.
   * \param proxy the calibration proxy containing the support points, generated by \a proxy_pers_poly(). The proxy should have size (2 x width x height x ... x views), where width and height specify the number of support points (often 33x25 or 65x49 for 4:3 images)
   * \param img_size the image size in pixels
   * \param opts optional calibration parameters, can be directly constructed from \a Flags bitflag
   * \param views_dims_start refers to the first view dimension, all lower dimension should refer to different cameras. Can be -1 to refer to the last dimension, -2 for the second last etc. If the \a proxy contains named dimensions, views_dims_start can be directly constructed from a string (e.g. "views").
   */
  Calib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const Options &opts = Options(0), const DimSpec &views_dims_start = DimSpec(-1));
  /** A variant of \a calibrate_rays() which uses a per support point Jacobian matrix (2x2) to calculate the residual in image coordinates. If the proxy has size (2 x W x H...) the jacobian must have size (2 x 2 x W x H ...)
   */
  Calib* calibrate_rays(Mat_<float> &proxy, const Mat_<float> &j, cv::Point2i img_size, const Options &opts = Options(0), const DimSpec &views_dims_start =  DimSpec(-1));
  
  /** Deserialize a previously serialized calibration. 
   * The parameters \a load_mat and \a load_string return a MetaMat::Mat respectively a cpath (aka boost::filesystem::path) from the supplied location. the std::function objects are often conveniently constructed using std::bind to integrate additional arguments.
   */
  Calib* load(std::function<Mat(cpath)> load_mat, std::function<const char *(cpath)> load_string);

}

#endif
