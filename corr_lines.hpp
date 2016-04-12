#ifndef _LUTCALIB_CORR_LINES_H
#define _LUTCALIB_CORR_LINES_H

#include "settings.hpp"
#include "simplecloud.hpp"

#include <mm/mat.hpp>
 
class DistCorr
{
public:
  int w, h, d;
  int img_w, img_h, img_d;
  int dims[3];
  int img_dims[3];
  bool ready = false;
  SimpleCloud3d cloud;
  cv::Mat camera_matrix;
  bool use_cam_mat;
  bool use_proxy = false;
  int x_min, x_max, y_min, y_max;
  double z_step = 1.0;
  
  static const int proxy_count = 1000;
  
  
  const static int poly_xd = 3, poly_yd = 3; 
  
  
  std::vector<std::vector<cv::Mat> > proxy_backwards_pers;
  std::vector<std::vector<int> > proxy_backwards_pers_counts;
  std::vector<std::vector<cv::Point2f>> proxy_backwards;
  std::vector<std::vector<int>> proxy_backwards_valid;
  std::vector<cv::Vec4d> linefits;
  cv::Point3d linefits_center;
  
  std::vector<std::vector<cv::Point2f> > img_points;
  std::vector<std::vector<cv::Point3f> > world_points;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  
  std::vector<std::vector<cv::Point2f> > img_points_proxy;
  std::vector<std::vector<cv::Point3f> > world_points_proxy;
  std::vector<std::vector<cv::Point2f> > img_points_corrected_proxy;
  
  std::vector<double> extrinsics;
  
  
  DistCorr() {};
  DistCorr(int cw, int ch, int cd, int iw, int ih, int id, Cam_Config &cam_config, Calib_Config &calib_config, cv::Point2i fit_size);
  //cv::Point2f correct_nn(cv::Point2f p, float depth);
  cv::Point2f correct(cv::Point2f p, float depth);
  void calcWriteCorrImgs(std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point3f> > &world_points, std::vector<char*> imgs, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs);
  void correct_points(std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point3f> > world_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, std::vector<std::vector<cv::Point2f> > &img_points_corrected);
  void img_corr_map(cv::Mat &img, cv::Mat &out, cv::Mat cameraMatrix, cv::Mat rvec, cv::Mat tvec, cv::Size size);
  void uncorrectProjectPoints(std::vector<cv::Point3f> world_points, std::vector<cv::Point2f> &img_points, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix);
  void calcWriteRefImgs(std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point3f> > &world_points, std::vector<char*> imgs, 
  std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs);
  double noDistCalib(std::vector<std::vector<cv::Point3f> > &world_points, std::vector<std::vector<cv::Point2f> > &img_points, cv::Mat &cameraMatrix, std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs, bool pnp_only = false, bool refine_only = false);
  void write(void);
  void correct_and_calc_rms(std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point3f> > world_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, std::vector<std::vector<cv::Point2f> > &img_points_corrected, cv::Mat &cameraMatrix);
  
  void check_calib_forward_calc_rms(std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point3f> > world_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, std::vector<std::vector<cv::Point2f> > &img_points_corrected, cv::Mat &cameraMatrix);
  
  void proxy_generate(void);
  void proxy_fit_lines(void);
  void proxy_backwards_generate(void);
  void proxy_backwards_poly_generate(void);
  void proxy_backwards_save(const char * const filename);
  void proxy_update(void);
  
  double proxy_fit_lines_global(void);
  
  void lines_correct_coordinate_system(void);
  void proxy_draw_lines(const char *prefix);
  void proxy_draw_line_fit(const char *prefix, double error_scale);
  void proxy_setlimit(cv::Point2i b, cv::Point2i e);
  void proxy_remove_line_fit_bias(void);
  void proxy_remove_line_fit_bias_perspective(void);
  void line_fit_projection_center(double z_ref);
  void line_fit_correct_by_projection_center(void);
  
  //calculate correction map for the specified depth
  void get_correction_map_at_depth(cv::Mat &map, double depth);
  //calculate opencv 8-parameter distortion correction for depth
  void get_correction_cv8_at_depth(cv::Mat &distCoeffs, double depth);
  
  //add to point cloud
  void add(cv::Point2f c, cv::Point2f p, float depth);
  
  //add collection of points
  //FIXME currently replaces current collection!
  void add(std::vector<std::vector<cv::Point2f> > &img_points_i, std::vector<std::vector<cv::Point3f> > &world_points_i, double step);
  
  //must be defined for inherited classes
  virtual cv::Point2f offset(cv::Point2f p, float depth) = 0;
  virtual cv::Point2f correct_forward(cv::Point2f p, float depth) = 0;
  virtual void recalc() = 0;
  
private:
  double correction_integrate(std::vector<std::vector<cv::Point3f> > &world_points, std::vector<std::vector<cv::Point2f> > &img_points, std::vector<std::vector<cv::Point2f> > &img_points_corrected, std::vector<char*> imgs);
  
protected:
  Cam_Config cam;
  Calib_Config config;
  cv::Point2i _fit_size;
};

class DistCorrLines : public DistCorr
{
public:
  DistCorrLines() {};
  int Draw(const char *prefix);
  DistCorrLines(int cw, int ch, int cd, int iw, int ih, int id, Cam_Config &cam_config, Calib_Config &calib_config, cv::Point2i fit_size);
  
  void calibrate(std::vector<std::vector<cv::Point2f> > &img_points_i, std::vector<std::vector<cv::Point3f> > &world_points_i, std::vector<char*> imgs);
  
  cv::Point2f offset(cv::Point2f p, float depth) {};
  cv::Point2f correct_forward(cv::Point2f p, float depth) {};
  void recalc() {};
private :
};

void proxy_backwards_poly_generate(clif::Mat_<float> proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim);

#endif
