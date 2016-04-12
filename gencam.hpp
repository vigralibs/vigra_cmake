
#include <opencv2/core/core.hpp>


#include <mm/mat.hpp>

//class representing a calibrated generic (non-pinhole) camera, expressed relative to a pin-hole model
class GenCam {
public:
  cv::Point2d f; //focal length of the fitted pinhole in pixels
  double rms;
  cv::Point2i idim; //image size
  cv::Point2i fdim; //grid calibrated lines representing the depth-dependend camera projection
  double z_ref; //calibrated for this depth
  std::vector<double> extrinsics;
  std::vector<cv::Vec4d> _linefits;
  
  cv::Point2d move;
  double rot;
  
  cv::Mat offset; //offset from pinhole mode in pixels (address with y*fit_w+x)
  cv::Mat grad;  //depth dependend component (*1/z) (in px/mm)
  
  GenCam() {};
  GenCam(std::vector<cv::Vec4d> &linefits, cv::Point2i image_size, cv::Point2i fit_size, double at_depth = 0.0); //calc GenCam from fitted lines, assumes origin correction (camera center at (0,0,0))
  GenCam(std::vector<cv::Vec4d> &linefits, cv::Point2i img_size, cv::Point2i fit_size, double at_depth, cv::Point2d move_, cv::Point2d f_, double rot_ );
  
  void init();
  
  void get_undist_map_for_depth(cv::Mat &map, double z, std::vector<cv::Point3f> *rect_points_in = NULL, std::vector<cv::Point3f> *rect_points_out = NULL);
  cv::Point2f world2distorted(float z, cv::Point3f wp);
};


cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);
