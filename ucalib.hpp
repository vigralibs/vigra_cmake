#ifndef _UCALIB_UCALIB_H
#define _UCALIB_UCALIB_H

#include <metamat/mat.hpp>

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);

//needs!
//proxy.names({"point","x","y","channels","cams","views"});
//lines.names({"line","x","y","channels","cams"})
double fit_cams_lines_multi(clif::Mat_<float>& proxy, cv::Point2i img_size, clif::Mat_<double> &lines, clif::Mat_<double> &extrinsics, clif::Mat_<double> &extrinsics_rel, clif::Mat_<double> &proj, bool vis = false, const clif::Mat_<float>& scales = clif::Mat_<float>());

namespace ucalib {
  
void projectPoints(const std::vector<cv::Point3f> &wpoints, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &cameraMatrix, clif::Mat_<double> lines, double z, cv::Point2i idim, std::vector<cv::Point2f> &ipoints);

}

#endif
