#ifndef _UCALIB_UCALIB_H
#define _UCALIB_UCALIB_H

#include <metamat/mat.hpp>

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);

double fit_cams_lines_multi(const clif::Mat_<float>& proxy, cv::Point2i img_size, clif::Mat_<double> &lines, clif::Mat_<double> &extrinsics, clif::Mat_<double> &extrinsics_rel);

#endif
