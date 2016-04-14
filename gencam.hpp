#ifndef _UCALIB_GENCAM_H
#define _UCALIB_GENCAM_H

#include <metamat/mat.hpp>

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f);
void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f);


#endif
