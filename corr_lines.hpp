#ifndef _LUTCALIB_CORR_LINES_H
#define _LUTCALIB_CORR_LINES_H

#include <metamat/mat.hpp>

void proxy_backwards_poly_generate(clif::Mat_<float> proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim);

#endif
