#ifndef _UCALIB_PROXY_H
#define _UCALIB_PROXY_H

#include <metamat/mat.hpp>

void proxy_backwards_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0);

void proxy_backwards_pers_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0);

#endif
