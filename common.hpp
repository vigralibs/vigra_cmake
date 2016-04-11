#ifndef _LUTCALIB_COMMON_H
#define _LUTCALIB_COMMON_H

#include <algorithm>
#include "opencv2/core/core.hpp"

template <typename T> inline T clamp(const T& n, const T& lower, const T& upper)
{
  return std::max<T>(lower, std::min<T>(n, upper));
}

template <typename T> inline T cnorm(const T& n, const T& nom, const T& denom)
{
  return std::max<T>(0, std::min<T>(n*nom/denom, nom-1));
}

void plane_calc_depth(cv::Mat cameraMatrix, cv::Mat rvec, cv::Mat tvec, cv::Size size, cv::Mat &out);
double z_from_world(cv::Point3f p, cv::Mat rvec, cv::Mat tvec);
double z_from_world2(cv::Point3f p, cv::Mat rvec, cv::Mat tvec);
void imwritenr(const char *name, cv::Mat &img, int nr);

//for 4 parameter camera only!
//p - img point, z - world!
cv::Point3f unproject(cv::Point2f p, double z, cv::Mat &c, cv::Mat &rvec, cv::Mat &tvec);

#endif