#ifndef _LUTCALIB_FITTING_H
#define _LUTCALIB_FITTING_H

#include "settings.hpp"

#include "opencv2/core/core.hpp"

double fit_line_s(std::vector<cv::Point3f> &points, std::vector<cv::Point2f> &scales, double *line);
double fit_line_suw(std::vector<cv::Point3f> &points, std::vector<cv::Point2f> &scales, std::vector<int> &counts, double *line);
double fit_line(std::vector<cv::Point3f> &points, double *line);

double fit_line_ransac(std::vector<cv::Point3f> &points, double *line, double incl_th, double succ_th, std::vector<int> *include = NULL);

double fit_center_of_projection(std::vector<cv::Vec4d> &lines, cv::Point3d &center, double z_ref);

double fit_projection(std::vector<cv::Vec4d> &lines, cv::Point2i img_size, cv::Point2i fit_size, cv::Point2d &projection, double &rot, cv::Point2d &move, double z_ref);

double fit_cams_lines(std::vector<std::vector<cv::Point2f>> &proxy_backwards, std::vector<cv::Vec4d> &linefits, cv::Point2i proxy_size, cv::Point2i img_size, double z_step, std::vector<double> &extrinsics_v);

#endif
