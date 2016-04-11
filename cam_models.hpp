#ifndef _LUTCALIB_CAMMODELS_H
#define _LUTCALIB_CAMMODELS_H

#include "opencv2/opencv.hpp"

#include "settings.hpp"

double ceres_calibrate_camera(std::vector<std::vector<cv::Point3f> > &world_points, std::vector<std::vector<cv::Point2f> > &img_points, cv::Mat &cameraMatrix, float initial_focus, std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs);

double ceres_refine_camera(std::vector<std::vector<cv::Point3f> > &world_points, std::vector<std::vector<cv::Point2f> > &img_points, cv::Mat &cameraMatrix, std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs);

double ceres_pnp_view_basic(std::vector<cv::Point3f> &world_points, std::vector<cv::Point2f> &img_points, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, double initial_focus, int num_points = 0);

double ceres_pnp_view_basic(std::vector<cv::Point3f> &world_points, std::vector<cv::Point2f> &img_points, double *intrinsics, double *extrinsics, double initial_focus, int num_points = 0);

cv::Mat init_cam(Cam_Config &cam);

#endif
