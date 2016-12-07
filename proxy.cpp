#include "proxy.hpp"

#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace cv;
using namespace std;

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <cstdarg>

using namespace cv;
using namespace std;

const static int poly_default_xd = 3, poly_default_yd = 3;

void proxy_backwards_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim, double sigma)
{
  proxy_backwards_poly_generate<poly_default_xd,poly_default_yd>(proxy, img_points, world_points, idim, sigma);
}

void proxy_backwards_pers_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim, double sigma)
{
  proxy_backwards_pers_poly_generate<poly_default_xd,poly_default_yd>(proxy, img_points, world_points, idim, sigma);
}

