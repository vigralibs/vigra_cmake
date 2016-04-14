#include "corr_lines.hpp"

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

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "loess.hpp"

#include <cstdarg>

using namespace cv;
using namespace std;

const static int poly_xd = 3, poly_yd = 3;

void proxy_backwards_poly_generate(clif::Mat_<float> proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim)
{
  int progress = 0;
  double sigma = norm(Point2f(idim.x/32,idim.y/32));
    
#pragma omp parallel for schedule(dynamic,4) collapse(2)
    for(int y=0;y<proxy[2];y++) {
      for(int x=0;x<proxy[1];x++) {
        int count;
        double coeffs[poly_xd*poly_yd*2];
        Point2f c = Point2f((x+0.5)*idim.x/proxy[1],(y+0.5)*idim.y/proxy[2]);
        double rms = fit_2d_poly_2d<poly_xd,poly_yd>(img_points, world_points, c, coeffs, sigma*0.5, &count);
        Point2f res;
        if (std::isnan(rms) || count < 50
          || rms >= 0.1)
          res = Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
        else
          res = eval_2d_poly_2d<poly_xd,poly_yd>(Point2f(0,0), coeffs);
        proxy(0,x,y) = res.x;
        proxy(1,x,y) = res.y;
#pragma omp critical
        printf("rms: %3dx%3d %fx%f %f px (%d points)\n", x, y, res.x, res.y, rms, count);
      }
    }
}

