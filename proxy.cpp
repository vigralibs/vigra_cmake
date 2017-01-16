#include "proxy.hpp"

using cv::Point2i;

namespace ucalib {
  
using namespace std;

const static int poly_default_xd = 3, poly_default_yd = 3;

void proxy_backwards_poly_generate(Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim, double sigma)
{
  proxy_backwards_poly_generate<poly_default_xd,poly_default_yd>(proxy, img_points, world_points, idim, sigma);
}

void proxy_backwards_pers_poly_generate(Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim, double sigma)
{
  proxy_backwards_pers_poly_generate<poly_default_xd,poly_default_yd>(proxy, img_points, world_points, idim, sigma);
}

}

