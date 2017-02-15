#include "proxy.hpp"

using cv::Point2i;

namespace ucalib {
  
using namespace std;
using namespace MetaMat;

const static int poly_default_xd = 3, poly_default_yd = 3;

void proxy_pers_poly(Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, Point2i idim, double sigma, int minpoints, Mat_<float> *J)
{
  proxy_pers_poly<poly_default_xd,poly_default_yd>(proxy, img_points, world_points, idim, sigma, minpoints, J);
}

}

