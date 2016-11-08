#ifndef _UCALIB_PROXY_H
#define _UCALIB_PROXY_H

#include <metamat/mat.hpp>
#include <opencv2/core/core.hpp>

#include "loess.hpp"

template<int x_degree, int y_degree> void proxy_backwards_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0, int minpoints = 0)
{
  int progress = 0;
  if (sigma == 0.0)
    sigma = norm(cv::Point2f(idim.x/proxy[1],idim.y/proxy[2]))*0.5;
#ifndef WIN32
  #pragma omp parallel for schedule(dynamic,4) collapse(2)
#else  
  #pragma omp parallel for schedule(dynamic,4)
#endif
    for(int y=0;y<proxy[2];y++) {
      for(int x=0;x<proxy[1];x++) {
        int count;
        double coeffs[x_degree*y_degree*2];
        cv::Point2f c = cv::Point2f((x+0.5)*idim.x/proxy[1],(y+0.5)*idim.y/proxy[2]);
        double rms = fit_2d_poly_2d<x_degree,y_degree>(img_points, world_points, c, coeffs, sigma, &count);
        cv::Point2f res;
        if (std::isnan(rms) || ((!minpoints && count < 50) || (count < minpoints))
          /*|| rms >= 0.1*/)
          res = cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
        else
          res = eval_2d_poly_2d<x_degree,y_degree>(cv::Point2f(0,0), coeffs);
        proxy(0,x,y) = res.x;
        proxy(1,x,y) = res.y;
#pragma omp critical
        printf("rms: %3dx%3d %fx%f %f mm (%d points)\n", x, y, res.x, res.y, rms, count);
      }
    }
}

template<int x_degree, int y_degree> void proxy_backwards_pers_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0, int minpoints = 0, clif::Mat_<float> *J = NULL)
{
  int progress = 0;
  if (sigma == 0.0)
    sigma = norm(cv::Point2f(idim.x/proxy[1],idim.y/proxy[2]))*0.5;
  
  float hx = idim.x / proxy[1] / 2;
  float hy = idim.y / proxy[2] / 2;
  printf("weight falloff in middle: %f\n", exp(-(hx*hx+hy*hy)/(2.0*sigma*sigma)));
  
#ifndef WIN32
  #pragma omp parallel for schedule(dynamic,4) collapse(2)
#else  
  #pragma omp parallel for schedule(dynamic,4)
#endif
    for(int y=0;y<proxy[2];y++) {
      for(int x=0;x<proxy[1];x++) {
        int count;
        double xscale, yscale;
        clif::Mat_<float> j_small;
        clif::Mat_<float> *j_ptr = NULL;
        if (J) {
          j_ptr = &j_small;
          j_small = J->bindAll(-1, -1, x, y);
        }
        double coeffs[9+x_degree*y_degree*2];
        cv::Point2f c = cv::Point2f((x+0.5)*idim.x/proxy[1],(y+0.5)*idim.y/proxy[2]);
        double rms = fit_2d_pers_poly_2d<x_degree,y_degree>(img_points, world_points, c, coeffs, sigma, &count, j_ptr);
        cv::Point2f res;
        if (std::isnan(rms) || ((!minpoints && count < 50) || (count < minpoints))
          /*|| rms >= 0.15*/) //FIXME debug large rms!
          res = cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
        else
          res = eval_2d_pers_poly_2d<x_degree,y_degree>(cv::Point2f(0,0), coeffs);
        proxy(0,x,y) = res.x;
        proxy(1,x,y) = res.y;
//#pragma omp critical
  //      printf("rms: %3dx%3d %fx%f %f px (%d points)\n", x, y, res.x, res.y, rms, count);
      }
    }
}

void proxy_backwards_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0);

void proxy_backwards_pers_poly_generate(clif::Mat_<float> &proxy, std::vector<cv::Point2f> img_points, std::vector<cv::Point3f> world_points, cv::Point2i idim, double sigma = 0.0);

#endif
