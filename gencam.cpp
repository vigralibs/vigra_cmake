#include "gencam.hpp"

#include <iostream>
#include <interpolation.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <omp.h>

#include <cstdarg>

using namespace cv;
using namespace alglib;
using namespace std;

double search_min_delta = 1e-6;

Point2d search_img_from_world(spline2dinterpolant img2world_spline[2], Point2d wp, Point2i img_size)
{
  Point2d it = Point2d(img_size.x*0.5,img_size.y*0.5);
  Point2d it_next;
  Point2d wp_it;
  Matx22d J, J_i;
  double dump;
  Point2d d(1,1);
  int i = 0;
  
  d.x = search_min_delta;
  d.y = search_min_delta;

  
  //printf("newton search %fx%f\n", wp.x, wp.y);
  
  while (abs(d.x)+abs(d.y) >= search_min_delta && i < 100) {
    i++;
    spline2ddiff(img2world_spline[0], it.x, it.y, wp_it.x, J(0,0), J(0,1), dump);
    spline2ddiff(img2world_spline[1], it.x, it.y, wp_it.y, J(1,0), J(1,1), dump);
  
    J_i = J.inv();
    
    //printf("%fx%f -> %fx%f\n", it.x, it.y, wp_it.x,wp_it.y);
    
    it_next = it + -J_i*(wp_it-wp);
    d = it_next-it;
    it = it_next;
  }
  
  //printf("newton search %fx%f : f(%f,%f) = %fx%f diff %f\n\n", wp.x, wp.y, it.x, it.y, wp_it.x, wp_it.y,abs(d.x)+abs(d.y));
  
  if (abs(d.x)+abs(d.y) >= search_min_delta) {
    it = Point2d(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
    //printf("fail for %dx%d\n", );
  }
  
  return it;
}

cv::Vec4d line_correct_proj(cv::Vec4d line, cv::Point2d f)
{
  cv::Point2d l_o(line[0], line[1]);
  cv::Point2d l_d(line[2], line[3]);
  
  return cv::Vec4d(l_o.x*f.x,l_o.y*f.y, l_d.x*f.x, l_d.y*f.y);
}

void get_undist_map_for_depth(clif::Mat_<double> lines, cv::Mat &map, double z, cv::Point2i idim, cv::Point2d f)
{
  cv::Point2i fdim(lines[1],lines[2]);
  
  cv::Mat_<cv::Point2d> offset(fdim.y, fdim.x);
  cv::Mat_<cv::Point2d> grad(fdim.y, fdim.x);
  
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      if (std::isnan(lines(0,x,y))) {
        offset.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        grad.at<cv::Point2d>(y,x) = cv::Point2d(NAN,NAN);
        continue;
      }
        
      
      cv::Point2d ip = cv::Point2d((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y);
      
      //line expressed in pixel using projection f
      cv::Vec4d line_unproj = line_correct_proj(cv::Vec4d(lines(0,x,y),lines(1,x,y),lines(2,x,y),lines(3,x,y)), f);
      
      offset.at<cv::Point2d>(y,x) = cv::Point2d(line_unproj[2], line_unproj[3]) - ip;
      
      grad.at<cv::Point2d>(y,x) = cv::Point2d(line_unproj[0], line_unproj[1]);      
    }
    
    
    
/***********************/
  
  real_1d_array cx;
  real_1d_array cy;
  real_2d_array fx;
  real_2d_array fy;
  spline2dinterpolant img_world_spline[2];
  map.create(idim.y,idim.x,CV_32FC2);
  
  cx.setlength(fdim.x);
  cy.setlength(fdim.y);
  fx.setlength(fdim.y, fdim.x);
  fy.setlength(fdim.y, fdim.x);
  
  for(int x=0;x<fdim.x;x++)
    cx[x] = (x+0.5)*idim.x/fdim.x;
  for(int y=0;y<fdim.y;y++)
    cy[y] = (y+0.5)*idim.y/fdim.y;

  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      fx(y, x) = (x+0.5)*idim.x/fdim.x + offset.at<Point2d>(y,x).x + (1.0/z)*grad.at<Point2d>(y,x).x;
      fy(y, x) = (y+0.5)*idim.y/fdim.y + offset.at<Point2d>(y,x).y + (1.0/z)*grad.at<Point2d>(y,x).y;
    }
      
  spline2dbuildbicubic(cx, cy, fx, fdim.y, fdim.x, img_world_spline[0]);
  spline2dbuildbicubic(cx, cy, fy, fdim.y, fdim.x, img_world_spline[1]);
  
  int approx_step = 8;
  
  omp_set_num_threads(8);
  
  int progress = 0;
#pragma omp parallel for num_threads(8)
  for(int y=0;y<idim.y;y+=approx_step) {
    for(int x=0;x<idim.x;x+=approx_step) {
      Point2d wp(x,y);
      Point2d ip = search_img_from_world(img_world_spline, wp, idim);
      map.at<Point2f>(y,x) = Point2f(ip.x, ip.y);
    }
  }
  
#pragma omp parallel for collapse(2) schedule(dynamic, 128)
  for(int y=0;y<idim.y-approx_step;y++) {
    for(int x=0;x<idim.x-approx_step;x++) {
      if (y % approx_step == 0 && x % approx_step == 0)
        continue;
      
      int x_m = x / approx_step * approx_step;
      int y_m = y / approx_step * approx_step;
      
      float xf, xf_i, yf, yf_i;
      xf_i = (x % approx_step)*1.0/approx_step;
      xf = 1.0-xf_i;
      yf_i = (y % approx_step)*1.0/approx_step;
      yf = 1.0-yf_i;
      
      Point2d tl = map.at<Point2f>(y_m,x_m);
      Point2d tr = map.at<Point2f>(y_m,x_m+approx_step);
      Point2d bl = map.at<Point2f>(y_m+approx_step,x_m);
      Point2d br = map.at<Point2f>(y_m+approx_step,x_m+approx_step);
      
      if (isnan(tl.x) || isnan(tr.x) || isnan(bl.x) || isnan(br.x)) {
        map.at<Point2f>(y,x) =  Point2d(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
        continue;
      }
        
      if (norm(tl - tr) >= 2.0*approx_step) {
        map.at<Point2f>(y,x) =  Point2d(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
        //printf("big step == fail? %f, %dx%d\n", norm(tl - tr),x ,y);
        continue;
      }
      
      Point2d t = tl*xf + tr*xf_i;
      Point2d b = bl*xf + br*xf_i;
      
      map.at<Point2f>(y,x) = t*yf + b*yf_i;
    }
  }
}
