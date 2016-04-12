
#include "gencam.hpp"

#include <iostream>
#include <interpolation.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <omp.h>

#include <cstdarg>

#include "fitting.hpp"


using namespace cv;
using namespace alglib;
using namespace std;

void get_rot_mat(double rot, Matx22d &mat)
{
  mat = Matx22d(cos(rot),-sin(rot),
                sin(rot), cos(rot)
               );
}

/*Point2d proj_world_from_img(Point2d ip, Matx22d &unrot, Point2d move, Point2d f)
{
  ip.x = ip.x / f.x;
  ip.y = ip.y / f.y;
  
  ip -= move;
  ip = unrot*ip;
}*/

//correct line by movement, rotation and projection!
//this means we don't get real (measured) world coordinate but world coordinates relative to camera
//use output to get world coordinate at z from line: w.x = (line[0]+z*line[1])/f.x
Vec4d line_correct_rot_move_proj(Vec4d line, Matx22d &rot, Point2d move, Point2d f)
{
  Point2d l_o(line[0], line[1]);
  Point2d l_d(line[2], line[3]);
  
  l_o = rot * l_o;
  
  l_d = rot * l_d + move;
  
  return Vec4d(l_o.x*f.x,l_o.y*f.y, l_d.x*f.x, l_d.y*f.y);
}

//correct line by movement and, rotation!
//this means we don't get real (measured) world coordinate but world coordinates relative to camera
//use output to get world coordinate at z from line: w.x = (line[0]+z*line[1])/f.x
Vec4d line_correct_rot_move(Vec4d line, Matx22d &rot, Point2d move)
{
  Point2d l_o(line[0], line[1]);
  Point2d l_d(line[2], line[3]);
  
  l_o = rot * l_o;
  l_o = l_o;
  
  l_d = rot * l_d + move;
  
  return Vec4d(l_o.x,l_o.y, l_d.x, l_d.y);
}

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

void GenCam::init()
{
  Matx22d rotmat;
  
  get_rot_mat(rot, rotmat);

  offset = Mat(fdim.y, fdim.x, CV_64FC2);
  grad = Mat(fdim.y, fdim.x, CV_64FC2);
  Mat ref = Mat(fdim.y, fdim.x, CV_64FC2);
  
  for(int y=0;y<fdim.y;y++)
    for(int x=0;x<fdim.x;x++) {
      if (std::isnan(_linefits[y*fdim.x+x][0])) {
        offset.at<Point2d>(y,x) = Point2d(NAN,NAN);
        grad.at<Point2d>(y,x) = Point2d(NAN,NAN);
        continue;
      }
        
      
      Point2d ip = Point2d((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y);
      
      //line expressed in pixel using projection f
      Vec4d line_unproj = line_correct_rot_move_proj(_linefits[y*fdim.x+x], rotmat, move, f);
      
      offset.at<Point2d>(y,x) = Point2d(line_unproj[2], line_unproj[3]) - ip;
      
      grad.at<Point2d>(y,x) = Point2d(line_unproj[0], line_unproj[1]);      
    }
}

GenCam::GenCam(std::vector<cv::Vec4d> &linefits, cv::Point2i img_size, cv::Point2i fit_size, double at_depth)
{
  idim = img_size;
  fdim = fit_size;
  z_ref = at_depth;
  
  _linefits = linefits;
  
  rms = fit_projection(linefits, idim, fdim, f, rot, move, z_ref);
  

  //printf("projection rms %f (%fx%f)\n", rms, f.x, f.y);
  //printf("move %fx%fmm rot %f°\n", move.x, move.y, fmod(rot, 2*M_PI)*180.0/M_PI);
  
  init();
}


GenCam::GenCam(std::vector<cv::Vec4d> &linefits, cv::Point2i img_size, cv::Point2i fit_size, double at_depth, cv::Point2d f_, cv::Point2d m_, double r_ )
{
  idim = img_size;
  fdim = fit_size;
  z_ref = at_depth;
  
  _linefits = linefits;
  
  f = f_;
  move = m_;
  rot = r_;
  
  init();
}

static void printprogress(int curr, int max, int &last, const char *fmt = NULL, ...)
{
  last = (last + 1) % 4;
  int pos = curr*60/max;
  char unf[] = "                                                             ]";
  char fin[] = "[============================================================]";
  char buf[100];
  
  char cs[] = "-\\|/";
  memcpy(buf, fin, pos+1);
  buf[pos+1] = cs[last];
  memcpy(buf+pos+2, unf+pos+2, 62-pos-2+1);
  if (!fmt) {
    printf("%s\r", buf);
  }
  else {
    printf("%s", buf);
    va_list arglist;
    va_start(arglist, fmt);
    vprintf(fmt, arglist);
    va_end(arglist);
    printf("\r");
  }
  fflush(NULL);
}

//FIXME depth is the problem
Point2f GenCam::world2distorted(float z, Point3f wp)
{
  real_1d_array cx;
  real_1d_array cy;
  real_2d_array fx;
  real_2d_array fy;
  spline2dinterpolant img_world_spline[2];
  
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
  
  //world to camera
  /*wp = wp - cv::Point2f(move.x, move.y);
  wp.x /= f.x;
  wp.y /= f.y;*/
  
  Matx22d rotmat;
  get_rot_mat(rot, rotmat);
  
  cv::Mat extr_rot;
  cv::Rodrigues(Vec3f(extrinsics[0],extrinsics[1],extrinsics[2]), extr_rot);
  Mat ip_w_m = extr_rot * Mat(wp);
  ip_w_m += Mat(Point3f(extrinsics[3],extrinsics[4],extrinsics[5]));
  Mat ip_w_cp = ip_w_m.clone();
  assert(ip_w_m.depth() == CV_32F);
  
  ip_w_m.at<float>(0) = rotmat(0, 0) * ip_w_cp.at<float>(0) + rotmat(0, 1) * ip_w_cp.at<float>(1);
  ip_w_m.at<float>(1) = rotmat(1, 0) * ip_w_cp.at<float>(0) + rotmat(1, 1) * ip_w_cp.at<float>(1);
  //translation:
  ip_w_m.at<float>(0) += move.x;
  ip_w_m.at<float>(1) += move.y;
  
  
  Point2f ip = Point2f(ip_w_m.at<float>(0) * f.x / ip_w_m.at<float>(2), ip_w_m.at<float>(1) * f.y / ip_w_m.at<float>(2));
  ip += Point2f(idim.x/2,idim.y/2);
  
  return search_img_from_world(img_world_spline, ip, idim);
}

void GenCam::get_undist_map_for_depth(cv::Mat &map, double z, vector<Point3f> *rect_points_in, vector<Point3f> *rect_points_out)
{  
  real_1d_array cx;
  real_1d_array cy;
  real_2d_array fx;
  real_2d_array fy;
  spline2dinterpolant img_world_spline[2];
  map.create(idim.y,idim.x,CV_32FC2);
  
  //calculate z from rect_points_in[0]
  /*if (rect_points_in) {
    Matx22d rotmat;
    get_rot_mat(rot, rotmat);
    
    cv::Mat extr_rot;
    cv::Rodrigues(Vec3f(extrinsics[0],extrinsics[1],extrinsics[2]), extr_rot);
    Mat ip_w_m = extr_rot * Mat((*rect_points_in)[0]);
    ip_w_m += Mat(Point3f(extrinsics[3],extrinsics[4],extrinsics[5]));
    Mat ip_w_cp = ip_w_m.clone();
    assert(ip_w_m.depth() == CV_32F);
    
    ip_w_m.at<float>(0) = rotmat(0, 0) * ip_w_cp.at<float>(0) + rotmat(0, 1) * ip_w_cp.at<float>(1);
    ip_w_m.at<float>(1) = rotmat(1, 0) * ip_w_cp.at<float>(0) + rotmat(1, 1) * ip_w_cp.at<float>(1);
    //translation:
    ip_w_m.at<float>(0) += move.x;
    ip_w_m.at<float>(1) += move.y;
    
    z = ip_w_m.at<float>(2);
    printf("ref point depth in this camera: %f\n", z);
  }*/
  
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
printf("calc parallel\n");
#pragma omp parallel for num_threads(8)
  for(int y=0;y<idim.y;y+=approx_step) {
    for(int x=0;x<idim.x;x+=approx_step) {
      Point2d wp(x,y);
      Point2d ip = search_img_from_world(img_world_spline, wp, idim);
      map.at<Point2f>(y,x) = Point2f(ip.x, ip.y);
    }
  }
  
printf("calc parallel p2\n");
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
printf("calced parallel\n");

  /*double dist_frac = 0.25; //max == 0.5!
  double dmax = 0.5+dist_frac; //max == 0.5!
  double dmin = 0.5-dist_frac; //max == 0.5!
  
  std::vector<Point2f> frac(4);
    
  frac[0] = Point2f(dmin, dmin);
  frac[1] = Point2f(dmax, dmin);
  frac[2] = Point2f(dmax, dmax);
  frac[3] = Point2f(dmin, dmax);
  
  //perspective normalisation:
  //single image
  /*vector<Point2f> src_sim(4), dst_sim(4);
  if (!rect_points_in && !rect_points_out) {
    for(int i=0;i<4;i++) {
      int x = fdim.x*frac[i].x;
      int y = fdim.y*frac[i].y;
      dst_sim[i] = Point2i((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y)
                   + Point2i(idim.x/2,idim.y/2);
    }
      

    for(int i=0;i<4;i++)
      src_sim[i] = map.at<Point2f>(dst_sim[i].y, dst_sim[i].x);
    
    //cv::Mat pers = cv::getPerspectiveTransform(src, dst);
    
    //perspectiveTransform(map, map, pers);
  }*/
  
  /*if (!rect_points_in && !rect_points_out)
    rect_points_out = new std::vector<Point3f>(4);
  
  
  std::vector<Point2f> img_corner(4);
  std::vector<Point2i> line_corner(4);
  
  for(int i=0;i<4;i++)
    line_corner[i] = Point2i(fdim.x*frac[i].x, fdim.y*frac[i].y);
  
  for(int i=0;i<4;i++) {
    float x = line_corner[i].x;
    float y = line_corner[i].y;
    Point2i tmp = Point2f((x+0.5-fdim.x*0.5)/fdim.x*idim.x, (y+0.5-fdim.y*0.5)/fdim.y*idim.y)
                  + Point2f(idim.x/2,idim.y/2);
    img_corner[i] = tmp;
  }
  
  //rectification create target coordinate reference points
  if (rect_points_out) {
    
    rect_points_out->resize(4);
  
    //take image point and project into world (in distorted! coordinates)
    Vec4d line[4];
    
    for(int i=0;i<4;i++)
      line[i] = _linefits[line_corner[i].x + line_corner[i].y * fdim.x];
    
    //reference points in target coordinates
    for(int i=0;i<4;i++) {
      (*rect_points_out)[i] = Point3f(line[i][0] + z*line[i][2], line[i][1] + z*line[i][3], z);
      (*rect_points_out)[i] -= Point3f(extrinsics[3],extrinsics[4],extrinsics[5]);
      cv::Mat rot, rot_t;
      cv::Rodrigues(Vec3f(extrinsics[0],extrinsics[1],extrinsics[2]), rot);
      transpose(rot, rot_t);
      Mat wp = rot_t * Mat((*rect_points_out)[i]);
      assert(wp.depth() == CV_32F);
      (*rect_points_out)[i] = Point3f(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2));
    }
    
    assert(!rect_points_in);
    rect_points_in = rect_points_out;
  }
  
  std::vector<Point2f> tmp_ip(4);
  std::vector<Point2f> dst(4);
  
  //project reference points using fitted projection
  //make reference points fall exactly on respective reference image points
  if (rect_points_in) {
    vector<Point2f> src(4);
    
    Matx22d rotmat;
    get_rot_mat(rot, rotmat);
    
    //src == what we obtain from target with fitted (cam local) projection - this is the result of our previous undistortion mask
    //FIXME handle center offset! (move) and rotation!
    for(int i=0;i<4;i++) {
      cv::Mat extr_rot;
      cv::Rodrigues(Vec3f(extrinsics[0],extrinsics[1],extrinsics[2]), extr_rot);
      Mat ip_w_m = extr_rot * Mat((*rect_points_in)[i]);
      ip_w_m += Mat(Point3f(extrinsics[3],extrinsics[4],extrinsics[5]));
      Mat ip_w_cp = ip_w_m.clone();
      assert(ip_w_m.depth() == CV_32F);
      
      ip_w_m.at<float>(0) = rotmat(0, 0) * ip_w_cp.at<float>(0) + rotmat(0, 1) * ip_w_cp.at<float>(1);
      ip_w_m.at<float>(1) = rotmat(1, 0) * ip_w_cp.at<float>(0) + rotmat(1, 1) * ip_w_cp.at<float>(1);
      //translation:
      ip_w_m.at<float>(0) += move.x;
      ip_w_m.at<float>(1) += move.y;
      
      
      Point2f ip = Point2f(ip_w_m.at<float>(0) * f.x / ip_w_m.at<float>(2), ip_w_m.at<float>(1) * f.y / ip_w_m.at<float>(2));
      ip += Point2f(idim.x/2,idim.y/2);

      //correct using reference projection
      //src[i] = img_corner[i]-(ip-img_corner[i]);
      
      tmp_ip[i] = world2distorted(z, (*rect_points_in)[i]);
    }
    
    //we already use some source - only correct difference!
    
    
    for(int i=0;i<4;i++) {
      dst[i] = img_corner[i] + (tmp_ip[i]-map.at<Point2f>(img_corner[i].y,img_corner[i].x));
      //Point2f wp = search_img_from_world(img_world_spline, img_corner[i], idim);
      //dst[i] = img_corner[i] + (tmp_ip[i]-wp);
    }
    
    cv::Mat pers = cv::getPerspectiveTransform(img_corner, dst);
    
    
    for(int i=0;i<4;i++)
      printf("map before %fx%f should be %fx%f\n", map.at<Point2f>(img_corner[i].y,img_corner[i].x).x, map.at<Point2f>(img_corner[i].y,img_corner[i].x).y, tmp_ip[i].x, tmp_ip[i].y);
    
    Point2f off = tmp_ip[0]-map.at<Point2f>(img_corner[0].y,img_corner[0].x);
    
    /*for(int y=0;y<idim.y;y++)
      for(int x=0;x<idim.x;x++)
        map.at<Point2f>(y,x) += off;*/
      
    /*perspectiveTransform(map, map, pers);
    
    for(int i=0;i<4;i++)
      printf("map %fx%f should be %fx%f\n", map.at<Point2f>(img_corner[i].y,img_corner[i].x).x, map.at<Point2f>(img_corner[i].y,img_corner[i].x).y, tmp_ip[i].x, tmp_ip[i].y);
    
    //FIXME why do we need a second iteration?
    printf("try 2\n");
    {
    for(int i=0;i<4;i++)
      dst[i] = img_corner[i] + (tmp_ip[i]-map.at<Point2f>(img_corner[i].y,img_corner[i].x));
    pers = cv::getPerspectiveTransform(img_corner, dst);
    perspectiveTransform(map, map, pers);
    
    for(int i=0;i<4;i++)
      printf("map %fx%f should be %fx%f\n", map.at<Point2f>(img_corner[i].y,img_corner[i].x).x, map.at<Point2f>(img_corner[i].y,img_corner[i].x).y, tmp_ip[i].x, tmp_ip[i].y);
    }
    
  }*/
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
