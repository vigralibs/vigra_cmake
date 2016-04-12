#include "corr_lines.hpp"

#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "cam_models.hpp"

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

#include "simplecloud.hpp"
#include "common.hpp"
#include "cam_models.hpp"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "fitting.hpp"
#include "loess.hpp"

#include <cstdarg>

#include <mm/mat.hpp>

using namespace cv;
using namespace std;

double DistCorr::noDistCalib(vector<vector<Point3f> > &world_points, vector<vector<Point2f> > &img_points, Mat &cameraMatrix, vector<Mat> &rvecs, vector<Mat> &tvecs, bool pnp_only, bool refine_only)
{
  double rms;
  Mat distCoeffs = Mat::zeros(1, 8, CV_64F);
  
  //FIXME damping?
  cameraMatrix = camera_matrix.clone();
  
  if (!pnp_only) {
    if (!refine_only)
      return ceres_calibrate_camera(world_points, img_points, cameraMatrix, cam.initial_focus, rvecs, tvecs);
    else
      return ceres_refine_camera(world_points, img_points, cameraMatrix, rvecs, tvecs);
  }
  
  rvecs.resize(world_points.size());
  tvecs.resize(world_points.size());
  for(int i=0;i<world_points.size();i++) {
    solvePnP(world_points[i], img_points[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
    printf("solvepnp img %d\n", i);
  }
}

DistCorr::DistCorr(int cw, int ch, int cd, int iw, int ih, int id, Cam_Config &cam_config, Calib_Config &calib_config, Point2i fit_size)
{ 
  cam = cam_config;
  config = calib_config;
  
  double f = 1.0/(1.0/cam.initial_f-1.0/cam.initial_focus);
  
  x_min = 0;
  x_max = fit_size.x-1;
  y_min = 0;
  y_max = fit_size.y-1;
  
  _fit_size = fit_size;
  
  w = cw;
  h = ch;
  d = cd;
  
  dims[0] = w;
  dims[1] = h;
  dims[2] = d;
  
  img_w = iw;
  img_h = ih;
  img_d = id;

  img_dims[0] = img_w;
  img_dims[1] = img_h;
  img_dims[2] = img_d;
  
  cloud = SimpleCloud3d(w,h,d, iw,ih,id);
  
  camera_matrix = Mat::zeros(3, 3, CV_64F);
  
  camera_matrix.at<double>(0,0) = f/cam.pixel_pitch;
  camera_matrix.at<double>(0,2) = img_w/2;
  
  camera_matrix.at<double>(1,1) = f/cam.pixel_pitch;
  camera_matrix.at<double>(1,2) = img_h/2;
  
  camera_matrix.at<double>(2,2) = 1;
  
}

void DistCorr::add(Point2f c, Point2f p, float z)
{
  cloud.add(DistPoint(c, Point3f(p.x, p.y, z)));
}


void DistCorr::proxy_generate(void)
{
  int idx;
  
  //random selection
  /*for(int i=0;i<img_points.size();i++) {
    if (img_points[i].size() <= proxy_count) {
      img_points_proxy.push_back(img_points[i]);
      world_points_proxy.push_back(world_points[i]);
      continue;
    }
    img_points_proxy.push_back(vector<Point2f>(0));
    world_points_proxy.push_back(vector<Point3f>(0));
    for(int p=0;p<proxy_count;p++) {
      idx = rand() % img_points[i].size();
      img_points_proxy[i].push_back((img_points[i])[idx]);
      world_points_proxy[i].push_back((world_points[i])[idx]);
    }
  }*/
  
  //poisson disk sampling
  /*const int select_iterations = 10;
  
  for(int i=0;i<img_points.size();i++) {
    if (img_points[i].size() <= proxy_count) {
      img_points_proxy.push_back(img_points[i]);
      world_points_proxy.push_back(world_points[i]);
      continue;
    }
    img_points_proxy.push_back(vector<Point2f>(0));
    world_points_proxy.push_back(vector<Point3f>(0));
    
    for(int p=0;p<proxy_count;p++) {
      int best_idx = 0;
      float best_d = -1;
      float dist;
      Point2f sel;
      for(int t=0;t<select_iterations;t++) {
        idx = rand() % img_points[i].size();
        sel = (img_points[i])[p];
        float dist_worst = 1000000000;
        for(int s=0;s<img_points_proxy[i].size();s++) {
          Point2f v = (img_points_proxy[i])[s] - sel;
          dist = v.x*v.x+v.y*v.y;
          if (dist < dist_worst)
            dist_worst = dist;
        }
        if (dist_worst > best_d) {
          best_d = dist_worst;
          best_idx = idx;
        }
      }
      img_points_proxy[i].push_back((img_points[i])[best_idx]);
      world_points_proxy[i].push_back((world_points[i])[best_idx]);
    }
  }*/

  img_points_proxy.resize(world_points.size());
  world_points_proxy.resize(world_points.size());
  
  printf("proxy generation "); fflush(NULL);
  
  //image based cell fitting - an exercise in applied indexing terrorism :-P
  for(int i=0;i<img_points.size();i++) {
    printf("."); fflush(NULL);
    vector<int> points[_fit_size.y][_fit_size.x];
    for(int p=0;p<img_points[i].size();p++) {
      Point2f pos = (img_points[i])[p];
      points[cnorm<int>(pos.y, _fit_size.y, img_h)][cnorm<int>(pos.x, _fit_size.x, img_w)].push_back(p);
    }
    for(int y=0;y<_fit_size.y;y++)
#pragma omp parallel for schedule(dynamic)
      for(int x=0;x<_fit_size.x;x++) {
        if (!points[y][x].size())
          continue;
        //do regular camera calibration - but only for a small extract!
        vector<vector<Point2f>> ips(1);
        vector<vector<Point3f>> wps(1);
        ips[0].resize(points[y][x].size());
        wps[0].resize(points[y][x].size());
        for(int c=0;c<points[y][x].size();c++) {
          ips[0][c] = (img_points[i])[(points[y][x])[c]];
          wps[0][c] = (world_points[i])[(points[y][x])[c]];
        }
        vector<Mat> rvecs, tvecs;
        Mat cameraMatrix = init_cam(cam);
        Point2f ip = Point2f((x+0.5)*img_w/_fit_size.x,(y+0.5)*img_h/_fit_size.y);
        //Point2f ip = ips[0][0];
        //distortion-less model
        //wps[0].pop_back();
        //ips[0].pop_back();
        double rms = ceres_calibrate_camera(wps, ips, cameraMatrix, cam.initial_focus, rvecs, tvecs);
        Point3f w = unproject(ip, 0, cameraMatrix, rvecs[0], tvecs[0]);
        
        if (rms >= 0.5) {
#pragma omp critical 
          printf("\nuhh ohh rms %f for %d %dx%d\n", rms, i, x, y);
        }
        
/*#pragma omp critical 
        {

        Matx31f v = Matx31f(w-wps[0][0]);
        double len = norm(v);
        if (len > 0.5) {
        printf("size:  %d\n", wps[0].size());
          cout << wps[0] << endl;
          cout << ips[0] << endl;
          printf("%.2f\n", len);
          printf("img %d %dx%d rms: %f from %d points\n", i, x, y, rms, wps[0].size());   
          printf("focal: %f %f\n",cameraMatrix.at<double>(0,0)*cam.pixel_pitch,cameraMatrix.at<double>(1,1)*cam.pixel_pitch);
          cout << w << " = " << wps[0][0] << endl;
          cout << w-wps[0][0] << endl;
        }
        }*/
        
        //TODO check numeric stability?
        /*
        vector<Point3f> tpoints(1);
        vector<Point2f> ppoints(1);
        tpoints[0] = w;
        projectPoints(tpoints, rvecs[0], tvecs[0], cameraMatrix, noArray(), ppoints);
        cout << Point2f((x+0.5)*_fit_size.x,(y+0.5)*_fit_size.y) << " = " << ppoints[0] << endl;*/
#pragma omp critical
        {
          if (ips[0].size() > 10 && rms < 0.5) {
            world_points_proxy[i].push_back(w);
            img_points_proxy[i].push_back(ip);
          }
          else {
            //printf("not enough for %d %dx%d - %d\n", i, x, y, ips[0].size());
            for(int c=0;c<points[y][x].size();c++) {
              img_points_proxy[i].push_back((img_points[i])[(points[y][x])[c]]);
              world_points_proxy[i].push_back((world_points[i])[(points[y][x])[c]]);
            }
          }
        }
      }
  }
  printf("\n");
}

void DistCorr::proxy_backwards_generate(void)
{
  int idx;

  img_points_proxy.resize(world_points.size());
  world_points_proxy.resize(world_points.size());
  
  printf("proxy generation "); fflush(NULL);
  
  proxy_backwards_pers.resize(world_points.size());
  proxy_backwards_pers_counts.resize(world_points.size());
  
  //image based cell fitting - an exercise in applied indexing terrorism :-P
  for(int i=0;i<img_points.size();i++) {
    printf("."); fflush(NULL);
    
    proxy_backwards_pers[i].resize(_fit_size.x*_fit_size.y);
    proxy_backwards_pers_counts[i].resize(_fit_size.x*_fit_size.y);
  
    vector<int> points[_fit_size.y][_fit_size.x];
    for(int p=0;p<img_points[i].size();p++) {
      Point2f pos = (img_points[i])[p];
      points[cnorm<int>(pos.y, _fit_size.y, _fit_size.x)][cnorm<int>(pos.x, _fit_size.x, _fit_size.y)].push_back(p);
    }
#pragma omp parallel for schedule(dynamic)
    for(int y=0;y<_fit_size.y;y++)
      for(int x=0;x<_fit_size.x;x++) {
        if (!points[y][x].size())
          continue;
        //do regular camera calibration - but only for a small extract!
        vector<Point2f> ips;
        vector<Point2f> wps;
        ips.resize(points[y][x].size());
        wps.resize(points[y][x].size());
        for(int c=0;c<points[y][x].size();c++) {
          ips[c] = (img_points[i])[(points[y][x])[c]];
          Point3f wp = (world_points[i])[(points[y][x])[c]];
          assert(wp.z == 0.0);
          wps[c] = Point2f(wp.x,wp.y);
        }
        
        proxy_backwards_pers_counts[i][y*_fit_size.x+x] = ips.size();
        if (ips.size() >= 4) {
          Mat pers = findHomography(ips, wps, 0);
          proxy_backwards_pers[i][y*_fit_size.x+x] = pers;
          
          vector<Point2f> proj;
          
          perspectiveTransform(ips, proj, pers);
          double rms = 0.0;
          for(int i=0;i<proj.size();i++) {
            Point2f d(proj[i]-wps[i]);
            rms += d.x*d.x+d.y*d.y;
          }
          
          printf("pers rms: %d %dx%d %f (from %lu points)\n", i, x, y, sqrt(rms/proj.size()), proj.size());
        }
        else {
          printf("not enough for %d %dx%d - %lu\n", i, x, y, ips.size());
        }
      }
  }
  printf("\n");
}

void printprogress(int curr, int max, int &last, const char *fmt = NULL, ...)
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

void DistCorr::proxy_backwards_poly_generate(void)
{
  int progress = 0;
  double sigma = norm(Point2f(img_w/32,img_h/32));
  
  proxy_backwards.resize(img_points.size());

  for(int i=0;i<img_points.size();i++) {
    proxy_backwards[i].resize(_fit_size.x*_fit_size.y);
    
//#pragma omp critical
  //    printprogress(i*_fit_size.y+y, _fit_size.y*img_points.size(), progress);
    #pragma omp parallel for schedule(dynamic,4) collapse(2)
    for(int y=y_min;y<=y_max;y++) {
      for(int x=x_min;x<=x_max;x++) {
        int count;
        double coeffs[poly_xd*poly_yd*2];
        Point2f c = Point2f((x+0.5)*img_w/_fit_size.x,(y+0.5)*img_h/_fit_size.y);
        double rms = fit_2d_poly_2d<poly_xd,poly_yd>(img_points[i], world_points[i], c, coeffs, sigma*0.5, &count);
        if (std::isnan(rms) || count < 50
          || rms >= 0.1)
          proxy_backwards[i][y*_fit_size.x+x] = Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
        else
          proxy_backwards[i][y*_fit_size.x+x] = eval_2d_poly_2d<poly_xd,poly_yd>(Point2f(0,0), coeffs);
#pragma omp critical
        printf("rms: %4d %3dx%3d %fx%f %f px (%d points)\n", i, x, y, proxy_backwards[i][y*_fit_size.x+x].x, proxy_backwards[i][y*_fit_size.x+x].y, rms, count);
      }
    }
  }
}

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

const double ransac_include_threshold = 0.1;
const double ransac_success_threshold = 0.9;

const double fit_line_max_rms = 1000.0;

void DistCorr::proxy_fit_lines(void)
{
  int count = proxy_backwards.size();
  
  linefits.resize(_fit_size.x*_fit_size.y);
  
  proxy_backwards_valid.resize(count);
  for(int i=0;i<count;i++)
    proxy_backwards_valid[i].resize(_fit_size.x*_fit_size.y);
  
  for(int y=y_min;y<=y_max;y++)
    for(int x=x_min;x<=x_max;x++) {
      int invalid = 0;
      int count_sum = 0;
      vector<int> valid(count);
      
      vector<Point3f> points(count);
      //vector<Point2f> scales;
      //vector<int> counts;
      
      for(int i=0;i<count;i++)
        points[i] = Point3f(proxy_backwards[i][y*_fit_size.x+x].x, proxy_backwards[i][y*_fit_size.x+x].y, z_step*i);
      
      double line[4];
      //double rms = fit_line_suw(points, scales, counts, line);
      //double rms = fit_line_s(points, scales, line);
      //double rms = fit_line(points, line);
      double rms = fit_line_ransac(points, line, ransac_include_threshold, ransac_success_threshold, &valid);
      
      for(int i=0;i<count;i++)
        proxy_backwards_valid[i][y*_fit_size.x+x] = valid[i];
      
      double mxe = 0.0, mye = 0.0;
      int valid_count = 0;
      for(int i=0;i<count;i++)
        if (valid[i]) {
        mxe = max(mxe, abs((points[i].x - (line[0]+line[2]*points[i].z))));
        mye = max(mye, abs((points[i].y - (line[1]+line[3]*points[i].z))));
        Point2d d = Point2d(points[i].x, points[i].y) - (Point2d(line[0],line[1])+Point2d(line[2],line[3])*points[i].z);
        valid_count++;
      }
      
      if (rms <= fit_line_max_rms) {
        printf("line %4dx%4d fit rms %.3f μm max error: %4.3fx%4.3f μm (ransac kept %3.1f%%)\n", x,y, rms*1000, mxe*1000, mye*1000,100.0*valid_count/count);
        linefits[y*_fit_size.x+x] = Vec4d(line[0],line[1],line[2],line[3]);
      }
      else {
        printf("invalid fit %4dx%4d\n", x,y);
        linefits[y*_fit_size.x+x] = Vec4d(NAN,NAN,NAN,NAN);
      }
    }
}

double DistCorr::proxy_fit_lines_global(void)
{
  linefits.resize(_fit_size.x*_fit_size.y);
  
  double rms = fit_cams_lines(proxy_backwards, linefits, Point2i(_fit_size.x, _fit_size.y), Point2i(img_w, img_h), z_step, extrinsics);
  
  printf("global cam fit rms %fmm\n", rms);
  
  return rms;
}


const float error_scale = 1.0;
double len = 1200.0;

int DistCorrLines::Draw(const char *prefix)
{
  int progress;
  char buf[1024];
  int count = proxy_backwards.size();
  
  sprintf(buf, "%s.obj", prefix);

  FILE *obj = fopen(buf, "w");
  
  int obj_v_counter = 0;
  
  for(int y=0;y<_fit_size.y;y++)
    for(int x=0;x<_fit_size.x;x++) {
        printprogress(y*_fit_size.x+x, _fit_size.y*_fit_size.x, progress);
        Vec4d line = linefits[y*_fit_size.x+x];
        
        if (std::isnan(line[0]))
          continue;
        
        Point3f o(line[0],line[1],0);
        Point3f d(line[2],line[3],1);
        Point2f v(x-_fit_size.x/2,y-_fit_size.y/2);
        double cd = norm(v);
        double md = norm(Point2f(_fit_size.x/2,_fit_size.y/2));
        double nd = cd/md;
        
        //if (nd >= 0.6)
          //continue;
        
        //TODO hack - use rms for coloring
        //nd = 1.0-rms*10.0;
        
        nd *= 255;
        
        Point3f bp = o-0.5*len*d;
        Point3f ep = o+0.5*len*d;
        bp = bp*1000.0;
        ep = ep*1000.0;
        
        if (norm(d) > 2.0) {
          printf("len %f\n", norm(d));
          continue;
        }

        fprintf(obj, "o o%dx%d\n", x,y);
        fprintf(obj, "g g%dx%d\n", x,y);
        fprintf(obj, "v %.0f %.0f %.0f %.2f %.2f %.2f\n", bp.x, bp.y, bp.z, nd, 0.0, 255.0-nd);
        fprintf(obj, "v %.0f %.0f %.0f %.2f %.2f %.2f\n", ep.x, ep.y, ep.z, nd, 0.0, 255.0-nd);
        fprintf(obj, "v %.0f %.0f %.0f %.2f %.2f %.2f\n", bp.x, bp.y, bp.z, nd, 0.0, 255.0-nd);
        fprintf(obj, "f %d %d %d\n\n", obj_v_counter+1, obj_v_counter+2,obj_v_counter+3);
        obj_v_counter += 3;
      /*}
      else
        printf("%d invalid avg count %.1f!\n", invalid, (float)count_sum/count);*/
    }
    
  Point3d lb, le;
  char *dir_str = "xyz";
  for(int dir=0;dir<3;dir++) {
    fprintf(obj, "o center_%c\n", dir_str[dir]);
    fprintf(obj, "g center_%c\n", dir_str[dir]);
    Vec3d off(0,0,0);
    off[dir] = 1;
    lb = (-Point3d(off))*1000;
    le = (Point3d(off))*1000;
    fprintf(obj, "v %.0f %.0f %.0f 255 255 255\n", lb.x, lb.y, lb.z);
    fprintf(obj, "v %.0f %.0f %.0f 255 255 255\n", le.x, le.y, le.z);
    fprintf(obj, "v %.0f %.0f %.0f 255 255 255\n", le.x, le.y, le.z);
    fprintf(obj, "f %d %d %d\n\n", obj_v_counter+1, obj_v_counter+2,obj_v_counter+3);
    obj_v_counter += 3;
  }
  
  fclose(obj);
}

void DistCorr::proxy_setlimit(Point2i b, Point2i e)
{
  x_min = b.x;
  x_max = e.x;
  y_min = b.y;
  y_max = e.y;
}


void DistCorr::proxy_remove_line_fit_bias(void)
{
  int count = proxy_backwards.size();
  
  
  for(int i=0;i<count;i++) {
    Point2d bias(0,0);  
    for(int y=y_min;y<=y_max;y++)
      for(int x=x_min;x<=x_max;x++) {
        Vec4d line = linefits[y*_fit_size.x+x];
        Point2d wp = proxy_backwards[i][y*_fit_size.x+x];
        Point2d fp(line[0]+i*z_step*line[2], line[1]+i*z_step*line[3]);
        bias += wp-fp;
    }
    bias *= 1.0/((y_max-y_min+1)*(x_max-x_min+1));
    cout << "bias " << i << " " << bias << endl;
    for(int y=y_min;y<=y_max;y++)
      for(int x=x_min;x<=x_max;x++)
        proxy_backwards[i][y*_fit_size.x+x] -= Point2f(bias.x, bias.y);
  }
}


void DistCorr::proxy_remove_line_fit_bias_perspective(void)
{
  int count = proxy_backwards.size();
  
  
  for(int i=0;i<count;i++) {
    Point2d bias(0,0);
    vector<Point2f> wps_fit;
    vector<Point2f> wps_raw;
    for(int y=y_min;y<=y_max;y++)
      for(int x=x_min;x<=x_max;x++) {
        Vec4d line = linefits[y*_fit_size.x+x];
        if (std::isnan(line[0]))
          continue;
        Point2d wp = proxy_backwards[i][y*_fit_size.x+x];
        wps_raw.push_back(wp);
        Point2d fp(line[0]+i*z_step*line[2], line[1]+i*z_step*line[3]);
        wps_fit.push_back(fp);
    }
    Mat pers = findHomography(wps_raw, wps_fit, CV_LMEDS);
    perspectiveTransform(proxy_backwards[i], proxy_backwards[i], pers);
  }
}


/*
 * correct coordinate system by removing movement and tilt using center line as reference
 */
void DistCorr::lines_correct_coordinate_system(void)
{
  double dx, dy, cx, cy;
  Vec4d center = linefits[(_fit_size.y/2-1)*_fit_size.x+_fit_size.x/2-1]
                +linefits[(_fit_size.y/2-1)*_fit_size.x+_fit_size.x/2]
                +linefits[_fit_size.y/2*_fit_size.x+_fit_size.x/2-1]
                +linefits[_fit_size.y/2*_fit_size.x+_fit_size.x/2];
  center *= 0.25;
  
  for(int y=0;y<_fit_size.y;y++)
    for(int x=0;x<_fit_size.x;x++)
      linefits[y*_fit_size.x+x] -= center;
}

void DistCorr::line_fit_projection_center(double z_ref)
{  
  double rms = fit_center_of_projection(linefits, linefits_center, z_ref);
  cout << "fitted projection center: " << linefits_center << " fit center ang. rms: " << rms << endl;
}

void DistCorr::line_fit_correct_by_projection_center(void)
{  
  for(int y=0;y<_fit_size.y;y++)
    for(int x=0;x<_fit_size.x;x++) {
      //linefits_center.z points in the opposite direction from the line dirs!
      linefits[y*_fit_size.x+x][0] = linefits[y*_fit_size.x+x][0] - linefits_center.x + linefits_center.z*linefits[y*_fit_size.x+x][2];
      linefits[y*_fit_size.x+x][1] = linefits[y*_fit_size.x+x][1] - linefits_center.y + linefits_center.z*linefits[y*_fit_size.x+x][3];
    }
}


//FIXME for testing only - need to generate proper proxy
void DistCorr::proxy_update(void)
{
  if (!rvecs.size()) {
    use_proxy = false;
    return;
  }
  
  img_points_proxy.resize(0);
  world_points_proxy.resize(0);
  img_points_corrected_proxy.resize(0);
  
  use_proxy = true;
  
  //filter by depth
  for(int i=0;i<img_points.size();i++) {
    img_points_proxy.push_back(vector<Point2f>(0));
    world_points_proxy.push_back(vector<Point3f>(0));
    for(int p=0;p<img_points[i].size();p++) {
      double z = z_from_world2((world_points[i])[p], rvecs[i], tvecs[i]);
      if (z >= config.zmin && z <= config.zmax) {
        img_points_proxy[i].push_back((img_points[i])[p]);
        world_points_proxy[i].push_back((world_points[i])[p]);
      }
    }
    /*if (img_points[i].size())
      printf("proxy zfilter img %d kept %f%%\n", i, img_points_proxy[i].size()*100.0/img_points[i].size());
    else
      printf("poxy skip empty img %d\n", i);*/
  }
}

void DistCorr::add(vector<vector<Point2f> > &img_points_i, vector<vector<Point3f> > &world_points_i, double step)
{
  img_points = img_points_i;
  world_points = world_points_i;
  
  z_step = step;
}

Mat map_invert(Mat &map)
{
  Rect box(Point(), map.size());
  int w = map.size().width;
  int h = map.size().height;
  Point2i tgt;
  assert(map.type() == CV_32FC2);
  Mat newmap = Mat::zeros(map.size(), map.type());
 
  for(int j=0;j<h;j++)
    for(int i=0;i<w;i++) {
      tgt = map.at<Point2f>(j, i);
      if (box.contains(tgt))
        newmap.at<Point2f>(tgt.y, tgt.x) = Point2f(i, j);
    }
    
  return newmap;
}

void DistCorr::get_correction_cv8_at_depth(cv::Mat &distCoeffs, double depth)
{ 
  vector<vector<Point3f>> wps(1);
  vector<vector<Point2f>> ips(1);
  
  for(int y=y_min;y<=y_max;y++)
    for(int x=x_min;x<=x_max;x++) {
      Vec4d line = linefits[y*_fit_size.x+x];
      if (std::isnan(line[0]))
        continue;
      Point3f wp = Point3d(line[0], line[1], 0) + depth*Point3d(line[2], line[3], 0);
      Point2f ip((x+0.5)*img_w/_fit_size.x, (y+0.5)*img_h/_fit_size.y);
      
      wps[0].push_back(wp);
      ips[0].push_back(ip);
    }
    
  Mat cameraMatrix;
  vector<Mat> rvecs, tvecs;
  double rms = calibrateCamera(wps, ips, Size(img_w, img_h), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_RATIONAL_MODEL);
  
  printf("opencv rms: %f at depth %f (CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_RATIONAL_MODEL)\n", rms, depth);
  
  rms = calibrateCamera(wps, ips, Size(img_w, img_h), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_RATIONAL_MODEL);
  
  printf("opencv rms: %f at depth %f (CV_CALIB_RATIONAL_MODEL)\n", rms, depth);
  
  cout << distCoeffs << endl;
}


DistCorrLines::DistCorrLines(int cw, int ch, int cd, int iw, int ih, int id, Cam_Config &cam_config, Calib_Config &calib_config, Point2i fit_size)
: DistCorr(cw,ch,cd, iw,ih,id, cam_config, calib_config, fit_size)
{

}


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct DepthReprojectionError {
  DepthReprojectionError(double observed_x, double observed_y, double p[3])
      : observed_x(observed_x), observed_y(observed_y) {
        w_point[0] = p[0];
        w_point[1] = p[1];
        w_point[2] = p[2];
      }

  template <typename T>
  bool operator()(const T* const extr, const T* const intr, const T* const dist, const T* const depth,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3], w_p[3];
    w_p[0] = T(w_point[0]);
    w_p[1] = T(w_point[1]);
    w_p[2] = T(w_point[2]);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& k1 = dist[0];
    const T& k2 = dist[1];
    const T& k3 = dist[2];
    const T& k4 = dist[3];
    const T& k5 = dist[4];
    const T& k6 = dist[5];
    const T r2 = xp*xp + yp*yp;
    const T r4 = r2*r2;
    const T r6 = r2*r4;
    const T& d1 = depth[0];
    const T& d2 = depth[1];
    const T& d3 = depth[2];
    const T z2 = (p[2]-d1)*(p[2]-d1);
    const T df = (1.0+d2*z2+d3);
    T distortion = (T(1.0) + k1*r2 + k2*r4 + k3*r6)/(T(1.0) + k4*r2 + k5*r4 + k6*r6)*df;

    const T& p1 = dist[6];
    const T& p2 = dist[7];
    
    // Compute final projected point position.
    const T& focal1 = intr[0];
    const T& focal2 = intr[1];
    T predicted_x = focal1 * xp * distortion + intr[2] + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
    T predicted_y = focal2 * yp * distortion + intr[3] + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     double *p) {
    return (new ceres::AutoDiffCostFunction<DepthReprojectionError, 2, 6, 4, 8, 3>(
                new DepthReprojectionError(observed_x, observed_y, p)));
  }

  double observed_x;
  double observed_y;
  double w_point[3];
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y, double p[3])
      : observed_x(observed_x), observed_y(observed_y) {
        w_point[0] = p[0];
        w_point[1] = p[1];
        w_point[2] = p[2];
      }

  template <typename T>
  bool operator()(const T* const extr, const T* const intr, const T* const dist,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3], w_p[3];
    w_p[0] = T(w_point[0]);
    w_p[1] = T(w_point[1]);
    w_p[2] = T(w_point[2]);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& k1 = dist[0];
    const T& k2 = dist[1];
    const T& k3 = dist[2];
    const T& k4 = dist[3];
    const T& k5 = dist[4];
    const T& k6 = dist[5];
    T r2 = xp*xp + yp*yp;
    T r4 = r2*r2;
    T r6 = r2*r4;
    T distortion = (T(1.0) + k1*r2 + k2*r4 + k3*r6)/(T(1.0) + k4*r2 + k5*r4 + k6*r6);

    const T& p1 = dist[6];
    const T& p2 = dist[7];
    
    // Compute final projected point position.
    const T& focal1 = intr[0];
    const T& focal2 = intr[1];
    T predicted_x = focal1 * xp * distortion + intr[2] + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
    T predicted_y = focal2 * yp * distortion + intr[3] + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     double *p) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 4, 8>(
                new SnavelyReprojectionError(observed_x, observed_y, p)));
  }

  double observed_x;
  double observed_y;
  double w_point[3];
};


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct BasicReprojectionError {
  BasicReprojectionError(double observed_x, double observed_y, double p[3])
      : observed_x(observed_x), observed_y(observed_y) {
        w_point[0] = p[0];
        w_point[1] = p[1];
        w_point[2] = p[2];
      }

  template <typename T>
  bool operator()(const T* const extr, const T* const intr,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3], w_p[3];
    w_p[0] = T(w_point[0]);
    w_p[1] = T(w_point[1]);
    w_p[2] = T(w_point[2]);
    ceres::AngleAxisRotatePoint(extr, w_p, p);

    // camera[3,4,5] are the translation.
    p[0] += extr[3];
    p[1] += extr[4];
    p[2] += extr[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Compute final projected point position.
    const T& focal1 = intr[0];
    const T& focal2 = intr[0];
    T predicted_x = focal1 * xp + intr[1];
    T predicted_y = focal2 * yp + intr[2];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     double *p) {
    return (new ceres::AutoDiffCostFunction<BasicReprojectionError, 2, 6, 3>(
                new BasicReprojectionError(observed_x, observed_y, p)));
  }

  double observed_x;
  double observed_y;
  double w_point[3];
};

void DistCorrLines::calibrate(std::vector<std::vector<cv::Point2f> > &img_points_i, std::vector<std::vector<cv::Point3f> > &world_points_i, std::vector<char*> imgs)
{
  double f = 1.0/(1.0/cam.initial_f-1.0/cam.initial_focus);
  double extrinsics[world_points_i.size()*6];
  double intrinsics[15];
  double *extr;
  
  intrinsics[0] = f/cam.pixel_pitch;
  intrinsics[1] = f/cam.pixel_pitch;
  intrinsics[2] = cam.w/2;
  intrinsics[3] = cam.h/2;
  for(int i=4;i<15;i++)
    intrinsics[i] = 0;
    
  for(int i=0;i<world_points_i.size();i++) {
    extrinsics[6*i] = 0;
    extrinsics[6*i+1] = 0;
    extrinsics[6*i+2] = 0;
    
    extrinsics[6*i+3] = 0;
    extrinsics[6*i+4] = 0;
    extrinsics[6*i+5] = cam.initial_focus; //translation
  }
  
#pragma omp parallel for schedule(dynamic)
  for(int i=0;i<world_points_i.size();i++) {
    double rms = ceres_pnp_view_basic(world_points_i[i], img_points_i[i], intrinsics, extrinsics+6*i, cam.initial_focus);
#pragma omp critical
    printf("img %d pnp score %f\n", i, rms);
  }
  
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;*/
  //options.function_tolerance = 1e-4;
  
  ceres::Solver::Summary summary;
  
  ceres::Problem problem_dist;
  for(int i=0;i<world_points_i.size();i++) {
    for(int p=0;p<(world_points_i[i]).size();p++) {
      extr = &extrinsics[i*6];
      double w_p[3];
      w_p[0] = (world_points_i[i])[p].x;
      w_p[1] = (world_points_i[i])[p].y;
      w_p[2] = (world_points_i[i])[p].z;

      ceres::CostFunction* cost_function =
          SnavelyReprojectionError::Create((img_points_i[i])[p].x,
                                          (img_points_i[i])[p].y,
                                          w_p
                                          );
      problem_dist.AddResidualBlock(cost_function,
                              NULL ,
                              extr, intrinsics, intrinsics+4);
    }
  }
  
  ceres::Solve(options, &problem_dist, &summary);
  //std::cout << summary.FullReport() << "\n";
  cout << "radial distortion score: " << 2*sqrt(summary.final_cost/problem_dist.NumResiduals()) << endl;
  
  return;
  
  options.function_tolerance = 1e-20;
  
  ceres::Problem problem_depth;
  for(int i=0;i<world_points_i.size();i++) {
    for(int p=0;p<(world_points_i[i]).size();p++) {
      extr = &extrinsics[i*6];
      double w_p[3];
      w_p[0] = (world_points_i[i])[p].x;
      w_p[1] = (world_points_i[i])[p].y;
      w_p[2] = (world_points_i[i])[p].z;

      ceres::CostFunction* cost_function =
          DepthReprojectionError::Create((img_points_i[i])[p].x,
                                          (img_points_i[i])[p].y,
                                          w_p
                                          );
      problem_depth.AddResidualBlock(cost_function,
                              NULL /* squared loss */,
                              extr, intrinsics, intrinsics+4, intrinsics+4+8);
    }
  }

  ceres::Solve(options, &problem_depth, &summary);
  std::cout << summary.FullReport() << "\n";
  cout << "distortion score: " << 2*sqrt(summary.final_cost/problem_depth.NumResiduals()) << endl;
  
  //now repeat with a proxy!
  cloud.reset();
  /*for(int i=0;i<world_points.size();i++)
    for(int p=0;<world_points[i].size();p++)
      add();*/
}

