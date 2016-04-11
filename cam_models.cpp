#include "cam_models.hpp"

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

#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace cv;
using namespace std;


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct PNPReprojectionError {
  PNPReprojectionError(double observed_x, double observed_y, double p[3], double i[4])
      : observed_x(observed_x), observed_y(observed_y) {
        w_point[0] = p[0];
        w_point[1] = p[1];
        w_point[2] = p[2];
        intr[0] = i[0];
        intr[1] = i[1];
        intr[2] = i[2];
        intr[3] = i[3];
      }

  template <typename T>
  bool operator()(const T* const extr,
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
    const T& focal1 = T(intr[0]);
    const T& focal2 = T(intr[1]);
    T predicted_x = focal1 * xp + T(intr[2]);
    T predicted_y = focal2 * yp + T(intr[3]);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     double *p, double *i) {
    return (new ceres::AutoDiffCostFunction<PNPReprojectionError, 2, 6>(
                new PNPReprojectionError(observed_x, observed_y, p, i)));
  }

  double observed_x;
  double observed_y;
  double w_point[3];
  double intr[4];
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
    const T& focal2 = intr[1];
    T predicted_x = focal1 * xp + intr[2];
    T predicted_y = focal2 * yp + intr[3];

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
    return (new ceres::AutoDiffCostFunction<BasicReprojectionError, 2, 6, 4>(
                new BasicReprojectionError(observed_x, observed_y, p)));
  }

  double observed_x;
  double observed_y;
  double w_point[3];
};

void convert_r_3d(double *in, Mat &out)
{
  out = Mat(3, 1, CV_64F);
  out.at<double>(0,0) = in[0];
  out.at<double>(1,0) = in[1];
  out.at<double>(2,0) = in[2];
}


void convert_t_3d(double *in, Mat &out)
{
  out = Mat(3, 1, CV_64F);
  out.at<double>(0,0) = in[0];
  out.at<double>(1,0) = in[1];
  out.at<double>(2,0) = in[2];
}

double ceres_pnp_refine_basic(vector<Point3f> &world_points, vector<Point2f> &img_points, double *intrinsics, double *extrinsics, int num_points)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  options.num_threads = 2;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;*/
  
  ceres::Solver::Summary summary;

  if (num_points)
    num_points = std::min(num_points, (int)world_points.size());
  else
    num_points = world_points.size();
  
  ceres::Problem problem_pnp;
  for(int p=0;p<num_points;p++) {
    double w_p[3];
    w_p[0] = world_points[p].x;
    w_p[1] = world_points[p].y;
    w_p[2] = world_points[p].z;

    ceres::CostFunction* cost_function =
        PNPReprojectionError::Create(img_points[p].x,
                                        img_points[p].y,
                                        w_p, intrinsics);
    problem_pnp.AddResidualBlock(cost_function,
                            NULL,
                            extrinsics);
  }
  
  ceres::Solve(options, &problem_pnp, &summary);
  
  
  //std::cout << summary.FullReport() << "\n";
  
  return 2*sqrt(summary.final_cost/problem_pnp.NumResiduals());
}


double ceres_pnp_view_basic(vector<Point3f> &world_points, vector<Point2f> &img_points, double *intrinsics, double *extrinsics, double initial_focus, int num_points)
{
  extrinsics[0] = 0;
  extrinsics[1] = 0;
  extrinsics[2] = 0;
  
  extrinsics[3] = 0;
  extrinsics[4] = 0;
  //FIXME how to scale this?? Opencv somehow manages a good solution!
  extrinsics[5] = initial_focus;// = initial_focus; //translation

  return ceres_pnp_refine_basic(world_points, img_points, intrinsics, extrinsics, num_points);
}

double ceres_pnp_view_basic(vector<Point3f> &world_points, vector<Point2f> &img_points, Mat &rvec, Mat &tvec, Mat &cameraMatrix, double initial_focus, int num_points)
{
  double extrinsics[6];
  double intrinsics[4];
  
  intrinsics[0] = cameraMatrix.at<double>(0,0);
  intrinsics[1] = cameraMatrix.at<double>(1,1);
  intrinsics[2] = cameraMatrix.at<double>(0,2);
  intrinsics[3] = cameraMatrix.at<double>(1,2);
  
  double rms = ceres_pnp_view_basic(world_points, img_points, intrinsics, extrinsics, initial_focus, num_points);
  
  convert_r_3d(extrinsics, rvec);
  convert_t_3d(extrinsics+3, tvec);
  
  return rms;
}


double ceres_calibrate_camera(vector<vector<Point3f> > &world_points, vector<vector<Point2f> > &img_points, double *intrinsics, float initial_focus, double *extrinsics)
{
  double *extr;
    
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  options.num_threads = 2;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-4;*/
  
  
//#pragma omp parallel for schedule(dynamic)
  for(int i=0;i<world_points.size();i++) {
    //double rms = ceres_pnp_view_basic(world_points[i], img_points[i], intrinsics, extrinsics+6*i, initial_focus, 8);
    //printf("img %d small pnp score %f\n", i, rms);
    double rms = ceres_pnp_view_basic(world_points[i], img_points[i], intrinsics, extrinsics+6*i, initial_focus);
    //printf("img %d pnp score %f\n", i, rms);
  }
  
  
  ceres::Solver::Summary summary;  
  ceres::Problem problem_basic;
  for(int i=0;i<world_points.size();i++) {
    for(int p=0;p<world_points[i].size();p++) {
      extr = &extrinsics[i*6];
      double w_p[3];
      w_p[0] = (world_points[i])[p].x;
      w_p[1] = (world_points[i])[p].y;
      w_p[2] = (world_points[i])[p].z;

      ceres::CostFunction* cost_function =
          BasicReprojectionError::Create((img_points[i])[p].x,
                                          (img_points[i])[p].y,
                                          w_p
                                          );
      problem_basic.AddResidualBlock(cost_function,
                              NULL /* squared loss */,
                              extr, intrinsics);
    }
  }
  
  options.minimizer_progress_to_stdout = true;
  options.function_tolerance = 1e-6;
  
  ceres::Solve(options, &problem_basic, &summary);
  //std::cout << summary.FullReport() << "\n";
  //cout << "intr/extr score: " << 2*sqrt(summary.final_cost/problem_basic.NumResiduals()) << endl;
  
  return 2*sqrt(summary.final_cost/problem_basic.NumResiduals());
}

double ceres_calibrate_camera(vector<vector<Point3f> > &world_points, vector<vector<Point2f> > &img_points, Mat &cameraMatrix, float initial_focus, vector<Mat> &rvecs, vector<Mat> &tvecs)
{
  double extrinsics[world_points.size()*6];
  double intrinsics[4];
  double *extr;
  
  intrinsics[0] = cameraMatrix.at<double>(0,0);
  intrinsics[1] = cameraMatrix.at<double>(1,1);
  intrinsics[2] = cameraMatrix.at<double>(0,2);
  intrinsics[3] = cameraMatrix.at<double>(1,2);
    
  double rms = ceres_calibrate_camera(world_points, img_points, intrinsics, initial_focus, extrinsics);
  
  rvecs.resize(world_points.size());
  tvecs.resize(world_points.size());
  
  for(int i=0;i<world_points.size();i++) {
    convert_r_3d(extrinsics+i*6, rvecs[i]);
    convert_t_3d(extrinsics+i*6+3, tvecs[i]);
  }
  
  cameraMatrix.at<double>(0,0) = intrinsics[0];
  cameraMatrix.at<double>(1,1) = intrinsics[1];
  cameraMatrix.at<double>(0,2) = intrinsics[2];
  cameraMatrix.at<double>(1,2) = intrinsics[3];
  
  return rms;
}


double ceres_refine_camera(vector<vector<Point3f> > &world_points, vector<vector<Point2f> > &img_points, Mat &cameraMatrix, vector<Mat> &rvecs, vector<Mat> &tvecs)
{
  double extrinsics[world_points.size()*6];
  double intrinsics[4];
  double *extr;
  
  intrinsics[0] = cameraMatrix.at<double>(0,0);
  intrinsics[1] = cameraMatrix.at<double>(1,1);
  intrinsics[2] = cameraMatrix.at<double>(0,2);
  intrinsics[3] = cameraMatrix.at<double>(1,2);
    
  for(int i=0;i<world_points.size();i++) {
    extrinsics[6*i] = rvecs[i].at<double>(0,0);
    extrinsics[6*i+1] = rvecs[i].at<double>(1,0);
    extrinsics[6*i+2] = rvecs[i].at<double>(2,0);
    
    extrinsics[6*i+3] = tvecs[i].at<double>(0,0);
    extrinsics[6*i+4] = tvecs[i].at<double>(1,0);
    extrinsics[6*i+5] = tvecs[i].at<double>(2,0);
  }
  
  ceres::Problem problem_basic;
  for(int i=0;i<world_points.size();i++) {
    for(int p=0;p<(world_points[i]).size();p++) {
      extr = &extrinsics[i*6];
      double w_p[3];
      w_p[0] = (world_points[i])[p].x;
      w_p[1] = (world_points[i])[p].y;
      w_p[2] = (world_points[i])[p].z;

      ceres::CostFunction* cost_function =
          BasicReprojectionError::Create((img_points[i])[p].x,
                                          (img_points[i])[p].y,
                                          w_p
                                          );
      problem_basic.AddResidualBlock(cost_function,
                              NULL /* squared loss */,
                              extr, intrinsics);
    }
  }
  
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  options.num_threads = 2;
  /*options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;*/
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_basic, &summary);
  //std::cout << summary.FullReport() << "\n";
  cout << "intr/extr score: " << 2*sqrt(summary.final_cost/problem_basic.NumResiduals()) << endl;
  
  rvecs.resize(world_points.size());
  tvecs.resize(world_points.size());
  
  for(int i=0;i<world_points.size();i++) {
    convert_r_3d(extrinsics+i*6, rvecs[i]);
    convert_t_3d(extrinsics+i*6+3, tvecs[i]);
  }
  
  cameraMatrix.at<double>(0,0) = intrinsics[0];
  cameraMatrix.at<double>(1,1) = intrinsics[1];
  cameraMatrix.at<double>(0,2) = intrinsics[2];
  cameraMatrix.at<double>(1,2) = intrinsics[3];
  
  return 2*sqrt(summary.final_cost/problem_basic.NumResiduals());
}


Mat init_cam(Cam_Config &cam)
{
  Mat m;
  
  double f = 1.0/(1.0/cam.initial_f-1.0/cam.initial_focus);

  m = Mat::zeros(3, 3, CV_64F);
  
  m.at<double>(0,0) = f/cam.pixel_pitch;
  m.at<double>(0,2) = cam.w/2;
  
  m.at<double>(1,1) = f/cam.pixel_pitch;
  m.at<double>(1,2) = cam.h/2;
  
  m.at<double>(2,2) = 1;
  
  return m;
}


