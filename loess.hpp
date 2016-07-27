#ifndef _LUTCALIB_LOESS_H
#define _LUTCALIB_LOESS_H

#include "ceres/ceres.h"
#include "ceres/types.h"
#include "opencv2/core/core.hpp"

template <int x_degree, int y_degree> struct Poly2dError {
  Poly2dError(double x, double y, double val, double w)
      : x_(x), y_(y), val_(val), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const {
    T xvars[x_degree];
    T yvar = T(1.0);
    xvars[0] = T(1.0);
    for(int i=1;i<x_degree;i++)
        xvars[i] = xvars[i-1]*T(x_);
    
    T res = T(0.0);
    for(int j=0;j<y_degree;j++) {
      for(int i=0;i<x_degree;i++) {
        res += p[j*x_degree+i]*(yvar*xvars[i]);
      }
      yvar = yvar*T(y_);
    }
        
    residuals[0] = (T(val_) - res)*T(w_);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double val, double w) {
    return (new ceres::AutoDiffCostFunction<Poly2dError, 1, x_degree*y_degree>(
                new Poly2dError(x, y, val, w)));
  }

  double x_,y_,val_, w_;
};

template <int x_degree, int y_degree> struct PolyPers2dError {
  PolyPers2dError(double x, double y, double valx, double valy, double w)
      : x_(x), y_(y), val_x_(valx), val_y_(valy), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const {
    T warped[3];
    warped[0] = T(x_)*p[0] + T(y_)*p[1] + p[2];
    warped[1] = T(x_)*p[3] + T(y_)*p[4] + p[5];
    warped[2] = T(x_)*p[6] + T(y_)*p[7] + p[8];
    warped[0] /= warped[2];
    warped[1] /= warped[2];

    T xvars[x_degree];
    T yvar = T(1.0);
    xvars[0] = T(1.0);
    for(int i=1;i<x_degree;i++)
      xvars[i] = xvars[i-1]*warped[0];
    
    T res_x = warped[0];
    T res_y = warped[1];
    for(int j=0;j<y_degree;j++) {
      for(int i=0;i<x_degree;i++) {
        res_x += p[9+j*x_degree+i]*(yvar*xvars[i]);
        res_y += p[9+j*x_degree+i+x_degree*y_degree]*(yvar*xvars[i]);
      }
      yvar = yvar*warped[1];
    }
    
    residuals[0] = sqrt(abs(T(val_x_) - res_x)*T(w_)+1e-18);
    residuals[1] = sqrt(abs(T(val_y_) - res_y)*T(w_)+1e-18);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double valx, double valy, double w) {
    return (new ceres::AutoDiffCostFunction<PolyPers2dError, 2, 9+2*x_degree*y_degree>(
                new PolyPers2dError(x, y, valx, valy, w)));
  }

  double x_,y_,val_x_,val_y_, w_;
};


template <int x_degree, int y_degree> struct Pers2dError {
  Pers2dError(double x, double y, double valx, double valy, double w)
      : x_(x), y_(y), val_x_(valx), val_y_(valy), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const {
    T warped[3];
    warped[0] = T(x_)*p[0] + T(y_)*p[1] + p[2];
    warped[1] = T(x_)*p[3] + T(y_)*p[4] + p[5];
    warped[2] = T(x_)*p[6] + T(y_)*p[7] + p[8];
    warped[0] /= warped[2];
    warped[1] /= warped[2];
    
    T res_x = warped[0];
    T res_y = warped[1];
    
    residuals[0] = sqrt(abs(T(val_x_) - res_x)*T(w_)+1e-18);
    residuals[1] = sqrt(abs(T(val_y_) - res_y)*T(w_)+1e-18);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double valx, double valy, double w) {
    return (new ceres::AutoDiffCostFunction<Pers2dError, 2, 9>(
                new Pers2dError(x, y, valx, valy, w)));
  }

  double x_,y_,val_x_,val_y_, w_;
};


template <int x_degree, int y_degree> struct PolyPers2dErrorTriangle {
  PolyPers2dErrorTriangle(double x, double y, double valx, double valy, double w)
      : x_(x), y_(y), val_x_(valx), val_y_(valy), w_(w) {}
      
  template <typename T>
  bool operator()(const T* const p,
                  T* residuals) const {
    T warped[3];
    warped[0] = T(x_)*p[0] + T(y_)*p[1] + p[2];
    warped[1] = T(x_)*p[3] + T(y_)*p[4] + p[5];
    warped[2] = T(x_)*p[6] + T(y_)*p[7] + p[8];
    warped[0] /= warped[2];
    warped[1] /= warped[2];

    T xvars[x_degree];
    T yvar = T(1.0);
    xvars[0] = T(1.0);
    for(int i=1;i<x_degree;i++)
      xvars[i] = xvars[i-1]*warped[0];
    
    T res_x = warped[0];
    T res_y = warped[1];
    for(int j=0;j<y_degree;j++) {
      for(int i=0;i<x_degree;i++) {
        if ((x_degree+y_degree) > std::max(x_degree,y_degree))
          continue;
        res_x += p[9+j*x_degree+i]*(yvar*xvars[i]);
        res_y += p[9+j*x_degree+i+x_degree*y_degree]*(yvar*xvars[i]);
      }
      yvar = yvar*warped[1];
    }
    
    residuals[0] = sqrt(abs(T(val_x_) - res_x)*T(w_)+1e-18);
    residuals[1] = sqrt(abs(T(val_y_) - res_y)*T(w_)+1e-18);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double x, double y, double valx, double valy, double w) {
    return (new ceres::AutoDiffCostFunction<PolyPers2dErrorTriangle, 2, 9+2*x_degree*y_degree>(
                new PolyPers2dErrorTriangle(x, y, valx, valy, w)));
  }

  double x_,y_,val_x_,val_y_, w_;
};

//TODO ignore residuals after below a certine weight!
//NOTE: z coordinate of wps is ignored (assumed to be constant - e.g. flat target)
template<int x_degree, int y_degree> double fit_2d_poly_2d(std::vector<cv::Point2f> &ips, std::vector<cv::Point3f> &wps, cv::Point2f center, double *coeffs, double sigma, int *count = NULL)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 2000;
  options.num_threads = 1;
  //options.minimizer_progress_to_stdout = true;
  //options.trust_region_strategy_type = ceres::DOGLEG;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.logging_type = ceres::SILENT;
  
  double w_sum = 0.0;
  
  cv::Point2f wc(0, 0);
  

  coeffs[0] = wps[0].x;
  for(int i=1;i<x_degree*y_degree;i++)
    coeffs[i] = 0.0;
  coeffs[x_degree*y_degree] = wps[0].y;
  for(int i=1;i<x_degree*y_degree;i++)
    coeffs[i+x_degree*y_degree] = 0.0;
  
  ceres::Problem problem_x;
  ceres::Problem problem_y;
  if (count)
    (*count) = 0;
  for(int i=0;i<ips.size();i++) {
      cv::Point2f ip = ips[i]-center;
      double w = exp(-(ip.x*ip.x+ip.y*ip.y)/(2.0*sigma*sigma));
      if (w <= 0.05)
        continue;
      if (count)
        (*count)++;
      w_sum += w;
      wc += w*ip;
      w = sqrt(w); //root to get w after squaring by ceres!
        ceres::CostFunction* cost_function =
            Poly2dError<x_degree,y_degree>::Create(ip.x, ip.y,wps[i].x, w);
            
        problem_x.AddResidualBlock(cost_function,
                                 NULL,
                                coeffs);
        
        cost_function =
            Poly2dError<x_degree,y_degree>::Create(ip.x, ip.y,wps[i].y, w);
            
        problem_y.AddResidualBlock(cost_function,
                                 NULL,
                                coeffs+x_degree*y_degree);
      }
      
  if (norm(wc*(1.0/w_sum)) >= 5.0)
    return std::numeric_limits<double>::quiet_NaN();  
  
  ceres::Solver::Summary summary_x;
  ceres::Solver::Summary summary_y;
  ceres::Solve(options, &problem_x, &summary_x);
  ceres::Solve(options, &problem_y, &summary_y);
 
  //std::cout << summary_x.FullReport() << "\n";
  //std::cout << summary_y.FullReport() << "\n";
  
  if (summary_x.termination_type == ceres::TerminationType::NO_CONVERGENCE || summary_y.termination_type == ceres::TerminationType::NO_CONVERGENCE)
    return std::numeric_limits<double>::quiet_NaN();  
  
  return sqrt((summary_x.final_cost+summary_y.final_cost)/w_sum);
}

//TODO ignore residuals after below a certine weight!
//NOTE: z coordinate of wps is ignored (assumed to be constant - e.g. flat target)
template<int x_degree, int y_degree> double fit_2d_pers_poly_2d(std::vector<cv::Point2f> &ips, std::vector<cv::Point3f> &wps, cv::Point2f center, double *coeffs, double sigma, int *count = NULL)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 2000;
  //options.num_threads = 8;
  //options.num_linear_solver_threads = 8;
  //options.minimizer_progress_to_stdout = true;
  //options.trust_region_strategy_type = ceres::DOGLEG;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  
  double w_sum = 0.0;
  
  cv::Point2f wc(0, 0);
  
  for(int i=0;i<9;i++)
    if (i % 4 == 0)
      coeffs[i] = 1.0;
    else
      coeffs[i] = 0.0;
  coeffs[2] = wps[0].x;
  coeffs[5] = wps[0].y;
    
  coeffs[9+0] = 1.0;
  for(int i=1;i<x_degree*y_degree;i++)
    coeffs[9+i] = 0.0;
  coeffs[9+x_degree*y_degree] = 1.0;
  for(int i=1;i<x_degree*y_degree;i++)
    coeffs[9+i+x_degree*y_degree] = 0.0;
  
  ceres::Problem problem;
  ceres::Problem problem_pers;
  if (count)
    (*count) = 0;
  for(int i=0;i<ips.size();i++) {
      cv::Point2f ip = (ips[i]-center);
      double w = exp(-(ip.x*ip.x+ip.y*ip.y)/(2.0*sigma*sigma));
      if (w <= 0.05)
        continue;
      if (count)
        (*count)++;
      w_sum += w;
      wc += w*ip;
        ceres::CostFunction* cost_function =
            PolyPers2dError<x_degree,y_degree>::Create(ip.x, ip.y, wps[i].x, wps[i].y, w);
            
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                coeffs);
        
        
        ceres::CostFunction* cost_function_pers =
            Pers2dError<x_degree,y_degree>::Create(ip.x, ip.y, wps[i].x, wps[i].y, w);
            
        problem_pers.AddResidualBlock(cost_function_pers,
                                 NULL,
                                coeffs);
      }
      
  if (norm(wc*(1.0/w_sum)) >= 10.0)
    return std::numeric_limits<double>::quiet_NaN();  
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_pers, &summary);
  //std::cout << summary.FullReport() << "\n";
  ceres::Solve(options, &problem, &summary);
  
  if (summary.termination_type == ceres::TerminationType::NO_CONVERGENCE)
    return std::numeric_limits<double>::quiet_NaN();  
 
  //std::cout << summary.FullReport() << "\n";

  return sqrt((summary.final_cost)/w_sum);
}

template<int x_degree, int y_degree> cv::Point2f eval_2d_poly_2d(cv::Point2f p, double *coeffs)
{
  double xvars[x_degree];
  double yvar = 1.0;
  xvars[0] = 1.0;
  for(int i=1;i<x_degree;i++)
    xvars[i] = xvars[i-1]*p.x;
  
  double res_x = 0.0;
  double res_y = 0.0;
  for(int j=0;j<y_degree;j++) {
    for(int i=0;i<x_degree;i++) {
      res_x += coeffs[j*x_degree+i]*(yvar*xvars[i]);
      res_y += coeffs[j*x_degree+i+x_degree*y_degree]*(yvar*xvars[i]);
    }
    yvar = yvar*p.y;
  }
      
  return cv::Point2f(res_x, res_y);
}

template<int x_degree, int y_degree> cv::Point2f eval_2d_pers_poly_2d(cv::Point2f p, double *coeffs, double scale = 1.0)
{
  double warped[3];
  p *= scale;
  warped[0] = p.x*coeffs[0] + p.y*coeffs[1] + coeffs[2];
  warped[1] = p.x*coeffs[3] + p.y*coeffs[4] + coeffs[5];
  warped[2] = p.x*coeffs[6] + p.y*coeffs[7] + coeffs[8];
  warped[0] /= warped[2];
  warped[1] /= warped[2];

  double xvars[x_degree];
  double yvar = 1.0;
  xvars[0] = 1.0;
  for(int i=1;i<x_degree;i++)
    xvars[i] = xvars[i-1]*warped[0];
  
  double res_x = warped[0];
  double res_y = warped[1];
  for(int j=0;j<y_degree;j++) {
    for(int i=0;i<x_degree;i++) {
      res_x += coeffs[9+j*x_degree+i]*(yvar*xvars[i]);
      res_y += coeffs[9+j*x_degree+i+x_degree*y_degree]*(yvar*xvars[i]);
    }
    yvar = yvar*warped[1];
  }
      
  return cv::Point2f(res_x, res_y);
}

template<int x_degree, int y_degree> cv::Point2f eval_2d_pers_poly_2d_tria(cv::Point2f p, double *coeffs)
{
  double warped[3];
  warped[0] = p.x*coeffs[0] + p.y*coeffs[1] + coeffs[2];
  warped[1] = p.x*coeffs[3] + p.y*coeffs[4] + coeffs[5];
  warped[2] = p.x*coeffs[6] + p.y*coeffs[7] + coeffs[8];
  warped[0] /= warped[2];
  warped[1] /= warped[2];

  double xvars[x_degree];
  double yvar = 1.0;
  xvars[0] = 1.0;
  for(int i=1;i<x_degree;i++)
    xvars[i] = xvars[i-1]*warped[0];
  
  double res_x = warped[0];
  double res_y = warped[1];
  for(int j=0;j<y_degree;j++) {
    for(int i=0;i<x_degree;i++) {
        if ((x_degree+y_degree) > std::max(x_degree,y_degree))
          continue;
      res_x += coeffs[9+j*x_degree+i]*(yvar*xvars[i]);
      res_y += coeffs[9+j*x_degree+i+x_degree*y_degree]*(yvar*xvars[i]);
    }
    yvar = yvar*warped[1];
  }
      
  return cv::Point2f(res_x, res_y);
}
#endif
