#include "common.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

void plane_calc_depth(Mat cameraMatrix, Mat rvec, Mat tvec, Size size, Mat &out)
{
  Mat p(3, 1, CV_64FC1);
  Mat res;
  p.at<double>(2, 0) = 1.0;
  Mat cam_i;
  double z, t3;
  Mat tvec_r;
  
  cam_i = cameraMatrix.inv();
  Mat rmat, rmat_i;
  Rodrigues(rvec, rmat);
  transpose(rmat, rmat_i);
  
  tvec_r = rmat_i*tvec;
  t3 = tvec_r.at<double>(2, 0);
  
  Mat combined = rmat*cam_i;
  
  out = Mat(size.height, size.width, CV_32FC1);
  
  for(int j=0;j<size.height;j++)
    for(int i=0;i<size.width;i++) {
      p.at<double>(0, 0) = i;
      p.at<double>(1, 0) = j;
      //res = cam_i*p;
      //res = rmat_i*res;
      res = combined*p;
      z =  t3 / res.at<double>(2, 0);
      out.at<float>(j,i) = z;
      //printf("%.1f ", z*marker_size);
      //cout << "from " << p << endl;
      //cout << " to " << z*marker_size << " " << " " << tvec << " " << res << endl;
      //printf("%f\n", res.at<double>(2, 0));
    }
}

double z_from_world(Point3f p, Mat rvec, Mat tvec)
{
  Mat rmat;
  Rodrigues(rvec, rmat);
  
  Mat p_world = Mat(p);
  p_world.convertTo(p_world, CV_64F);
  Mat p_cam_world = rmat*p_world + tvec; //point in camera coordinate system but world (markers) units
  
  return p_cam_world.at<double>(2,0);
}

double z_from_world2(Point3f p, Mat rvec, Mat tvec)
{
  Mat rmat;
  Rodrigues(rvec, rmat);
  
  return rmat.at<double>(2,0)*(p.x+p.y+p.z)+tvec.at<double>(2,0);
}

void imwritenr(const char *name, Mat &img, int nr)
{
  char buf[128];
  
  sprintf(buf, name, nr);
  
  imwrite(buf, img);
}


//for 4 parameter camera only!
Point3f unproject(Point2f p, double z, Mat &c, Mat &rvec, Mat &tvec)
{
  //invert camera Matrix (4 parameter form only!)
  Matx33d r_i;
  Rodrigues(rvec, r_i);
  r_i = r_i.t();
  Matx33d c_i = Matx33d::zeros();
  c_i(0,0) = 1.0/c.at<double>(0,0);
  c_i(1,1) = 1.0/c.at<double>(1,1);
  c_i(0,2) = - c.at<double>(0,2)/c.at<double>(0,0);
  c_i(1,2) = - c.at<double>(1,2)/c.at<double>(1,1);
  c_i(2,2) = 1.0;
  
  Matx31d p3(p.x, p.y, 1);
  p3 = c_i * p3;
  p3 = r_i * p3;
  
  Matx31d t_i;
  t_i = tvec;
  t_i = r_i*t_i;
  
  Point3f wp;
  double z_cam = (z+t_i(2,0)) / p3(2,0);
  
  wp.x = z_cam*p3(0,0)-t_i(0, 0);
  wp.y = z_cam*p3(1,0)-t_i(1, 0);
  wp.z = z;

  return wp;
}