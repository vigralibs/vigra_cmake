#include <hdmarker/subpattern.hpp>
#include <ucalib/ucalib.hpp>
#include <cliini/cliini.hpp>

#include <opencv2/highgui.hpp>

#include <metamat/mat.hpp>
#include <ucalib/proxy.hpp>

#include <iostream>

using namespace cv;
using namespace cliini;
using namespace hdmarker;

cliini_opt opts[] = {
  {
    "help",
    0, //argcount
    0, //argcount
    CLIINI_NONE, //type
    0, //flags
    'h'
  },
  {
    "imgs",
    1, //argcount min
    CLIINI_ARGCOUNT_ANY, //argcount max
    CLIINI_STRING, //type
    0, //flags
    'i',
    NULL,
    "read imgs and detect calibration points with HDMarkers",
    "[file1] [file2] ..."
  },
  {
    "store",
    1, //argcount min
    1, //argcount max
    CLIINI_STRING, //type
    0, //flags
    's',
    NULL,
    "store detected calibration points",
    "<file>"
  },
  {
    "load",
    1, //argcount min
    1, //argcount max
    CLIINI_STRING, //type
    0, //flags
    'l',
    NULL,
    "load stored calibration points and execute calibration",
    "<file>"
  }
};

cliini_optgroup group = {
  opts,
  NULL,
  sizeof(opts)/sizeof(*opts),
  0,
  0
};

int main(int argc, const char *argv[])
{
  Marker::init();
  
  cliargs args(argc, argv, &group);
  
  std::vector<Corner> corners_rough;
  std::vector<Corner> corners;
  
  double unit_size = 39.94; //marker size in mm
  
  MetaMat::Mat_<float> proxy({2, 33, 25, args["imgs"].count()});
  
  std::string img_f = args["imgs"].str(0);
  Mat ex_img = imread(img_f, CV_LOAD_IMAGE_GRAYSCALE);
  
  for(int i=0;i<args["imgs"].count();i++) {
    img_f = args["imgs"].str(i);
    
    std::vector<Point2f> ipoints_v;
    std::vector<Point3f> wpoints_v;
      
    if (boost::filesystem::exists(img_f+".pointcache.yaml")) {
      FileStorage fs(img_f+".pointcache.yaml", FileStorage::READ);
      fs["img_points"] >> ipoints_v;
      fs["world_points"] >> wpoints_v;
      
      printf("%d points\n", ipoints_v.size());
    }
    else {
      std::cout << img_f+".pointcache.yaml" << "does not exist!\n";
      Mat img = imread(img_f, CV_LOAD_IMAGE_GRAYSCALE);
      printf("detect img %s\n", args["imgs"].str(i).c_str());
      corners_rough.resize(0);
      Marker::detect(img, corners_rough, false, 0, 100, 3);
      Mat debug;
      corners.resize(0);
      double unit_size_res = unit_size;
      hdmarker_detect_subpattern(img, corners_rough, corners, 3, &unit_size_res, &debug, NULL, 0);
      imwrite((img_f+".detect.png").c_str(), debug);
    
      ipoints_v.resize(corners.size());
      wpoints_v.resize(corners.size());
      
      for(int ci=0;ci<corners.size();ci++) {
        ipoints_v[ci] = corners[ci].p;
        Point2f w_2d = unit_size_res*Point2f(corners[ci].id.x, corners[ci].id.y);
        wpoints_v[ci] = Point3f(w_2d.x, w_2d.y, 0);
      }
    
      std::cout << "write " << img_f+".pointcache.yaml" << "\n";
      FileStorage fs(img_f+".pointcache.yaml", FileStorage::WRITE);
      fs << "img_points" << ipoints_v;
      fs << "world_points" << wpoints_v;
    }
  
    if (boost::filesystem::exists(img_f+".proxycache.yaml")) {
      cv::Mat sub_proxy;
      
      FileStorage fs(img_f+".proxycache.yaml", FileStorage::READ);
      fs["proxy"] >> sub_proxy;
      
      sub_proxy.copyTo(cvMat(proxy.bind(3, i)));
    }
    else {
      MetaMat::Mat_<float> sub_proxy = proxy.bind(3, i);
      ucalib::proxy_backwards_pers_poly_generate<0,0>(sub_proxy, ipoints_v, wpoints_v, ex_img.size());
      
      FileStorage fs(img_f+".proxycache.yaml", FileStorage::WRITE);
      fs << "proxy" << cvMat(sub_proxy);
    }
  
  }
  
  
  ucalib::calibrate_rays(proxy, ex_img.size(), MetaMat::DimSpec(-1), ucalib::LIVE | ucalib::SHOW_TARGET);

  //TODO calibration...
  
  return EXIT_SUCCESS;
}
