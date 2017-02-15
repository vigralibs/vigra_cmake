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
    "store calibration",
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
    "load calibration",
    "<file>"
  },
  {
    "undistort",
    1, //argcount min
    CLIINI_ARGCOUNT_ANY, //argcount max
    CLIINI_STRING, //type
    0, //flags
    'u',
    NULL,
    "undistort image with calibration model",
    "[file1] [file2] ..."
  },
  {
    "z",
    1, //argcount min
    1, //argcount max
    CLIINI_DOUBLE, //type
    0, //flags
    'z',
    NULL,
    "z distance for undistortion",
    "<depth>"
  }
};

cliini_optgroup group = {
  opts,
  NULL,
  sizeof(opts)/sizeof(*opts),
  0,
  0
};

std::string replace_all(std::string str, const std::string& search, const std::string& replace) {
    size_t pos = 0;
    while ((pos = str.find(search, pos)) != std::string::npos) {
         str.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return str;
}

void save_mat(boost::filesystem::path path, MetaMat::Mat m, FileStorage &fs)
{
  fs << replace_all(path.generic_string(),"/","_S_") << cvMat(m);
}

void save_str(boost::filesystem::path path, const char* str, FileStorage &fs)
{
  fs << replace_all(path.generic_string(),"/","_S_") << str;
}

MetaMat::Mat load_mat(boost::filesystem::path path, FileStorage &fs)
{
  cv::Mat m;
  fs[replace_all(path.generic_string(),"/","_S_")] >> m;
  return MetaMat::Mat(m);
}

const char *load_str(boost::filesystem::path path, FileStorage &fs)
{
  std::string str;
  fs[replace_all(path.generic_string(),"/","_S_")] >> str;
  return strdup(str.c_str());
}

int main(int argc, const char *argv[])
{
  Marker::init();
  
  cliargs args(argc, argv, &group);
  
  std::vector<Corner> corners_rough;
  std::vector<Corner> corners;
  
  double unit_size = 39.94; //marker size in mm
  
  ucalib::Calib *calib = NULL;
  
  if (args["imgs"].valid()) {
    MetaMat::Mat_<float> proxy({2, 33, 25, args["imgs"].count()});
    MetaMat::Mat_<float> jacobian({2, 2, 33, 25, args["imgs"].count()});
    
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
        cv::Mat sub_j;
        
        FileStorage fs(img_f+".proxycache.yaml", FileStorage::READ);
        fs["proxy"] >> sub_proxy;
        fs["jacobian"] >> sub_j;
        
        sub_proxy.copyTo(cvMat(proxy.bind(3, i)));
        sub_j.copyTo(cvMat(jacobian.bind(4, i)));
      }
      else {
        MetaMat::Mat_<float> sub_proxy = proxy.bind(3, i);
        MetaMat::Mat_<float> sub_j = jacobian.bind(4, i);
        ucalib::proxy_pers_poly<3,3>(sub_proxy, ipoints_v, wpoints_v, ex_img.size(), 0, 0, &sub_j);
        
        FileStorage fs(img_f+".proxycache.yaml", FileStorage::WRITE);
        fs << "proxy" << cvMat(sub_proxy);
        fs << "jacobian" << cvMat(sub_j);
      }
    
    }
    
    calib = ucalib::calibrate_rays(proxy, jacobian, ex_img.size(), ucalib::LIVE | ucalib::SHOW_TARGET | ucalib::PLANAR, MetaMat::DimSpec(-1));
    

  }
  
  if (args["load"]) {
    FileStorage fs(args["load"].str(), FileStorage::READ);
    std::function<ucalib::Mat(ucalib::cpath)> _l_m = std::bind(load_mat,std::placeholders::_1, fs);
    std::function<const char*(ucalib::cpath)> _l_y = std::bind(load_str,std::placeholders::_1, fs);
    calib = ucalib::load(_l_m, _l_y);
  }
  
  if (args["store"]) {
    FileStorage fs(args["store"].str(), FileStorage::WRITE);
    std::function<void(ucalib::cpath,ucalib::Mat)> _s_m = std::bind(save_mat,std::placeholders::_1, std::placeholders::_2, fs);
    std::function<void(ucalib::cpath,const char*)> _s_y = std::bind(save_str,std::placeholders::_1, std::placeholders::_2, fs);
    calib->save(_s_m, _s_y);
  }
  
  if (calib && args["undistort"]) {    
    double z = 1000;
    
    if (args["z"])
      z = args["z"];
    
    printf("z depth : %f\n", z);
    
    //FIXME should also work with empty idx!
    calib->set_ref_cam(calib->cam());
    
    for(int i=0;i<args["undistort"].count();i++) {
      std::string img_f = args["undistort"].str(i);
      cv::Mat img = imread(img_f, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat undist = imread(img_f, CV_LOAD_IMAGE_GRAYSCALE);
      calib->rectify(img, undist, {0}, z);
      imwrite(img_f+".undist.png", undist);
    }
  }
  
  return EXIT_SUCCESS;
}
