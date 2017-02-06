#include <hdmarker/subpattern.hpp>
#include <ucalib/ucalib.hpp>
#include <cliini/cliini.hpp>

#include <opencv2/highgui.hpp>

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
  double unit_size_res = unit_size;
  
  for(int i=0;i<args["imgs"].count();i++) {
    std::string img_f = args["imgs"].str(i);
    
    std::vector<Point2f> ipoints_v(corners.size());
    std::vector<Point2f> wpoints_v(corners.size());
      
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
      hdmarker_detect_subpattern(img, corners_rough, corners, 3, &unit_size_res, &debug, NULL, 0, {cv::Rect(7,20,2,2)});
      imwrite((img_f+".detect.png").c_str(), debug);
    
      ipoints_v.resize(corners.size());
      wpoints_v.resize(corners.size());
      
      for(int ci=0;ci<corners.size();ci++) {
        ipoints_v[ci] = corners[ci].p;
        Point2f w_2d = unit_size_res*Point2f(corners[ci].id.x, corners[ci].id.y);
        wpoints_v[ci] = Point2f(w_2d.x, w_2d.y);
      }
    
      std::cout << "write " << img_f+".pointcache.yaml" << "\n";
      FileStorage fs(img_f+".pointcache.yaml", FileStorage::WRITE);
      fs << "img_points" << ipoints_v;
      fs << "world_points" << wpoints_v;
    }
  }
  

  //TODO calibration...
  
  return EXIT_SUCCESS;
}
