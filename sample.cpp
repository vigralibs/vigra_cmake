#include <hdmarker/subpattern.hpp>
#include <ucalib/ucalib.hpp>
#include <cliini/cliini.hpp>

#include <opencv2/highgui.hpp>

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
  
  cliargs args(argc, argv, NULL);
  
  std::vector<Corner> corners_rough;
  std::vector<Corner> corners;
  
  double unit_size = 39.94; //marker size in mm
  double unit_size_res = unit_size;
  
  for(int i=0;i<args["imgs"].count();i++) {
    Mat img = imread(args["imgs"].str(i));
    Marker::detect(img, corners_rough);
    hdmarker_detect_subpattern(img, corners_rough, corners, 3, &unit_size_res, NULL, NULL, 0);
  }
  
  std::vector<Point2f> ipoints_v(corners.size());
  std::vector<Point2f> wpoints_v(corners.size());
  
  for(int ci=0;ci<corners.size();ci++) {
    ipoints_v[ci] = corners[ci].p;
    Point2f w_2d = unit_size_res*Point2f(corners[ci].id.x, corners[ci].id.y);
    wpoints_v[ci] = Point2f(w_2d.x, w_2d.y);
  }
  
  if (args["store"].valid())
  {
    FileStorage fs(args["store"].str(), FileStorage::WRITE);

    fs << "img_points" << ipoints_v;
    fs << "world_points" << wpoints_v;
  }
  if (args["load"].valid())
  {
    FileStorage fs(args["load"].str(), FileStorage::READ);
    
    fs["img_points"] >> ipoints_v;
    fs["world_points"] >> wpoints_v;
  }

  //TODO calibration...
  
  return EXIT_SUCCESS;
}
