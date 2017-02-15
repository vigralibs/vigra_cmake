Basic Calibration Walktrough (API)
================

[TOC]


This Chapter gives a complete walktrough of the API for a simple calibration problem, calibrating and undistorting grayscale images from a single bw camera. Wherever possible we use the most simple route, for more details and possibilities follow the more detailed chapters linked from the overview or the respective sections.

For an instruction on how to achieve good calibration from a user perspective see [calibration tutorial](@ref calib_howto.md)

# Overview {#wt_over}

The full calibration workflow from recorded calibration images to the undistortion of a distorted image consists of the following steps:
1. [Marker Detection](@ref wt_detection) (*[chapter](@ref external/hdmarker/doc/detection.md)*)
2. [Proxy Generation](@ref wt_proxy) (*[chapter](@ref doc/proxy.md)*)
3. [Solving Calibration Problem](@ref wt_calib) (*[chapter](@ref doc/solve.md)*)
4. [Selecting a Reference Camera](@ref wt_ref_cam) (*[chapter: Rectification](@ref doc/rectification.md)*)
5. [Perform Undistortion](@ref wt_undist) (*[chapter](@ref doc/undistortion.md)*)

The following sections give a brief description of these steps as well as an example for the simplest case. In addition pointers to more details and the respective documentation are provided. 

## Marker Detection {#wt_detection}

The detection of the calibration pattern consists of two steps, marker detection and fractal refinement. Those are implemented by the hdmarker library, which is included with:

~~~~~~~~~~~~~{.cpp}
//for marker detection
#include <hdmarker/hdmarker.hpp>

//for fractal refinement
#include <hdmarker/subpattern.hpp>
~~~~~~~~~~~~~

The marker api is contained in the hdmarker namespace and the library needs to be initialized before usage:
~~~~~~~~~~~~~{.cpp}
using namespace hdmarker;

Marker::init();
~~~~~~~~~~~~~

Note that the APIs make use of opencv structures and conventions, hence we store detected world/object matches in std::vector<cv::Point3f> and std::vector<cv::Point2f>.

~~~~~~~~~~~~~{.cpp}
cv::Mat img = ...
std::vector<hdmarker::Corner> corners_rough, corners;

//detect markers
Marker::detect(img, corners_rough);

double marker_size = 40; //use marker size of 40mm
double multiplier = marker_size;

//fractal refinement up to 3 layers
refine_recursive(img, corners_rough, corners, 3, &multiplier);

ipoints.resize(corners.size());
wpoints.resize(corners.size());
        
for(int i=0;i<corners.size();i++) {
  ipoints[i] = corners[i].p;
  wpoints_v[ci] = Point3f(multiplier*corners[ci].id.x, multiplier*corners[ci].id.y, 0);
}
~~~~~~~~~~~~~

This above code first detects the calibration markers from a grayscale input image:
~~~~~~~~~~~~~{.cpp}
Marker::detect(img, corners_rough);
~~~~~~~~~~~~~

and then performs fractal refinement:
~~~~~~~~~~~~~{.cpp}
refine_recursive(img, corners_rough, corners, 3, &multiplier);
~~~~~~~~~~~~~
The refinement is performed with *up to* three recursion steps. The multiplier variable is modified accordingly, e.g. if no refinement is performed it will keep the input value, at each refinement step it is multiplied by 1/5. This means that the true target coordinate can be calculated with the multiplier if the true marker size is known:
~~~~~~~~~~~~~{.cpp}
for(int i=0;i<corners.size();i++) {
  ipoints[i] = corners[i].p;
  wpoints[i] = Point3f(multiplier*corners[i].id.x, multiplier*corners[i].id.y, 0);
}
~~~~~~~~~~~~~

## Proxy Generation {#wt_proxy}

The calibration library makes use of the MetaMat library:

~~~~~~~~~~~~~{.cpp}
#include <metamat/mat.hpp>
#include <ucalib/proxy.hpp>
~~~~~~~~~~~~~

The calibration proxy consists of the ray support points, generated using a local fit around the desired image position. For the calibration the proxy is written into a multi-dimensional matrix, where the first dimension is the (2d) target position of the support point, the following two are the x and y (image) position. The last dimension should contain the different calibration views. Additional dimensions inbetween can contain multiple cameras (for camera arrays), different motion directions of a camera (translation stage) or combinations of both (e.g. array on a translation stage).

In this case we use a single camera and hence the matrix id 4D:
~~~~~~~~~~~~~{.cpp}
int imgs = 8; //use 8 calibration images
MetaMat::Mat_<float> proxy({2, 33, 25, 8});
~~~~~~~~~~~~~
This allocates a matrix which will store the support points of 8 calibration views and 33x25 support points per view.

The proxy is generated image by image from OpenCV style std::vector<std::vector<cv::Point2f>> calibration points:
~~~~~~~~~~~~~{.cpp}
MetaMat::Mat_<float> sub_proxy = proxy.bind(3, currimg);
ucalib::proxy_backwards_pers_poly_generate(sub_proxy, ipoints, wpoints, img_size);
~~~~~~~~~~~~~
In the above code the MetaMat::bind() method creates a sub-matrix from proxy by binding dimension 3 to the respective image. This means that proxy_backwards_pers_poly_generate() only writes into the sub-matrix associated with the specified image and world points.

## Calibration {#wt_calib}

After filling the calibration proxy, the actual calibration can be executed:
~~~~~~~~~~~~~{.cpp}
ucalib::Calib *c = ucalib::calibrate_rays(proxy, img_size);
~~~~~~~~~~~~~
This call assumes that the last dimension containes the calibration views. If we want to specify some extra options (for example ucalib::CENTRAL for a central camera), then we can specify extra flags like this:
~~~~~~~~~~~~~{.cpp}
ucalib::Calib *c = ucalib::calibrate_rays(proxy, img_size, ucalib::PLANAR | ucalib::CENTRAL);
~~~~~~~~~~~~~

## Reference Camera {#wt_ref_cam}

After calibration the ucalib::Calib object contains all calibration properties. To perform undistortion or rectification we need to select a reference model camera which should be emulated. For undistortion this can simply be the pinhole version of the calibrated camera which can be retrieved by:
~~~~~~~~~~~~~{.cpp}
ucalib::Cam *cam = calib->cam();
~~~~~~~~~~~~~
This constructs a pinhole camera object which represents camera_0 at view_0 (ucalib::calib::cam() defaults to 0/0 for the camera/view).

Now we set this camera as the reference camera for further processing:
~~~~~~~~~~~~~{.cpp}
c->set_ref_cam(cam);
~~~~~~~~~~~~~

## Undistortion {#wt_undist}

There is only a single method for undistortion and rectification: ucalib::Calib::rectify(). 
The rectify function warps an input image from one of the calibrated cameras/viewpoints, so that it is identical to what the reference camera would have recorded *for one specific depth plane*. Hence rectify() requires both the camera/view index (default is 0) and the depth (default is 0 which means infinity). Because for undistortion the depth is irrelevant and because there is only one camera the default is fine:
~~~~~~~~~~~~~{.cpp}
cv::Mat img = ...
cv::Mat undist = ...
calib->rectify(img, undist);
~~~~~~~~~~~~~
