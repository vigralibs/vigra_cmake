Ray based Solver
================

[TOC]


# Overview {#solve_over}

This chapter documents the options for the ray-based solver, which is invoked with \a calibrate_rays().

The ray based calibration method derives a camera model by finding ray parameters and sets of camera extrinsics (one for each camera) and view extrinsics (one for each view), so that the observed ray support points specified by the proxy, are consistent with the found ray mode, by minimizing an error metric. This metric is the least square error either in target coordinates (default) or in pixels (when using the support point Jacobians, see [below](@ref solve_invoke).

See [Camera Layouts](@ref dims) to see how the support points are represented by the multi-dimensional proxy matrix for different camera setups.

# Invokation {#solve_invoke}

There are two signatures for \a calibrate_rays(). The first provides calibration just from the proxy:
~~~~~~~~~~~~~{.cpp}
Calib* calibrate_rays(Mat_<float> &proxy, cv::Point2i img_size, const Options &opts = Options(0), const DimSpec &views_dims_start = DimSpec(-1));
~~~~~~~~~~~~~
the second makes use of the jacobian (which can optionally be calculated by \a proxy_pers_poly().
~~~~~~~~~~~~~{.cpp}
Calib* calibrate_rays(Mat_<float> &proxy, const Mat_<float> &j, cv::Point2i img_size, const Options &opts = Options(0), const DimSpec &views_dims_start =  DimSpec(-1));
~~~~~~~~~~~~~

The first option will calibrate using the target coordinate system for minimization, while the second variant uses the derivatives to calculate the error in image coordinates. As the actual measurement errors depend on the input images, the different orientation of the target for different views means error from a target which is less magnified by the camera projection become amplified in the solver (large error in target coordinates) compared to more magnified views. However in reality this effect seems to be quite small. Still, measuring the error in image coordinates results in a more meaningful error metric.

# Options {#solve_opts}

Currently all options are simple boolean flags, however in the future the ucalib::Options object will implement additional tunables (like constraining the focal length, target mesh size and resolution, outlier thresholds, etc...).

The Options object can be directly constructed from the respective bitflags, e.g:
~~~~~~~~~~~~~{.cpp}
ucalib::Calib *c = ucalib::calibrate_rays(proxy, img_size, ucalib::PLANAR | ucalib::CENTRAL);
~~~~~~~~~~~~~
 
## Visual feedback {#solve_vis}

If mm-mesh was compiled with the opengl viewer (requires glfw3 and GLEW) then live output while calibrating is possible.

- ucalib::LIVE will trigger the live display (and keep it open after solving until ESC is pressed)
- ucalib::SHOW_TARGET will show the target after calibration
- ucalib::SHOW_CENTER will show 1mm in each direction of the rays around the camera center

## Camera options {#solve_cam}

The following options specify the properties (constraints) of the used camera(s). Note that currently these properties can be set only globally for one calibration problem, not for individual cameras:

- ucalib::CENTRAL force cameras to be central (all rays intersect in a single point for each cameras)
- ucalib::PLANAR the calibration target is perfectly planar (don't solve for target geometry)

# Examples

TODO
- central, planar
- show live output and meshes
