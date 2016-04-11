#ifndef _LUTCALIB_SETTINGS_H
#define _LUTCALIB_SETTINGS_H

struct Cam_Config {
  double pixel_pitch;
  double initial_f;
  double initial_focus;
  int w, h;
};

struct Marker_Config {
  const double marker_size;
  int grid_width;
  int grid_height;
};

struct Calib_Config {
  bool zlimit;
  double zmin; //in mm
  double zmax; //in mm
};

#endif
