Undistortion:


1. Input: 3d lines in space relative to camera center + respective projected image points (proxy_w X proxy_h)
  - but target might be arbitarily rotated in space
2. Fit a rectilinear projection (with rotation term to account for target orientation)
3. Calculate undistortion terms by backtracing from target pixels through fitted projection to input point in space to input 3d line to input pixel

Consider:
  - perspective normalization?
  - rectification (perspective correction) relative to 3d points in space?

Solution:
  - perspective normalization : normalize 4 img points to have distortion of zero (perspective transform) - also store projected world coordinates!
  - rectification: project reference world point into distortied and undistorted image and do perspective transform on those!