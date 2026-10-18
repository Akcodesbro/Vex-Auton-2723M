#pragma once

//  ___            ___
// | _ \_ _ ___ __| _ \___ ___ ___
// |  _/ '_/ _ (_-<  _/ _ (_-</ -_)
// |_| |_| \___/__/_| \___/__/\___|
//

// -----------------------------------------------------------------------------
//   Simple homography-based AprilTag pose estimation
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//   Structs
// -----------------------------------------------------------------------------

struct ProsPoseResult {
  // Translation relative to camera (in meters)
  double x;
  double y;
  double z;

  // Rotation relative to camera (in degrees)
  double yaw;
  double pitch;
  double roll;

  // Validity flag
  bool is_valid;
};

// -----------------------------------------------------------------------------
//   ProsPose class
// -----------------------------------------------------------------------------

class ProsPose {
public:
  // ━━ Parameters ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //   Defaults are meant for goal tags and Ai Vision Sensor
  //
  //   fx, fy  — Focal lengths in pixels.
  //             Formula: fx = (sensor_width / 2) / tan(HFOV / 2).
  //             Default 212.3 (VEX AI Vision, 320 px, 74° HFOV).
  //
  //   cx, cy  — Principal point (optical center) in pixels.
  //             Shifts the XY origin. Wrong values cause error that
  //             grows toward image edges.
  //             Defaults 160.0, 120.0 (center of 320 × 240).
  //
  //   tag_size — Physical tag width in meters.
  //              Directly scales all translation output linearly.
  //              This is the most critical value.
  //              Default 0.018 m (18 mm).
  //
  // ━━ Output: ProsPoseResult ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  //   x, y, z  — Position in metres in the camera frame.
  //   yaw, pitch, roll  — Rotation in degrees (ZYX Euler angles).
  //
  ProsPose(double fx = 212.3, double fy = 195.82, double cx = 160.0,
           double cy = 120.0, double tag_size = 0.018);

  ProsPoseResult  estimate(double corners[4][2]);

private:
  double fx_, fy_, cx_, cy_;
  double tag_size_;
};
