#pragma once

#include <string>

namespace d455_camera {

// Both streams use the same resolution and frame rate. Rebuild after editing.
struct CameraConfig {
  int width = 640;
  int height = 480;
  int fps = 30;
  // The robot model supplies this optical frame's transform.
  std::string color_frame_id = "d455_color_optical_frame";
};

}  // namespace d455_camera
