#include "geometry.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <math.h>

const uint32_t imageWidth = 100;
const uint32_t imageHeight = 100;

const float near_clip = 1;
const float alpha = 45.f * M_PI / 180.f;
const float beta = 45.f * M_PI / 180.f;
const Vec2<float> canvas =
    Vec2<float>(2. * tan(alpha) * near_clip, 2. * tan(beta) * near_clip);

int main(int argc, char **argv) {

  // Start with a global point
  Vec3<float> point_global_pos = Vec3<float>(-0.98, -0.98, -3);

  // Skipping camera transformation
  Vec3<float> point_camera_pos = point_global_pos;

  // Convert to screen pos
  Vec3<float> point_screen_pos;
  point_screen_pos.x = near_clip * point_camera_pos.x / -point_camera_pos.z;
  point_screen_pos.y = near_clip * point_camera_pos.y / -point_camera_pos.z;
  point_screen_pos.z = -point_camera_pos.z;

  // Convert to NDC
  float t = canvas.y / 2.f;
  float b = -canvas.y / 2.f;
  float l = -canvas.x / 2.f;
  float r = canvas.x / 2.f;

  Vec2<float> point_ndc_pos;
  point_ndc_pos.x = (2.f * point_screen_pos.x / (r - l)) - ((r + l) / (r - l));
  point_ndc_pos.y = (2.f * point_screen_pos.y / (t - b)) - ((t + b) / (t - b));

  // Normalize to 0-1
  Vec2<float> point_norm_pos;
  point_norm_pos = (point_ndc_pos + 1.) / 2.;

  // Convert to raster
  Vec2<uint32_t> point_rast_pos;
  point_rast_pos.x = std::floor(point_norm_pos.x * imageWidth);
  point_rast_pos.y = std::floor((1 - point_norm_pos.y) * imageHeight);

  Vec3<unsigned char> *frameBuffer =
      new Vec3<unsigned char>[imageWidth * imageHeight];

  for (uint32_t i = 0; i < imageWidth * imageHeight; ++i)
    frameBuffer[i] = Vec3<unsigned char>(255);

  frameBuffer[point_rast_pos.x + point_rast_pos.y * imageWidth] =
      Vec3<unsigned char>(255, 0, 0);

  std::ofstream ofs;
  ofs.open("./output.ppm");
  ofs << "P6\n" << imageWidth << " " << imageHeight << "\n255\n";
  ofs.write((char *)frameBuffer, imageWidth * imageHeight * 3);
  ofs.close();

  delete[] frameBuffer;

  return 0;
}