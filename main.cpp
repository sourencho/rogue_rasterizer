#include "geometry.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <math.h>

const float alpha = 45.f * M_PI / 180.f;
const float beta = 45.f * M_PI / 180.f;
const Vec3<float> camera_position = Vec3<float>(0, 0, 1);
const float near_clip = abs(camera_position.z);
const Vec3<float> camera_orientation = Vec3<float>(0, 0, 0);
const Vec2<uint32_t> image_size = Vec2<uint32_t>(100, 100);
const Vec2<float> canvas_size =
    Vec2<float>(2. * tan(alpha) * near_clip, 2. * tan(beta) * near_clip);

Vec2<uint32_t> pointGlobalToRaster(Vec3<float> point_global, float near_clip,
                                   Vec2<float> canvas_size,
                                   Vec2<uint32_t> image_size) {

  // translate global point via camera position
  Vec3<float> point_global_translated = point_global - camera_position;

  float sx = sin(camera_orientation.x);
  float sy = sin(camera_orientation.y);
  float sz = sin(camera_orientation.z);
  float cx = cos(camera_orientation.x);
  float cy = cos(camera_orientation.y);
  float cz = cos(camera_orientation.z);

  float dx = cy * ((sz * point_global_translated.y +
                    (cz * point_global_translated.x))) -
             sy * point_global_translated.z;
  float dy =
      sx * (cy * point_global_translated.z +
            sy * (sz * point_global_translated.y +
                  cz * point_global_translated.x)) +
      cx * (cz * point_global_translated.y - sz * point_global_translated.x);

  float dz =
      cx * (cy * point_global_translated.z +
            sy * (sz * point_global_translated.y +
                  cz * point_global_translated.x)) -
      sx * (cz * point_global_translated.y - sz * point_global_translated.x);

  // Skipping camera transformation
  Vec3<float> point_camera_pos = Vec3<float>(dx, dy, dz);

  // Convert to screen pos
  Vec3<float> point_screen_pos;
  point_screen_pos.x = near_clip * point_camera_pos.x / -point_camera_pos.z;
  point_screen_pos.y = near_clip * point_camera_pos.y / -point_camera_pos.z;
  point_screen_pos.z = -point_camera_pos.z;

  // Convert to NDC
  float t = canvas_size.y / 2.f;
  float b = -canvas_size.y / 2.f;
  float l = -canvas_size.x / 2.f;
  float r = canvas_size.x / 2.f;

  Vec2<float> point_ndc_pos;
  point_ndc_pos.x = (2.f * point_screen_pos.x / (r - l)) - ((r + l) / (r - l));
  point_ndc_pos.y = (2.f * point_screen_pos.y / (t - b)) - ((t + b) / (t - b));

  // Normalize to 0-1
  Vec2<float> point_norm_pos;
  point_norm_pos = (point_ndc_pos + 1.) / 2.;

  // Convert to raster
  Vec2<uint32_t> point_rast_pos;
  point_rast_pos.x = std::floor(point_norm_pos.x * image_size.x);
  point_rast_pos.y = std::floor((1 - point_norm_pos.y) * image_size.y);

  return point_rast_pos;
}

int main(int argc, char **argv) {
  // Start with a global point
  Vec3<float> point_global = Vec3<float>(0, 0, -1);
  Vec3<float> point_global_2 = Vec3<float>(0.5f, 0.5f, -1);
  Vec3<float> point_global_3 = Vec3<float>(0.5f, 0, -1);

  Vec2<uint32_t> point_rast_pos =
      pointGlobalToRaster(point_global, near_clip, canvas_size, image_size);

  Vec2<uint32_t> point_rast_pos_2 =
      pointGlobalToRaster(point_global_2, near_clip, canvas_size, image_size);

  Vec2<uint32_t> point_rast_pos_3 =
      pointGlobalToRaster(point_global_3, near_clip, canvas_size, image_size);

  Vec3<unsigned char> *frameBuffer =
      new Vec3<unsigned char>[image_size.x * image_size.y];

  for (uint32_t i = 0; i < image_size.x * image_size.y; ++i)
    frameBuffer[i] = Vec3<unsigned char>(255);

  frameBuffer[point_rast_pos.x + point_rast_pos.y * image_size.y] =
      Vec3<unsigned char>(255, 0, 0);

  frameBuffer[point_rast_pos_2.x + point_rast_pos_2.y * image_size.y] =
      Vec3<unsigned char>(0, 255, 0);

  frameBuffer[point_rast_pos_3.x + point_rast_pos_3.y * image_size.y] =
      Vec3<unsigned char>(0, 0, 255);

  std::ofstream ofs;
  ofs.open("./output.ppm");
  ofs << "P6\n" << image_size.x << " " << image_size.y << "\n255\n";
  ofs.write((char *)frameBuffer, image_size.x * image_size.y * 3);
  ofs.close();

  delete[] frameBuffer;

  return 0;
}