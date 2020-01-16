#include "geometry.h"
#include <algorithm>
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

Vec3<float> pointGlobalToNormal(Vec3<float> point_global, float near_clip,
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

  Vec3<float> point_ndc_pos;
  point_ndc_pos.x = (2.f * point_screen_pos.x / (r - l)) - ((r + l) / (r - l));
  point_ndc_pos.y = (2.f * point_screen_pos.y / (t - b)) - ((t + b) / (t - b));
  point_ndc_pos.z = point_screen_pos.z;

  // Normalize to 0-1
  Vec3<float> point_norm_pos;
  point_norm_pos.x = (point_ndc_pos.x + 1.) / 2.;
  point_norm_pos.y = (point_ndc_pos.y + 1.) / 2.;
  point_norm_pos.z = point_ndc_pos.z;

  return point_norm_pos;
}

Vec2<uint32_t> pointNormalToRaster(Vec3<float> point_norm_pos) {
  // Convert to raster
  Vec2<uint32_t> point_rast_pos;
  point_rast_pos.x = std::floor(point_norm_pos.x * image_size.x);
  point_rast_pos.y = std::floor((1 - point_norm_pos.y) * image_size.y);
  // We don't use the z value yet

  return point_rast_pos;
}

/**
 * Returns the top left and bottom right points of the triangle's bounding box.
 */
Vec2<Vec2<float>> boundingBox(Vec3<Vec3<float>> triangle) {
  Vec2<float> top_left, bottom_right;
  top_left.x = std::min(std::min(triangle.x.x, triangle.y.x), triangle.z.x);
  bottom_right.x = std::max(std::max(triangle.x.x, triangle.y.x), triangle.z.x);
  top_left.y = std::max(std::max(triangle.x.y, triangle.y.y), triangle.z.y);
  bottom_right.y = std::min(std::min(triangle.x.y, triangle.y.y), triangle.z.y);
  return Vec2<Vec2<float>>(top_left, bottom_right);
}

int main(int argc, char **argv) {
  // Start with a global point
  Vec3<Vec3<float>> triangle = {Vec3<float>(0, 0, -1),
                                Vec3<float>(0.5f, 0.5f, -1),
                                Vec3<float>(0.5f, 0, -1)};

  Vec3<Vec3<float>> triangle_normal = {
      pointGlobalToNormal(triangle.x, near_clip, canvas_size, image_size),
      pointGlobalToNormal(triangle.y, near_clip, canvas_size, image_size),
      pointGlobalToNormal(triangle.z, near_clip, canvas_size, image_size)};

  auto bounding_box = boundingBox(triangle_normal);

  auto point_rast_pos =
      pointNormalToRaster(Vec3<float>(bounding_box.x.x, bounding_box.x.y, 0));
  auto point_rast_pos_2 =
      pointNormalToRaster(Vec3<float>(bounding_box.y.x, bounding_box.y.y, 0));

  Vec3<unsigned char> *frameBuffer =
      new Vec3<unsigned char>[image_size.x * image_size.y];

  for (uint32_t i = 0; i < image_size.x * image_size.y; ++i)
    frameBuffer[i] = Vec3<unsigned char>(255);

  frameBuffer[point_rast_pos.x + point_rast_pos.y * image_size.y] =
      Vec3<unsigned char>(255, 0, 0);

  frameBuffer[point_rast_pos_2.x + point_rast_pos_2.y * image_size.y] =
      Vec3<unsigned char>(0, 255, 0);

  std::ofstream ofs;
  ofs.open("./output.ppm");
  ofs << "P6\n" << image_size.x << " " << image_size.y << "\n255\n";
  ofs.write((char *)frameBuffer, image_size.x * image_size.y * 3);
  ofs.close();

  delete[] frameBuffer;

  return 0;
}