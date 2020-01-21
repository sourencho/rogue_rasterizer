#include "geometry.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <math.h>
#include <vector>

const float alpha = 45.f * M_PI / 180.f;
const float beta = 45.f * M_PI / 180.f;
const Vec3<float> camera_position = Vec3<float>(0.4, 0, 1);
const float near_clip = abs(camera_position.z);
const Vec3<float> camera_orientation = Vec3<float>(0, 0, 0);
const Vec2<uint32_t> image_size = Vec2<uint32_t>(140, 140);
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

std::pair<Vec2<uint32_t>, bool>
pointNormalToRaster(Vec3<float> point_norm_pos) {

  bool out_of_bounds = false;
  if (point_norm_pos.x > 1.f || point_norm_pos.x < 0.f ||
      point_norm_pos.y > 1.f || point_norm_pos.y < 0.f) {
    out_of_bounds = true;
  }

  // Convert to raster
  Vec2<uint32_t> point_rast_pos;
  point_rast_pos.x = std::floor(point_norm_pos.x * image_size.x);
  point_rast_pos.y = std::floor((1 - point_norm_pos.y) * image_size.y);
  // We don't use the z value yet

  return std::make_pair(point_rast_pos, out_of_bounds);
}

bool edgeFunction(const Vec3<float> &a, const Vec3<float> &b,
                  const Vec3<float> &c) {
  return ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x) >= 0);
}

/**
 * Returns the top left and bottom right points of the triangle's bounding box.
 */
std::vector<Vec3<float>> trianglePoints(Vec3<Vec3<float>> triangle,
                                        uint32_t img_width,
                                        uint32_t img_height) {
  // Bounding box
  Vec2<float> top_left, bottom_right;
  top_left.x = std::min(std::min(triangle.x.x, triangle.y.x), triangle.z.x);
  bottom_right.x = std::max(std::max(triangle.x.x, triangle.y.x), triangle.z.x);
  top_left.y = std::max(std::max(triangle.x.y, triangle.y.y), triangle.z.y);
  bottom_right.y = std::min(std::min(triangle.x.y, triangle.y.y), triangle.z.y);

  float bounding_box_width = bottom_right.x - top_left.x;
  float bound_box_height = top_left.y - bottom_right.y;

  float pixel_width = 1.f / img_width;
  float pixel_height = 1.f / img_height;

  int start_x_floor;
  remquo(top_left.x, pixel_width, &start_x_floor);
  float start_x = start_x_floor * pixel_width + 0.5 * pixel_width;

  int start_y_floor;
  remquo(bottom_right.y, pixel_height, &start_y_floor);
  float start_y = start_y_floor * pixel_height + 0.5 * pixel_height;

  // Pixel center points inside triangle
  std::vector<Vec3<float>> tri_points;

  // Loop through points in bounding box points
  for (float x = start_x; x <= bottom_right.x; x += pixel_width) {
    for (float y = start_y; y <= top_left.y; y += pixel_height) {
      // checking if point is inside traingle
      Vec3<float> p(x, y, 0);
      bool inside = true;
      inside &= edgeFunction(triangle.x, triangle.y, p);
      inside &= edgeFunction(triangle.y, triangle.z, p);
      inside &= edgeFunction(triangle.z, triangle.x, p);
      if (inside) {
        tri_points.push_back(p);
      }
    }
  }

  return tri_points;
}

int main(int argc, char **argv) {
  // Start with a global point
  Vec3<Vec3<float>> triangle = {Vec3<float>(0, 0, -1000),
                                Vec3<float>(0.5f, 0.5f, -1),
                                Vec3<float>(0.5f, 0, -1)};

  Vec3<Vec3<float>> triangle_normal = {
      pointGlobalToNormal(triangle.x, near_clip, canvas_size, image_size),
      pointGlobalToNormal(triangle.y, near_clip, canvas_size, image_size),
      pointGlobalToNormal(triangle.z, near_clip, canvas_size, image_size)};

  auto tri_points = trianglePoints(triangle_normal, image_size.x, image_size.y);

  Vec3<unsigned char> *frameBuffer =
      new Vec3<unsigned char>[image_size.x * image_size.y];

  for (uint32_t i = 0; i < image_size.x * image_size.y; ++i)
    frameBuffer[i] = Vec3<unsigned char>(255);

  for (auto p : tri_points) {
    Vec2<uint32_t> p_r;
    bool out_of_bounds;
    std::tie(p_r, out_of_bounds) = pointNormalToRaster(p);
    if (!out_of_bounds) {
      frameBuffer[p_r.x + p_r.y * image_size.y] =
          Vec3<unsigned char>(255, 0, 0);
    }
  }

  std::ofstream ofs;
  ofs.open("./output.ppm");
  ofs << "P6\n" << image_size.x << " " << image_size.y << "\n255\n";
  ofs.write((char *)frameBuffer, image_size.x * image_size.y * 3);
  ofs.close();

  delete[] frameBuffer;

  return 0;
}