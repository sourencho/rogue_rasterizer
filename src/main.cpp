#include "../include/geometry.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <math.h>
#include <vector>

struct Vertex {
    Vec3<float> pos;
    Vec3<float> rgb;
};

const float alpha = 45.f * M_PI / 180.f;
const float beta = 45.f * M_PI / 180.f;
const Vec3<float> camera_position = Vec3<float>(1.5, 2, 2);

const float near_clip = camera_position.z + 1;
const Vec3<float> camera_orientation = Vec3<float>(0, 0, 0);
const uint32_t image_width = 200;
const uint32_t image_height = 200;
float z_buffer[image_height][image_width];
Vec3<unsigned char> *frame_buffer = new Vec3<unsigned char>[image_width * image_height];

const uint32_t ntris = 3156;

const Vec2<float> canvas_size =
    Vec2<float>(2. * tan(alpha) * near_clip, 2. * tan(beta) * near_clip);

Vec3<float> pointGlobalToNormal(Vec3<float> point_global, float near_clip, Vec2<float> canvas_size,
                                uint32_t image_width, uint32_t image_height) {

    // translate global point via camera position
    Vec3<float> point_global_translated = point_global - camera_position;

    float sx = sin(camera_orientation.x);
    float sy = sin(camera_orientation.y);
    float sz = sin(camera_orientation.z);
    float cx = cos(camera_orientation.x);
    float cy = cos(camera_orientation.y);
    float cz = cos(camera_orientation.z);

    float dx = cy * ((sz * point_global_translated.y + (cz * point_global_translated.x))) -
               sy * point_global_translated.z;
    float dy = sx * (cy * point_global_translated.z +
                     sy * (sz * point_global_translated.y + cz * point_global_translated.x)) +
               cx * (cz * point_global_translated.y - sz * point_global_translated.x);

    float dz = cx * (cy * point_global_translated.z +
                     sy * (sz * point_global_translated.y + cz * point_global_translated.x)) -
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

    // Convert to [0,1) normal space
    Vec3<float> point_norm_pos;
    point_norm_pos.x = (point_ndc_pos.x + 1.) / 2.;
    point_norm_pos.y = (point_ndc_pos.y + 1.) / 2.;
    point_norm_pos.z = point_ndc_pos.z;

    return point_norm_pos;
}

std::pair<Vec2<uint32_t>, bool> pointNormalToRaster(Vec3<float> point_norm_pos) {

    bool out_of_bounds = false;
    if (point_norm_pos.x > 1.f || point_norm_pos.x < 0.f || point_norm_pos.y > 1.f ||
        point_norm_pos.y < 0.f) {
        out_of_bounds = true;
    }

    // Convert to raster
    Vec2<uint32_t> point_rast_pos;
    point_rast_pos.x = std::floor(point_norm_pos.x * image_width);
    point_rast_pos.y = std::floor((1 - point_norm_pos.y) * image_height);

    return std::make_pair(point_rast_pos, out_of_bounds);
}

float edgeFunction(const Vec3<float> &a, const Vec3<float> &b, const Vec3<float> &c) {
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

/**
 * Returns the pixels inside a triangle in [0, 1) normal coordinates.
 */
std::vector<Vertex> trianglePoints(Vec3<Vertex> triangle, uint32_t img_width, uint32_t img_height) {
    // Bounding box
    Vec2<float> top_left, bottom_right;
    top_left.x = std::min(std::min(triangle[0].pos.x, triangle[1].pos.x), triangle[2].pos.x);
    bottom_right.x = std::max(std::max(triangle[0].pos.x, triangle[1].pos.x), triangle[2].pos.x);
    top_left.y = std::max(std::max(triangle[0].pos.y, triangle[1].pos.y), triangle[2].pos.y);
    bottom_right.y = std::min(std::min(triangle[0].pos.y, triangle[1].pos.y), triangle[2].pos.y);

    float bounding_box_width = bottom_right.x - top_left.x;
    float bound_box_height = top_left.y - bottom_right.y;

    float pixel_width = 1.f / img_width;
    float pixel_height = 1.f / img_height;

    // Start of pixel center coordinates inside bounding box
    int start_x_floor;
    remquo(top_left.x, pixel_width, &start_x_floor);
    float start_x = start_x_floor * pixel_width + 0.5 * pixel_width;

    int start_y_floor;
    remquo(bottom_right.y, pixel_height, &start_y_floor);
    float start_y = start_y_floor * pixel_height + 0.5 * pixel_height;

    // Stores all the pixels positions inside the triangle
    std::vector<Vertex> tri_points;

    // Loop through points in bounding box points
    for (float x = start_x; x <= bottom_right.x; x += pixel_width) {
        for (float y = start_y; y <= top_left.y; y += pixel_height) {

            Vec3f AB = Vec3f(triangle[1].pos.x, triangle[1].pos.y, 0) +
                       Vec3f(-triangle[0].pos.x, -triangle[0].pos.y, 0);
            Vec3f BC = Vec3f(triangle[2].pos.x, triangle[2].pos.y, 0) +
                       Vec3f(-triangle[1].pos.x, -triangle[1].pos.y, 0);

            Vec3<float> p(x, y, triangle[0].pos.z);

            // We must rotate counter clockwise over the triangle for the edge function if the
            // normal is inversed
            bool triangle_normal_inversed = AB.crossProduct(BC).z <= 0.f;
            const Vertex &vertex_a = triangle_normal_inversed ? triangle[0] : triangle[1];
            const Vertex &vertex_b = triangle_normal_inversed ? triangle[1] : triangle[0];
            const Vertex &vertex_c = triangle[2];

            float area = edgeFunction(vertex_a.pos, vertex_b.pos, vertex_c.pos);
            float w0 = edgeFunction(vertex_b.pos, vertex_c.pos, p);
            float w1 = edgeFunction(vertex_c.pos, vertex_a.pos, p);
            float w2 = edgeFunction(vertex_a.pos, vertex_b.pos, p);

            // If the point is inside the triangle
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                // If the point is inside the screen
                if (x >= 0 && x < 1.0 && y >= 0 && y < 1.0) {
                    // Interpolate z value
                    w0 /= area;
                    w1 /= area;
                    w2 /= area;
                    float z =
                        1 / ((w0 / vertex_a.pos.z) + (w1 / vertex_b.pos.z) + (w2 / vertex_c.pos.z));
                    z *= triangle_normal_inversed ? 1 : -1;

                    // Interpolate color
                    float r = (w0 * vertex_a.rgb.x + w1 * vertex_b.rgb.x + w2 * vertex_c.rgb.x);
                    float g = (w0 * vertex_a.rgb.y + w1 * vertex_b.rgb.y + w2 * vertex_c.rgb.y);
                    float b = (w0 * vertex_a.rgb.z + w1 * vertex_b.rgb.z + w2 * vertex_c.rgb.z);

                    // If point has a smaller z (in front) than previous point in this position
                    Vec2i pixel_position((int)(y * img_height), (int)(x * img_width));
                    if (z_buffer[pixel_position.x][pixel_position.y] >= z) {
                        // Overwrite buffer and push point
                        z_buffer[pixel_position.x][pixel_position.y] = z;
                        tri_points.push_back((Vertex){.pos = p, .rgb = {r, g, b}});
                    }
                }
            }
        }
    }

    return tri_points;
}

int main(int argc, char **argv) {
    std::vector<Vec3<Vertex>> triangles;

    // Read in cow.h vertices
    // for (uint i = 0; i < ntris; ++i) {
    //     Vec3<float> &v0 = vertices[nvertices[i * 3]];
    //     Vec3<float> &v1 = vertices[nvertices[i * 3 + 1]];
    //     Vec3<float> &v2 = vertices[nvertices[i * 3 + 2]];

    //     Vertex vertex_0 =
    //         (Vertex){.pos = v0,
    //                  .rgb = {(float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
    //                          (float)rand() / (float)RAND_MAX}};
    //     Vertex vertex_1 =
    //         (Vertex){.pos = v1,
    //                  .rgb = {(float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
    //                          (float)rand() / (float)RAND_MAX}};
    //     Vertex vertex_2 =
    //         (Vertex){.pos = v2,
    //                  .rgb = {(float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
    //                          (float)rand() / (float)RAND_MAX}};

    //     triangles.push_back(Vec3<Vertex>(vertex_0, vertex_1, vertex_2));
    // }

    // Use custom triangles

    // CROSS
    // triangles.push_back({
    //     (Vertex){.pos = Vec3<float>(-0.5f, 0.f, -1.f), .rgb = Vec3<float>(1, 0, 1)},
    //     (Vertex){.pos = Vec3<float>(0, 0.5f, -1.5), .rgb = Vec3<float>(1, 1, 0)},
    //     (Vertex){.pos = Vec3<float>(0.5, 0, -2), .rgb = Vec3<float>(0, 1, 1)},
    // });
    // triangles.push_back({
    //     (Vertex){.pos = Vec3<float>(-0.5, 0, -2), .rgb = Vec3<float>(1, 0, 0)},
    //     (Vertex){.pos = Vec3<float>(0, 0.5, -1.5), .rgb = Vec3<float>(0, 0, 1)},
    //     (Vertex){.pos = Vec3<float>(0.5, 0, -1), .rgb = Vec3<float>(0, 1, 0)},
    // });

    // CUBE

    // back
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 0, -1), .rgb = Vec3<float>(1, 0, 0)},
        (Vertex){.pos = Vec3<float>(0, 1, -1), .rgb = Vec3<float>(1, 0, 0)},
        (Vertex){.pos = Vec3<float>(1, 0, -1), .rgb = Vec3<float>(1, 0, 0)},
    });
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 1, -1), .rgb = Vec3<float>(1, 0, 0)},
        (Vertex){.pos = Vec3<float>(1, 1, -1), .rgb = Vec3<float>(1, 0, 0)},
        (Vertex){.pos = Vec3<float>(1, 0, -1), .rgb = Vec3<float>(1, 0, 0)},
    });

    // left
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 1, -1), .rgb = Vec3<float>(1, 0, 1)},
        (Vertex){.pos = Vec3<float>(0, 0, -1), .rgb = Vec3<float>(1, 0, 1)},
        (Vertex){.pos = Vec3<float>(0, 0, 0), .rgb = Vec3<float>(1, 0, 1)},

    });
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 1, 0), .rgb = Vec3<float>(1, 0, 1)},
        (Vertex){.pos = Vec3<float>(0, 1, -1), .rgb = Vec3<float>(1, 0, 1)},
        (Vertex){.pos = Vec3<float>(0, 0, 0), .rgb = Vec3<float>(1, 0, 1)},

    });

    // right
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(1, 0, 0), .rgb = Vec3<float>(1, 0, 1)},
        (Vertex){.pos = Vec3<float>(1, 0, -1), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(1, 1, -1), .rgb = Vec3<float>(0, 0, 1)},
    });

    triangles.push_back({
        (Vertex){.pos = Vec3<float>(1, 1, -1), .rgb = Vec3<float>(0, 0, 1)},
        (Vertex){.pos = Vec3<float>(1, 1, 0), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(1, 0, 0), .rgb = Vec3<float>(1, 0, 1)},

    });

    // bottom
    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 0, 0), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(1, 0, -1), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(1, 0, 0), .rgb = Vec3<float>(0, 1, 1)},
    });

    triangles.push_back({
        (Vertex){.pos = Vec3<float>(0, 0, 0), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(0, 0, -1), .rgb = Vec3<float>(0, 1, 1)},
        (Vertex){.pos = Vec3<float>(1, 0, -1), .rgb = Vec3<float>(0, 1, 1)},

    });

    for (uint32_t i = 0; i < image_width * image_height; ++i)
        frame_buffer[i] = Vec3<unsigned char>(255);

    for (int i = 0; i < image_height; i++) {
        for (int j = 0; j < image_width; j++) {
            z_buffer[i][j] = std::numeric_limits<float>::infinity();
        }
    }

    for (auto triangle : triangles) {
        Vec3<Vertex> triangle_normal = {
            (Vertex){
                .pos = pointGlobalToNormal(triangle.x.pos, near_clip, canvas_size, image_width,
                                           image_height),
                triangle.x.rgb,
            },
            (Vertex){
                .pos = pointGlobalToNormal(triangle.y.pos, near_clip, canvas_size, image_width,
                                           image_height),
                .rgb = triangle.y.rgb,
            },
            (Vertex){
                .pos = pointGlobalToNormal(triangle.z.pos, near_clip, canvas_size, image_width,
                                           image_height),
                .rgb = triangle.z.rgb,
            }};

        auto tri_points = trianglePoints(triangle_normal, image_width, image_height);

        for (auto p : tri_points) {
            Vec2<uint32_t> p_r;
            bool out_of_bounds;
            std::tie(p_r, out_of_bounds) = pointNormalToRaster(p.pos);
            if (!out_of_bounds) {
                frame_buffer[p_r.x + p_r.y * image_height] =
                    Vec3<unsigned char>(255 * p.rgb.x, 255 * p.rgb.y, 255 * p.rgb.z);
            }
        }
    }

    std::ofstream ofs;
    ofs.open("./output.ppm");
    ofs << "P6\n" << image_width << " " << image_height << "\n255\n";
    ofs.write((char *)frame_buffer, image_width * image_height * 3);
    ofs.close();

    delete[] frame_buffer;

    return 0;
}