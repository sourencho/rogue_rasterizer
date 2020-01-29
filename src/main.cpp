#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <math.h>
#include <thread>
#include <vector>

#include "../include/geometry.h"
#include "../include/loader.h"
#include "../include/utils.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_WHITE "\x1b[37m"
#define ANSI_COLOR_BLACK "\x1b[30m"

#define ANSI_COLOR_RESET "\x1b[0m"

std::vector<std::pair<Vec3i, std::string>> terminal_colors = {
    std::make_pair(Vec3i(194, 54, 33), std::string(ANSI_COLOR_RED)),
    std::make_pair(Vec3i(37, 188, 36), std::string(ANSI_COLOR_GREEN)),
    std::make_pair(Vec3i(173, 173, 39), std::string(ANSI_COLOR_YELLOW)),
    std::make_pair(Vec3i(73, 46, 225), std::string(ANSI_COLOR_BLUE)),
    std::make_pair(Vec3i(211, 56, 211), std::string(ANSI_COLOR_MAGENTA)),
    std::make_pair(Vec3i(51, 187, 200), std::string(ANSI_COLOR_CYAN)),
    std::make_pair(Vec3i(203, 204, 205), std::string(ANSI_COLOR_WHITE)),
    std::make_pair(Vec3i(0, 0, 0), std::string(ANSI_COLOR_BLACK)),
};

struct Vertex {
    Vec3f pos;
    Vec3f rgb;
};

const float alpha = 45.f * M_PI / 180.f;
const float beta = 45.f * M_PI / 180.f;

Vec3f camera_position = Vec3f(0, 0, 0);
Vec3f camera_orientation = Vec3f(0, 0, 0);

const float near_clip = 1;
const uint32_t image_width = 80;
const uint32_t image_height = 80;
float z_buffer[image_height][image_width];
Vec3<unsigned char> *frame_buffer = new Vec3<unsigned char>[image_width * image_height];

const Vec2<float> canvas_size =
    Vec2<float>(2. * tan(alpha) * near_clip, 2. * tan(beta) * near_clip);

float norm(const Vec3i &a, const Vec3i &b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}

Vec3f pointGlobalToNormal(const Vec3f &point_global, float near_clip, Vec2<float> canvas_size,
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
    Vec3<float> point_camera = Vec3<float>(dx, dy, dz);

    // Convert to screen pos
    Vec3<float> point_screen;
    point_screen.x = near_clip * point_camera.x / -point_camera.z;
    point_screen.y = near_clip * point_camera.y / -point_camera.z;
    point_screen.z = -point_camera.z;

    // Convert to NDC
    float t = canvas_size.y / 2.f;
    float b = -canvas_size.y / 2.f;
    float l = -canvas_size.x / 2.f;
    float r = canvas_size.x / 2.f;

    Vec3<float> point_ndc;
    point_ndc.x = (2.f * point_screen.x / (r - l)) - ((r + l) / (r - l));
    point_ndc.y = (2.f * point_screen.y / (t - b)) - ((t + b) / (t - b));
    point_ndc.z = point_screen.z;

    // Convert to [0,1) normal space
    Vec3<float> point_norm_pos;
    point_norm_pos.x = (point_ndc.x + 1.) / 2.;
    point_norm_pos.y = (point_ndc.y + 1.) / 2.;
    point_norm_pos.z = point_ndc.z;

    return point_norm_pos;
}

std::pair<Vec2<uint32_t>, bool> pointNormalToRaster(Vec3f point_norm_pos) {

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

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

/**
 * Returns the pixels inside a triangle in [0, 1) normal coordinates.
 */
std::vector<Vertex> trianglePoints(const Vec3<Vertex> &triangle, uint32_t img_width,
                                   uint32_t img_height) {
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
        // If the point is not inside the screen
        if (x < 0 || x >= 1.0) {
            continue;
        }
        for (float y = start_y; y <= top_left.y; y += pixel_height) {
            // If the point is not inside the screen
            if (y < 0 || y >= 1.0) {
                continue;
            }

            Vec3f p(x, y, triangle[0].pos.z);

            // We must rotate counter clockwise over the triangle for the edge function if the
            // normal is inversed
            const Vec3f &AB = Vec3f(triangle[1].pos.x, triangle[1].pos.y, 0) +
                              Vec3f(-triangle[0].pos.x, -triangle[0].pos.y, 0);
            const Vec3f &BC = Vec3f(triangle[2].pos.x, triangle[2].pos.y, 0) +
                              Vec3f(-triangle[1].pos.x, -triangle[1].pos.y, 0);
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
                // Interpolate z value
                w0 /= area;
                w1 /= area;
                w2 /= area;

                float z =
                    1 / ((w0 / vertex_a.pos.z) + (w1 / vertex_b.pos.z) + (w2 / vertex_c.pos.z));

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

    return tri_points;
}

void rasterize(const std::vector<Vec3<Vertex>> &triangles) {
    // Preload buffer values
    for (uint32_t i = 0; i < image_width * image_height; ++i)
        frame_buffer[i] = Vec3<unsigned char>(0);

    for (int i = 0; i < image_height; i++) {
        for (int j = 0; j < image_width; j++) {
            z_buffer[i][j] = std::numeric_limits<float>::infinity();
        }
    }

    for (auto triangle : triangles) {
        // Project triangle vertices to normal space
        triangle[0].pos =
            pointGlobalToNormal(triangle[0].pos, near_clip, canvas_size, image_width, image_height);
        triangle[1].pos =
            pointGlobalToNormal(triangle[1].pos, near_clip, canvas_size, image_width, image_height);
        triangle[2].pos =
            pointGlobalToNormal(triangle[2].pos, near_clip, canvas_size, image_width, image_height);

        // Get a list of points to be rendered per triangle
        std::vector<Vertex> tri_points = trianglePoints(triangle, image_width, image_height);

        // Render points
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
}

int main(int argc, char **argv) {

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <geo_file_path> image/text" << std::endl;
        return 1;
    }

    // Read in geometry file
    uint32_t ntris;
    std::unique_ptr<Vec3f[]> vertices;
    std::unique_ptr<Vec2f[]> st;
    std::unique_ptr<uint32_t[]> nvertices;

    scratch::loader::loadGeoFile(argv[1], ntris, vertices, st, nvertices);
    fprintf(stderr, "Geometry file read!\n");

    // Assign random color values to vertices
    std::unique_ptr<Vec3f[]> colors = std::unique_ptr<Vec3f[]>(new Vec3f[ntris * 3]);
    for (uint i = 0; i < ntris; ++i) {
        colors[nvertices[i * 3]] =
            Vec3f((float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
                  (float)rand() / (float)RAND_MAX);
        colors[nvertices[i * 3 + 1]] =
            Vec3f((float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
                  (float)rand() / (float)RAND_MAX);
        colors[nvertices[i * 3 + 2]] =
            Vec3f((float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
                  (float)rand() / (float)RAND_MAX);
    }

    std::vector<Vec3<Vertex>> triangles;

    // Create list of triangles
    for (uint i = 0; i < ntris; ++i) {
        Vec3<float> &v0 = vertices[nvertices[i * 3]];
        Vec3<float> &v1 = vertices[nvertices[i * 3 + 1]];
        Vec3<float> &v2 = vertices[nvertices[i * 3 + 2]];

        Vec3<float> &c0 = colors[nvertices[i * 3]];
        Vec3<float> &c1 = colors[nvertices[i * 3 + 1]];
        Vec3<float> &c2 = colors[nvertices[i * 3 + 2]];

        Vertex vertex_0 = (Vertex){.pos = v0, .rgb = c0};
        Vertex vertex_1 = (Vertex){.pos = v1, .rgb = c1};
        Vertex vertex_2 = (Vertex){.pos = v2, .rgb = c2};

        triangles.push_back(Vec3<Vertex>(vertex_0, vertex_1, vertex_2));
    }

    fprintf(stderr, "Triangles generated!\n");

    if (argv[2] == std::string("image")) {
        // Set image camera position and orientation
        camera_position = Vec3f(0, 10, -18);
        camera_orientation = Vec3f(0, M_PI, 0);

        rasterize(triangles);

        std::ofstream ofs;
        ofs.open("./output.ppm");
        ofs << "P6\n" << image_width << " " << image_height << "\n255\n";
        ofs.write((char *)frame_buffer, image_width * image_height * 3);
        ofs.close();
    } else if (argv[2] == std::string("text")) {
        // Set rotation distance, angle delta, sleep time per step and output character
        const float distance = 25.f;
        const float delta = M_PI / 8;
        const std::chrono::milliseconds SLEEP_TIME(500);
        const std::string PIXEL_CHAR("@");

        // Initial position
        camera_position.y = 5;
        camera_orientation.y = delta;

        while (true) {
            // Spin camera around object
            camera_position.x = cos(camera_orientation.y - M_PI / 2) * distance;
            if (std::abs(camera_position.x) < 0.001) {
                camera_position.x = 0;
            }
            camera_position.z = -sin(camera_orientation.y - M_PI / 2) * distance;
            if (std::abs(camera_position.z) < 0.001) {
                camera_position.z = 0;
            }

            rasterize(triangles);

            camera_orientation.y += delta;

            // Output text
            for (int i = 0; i < image_height * image_width; i++) {
                Vec3<unsigned char> pixel_rgb = frame_buffer[i];
                if (i % image_width == 0) {
                    std::cout << "\n";
                }

                // Find terminal color closest to pixel color
                size_t closest_color_index = 0;
                float closest_color_norm = std::numeric_limits<float>::infinity();
                for (int j = 0; j < terminal_colors.size(); j++) {
                    Vec3i terminal_rgb;
                    std::string ansi;
                    std::tie(terminal_rgb, ansi) = terminal_colors[j];
                    float rgb_norm =
                        norm(Vec3i(pixel_rgb.x, pixel_rgb.y, pixel_rgb.z), terminal_rgb);
                    if (rgb_norm < closest_color_norm) {
                        closest_color_norm = rgb_norm;
                        closest_color_index = j;
                    }
                }

                std::cout << terminal_colors[closest_color_index].second << PIXEL_CHAR
                          << ANSI_COLOR_RESET;
            }

            std::cout << std::endl;
            std::this_thread::sleep_for(SLEEP_TIME);

            // Clear output for next frame
            printf("\033[2J");
            printf("\033[%d;%dH", 0, 0);
        }

    } else {
        std::cerr << "Invalid input" << std::endl;
        return 1;
    }

    fprintf(stderr, "Done!\n");

    delete[] frame_buffer;
    return 0;
}