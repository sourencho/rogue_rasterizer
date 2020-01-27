# rogue_rasterizer

A simple software rasterizer written for learning purposes using the tutorial [Scratchapixel: Rasterization Practical Implementation](https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation).

> Rasterization is the task of taking an image described in a vector graphics format (shapes) and converting it into a raster image (a series of pixels, dots or lines, which, when displayed together, create the image which was represented via shapes).

In this case we are taking as input a shape defined by triangles in 3D cooridnates (.geo file) and coordinates/orientation for the camera and outputting the 2D projection of the camera's perspective as an image (.ppm file).

Authors: [@hwacha](https://github.com/hwacha) and [@sourenp](https://github.com/Sourenp)

### Run

maxOS
```bash
# Usage: ./main <geo_file_path>
c++  -o main src/main.cpp -std=c++11 -O3
./main "data/cow.geo"
open output.ppm
```

![cow output](images/cow.jpg)


### Todo

- [ ] Perspective correct vertex attribute interpolation (color)
- [ ] Import color from .obj files instead of assigning random colors
