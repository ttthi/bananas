# ArUco-based positioner and box detector

## Building

### Dependencies

- A recent version of [OpenCV](https://opencv.org/). Tested with 4.10.0 and
  4.11.0
- [Eigen](https://eigen.tuxfamily.org/) >= 3.4
- [nlohmann_json](https://github.com/nlohmann/json) >= 3.11.3
- [OGRE](https://www.ogre3d.org/) >= 1.12, but not Ogre-Next
- [tinyxml2](https://github.com/leethomason/tinyxml2) 10.\*

### Compilation

``` sh
cmake -S . -B build
cmake --build build
```

## Usage

The positioner requires four inputs:

- A JSON description of the boxes
- A JSON description of the static environment
- A JSON file containing the camera parameters
- An input video to analyze

The file formats are currently not documented. An example configuration is
available at [Google
Drive](https://drive.google.com/drive/folders/1tW-6yEx6MTxDuQMm00okEwCPTrKfiRjt?usp=sharing).

``` sh
./build/apps/positioner -boxes=boxes.json -env=static_environment.json -camera=camera.json video.mp4
```
