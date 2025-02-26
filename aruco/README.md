# ArUco-based positioner and box detector

## Preparing the system

### Host system dependencies

- podman
- distrobox

### Preparing the image

``` sh
podman build -t bananas-base:0.0.2 ./containers/aruco-base
```

### Preparing a container

``` sh
distrobox create --name bananas-base --image bananas-base:0.0.2
```

Add `--nvidia` to include support for NVIDIA's proprietary GPU driver.

## Building

Enter the container you created earlier:

``` sh
distrobox enter bananas-aruco
```

### Configuring

From inside the Distrobox container, run

``` sh
cmake -GNinja -S . -B build -DCMAKE_PREFIX_PATH='/usr/opencv;/usr/ogre' \
                            -DCMAKE_BUILD_TYPE=Release \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Compilation

``` sh
cmake --build build
```

## Usage

### Positioner

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

### Gazebo demo

Enter the `gazebo` subdirectory of your build directory. Example:

``` sh
cd build/gazebo
```

Start up Gazebo:

``` sh
gz sim box_world.sdf
```

NOTE: Gazebo will freeze if you have are using a firewall that blocks multicast
traffic. Example for temporarily allowing multicast on `firewalld`:

``` sh
firewall-cmd --add-rich-rule='rule family=ipv4 destination address="224.0.0.0/4" accept'
```
