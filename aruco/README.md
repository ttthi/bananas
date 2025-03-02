# ArUco-based positioner and box detector

## Preparing the system

### Host system dependencies

- podman
- distrobox

### Preparing the image

``` sh
podman build -t bananas-base:0.0.3 ./containers/aruco-base
```

### Preparing a container

``` sh
distrobox create --name bananas-base --image bananas-base:0.0.3
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
cmake -GNinja -S . -B build -DCMAKE_PREFIX_PATH='/usr/opencv;/usr/ogre;/usr/cv_bridge' \
                            -DCMAKE_BUILD_TYPE=Release \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

If you want to include ROS support, run

``` sh
. /opt/ros/jazzy/setup.bash
```

to set up the ROS environment and then add `-DWITH_ROS2=ON` to the CMake command.

### Compilation

``` sh
cmake --build build
```

## Usage

### Positioner

The positioner requires three or four inputs depending on whether it was built
with ROS support:

- A JSON description of the boards
- A JSON description of the static environment
- A JSON file containing the camera parameters
- An input video to analyze if `WITH_ROS2` is not enabled

The file formats are currently not documented. An example configuration is
available at [Google
Drive](https://drive.google.com/drive/folders/1jW_gUaRNqzDQmUnwXLOY9ooAgiT-EK1z?usp=drive_link).

#### Non-ROS

``` sh
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json video.mp4
```

#### ROS

``` sh
. /opt/ros/jazzy/setup.bash
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json &
./build/apps/ros_video_publisher video.mp4
```

### Gazebo demo

The Gazebo integration requires the project built with `-DWITH_ROS2=ON`.

#### Gazebo startup

In one shell instance, enter the `gazebo` subdirectory of your build directory.
Example:

``` sh
cd build/gazebo
```

Start up Gazebo:

``` sh
gz sim box_world.sdf
```

Unpause the simulation to make it start producing image data.

NOTE: This must be run from a session in which `. /opt/ros/jazzy/setup.bash` has
not been run, or `gz` won't find the `sim` subcommand.

NOTE: Gazebo will freeze if you have are using a firewall that blocks multicast
traffic. Example for temporarily allowing multicast on `firewalld`:

``` sh
firewall-cmd --add-rich-rule='rule family=ipv4 destination address="224.0.0.0/4" accept'
```

#### Positioner startup

In another shell instance, run inside your build directory:

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/box_world/model/camera/link/link/sensor/sensor/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/boards.json -env=gazebo/static_environment.json -camera=gazebo/camera.json \
                  --ros-args -r aruco_camera/image:=/world/box_world/model/camera/link/link/sensor/sensor/image --
```
