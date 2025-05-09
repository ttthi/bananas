# ArUco-based positioner and box detector

## Preparing the system

### Downloading submodules

``` sh
git submodule update --init
```

### Host system dependencies

- podman
- distrobox
- A GPU driver supported by distrobox and matching hardware

### Building the images

``` sh
./build_images.sh
```

### Preparing a container

``` sh
distrobox create --name bananas-aruco --image bananas-base:0.0.4
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
# Remove the following line if you don't need ROS support.
. /opt/ros/jazzy/setup.bash
cmake -GNinja -S . -B build -DCMAKE_PREFIX_PATH='/usr/opencv;/usr/ogre;/usr/cv_bridge;/usr/mavlink' \
                            -DCMAKE_BUILD_TYPE=Release \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                            # Set to OFF if you don't need ROS support.
                            -DWITH_ROS2=ON
```

### Compilation

``` sh
cmake --build build
```

### Generating documentation

Run `doxygen` in the directory of this README. The generated documentation will
be placed under the `docs/` subdirectory, with `docs/html/index.html` as the
entrypoint.

## Usage

### Positioner

The positioner requires three or four inputs depending on whether it was built
with ROS support:

- A JSON description of the boards
- A JSON description of the static environment
- A JSON file containing the camera parameters
- An input video to analyze if `WITH_ROS2` is not enabled

The file formats are currently not documented, but their structure is somewhat
simple. Example configurations are available in the [gazebo](gazebo/)
subdirectory.

#### Running with a prerecorded input video

##### Non-ROS

``` sh
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json video.mp4
```

##### ROS

``` sh
. /opt/ros/jazzy/setup.bash
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json &
./build/apps/ros_video_publisher video.mp4
```

### Gazebo simulation

#### General notes

- The Gazebo integration requires the project built with `-DWITH_ROS2=ON`.
- Gazebo must not be run from a shell session in which
  `/opt/ros/jazzy/setup.bash` has been sourced, or `gz` won't find the `sim`
  subcommand. Hence, if you have just built the project with ROS support, you
  should run Gazebo from a different shell instance.
- Gazebo will freeze if you have are using a firewall that blocks multicast
  traffic. Example for temporarily allowing multicast on `firewalld`:

  ``` sh
  firewall-cmd --add-rich-rule='rule family=ipv4 destination address="224.0.0.0/4" accept'
  ```

#### box_world demo

![box_world screenshot](https://github.com/user-attachments/assets/3fb09ce6-482a-4460-916a-921bc582887f)

This demo shows how to define a camera in Gazebo and read its image through ROS.
It replicates the physical demo available on [Google
Drive](https://drive.google.com/file/d/1ZGzvk3Pro302nx5kvyeuDO0ZOaWG-0qF/view?usp=drive_link).

##### Gazebo startup

In one shell instance, start up Gazebo with the main SDF file. Example, run from
the aruco source subdirectory in which this README is located:

``` sh
GZ_SIM_RESOURCE_PATH=$(pwd)/build/gazebo/box_world gz sim box_world.sdf
```

Unpause the simulation to make it start producing image data.

##### Positioner startup

In another shell instance, run inside your build directory:

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/box_world/model/camera/link/link/sensor/sensor/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/box_world/boards.json -env=gazebo/box_world/static_environment.json -camera=gazebo/box_world/camera.json \
                  --ros-args -r aruco_camera/image:=/world/box_world/model/camera/link/link/sensor/sensor/image --
```

#### drone_world demo

![drone_world screenshot](https://github.com/user-attachments/assets/78b603c7-f4aa-433f-a881-7bb3ed8049cd)

This demo shows how a drone can be positioned using ArUco markers.

##### Component startup

###### Gazebo startup 

In one shell instance, start up Gazebo with the main SDF file. Example, run from
the aruco source subdirectory in which this README is located:

``` sh
GZ_SIM_RESOURCE_PATH=$(pwd)/build/gazebo/drone_world:$(pwd)/gazebo/PX4-gazebo-models/models gz sim drone_world.sdf
```

Remember to unpause the simulation before attempting to take off.

###### PX4 startup

After starting up Gazebo, run in another shell instance:

``` sh
PX4_SIM_MODEL=gz_x500_mono_cam_down /usr/px4/px4/bin/px4 /usr/px4/px4/etc
```

PX4 will store extra files to whatever directory you are in.

###### Positioner startup

In a yet another shell instance, run inside your build directory:

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/drone_world/boards.json -env=gazebo/drone_world/static_environment.json -camera=gazebo/drone_world/camera.json -mavlink=udpin://0.0.0.0:14540 --ros-args -r aruco_camera/image:=/world/drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image --
```

##### Parameter setup

Set the following PX4 parameter values, for example using QGroundControl:

| Parameter | Value | Explanation |
| --------- | ----- | ----------- |
| SYS_HAS_MAG | 0 | Disable the magnetometer (requires restarting PX4) |
| EKF2_EV_CTRL | 11 | Use position and yaw data from our positioner |

Note that the parameters are persisted by PX4 in whatever directory you run it
from. If you want to reuse the same parameters later, run PX4 from the same
directory in which you set the parameters up earlier.

##### Global positioning

To allow using flight modes that depend on global position information, set an
origin for the local coordinate system. The PX4 console seems to be the most
reliable way to do this. Example PX4 console command:

```
commander set_ekf_origin 60.184933 24.828757 0.0
```

##### Taking off

Start up all the required software and connect QGroundControl to the drone (no
need to run it inside the container). Don't use QGroundControl's "Takeoff"
button to take off since it forces a minimum altitude of 10 meters. Instead, set
the `MIS_TAKEOFF_ALT` parameter to the wanted target altitude and write
`commander takeoff` on the PX4 console to manually take off. If you want to fly
a mission, then starting the mission in QGroundControl will make the drone take
off as expected.

#### demo_world demo

![demo_world screenshot](https://github.com/user-attachments/assets/f5c33946-f673-49a8-8b65-41a0f7f5420d)

Similar to drone_world, but `drone_world` needs to be replaced with `demo_world`
in the Gazebo and positioner startup. To keep the drone altitude sane, I
recommend setting the takeoff altitude (`MIS_TAKEOFF_ALT` parameter,
configurable in QGroundControl) to 0.2 m and taking off from the PX4 console
using `commander takeoff`.

##### Gazebo startup

``` sh
GZ_SIM_RESOURCE_PATH=$(pwd)/build/gazebo/demo_world:$(pwd)/gazebo/PX4-gazebo-models/models gz sim demo_world.sdf
```

##### Positioner startup

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/demo_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/demo_world/boards.json -env=gazebo/demo_world/static_environment.json -camera=gazebo/demo_world/camera.json -mavlink=udpin://0.0.0.0:14540 --ros-args -r aruco_camera/image:=/world/demo_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image --
```
