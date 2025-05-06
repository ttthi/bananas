#!/bin/sh

set -e

podman build -t bananas-opencv:0.0.2 containers/build-opencv
podman build -t bananas-ogre:0.0.1 containers/build-ogre
podman build -t bananas-mavsdk:0.0.1 containers/build-mavsdk
podman build -t bananas-px4:0.0.1 containers/build-px4
podman build -t bananas-base:0.0.4 containers/aruco-base
