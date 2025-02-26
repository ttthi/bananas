# Dockerfile for downloading dependencies of the ArUco detector. This image is
# useful both for local testing and as a basis for running the CI on.
#
# Updating for CI (needs access to docker.io/mainiomano/bananas-base):
# VERSION=0.0.2
# podman build -t bananas-base:"$VERSION" . && podman push bananas-base:"$VERSION" docker://docker.io/mainiomano/bananas-base:"$VERSION"

FROM ubuntu:24.04

ADD --chown=_apt https://packages.osrfoundation.org/gazebo.gpg /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    # OGRE dependencies \
    libsdl2-2.0.0 \
    # OpenCV dependencies
    libavcodec60 \
    libavformat60 \
    libavutil58 \
    libswscale7 \
    libqt6core6t64 \
    libqt6gui6t64 \
    libqt6test6t64 \
    libqt6widgets6t64 \
    # Direct dependencies
    libeigen3-dev \
    nlohmann-json3-dev \
    libtinyxml2-dev \
    # Gazebo
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install prebuilt dependencies.
COPY --from=docker.io/mainiomano/bananas-opencv:0.0.1 /usr/opencv /usr/opencv
COPY --from=docker.io/mainiomano/bananas-ogre:0.0.1 /usr/ogre /usr/ogre

# Let ld.so find our prebuilt dependencies.
RUN ldconfig /usr/opencv /usr/ogre
