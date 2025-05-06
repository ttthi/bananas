# Dockerfile for building a recent version of OpenCV on Ubuntu 24.04.

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    tar \
    # Build tools
    cmake \
    g++ \
    ninja-build \
    pkg-config \
    # Dependencies
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    qt6-base-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/opencv-src

# Download OpenCV.
ADD https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz opencv-4.10.0.tar.gz
RUN tar -xzf opencv-4.10.0.tar.gz

WORKDIR /usr/opencv-src/opencv-4.10.0

# Build OpenCV.
RUN cmake -GNinja -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_LIST=core,objdetect,video,videoio,highgui,calib3d,imgcodecs \
    -DWITH_FFMPEG=ON \
    -DWITH_QT=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/opencv
RUN cmake --build build --target install

WORKDIR /usr/opencv
