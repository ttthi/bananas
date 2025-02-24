# A Dockerfile for the CI system configuration. This exists to give GitHub
# Actions prebuilt versions of our dependencies as needed so that they don't
# need to be rebuilt every time our CI runs.

# Usage:
# VERSION=0.0.1
# podman build -t bananas-base:"$VERSION" . && podman push bananas-base:"$VERSION" docker://docker.io/mainiomano/bananas-base:"$VERSION"

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basics
    ca-certificates \
    curl \
    git \
    parallel \
    tar \
    # Build tools and linters
    clang \
    clang-format \
    clang-tidy \
    cmake \
    g++ \
    libclang-rt-dev \
    ninja-build \
    # Library dependencies
    libeigen3-dev \
    libogre-1.12-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src

# Download OpenCV.
RUN ["/bin/bash", "-o", "pipefail", "-c", "curl -L https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz | tar -xz"]

WORKDIR /usr/src/opencv-4.10.0

# Build OpenCV.
RUN cmake -GNinja -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_LIST=core,objdetect,videoio,highgui,calib3d,imgcodecs
RUN cmake --build build --target install

WORKDIR /usr/src

# We no longer need the OpenCV source code.
RUN rm -r opencv-4.10.0
