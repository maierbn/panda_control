# `libfranka` for controlling Fanka Emika Panda with C++
#
# 2019 Christoph Hinze, ISW Uni Stuttgart 
#
# built with:
# run with: 

FROM ubuntu:18.04 as build_stage

ARG VERSION=0.6.0

WORKDIR /frankaemika/

RUN apt-get update && \
    apt-get install --yes build-essential cmake git libpoco-dev libeigen3-dev && \
    git clone --recursive https://github.com/frankaemika/libfranka && cd libfranka && \
    git checkout ${VERSION} && \
    git submodule update && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF .. && \
    cmake --build . -- -j4 && \
    make install && \
    tar -cvf libfranka.tar /usr/local/lib/libfranka.so* /usr/local/include/franka /usr/local/lib/cmake/Franka

FROM ubuntu:18.04 as exec_stage

COPY --from=build_stage /frankaemika/libfranka/build/libfranka.tar /

RUN tar -xvf libfranka.tar && rm libfranka.tar && \
    apt-get update && \
    apt-get install --yes   libpoco-dev \
    libeigen3-dev \
    build-essential \
    cmake \
    rapidjson-dev && \
    ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen && \
    ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported

