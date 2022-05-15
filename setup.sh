#!/usr/bin/env bash

project_path=$(pwd)
git submodule update --recursive --init
cd $project_path
mkdir -p build
cd build
cmake ..
make -j