#!/bin/bash
mkdir build
cd build
cmake                                                               \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}                                \
    -DGTSAM_WITH_EIGEN_MKL=OFF                                       \
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF                                \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF                               \
    -DGTSAM_BUILD_TESTS=OFF                                         \
    ..
make -j $CPU_COUNT
make install
