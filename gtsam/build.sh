#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi

# Install gtsam
mkdir build
cd build
cmake                                                               \
    -DPYTHON_EXECUTABLE=${PREFIX}/bin/python                        \
    -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python2.7/               \
    -DPYTHON_LIBRARY=${PREFIX}/lib/$PY_LIB                          \
    -DPYTHON_PACKAGES_PATH=${PREFIX}/lib/python2.7/site-packages/   \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}                                \
    -DGTSAM_WITH_EIGEN_MKL=ON                                       \
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=ON                                \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF                               \
    -DGTSAM_BUILD_TESTS=OFF                                         \
    ..
make -j 6
make install

# Install pywrappers
cmake \
    -DCMAKE_INSTALL_PREFIX=${PREFIX} \
    ../pywrappers
make -j 6
make install
