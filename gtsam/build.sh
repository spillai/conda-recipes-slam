#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi

mkdir build
cd build
cmake                                                               \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF                           \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}                                \
    -DGTSAM_WITH_EIGEN_MKL=ON                                       \
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=ON                                \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF                               \
    -DGTSAM_BUILD_TESTS=OFF                                         \
    -DGTSAM_BUILD_PYTHON=ON                                         \
    ..
make -j 6
make install

    # -DPYTHON_EXECUTABLE=${PREFIX}/bin/python                        \
    # -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python2.7/               \
    # -DPYTHON_LIBRARY=${PREFIX}/lib/$PY_LIB                          \
    # -DPYTHON_PACKAGES_PATH=${PREFIX}/lib/python2.7/site-packages/   \
