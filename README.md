tar -xvzf gtsam-3.2.1.tgz
cd gtsam-3.2.1


mkdir build
cd build
cmake                                                               \
    -DPYTHON_EXECUTABLE=$PREFIX/bin/python                          \
    -DPYTHON_INCLUDE_DIR=$PREFIX/include/python2.7/                 \
    -DPYTHON_LIBRARY=$PREFIX/lib/$PY_LIB                            \
    -DPYTHON_PACKAGES_PATH=$PREFIX/lib/python2.7/site-packages/     \
    -DCMAKE_INSTALL_PREFIX=$PREFIX                                  \
    -DGTSAM_WITH_EIGEN_MKL=OFF                                      \
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF                               \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF                               \
    -DGTSAM_BUILD_TESTS=OFF                                         \
    ..
make -j 4
make install
