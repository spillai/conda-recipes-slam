#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi

# Install gtsam
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${PREFIX} ..
make -j 6
make install
