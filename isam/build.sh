#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi

mkdir build
cd build
cmake                                                               \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}                                \
    -DUSE_GUI=OFF                                                   \
    ..
make -j 6
make install
