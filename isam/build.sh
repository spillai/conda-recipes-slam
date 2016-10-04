#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi

mkdir build
cd build

if [ `uname` == Darwin ]; then
    cmake                                \
        -DCMAKE_INSTALL_PREFIX=${PREFIX} \
        -DCMAKE_OSX_DEPLOYMENT_TARGET="" \
        -DUSE_GUI=OFF                    \
        ..
else
    cmake                                \
        -DCMAKE_INSTALL_PREFIX=${PREFIX} \
        -DUSE_GUI=OFF                    \
        ..
fi

make -j $CPU_COUNT
make install
