#!/bin/bash

if [ `uname` == Darwin ]; then
    PY_LIB="libpython2.7.dylib"
else
    PY_LIB="libpython2.7.so"
fi


# Install pygtsam
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${PREFIX} \
      -DPYTHON_EXECUTABLE=${PREFIX}/bin/python \
      -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python2.7/ \
      -DPYTHON_LIBRARY=${PREFIX}/lib/$PY_LIB \
      ..
make -j $CPU_COUNT
make install
