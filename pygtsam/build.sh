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

ACTIVATE_DIR=$PREFIX/etc/conda/activate.d
DEACTIVATE_DIR=$PREFIX/etc/conda/deactivate.d
mkdir -p $ACTIVATE_DIR
mkdir -p $DEACTIVATE_DIR

cp $RECIPE_DIR/activate.sh $ACTIVATE_DIR/pygtsam-activate.sh
cp $RECIPE_DIR/deactivate.sh $DEACTIVATE_DIR/pygtsam-deactivate.sh
