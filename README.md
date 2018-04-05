# Build GTSAM and iSAM with conda

Author: [Sudeep Pillai](http://people.csail.mit.edu/spillai)<br>
License: MIT

[![Build Status](https://travis-ci.org/spillai/conda-recipes-slam.svg?branch=master)](https://travis-ci.org/spillai/conda-recipes-slam)

### Install
Install miniconda if you don't have it and add to path
```sh
wget https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh
sh Miniconda2-latest-Linux-x86_64.sh -b -p $HOME/anaconda
export PATH=$HOME/anaconda/bin:$PATH
```

Get the newest version of conda, as well as some conda build tools.
```sh
conda update conda -y
conda install conda-build anaconda-client -y
```

Create a new environment, and build all packages.
Both `PYTHONPATH` and `LD_LIBRARY_PATH` should have been set automatically on environment activation. See activate.sh / deactivate.sh.
```sh
conda create -n slam_env python=2
source activate slam_env
build_all.sh
```
