# Build GTSAM and iSAM with conda

Author: [Sudeep Pillai](http://people.csail.mit.edu/spillai)<br>
License: MIT

### Install miniconda and add to PATH
Install anaconda if you don't have it (instructions here for OS X)
```sh
wget http://repo.continuum.io/miniconda/Miniconda-latest-MacOSX-x86_64.sh
sh Miniconda-latest-MacOSX-x86_64.sh -b -p $HOME/anaconda
export PATH=$HOME/anaconda/bin:$PATH
```

### To build packages
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
