# Build GTSAM and iSAM with conda

https://github.com/jakevdp/conda-recipes
https://github.com/alexbw/conda-lua-recipes

```
conda create -n pybot python=2
```
Both `PYTHONPATH` and `LD_LIBRARY_PATH` should have been set automatically on environment activation. See activate.sh / deactivate.sh.

### To install packages
```
# Install anaconda if you don't have it (instructions here for OS X)
wget
http://repo.continuum.io/miniconda/Miniconda-latest-MacOSX-x86_64.sh
sh Miniconda-latest-MacOSX-x86_64.sh -b -p $HOME/anaconda

# Add anaconda to your $PATH
export PATH=$HOME/anaconda/bin:$PATH

### To build packages
```
# Get the newest version of conda, as well as some conda build tools
conda update conda -y
conda install conda-build anaconda-client -y

# Build all packages
sh build_all.sh
```

# More useful stuff
```
conda install -c asmeurer emacs=24.5
conda install -c menpo eigen=3.2.7
```
