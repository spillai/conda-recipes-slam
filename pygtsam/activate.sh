echo "--------------------------------------------------------------"
echo "Activating pygtsam: $CONDA_PREFIX"
export _PYGTSAM_PYTHONPATH_BACKUP=$PYTHONPATH
export _PYGTSAM_LD_LIBRARY_PATH_BACKUP=$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:$CONDA_PREFIX/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib

echo "Exported variables: "
echo "PYTHONPATH: $PYTHONPATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "--------------------------------------------------------------"
