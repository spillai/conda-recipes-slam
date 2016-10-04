echo "--------------------------------------------------------------"
echo "Deactivating pygtsam: $CONDA_PREFIX"
export PYTHONPATH=$_PYGTSAM_PYTHONPATH_BACKUP
export LD_LIBRARY_PATH=$_PYGTSAM_LD_LIBRARY_PATH_BACKUP
unset _PYGTSAM_PYTHONPATH_BACKUP
unset _PYGTSAM_LD_LIBRARY_PATH_BACKUP
echo "--------------------------------------------------------------"
