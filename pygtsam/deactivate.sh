echo "--------------------------------------------------------------"
echo "Deactivating pygtsam: $CONDA_PREFIX"
export PYTHONPATH=$_PYGTSAM_PYTHONPATH_BACKUP
unset _PYGTSAM_PYTHONPATH_BACKUP
echo "--------------------------------------------------------------"
