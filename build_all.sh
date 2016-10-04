conda config --set anaconda_upload no

conda build isam
conda build pyisam

conda build gtsam
conda build pygtsam
