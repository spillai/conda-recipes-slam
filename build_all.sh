conda install suitesparse -c menpo -y

# conda build isam
# conda install --use-local isam -y

# conda build pyisam
# conda install --use-local pyisam -y

conda install -c intel mkl tbb -y
conda build gtsam
conda install --use-local gtsam -y

conda build pygtsam
conda install --use-local pygtsam -y

