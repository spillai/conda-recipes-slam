cmake_minimum_required(VERSION 2.6.0)
set(CMAKE_CXX_FLAGS "-fopenmp -fpermissive -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse -DNDEBUG  -std=c++11 -O3 -fPIC")

# Check the architecture and set CMAKE_LIBRARY_ARCHITECTURE accordingly
if(UNIX)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_LIBRARY_ARCHITECTURE "x86_64-linux-gnu")
  endif()
endif()

# find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
# PKG_CHECK_MODULES(OpenCV REQUIRED opencv>=2.4.10)
# message("FOUND OPENCV: ${OPENCV_LDFLAGS}")

include(cmake/boost-python.cmake)
# include(cmake/FindEigen3.cmake)
# include(cmake/FindCholmod.cmake)
# include(cmake/FindNumPy.cmake)
# include(GTSAM/GTSAMConfig.cmake)

# set(GTSAM_CMAKE_TOOLS_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/GTSAM")
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/lib/cmake/GTSAM")

# include(GtsamBuildTypes) # Load build type flags and default to Debug mode
# include(GtsamTesting)    # Easy functions for creating unit tests and scripts
# include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
# include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

# Include python
set(EIGEN3_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/Eigen)
include_directories(${CMAKE_INSTALL_PREFIX}/include)
include_directories(${NUMPY_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CHOLMOD_INCLUDES})

# message("INCLUDE *********** ${CMAKE_INSTALL_PREFIX}/")
# message("CURR *********** ${CMAKE_CURRENT_SOURCE_DIR}/")

# =========== bot_types Python wrapper ==============
boost_python_module(pybot_types
    src/pybot_types.cpp)
target_link_libraries(pybot_types boost_system boost_python)
install(TARGETS pybot_types
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# # =========== bot_cv_types Python wrapper ==============
# boost_python_module(pybot_cv_types
#     src/pybot_cv_types.cpp
#     src/opencv_numpy_conversion.cpp)
# target_link_libraries(pybot_cv_types boost_system boost_python ${OpenCV_LDFLAGS})
# install(TARGETS pybot_cv_types
#   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
# )

# =========== bot_eigen_types Python wrapper ==============
boost_python_module(pybot_eigen_types
    src/pybot_eigen_types.cpp
    src/eigen_numpy_conversion.cpp)
target_link_libraries(pybot_eigen_types boost_system boost_python glog ${CHOLMOD_LIBRARIES})
install(TARGETS pybot_eigen_types
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# # =========== pyisam Python wrapper ==============
# boost_python_module(pyisam src/pyisam.cpp)
# target_link_libraries(pyisam boost_system pybot_types pybot_eigen_types isam ${CHOLMOD_LIBRARIES})
# install(TARGETS pyisam
#   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
# )

# =========== pygtsam Python wrapper ==============
boost_python_module(pygtsam src/pygtsam.cpp)
target_link_libraries(pygtsam boost_python boost_system pybot_types pybot_eigen_types gtsam)
install(TARGETS pygtsam
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
