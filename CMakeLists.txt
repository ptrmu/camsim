cmake_minimum_required(VERSION 3.15)
project(camsim)

set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen
  ${OpenCV_INCLUDE_DIR}
)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if (_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach (_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach ()
endif ()

add_executable(camsim
  src/camsim.cpp
  src/gtsam_resection.cpp
  src/opencv_resection.cpp
  )

target_link_libraries(camsim
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  )