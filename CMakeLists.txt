cmake_minimum_required(VERSION 3.15)
project(camsim)

set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(VTK REQUIRED)
#find_package(VTK COMPONENTS
#  vtkChartsCore
#  vtkCommonColor
#  vtkCommonCore
#  vtkCommonDataModel
#  vtkInteractionStyle
#  vtkRenderingAnnotation
#  vtkRenderingContext2D
#  vtkRenderingContextOpenGL2
#  vtkRenderingCore
#  vtkRenderingFreeType
#  vtkRenderingGL2PSOpenGL2
#  vtkRenderingOpenGL2
#  vtkViewsContext2D
#  QUIET)

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
#  src/vtk_line_plot.cpp
#  src/vtk_rand_dist.cpp
#  src/vtk_multi_plot.cpp
  src/pfm_model.cpp
  )

target_link_libraries(camsim
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
#  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS camsim
#  MODULES ${VTK_LIBRARIES}
#  )

add_executable(pfm_run
  src/model.cpp
  src/pfm_model.cpp
  src/pfm_run.cpp
  src/pfm_resection_gtsam.cpp
  src/pfm_resection_opencv.cpp
  src/pfm_simple_rotation_example.cpp 
  src/pfm_optimize_pose3.cpp
  src/pose_with_covariance.cpp
  src/pfm_resection_projection.cpp
  src/pfm_resection_monte_carlo.cpp)

target_link_libraries(pfm_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
#  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS pfm_run
#  MODULES ${VTK_LIBRARIES}
#  )

add_executable(sfm_run
  src/model.cpp
  src/sfm_run.cpp
  src/sfm_gtsam_example.cpp
  src/sfm_gtsam_slam_example.cpp
  src/sfm_resectioning.cpp
  src/sfm_isam2_example.cpp
  src/sfm_isam2.cpp
  src/pose_with_covariance.cpp
  )

target_link_libraries(sfm_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  #  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS sfm_run
#  MODULES ${VTK_LIBRARIES}
#  )

add_executable(pfp_run
  src/model.cpp
  src/pfp_run.cpp
  src/pose_with_covariance.cpp
  src/pfp_odometry_example_3d.cpp)

target_link_libraries(pfp_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  #  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS sfm_run
#  MODULES ${VTK_LIBRARIES}
#  )
