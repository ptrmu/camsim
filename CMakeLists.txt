cmake_minimum_required(VERSION 3.15)
project(camsim)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-unknown-pragmas)
endif()

find_package(GTSAM REQUIRED)
find_package(OpenCV 4 REQUIRED)
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
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen
  ${OpenCV_INCLUDE_DIR}
  include
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
  src/pfm/pfm_run.cpp
  src/pfm/pfm_resection_gtsam.cpp
  src/pfm/pfm_resection_opencv.cpp
  src/pfm/pfm_simple_rotation_example.cpp
  src/pfm/pfm_optimize_pose3.cpp
  src/pose_with_covariance.cpp
  src/pfm/pfm_resection_projection.cpp
  src/pfm/pfm_resection_monte_carlo.cpp
  src/pfm/pfm_run_main.cpp
  )

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
  src/sfm/sfm_run.cpp
  src/sfm/sfm_gtsam_example.cpp
  src/sfm/sfm_gtsam_slam_example.cpp
  src/sfm/sfm_resectioning.cpp
  src/sfm/sfm_isam2_example.cpp
  src/sfm/sfm_isam2.cpp
  src/pose_with_covariance.cpp
  src/sfm/sfm_run_main.cpp
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
  src/pose_with_covariance.cpp
  src/pfp/pfp_run.cpp
  src/pfp/pfp_odometry_example_3d.cpp
  src/pfp/pfp_pose_unit_test.cpp
  src/pfp/pfp_marker_pose_from_corners.cpp
  src/pfp/pfp_gspnp.cpp
  src/pfp/pfp_run_main.cpp
  )

target_link_libraries(pfp_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  #  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS sfm_run
#  MODULES ${VTK_LIBRARIES}
#  )

add_executable(map_run
  src/map/map_run.cpp
  src/map/map_global.cpp
  src/model.cpp
  src/pose_with_covariance.cpp
  src/map/map_solver_marker_marker.cpp
  src/map/map_solver_project_between.cpp
  )

target_link_libraries(map_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  ${CMAKE_THREAD_LIBS_INIT}
  #  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS sfm_run
#  MODULES ${VTK_LIBRARIES}
#  )


add_executable(cal_run
  src/cal/cal_run.cpp
  src/model.cpp
  src/pose_with_covariance.cpp
  src/cal/cal_run_main.cpp
  src/calibration_model.cpp
  src/cal/cal_solver_opencv.cpp
  src/cal/cal_solver_runner.cpp
  src/cal/cal_solver_project_calibrate.cpp
  src/cal/cal_solver_homography.cpp
  )

target_link_libraries(cal_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  )


add_executable(sho_run
  src/fvlam/build_marker_map_tmm.cpp
  src/fvlam/conversions_cv.cpp
  src/fvlam/conversions_gtsam.cpp
  src/fvlam/to_string.cpp
  src/fvlam/transform3_with_covariance.cpp
  src/sho/sho_inter_marker_rotation.cpp
  src/sho/sho_run.cpp
  src/sho/sho_run_main.cpp
  src/build_marker_map_runner.cpp
  src/model.cpp
  src/pose_with_covariance.cpp
  )

target_link_libraries(sho_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  )


add_executable(kal_run
  src/kal/kal_run.cpp
  src/kal/kal_run_main.cpp
#    src/vtk_line_plot.cpp
#    src/vtk_rand_dist.cpp
#    src/vtk_multi_plot.cpp
  src/kal/kal_opencv_filter.cpp
  src/kal/kal_kalman_filter_x.cpp
)

target_link_libraries(kal_run
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  PRIVATE opencv_video
  PRIVATE opencv_highgui
#  PRIVATE ${VTK_LIBRARIES}
  )

#vtk_module_autoinit(TARGETS cal_run
#  MODULES ${VTK_LIBRARIES}
#  )


add_executable(camsim_test
  src/test/camsim_test.cpp
  src/pose_with_covariance.cpp
  src/model.cpp
  src/gtsam_resection.cpp
  src/opencv_resection.cpp
  src/test/map_test.cpp
  src/map/map_solver_marker_marker.cpp
  src/map/map_solver_project_between.cpp
  src/test/pfm_test.cpp
  src/pfm/pfm_run.cpp
  src/pfm/pfm_resection_gtsam.cpp
  src/pfm/pfm_resection_opencv.cpp
  src/pfm/pfm_simple_rotation_example.cpp
  src/pfm/pfm_optimize_pose3.cpp
  src/pfm/pfm_resection_projection.cpp
  src/pfm/pfm_resection_monte_carlo.cpp
  src/test/sfm_test.cpp
  src/sfm/sfm_run.cpp
  src/sfm/sfm_gtsam_example.cpp
  src/sfm/sfm_gtsam_slam_example.cpp
  src/sfm/sfm_resectioning.cpp
  src/sfm/sfm_isam2_example.cpp
  src/sfm/sfm_isam2.cpp
  src/test/pfp_test.cpp
  src/pfp/pfp_run.cpp
  src/pfp/pfp_odometry_example_3d.cpp
  src/pfp/pfp_pose_unit_test.cpp
  src/pfp/pfp_marker_pose_from_corners.cpp
  src/pfp/pfp_gspnp.cpp
  src/calibration_model.cpp
  src/test/cal_test.cpp
  src/cal/cal_solver_opencv.cpp
  src/cal/cal_solver_runner.cpp
  src/cal/cal_solver_project_calibrate.cpp
  src/cal/cal_solver_homography.cpp
  src/test/sho_test.cpp
  src/sho/sho_run.cpp
  src/sho/sho_inter_marker_rotation.cpp
  src/fvlam/build_marker_map_tmm.cpp
  src/fvlam/conversions_cv.cpp
  src/fvlam/conversions_gtsam.cpp
  src/fvlam/file_storage.cpp
  src/fvlam/to_string.cpp
  src/fvlam/transform3_with_covariance.cpp
  src/build_marker_map_runner.cpp
  src/test/tmm_test.cpp
  src/test/file_storage_test.cpp
  )


target_link_libraries(camsim_test
  PRIVATE gtsam
  PRIVATE opencv_core
  PRIVATE opencv_calib3d
  )
