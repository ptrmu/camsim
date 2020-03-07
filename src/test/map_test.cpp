
#include "catch2/catch.hpp"

#include "../map_run.hpp"
#include "../map_solver_runner.hpp"
#include "../model.hpp"

namespace camsim
{
  struct TestParams
  {
    int n_markers = 16;
    int n_cameras = 64;

    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 1.0;
    double u_noise_sigma = 1.0;
  };

  static TestParams test_params;

  static auto run_solvers = [](ModelConfig &mc, TestParams &tp) -> int
  {
    Model model{mc};

    SolverRunner solver_runner{model,
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(tp.r_sigma),
                                 gtsam::Vector3::Constant(tp.t_sigma)).finished(),
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(tp.r_sigma),
                                 gtsam::Vector3::Constant(tp.t_sigma)).finished(),
                               gtsam::Vector2::Constant(tp.u_sampler_sigma),
                               gtsam::Vector2::Constant(tp.u_noise_sigma),
                               false};

    solver_runner([](SolverRunner &solver_runner)
                  { return solver_project_between_isam_factory(solver_runner); });

    solver_runner([](SolverRunner &solver_runner)
                  { return solver_project_between_opencv_factory(solver_runner); });

    return 0;
  };

//  TEST_CASE("map_test - project_between ring of markers")
//  {
//    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{test_params.n_markers, 2.},
//                             PoseGens::SpinAboutZAtOriginFacingOut{test_params.n_cameras},
//                             camsim::CameraTypes::simulation,
//                             0.1775};
//
//    REQUIRE(run_solvers(model_config, test_params) == 0);
//  }
//
//  TEST_CASE("map-test - project_between circle of markers")
//  {
//    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingAlongZ{test_params.n_markers, 2., 2., false},
//                             PoseGens::CircleInXYPlaneFacingAlongZ{test_params.n_cameras, 2., 0., true},
//                             camsim::CameraTypes::simulation,
//                             0.1775};
//
//    REQUIRE(run_solvers(model_config, test_params) == 0);
//  }
}