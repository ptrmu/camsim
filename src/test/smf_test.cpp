

#include "catch2/catch.hpp"

#include "../smf/smf_run.hpp"

TEST_CASE("sfm_example_smart_factor - run - zero", "[.][all]")
{
  REQUIRE(0 == sfm_example_smart_factor());
}

TEST_CASE("isam2_example_smart_factor - run - zero", "[.][all]")
{
  REQUIRE(0 == isam2_example_smart_factor());
}

TEST_CASE("fixed_lag_smoother_example - run - zero", "[.][all]")
{
  REQUIRE(0 == fixed_lag_smoother_example());
}

TEST_CASE("concurrent_filtering_and_smoothing_example - run - zero", "[.][all]")
{
  REQUIRE(0 == concurrent_filtering_and_smoothing_example());
}

TEST_CASE("smart_factor_pose_simple - run - zero", "[.][all]")
{
  REQUIRE(0 == camsim::smart_factor_pose_simple());
}

TEST_CASE("imager_relative_pose - run - zero", "[.][all]")
{
  REQUIRE(0 == camsim::imager_relative_pose());
}

TEST_CASE("factors_gtsam_test - run - zero", "[.][all]")
{
  REQUIRE(0 == camsim::factors_gtsam_test());
}

TEST_CASE("inter_marker_pose_test - run - zero", "[all]")
{
  REQUIRE(0 == camsim::inter_marker_pose_test());
}

