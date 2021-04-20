

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

TEST_CASE("fixed_lag_smoother_example - run - zero", "[all]")
{
  REQUIRE(0 == fixed_lag_smoother_example());
}

TEST_CASE("smart_factor_pose_simple - run - zero", "[.][all]")
{
  REQUIRE(0 == camsim::smart_factor_pose_simple());
}

