

#include "catch2/catch.hpp"

#include "../smf/smf_run.hpp"

TEST_CASE("sfm_example_smart_factor - run - zero", "[.][all]")
{
  REQUIRE(sfm_example_smart_factor() == 0);
}

TEST_CASE("isam2_example_smart_factor - run - zero", "[.][all]")
{
  REQUIRE(isam2_example_smart_factor() == 0);
}

TEST_CASE("smart_factor_pose_simple - run - zero", "[all]")
{
  REQUIRE(camsim::smart_factor_pose_simple() == 0);
}

