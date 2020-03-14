
#include "catch2/catch.hpp"

#include "../sfm_run.hpp"
#include "../sfm_resectioning.hpp"
#include "../sfm_isam2.hpp"

namespace camsim
{
#if 0
  TEST_CASE("sfm_gtsam_slam_example")
  {
    REQUIRE(sfm_gtsam_slam_example() == 0);
  }

  TEST_CASE("sfm_gtsam_example")
  {
    REQUIRE(sfm_gtsam_example() == 0);
  }

  TEST_CASE("sfm_isam_example")
  {
    REQUIRE(sfm_isam_example() == 0);
  }

  TEST_CASE("sfm_run")
  {
    REQUIRE(sfm_run() == 0);
  }

  TEST_CASE("sfm_run_resectioning")
  {
    REQUIRE(sfm_run_resectioning() == 0);
  }

  TEST_CASE("sfm_run_isam2")
  {
    REQUIRE(sfm_run_isam2() == 0); // Someday figure out why this exceptions
  }

  TEST_CASE("sfm_test_clear")
  {
    REQUIRE(sfm_test_clear() == 0);
  }
#endif
}
