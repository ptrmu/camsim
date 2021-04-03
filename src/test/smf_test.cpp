

#include "catch2/catch.hpp"

#include "../smf/smf_run.hpp"

TEST_CASE("sfm_example_smart_factor - run - zero", "[all]")
{
  REQUIRE(sfm_example_smart_factor() == 0);
}

