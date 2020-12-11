

#include "catch2/catch.hpp"

#include "../sho/sho_run.hpp"

namespace camsim
{
#if 1
  TEST_CASE("sho_test - shonan_rotation_averaging")
  {
    REQUIRE(shonan_rotation_averaging() == 0);
  }

  TEST_CASE("sho_test - shonan_RA_simple")
  {
    REQUIRE(shonan_RA_simple() == 0);
  }

  TEST_CASE("sho_test - test_rotate3()")
  {
    REQUIRE(test_rotate3() == 0);
  }

  TEST_CASE("sho_test - inter_marker_rotation()")
  {
    REQUIRE(inter_marker_rotation() == 0);
  }
#endif
}

