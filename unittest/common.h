#pragma once
#include <cstdlib>

#define REQUIRE_NO_ERROR(sim)                                                                     \
  { REQUIRE(sim.mErrorCallback.getLastErrorCode() == PxErrorCode::eNO_ERROR); }

#define REQUIRE_CLOSE(a, b)                                                                       \
  { REQUIRE(abs((a) - (b) < 1e-8)); }
