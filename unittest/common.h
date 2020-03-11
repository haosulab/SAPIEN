#pragma once

#define REQUIRE_NO_ERROR(sim)                                           \
  { REQUIRE(sim.mErrorCallback.getLastErrorCode() == PxErrorCode::eNO_ERROR); }
