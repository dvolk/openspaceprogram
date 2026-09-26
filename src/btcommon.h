// btcommon.h -- the single place that pulls Bullet's headers into the build.
//
// BT_USE_DOUBLE_PRECISION is a compile-time ABI switch: btScalar becomes
// double and every Bullet type grows, so EVERY translation unit that
// includes Bullet must see the same value. Before this file the contract
// lived in body.h (plus a copy in main.cpp, terrain.cpp and each test) --
// any one TU including a Bullet header without it compiled float-precision
// types that silently mismatch the double-precision ones elsewhere in the
// binary. Route the precision-critical includes through here instead
// (a couple of sub-headers, e.g. btHeightfieldTerrainShape, may still be
// included directly AFTER this, since the macro is already settled).
#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <btBulletDynamicsCommon.h>
