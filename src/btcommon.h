// btcommon.h -- the single place that pulls Bullet's headers into the build.
// BT_USE_DOUBLE_PRECISION is a compile-time ABI switch: EVERY TU including
// Bullet must see the same value, so route Bullet includes through here.
#pragma once

#define BT_USE_DOUBLE_PRECISION true
#include <btBulletDynamicsCommon.h>
