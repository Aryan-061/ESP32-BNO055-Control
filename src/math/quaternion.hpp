#pragma once
#include <cmath>
#include "state/state.hpp"

// Quaternion math function declarations ONLY

Quaternion quatConjugate(const Quaternion& q);

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b);

void quatNormalize(Quaternion& q);