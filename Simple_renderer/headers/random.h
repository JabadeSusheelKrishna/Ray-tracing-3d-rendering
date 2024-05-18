#pragma once

#include <cstdlib>
#include <limits>

// Utility Functions
inline float next_float() {
    // Returns a random real in [0,1).
    return (float)(rand() / (RAND_MAX + 1.0));
}