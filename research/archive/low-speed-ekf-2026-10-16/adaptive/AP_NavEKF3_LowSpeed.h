#pragma once

#include <AP_Math/AP_Math.h>

#ifndef AP_NAVEKF3_LOW_SPEED_DIAGNOSTICS
#define AP_NAVEKF3_LOW_SPEED_DIAGNOSTICS 0
#endif

namespace EKF3LowSpeed {

// Preserve the historical threshold for unspecified or conventional-speed vehicles.
inline float minimum_forward_tas(float minimum_eas, float eas2tas)
{
    if (!isfinite(minimum_eas) || !isfinite(eas2tas) || minimum_eas <= 0 || eas2tas <= 0) {
        return 5.0f;
    }
    return constrain_float(0.5f * minimum_eas * eas2tas, 1.0f, 5.0f);
}

inline bool airborne_evidence(float height_change, uint32_t time_flying_ms)
{
    return fabsf(height_change) > 3.0f || time_flying_ms > 5000;
}

// Fixed angular noise alone becomes an unrealistically tight lateral-velocity
// constraint at low speed. Add a velocity-noise floor, not wind-state uncertainty
// (the latter already participates in HPH').
inline float sideslip_variance(float forward_tas, float minimum_tas, float velocity_noise)
{
    const float base_variance = 0.03f;
    if (minimum_tas >= 5.0f) {
        return base_variance;
    }
    return base_variance + sq(velocity_noise / MAX(forward_tas, minimum_tas));
}

}
