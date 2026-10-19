
// AI-assisted isolated research policy. Confidence is a heuristic, not covariance.
void AP_CustomControl_LQI::prepare_startup(Input &in)
{
    auto &s = in.state;
    const bool raw_valid = s.wind_valid;
    const bool supported_mode = in.flight_mode == 13 || in.flight_mode == 10;
    if (!supported_mode) {
        _startup_started = false;
    }
    if (!_startup_started && in.flight_mode == 13 && in.eligible) {
        _startup_started = true;
        _startup_start_ms = in.time_ms;
        _startup_last_ms = in.time_ms;
        _startup_valid_ms = 0;
        _startup_confidence = 0.15f;
        _startup_wind = Vector3f(_wind_prior_n, _wind_prior_e, 0);
    }
    const float dt = constrain_float((in.time_ms-_startup_last_ms)*0.001f, 0.0f, 0.1f);
    _startup_last_ms = in.time_ms;
    const float age = (in.time_ms-_startup_start_ms)*0.001f;
    const bool policy_valid = _startup_mode >= 0 && _startup_mode <= 2 &&
        isfinite(_wind_prior_n.get()) && fabsf(_wind_prior_n.get()) <= 10 &&
        isfinite(_wind_prior_e.get()) && fabsf(_wind_prior_e.get()) <= 10 &&
        isfinite(_wind_delay.get()) && _wind_delay >= 0 && _wind_delay <= 60;
    if (!policy_valid) {
        in.eligible = false;
    }
    if (_startup_started && age < _wind_delay) {
        s.wind_valid = false; // Controller-side test delay; EKF is unchanged.
    }
    const bool completing_blend = _startup_mode == 2 && s.wind_valid && _startup_confidence < 1.0f;
    if (policy_valid && _startup_started && _startup_mode > 0 && (age < 45 || completing_blend)) {
        if (_startup_mode == 1) {
            if (!s.wind_valid) {
                s.wind_ned_mps.zero();
                s.wind_assumed = true;
                s.true_airspeed_mps = s.velocity_ned_mps.length();
            }
        } else {
            const float raw_airspeed = s.true_airspeed_mps;
            if (s.wind_valid) {
                if (_startup_valid_ms == 0) {
                    _startup_valid_ms = in.time_ms;
                }
                _startup_wind += (s.wind_ned_mps-_startup_wind)*(dt/(2.0f+dt));
                if (in.time_ms-_startup_valid_ms >= 1000) {
                    _startup_confidence = MIN(1.0f, _startup_confidence+dt/3.0f);
                }
            } else {
                _startup_valid_ms = 0;
                _startup_confidence = 0.15f;
            }
            s.wind_assumed = !s.wind_valid;
            s.wind_confidence = _startup_confidence;
            const float blend = s.wind_valid ? constrain_float((_startup_confidence-0.15f)/0.85f, 0.0f, 1.0f) : 0.0f;
            s.wind_ned_mps = _startup_wind+(s.wind_ned_mps-_startup_wind)*blend;
            const float prior_airspeed = (s.velocity_ned_mps-s.wind_ned_mps).length();
            s.true_airspeed_mps = s.wind_valid ?
                (1-blend)*prior_airspeed+blend*raw_airspeed :
                prior_airspeed;
        }
    }
    if (in.time_ms-_startup_log_ms >= 100) {
        _startup_log_ms = in.time_ms;
        gcs().send_named_float("TKRAW", raw_valid ? 1.0f : 0.0f);
        gcs().send_named_float("TKWQ", s.wind_confidence);
        gcs().send_named_float("TKUSE", s.wind_assumed ? 1.0f : 0.0f);
        gcs().send_named_float("TKWN", s.wind_ned_mps.x);
        gcs().send_named_float("TKWE", s.wind_ned_mps.y);
    }
}

void AP_CustomControl_LQI::weight_velocity(float *value, const Input &in) const
{
    if (in.state.wind_confidence >= 1.0f) {
        return;
    }
    Matrix3f rotation;
    in.state.attitude_body_to_ned.rotation_matrix(rotation);
    Vector3f ned = rotation*Vector3f(value[0]*XS[5], value[1]*XS[6], value[2]*XS[7]);
    ned.x *= in.state.wind_confidence;
    ned.y *= in.state.wind_confidence;
    const Vector3f body = rotation.transposed()*ned;
    value[0] = body.x/XS[5];
    value[1] = body.y/XS[6];
    value[2] = body.z/XS[7];
}
