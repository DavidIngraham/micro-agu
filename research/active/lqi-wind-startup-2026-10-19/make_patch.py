import difflib
from pathlib import Path
P=Path(__file__).resolve().parent
AP=Path('/home/david/repos/ardupilot-lqi-pipeline')
names=['ArduPlane/Plane.cpp','libraries/AP_CustomControl/AP_CustomControl_MIMO.h','libraries/AP_CustomControl/AP_CustomControl_LQI.h','libraries/AP_CustomControl/AP_CustomControl_LQI.cpp']
old={n:(AP/n).read_text() for n in names}
new=old.copy()
n=names[0];s=new[n]
s=s.replace('flight_stage == AP_FixedWing::FlightStage::NORMAL &&','(flight_stage == AP_FixedWing::FlightStage::NORMAL ||\n         (control_mode == &mode_takeoff && flight_stage == AP_FixedWing::FlightStage::TAKEOFF)) &&')
s=s.replace('state.wind_valid = ahrs.get_wind(state.wind_ned_mps);','state.wind_valid = ahrs.get_wind(state.wind_ned_mps);\n    state.wind_assumed = false;\n    state.wind_confidence = 1.0f;')
s=s.replace('gcs().send_named_float("MIMO", custom_control.mimo_running() ? 1.0f : 0.0f);','gcs().send_named_float("MIMO", custom_control.mimo_running() ? 1.0f : 0.0f);\n    gcs().send_named_float("TKST", flight_stage == AP_FixedWing::FlightStage::TAKEOFF ? 1.0f : 0.0f);')
new[n]=s
n=names[1];new[n]=new[n].replace('    bool wind_valid;','    bool wind_valid;\n    bool wind_assumed;\n    float wind_confidence;')
n=names[2];new[n]=new[n].replace('    AP_Int8 _use_file;',"""    // Research-only takeoff initialization; selected gain file is unchanged.
    AP_Int8 _startup_mode;
    AP_Float _wind_prior_n;
    AP_Float _wind_prior_e;
    AP_Float _wind_delay;
    bool _startup_started;
    uint32_t _startup_start_ms;
    uint32_t _startup_last_ms;
    uint32_t _startup_valid_ms;
    uint32_t _startup_log_ms;
    float _startup_confidence;
    Vector3f _startup_wind;
    void prepare_startup(AP_CustomControl_MIMO::Input &input);
    void weight_velocity(float *value, const AP_CustomControl_MIMO::Input &input) const;
    AP_Int8 _use_file;""")
n=names[3];s=new[n]
params=""
for index,name,member,default,desc,extra in [
    (10,'STMODE','_startup_mode',0,'Research-only takeoff startup policy. Retains checks except explicit temporary wind assumption.','    // @Values: 0:Classical fallback,1:Zero wind full confidence,2:Weighted wind prior\n'),
    (11,'WPRN','_wind_prior_n',0,'Research-only north wind prior, positive toward north. Used only by weighted startup.','    // @Units: m/s\n    // @Range: -10 10\n'),
    (12,'WPRE','_wind_prior_e',0,'Research-only east wind prior, positive toward east. Used only by weighted startup.','    // @Units: m/s\n    // @Range: -10 10\n'),
    (13,'WDLY','_wind_delay',0,'Research-only withholding of controller wind availability after takeoff eligibility. Does not modify EKF states.','    // @Units: s\n    // @Range: 0 60\n')]:
    params+=f'    // @Param: {name}\n    // @DisplayName: Research startup {name}\n    // @Description: {desc}\n'+extra+f'    // @User: Advanced\n    AP_GROUPINFO("{name}", {index}, AP_CustomControl_LQI, {member}, {default}),\n'
s=s.replace('    AP_GROUPEND',params+'    AP_GROUPEND',1)
s=s.replace('s.airspeed_valid && s.wind_valid &&','s.airspeed_valid && (s.wind_valid || s.wind_assumed) &&\n        isfinite(s.wind_confidence) && s.wind_confidence >= 0 && s.wind_confidence <= 1 &&')
s=s.replace('bool AP_CustomControl_LQI::can_run()\n{','bool AP_CustomControl_LQI::can_run()\n{\n    prepare_startup(_frontend.mimo_input);')
s=s.replace('    measurements(in, y);\n    const uint8_t indices[9]','    measurements(in, y);\n    weight_velocity(&y[2], in);\n    const uint8_t indices[9]',1)
s=s.replace('turn_trim(degrees(turn),in.state.true_airspeed_mps,observer_x,observer_u);',"""const float confidence = in.state.wind_confidence;
        const float observer_speed = confidence*in.state.true_airspeed_mps +
            (1-confidence)*in.reference.equivalent_airspeed_mps*in.state.eas_to_tas;
        turn_trim(degrees(turn),observer_speed,observer_x,observer_u);""")
needle="""    for (uint8_t i = 0; i < 16; i++) {
        _state[i] = predicted[i];"""
replacement="""    float innovation[9];
    for (uint8_t j = 0; j < 9; j++) {
        innovation[j] = y[j]-predicted[indices[j]];
    }
    weight_velocity(&innovation[2], in);
    for (uint8_t i = 0; i < 16; i++) {
        _state[i] = predicted[i];"""
assert needle in s;s=s.replace(needle,replacement)
s=s.replace('L[i][j]*(y[j]-predicted[indices[j]])','L[i][j]*innovation[j]')
s=s.replace('const float turn = constrain_float(in.reference.heading_rate_rads, -radians(_turn_max), radians(_turn_max));',"""const float turn_limit = MIN(_turn_max.get(), 10.0f+25.0f*in.state.wind_confidence);
    const float turn = constrain_float(in.reference.heading_rate_rads, -radians(turn_limit), radians(turn_limit));""")
s=s.replace('(in.state.true_airspeed_mps-speed_target)/0.5f','in.state.wind_confidence*(in.state.true_airspeed_mps-speed_target)/0.5f')
needle="""    for (uint8_t i = 0; i < 3; i++) {
        float feedback = 0;
        for (uint8_t j = 0; j < 16; j++) {
            feedback += feedback_gain(i, j)*(_state[j]-target_state[j]);
        }"""
replacement="""    float feedback_state[16];
    for (uint8_t j = 0; j < 16; j++) {
        feedback_state[j] = _state[j]-target_state[j];
    }
    weight_velocity(&feedback_state[5], in);
    for (uint8_t i = 0; i < 3; i++) {
        float feedback = 0;
        for (uint8_t j = 0; j < 16; j++) {
            feedback += feedback_gain(i, j)*feedback_state[j];
        }"""
assert needle in s;s=s.replace(needle,replacement)
idx=s.rfind('#endif')
assert idx>=0
s=s[:idx]+s[idx+len('#endif'):]
s+=(P/'startup-methods.cpp').read_text()
s+='\n#endif // AP_PLANE_CUSTOMCONTROL_LQI_ENABLED\n'
new[n]=s
patch=''.join(''.join(difflib.unified_diff(old[n].splitlines(True),new[n].splitlines(True),fromfile='a/'+n,tofile='b/'+n)) for n in names)
(P/'startup.patch').write_text(patch)
