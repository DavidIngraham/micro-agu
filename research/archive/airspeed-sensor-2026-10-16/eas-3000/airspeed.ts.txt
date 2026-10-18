/** Match AP_Baro's 1976 standard-atmosphere troposphere for SITL sensor input. */
export function equivalentAirspeed(trueAirspeed: number, altitudeAmsl: number) {
  if (
    !Number.isFinite(trueAirspeed) ||
    trueAirspeed < 0 ||
    !Number.isFinite(altitudeAmsl)
  )
    throw Error("Invalid airspeed or altitude");
  const earthRadius = 6356766;
  const height = (earthRadius * altitudeAmsl) / (earthRadius + altitudeAmsl);
  if (height < -5000 || height > 11000)
    throw Error("Browser airspeed conversion requires troposphere altitude");
  // AP_Baro/AP_Baro_atmosphere.cpp uses the -5 km base of this layer.
  const temperatureRatio = (320.65 - 0.0065 * (height + 5000)) / 320.65;
  const density =
    1.930467 * temperatureRatio ** (9.80665 / (0.0065 * 287.053072) - 1);
  return trueAirspeed * Math.sqrt(density / 1.225);
}
