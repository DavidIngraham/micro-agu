import type { Telemetry } from "@microagu/contracts/telemetry";

/** AP boot time and JSON simulation time share the same clock in this backend.
 * HAGL is already the EKF current output, not a delayed range measurement.
 * Propagate only its short telemetry age; never compensate sensor delay twice.
 */
export function landingHeight(state: Telemetry, now: number): number | null {
  const h = state.hagl;
  if (
    !h?.valid ||
    !Number.isFinite(h.metres) ||
    now - h.receivedTimeS > 0.3 ||
    now - (state.navigationTimeS ?? -Infinity) > 0.3
  )
    return null;
  const age = now - h.timeBootMs / 1000;
  if (!Number.isFinite(age) || age < -0.05 || age > 0.3) return null;
  return Math.max(0, h.metres + state.climb * Math.max(0, age));
}
