export interface Weather {
  /** Omitted selects the archived OU model for existing experiments. */
  turbulenceModel?: "legacy" | "dryden";
  /** Dryden component length scale in metres. */
  gustLength?: number;
  /** Vertical RMS divided by horizontal component RMS. */
  verticalRatio?: number;
  /** Optional nonzero uint32 seed; supplying it restarts the synthetic gust sequence. */
  seed?: number;
  /** Horizontal gust RMS in m/s; vertical RMS is half this value. */
  turbulence: number;
  /** Upward speed at the centre of a ground-fixed Gaussian thermal. */
  thermalStrength: number;
  thermalRadius: number;
  thermalNorth: number;
  thermalEast: number;
}
export const calmWeather: Weather = {
  turbulence: 0,
  thermalStrength: 0,
  thermalRadius: 40,
  thermalNorth: 0,
  thermalEast: 0,
};
