# Rigging refinement study — protocol fixed before replay evaluation

Hypothesis: missing D-ring routing converts relative attitude into brake pull and explains some replay error. Compare the old model, measured 132 mm yaw spacing, routing alone, and both. Fixed neutral reference is each flight's existing straight-level equilibrium; never re-zero per replay. No hardware flights at 185 mm exist.

Existing development windows are split by alternating position: fit starts HR 300,480,690,930; TL 600,900,1920,2790; check starts HR 360,600,720,1020; TL 840,1350,2400,2910. Both partitions have been inspected in earlier research; check is a within-dataset generalization check, not independent validation. Fit uses 10 s, alpha .35; report 30 s and alpha .25/.35/.45.

Matched tuning budget for legacy and routed/132 mm models: common pitch/yaw damping multiplier .7,1,1.4; brake aerodynamic derivative multiplier .85,1,1.15; yaw stiffness multiplier 1 or 74.3/66 (18 candidates each). Objective is mean normalized roll/pitch/yaw rate RMSE, with .25 weights on bank and climb RMSE, normalized to legacy per-window metrics. Select on fit partition only. Ring height remains 60 mm assumption, sensitivity 20/40/80 mm; support/cascade points remain effective geometry, not photogrammetry.

Hardware extrapolation uses 185 mm risers, unchanged 91 mm guides, neutral re-tied, yaw k scaled by spacing ratio. Fixed-area canopy curvature sensitivity uses unchanged arc length and 0/half/full transfer of 53 mm attachment widening to projected canopy span; fixed panel area, no independent lift boost. Preserve empirical polar unless explicitly marked induced-drag sensitivity. Compare matched open-loop inputs from individual straight-level trim; the result is predicted plant response, not controller validation or a flight recording.

## Boundary extension
The initial fit selected the highest damping multiplier, 1.4. After inspecting that result and the initial check summaries, extend both models identically to 1.8 and 2.5. Select only on original fit windows. This is iterative development, not preregistered independent validation; retain both rounds. Stop at 2.5 rather than imply an identified optimum.
