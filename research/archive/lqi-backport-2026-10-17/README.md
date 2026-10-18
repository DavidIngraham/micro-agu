# Original LQI reference-filter backport

AI-assisted study, 2026-10-17. Tests the selected 0.7 s reference filter with existing gains and the original observer, without pitot or canopy sensors. See [protocol](protocol.json) and [results](RESULTS.md).

Two baselines are deliberately kept separate: the current local non-enhanced controller (zero common trim and subsequent local EKF work), and the live web simulator's earlier controller at AP revision `010ada6a4f0b26910fc18ecca176d2e65c5ed9d7` (10% common trim). The latter is the deployment candidate; success on local research code alone cannot qualify a web release.

Each pair uses the same firmware with CP3_HRTC set to 0 or 0.7. Existing ±0.5 m/s reference limits, 2 s approach constant, gains, observer, TECS, EKF, and as-built physics remain unchanged within a pair. Three training cases and eight fresh validation cases compare commands and gusts; eight fresh AUTO/CRUISE missions check tracking and late motion. The candidate is fixed in advance; no retuning uses validation data.

Acceptance requires all operational screens, at least 5% median paired score improvement in both transient sets, no task-loss median regression above 20%, no acquisition-time regression above 10%, no energy/power regression above 5%, and mission tracking median ratios below 1.2 with a 1 m denominator floor. The score includes command-induced canopy motion; it is inherited from the transition study. All raw cases, firmware and source hashes are retained. `campaign.py` restores pre-existing dirty files and refuses to overwrite snapshots.

Only a qualified deployed-cohort candidate may be published. Publication must preserve the existing web physics, gains and frontend, include the exact source patch and provenance, and verify the deployed artifact hashes. The enhanced sensor model is not part of this backport.

## Outcome

**Subsequent test correction:** the [filter-retune study](../lqi-filter-retune-2026-10-17/README.md) found that this study's fixed 90° earth-bearing gust reached paired aircraft at materially different headings. Its crosswind loss ratios confound controller response with gust incidence and should not be interpreted as a causal filter penalty or benefit. Raw results remain intact. The follow-up reruns both baselines and candidates with a lateral gust defined relative to aircraft heading at onset; the original command-step results are unaffected by this specific confound.

All 76 runs completed. The pre-promotion audit verified restoration of 25 existing files, unchanged gains, no extra sensors, and physics/configuration matching the live simulator. The local cohort passed acceptance; its source patch and tested firmware are retained locally with `CP3_HRTC=0.7` in the canonical parameter file and browser configuration. `local-promotion.json` records the resulting hashes. This is a local research build, not a public release.

Across eight fresh local validation pairs, median paired score improved 20.2%, relative pitch-rate peaks fell 83–84%, and modeled shaft energy per distance fell 2.4%. All eight scores improved (12.6–34.2%). Energy improved in six cases; the worst increase was 0.94%. Acquisition time was essentially unchanged, but acquisition/descent task losses increased 2.3%/8.0%. Mission tracking is not uniformly better; see the dimensional tables and all-case plots in [RESULTS.md](RESULTS.md).

**No web release:** the earlier deployed tune failed the same acceptance rule. Its held-out score improved only 2.6%, crosswind loss increased 76.4%, modeled energy increased 2.1%, and CRUISE mission power exceeded the 5% regression limit. The filter smooths commanded transitions on both tunes, but does not establish improved disturbance rejection. Existing local zero-trim-brake and EKF changes were not bundled into a release to bypass that failure.

Recompute the report with `python report.py`, `python gust_details.py`, and `python plot.py` in this directory using the study's numpy/scipy/matplotlib environment. `campaign.py` refuses to overwrite its original snapshot; a rerun needs a new study directory and baseline. `audit.json` describes the restored state before the intentional local promotion; `audit.py` is not expected to pass after promotion. The conditional release preparation and browser smoke scripts were not executed because acceptance failed.
