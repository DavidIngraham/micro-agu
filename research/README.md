# Research index

Maintained software lives in [simulation](../simulation/README.md), [control](../control/README.md) and [viewer](../viewer/README.md). The [current aircraft manifest](../aircraft/microagu/current.json) identifies what is deployed.

`active/` contains ongoing model-fidelity and identification work. `archive/` contains completed or superseded evidence; it is excluded from normal test discovery and is not a collection of maintained applications. See [catalog.json](catalog.json) for dispositions and original conclusions.

## Active studies

| Study | Location |
|---|---|
| brake-tension-feedback-2026-10-18 | [brake-tension-feedback-2026-10-18](active/brake-tension-feedback-2026-10-18/) |
| controller-pipeline-2026-10-18 | [controller-pipeline-2026-10-18](active/controller-pipeline-2026-10-18/) |
| coupled-brake-2026-10-18 | [coupled-brake-2026-10-18](active/coupled-brake-2026-10-18/) |
| coupled-oscillation-2026-10-18 | [coupled-oscillation-2026-10-18](active/coupled-oscillation-2026-10-18/) |
| flight-fidelity-2026-10-18 | [flight-fidelity-2026-10-18](active/flight-fidelity-2026-10-18/) |
| gradient-fit-2026-10-18 | [gradient-fit-2026-10-18](active/gradient-fit-2026-10-18/) |
| greybox-pipeline-2026-10-18 | [greybox-pipeline-2026-10-18](active/greybox-pipeline-2026-10-18/) |
| hood-river-pipeline-2026-10-18 | [hood-river-pipeline-2026-10-18](active/hood-river-pipeline-2026-10-18/) |
| manual-coupling-fit-2026-10-18 | [manual-coupling-fit-2026-10-18](active/manual-coupling-fit-2026-10-18/) |
| missing-physics-2026-10-18 | [missing-physics-2026-10-18](active/missing-physics-2026-10-18/) |
| trout-lake-dynamics-2026-10-18 | [trout-lake-dynamics-2026-10-18](active/trout-lake-dynamics-2026-10-18/) |
| trout-lake-fidelity-2026-10-18 | [trout-lake-fidelity-2026-10-18](active/trout-lake-fidelity-2026-10-18/) |

## Historical evidence

Retained studies are under [archive/](archive/README.md). Flight figures, blog inputs, failure cases and evidence supporting the current envelope remain available. Their original reproduction environments are preserved by Git tag `archive/pre-layout-2026-10-18`; historical source hashes and raw result manifests have not been rewritten.

## New work

Create one study directory with a question, inputs, protocol, result and disposition. Use the installed `microagu_control` and `microagu_sim` packages; do not copy their implementations. Keep raw logs and generated firmware out of Git. Record their hashes and archive locations when they cannot be regenerated. Promote useful scenarios into maintained tests, then close the study with a short conclusion and successor link.

```sh
uv sync --project research --locked
uv run --project research python research/active/greybox-pipeline-2026-10-18/study.py
```

Studies may require flight inputs described in their own instructions. A successful package build is not a claim that every historical campaign is reproducible without those inputs.

- [Pitch inertia and damping study](active/pitch-inertia-damping-2026-10-18/README.md): 450 replays, mode/decay and initialization checks; missing coupled oscillation remains unresolved.

- [ESC RPM harmonic audit](active/rpm-harmonics-2026-10-18/README.md): shaft/blade-pass alias comparisons, original binary log inventory, and measurement limitations.

- [Propulsion RPM audit](active/propulsion-rpm-audit-2026-10-18/README.md): measured versus predicted RPM, telemetry slew, and native replay checks.

- [Whole-flight oscillation timing](active/oscillation-timing-2026-10-18/README.md): full-flight FFT, time histories and command-excitation association.

- [Asymmetric canopy-twist experiment](active/canopy-twist-2026-10-18/README.md): deformation feedback, cross-flight comparisons and stability checks; no production promotion.

- [Powered versus stopped-prop modes](active/power-off-modes-2026-10-18/README.md): flight spectra, trim sweep, gyro ablation, and numerical checks.
