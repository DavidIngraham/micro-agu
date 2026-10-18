# Reproducing the longitudinal campaign

Use the microAGU revision containing this study, Node 24, the viewer npm dependencies, and the installed Rust Python binding (uv sync in simulation). Python analysis requires NumPy and SciPy. No other research directory is imported.

1. Run python research/lqi-longitudinal-2026-10-17/prepare_runtimes.py. It restores the original runtime from Git revision 058a8df and the fixed runtime from the release revision, and verifies hashes. On a later checkout pass --fixed-ref with the release commit.
2. Extract raw-runs.tar.gz into this study directory to reproduce metrics without rerunning flights. Run python research/lqi-longitudinal-2026-10-17/report.py.
3. For new flight execution, use a separate checkout with no extracted runs directory. campaign.py reuses existing results. Run phases screen, heldout, final, diagnostic, bug, fix-check, fixed in that order. CASE_WORKERS=3 is the default concurrency. Node must be available on PATH; the runner also knows the original host's Node 24 installation.
4. Run simulation/.venv/bin/python research/lqi-longitudinal-2026-10-17/trim.py after the screen flights. It uses their trajectories only as additional numerical seeds. trims-initial.json preserves the earlier grid without those seeds.

The frozen *-specs.json files, protocol.json, selection.json, physics-config.json, and runtime-manifest.json are authoritative. Runtime paths in archived specifications record the original host; the runner resolves them to the local study directory. The Rust physics binary and propeller table hashes must match the manifest. firmware.patch is the complete ArduPilot patch against 010ada6a4f0b26910fc18ecca176d2e65c5ed9d7.

To rebuild firmware, apply that patch to its base, initialize ArduPilot submodules, source Emscripten 6.0.8, set EMCC_CFLAGS=-sEXPORTED_RUNTIME_METHODS=cwrap,HEAPU8,FS,SOCKFS,PThread, then run ./waf configure --board wasm and ./waf plane. The committed binary is the tested artifact; a rebuilt binary requires hash verification or revalidation.

The native trim solver and browser trajectory evaluation serve different purposes. Failed trim convergence is not a physical envelope boundary. Battery and aerodynamic variations are simulated, and reported shaft energy excludes electrical and servo losses.
