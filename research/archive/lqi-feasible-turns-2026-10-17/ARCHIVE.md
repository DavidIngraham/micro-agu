# Archived evidence

123 nested case/configuration artifacts are preserved in `evidence.tar.gz`, with individual hashes in `evidence-manifest.json`. Summaries and directly linked artifacts remain beside this file.

Verify without extracting:

```sh
python3 tools/restore_study.py research/archive/lqi-feasible-turns-2026-10-17 --verify
```

Restore the study into a new directory with `--destination /tmp/lqi-feasible-turns-2026-10-17`. Historical runners retain their original assumptions; extraction does not make them part of the maintained test suite.
