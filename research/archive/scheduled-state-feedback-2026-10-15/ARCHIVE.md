# Archived evidence

22 nested case/configuration artifacts are preserved in `evidence.tar.gz`, with individual hashes in `evidence-manifest.json`. Summaries and directly linked artifacts remain beside this file.

Verify without extracting:

```sh
python3 tools/restore_study.py research/archive/scheduled-state-feedback-2026-10-15 --verify
```

Restore the study into a new directory with `--destination /tmp/scheduled-state-feedback-2026-10-15`. Historical runners retain their original assumptions; extraction does not make them part of the maintained test suite.
