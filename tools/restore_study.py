"""Verify a compact study archive or restore it into a new directory."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile


def safe(root, name):
    part = PurePosixPath(name)
    if part.is_absolute() or ".." in part.parts or not part.parts:
        raise ValueError(f"Unsafe archive path: {name}")
    target = root.joinpath(*part.parts)
    if not target.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"Archive path escapes root: {name}")
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--verify", action="store_true")
    group.add_argument("--destination", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.study / "evidence-manifest.json").read_text())
    archive = safe(args.study, manifest["archive"])
    if hashlib.sha256(archive.read_bytes()).hexdigest() != manifest["sha256"]:
        raise ValueError("Study archive checksum mismatch")
    contents = {}
    with tarfile.open(archive, "r:gz") as tar:
        expected = {e["path"]: e for e in manifest["files"]}
        if len(expected) != len(manifest["files"]) or set(tar.getnames()) != set(expected):
            raise ValueError("Study archive inventory mismatch")
        for name, entry in expected.items():
            safe(args.study, name)
            member = tar.getmember(name)
            if not member.isfile():
                raise ValueError("Only ordinary evidence files are supported")
            data = tar.extractfile(member).read()
            if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
                raise ValueError(f"Evidence checksum mismatch: {name}")
            contents[name] = data
    if args.destination:
        # Validate all inputs before creating anything; never overwrite a destination.
        retained = {name: safe(args.study, name).read_bytes() for name in manifest["retained"]}
        for name in contents.keys() | retained.keys():
            safe(args.destination, name)
        args.destination.mkdir(parents=True, exist_ok=False)
        for name, data in (retained | contents).items():
            target = safe(args.destination, name)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
    print(f"Verified {len(contents)} archived files" + (f"; restored {args.destination}" if args.destination else ""))


if __name__ == "__main__":
    main()
