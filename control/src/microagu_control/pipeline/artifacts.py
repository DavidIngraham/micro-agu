"""Strict gain-only interchange for the existing 16-state/3-integrator firmware."""

import ast
import hashlib
import json
import re
import struct
import zlib
from pathlib import Path

import numpy as np

SHAPES = {
    "A": (16, 16),
    "B": (16, 3),
    "K": (3, 19),
    "L": (16, 9),
    "XS": (16,),
    "US": (3,),
    "TRIM": (3,),
    "XREF": (16, 4),
    "UREF": (3, 4),
    "AW": (3, 3),
    "EULER": (3,),
    "VELOCITY": (3,),
    "RATES": (3,),
}
CONTEXT = [k for k in SHAPES if k not in ("K", "AW")]
MAGIC = b"AGULQI01"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def read_header(path):
    text = Path(path).read_text()
    arrays = {
        m[1]: np.array(
            ast.literal_eval(m[2].replace("{", "[").replace("}", "]").replace("f", "")),
            float,
        )
        for m in re.finditer(
            r"static constexpr float (\w+)(?:\[\d+\])+ = ([^;]+);", text
        )
    }
    for key, shape in SHAPES.items():
        if (
            key not in arrays
            or arrays[key].shape != shape
            or not np.isfinite(arrays[key]).all()
        ):
            raise ValueError(f"Unsupported controller layout: {key}, expected {shape}")
    if np.any(arrays["XS"] <= 0) or np.any(arrays["US"] <= 0):
        raise ValueError("Controller scales must be positive")
    return arrays


def context_id(arrays):
    # CRC is a compatibility check, not a cryptographic authenticity guarantee.
    return zlib.crc32(
        b"".join(np.asarray(arrays[k], dtype="<f4").tobytes() for k in CONTEXT)
    )


def gain_bytes(arrays):
    k = np.asarray(arrays["K"], dtype="<f4")
    aw = np.asarray(arrays["AW"], dtype="<f4")
    if (
        k.shape != (3, 19)
        or aw.shape != (3, 3)
        or not np.isfinite(k).all()
        or not np.isfinite(aw).all()
    ):
        raise ValueError("Invalid gain matrix")
    payload = k.tobytes() + aw.tobytes()
    return MAGIC + struct.pack("<II", context_id(arrays), zlib.crc32(payload)) + payload


def decode_gains(data, reference):
    if len(data) != 280 or data[:8] != MAGIC:
        raise ValueError("Gain file version or length mismatch")
    context, crc = struct.unpack("<II", data[8:16])
    payload = data[16:]
    if context != context_id(reference) or crc != zlib.crc32(payload):
        raise ValueError("Gain file context or CRC mismatch")
    a = np.frombuffer(payload, dtype="<f4").astype(float)
    if not np.isfinite(a).all():
        raise ValueError("Nonfinite gains")
    k = a[:57].reshape(3, 19)
    aw = a[57:].reshape(3, 3)
    if not np.allclose(
        aw @ (reference["US"][:, None] * k[:, 16:]), np.eye(3), atol=2e-3, rtol=0
    ):
        raise ValueError("Inconsistent antiwindup matrix")
    return k, aw


def export_header(template, arrays):
    text = Path(template).read_text()

    def val(x):
        return (
            "{" + ", ".join(val(v) for v in x) + "}"
            if np.ndim(x)
            else f"{float(x):.9e}f"
        )

    for key in ["K", "AW"]:
        text, n = re.subn(
            r"(static constexpr float " + key + r"(?:\[\d+\])+ = )[^;]+;",
            lambda m, key=key: m[1] + val(arrays[key]) + ";",
            text,
        )
        if n != 1:
            raise ValueError("Missing gain array")
    return text


def verify_run(folder):
    folder = Path(folder).resolve()
    manifest = json.loads((folder / "artifact-hashes.json").read_text())
    for name, expected in manifest.items():
        path = (folder / name).resolve()
        if not path.is_relative_to(folder) or sha(path) != expected:
            raise ValueError("Run artifact changed: " + name)
