"""Cross-language tests against the actual AP decoder; set MICROAGU_ARDUPILOT."""

import os
import struct
import subprocess
import zlib
from pathlib import Path

import numpy as np
import pytest
from microagu_sim.pipeline.artifacts import SHAPES, context_id, gain_bytes


@pytest.fixture(scope="module")
def decoder(tmp_path_factory):
    ap = os.environ.get("MICROAGU_ARDUPILOT")
    if not ap:
        pytest.skip("Set MICROAGU_ARDUPILOT for the cross-language decoder tests")
    folder = tmp_path_factory.mktemp("decoder")
    source = folder / "probe.cpp"
    exe = folder / "probe"
    source.write_text(r"""#include "AP_CustomControl_LQI_File.h"
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdlib>
int main(int argc, char **argv) {
    if (argc != 3) return 3;
    std::ifstream input(argv[1], std::ios::binary);
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)), {});
    const float scale[3] = {.1f,.1f,.1f};
    MicroAGU_LQI_File::Gains result{};
    result.aw[0][0] = 12345;
    bool valid=MicroAGU_LQI_File::decode(bytes.data(),bytes.size(),std::strtoul(argv[2],nullptr,10),scale,result);
    if (!valid && result.aw[0][0] != 12345) return 4;
    return valid ? 0 : 2;
}
""")
    subprocess.run(
        [
            "g++",
            "-std=c++11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(Path(ap) / "libraries/AP_CustomControl"),
            str(source),
            "-o",
            str(exe),
        ],
        check=True,
    )
    return exe


@pytest.mark.parametrize(
    "damage", ["none", "context", "crc", "length", "extra", "nan", "antiwindup"]
)
def test_actual_firmware_decoder(decoder, tmp_path, damage):
    z = {k: np.zeros(shape) for k, shape in SHAPES.items()}
    z["XS"][:] = 1
    z["US"][:] = 0.1
    z["K"][:, 16:] = np.eye(3)
    z["AW"] = np.eye(3) * 10
    raw = gain_bytes(z)
    context = context_id(z)
    if damage == "context":
        context ^= 1
    if damage == "crc":
        raw = raw[:-1] + bytes([raw[-1] ^ 1])
    if damage == "length":
        raw = raw[:-1]
    if damage == "extra":
        raw += b"x"
    if damage in ["nan", "antiwindup"]:
        payload = bytearray(raw[16:])
        struct.pack_into(
            "<f",
            payload,
            0 if damage == "nan" else 57 * 4,
            float("nan") if damage == "nan" else 0,
        )
        raw = raw[:12] + struct.pack("<I", zlib.crc32(payload)) + payload
    file = tmp_path / "lqi-gains.bin"
    file.write_bytes(raw)
    rc = subprocess.run([str(decoder), str(file), str(context)], check=False).returncode
    assert rc == (0 if damage == "none" else 2)
