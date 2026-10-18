"""Retained-RPM shaft power using the frozen simulator propeller coefficients."""
import re, hashlib, json
from pathlib import Path
import numpy as np

SOURCE = Path(__file__).resolve().parents[2] / "simulation/crates/sim-core/src/propeller_data.rs"

assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == json.loads((Path(__file__).parent / "runtime-manifest.json").read_text())["propeller_data_sha256"], "Propeller table changed"

def table(name):
    body = re.search(rf"{name}: \[f64; \d+\] = \[(.*?)\];", SOURCE.read_text(), re.S)[1]
    return np.array([float(x.strip()) for x in body.split(",") if x.strip()])

RPM = table("RPM_GRID")
CP = table("CP_GRID").reshape(25, 401)

def shaft_power(rpm, axial):
    valid = (rpm >= RPM[0]) & (rpm < RPM[-1])
    i = np.clip(np.searchsorted(RPM, rpm, side='right')-1, 0, 23)
    f = (rpm-RPM[i])/(RPM[i+1]-RPM[i])
    advance = np.clip(np.maximum(axial, 0)/(np.maximum(rpm, 1e-9)/60*.2286), 0, 2)*200
    k = np.minimum(advance.astype(int), 399); g = advance-k
    cp = (CP[i, k]*(1-g)+CP[i, k+1]*g)*(1-f)+(CP[i+1, k]*(1-g)+CP[i+1, k+1]*g)*f
    return np.where(valid, 1.225*(rpm/60)**3*.2286**5*cp, 0)
