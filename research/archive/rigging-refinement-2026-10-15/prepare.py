"""Verify the installed Rust model is ready; uv handles building the wheel."""
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat import dynamics

for flight in ['hood_river', 'trout_lake']:
    sim = make_simulator(flight)
    assert len(dynamics.params(sim.parameters)) == 70
print('Rust simulator ready. Run this study with research/uv run.')
