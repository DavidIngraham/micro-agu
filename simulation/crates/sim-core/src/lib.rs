//! Deterministic two-body parafoil simulation. SI units, NED world, FRD bodies.
//! Quaternions are body-to-world in xyzw order. No transport or filesystem I/O.
pub mod actuators;
pub mod atmosphere;
pub mod config;
pub mod dynamics;
pub mod integrator;
pub mod kernel;
pub mod launch;
pub mod propulsion;
pub mod sensors;
pub mod simulator;
pub mod state;
pub mod surface;
pub use config::{Parameters, SimulationConfig};
pub use simulator::Simulator;
pub type Result<T> = std::result::Result<T, String>;
