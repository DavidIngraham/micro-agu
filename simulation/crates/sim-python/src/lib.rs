use numpy::{IntoPyArray, PyArray2, PyReadonlyArray1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};
use sim_core::{kernel, state::*};
#[gen_stub_pyclass]
#[pyclass(module = "microagu_sim._native")]
struct Dryden {
    inner: sim_core::atmosphere::Dryden,
}
#[gen_stub_pymethods]
#[pymethods]
impl Dryden {
    #[new]
    fn new(seed: u32) -> PyResult<Self> {
        Ok(Self {
            inner: sim_core::atmosphere::Dryden::new(seed).map_err(err)?,
        })
    }
    fn step(
        &mut self,
        dt: f64,
        speed: f64,
        lengths: Vec<f64>,
        sigma: Vec<f64>,
    ) -> PyResult<Vec<f64>> {
        self.inner
            .step(
                dt,
                speed,
                kernel::array(&lengths).map_err(err)?,
                kernel::array(&sigma).map_err(err)?,
            )
            .map(|v| v.to_vec())
            .map_err(err)
    }
}
fn err(e: String) -> PyErr {
    PyValueError::new_err(e)
}
#[gen_stub_pyclass]
#[pyclass(module = "microagu_sim._native")]
struct Kernel {
    inner: kernel::Kernel,
}
#[gen_stub_pymethods]
#[pymethods]
impl Kernel {
    fn inverted_launch(
        &self,
        trim: Vec<f64>,
        time_s: f64,
        hand_height_m: f64,
    ) -> PyResult<Vec<f64>> {
        self.inner
            .inverted_launch(&trim, time_s, hand_height_m)
            .map_err(err)
    }
    #[new]
    fn new(parameters: Vec<f64>, mass: f64) -> PyResult<Self> {
        Ok(Self {
            inner: kernel::Kernel::new(&parameters, mass).map_err(err)?,
        })
    }
    fn derivative(&self, state: Vec<f64>, command: Vec<f64>, wind: Vec<f64>) -> PyResult<Vec<f64>> {
        self.inner.derivative(&state, &command, &wind).map_err(err)
    }
    #[allow(clippy::too_many_arguments)]
    fn replay(
        &self,
        state: Vec<f64>,
        commands: Vec<f64>,
        wind: Vec<f64>,
        voltage: f64,
        rpm: f64,
        dt: f64,
        substeps: usize,
        causal: bool,
    ) -> PyResult<Vec<f64>> {
        self.inner
            .replay(&state, &commands, &wind, voltage, rpm, dt, substeps, causal)
            .map_err(err)
    }
    #[allow(clippy::too_many_arguments)]
    fn surface_step(
        &self,
        state: Vec<f64>,
        command: Vec<f64>,
        wind: Vec<f64>,
        voltage: f64,
        rpm: f64,
        dt: f64,
        env: Vec<f64>,
        settings: Vec<f64>,
    ) -> PyResult<Vec<f64>> {
        self.inner
            .surface_step(&state, &command, &wind, voltage, rpm, dt, &env, &settings)
            .map_err(err)
    }
    fn brake_paths(&self, state: Vec<f64>) -> PyResult<Vec<f64>> {
        self.inner.brake_paths(&state).map_err(err)
    }
    /// Batch replay with contiguous float64 input and an owned Nx27 NumPy result.
    #[allow(clippy::too_many_arguments)]
    fn replay_numpy<'py>(
        &self,
        py: Python<'py>,
        state: PyReadonlyArray1<'py, f64>,
        commands: PyReadonlyArray1<'py, f64>,
        wind: PyReadonlyArray1<'py, f64>,
        voltage: f64,
        rpm: f64,
        dt: f64,
        substeps: usize,
        causal: bool,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let result = self
            .inner
            .replay(
                state.as_slice()?,
                commands.as_slice()?,
                wind.as_slice()?,
                voltage,
                rpm,
                dt,
                substeps,
                causal,
            )
            .map_err(err)?;
        let rows = result.len() / 27;
        let array = numpy::ndarray::Array2::from_shape_vec((rows, 27), result)
            .map_err(|e| err(e.to_string()))?;
        Ok(array.into_pyarray(py))
    }
}
#[gen_stub_pyclass]
#[pyclass(module = "microagu_sim._native", name = "Simulator")]
struct PySimulator {
    inner: sim_core::Simulator,
}
#[gen_stub_pymethods]
#[pymethods]
impl PySimulator {
    #[new]
    fn new(config_json: &str) -> PyResult<Self> {
        Ok(Self {
            inner: sim_core::Simulator::new(sim_io::parse_config(config_json).map_err(err)?)
                .map_err(err)?,
        })
    }
    fn step(&mut self, command: Vec<f64>, wind: Vec<f64>, dt_s: f64) -> PyResult<Vec<f64>> {
        self.inner
            .step(
                kernel::array(&command).map_err(err)?,
                kernel::array(&wind).map_err(err)?,
                dt_s,
            )
            .map_err(err)?;
        Ok(self.inner.state.as_slice().to_vec())
    }
    #[getter]
    fn state(&self) -> Vec<f64> {
        self.inner.state.as_slice().to_vec()
    }
    #[getter]
    fn rpm(&self) -> f64 {
        self.inner.rpm
    }
    #[getter]
    fn time_s(&self) -> f64 {
        self.inner.time_s
    }
    fn ground_start(&mut self) {
        self.inner.ground_start();
    }
    fn release(&mut self) {
        self.inner.release();
    }
}
#[gen_stub_pyfunction]
#[pyfunction]
fn mapped_thrust(rpm: f64, axial: f64) -> f64 {
    sim_core::propulsion::thrust(rpm, axial)
}
#[gen_stub_pyfunction]
#[pyfunction]
fn mapped_torque(rpm: f64, axial: f64) -> f64 {
    sim_core::propulsion::torque(rpm, axial)
}
#[gen_stub_pyfunction]
#[pyfunction]
fn rotor_loads(
    omega: Vec<f64>,
    rotation: Vec<f64>,
    inertia: f64,
    rpm: f64,
    rate: f64,
) -> PyResult<Vec<f64>> {
    let w = V::from(kernel::array(&omega).map_err(err)?);
    let r = M::from_column_slice(&kernel::array::<9>(&rotation).map_err(err)?);
    Ok(sim_core::dynamics::rotor_moment(w, r, inertia, rpm, rate)
        .as_slice()
        .to_vec())
}
#[gen_stub_pyfunction]
#[pyfunction]
fn config_schema() -> String {
    sim_io::config_schema()
}
#[gen_stub_pyfunction]
#[pyfunction]
fn parameter_keys() -> Vec<String> {
    sim_core::Parameters::KEYS
        .iter()
        .map(|s| s.to_string())
        .collect()
}
#[gen_stub_pyfunction]
#[pyfunction]
fn parameter_defaults_json() -> String {
    serde_json::to_string(&sim_core::Parameters::default()).expect("finite defaults")
}
#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Dryden>()?;
    m.add_function(wrap_pyfunction!(parameter_defaults_json, m)?)?;
    m.add_class::<Kernel>()?;
    m.add_class::<PySimulator>()?;
    m.add_function(wrap_pyfunction!(mapped_thrust, m)?)?;
    m.add_function(wrap_pyfunction!(mapped_torque, m)?)?;
    m.add_function(wrap_pyfunction!(rotor_loads, m)?)?;
    m.add_function(wrap_pyfunction!(config_schema, m)?)?;
    m.add_function(wrap_pyfunction!(parameter_keys, m)?)?;
    Ok(())
}
pyo3_stub_gen::define_stub_info_gatherer!(stub_info);
