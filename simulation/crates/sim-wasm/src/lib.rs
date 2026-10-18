use sim_core::kernel;
use wasm_bindgen::prelude::*;
#[wasm_bindgen]
pub struct Dryden {
    inner: sim_core::atmosphere::Dryden,
}
#[wasm_bindgen]
impl Dryden {
    #[wasm_bindgen(constructor)]
    pub fn new(seed: u32) -> Result<Dryden, JsValue> {
        Ok(Self {
            inner: sim_core::atmosphere::Dryden::new(seed).map_err(error)?,
        })
    }
    pub fn step(
        &mut self,
        dt: f64,
        speed: f64,
        lengths: &[f64],
        sigma: &[f64],
    ) -> Result<Vec<f64>, JsValue> {
        self.inner
            .step(
                dt,
                speed,
                kernel::array(lengths).map_err(error)?,
                kernel::array(sigma).map_err(error)?,
            )
            .map(|v| v.to_vec())
            .map_err(error)
    }
}
fn error(e: String) -> JsValue {
    JsValue::from_str(&e)
}
#[wasm_bindgen]
pub struct Kernel {
    inner: kernel::Kernel,
}
#[wasm_bindgen]
impl Kernel {
    pub fn inverted_launch(
        &self,
        trim: &[f64],
        time_s: f64,
        hand_height_m: f64,
    ) -> Result<Vec<f64>, JsValue> {
        self.inner
            .inverted_launch(trim, time_s, hand_height_m)
            .map_err(error)
    }
    #[wasm_bindgen(constructor)]
    pub fn new(parameters: &[f64], mass: f64) -> Result<Kernel, JsValue> {
        Ok(Self {
            inner: kernel::Kernel::new(parameters, mass).map_err(error)?,
        })
    }
    pub fn derivative(
        &self,
        state: &[f64],
        command: &[f64],
        wind: &[f64],
    ) -> Result<Vec<f64>, JsValue> {
        self.inner.derivative(state, command, wind).map_err(error)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn replay(
        &self,
        state: &[f64],
        commands: &[f64],
        wind: &[f64],
        voltage: f64,
        rpm: f64,
        dt: f64,
        substeps: usize,
        causal: bool,
    ) -> Result<Vec<f64>, JsValue> {
        self.inner
            .replay(state, commands, wind, voltage, rpm, dt, substeps, causal)
            .map_err(error)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn surface_step(
        &self,
        state: &[f64],
        command: &[f64],
        wind: &[f64],
        voltage: f64,
        rpm: f64,
        dt: f64,
        env: &[f64],
        settings: &[f64],
    ) -> Result<Vec<f64>, JsValue> {
        self.inner
            .surface_step(state, command, wind, voltage, rpm, dt, env, settings)
            .map_err(error)
    }
}
#[wasm_bindgen]
pub struct Simulator {
    inner: sim_core::Simulator,
}
#[wasm_bindgen]
impl Simulator {
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<Simulator, JsValue> {
        Ok(Self {
            inner: sim_core::Simulator::new(sim_io::parse_config(config_json).map_err(error)?)
                .map_err(error)?,
        })
    }
    pub fn step(&mut self, command: &[f64], wind: &[f64], dt_s: f64) -> Result<Vec<f64>, JsValue> {
        self.inner
            .step(
                kernel::array(command).map_err(error)?,
                kernel::array(wind).map_err(error)?,
                dt_s,
            )
            .map_err(error)?;
        Ok(self.inner.state.as_slice().to_vec())
    }
    #[wasm_bindgen(getter)]
    pub fn state(&self) -> Vec<f64> {
        self.inner.state.as_slice().to_vec()
    }
    #[wasm_bindgen(getter)]
    pub fn rpm(&self) -> f64 {
        self.inner.rpm
    }
    #[wasm_bindgen(getter)]
    pub fn time_s(&self) -> f64 {
        self.inner.time_s
    }
    pub fn ground_start(&mut self) {
        self.inner.ground_start();
    }
    pub fn release(&mut self) {
        self.inner.release();
    }
}
#[wasm_bindgen]
pub fn config_schema() -> String {
    sim_io::config_schema()
}
