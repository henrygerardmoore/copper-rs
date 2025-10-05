use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use optimization_engine::constraints::Constraint;
use optimization_engine::panoc::PANOCCache;
use serde::Serialize;
use std::marker::PhantomData;

/// Output of the MPC controller.
#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct MPCControlOutputPayload {
    /// Final output
    pub output: Vec<f64>,
}

/// This is the underlying standard MPC controller.
pub struct MPCController<const N: usize> {
    setpoint: [f64; N],
    output_limits: [(f64, f64); N],
    sample_period: CuDuration,
    // f(x, u) -> xdot
    dynamics_function: Box<dyn Fn(&[f64], &[f64]) -> [f64] + Send>,

    // MPC controller must have at least one of the below cost functions
    // optional state cost function, J(x, u) -> f64
    state_cost: Option<Box<dyn Fn(&[f64], &[f64]) -> f64 + Send>>,

    // optional terminal cost function, J(x) -> f64
    terminal_cost: Option<Box<dyn Fn(&[f64]) -> f64 + Send>>,

    constraint: Box<dyn Constraint>,

    // Internal state
    tolerance: f64,
    cache: PANOCCache,
    last_error: [f64; N],
    elapsed: CuDuration,
    last_output: MPCControlOutputPayload,
}

impl<const N: usize> MPCController<N> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        setpoint: [f64; N],
        output_limits: [(f64, f64); N],
        sample_period: CuDuration,
        dynamics_function: impl Fn(&[f64], &[f64]) -> [f64] + Send + 'static,
        state_cost: Option<impl Fn(&[f64], &[f64]) -> f64 + Send + 'static>,
        terminal_cost: Option<impl Fn(&[f64]) -> f64 + Send + 'static>,
        constraint: impl Constraint + Send + 'static,
        tolerance: f64,
    ) -> Self {
        MPCController {
            setpoint,
            output_limits,
            sample_period,
            dynamics_function: Box::new(dynamics_function),
            state_cost: state_cost.map(|state_cost_function| {
                Box::new(state_cost_function) as Box<dyn Fn(&[f64], &[f64]) -> f64 + Send>
            }),
            terminal_cost: terminal_cost.map(|terminal_cost_function| {
                Box::new(terminal_cost_function) as Box<dyn Fn(&[f64]) -> f64 + Send>
            }),
            constraint: Box::new(constraint),
            cache: PANOCCache::new(N, tolerance, 20),
            elapsed: CuDuration::default(),
            last_output: MPCControlOutputPayload::default(),
            last_error: [0.0; N],
            tolerance: tolerance,
        }
    }

    pub fn reset(&mut self) {
        self.cache = PANOCCache::new(N, self.tolerance, 20);
        self.last_error = [0.0; N];
    }

    pub fn init_measurement(&mut self, measurement: &[f64; N]) {
        self.update_error(measurement);
        self.elapsed = self.sample_period; // force the computation on the first next_control_output
    }

    fn update_error(&mut self, measurement: &[f64; N]) {
        for i in 0..N {
            self.last_error[i] = self.setpoint[i] - measurement[i];
        }
    }

    pub fn next_control_output(
        &mut self,
        measurement: &[f64; N],
        dt: CuDuration,
    ) -> MPCControlOutputPayload {
        self.elapsed += dt;

        if self.elapsed < self.sample_period {
            // if we update the MPC controller too fast, return its previous output
            return self.last_output.clone();
        }

        self.update_error(measurement);
        let CuDuration(elapsed) = self.elapsed;
        let dt = elapsed as f32 / 1_000_000f32;

        // do MPC calculation

        let output = MPCControlOutputPayload { output: vec![] };

        self.last_output = output.clone();
        self.elapsed = CuDuration::default();
        output
    }
}

/// This is the Copper task encapsulating the MPC controller.
pub struct GenericMPCTask<I, const N: usize>
where
    [f64; N]: for<'a> From<&'a I>,
{
    _marker: PhantomData<I>,
    mpc: MPCController<N>,
    first_run: bool,
    last_tov: CuTime,
    setpoint: f32,
}

impl<I, const N: usize> CuTask for GenericMPCTask<I, N>
where
    [f64; N]: for<'a> From<&'a I>,
    I: CuMsgPayload,
{
    type Input<'m> = input_msg!(I);
    type Output<'m> = output_msg!(MPCControlOutputPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        match config {
            Some(_config) => Err(CuError::from("WIP")),
            None => Err(CuError::from("MPCTask needs a config.")),
        }
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        match input.payload() {
            Some(payload) => {
                // WIP
                output.clear_payload()
            }
            None => output.clear_payload(),
        };
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.mpc.reset();
        self.first_run = true;
        Ok(())
    }
}

/// Store/Restore the internal state of the MPC controller.
impl<I, const N: usize> Freezable for GenericMPCTask<I, N>
where
    [f64; N]: for<'a> From<&'a I>,
{
    fn freeze<E: Encoder>(&self, _encoder: &mut E) -> Result<(), EncodeError> {
        // WIP
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, _decoder: &mut D) -> Result<(), DecodeError> {
        // WIP
        Ok(())
    }
}
