pub use cu_sensor_payloads::ImuPayload;
use cu29::{
    cutask::{CuSrcTask, Freezable},
    output_msg,
};

pub struct BevyIMU {}

impl Freezable for BevyIMU {}

impl CuSrcTask for BevyIMU {
    type Output<'m> = output_msg!(ImuPayload);

    type Resources<'r> = ();

    fn new(
        _config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        todo!()
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        todo!()
    }
}
