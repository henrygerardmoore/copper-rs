use std::path::PathBuf;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
mod messages;
mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct FlightSim {}

fn main() {
    let logger_path = "/tmp/mylogfile.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), None, true, None)
        .expect("Failed to setup logger.");

    debug!("Logger created at {}.", logger_path);

    let clock = copper_ctx.clock;

    debug!("Creating application... ");
    let mut application = FlightSim::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
}
