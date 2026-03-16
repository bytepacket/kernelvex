extern crate kernelvex as kv;
use kv::{Controller, Button, SolenoidGroup};
use vexide::prelude::*;

#[vexide::main(banner(enabled = true))]
async fn main(peripherals: Peripherals) {
    let pto = SolenoidGroup::new(vec![AdiDigitalOut::new(peripherals.adi_b)]);
    let mut controller = Controller::new(peripherals.primary_controller);

    controller.bind(Button::R1, move || {
        let clutch = pto.clone();
        async move {
            let _ = clutch.toggle().await;
        }
    }).await;

    controller.build().await;
}