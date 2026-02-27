use std::pin::Pin;
use std::sync::{Arc};
use vexide_async::sync::Mutex;
use vexide_async::task::Task;
use vexide_async::time::sleep;
use vexide_devices::controller::{ButtonState, Controller as VEXController, ControllerState};


type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

type AsyncCallback = Box<dyn Fn() -> BoxFuture<'static, ()> + Send + Sync + 'static>;

pub struct Controller {
    controller: Arc<Mutex<VEXController>>,
    bindings: Arc<Mutex<Vec<(Button, AsyncCallback)>>>,
    _task: Option<Task<()>>,
}

#[derive(Copy, Clone)]
pub enum Button {
    A,
    B,
    X,
    Y,
    Up,
    Down,
    Left,
    Right,
    L1,
    L2,
    R1,
    R2,
}

impl std::ops::Index<Button> for ControllerState {
    type Output = ButtonState;

    fn index(&self, index: Button) -> &Self::Output {
        match index {
            Button::A => &self.button_a,
            Button::B => &self.button_b,
            Button::X => &self.button_x,
            Button::Y => &self.button_y,
            Button::Up => &self.button_up,
            Button::Down => &self.button_down,
            Button::Left => &self.button_left,
            Button::Right => &self.button_right,
            Button::L1 => &self.button_l1,
            Button::L2 => &self.button_l2,
            Button::R1 => &self.button_r1,
            Button::R2 => &self.button_r2,
        }
    }
}

impl Controller {
    #[must_use]
    pub fn new(controller: VEXController) -> Self {
        let bindings = Arc::new(Mutex::new(Vec::new()));

        Self {
            controller: Arc::new(Mutex::new(controller)),
            bindings,
            _task: None, // TODO: add task
        }
    }

    #[must_use]
    pub fn from_shared(controller: Arc<Mutex<VEXController>>) -> Self {
        let bindings = Arc::new(Mutex::new(Vec::new()));
        Self {
            controller,
            bindings,
            _task: None,
        }
    }

    pub async fn bind<F, Fut>(&mut self, button: Button, callback: F)
    where
        F: Fn() -> Fut + Send + Sync + 'static,
        Fut: Future<Output = ()> + Send + 'static,
    {
        let mut bindings = self.bindings.lock().await;

        let wrapped_callback: AsyncCallback = Box::new(move || {
            Box::pin(callback())
        });

        bindings.push((button, wrapped_callback));
    }

    pub async fn build(&mut self) {

        self._task = Some(vexide_async::task::spawn(Self::task(

            self.controller.clone(),

            self.bindings.clone(),

        )));

    }

    async fn task(controller: Arc<Mutex<VEXController>>, bindings: Arc<Mutex<Vec<(Button, AsyncCallback)>>>) {
        let mut last_state = ControllerState::default();
        loop {

            let state = controller.lock().await.state().unwrap_or_default();

            for (k, v) in bindings.lock().await.iter() {
                if state[*k].is_now_pressed() && !last_state[*k].is_now_pressed() {
                    vexide_async::task::spawn(v()).detach();
                }
            }

            last_state = state;

            sleep(VEXController::UPDATE_INTERVAL).await;
        }
    }
}

/*
async unsafe fn test() {
    let solenoid = SolenoidGroup::new(vec![AdiDigitalOut::new(unsafe { AdiPort::new(1, None) })]);

    let control = Arc::new(Mutex::new(unsafe { VEXController::new(ControllerId::Primary) }));

    let mut controller = Controller::from_shared(control.clone());

    controller.bind(Button::A, || async move {
        println!("Doing a side effect with no errors!");
    }).await;


    controller.bind(Button::A, move || {
        let sol = solenoid.clone();
        async move {
            println!("Firing solenoid!");
            let _ = sol.extend().await;
        }
    }).await;

    controller.build().await;

    let mut guard = control.lock().await;

    guard.clear_screen().await.expect("cant clear");
}
*/


