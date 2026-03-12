//! LED indicator for BLE advertising state.

use embassy_nrf::gpio::Output;
use embassy_time::Timer;
use rmk::ble::BleState;
use rmk::channel::{CONTROLLER_CHANNEL, ControllerSub};
use rmk::controller::Controller;
use rmk::embassy_futures::select::{Either, select};
use rmk::event::ControllerEvent;

/// LED active level: false = active-high, true = active-low.
const LED_LOW_ACTIVE: bool = false;

/// Blink interval in milliseconds.
const BLINK_INTERVAL_MS: u64 = 200;
/// Number of ticks in one animation cycle.
const CYCLE_TICKS: u8 = 10;
/// Number of supported BLE profiles.
const MAX_PROFILES: u8 = 3;

// Compile-time checks.
const _: () = assert!(2 * (MAX_PROFILES as u16) <= CYCLE_TICKS as u16, "MAX_PROFILES too large for animation cycle");
const _: () = assert!(CYCLE_TICKS as u32 <= u16::BITS, "CYCLE_TICKS too large for u16 mask width");

/// Build a toggle bitmask for a given profile.
/// Profile N blinks N+1 times = 2*(N+1) consecutive toggle bits.
const fn toggle_mask(profile: u8) -> u16 {
    (1 << (2 * (profile as u16 + 1))) - 1
}

/// Controller that drives an LED according to the BLE advertising state.
pub struct BleStateLedController<'d> {
    pin: Output<'d>,
    sub: ControllerSub,
    state: BleState,
    profile: u8,
    tick: u8,
    /// Pending state update received while advertising; applied at next cycle start.
    pending_state: Option<(u8, BleState)>,
}

/// Events processed by this controller.
pub enum BleLedEvent {
    Tick,
    Controller(ControllerEvent),
}

impl<'d> BleStateLedController<'d> {
    pub fn new(pin: Output<'d>) -> Self {
        let mut this = Self {
            pin,
            sub: CONTROLLER_CHANNEL.subscriber().unwrap(),
            state: BleState::None,
            profile: 0,
            tick: 0,
            pending_state: None,
        };
        this.set_off();
        this
    }

    fn set_off(&mut self) {
        if LED_LOW_ACTIVE {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
    }

    fn led_blink(&mut self) {
        if toggle_mask(self.profile) & (1u16 << self.tick) != 0 {
            self.pin.toggle();
        }
    }
}

impl<'d> Controller for BleStateLedController<'d> {
    type Event = BleLedEvent;

    async fn process_event(&mut self, event: Self::Event) {
        match event {
            BleLedEvent::Controller(ControllerEvent::BleState(profile, state)) => {
                if matches!(self.state, BleState::Advertising) {
                    // Don't interrupt current cycle; store for later.
                    self.pending_state = Some((profile, state));
                } else {
                    // LED off → apply immediately.
                    self.profile = profile;
                    self.state = state;
                    self.tick = 0;
                    self.set_off();
                }
            }
            BleLedEvent::Tick => {
                // At cycle start, apply any pending state.
                if self.tick == 0 {
                    if let Some((p, s)) = self.pending_state.take() {
                        self.profile = p;
                        self.state = s;
                        // LED is already off at cycle end.
                    }
                }

                if matches!(self.state, BleState::Advertising) && self.profile < MAX_PROFILES {
                    self.led_blink();
                    self.tick = (self.tick + 1) % CYCLE_TICKS;
                }
            }
            BleLedEvent::Controller(_) => {}
        }
    }

    async fn next_message(&mut self) -> Self::Event {
        if matches!(self.state, BleState::Advertising) {
            match select(Timer::after_millis(BLINK_INTERVAL_MS), self.sub.next_message_pure()).await {
                Either::First(_) => BleLedEvent::Tick,
                Either::Second(event) => BleLedEvent::Controller(event),
            }
        } else {
            BleLedEvent::Controller(self.sub.next_message_pure().await)
        }
    }
}
