mod generic_timer;
mod local_timer;
mod system_timer;

pub use generic_timer::GenericTimer;
pub use local_timer::LocalTimer;
pub use system_timer::SystemTimer;

/// The main timer
pub type Timer = GenericTimer;

/// The Raspberry Pi timer.
pub trait BasicTimer {
    /// The timer frequency (Hz)
    fn freq() -> u64;

    /// Returns a new instance.
    fn new() -> Self;

    /// Initialization timer.
    fn init(&mut self);

    /// Stop timer.
    fn stop(&mut self);

    /// Reads the timer's counter and returns the 64-bit counter value.
    /// The returned value is the number of elapsed microseconds.
    fn read(&self) -> u64;

    /// Sets up a match in timer 1 to occur `us` microseconds from now. If
    /// interrupts for timer 1 are enabled and IRQs are unmasked, then a timer
    /// interrupt will be issued in `us` microseconds.
    fn tick_in(&mut self, us: usize);

    /// Returns `true` if timer interruption is pending. Otherwise, returns `false`.
    fn is_pending(&self) -> bool;
}

/// wait for `cycle` CPU cycles
#[inline(always)]
pub fn delay(cycle: usize) {
    for _ in 0..cycle {
        unsafe { asm!("nop") }
    }
}

/// wait for us microsecond
pub fn delay_us(us: usize) {
    let mut timer = Timer::new();
    timer.init();
    let start_time = timer.read();
    while timer.read() - start_time < us as u64 {
        unsafe { asm!("nop") }
    }
}
