#![no_std]
#![feature(alloc)]
#![feature(asm)]

extern crate alloc;
extern crate volatile;

pub mod atags;
pub mod consts;
pub mod gpio;
pub mod interrupt;
pub mod mailbox;
pub mod mini_uart;
pub mod timer;
pub mod pwm_sound_device;
pub mod util;