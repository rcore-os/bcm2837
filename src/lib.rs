#![no_std]
#![feature(alloc)]
#![feature(asm)]

extern crate alloc;
extern crate volatile;
#[macro_use]
extern crate log;
extern crate aarch64;

pub mod atags;
pub mod consts;
pub mod dma;
pub mod emmc;
pub mod gpio;
pub mod interrupt;
pub mod mailbox;
pub mod mini_uart;
pub mod pwm;
pub mod timer;
