use crate::consts::*;
use core::convert::TryFrom;

pub fn to_usize(x: u32) -> usize {
    usize::try_from(x).unwrap()
}

pub fn to_u32(x: usize) -> u32 {
    u32::try_from(x).unwrap()
}


pub unsafe fn read32(nAddress: * const u32) -> u32 {
    return *nAddress;
}

pub unsafe fn write32 (nAddress: * mut u32, nValue:u32) {
    *nAddress = nValue;
}
