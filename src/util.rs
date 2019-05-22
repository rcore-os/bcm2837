use core::convert::TryFrom;

pub fn to_usize(x: u32) -> usize {
    usize::try_from(x).unwrap()
}

pub fn to_u32(x: usize) -> u32 {
    u32::try_from(x).unwrap()
}

/*
pub unsafe fn read64(nAddress: * const usize) -> usize {
    return *nAddress;
}

pub unsafe fn write64(nAddress: * mut usize, nValue:usize) {
    *nAddress = nValue;
}
*/

pub unsafe fn read32(nAddress: usize) -> u32 {
    return *(nAddress as *const u32);
}

pub unsafe fn write32(nAddress: usize, nValue: u32) {
    *(nAddress as *mut u32) = nValue;
}
