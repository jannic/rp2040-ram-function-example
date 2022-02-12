#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::{pac, rom_data};
use core::arch::asm;
use core::convert::TryInto;

extern "C" {
    static mut __sixiptext: u8;
    static mut __sxiptext: u8;
    static mut __exiptext: u8;
}

// Execute a flash command.
//
// # Safety
// This symbol is located in section xiptext, which must be manually
// initialized before calling this function. See read_uid for
// an example.
#[link_section = ".xiptext"]
#[inline(never)]
unsafe fn do_flash_cmd(txbuf: *const u8, rxbuf: *mut u8, count: usize) {
    asm!(
        include_str!("./asm.s"),
        inout("r0") txbuf => _,
        inout("r1") rxbuf => _,
        in("r2") count,
        options(raw),
    );
}

/// Disable XIP caching, and initialize memory region XIP_RAM with contents from flash
///
/// This is safe to call: XIP access to flash is still possible.
///
/// However, code will run slower, as flash access is no longer cached.
fn initialize_xip_ram() {
    // disable XIP caching
    unsafe {
        (*pac::XIP_CTRL::ptr())
            .ctrl
            .modify(|_, w| w.en().clear_bit());
    }
    // copy xiptext from flash to XIP RAM
    unsafe {
        rom_data::memcpy(
            &mut __sxiptext as *mut u8,
            &mut __sixiptext as *mut u8,
            (&__exiptext as *const u8).offset_from(&__sxiptext as *const u8) as u32,
        );
    }
}

/// Enable XIP caching.
///
/// Note that this function is located in flash, so it can't be called while
/// XIP is completely disabled!
///
/// # Safety
///
/// Temporarily disables XIP access. Therefore it must be called from
/// within a critical section, and the second core must not be executing
/// code from flash concurrently!
#[inline(never)] // only for readability of disassembly
unsafe fn reenable_xip_cache() {
    // This must be copied to ram, as XIP access will be temporarily be
    // disabled while this is running.
    //
    // Uses [u32] to force alignment
    let boot2_copyout: [u32; 64] = core::mem::transmute(bsp::BOOT2_FIRMWARE);

    // NOTE: I'm not sure if this is formally sufficient. It seems to work
    // in practice, though.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    let flash_enable_xip_via_boot2: extern "C" fn() =
        core::mem::transmute((boot2_copyout.as_ptr() as *const u8).add(1));
    flash_enable_xip_via_boot2();
}

#[inline(never)] // only for readability of disassembly
fn read_uid() -> u64 {
    const FLASH_RUID_CMD: u8 = 0x4b;
    const FLASH_RUID_DUMMY_BYTES: usize = 4;
    const FLASH_RUID_DATA_BYTES: usize = 8;
    const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

    let mut txbuf = [0; FLASH_RUID_TOTAL_BYTES];
    let mut rxbuf = [0; FLASH_RUID_TOTAL_BYTES];

    txbuf[0] = FLASH_RUID_CMD;

    critical_section::with(|_| {
        unsafe {
            initialize_xip_ram();
            do_flash_cmd(txbuf.as_ptr(), rxbuf.as_mut_ptr(), FLASH_RUID_TOTAL_BYTES);
            reenable_xip_cache();
        }
    });

    u64::from_le_bytes(rxbuf[FLASH_RUID_DUMMY_BYTES + 1..].try_into().unwrap())
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let uid = read_uid();
    info!("uid: 0x{=u64:x}", uid);

    #[allow(clippy::empty_loop)]
    loop {}
}
