//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception, ExceptionFrame};
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::pac;
use bsp::hal::rom_data;

extern "C" {
    static mut __sixiptext: u8;
    static mut __sxiptext: u8;
    static mut __exiptext: u8;
}

fn initialize_xip_ram() {
    // disable XIP caching
    unsafe {
        (*pac::XIP_CTRL::ptr())
            .ctrl
            .modify(|_, w| w.en().clear_bit());
    }
    // copy xiptext from flash to XIP RAM
    unsafe {
        bsp::hal::rom_data::memcpy(
            &mut __sxiptext as *mut u8,
            &mut __sixiptext as *mut u8,
            (&__exiptext as *const u8).offset_from(&__sxiptext as *const u8) as u32,
        );
    }
}

static mut BOOT2_COPYOUT: [u32; 64] = [0; 64];

#[link_section = ".xiptext"]
// safety: must only be called while XIP RAM is initialized with
// the RAM functions
#[inline(never)]
unsafe fn boot_2_copyout() {
    let xip_base = 0x10000000 as *const u32;
    let copyout = BOOT2_COPYOUT.as_mut_ptr();

    use core::ptr;

    for i in 0..64 {
        ptr::write_volatile(copyout.add(i), ptr::read_volatile(xip_base.add(i)));
    }
}

#[link_section = ".xiptext"]
// safety: must only be called while XIP RAM is initialized with
// the RAM functions
#[inline(never)]
unsafe fn set_cs(level: bool) {
    (&*pac::IO_QSPI::ptr())
        .gpio_qspiss
        .gpio_ctrl
        .modify(|_, w| {
            if level {
                w.outover().high()
            } else {
                w.outover().low()
            }
        });
}

#[link_section = ".xiptext"]
// safety: must only be called while XIP RAM is initialized with
// the RAM functions
#[inline(never)]
unsafe fn do_flash_cmd(mut txbuf: *const u8, mut rxbuf: *mut u8, count: usize) {
    let connect_internal_flash = rom_data::connect_internal_flash;
    let flash_exit_xip = rom_data::flash_exit_xip;
    let flash_flush_cache = rom_data::flash_flush_cache;

    boot_2_copyout();

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    connect_internal_flash();
    flash_exit_xip();

    // set_cs(false);

    // let ssi = &*pac::XIP_SSI::ptr();

    // for i in 0..count {
    //     while !ssi.sr.read().tfnf().bit_is_set() {}
    //     ssi.dr0.write(|w| w.dr().bits(*txbuf.add(i) as _));

    //     while !ssi.sr.read().rfne().bit_is_set() {}
    //     ssi.dr0.write(|w| w.dr().bits(*txbuf.add(i) as _));
    // }

    // let mut tx_rem = count;
    // let mut rx_rem = count;

    // while rx_rem > 0 || tx_rem > 0 {
    //     let flags = ssi.sr.read();
    //     let can_put = flags.tfnf().bit_is_set() && tx_rem > 0 && rx_rem - tx_rem < MAX_IN_FLIGHT;
    //     let can_get = flags.rfne().bit_is_set() && rx_rem > 0;

    //     const MAX_IN_FLIGHT: usize = 16 - 2;

    //     if can_put {
    //         ssi.dr0.write(|w| w.dr().bits(*txbuf as _));
    //         txbuf = txbuf.add(1);
    //         tx_rem -= 1;
    //     }

    //     if can_get {
    //         core::ptr::write(rxbuf, ssi.dr0.read().dr().bits() as _);
    //         rxbuf = rxbuf.add(1);
    //         rx_rem -= 1;
    //     }
    // }

    // set_cs(true);

    flash_flush_cache();
    flash_enable_xip_via_boot2();
}

#[link_section = ".xiptext"]
// safety: must only be called while XIP RAM is initialized with
// the RAM functions
#[inline(never)]
unsafe fn flash_enable_xip_via_boot2() {
    let start: extern "C" fn() =
        core::mem::transmute((BOOT2_COPYOUT.as_mut_ptr() as *const u8).add(1));
    start();
}

const FLASH_RUID_CMD: u8 = 0x4b;
const FLASH_RUID_DUMMY_BYTES: usize = 4;
const FLASH_RUID_DATA_BYTES: usize = 8;
const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

fn read_uid() -> u64 {
    let mut txbuf = [0; FLASH_RUID_TOTAL_BYTES];
    let mut rxbuf = [0; FLASH_RUID_TOTAL_BYTES];

    txbuf[0] = FLASH_RUID_CMD;

    unsafe {
        initialize_xip_ram();
        do_flash_cmd(txbuf.as_ptr(), rxbuf.as_mut_ptr(), FLASH_RUID_TOTAL_BYTES);
    }

    info!("rx: {}", rxbuf);

    // u64::from_le_bytes(rxbuf[FLASH_RUID_DUMMY_BYTES..].try_into().unwrap())
    0
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let uid = read_uid();
    info!("uid: {}", uid);

    #[allow(clippy::empty_loop)]
    loop {}
}

#[exception]
unsafe fn HardFault(eh: &ExceptionFrame) -> ! {
    panic!("Hardfault: {:?}", eh);
}
