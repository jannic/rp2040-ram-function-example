//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::pac;

#[link_section = ".xiptext"]
// safety: must only be called while XIP RAM is initialized with
// the RAM functions
#[inline(never)]
unsafe fn run_from_xip_ram() {
    // Of course, it doesn't make much sense to call info!() here,
    // as it causes method calls back into flash.
    //
    // Everything which should run purely from RAM must be carefully
    // crafted to be self-contained. This doesn't only affect explicit method
    // calls, but also stuff like integer division, which also translated
    // to a method call by the compiler.
    info!("running from RAM");
}

extern "C" {
    static mut __sixiptext: u8;
    static mut __sxiptext: u8;
    static mut __exiptext: u8;
}

fn initialize_xip_ram() {
    // disable XIP caching
    unsafe { (*pac::XIP_CTRL::ptr()).ctrl.modify(|_, w| w.en().clear_bit()); }
    // copy xiptext from flash to XIP RAM
    unsafe {
        bsp::hal::rom_data::memcpy(&mut __sxiptext as *mut u8, &mut __sixiptext as *mut u8, (&__exiptext as *const u8).offset_from(&__sxiptext as *const u8) as u32);
    }
}

// safety: must not be called if some concurrent code may be running
// from XIP RAM
unsafe fn reenable_xip_cache() {
    // this is all covered by a rom function
    bsp::hal::rom_data::flash_flush_cache();

    // The same function could also be implemented like this:
    /*
    // flush XIP cache contents
    (*pac::XIP_CTRL::ptr()).flush.modify(|_, w| w.flush().set_bit());
    // wait for flush to finish
    (*pac::XIP_CTRL::ptr()).flush.read();

    // re-enable xip caching
    (*pac::XIP_CTRL::ptr()).ctrl.modify(|_, w| w.en().set_bit());
    */
}

#[entry]
fn main() -> ! {
    info!("Program start");
    unsafe { 
        initialize_xip_ram();
        run_from_xip_ram();
        reenable_xip_cache();
    }
    info!("back to main");

    #[allow(clippy::empty_loop)]
    loop {}
}

