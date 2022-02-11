#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::pac;
use core::convert::TryInto;

extern "C" {
    static mut __sixiptext: u8;
    static mut __sxiptext: u8;
    static mut __exiptext: u8;
}

/// Disable XIP caching, and initialize
/// memory region XIP_RAM with contents from flash
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

// safety: must only be called while XIP RAM is initialized with
// the RAM functions, ie. after calling initialize_xip_ram()
#[link_section = ".xiptext"]
unsafe fn force_cs_low(level: bool) {
    (*pac::IO_QSPI::ptr()).gpio_qspiss.gpio_ctrl.modify(|_, w| {
        if level {
            w.outover().low()
        } else {
            w.outover().normal()
        }
    });
}

// safety: must only be called while XIP RAM is initialized with
// the RAM functions, ie. after calling initialize_xip_ram()
#[inline(never)]
#[link_section = ".xiptext"]
unsafe fn do_flash_cmd(mut txbuf: *const u8, mut rxbuf: *mut u8, count: usize) {
    // need to do rom lookups manually, as the obvious way, like
    // `let connect_internal_flash = rom_data::connect_internal_flash;`
    // returns a pointer to flash
    let connect_internal_flash: extern "C" fn() -> () = rom::rom_table_lookup(*b"IF");
    let flash_exit_xip: extern "C" fn() -> () = rom::rom_table_lookup(*b"EX");
    let flash_enter_cmd_xip: extern "C" fn() -> () = rom::rom_table_lookup(*b"CX");

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    connect_internal_flash();
    flash_exit_xip();

    force_cs_low(true);

    let ssi = &*pac::XIP_SSI::ptr();

    let mut tx_rem = count;
    let mut rx_rem = count;

    while rx_rem > 0 || tx_rem > 0 {
        const MAX_IN_FLIGHT: usize = 16 - 2;
        let flags = ssi.sr.read();
        let can_put = flags.tfnf().bit_is_set() && tx_rem > 0 && rx_rem - tx_rem < MAX_IN_FLIGHT;
        let can_get = flags.rfne().bit_is_set() && rx_rem > 0;

        if can_put {
            ssi.dr0.write(|w| w.dr().bits(*txbuf as _));
            txbuf = txbuf.add(1);
            tx_rem -= 1;
        }

        if can_get {
            core::ptr::write(rxbuf, ssi.dr0.read().dr().bits() as _);
            rxbuf = rxbuf.add(1);
            rx_rem -= 1;
        }
    }

    force_cs_low(false);
    // Can't use this, as enabling the cache would overwrite the currently running method:
    // flash_flush_cache();
    // Instead, re-enable XIP, but without caching, so XIP RAM stays available:
    flash_enter_cmd_xip();
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

const FLASH_RUID_CMD: u8 = 0x4b;
const FLASH_RUID_DUMMY_BYTES: usize = 4;
const FLASH_RUID_DATA_BYTES: usize = 8;
const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

#[inline(never)] // only for readability of disassembly
fn read_uid() -> u64 {
    let mut txbuf = [0; FLASH_RUID_TOTAL_BYTES];
    let mut rxbuf = [0; FLASH_RUID_TOTAL_BYTES];

    txbuf[0] = FLASH_RUID_CMD;

    unsafe {
        // This can't be in XIP_RAM, as calling it will enable
        // XIP caching and therefore overwrites XIP_RAM.
        // Uses [u32] to force alignment
        let boot2_copyout: [u32; 64] = core::mem::transmute(bsp::BOOT2_FIRMWARE);
        // make sure boot2_copyout is really initalized - otherwise, initializing
        // this memory might get elided by optimizer.
        // NOTE: I'm not sure if this is formally sufficient. It seems to work
        // in practice, though.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        initialize_xip_ram();
        do_flash_cmd(txbuf.as_ptr(), rxbuf.as_mut_ptr(), FLASH_RUID_TOTAL_BYTES);
        let flash_enable_xip_via_boot2: extern "C" fn() =
            core::mem::transmute((boot2_copyout.as_ptr() as *const u8).add(1));
        flash_enable_xip_via_boot2();
    }

    u64::from_le_bytes(rxbuf[FLASH_RUID_DUMMY_BYTES + 1..].try_into().unwrap())
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let uid = read_uid();
    info!("uid: {}", uid);

    #[allow(clippy::empty_loop)]
    loop {}
}


mod rom {

    // Some non-pub stuff copied from HAL. 
    // TODO: add methods to return those pointers to HAL

    unsafe fn rom_hword_as_ptr(rom_address: *const u16) -> *const u32 {
        let ptr: u16 = *rom_address;
        ptr as *const u32
    }

    /// A bootrom function table code.
    pub type RomFnTableCode = [u8; 2];

    /// This function searches for (table)
    type RomTableLookupFn<T> = unsafe extern "C" fn(*const u16, u32) -> T;

    /// The following addresses are described at `2.8.2. Bootrom Contents`
    /// Pointer to the lookup table function supplied by the rom.
    const ROM_TABLE_LOOKUP_PTR: *const u16 = 0x0000_0018 as _;

    /// Pointer to helper functions lookup table.
    const FUNC_TABLE: *const u16 = 0x0000_0014 as _;

    /// Retrive rom content from a table using a code.
    pub fn rom_table_lookup<T>(tag: RomFnTableCode) -> T {
        let table = FUNC_TABLE;
        unsafe {
            let rom_table_lookup_ptr: *const u32 = rom_hword_as_ptr(ROM_TABLE_LOOKUP_PTR);
            let rom_table_lookup: RomTableLookupFn<T> = core::mem::transmute(rom_table_lookup_ptr);
            rom_table_lookup(
                rom_hword_as_ptr(table) as *const u16,
                u16::from_le_bytes(tag) as u32,
            )
        }
    }
}
