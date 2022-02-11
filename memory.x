MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
    XIP   : ORIGIN = 0x15000000, LENGTH = 16K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2

} INSERT BEFORE .text;

SECTIONS {
    /* ### to be copied to XIP ram on demand */
    .xiptext ORIGIN(XIP) : {
        . = ALIGN(4);
    	__sxiptext = .;
      	KEEP(*(.xiptext .xiptext.*));		
        . = ALIGN(4);
	__exiptext = .;
    } > RAM AT > FLASH

    __sixiptext = LOADADDR(.xiptext);

} INSERT AFTER .rodata;
