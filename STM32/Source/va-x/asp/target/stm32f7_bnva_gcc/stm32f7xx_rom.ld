MEMORY
{
    ROM1 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    ROM2 (rx) : ORIGIN = 0x08040000, LENGTH = 512K
    RAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    RAM2 (rwx) : ORIGIN = 0x20020000, LENGTH = 368K
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm","elf32-littlearm") 
OUTPUT_ARCH(arm)

PROVIDE(hardware_init_hook = 0);
PROVIDE(software_init_hook = 0);
PROVIDE(software_term_hook = 0);
STARTUP(start.o)

SECTIONS
{
/*
    .bootloader :
    {
        KEEP(*(.vector))
        . = ALIGN(16);
        *start.o(.text*)
        __bootload = . ;
        *bootloader.o(.text*)
        *drviflx.o(.text*)
        *drviflx_hal.o(.text*)
        *drviflx_hal_ex.o(.text*)
    } > ROM1
    
    .bootloaderrodata : { 
        *start.o(.rodata*) 
        *bootloader.o(.rodata*)
        *drviflx.o(.rodata*)
        *drviflx_hal.o(.rodata*)
        *drviflx_hal_ex.o(.rodata*)
    } > ROM1
    
    .text :
    {
        __text = . ;
        *startup.o(.text*)
        *(.text*)
        *(.glue_7t)
        *(.glue_7)
    } > ROM2
*/

/*
    .ramfunc : 
    {
        *bootloader.o(.text*)
        *bootloader.o(.rodata*)
    } > RAM1
*/

    .text :
    {
        KEEP(*(.vector))
        . = ALIGN(16);
        __text = . ;
        *(.text*)
        *(.glue_7t)
        *(.glue_7)
    } > ROM1

    _etext = .	;
    PROVIDE (etext = .)	;

    .rodata : { 
        *(.rodata*) 
    } > ROM2

    . = ALIGN(4);
    __idata_start = . ;
    .data   :  AT(__idata_start) 
    {
        __data_start = . ;
        *(vtable)
        *(.data*)
    } > RAM1
    
    __ramfunc_start = __idata_start + SIZEOF(.data);
    .ramfunc : AT(__ramfunc_start)
    {
    } > RAM1
    __idata_end = __ramfunc_start + SIZEOF(.ramfunc);
    _edata  =  . ;
    PROVIDE (edata = .);
    . = ALIGN(4);

    .bss       :
    {
       __bss_start = .	;
      *(.bss*)
      *(COMMON)
      . = ALIGN(32 / 8);
    } > RAM1
   . = ALIGN(32 / 8);
     _end = .		;
    __bss_end = .;
    PROVIDE (end = .)	;

    .largebuf (NOLOAD)      :
    {
       __largebuf_start = .	;
      *(.largebuf)
      . = ALIGN(32 / 8);
    } > RAM2
   . = ALIGN(32 / 8);
    __largebuf_end = .;
    PROVIDE (largebuf_end = .)	;

    .comment 0 : { *(.comment) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to 
     the beginning of the section so we begin them at 0.  */

  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }

  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }

  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }

  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }

  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
