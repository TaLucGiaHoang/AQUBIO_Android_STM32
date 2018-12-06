MEMORY
{
    ROM1 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    ROM2 (rx) : ORIGIN = 0x08040000, LENGTH = 768K
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
    .bootloader :
    {
        KEEP(*(.vector))
        . = ALIGN(16);
        *target_inithook.o(.text*)
        *core_support.o(.text*)
        *chip_serial.o(.text*)
        *core_timer.o(.text*)
        *chip_config.o(.text*)
        *core_config.o(.text*)
        *log_output.o(.text*)

        *task.o(.text*)
        *wait.o(.text*)
        *time_event.o(.text*)
        *task_manage.o(.text*)
        *task_refer.o(.text*)
        *task_sync.o(.text*)
        *task_except.o(.text*)
        *semaphore.o(.text*)
        *eventflag.o(.text*)
        *dataqueue.o(.text*)
        *pridataq.o(.text*)
        *mailbox.o(.text*)
        *mempfix.o(.text*)
        *time_manage.o(.text*)
        *cyclic.o(.text*)
        *alarm.o(.text*)
        *sys_manage.o(.text*)
        *interrupt.o(.text*)
        *exception.o(.text*)
        
        *log_output.o(.text*)
        *vasyslog.o(.text*)
        *t_perror.o(.text*)
        *strerror.o(.text*)
        
        *banner.o(.text*)
        *syslog.o(.text*)
        *serial.o(.text*)
        *logtask.o(.text*)
        
        *lib_a-memset.o(.text*)
        *lib_a-memcpy-stub.o(.text*)
        
        *bootloader.o(.text*)
        *drviflx.o(.text*)
        *drviflx_hal.o(.text*)
        *drviflx_hal_ex.o(.text*)
        *drvqflx.o(.text*)
        *drvcmn_gpio.o(.text*)
    } > ROM1
    
    .bootloader_rodata : { 
        *target_inithook.o(.*)
        *core_support.o(.rodata*)
        *chip_serial.o(.rodata*)
        *core_timer.o(.rodata*)
        *chip_config.o(.rodata*)
        *core_config.o(.rodata*)
        *log_output.o(.rodata*)
        
        *task.o(.rodata*)
        *wait.o(.rodata*)
        *time_event.o(.rodata*)
        *task_manage.o(.rodata*)
        *task_refer.o(.rodata*)
        *task_sync.o(.rodata*)
        *task_except.o(.rodata*)
        *semaphore.o(.rodata*)
        *eventflag.o(.rodata*)
        *dataqueue.o(.rodata*)
        *pridataq.o(.rodata*)
        *mailbox.o(.rodata*)
        *mempfix.o(.rodata*)
        *time_manage.o(.rodata*)
        *cyclic.o(.rodata*)
        *alarm.o(.rodata*)
        *sys_manage.o(.rodata*)
        *interrupt.o(.rodata*)
        *exception.o(.rodata*)
        
        *log_output.o(.rodata*)
        *vasyslog.o(.rodata*)
        *t_perror.o(.rodata*)
        *strerror.o(.rodata*)
        
        *banner.o(.rodata*)
        *syslog.o(.rodata*)
        *serial.o(.rodata*)
        *logtask.o(.rodata*)
        
        *lib_a-memset.o(.rodata*)
        *lib_a-memcpy-stub.o(.rodata*)
        
        *bootloader.o(.rodata*)
        *drviflx.o(.rodata*)
        *drviflx_hal.o(.rodata*)
        *drviflx_hal_ex.o(.rodata*)
        *drvqflx.o(.rodata*)
        *drvcmn_gpio.o(.rodata*)
    } > ROM1
    
    .text :
    {
        __text = . ;
        *kernel_cfg.o(*.text*)
        *kernel_cfg.o(*.rodata*)
        *start.o(.text*)
        *start.o(.rodata*)
        *startup.o(.text*)
        *(.text*)
        *(.glue_7t)
        *(.glue_7)
    } > ROM2

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
    __idata_end = __idata_start + SIZEOF(.data);
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
