/* memory map */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 3840K
  AXISRAM  (xrw): ORIGIN = 0x24000000, LENGTH = 512K
  ITCMRAM  (xrw): ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM  (xrw): ORIGIN = 0x20000000, LENGTH = 128K
  SRAM0    (xrw): ORIGIN = 0x30000000, LENGTH = 16K
  SRAM1    (xrw): ORIGIN = 0x30004000, LENGTH = 16K
}

ENTRY(Reset_Handler)

SECTIONS
{
  __stack_size = DEFINED(__stack_size) ? __stack_size : 2K;
  __heap_size = DEFINED(__heap_size) ? __heap_size : 1K;

  /* ISR vectors */
  .vectors :
  {
    . = ALIGN(4);
    KEEP(*(.vectors))
    . = ALIGN(4);
    __Vectors_End = .;
    __Vectors_Size = __Vectors_End - __gVectors;
  } >FLASH

  /* text section, such as program code */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    
    . = ALIGN(4);
    /* the symbol '_etext' will be defined at the end of code section */
    _etext = .;
  } >FLASH

  /* ro-data section, such as constant data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
    
        /* section information for finsh shell */
    . = ALIGN(4);
    __fsymtab_start = .;
    KEEP(*(FSymTab))
    __fsymtab_end = .;
    . = ALIGN(4);
    __vsymtab_start = .;
    KEEP(*(VSymTab))
    __vsymtab_end = .;
    . = ALIGN(4);

    /* section information for initial. */
    . = ALIGN(4);
    __rt_init_start = .;
    KEEP(*(SORT(.rti_fn*)))
    __rt_init_end = .;
    . = ALIGN(4);
  } >FLASH

  /* exception process table */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  /* exception process table index */
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* ARM attributes */
  .ARM.attributes :
  {
   *(.ARM.attributes)
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* provide some necessary symbols for initialized data */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    /* the symbol '_sdata' will be defined at the data section end start */
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    /* the symbol '_edata' will be defined at the data section end */
    _edata = .;
  } > AXISRAM AT> FLASH

  /* heap and stack space, must be initialized before the .bss */
  .heap_stack :
  {
    . = ALIGN(8);
    . = . + __stack_size;
    PROVIDE( _sp = . ); 
    . = ALIGN(8);
  } > AXISRAM

  /* provide some necessary symbols for uninitialized data */
  . = ALIGN(4);
  .bss :
  {
    . = ALIGN(4);
    /* the symbol '_sbss' will be defined at the bss section start */
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    /* the symbol '_ebss' will be defined at the bss section end */
    _ebss = .;
    __bss_end = _ebss;
  } > AXISRAM

  _siitcmram = LOADADDR(.itcmram);
  .itcmram :
  {
    . = ALIGN(4);
    .sitcmram = .;
    *(.itcmram)
    *(.itcmram*)
    . = ALIGN(4);
    _eitcmram = .;
  } > ITCMRAM AT> FLASH

  _sidtcmram = LOADADDR(.dtcmram);
  .dtcmram :
  {
    . = ALIGN(4);
    .sdtcmram = .;
    *(.dtcmram)
    *(.dtcmram*)
    . = ALIGN(4);
    _edtcmram = .;
  } > DTCMRAM AT> FLASH

  _sisram0 = LOADADDR(.sram0);
  .sram0 :
  {
    . = ALIGN(4);
    .ssram0 = .;
    *(.sram0)
    *(.sram0*)
    . = ALIGN(4);
    _esram0 = .;
  } > SRAM0 AT> FLASH

  _sisram1 = LOADADDR(.sram1);
  .sram1 :
  {
    . = ALIGN(4);
    .ssram1 = .;
    *(.sram1)
    *(.sram1*)
    . = ALIGN(4);
    _esram1 = .;
  } > SRAM1 AT> FLASH
}

/* input sections */
GROUP(libgcc.a libc.a libm.a libnosys.a)
