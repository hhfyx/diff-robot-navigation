/**
 ******************************************************************************
 * @file    stm32f103c8.ld
 * @brief   链接脚本：STM32F103C8T6 (64 KB Flash / 20 KB RAM)
 *
 * 内存布局：
 *   Flash  0x08000000 — 0x0800FFFF  (64 KB)
 *   RAM    0x20000000 — 0x20004FFF  (20 KB，其中栈从顶部往下长)
 ******************************************************************************
 */

ENTRY(Reset_Handler)

/* 栈顶设在 RAM 末尾。留 1 KB 栈，足够本工程中断嵌套使用；
 * 若加浮点运算或递归，注意加大。 */
_estack = 0x20005000;

_Min_Heap_Size  = 0x200;    /* 512 B 堆，本工程不用 malloc，留作余量 */
_Min_Stack_Size = 0x400;    /* 1 KB 栈 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
  /* 向量表必须放在 Flash 最开头，上电时硬件从这里取栈顶和复位地址 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

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
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* .data 初值存在 Flash，运行时由启动代码拷到 RAM */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 检查堆栈是否够用：不够时链接直接报错，而不是运行时神秘跑飞 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* 调试符号段 */
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
