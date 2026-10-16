/*
 * ports/chip/imxrt/imxrt_ocram.ld — places .ocram.bss in the OCRAM.
 *
 * The SDK's flash linker scripts define the OCRAM (m_data2) and leave it
 * empty; everything else lands in the DTCM. This fragment, INCLUDEd after
 * the SDK script by the script port.cmake generates, adds one NOLOAD output
 * section there for the
 * objects a board marks ROSETTE_IMXRT_OCRAM_BSS (imxrt_memory.h). The OCRAM
 * is cached (rosette_imxrt_start_chip() enables the D-cache), so it suits
 * data only the CPU touches, such as the RTOS heap; DMA buffers stay in the
 * non-cached DTCM. The startup code does not clear this section.
 *
 * References: GNU ld manual, "Output Section Description" and INCLUDE;
 * i.MX RT1020 Reference Manual, system memory map (OCRAM at 0x20200000).
 */
SECTIONS
{
  .ocram_bss (NOLOAD) :
  {
    . = ALIGN(8);
    __ocram_bss_start__ = .;
    *(.ocram.bss .ocram.bss.*)
    . = ALIGN(8);
    __ocram_bss_end__ = .;
  } > m_data2
}
