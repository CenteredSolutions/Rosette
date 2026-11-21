// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_flexspi_nor_config.c
 * @brief The FlexSPI configuration block the i.MX RT boot ROM reads to execute from QSPI NOR flash.
 *
 * The boot ROM reads this 512-octet block at the start of flash
 * (0x60000000, section .boot_hdr.conf), configures FlexSPI from it, and
 * then finds the image vector table at 0x60001000 (the SDK's
 * fsl_flexspi_nor_boot.c). The block describes a generic quad-SPI NOR flash:
 * fast-read quad I/O (0xEB, 24-bit address, 6 dummy cycles), read status
 * (0x05, busy in bit 0), write enable (0x06), 4 KiB sector erase (0x20),
 * 64 KiB block erase (0xD8), page program (0x02), and chip erase (0x60), at
 * the sequence indices the ROM's NOR driver expects (NOR_CMD_LUT_SEQ_IDX_*).
 * It issues no quad-enable command: the flash must already have its QE bit
 * set, as every Teensy 4.x's flash does. Data is sampled on the DQS pad's
 * loopback, which the RT1062 pads support at the configured clock.
 *
 * The board supplies the flash size (ROSETTE_IMXRT_FLASH_SIZE_BYTES) and may
 * lower the read clock (ROSETTE_IMXRT_FLASH_SERIAL_CLK, a
 * kFLEXSPISerialClk_* value; 133 MHz unless set).
 *
 * Verification:
 * - Correct means: the ROM boots the image (the console prints the boot
 *   banner after a power cycle); the block matches PJRC's Teensy 4.x
 *   bootdata.c in every field the ROM reads, except the read clock.
 * - Covered by: every image on the micromod-teensy-w5500 board.
 *
 * References:
 * - i.MX RT1060 Reference Manual §9.6.3 (FlexSPI configuration block, serial NOR)
 * - MCUXpresso SDK fsl_romapi.h (flexspi_nor_config_t, LUT macros)
 * - PJRC Teensyduino cores/teensy4/bootdata.c (FlexSPI_NOR_Config)
 */

#include "fsl_romapi.h"

#ifndef ROSETTE_IMXRT_FLASH_SIZE_BYTES
#  error "The board must set ROSETTE_IMXRT_FLASH_SIZE_BYTES (board.cmake)"
#endif

#ifndef ROSETTE_IMXRT_FLASH_SERIAL_CLK
#  define ROSETTE_IMXRT_FLASH_SERIAL_CLK kFLEXSPISerialClk_133MHz
#endif

/// One LUT entry: two instructions (opcode, pad count, operand) each.
#define LUT(cmd0, pad0, op0, cmd1, pad1, op1) \
    FSL_ROM_FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1)

/// The first LUT word of command sequence `seq` (four words per sequence).
#define SEQ(seq) (4u * (seq))

/// Placed at the start of flash, where the boot ROM reads it. Nothing refers to
/// it, so the link keeps it by name (-u in port.cmake) and the linker script
/// KEEPs its section.
extern const flexspi_nor_config_t rosette_imxrt_flexspi_nor_config
    __attribute__((section(".boot_hdr.conf")));

const flexspi_nor_config_t rosette_imxrt_flexspi_nor_config = {
    .memConfig =
        {
            .tag = FLEXSPI_CFG_BLK_TAG,
            .version = FLEXSPI_CFG_BLK_VERSION,
            .readSampleClkSrc = kFLEXSPIReadSampleClk_LoopbackFromDqsPad,
            .csHoldTime = 3u,
            .csSetupTime = 3u,
            .deviceType = kFLEXSPIDeviceType_SerialNOR,
            .sflashPadType = kSerialFlash_4Pads,
            .serialClkFreq = ROSETTE_IMXRT_FLASH_SERIAL_CLK,
            .sflashA1Size = ROSETTE_IMXRT_FLASH_SIZE_BYTES,
            .lookupTable =
                {
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_READ)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 24),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_READ) + 1] =
                        LUT(DUMMY_SDR, FLEXSPI_4PAD, 6, READ_SDR, FLEXSPI_4PAD, 4),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_READSTATUS)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 4),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_WRITEENABLE)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_ERASESECTOR)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 24),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0xD8, RADDR_SDR, FLEXSPI_1PAD, 24),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 24),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM) + 1] =
                        LUT(WRITE_SDR, FLEXSPI_1PAD, 4, STOP, FLEXSPI_1PAD, 0),
                    [SEQ(NOR_CMD_LUT_SEQ_IDX_CHIPERASE)] =
                        LUT(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0),
                },
        },
    .pageSize = 256u,
    .sectorSize = 4096u,
    .ipcmdSerialClkFreq = kFLEXSPISerialClk_30MHz,
    .blockSize = 0x10000u,
};
