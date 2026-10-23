# i.MX RT1021 memory budget: rtps_chatter on the MIMXRT1020-EVK

The RT1021 has 256 KiB of on-chip RAM, all of it FlexRAM: eight 32 KiB banks, each of which serves as ITCM, DTCM,
or OCRAM. Rosette's RTPS stack, FreeRTOS, and FreeRTOS+TCP with the zero-copy ENET fit in about two fifths of it.
This document gives the budget, the layout that holds it, the work that gets `rtps_chatter` onto the EVK, and the
checks that keep the budget honest as the image grows toward `rclc_chatter`.

## The regions

| Region | Fuse default | Properties | Holds |
|---|---|---|---|
| ITCM | 64 KiB at `0x00000000` | single-cycle instruction and data, uncached | nothing (code executes in place from QSPI flash through the 16 KiB I-cache) |
| DTCM | 64 KiB at `0x20000000` | single-cycle, uncached, reachable by the ENET's DMA | `.data`, `.bss`, the newlib heap, the main stack (MSP) |
| OCRAM | 128 KiB at `0x20200000` | on the AXI bus behind the 16 KiB D-cache | the FreeRTOS heap (`ROSETTE_IMXRT_OCRAM_BSS`) |

The DTCM is the right default home for data: it is the fastest RAM on the part, and because it is uncached a DMA
buffer placed there is coherent with no cache maintenance. The OCRAM suits data only the CPU touches.

## Capacity is sized to what the firmware creates

Every writer, reader, proxy, and name octet is RAM in the participant whether the application uses it or not. A
firmware that states what it creates gets a participant sized for exactly that: `cmake/rosette_rtps.cmake` takes
`ROSETTE_RTPS_MAX_WRITERS`, `_MAX_READERS`, `_MAX_WRITER_DEPTH`, `_MAX_MATCHED_ENDPOINTS`, `_MATCHED_READERS`,
`_MATCHED_WRITERS`, `_MATCH_WAITERS`, and `_LOCAL_NAME_BYTES`, the same model `rmw_rosette`'s `RMW_ROSETTE_*`
capacity inputs apply one layer up (`rmw_rosette/cmake/rmw_rosette_capacity.cmake`). `rtps_chatter` creates one
writer of depth 4 and one reader, whose names take 67 octets, and says so in its `CMakeLists.txt`. The network-sized
inputs (peers, remote endpoints, matches per endpoint) stay at their board or command-line values.

`rtps_chatter` on the W5100S-EVB-Pico2 (`build-hw/pico2`), the same 32-bit ABI and RTPS configuration the EVK
builds:

| Symbol | Sized for 4 writers and 4 readers | Sized for what it creates |
|---|---|---|
| `main::participant` (`rtps::RTPSParticipant`: endpoints, proxies, peer tables, match records) | 26,864 | 13,664 |
| `rtps::g_buf_pool` (`MultiBufPool`: the 256 / 512 / 1024 / datagram / sample classes) | 26,988 | 26,988 |
| `g_rx_buffer` (one datagram) | 1,472 | 1,472 |
| transport, driver, writer, reader handles | about 200 | about 200 |
| **RTPS total** | **55.2 KiB** | **41.3 KiB** |
| image `.data` + `.bss` | 70,256 | 57,056 |

The buffer pool is now the largest item. Its classes are sized by formula; sizing them from use needs the
requested-size histogram (`RTPS_BUFPOOL_DEBUG`) and a workload on the bench first, which step 1 below provides.

## Measured on the EVK

Every image reports its reservations' peaks: `rosette::platform::format_memory_report()`
(`rosette/platform/memory_report.hpp`) over the platform contract's `main_stack_use()`, `c_heap_use()`,
`rtos_heap_use()`, and `visit_task_stacks()`, with the network buffers from `net::board_network_buffer_use()`.
`udp_echo` prints them on the console and answers them on its status port (UDP 8), and every link prints each
region's fill (`--print-memory-usage`).

`udp_echo` with the zero-copy ENET (`build-hw/rt1020-evk`), after the echo probe, a 5000-frame ping flood at 1400
octets, a 5 s UDP blast, and the probe again; the reservations are sized from these peaks:

| Item | Region | Peak | Reserved |
|---|---|---|---|
| `.data` | DTCM | 2,892 B | 2,892 B |
| `.bss`: network buffers (`g_pool`, 15,424 B), ENET rings, FreeRTOS+TCP and kernel statics | DTCM | 22,556 B | 22,556 B |
| newlib heap (`ROSETTE_IMXRT_HEAP_BYTES`, `board.cmake`) | DTCM | 3,168 B | 8,192 B |
| main stack, MSP (`ROSETTE_IMXRT_STACK_BYTES`, `board.cmake`): `main()` before the scheduler, then interrupts | DTCM | 1,568 B | 4,096 B |
| FreeRTOS heap (`bsp.cmake`) | OCRAM | 16,120 B | 32,768 B |
| task `main` (`ROSETTE_FREERTOS_MAIN_STACK_WORDS`) | FreeRTOS heap | 3,460 B | 8,188 B |
| task `IP-task` / `enet0_rx` | FreeRTOS heap | 768 / 348 B | 2,044 B each |
| network buffers | DTCM (`g_pool`) | 10 of 10 (the flood) | 10 |
| **DTCM** | | | **37,740 of 65,536 B** |

The copying ENET (`-DROSETTE_IMXRT_ENET_ZERO_COPY=OFF`) under the same workload peaks at 5,280 B of newlib heap and
31,480 B of FreeRTOS heap, whose reservation is 64 KiB in that build because its network buffers come from it.

The task stacks keep their sizes until `rtps_chatter` runs: RTPS processing deepens `main` and `IP-task`. The
fuse-default DTCM has no room for RTPS beside the network; the part as a whole has room to spare.

## The budget

`rtps_chatter` on the EVK, each line measured above or sized from a measurement:

| Item | Bytes | Basis |
|---|---|---|
| `.data` | 3 KiB | measured (`udp_echo`) |
| network `.bss` | 22.6 KiB | measured |
| RTPS state | 41.3 KiB | measured on pico2, sized for what `rtps_chatter` creates |
| FreeRTOS heap | 32 KiB | 16 KiB peak measured, doubled |
| newlib heap | 8 KiB | 3.1 KiB peak measured |
| MSP | 4 KiB | 1.5 KiB peak measured |
| **Total** | **about 111 KiB of 256 KiB (43 %)** | |

The EVK's link of `rtps_chatter` confirms the DTCM share: 78,560 B (`.data` and `.bss` 66,272 B, of which
`g_buf_pool` is 26,988, the network buffers 15,424, and the participant 13,664, plus the newlib heap and the
MSP), 13,024 B more than the fuse-default 64 KiB DTCM holds and 60 % of the 128 KiB the layout below gives it.

## The layout

Reallocate the FlexRAM banks at reset: **ITCM 0, DTCM 128 KiB (4 banks), OCRAM 128 KiB (4 banks)**.

| Region | Size | Holds | `rtps_chatter` |
|---|---|---|---|
| DTCM | 128 KiB | everything the linker places by default: `.data`, `.bss` (the network, RTPS, the application), the newlib heap, the MSP | about 79 of 128 KiB |
| OCRAM | 128 KiB | the FreeRTOS heap; large CPU-only pools when DTCM runs short | 32 of 128 KiB |

Why this layout:

- **The default section stays the right answer.** Rosette's statics, the application's, and the DMA buffers all
  land in fast, uncached, DMA-reachable RAM with no placement attributes in portable code and no per-library rules
  in the linker script.
- **No RAM sits idle.** The fuse default leaves 64 KiB of ITCM unused, a quarter of the part. Execute-in-place from
  flash through the I-cache is fast enough for a network node; when a hot path (an ISR, a checksum) wants ITCM, one
  bank can come back (ITCM 32, DTCM 128, OCRAM 96).
- **The OCRAM keeps a clear role** as the RTOS heap and the overflow for large CPU-only storage, where the D-cache
  hides its latency.

The fuse default also holds this image, with RTPS moved to the OCRAM by a linker rule: DTCM about 37 of 64 KiB,
OCRAM about 73 of 128 KiB. It needs no reset hook, but it leaves the ITCM idle and makes the linker script name
Rosette's libraries, so it is the fallback rather than the plan.

The reallocation's cost is a port-owned reset handler (`ports/chip/imxrt/flexram_reset_handler.S`). The boot ROM
applies the fuse layout and enters `Reset_Handler` with the stack pointer loaded from the vector table, which is the
top of the board's 128 KiB DTCM and so lies past the fuse layout's 64 KiB; the new layout must be in place before
the stack is used. The handler writes `IOMUXC_GPR17` (`FLEXRAM_BANK_CFG`, `0xAA55`: banks 0 to 3 OCRAM, 4 to 7 DTCM)
and sets `GPR16.FLEXRAM_BANK_CFG_SEL` using registers alone, the same two writes as the SDK's
`FLEXRAM_AllocateRam()` for the RT1021, then runs the SDK startup's sequence: VTOR and the MSP, `SystemInit()`,
the `.data` copy, and the `.bss` clear. The RT1021 sizes its TCM interfaces from the bank assignment, so it has no
`GPR14` TCM-size fields to write; the RT1062 does, which the Teensy 4 core's `ResetHandler()` writes. The SDK
defines its `Reset_Handler` weak, so the port's strong one replaces it only in a board that sets a layout. The
layout is a board parameter in `board.cmake` (`ROSETTE_IMXRT_FLEXRAM_ITCM_KIB`, `_DTCM_KIB`, `_OCRAM_KIB`), from
which `ports/chip/imxrt/flexram_layout.cmake` derives both the `GPR17` value and the linker `MEMORY` lengths, so the
two cannot disagree; a board that keeps the fuse default (the Teensy MicroMod) changes nothing.

## Order of work

1. **Measure what is reserved.** The memory report above, and the reservations sized from it. The RTPS pool's
   per-class high-water marks come from `RTPS_BUFPOOL_DEBUG` once `rtps_chatter` runs (step 4).
2. **Make `rtps_chatter` board-neutral.** Replace `pico/stdlib.h`, `stdio_usb_connected()`, and `sleep_ms()` with
   `rosette::platform`, run the loop through `platform::run_main_task()` as `udp_echo` does, and gate the example on
   a board network rather than on WIZnet and the Pico SDK. It must still build and run on the W5100S-EVB-Pico2. On
   the EVK it links only once step 3 lands; the overflow the linker reports is the budget's first check.
3. **FlexRAM layout as a board parameter.** The reset hook and the generated `MEMORY` regions in
   `ports/chip/imxrt/`, and the EVK set to ITCM 0, DTCM 128, OCRAM 128. Verified on the bench
   (`tools/imxrt_flexram_check.py`): `GPR16` and `GPR17` read back as written (`0x00200007`, `0xAA55`); an
   address-tagged pattern in both polarities over all 128 KiB of DTCM and 128 KiB of OCRAM passes, and the word past
   the DTCM's end faults; `rosette_platform_contract` and the `udp_echo` probe and ping flood still pass. The part
   accepts a zero-size ITCM; the fallback, if a hot path wants ITCM, is ITCM 32, DTCM 128, OCRAM 96.
4. **`rtps_chatter` on the wire.** Against `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp`: `ros2 topic echo
   /rosette/chatter` receives the board's samples, and samples published on `/rosette/command` reach the board. The
   writer, the reader, and their names fit the participant sized for them (no `RTPS_LOCAL_NAME_BYTES` or table-full
   log line). SPDP multicast is the first real test of the ENET's hash filter and IGMP. Record the measured budget in
   this document.
5. **Guard the budget.** Each region keeps a stated headroom (DTCM and OCRAM at least 16 KiB free), enforced by an
   `ASSERT` in the generated linker script, so an image that eats the margin fails to link instead of failing on
   the bench.

## Beyond rtps_chatter

`rclc_chatter` creates one publisher and one subscription, which `rmw_rosette` turns into two RTPS writers (the
publisher and `ros_discovery_info`) and one reader. Sized for that, with 48 known participants for a demo network,
its participant takes 19.4 KiB and its buffer pool 31.7 KiB (the firmware pool profile), against 55.6 KiB and
34.3 KiB at the firmware defaults of four publishers, four subscriptions, four services, and two clients with 96
known participants. Each known participant costs 68 octets. With `rmw_rosette`'s entity pools, its receive buffer,
and the rcl heap (4 KiB in use on the pico2), `rclc_chatter` needs about 18 KiB more than `rtps_chatter`: about
129 KiB of 256 KiB in all, and DTCM about 97 of 128 KiB, with no pool moved out of it. The rcl heap is newlib's, so
that image's newlib heap grows to about 12 KiB, sized again from its own report.

The rmw capacity inputs must be set before `cmake/rmw_rosette_firmware.cmake` first runs, since `led_files` and
`rclc_chatter` share the `rmw_rosette_firmware` target it creates. Giving each example its own capacity is part of
planning that step.

## References

- i.MX RT1020 Processor Reference Manual: system memory map; FlexRAM chapter; IOMUXC_GPR `GPR16`, `GPR17`.
- NXP AN12077, "Using the i.MX RT FlexRAM".
- MCUXpresso SDK `devices/MIMXRT1021/drivers/fsl_flexram_allocate.h` (`FLEXRAM_AllocateRam()`); bank size and count
  from `MIMXRT1021_features.h` (`FSL_FEATURE_FLEXRAM_INTERNAL_RAM_BANK_SIZE`, `..._TOTAL_BANK_NUMBERS`).
- Teensy 4 core, `cores/teensy4/startup.c` `ResetHandler()`: FlexRAM configured before the stack is used.
- CMake policy CMP0126 (`set(CACHE)` and a normal variable of the same name), which `cmake/rosette_rtps.cmake`
  sets so that a firmware's stated capacity wins over the cache default.
- `docs/design/imxrt_enet_mac_device.md`: the ENET and its buffers, whose placement this layout keeps in DTCM.
