# boards/mimxrt1020-evk/bsp.cmake — this board's own composition, added to rosette_bsp.
#
#   enet_zero_copy_interface.cpp   ports/net/freertos_tcp/board_interface.hpp over
#                                the RT1021's ENET in the stack's own buffers
#                                (ports/chip/imxrt/imxrt_enet_zero_copy_device.hpp),
#                                with ROSETTE_IMXRT_ENET_ZERO_COPY (the default)
#   enet_copying_interface.cpp   the same over the copying ENET device
#                                (ports/chip/imxrt/imxrt_enet_mac_device.hpp),
#                                without it
#   ksz8081_phy.cpp              the PHY's RMII clock mode, for either
if(ROSETTE_IMXRT_ENET_ZERO_COPY)
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/enet_zero_copy_interface.cpp)
else()
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/enet_copying_interface.cpp)
endif()
target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ksz8081_phy.cpp)

# The FreeRTOS heap (FreeRTOSConfig.h; board_init.c's ucHeap in the OCRAM), at
# twice its measured peak: 16 KiB of task stacks and kernel objects with the
# zero-copy ENET, 31 KiB once the copying ENET's network buffers come from it
# too. The kernel and board_init.c both take it from freertos_config.
if(ROSETTE_IMXRT_ENET_ZERO_COPY)
  target_compile_definitions(freertos_config INTERFACE ROSETTE_EVK_FREERTOS_HEAP_KIB=32)
else()
  target_compile_definitions(freertos_config INTERFACE ROSETTE_EVK_FREERTOS_HEAP_KIB=64)
endif()
