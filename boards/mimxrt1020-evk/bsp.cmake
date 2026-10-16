# boards/mimxrt1020-evk/bsp.cmake — this board's own composition, added to rosette_bsp.
#
#   enet_interface.cpp   ports/net/freertos_tcp/board_interface.hpp over the
#                        RT1021's ENET and the KSZ8081 PHY
#                        (ports/chip/imxrt/imxrt_enet_mac_device.hpp)
target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/enet_interface.cpp)
