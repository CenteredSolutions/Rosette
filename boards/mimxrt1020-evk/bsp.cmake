# boards/mimxrt1020-evk/bsp.cmake — this board's own composition, added to rosette_bsp.
#
#   enet_lending_interface.cpp   ports/net/freertos_tcp/board_interface.hpp over
#                                the RT1021's ENET lent the stack's buffers
#                                (ports/chip/imxrt/imxrt_enet_lending_device.hpp),
#                                with ROSETTE_IMXRT_ENET_LENDING (the default)
#   enet_copying_interface.cpp   the same over the copying ENET device
#                                (ports/chip/imxrt/imxrt_enet_mac_device.hpp),
#                                without it
#   ksz8081_phy.cpp              the PHY's RMII clock mode, for either
if(ROSETTE_IMXRT_ENET_LENDING)
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/enet_lending_interface.cpp)
else()
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/enet_copying_interface.cpp)
endif()
target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ksz8081_phy.cpp)
