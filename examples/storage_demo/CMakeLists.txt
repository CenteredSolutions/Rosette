# storage_demo — exercises rosette::storage (Volume / FileStore / DeviceConfig)
# against the host file-backed volume. Host build only.

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    return()
endif()

add_executable(storage_demo main.cpp)
target_link_libraries(storage_demo PRIVATE rosette_storage)
