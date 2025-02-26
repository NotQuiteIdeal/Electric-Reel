# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(BluetoothLE_Server_Reel C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable for Bluetooth Server
add_executable(BluetoothLE_Server_Reel
    BluetoothLE_Server_Reel.c server_common.c
)

pico_set_program_name(BluetoothLE_Server_Reel "BluetoothLE_Server_Reel")
pico_set_program_version(BluetoothLE_Server_Reel "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(BluetoothLE_Server_Reel 0)
pico_enable_stdio_usb(BluetoothLE_Server_Reel 1)

# Add the standard library to the build
target_link_libraries(BluetoothLE_Server_Reel
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(BluetoothLE_Server_Reel PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/generated/BluetoothLE_Server_Reel_gatt_header
)

pico_btstack_make_gatt_header(BluetoothLE_Server_Reel PRIVATE "${CMAKE_CURRENT_LIST_DIR}/Reel.gatt")

pico_add_extra_outputs(BluetoothLE_Server_Reel)

# Add any user requested libraries
target_link_libraries(BluetoothLE_Server_Reel 
        hardware_timer
        hardware_clocks
        pico_btstack_ble
        pico_btstack_cyw43
        pico_cyw43_arch_none
)

pico_add_extra_outputs(BluetoothLE_Server_Reel)


####### Add BluetoothLE Client Build


# Create a separate executable for the BLE Client
add_executable(BluetoothLE_Client_Reel
    client.c
)

pico_set_program_name(BluetoothLE_Client_Reel "BluetoothLE_Client_Reel")
pico_set_program_version(BluetoothLE_Client_Reel "0.1")

pico_enable_stdio_uart(BluetoothLE_Client_Reel 0)
pico_enable_stdio_usb(BluetoothLE_Client_Reel 1)

target_include_directories(BluetoothLE_Client_Reel PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated/BluetoothLE_Client_Reel_gatt_header
    ${BTSTACK_ROOT}/src
    ${BTSTACK_ROOT}/src/ble
    )

target_link_libraries(BluetoothLE_Client_Reel
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none

)

target_compile_definitions(BluetoothLE_Client_Reel PRIVATE
    ENABLE_BLE
    ENABLE_LE_CENTRAL
    RUNNING_AS_CLIENT
    -DENABLE_GATT_CLIENT
    -DENABLE_ATT_CLIENT
)


pico_add_extra_outputs(BluetoothLE_Client_Reel)
