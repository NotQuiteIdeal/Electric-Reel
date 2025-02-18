# Generated CMake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Bluetooth_LE_Reel C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Bluetooth_LE_Reel Bluetooth_LE_Reel.c)

pico_set_program_name(Bluetooth_LE_Reel "Bluetooth_LE_Reel")
pico_set_program_version(Bluetooth_LE_Reel "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Bluetooth_LE_Reel 0)
pico_enable_stdio_usb(Bluetooth_LE_Reel 1)

# Add the standard library to the build
target_link_libraries(Bluetooth_LE_Reel
        pico_stdlib
        #pico_btstack  # Add BTstack support for BLE
        )

# Add the standard include files to the build
target_include_directories(Bluetooth_LE_Reel PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${PICO_SDK_PATH}/lib/btstack/src
        ${PICO_SDK_PATH}/lib/lwip/src/include
)

#add_subdirectory(${PICO_SDK_PATH}/lib/btstack ${CMAKE_BINARY_DIR}/btstack)

# Add any user requested libraries
target_link_libraries(Bluetooth_LE_Reel  
    pico_btstack_cyw43
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_lwip_iperf
    pico_cyw43_arch_lwip_threadsafe_background
)


pico_add_extra_outputs(Bluetooth_LE_Reel)
