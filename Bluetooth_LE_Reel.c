#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "btstack_config.h"
#include "btstack.h"

#define DEVICE_NAME "PicoW_Reel"

// Define the UUIDs for your custom BLE service and characteristic
static const uint8_t custom_service_uuid[] = { 0x34, 0x12 };  // 0x1234
static const uint8_t custom_char_uuid[] = { 0x78, 0x56 };     // 0x5678

// Callback function to handle received data
static int custom_write_callback(uint16_t connection_handle, uint16_t attribute_handle, uint8_t *buffer, uint16_t length) {
    printf("Received data: ");
    for (int i = 0; i < length; i++) {
        printf("%c", buffer[i]);  // Print received characters
    }
    printf("\n");
    return 0;  // Return success
}

// Function to add the custom service to the GATT database
static void add_custom_ble_service(void) {
    att_db_util_add_service_uuid16(0x1234);  // Add Custom Service

    // Add Write Characteristic (allows sending data to the Pico W)
    att_db_util_add_characteristic_uuid16(
        0x5678,                              // Characteristic UUID
        ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, // Write permissions
        ATT_SECURITY_NONE,                   // No authentication required
        ATT_SECURITY_NONE,                   // No encryption required
        custom_write_callback,               // Function that handles received data
        NULL                                 // No extra data needed
    );
}




// Declare attribute database (needed for att_server_init)
static const uint8_t *att_db;  

// Callback function for receiving BLE data
static void nus_rx_callback(uint16_t conn_handle, uint16_t attribute_handle, uint8_t *buffer, uint16_t length) {
    printf("Received Data: ");
    for (int i = 0; i < length; i++) {
        printf("%c", buffer[i]);  // Print received characters
    }
    printf("\n");
}


// Function to setup BLE GATT service
static void nus_gatt_setup(void) {
    static const uint8_t nus_service_uuid[] = { 0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
    static const uint8_t nus_char_tx_uuid[] = { 0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
    static const uint8_t nus_char_rx_uuid[] = { 0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };

    att_db_util_add_service_uuid128(nus_service_uuid);

    uint16_t tx_handle = att_db_util_add_characteristic_uuid128(
        nus_char_tx_uuid,
        ATT_PROPERTY_NOTIFY, 
        ATT_SECURITY_NONE, 
        ATT_SECURITY_NONE, 
        NULL, 
        NULL
    );
    
    uint16_t rx_handle = att_db_util_add_characteristic_uuid128(
        nus_char_rx_uuid,
        ATT_PROPERTY_WRITE_WITHOUT_RESPONSE,
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        (att_write_callback_t) &nus_rx_callback,
        NULL
    );
    

    // Set the attribute database pointer
    att_db = att_db_util_get_address();
}

// Initialize Bluetooth Stack
static void btstack_setup(void) {
    l2cap_init();
    sm_init();
    nus_gatt_setup();
    
    add_custom_ble_service();

    // Initialize the Attribute Server
    att_server_init(att_db, NULL, NULL);

    // Set advertising parameters
    gap_advertisements_set_params(0x0800, 0x0800, 0, 0, NULL, 0, 0x07);

    // Set advertising data
    const uint8_t adv_data[] = { 
        3, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x34, 0x12,
        strlen(DEVICE_NAME) + 1, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', 'W', '_', 'R', 'e', 'e', 'l' 
    };
    
    gap_advertisements_set_data(sizeof(adv_data), adv_data);
    
    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);

    // Start advertising
    gap_advertisements_enable(1);
    
    printf("Bluetooth Initialized. Advertising as '%s'...\n", DEVICE_NAME);
}


// Main function
int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("Initializing...\n");
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi/BLE chip.\n");
        return -1;
    }
    printf("Starting Nordic BLE Server...\n");

    btstack_setup();
    hci_power_control(HCI_POWER_ON);

    printf("Success!\n");

    while (1) {
        sleep_ms(1000);
        printf("Running as %s\n", DEVICE_NAME);
    }

    return 0;
}
