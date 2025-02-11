#include <stdio.h>
#include "pico/stdlib.h"
#include "btstack.h"
#include "hci.h"
#include "pico/cyw43_arch.h"


// Bluetooth event callback
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type == HCI_EVENT_PACKET) {
        uint8_t event_type = hci_event_packet_get_type(packet);
        printf("BT Event: %d\n", event_type);  

        switch (event_type) {
            case BTSTACK_EVENT_STATE:
                printf("BTSTACK_EVENT_STATE Received\n");  
                if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                    printf("BLE Server is up and running!\n");
                }
                break;

            case HCI_EVENT_LE_META:
                printf("HCI_EVENT_LE_META Received\n");  
                if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_ADVERTISING_REPORT) {
                    printf("Advertising...\n");
                }
                break;

            default:
                printf("Unhandled Event: %d\n", event_type);
                break;
        }
    }
}

int main() {
    // Initialize standard input/output
    stdio_init_all();
    sleep_ms(5000);
    printf("Starting BLE Server...\n");

    if (cyw43_arch_init()) {
        printf("CYW43 init failed!\n");
        return -1;
    }
    printf("CYW43 Initialized!\n");

    //hci_init();
    printf("HCI Initialized!\n");
    // Initialize BTstack memory
    btstack_memory_init();
    printf("Memory Initialized!\n");
    
    // Use base run loop initialization
    btstack_run_loop_base_init();
    printf("Run Loop Base Initialized!\n");

    // Register Bluetooth event callback
    btstack_packet_callback_registration_t event_callback;
    printf("btstack_packet_callback_registration_t event_callback;\n");
    event_callback.callback = &packet_handler;
    printf("event_callback.callback = &packet_handler;\n");
    hci_add_event_handler(&event_callback);
    printf("hci_add_event_handler(&event_callback);\n");
    printf("Event Handling Set Up!\n");

    // Enable Bluetooth
    hci_power_control(HCI_POWER_ON);
    printf("Power On!\n");

    // Run the BTstack loop
    btstack_run_loop_execute();
    printf("Running Loop!\n");

    return 0;
}


