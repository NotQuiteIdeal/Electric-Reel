#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdlib.h"
#include "server_common.h"


// Bluetooth event handlers
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Declare global BLE variables from server_common.c
extern uint16_t line_length;
extern uint16_t drag_set;
extern uint8_t motor_status;
extern uint8_t motor_speed;
extern uint8_t fish_alarm;

// Bluetooth connection handle
extern hci_con_handle_t con_handle;
extern int le_notification_enabled;
extern volatile int send_update_flag;  // External update trigger

// Timer callback to send pings
bool ping_timer_callback(struct repeating_timer *t) {
    send_ping_notification();
    return true;  // Keep the timer running
}




int main() {
    stdio_init_all();

    sleep_ms(5000);
    // Initialize CYW43 driver
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43 driver\n");
        return -1;
    }

    // Initialize Bluetooth
    l2cap_init();
    sm_init();
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // Register event handlers
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);

    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);

    printf("Bluetooth LE Server Running!\n");

    // Set up a repeating hardware timer to request BLE updates every 200ms
    struct repeating_timer timer;
    add_repeating_timer_ms(-200, ping_timer_callback, NULL, &timer);
    
    // Main loop to update values
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

