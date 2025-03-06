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
extern uint8_t ping_test_status;
extern int ping_rate_count;
bool ping_start = false;
struct repeating_timer timer2;

// Bluetooth connection handle
extern hci_con_handle_t con_handle;
extern int le_notification_enabled;
extern volatile int send_update_flag;  // External update trigger


bool ping_rate_callback(struct repeating_timer *q) {
    ping_test_status = 2; // Indicates that ping test is complete
    return false;
}

// Timer callback to send pings
bool ping_timer_callback(struct repeating_timer *t) {
    if (ping_test_status == 1 && ping_start == false) {
        ping_start = true;
        bool ok = add_repeating_timer_ms(5000, ping_rate_callback, NULL, &timer2);
        printf("Timer started? %d\n", ok); // 1 if started, 0 if failed
    }
    if (ping_test_status == 1) {
        att_server_request_can_send_now_event(con_handle);
    }
    if (ping_test_status == 2) {
        float ping_rate = ping_rate_count / 5.0;
        printf("Ping test concluded! Average ping rate per second: %.2f\n", ping_rate);
        ping_test_status = 3;
        return false;
    }
    return true;
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
    struct repeating_timer timer1;
    add_repeating_timer_ms(-175, ping_timer_callback, NULL, &timer1);

    
    // Main loop to update values
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

