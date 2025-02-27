#include <stdio.h>
#include "btstack.h"
#include "server_common.h"
#include "Reel.h"
#include "pico/time.h"


extern const uint8_t profile_data[];

// Advertisement Data
#define APP_AD_FLAGS 0x06
static uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x0F, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', '_', 'R', 'e', 'e', 'l',
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS
};

static const uint8_t adv_data_len = sizeof(adv_data);

// Bluetooth connection state
int le_notification_enabled = 0;
hci_con_handle_t con_handle = 0;

// Reel characteristic values
uint16_t line_length = 0;
uint16_t drag_set = 0;
uint8_t motor_status = 0;
uint8_t motor_speed = 0;
uint8_t fish_alarm = 0;

// Variables for testing ping
absolute_time_t ping_received_time;
static uint32_t received_ping_count = 0;
static uint32_t ping_sent_time = 0;
static btstack_timer_source_t refresh_timer;

// Update flag for external updates
volatile int send_update_flag = 0;

// Function to send a ping notification
void send_ping_notification() {
    if (le_notification_enabled) {
        absolute_time_t current_time = get_absolute_time();
        ping_sent_time = to_us_since_boot(current_time);

        printf("Sent Ping Request at: %llu us\n", ping_sent_time);

        uint8_t status = att_server_indicate(con_handle, ATT_CHARACTERISTIC_a4b7e118_7b77_4ef9_a618_967a2842e630_01_VALUE_HANDLE, (uint8_t*)&ping_sent_time, sizeof(ping_sent_time));

        printf("Ping Indication status: %d\n", status);
    }
}


void send_ble_updates() {
    if (le_notification_enabled) {
        att_server_notify(con_handle, ATT_CHARACTERISTIC_1476a75a_2c6d_4649_8819_bb830daaa603_01_VALUE_HANDLE, (uint8_t*)&line_length, sizeof(line_length));
        att_server_notify(con_handle, ATT_CHARACTERISTIC_950e9e70_c453_4505_87e3_9dd6db626cc1_01_VALUE_HANDLE, (uint8_t*)&drag_set, sizeof(drag_set));
        att_server_notify(con_handle, ATT_CHARACTERISTIC_d966cdb4_f14c_4113_adb4_8c9925a29c52_01_VALUE_HANDLE, (uint8_t*)&motor_status, sizeof(motor_status));
        att_server_notify(con_handle, ATT_CHARACTERISTIC_cb8822a5_38c0_41fd_8c2b_d33fde778187_01_VALUE_HANDLE, (uint8_t*)&motor_speed, sizeof(motor_speed));
        att_server_notify(con_handle, ATT_CHARACTERISTIC_d45efe09_1eee_47a7_9026_0c4152740a66_01_VALUE_HANDLE, (uint8_t*)&fish_alarm, sizeof(fish_alarm));
    }
}

// Function to print refresh rate
void update_refresh_rate(btstack_timer_source_t *ts) {
    printf("Refresh Rate: %u pings per second\n", received_ping_count);
    
    // Reset the counter for the next measurement
    received_ping_count = 0;

    // Restart the timer for the next update
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}


// Call this function every second
void start_refresh_rate_timer() {
    btstack_run_loop_set_timer(&refresh_timer, 1000); // 1-second interval
    btstack_run_loop_set_timer_handler(&refresh_timer, update_refresh_rate);
    btstack_run_loop_add_timer(&refresh_timer);
}

// Packet handler for BLE events
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // Start BLE Advertising
            uint16_t adv_int_min = 100;
            uint16_t adv_int_max = 200;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);

            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // BLE limit
            
            // Print advertisement data before setting
            printf("Setting advertisement data: ");
            for (int i = 0; i < adv_data_len; i++) {
                printf("%02X ", adv_data[i]);
            }
            printf("\n");

            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1); // Ensure Advertising is enabled
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE: // Covers disconnection and timeout
            le_notification_enabled = 0;
            break;

        case ATT_EVENT_CAN_SEND_NOW: // Checks if event is 
            if (le_notification_enabled) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_1476a75a_2c6d_4649_8819_bb830daaa603_01_VALUE_HANDLE, (uint8_t*)&line_length, sizeof(line_length));
                att_server_notify(con_handle, ATT_CHARACTERISTIC_950e9e70_c453_4505_87e3_9dd6db626cc1_01_VALUE_HANDLE, (uint8_t*)&drag_set, sizeof(drag_set));
            }
            break;
        
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
        
                static bool connection_handled = false;
                if (connection_handled) break;
                connection_handled = true;
        
                // Request a faster connection interval
                printf("Client Connected! Requesting faster connection interval...\n");
                gap_request_connection_parameter_update(con_handle, 6, 12, 0, 200); 
                // Min: 10ms (8*1.25ms), Max: 20ms (16*1.25ms)
        
                // Record the start time
                absolute_time_t start_time = get_absolute_time();
                ping_sent_time = to_us_since_boot(start_time);
        
                printf("Starting Ping Test at: %u us\n", ping_sent_time);
        
                // Start sending pings
                send_ping_notification();
            }
            break;
        
        
        
        default:
            break;
    }
}

// Handle BLE Read Requests
uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    const char *descriptor_value = NULL;
    
    if (att_handle == ATT_CHARACTERISTIC_1476a75a_2c6d_4649_8819_bb830daaa603_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&line_length, sizeof(line_length), offset, buffer, buffer_size);
    } 
    if (att_handle == ATT_CHARACTERISTIC_950e9e70_c453_4505_87e3_9dd6db626cc1_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&drag_set, sizeof(drag_set), offset, buffer, buffer_size);
    }

    // Handle Characteristic User Descriptions (0x2901)
    if (att_handle == ATT_CHARACTERISTIC_1476a75a_2c6d_4649_8819_bb830daaa603_01_VALUE_HANDLE + 1) {
        descriptor_value = "Line Length";
    } 
    if (att_handle == ATT_CHARACTERISTIC_950e9e70_c453_4505_87e3_9dd6db626cc1_01_VALUE_HANDLE + 1) { //
        descriptor_value = "Drag Set Value";
    }

    if (descriptor_value) {
        return att_read_callback_handle_blob((const uint8_t *)descriptor_value, strlen(descriptor_value), offset, buffer, buffer_size);
    }

    return 0;
}


// Handle BLE Write Requests
int att_write_callback(hci_con_handle_t connection_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    if (attribute_handle == 0x018) { //Checks if Characteristic
        uint16_t cccd_value = little_endian_read_16(buffer, 0);
        if (cccd_value == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_INDICATION) {
            le_notification_enabled = 1;
            printf("Client enabled indications!\n");
        } else {
            le_notification_enabled = 0;
            printf("Client disabled indications.\n");
        }
        return 0; // Successfully handled CCCD write
    }

    if (attribute_handle == 0x0017) {  
        
        // Get the server's current timestamp when receiving the response
        absolute_time_t server_received_time = get_absolute_time();
        uint32_t server_timestamp = to_us_since_boot(server_received_time);

        // Calculate the round-trip time
        uint32_t round_trip_time = server_timestamp - ping_sent_time;

        printf("Received Ping Response from Client!\n");
        printf("Round-trip time: %u us (%.3f ms)\n", round_trip_time, round_trip_time / 1000.0);
        
        received_ping_count++;
        
        uint8_t new_ping_value = 1;  // Keep sending a value
        printf("Pinging!\n");
        send_ping_notification();

        uint8_t data_to_send = 42;
        att_server_notify(connection_handle, attribute_handle, &data_to_send, sizeof(data_to_send));

        if (buffer_size > 0) {
            printf("Client sent value: %02X\n", buffer[0]);
        } else {
            printf("WARNING: Empty write received from client!\n");
        }
        
        // Start the refresh rate timer
        start_refresh_rate_timer();
    }
    if (attribute_handle == 0x001a) { // Handle for ping rate tester only
        received_ping_count++;
    }
    return 0;
}





