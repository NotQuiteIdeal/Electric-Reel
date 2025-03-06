#include <stdio.h>
#include "btstack.h"
#include "server_common.h"
#include "Reel.h"
#include "pico/time.h"

#define PING_CHAR_VALUE_HANDLE          0x0020
#define PING_CHAR_CCCD_HANDLE           0x0021
#define LINE_LENGTH_CHAR_VALUE_HANDLE   0x0009
#define LINE_LENGTH_CHAR_CCCD_HANDLE    0x000a
#define DRAG_SET_CHAR_VALUE_HANDLE      0x000c
#define DRAG_SET_CHAR_CCCD_HANDLE       0x000d
#define MOTOR_STATUS_CHAR_VALUE_HANDLE  0x0010
#define MOTOR_STATUS_CHAR_CCCD_HANDLE   0x0011
#define MOTOR_SPEED_VALUE_HANDLE        0x0013
#define MOTOR_SPEED_CCCD_HANDLE         0x0014
#define FISH_ALARM_VALUE_HANDLE         0x0016
#define FISH_ALARM_CCCD_HANDLE          0x0017
#define AUTO_STOP_LENGTH_VALUE_HANDLE   0x0019
#define AUTO_STOP_LENGTH_CCCD_HANDLE    0x001a
#define MEASUREMENT_SYtSEM_VALUE_HANDLE 0x001c
#define MEASUREMENT_SYSTEM_CCCD_HANDLE  0x001d

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
static uint64_t ping_sent_time = 0;
uint8_t ping_test_status = 0;
static btstack_timer_source_t refresh_timer;
bool refresh_timer_started = false;
int ping_rate_count = 0;

// Update flag for external updates
volatile int send_update_flag = 0;

// Function to send a ping notification
void send_ping_notification() {

    if (ping_test_status != 1) {
        printf("Skipping final ping, test is done.\n");
        return;
    }

    absolute_time_t current_time = get_absolute_time();
    ping_sent_time = to_us_since_boot(current_time);

    printf("Sent Ping Request at: %llu us\n", ping_sent_time);

    uint8_t status = att_server_notify(con_handle, PING_CHAR_VALUE_HANDLE, (uint8_t*)&ping_sent_time, sizeof(ping_sent_time));

    if (status == 0) {
        printf("Ping Notification successfully sent.\n");
        ping_test_status = 1; // Prevents sending another ping until response is received
    } else {
        printf("Ping Notification failed! Status: %d\n", status);
    }
}

void send_ble_updates() {
    att_server_notify(con_handle, LINE_LENGTH_CHAR_VALUE_HANDLE, (uint8_t*)&line_length, sizeof(line_length));
    att_server_notify(con_handle, DRAG_SET_CHAR_VALUE_HANDLE, (uint8_t*)&drag_set, sizeof(drag_set));
    att_server_notify(con_handle, MOTOR_STATUS_CHAR_VALUE_HANDLE, (uint8_t*)&motor_status, sizeof(motor_status));
    att_server_notify(con_handle, ATT_CHARACTERISTIC_cb8822a5_38c0_41fd_8c2b_d33fde778187_01_VALUE_HANDLE, (uint8_t*)&motor_speed, sizeof(motor_speed));
    att_server_notify(con_handle, ATT_CHARACTERISTIC_d45efe09_1eee_47a7_9026_0c4152740a66_01_VALUE_HANDLE, (uint8_t*)&fish_alarm, sizeof(fish_alarm));
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

        case ATT_EVENT_CAN_SEND_NOW:
            if (ping_test_status == 1) {
                send_ping_notification();
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
                gap_request_connection_parameter_update(con_handle, 16, 32, 0, 200); 
                // Min: 10ms (8*1.25ms), Max: 20ms (16*1.25ms)

                printf("Waiting for client write before beginning tests...\n");
                
                /* Temporarily removed for testing other ping route
                // Record the start time
                absolute_time_t start_time = get_absolute_time();
                ping_sent_time = to_us_since_boot(start_time);
                ping_test_status = 1;
        
                printf("Starting Ping Test at: %u us\n", ping_sent_time);
                
                if (ping_test_status == 1) {
                    // Start sending pings
                    send_ping_notification();
                    ping_test_status = 2;
                }
                */
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
    printf("Received write on handle: 0x%04X\n", attribute_handle);
    if (attribute_handle == PING_CHAR_VALUE_HANDLE) {
        if (ping_test_status == 1) {
            absolute_time_t server_received_time = get_absolute_time();
            uint32_t server_timestamp = to_us_since_boot(server_received_time);

            uint32_t round_trip_time = (server_timestamp > ping_sent_time) ? (server_timestamp - ping_sent_time) : 0;

            printf("Received Ping Response from Client!\n");
            printf("Round-trip time: %u us (%.3f ms)\n", round_trip_time, round_trip_time / 1000.0);
            ping_rate_count++;
            printf("%d\n", ping_rate_count);
        }
        
        if (buffer_size > 0) {
            printf("Received write on handle: 0x%04X, Value: %02X\n", attribute_handle, buffer[0]);
        } else {
            printf("WARNING: Empty write received from client!\n");
        }
    }
    if (attribute_handle == PING_CHAR_CCCD_HANDLE) {
        if (buffer_size == 2) {
            uint16_t cccd_value = little_endian_read_16(buffer, 0);

            // Check if any\thing is enabled (0x0001 for notifications, 0x0002 for indications)
            if (cccd_value & GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) {
                printf("Client subscribed to notifications for Ping Characteristic.\n");
                
                
                ping_test_status = 1;
            } else {
                printf("Client unsubscribed from notifications.");
                ping_test_status = 0;
            }
        }
    }
    
    return 0;
}

