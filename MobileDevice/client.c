#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hci.h"
#include "gap.h"
#include "client.h"

#define LINE_LENGTH_CHAR_VALUE_HANDLE   0x0009
#define LINE_LENGTH_CHAR_CCCD_HANDLE    0x000a
#define DRAG_SET_CHAR_VALUE_HANDLE      0x000c
#define DRAG_SET_CHAR_CCCD_HANDLE       0x000d
#define MOTOR_STATUS_CHAR_VALUE_HANDLE  0x000f
#define MOTOR_STATUS_CHAR_CCCD_HANDLE   0x0010
#define MOTOR_SPEED_VALUE_HANDLE        0x0012
#define MOTOR_SPEED_CCCD_HANDLE         0x0013
#define FISH_ALARM_VALUE_HANDLE         0x0015
#define FISH_ALARM_CCCD_HANDLE          0x0016
#define AUTO_STOP_LENGTH_VALUE_HANDLE   0x0018
#define AUTO_STOP_LENGTH_CCCD_HANDLE    0x0019
#define MEASUREMENT_SYSTEM_VALUE_HANDLE 0x001b
#define MEASUREMENT_SYSTEM_CCCD_HANDLE  0x001c
#define PING_CHAR_VALUE_HANDLE          0x001e
#define PING_CHAR_CCCD_HANDLE           0x001f

// Global Variables
client_state_t state = CLIENT_IDLE;
static hci_con_handle_t connection_handle;
static gatt_client_service_t server_service;
static gatt_client_characteristic_t ping_characteristic;
static gatt_client_notification_t notification_listener;
static bd_addr_t server_addr;
static bd_addr_type_t server_addr_type;
volatile uint16_t line_length;
volatile uint16_t drag_set;
volatile uint8_t motor_status;
volatile uint8_t motor_speed;
volatile uint8_t fish_alarm;
volatile uint8_t auto_stop_length;
volatile uint8_t measurement_system;

// Value that is sent during ping test
uint8_t response_value = 0;

// UUIDs (match server)
static const uint8_t REEL_SERVICE_UUID[16] = {0xc0, 0xff, 0x29, 0xe1, 0x41, 0xa6, 0x40, 0xd9, 0x87, 0x35, 0x9d, 0x22, 0xae, 0x02, 0xd1, 0x29};
static const uint8_t LINE_LENGTH_CHARACTERISTIC_UUID[16] = {0x14, 0x76, 0xa7, 0x5a, 0x2c, 0x6d, 0x46, 0x49, 0x88, 0x19, 0xbb, 0x83, 0x0d, 0xaa, 0xa6, 0x03};
static const uint8_t DRAG_SET_CHARACTERISTIC_UUID[16] = {0x95, 0x0e, 0x9e, 0x70, 0xc4, 0x53, 0x45, 0x05, 0x87, 0xe3, 0x9d, 0xd6, 0xdb, 0x62, 0x6c, 0xc1};
static const uint8_t MOTOR_STATUS_CHARACTERISTIC_UUID[16] = {0xd9, 0x66, 0xcd, 0xb4, 0xf1, 0x4c, 0x41, 0x13, 0xad, 0xb4, 0x8c, 0x99, 0x25, 0xa2, 0x9c, 0x52};
static const uint8_t MOTOR_SPEED_CHARACTERISTIC_UUID[16] = {0xcb, 0x88, 0x22, 0xa5, 0x38, 0xc0, 0x41, 0xfd, 0x8c, 0x2b, 0xd3, 0x3f, 0xde, 0x77, 0x81, 0x87};
static const uint8_t FISH_ALARM_CHARACTERISTIC_UUID[16] = {0xd4, 0x5e, 0xfe, 0x09, 0x1e, 0xee, 0x47, 0xa7, 0x90, 0x26, 0x0c, 0x41, 0x52, 0x74, 0x0a, 0x66};
static const uint8_t AUTO_STOP_CHARACTERISTIC_UUID[16] = {0xf8, 0x33, 0x4e, 0x43, 0x52, 0x62, 0x4e, 0x64, 0xa0, 0xff, 0xcb, 0xde, 0xf3, 0xbf, 0xa3, 0x13};
static const uint8_t MEASUREMENT_SYSTEM_CHARACTERISTIC_UUID[16] = {0xd0, 0x1d, 0x54, 0xbc, 0x6a, 0x06, 0x4d, 0xd6, 0xbc, 0xe6, 0x18, 0x0b, 0x42, 0x17, 0x43, 0x7c};
static const uint8_t PING_CHARACTERISTIC_UUID[16] = {0xa4, 0xb7, 0xe1, 0x18, 0x7b, 0x77, 0x4e, 0xf9, 0xa6, 0x18, 0x96, 0x7a, 0x28, 0x42, 0xe6, 0x30};

static btstack_packet_callback_registration_t hci_event_callback_registration;

#define MAX_CHARACTERISTICS 8
static gatt_client_characteristic_t characteristics[MAX_CHARACTERISTICS];
static int num_characteristics = 0;

void subscribe_to_characteristics();
int characteristic_index = 0;


// Checks if the advertisement contains the correct service (UNUSED)
static int advertisement_contains_service(uint8_t *packet, const uint8_t *service_uuid) {
    uint8_t ad_len = gap_event_advertising_report_get_data_length(packet);
    const uint8_t *ad_data = gap_event_advertising_report_get_data(packet);

    ad_context_t context;
    ad_iterator_init(&context, ad_len, ad_data);

    printf("Scanning advertisement data...\n");

    while (ad_iterator_has_more(&context)) {
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_size;
        const uint8_t *data = ad_iterator_get_data(&context);

        // Print all UUIDs found in the advertisement
        if (data_size == 16) {
            printf("Found UUID: ");
            for (int i = 0; i < 16; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");
        }

        // Check if the UUID matches the expected one
        if ((data_type == BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS || data_type == BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS) && data_size == 16 && memcmp(data, service_uuid, 16) == 0) {
            printf("UUID MATCH FOUND!\n");
            return 1;
        }

        ad_iterator_next(&context);
    }
    return 0;
}

// Handles post-write calls
void gatt_write_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type == HCI_EVENT_PACKET && packet[0] == GATT_EVENT_QUERY_COMPLETE) {
        uint8_t status = gatt_event_query_complete_get_att_status(packet);
        if (status == ATT_ERROR_SUCCESS) {
            printf("Write successful!\n");
        } else {
            printf("Write failed! Status: 0x%02X\n", status);
        }
    }
}

// Sends a write to the server pico
void send_test_write() {
    if (connection_handle) {
        printf("Attempting to send manual test write to handle PING_CHAR_VALUE_HANDLE...\n");
        
        uint8_t test_value = 0x42;
        uint8_t status = gatt_client_write_value_of_characteristic(gatt_write_callback, connection_handle, PING_CHAR_VALUE_HANDLE, sizeof(test_value), &test_value);
        
        printf("Write request sent with status: 0x%02X to handle: PING_CHAR_VALUE_HANDLE\n", status);
    } else {
        printf("No connection, cannot send write!\n");
    }
}

// Searches for and connects to server pico
void connect_to_known_device() {
    bd_addr_t known_server = {0x28, 0xCD, 0xC1, 0x10, 0xA5, 0x24};  // Known BLE MAC address
    bd_addr_type_t known_server_type = BD_ADDR_TYPE_LE_PUBLIC;  // Checks LE Public type

    printf("Attempting direct connection to %s...\n", bd_addr_to_str(known_server));

    int status = gap_connect(known_server, known_server_type); // Attempts connection
    if (status) {
        printf("gap_connect() failed! Error code: %d\n", status);
    } else {
        printf("gap_connect() successfully called!\n");
    }
}

// Handle GATT events (service discovery, subscribing, responding to pings)
void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);
    
    uint8_t event = hci_event_packet_get_type(packet);
    printf("Received GATT Event: 0x%02X\n", event);  // Debug print

    // Immediately handles notifications
    if (event == GATT_EVENT_NOTIFICATION) {
        uint16_t attribute_handle = gatt_event_notification_get_value_handle(packet);
        uint16_t length = gatt_event_notification_get_value_length(packet);
        const uint8_t *value = gatt_event_notification_get_value(packet);
        uint16_t int_value = 0;

        /* printf("Notification arrived! Handle: 0x%04X, Data: ", attribute_handle);
        for (int i = 0; i < length; i++) {
            printf("%02X ", value[i]);
        }
        printf("\n");
*/
        if (length == 2) {
            int_value = little_endian_read_16(value, 0);
        } else if (length == 1) {
            int_value = value[0];
        }

        switch (attribute_handle) {
            case 0x0009:
                line_length = int_value;
                //printf("Line Length: %u\n", int_value);
                break;
            case 0x000C:
                drag_set = int_value;
                //printf("Drag Set: %u\n", int_value);
                break;
            case 0x000F:
                motor_status = int_value;
                //printf("Motor Status: %u\n", int_value);
                break;
            case 0x0012:   
                motor_speed = int_value;
                //printf("Motor Speed: %u\n", int_value);
                break;
            case 0x0015:
                fish_alarm = int_value;
                //printf("Fish Alarm: %u\n", int_value);
                break;
            case 0x0018:
                auto_stop_length = int_value;
                //printf("Auto Stop Length: %u\n", int_value);
                break;
            case 0x001B:
                measurement_system = int_value;
                //printf("Measurement System: %u\n", int_value);
                break;
            default:
                //printf("Notification from Handle 0x%04X: %u\n", attribute_handle, int_value);
                break;
        }
        
        if (attribute_handle == PING_CHAR_VALUE_HANDLE) {
            printf("Responding to ping...\n");
            response_value++;
            gatt_client_write_value_of_characteristic(
                gatt_write_callback,
                connection_handle,
                attribute_handle,
                sizeof(response_value),
                &response_value
            );
        }
        return; 
    }

    // These are used for "states" of connection, not for notifications
    switch (state) {
        case CLIENT_DISCOVERING_SERVICE: // Discovering services on server
            if (hci_event_packet_get_type(packet) == GATT_EVENT_SERVICE_QUERY_RESULT) {
                gatt_event_service_query_result_get_service(packet, &server_service);
                printf("Service discovered: Start Handle: 0x%04X, End Handle: 0x%04X\n", server_service.start_group_handle, server_service.end_group_handle);
            } else if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
                if (server_service.start_group_handle == 0) {
                    printf("Error: Service UUID not found on server!\n");
                    return;
                }
                state = CLIENT_DISCOVERING_CHARACTERISTIC;
                printf("Service found, discovering characteristics...\n");
                gatt_client_discover_characteristics_for_service(
                    handle_gatt_client_event, connection_handle, &server_service);
                
            }
            break;
            
        case CLIENT_DISCOVERING_CHARACTERISTIC:
            if (hci_event_packet_get_type(packet) == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT) {
                // Temporary variable to hold the discovered characteristic
                gatt_client_characteristic_t new_characteristic;
                gatt_event_characteristic_query_result_get_characteristic(packet, &new_characteristic);
        
                // Store it in an array (or linked list if more dynamic handling is needed)
                if (num_characteristics < MAX_CHARACTERISTICS) {
                    characteristics[num_characteristics++] = new_characteristic;
                    printf("Discovered characteristic handle: 0x%04X\n", new_characteristic.value_handle);
                } else {
                    printf("Warning: Too many characteristics discovered! Increase MAX_CHARACTERISTICS.\n");
                }
            } 
            else if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
                if (num_characteristics == 0) {
                    printf("Error: No characteristics found for service!\n");
                    return;
                }
        
                state = CLIENT_SUBSCRIBING;
                printf("Discovered %d characteristics. Proceeding to subscribe...\n", num_characteristics);

                gatt_client_listen_for_characteristic_value_updates(
                    &notification_listener,  
                    handle_gatt_client_event,  
                    connection_handle,  
                    NULL 
                );

                subscribe_to_characteristics();
            }
            break;
        
        
        case GATT_EVENT_NOTIFICATION: { // Handle notifications from server
            uint16_t attribute_handle = gatt_event_notification_get_value_handle(packet);
            uint16_t length = gatt_event_notification_get_value_length(packet);
            const uint8_t *value = gatt_event_notification_get_value(packet);
        
            printf("Received Notification from Server! Handle: 0x%04X, Data: ", attribute_handle);
            for (int i = 0; i < length; i++) {
                printf("%02X ", value[i]);  // Print received data
            }
            printf("\n");
        
            // Check if handle is correct on notification
            if (attribute_handle == PING_CHAR_VALUE_HANDLE) {
                printf("Responding to server notification...\n");
        
                
                // Send a response back to the server
                uint8_t response_value = 1;  
                gatt_client_write_value_of_characteristic(
                    gatt_write_callback, connection_handle, attribute_handle, 
                    sizeof(response_value), &response_value);
            }
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_event_characteristic_query_result_get_characteristic(packet, &ping_characteristic);
            printf("Ping Characteristic properties: 0x%02X\n", ping_characteristic.properties);
            break;
        }
        
        case GATT_EVENT_QUERY_COMPLETE: { // Prints detected characteristics
            printf("Subscribed to characteristic %i\n", characteristic_index);
        
            characteristic_index++;
        
            break;
        }        

        case CLIENT_SUBSCRIBING:

            if (characteristic_index < num_characteristics) {
                subscribe_to_characteristics();
            } else {
                printf("All characteristics subscribed!\n");
            }
            break;

    }
}

void write_single_byte_characteristic(uint16_t handle, uint8_t value) {
    if (!connection_handle) {
        printf("No connection — cannot write!\n");
        return;
    }

    printf("Writing 0x%02X to handle 0x%04X\n", value, handle);
    gatt_client_write_value_of_characteristic(
        gatt_write_callback,
        connection_handle,
        handle,
        sizeof(value),
        &value
    );
}

void write_motor_speed(uint8_t speed) {
    write_single_byte_characteristic(MOTOR_SPEED_VALUE_HANDLE, speed);
}

void write_fish_alarm(uint8_t enabled) {
    write_single_byte_characteristic(FISH_ALARM_VALUE_HANDLE, enabled);
}

void write_auto_stop_length(uint8_t length) {
    write_single_byte_characteristic(AUTO_STOP_LENGTH_VALUE_HANDLE, length);
}

void write_measurement_system(uint8_t metric) {
    write_single_byte_characteristic(MEASUREMENT_SYSTEM_VALUE_HANDLE, metric);
}


void subscribe_to_characteristics() {
    printf("Entering CLIENT_SUBSCRIBING manually...\n");
        printf("Attempting subscription to characteristic handle: 0x%04X\n", characteristics[characteristic_index].value_handle);

        uint8_t status = gatt_client_write_client_characteristic_configuration(
            handle_gatt_client_event,
            connection_handle,
            &characteristics[characteristic_index],
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
        );

        printf("Subscription status for 0x%04X: 0x%02X\n", characteristics[characteristic_index].value_handle, status);
        characteristic_index++;
}

// Handle Bluetooth events (connection, scanning, indications)
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return; // Only handle HCI events

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case BTSTACK_EVENT_STATE: // Pre-connection state
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("Bluetooth ready. Connecting to known device...\n");
                connect_to_known_device();  // Direct connection instead of scanning
            } else {
                printf("Bluetooth not ready yet...\n");
            }
            break;

        case GAP_EVENT_ADVERTISING_REPORT: {
            bd_addr_t found_addr;
            gap_event_advertising_report_get_address(packet, found_addr);
            bd_addr_type_t found_addr_type = gap_event_advertising_report_get_address_type(packet);
        
            printf("Found device: %s (Type: %d)\n", bd_addr_to_str(found_addr), found_addr_type);
        
            // Get advertisement data length
            uint8_t ad_len = gap_event_advertising_report_get_data_length(packet);
            printf("Advertisement data length: %d\n", ad_len);
        
            if (ad_len > 0) {
                const uint8_t *ad_data = gap_event_advertising_report_get_data(packet);
        
                // Print raw advertisement data
                printf("Raw advertisement data: ");
                for (int i = 0; i < ad_len; i++) {
                    printf("%02X ", ad_data[i]);
                }
                printf("\n");
            } else {
                printf("No advertisement data found!\n");
            }
        
            break;
        } 
            
        case HCI_EVENT_LE_META: // Handles HCI events during connection
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                if (status == 0) {
                    printf("Connection successful!\n");

                    connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    state = CLIENT_DISCOVERING_SERVICE;

                    printf("Connection handle: 0x%04X\n", connection_handle);

                    gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, connection_handle, REEL_SERVICE_UUID);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                    printf("Service start handle: 0x%04X\n", server_service.start_group_handle);

                    //gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, connection_handle, READ_ONLY_SERVICE_UUID);
                    //printf("Service start handle: 0x%04X\n", server_service.start_group_handle);

                    //gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, connection_handle, READ_WRITE_SERVICE_UUID);
                    

                    // Request to send data
                    att_server_request_can_send_now_event(connection_handle);
                } else {
                    printf("Connection failed! Status: %d\n", status);
                }
            }
            break;

        case ATT_EVENT_CAN_SEND_NOW: // Runs if server is ready to receive write
            printf("Sending test write...\n");

            uint8_t test_value = 0x42;
            gatt_client_write_value_of_characteristic(gatt_write_callback, connection_handle, PING_CHAR_VALUE_HANDLE, sizeof(test_value), &test_value);

            break;
    }
}



void att_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {
        case GATT_EVENT_NOTIFICATION: {
            uint16_t handle = gatt_event_notification_get_value_handle(packet);
            uint16_t length = gatt_event_notification_get_value_length(packet);
            const uint8_t *value = gatt_event_notification_get_value(packet);
        
            printf("🔹 Notification Received! Handle: 0x%04X, Data: ", handle);
            for (int i = 0; i < length; i++) {
                printf("%02X ", value[i]);  
            }
            printf("\n");
        
            break;
        }
    }
}

void init_bluetooth() {
    printf("Initializing Bluetooth...\n");
    if (cyw43_arch_init()) {
        printf("Failed to initialize Bluetooth.\n");
        return;
    }
    l2cap_init();
    sm_init();
    att_server_init(NULL, NULL, NULL);
    att_server_register_packet_handler(handle_gatt_client_event);
    gatt_client_init();
    
    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
}


/*
int main() {
    stdio_init_all();
    sleep_ms(5000);

    if (cyw43_arch_init()) {
        printf("Failed to initialize Bluetooth.\n");
        return -1;
    }

    // BLE Initialization
    l2cap_init();
    sm_init();
    att_server_init(NULL, NULL, NULL);
    att_server_register_packet_handler(handle_gatt_client_event);
    gatt_client_init();

    // Register Event Handlers
    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}
*/