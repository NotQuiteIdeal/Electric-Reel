#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hci.h"
#include "gap.h"
#include "C:\Users\harri\.pico-sdk\btstack\src\ble\gatt_client.h"


// Bluetooth connection state
typedef enum {
    CLIENT_IDLE,
    CLIENT_SCANNING,
    CLIENT_CONNECTING,
    CLIENT_DISCOVERING_SERVICE,
    CLIENT_DISCOVERING_CHARACTERISTIC,
    CLIENT_SUBSCRIBING,
    CLIENT_READY
} client_state_t;

// Global Variables
static client_state_t state = CLIENT_IDLE;
static hci_con_handle_t connection_handle;
static gatt_client_service_t server_service;
static gatt_client_characteristic_t ping_characteristic;
static gatt_client_notification_t notification_listener;
static bd_addr_t server_addr;
static bd_addr_type_t server_addr_type;

// UUIDs (match server)
static const uint8_t PING_SERVICE_UUID[16] = {0xc0, 0xff, 0x29, 0xe1, 0x41, 0xa6, 0x40, 0xd9, 0x87, 0x35, 0x9d, 0x22, 0xae, 0x02, 0xd1, 0x29};
static const uint8_t PING_CHARACTERISTIC_UUID[16] = {0xa4, 0xb7, 0xe1, 0x18, 0x7b, 0x77, 0x4e, 0xf9, 0xa6, 0x18, 0x96, 0x7a, 0x28, 0x42, 0xe6, 0x30};

static btstack_packet_callback_registration_t hci_event_callback_registration;

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
        if ((data_type == BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS ||
             data_type == BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS) &&
            data_size == 16 && memcmp(data, service_uuid, 16) == 0) {
            printf("UUID MATCH FOUND!\n");
            return 1;
        }

        ad_iterator_next(&context);
    }
    return 0;
}

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

void send_test_write() {
    if (connection_handle) {
        printf("Attempting to send manual test write to handle 0x0017...\n");
        
        uint8_t test_value = 0x42;
        uint8_t status = gatt_client_write_value_of_characteristic(
            gatt_write_callback, connection_handle, 0x0017, sizeof(test_value), &test_value);
        
        printf("Write request sent with status: 0x%02X\n", status);
    } else {
        printf("No connection, cannot send write!\n");
    }
}

static void connect_to_known_device() {
    bd_addr_t known_server = {0x28, 0xCD, 0xC1, 0x10, 0xA5, 0x24};  // Known BLE MAC address
    bd_addr_type_t known_server_type = BD_ADDR_TYPE_LE_PUBLIC;  // Try LE Public first

    printf("Attempting direct connection to %s...\n", bd_addr_to_str(known_server));

    int status = gap_connect(known_server, known_server_type);
    if (status) {
        printf("gap_connect() failed! Error code: %d\n", status);
    } else {
        printf("gap_connect() successfully called!\n");
    }
}


// Start scanning for server UNREFERENCED CURRENTLY trying to connect via MAC Address currently
static void client_start_scan(void) {
    /*
    // Set scanning parameters
    uint8_t scan_type = 1;  // Active scanning
    uint16_t scan_interval = 0x0060;  // 60ms
    uint16_t scan_window = 0x0060;  // 60ms
    uint8_t scanning_filter_policy = 0x00;  // Accept only known devices

    // Set the known server as the only allowed device
    gap_set_scan_params(scan_type, scan_interval, scan_window, scanning_filter_policy);
    gap_start_scan();

    printf("Scanning started...\n"); */
    printf("Not scanning.");
}

// Handle Bluetooth events (connection, scanning, indications)
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case BTSTACK_EVENT_STATE:
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
            
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                if (status == 0) {
                    printf("Connection successful!\n");

                    connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    state = CLIENT_DISCOVERING_SERVICE;

                    printf("Connection handle: 0x%04X\n", connection_handle);

                    // Request to send data
                    att_server_request_can_send_now_event(connection_handle);

                    send_test_write();
                } else {
                    printf("Connection failed! Status: %d\n", status);
                }
            }
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            printf("Sending test write...\n");

            uint8_t test_value = 0x42;
            gatt_client_write_value_of_characteristic(gatt_write_callback, connection_handle, 0x0017, sizeof(test_value), &test_value);



            break;


    }
}

// Handle GATT events (service discovery, subscribing, responding to pings)
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);
    
    switch (state) {
        case CLIENT_DISCOVERING_SERVICE:
            if (hci_event_packet_get_type(packet) == GATT_EVENT_SERVICE_QUERY_RESULT) {
                gatt_event_service_query_result_get_service(packet, &server_service);
                printf("Service discovered: Start Handle: 0x%04X, End Handle: 0x%04X\n", 
                    server_service.start_group_handle, server_service.end_group_handle);
            } else if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
                if (server_service.start_group_handle == 0) {
                    printf("Error: Service UUID not found on server!\n");
                    return;
                }
                state = CLIENT_DISCOVERING_CHARACTERISTIC;
                printf("Service found, discovering characteristics...\n");
                gatt_client_discover_characteristics_for_service_by_uuid128(handle_gatt_client_event, connection_handle, &server_service, PING_CHARACTERISTIC_UUID);
            }
            break;
            
        case CLIENT_DISCOVERING_CHARACTERISTIC:
            if (hci_event_packet_get_type(packet) == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT) {
                gatt_event_characteristic_query_result_get_characteristic(packet, &ping_characteristic);
            } else if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
                state = CLIENT_SUBSCRIBING;
                printf("Subscribing to Ping Test characteristic...\n");
        
                // Register for notifications
                gatt_client_listen_for_characteristic_value_updates(
                    &notification_listener,  // Persistent listener struct
                    handle_gatt_client_event,  // Event handler
                    connection_handle,  // BLE connection handle
                    &ping_characteristic  // The characteristic to listen for
                );
        
                // Enable notifications on the server
                uint8_t status = gatt_client_write_client_characteristic_configuration(
                    handle_gatt_client_event, 
                    connection_handle, 
                    &ping_characteristic, 
                    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
                );
        
                if (status == ERROR_CODE_SUCCESS) {
                    printf("Notifications enabled on server.\n");
                } else {
                    printf("Failed to enable notifications. Error: 0x%02X\n", status);
                }
            }
            break;

        case CLIENT_SUBSCRIBING:
            if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
                state = CLIENT_READY;
                printf("Subscribed to Ping Test!\n");
            }
            break;

        case CLIENT_READY:
            if (hci_event_packet_get_type(packet) == GATT_EVENT_INDICATION) {
                printf("Ping received! Sending response...\n");
                uint8_t response_value = 1;
                gatt_client_write_value_of_characteristic(handle_gatt_client_event, connection_handle, ping_characteristic.value_handle, sizeof(response_value), &response_value);
                printf("Ping response sent.\n");
            }
            break;
    }
}

void att_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {
        case GATT_EVENT_NOTIFICATION: {
            uint16_t attribute_handle = gatt_event_notification_get_value_handle(packet);
            uint16_t length = gatt_event_notification_get_value_length(packet);
            const uint8_t *value = gatt_event_notification_get_value(packet);

            printf("Received Notification from Server! Handle: 0x%04X, Data: ", attribute_handle);
            for (int i = 0; i < length; i++) {
                printf("%02X ", value[i]);  // Print the received data as hex
            }
            printf("\n");

            // **Now Send a Response Back to the Server**
            uint8_t response_value = 1;  // Placeholder response
            gatt_client_write_value_of_characteristic(
                gatt_write_callback, 
                connection_handle, 
                attribute_handle, 
                sizeof(response_value), 
                &response_value
            );
            printf("Sent Response to Server!\n");
            break;
        }
    }
}




int main() {
    stdio_init_all();
    sleep_ms(5000);

    if (cyw43_arch_init()) {
        printf("Failed to initialize Bluetooth.\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    att_server_init(NULL, NULL, NULL);
    gatt_client_init();

    // Register Event Handlers
    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}
