#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Enable BLE features
// #define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL

// Fix deprecated setting
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE

// Set required payload size for L2CAP
#define HCI_ACL_PAYLOAD_SIZE 52

// Fix Attribute Database memory allocation
#define MAX_ATT_DB_SIZE 512

// Fix required BTstack memory settings
#define HCI_OUTGOING_PRE_BUFFER_SIZE 32
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// Fix LE Device Database Settings
#define MAX_NR_LE_DEVICE_DB_ENTRIES 4
#define NVM_NUM_DEVICE_DB_ENTRIES 4

// Enable debugging
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEX
#define ENABLE_PRINTF_HEXDUMP

#endif // BTSTACK_CONFIG_H
