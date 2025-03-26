#ifndef CLIENT_H
#define CLIENT_H

#include <stdint.h>
#include "btstack.h"

// Bluetooth Connection State Enum
typedef enum {
    CLIENT_IDLE,
    CLIENT_SCANNING,
    CLIENT_CONNECTING,
    CLIENT_DISCOVERING_SERVICE,
    CLIENT_DISCOVERING_CHARACTERISTIC,
    CLIENT_SUBSCRIBING,
    CLIENT_READY
} client_state_t;

extern client_state_t state;

// Function prototypes for client.c
void init_bluetooth();
void connect_to_known_device();
void subscribe_to_characteristics();
void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void send_test_write();

// Global variables to hold received values
extern volatile uint16_t line_length;
extern volatile uint16_t drag_set;
extern volatile uint8_t motor_status;
extern volatile uint8_t motor_speed;
extern volatile uint8_t fish_alarm;
extern volatile uint8_t auto_stop_length;
extern volatile uint8_t measurement_system;

// Write functions for each writeable characteristic
void write_motor_speed(uint8_t speed);
void write_fish_alarm(uint8_t enabled);
void write_auto_stop_length(uint8_t length);
void write_measurement_system(uint8_t metric);


#endif // CLIENT_H
