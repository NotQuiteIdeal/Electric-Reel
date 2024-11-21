#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define I2C_PORT i2c1
#define SDA_PIN 4
#define SCL_PIN 5
#define DEVICE_ADDRESS 0x42 // CFA634 I2C address in 7-bit format

void init_i2c() {
    i2c_init(I2C_PORT, 100 * 1000); // 400 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Clears the display and resets the cursor position
void clear_display() {
    uint8_t command[] = {0x0C}; // Form Feed (Clear Display)
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 1, false);
}

// Moves the cursor to the home position (top-left corner)
void return_home() {
    uint8_t command[] = {0x01}; // Cursor Home
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 1, false);
}

// Sets the cursor position to the specified column and row
void set_cursor_position(uint8_t column, uint8_t row) {
    uint8_t command[] = {0x11, column, row}; // Set Cursor Position
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 3, false);
}

// Displays a string of text at the current cursor position
void write_text_to_display(const char *text) {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, (const uint8_t *)text, strlen(text), false);
}

// Turns the display on
void show_display() {
    uint8_t command[] = {0x03}; // Show Display
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 1, false);
}

// Hides the display (keeps content intact)
void hide_display() {
    uint8_t command[] = {0x02}; // Hide Display
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 1, false);
}

// Enables the blinking block cursor
void show_blinking_cursor() {
    uint8_t command[] = {0x06}; // Show Blinking Block Cursor
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 1, false);
}

// Sets the backlight brightness (0-100%)
void set_backlight_brightness(uint8_t brightness) {
    uint8_t command[] = {0x0E, brightness}; // Backlight Control
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, command, 2, false);
}

int main() {
    stdio_init_all();
    init_i2c();

    // Example usage:
    clear_display();                      // Clear the screen
    set_backlight_brightness(50);         // Set backlight to 50%
    return_home();                        // Move cursor to top-left
    write_text_to_display("Hello, World!"); // Display text
    set_cursor_position(5, 1);            // Move cursor to column 5, row 1
    write_text_to_display("Line 2");      // Display text on second line

    return 0;
}
