/*
 * ----------------------------------------------------------------------
 * DDReels Mobile Device - Encoder-Based Settings Navigation with UI (Updated)
 * Project: Dynamic Drag Reels (D.D. Reels)
 * Author: Gabriel Joda / Modified by ChatGPT
 * Platform: Raspberry Pi Pico W
 * 
 * Purpose:
 *   Implements rotary encoder-driven settings navigation and adjustments with
 *   enhanced icon management and display features.
 * 
 * Key Features and Updates:
 *   - Rotary encoder controls menu selection with real-time updates and
 *     robust debounce logic.
 *   - Long press detection to seamlessly enter and exit settings.
 *   - Submenus for fine-tuning key parameters:
 *         • Reel-Up Speed (displayed as a percentage)
 *         • Auto Stop Length (ft in Imperial; m in Metric)
 *         • Metric/Imperial toggle (unit conversion on display)
 *         • Alarm On/Off toggle (disables buzzer when off)
 *         • Contrast (0–100%, controlled via GPIO 21 PWM)
 *         • Brightness (0–100%, controlled via GPIO 22 PWM; default updated to 100%)
 *         • Save settings
 *   - Main screen displays remaining reel length (200ft – auto stop length)
 *     and an independent drag value (converted to N·m in metric mode).
 *   - LCD updates occur only when necessary to optimize efficiency.
 *   - LED and button feedback system for improved user interaction.
 *   - Expanded framework to support future feature integration.
 * 
 * Enhancements:
 *   - Added a custom Bluetooth icon created via CGRAM (function create_bluetooth_char)
 *     using the following 5x8 pixel pattern:
 *         Row 0: 00100 (0x04)
 *         Row 1: 10110 (0x16)
 *         Row 2: 01101 (0x0D)
 *         Row 3: 00110 (0x06)
 *         Row 4: 00110 (0x06)
 *         Row 5: 01101 (0x0D)
 *         Row 6: 10110 (0x16)
 *         Row 7: 00100 (0x04)
 *     This icon is displayed on the leftmost side of line 4 of the LCD.
 * 
 *   - Implemented a generic update_icon() function using an IconMode enumeration
 *     (ICON_CLEAR, ICON_SOLID, ICON_BLINK) to easily control icon states.
 * 
 *   - Introduced update_battery_icon() to display a full battery icon (using SEGRAM
 *     address 0x0F with data 0x1F), laying the groundwork for future dynamic battery
 *     monitoring.
 * 
 *   - Refined PWM mapping for contrast and brightness, with brightness now defaulting
 *     to 100%.
 * 
 * Status:
 *   Navigation and settings adjustment work as before. The UI now features dynamic
 *   icon handling (for Bluetooth, battery, and alarm states) and improved visual feedback,
 *   with modular routines in place to support further enhancements.
 * ----------------------------------------------------------------------
 */


 #include <stdio.h>
 #include <string.h>
 #include "pico/stdlib.h"
 #include "hardware/gpio.h"
 #include "hardware/pwm.h"
 #include "hardware/adc.h"
 #include "client.h"
 #include "pico/multicore.h"
 //#include "pico/flash.h"
 //#include "hardware/flash.h"



 // Function to update an icon on the LCD.
 typedef enum {
     ICON_CLEAR,  // Clear the icon (off)
     ICON_SOLID,  // Display the solid version of the icon
     ICON_BLINK   // Display the blinking version of the icon
 } IconMode;


 /*
  // Struct for saving settings
  typedef struct __attribute__((aligned(4))) {
    uint32_t magic;
    int brightness;
    int contrast;
    int reel_speed;
    bool alarm_enabled;
    uint32_t crc32;
 } settings_t;

 #define SETTINGS_MAGIC 0xDEADBEEF */

 // =========================
 // New PWM Settings for Contrast & Brightness
 // =========================
 // We use GPIO 21 for contrast and GPIO 22 for brightness.
 // The contrast setting is mapped such that the PWM (before the resistor) produces a voltage 
 // that, after a 0.6 V drop, will be in the range of ~2.0 V to 2.4 V.
 // For brightness, the full 0–100% PWM range is used.
 #define CONTRAST_PWM_PIN    20
 #define BRIGHTNESS_PWM_PIN  8
 
 // New global variables for contrast and brightness (0–100%).
 int contrast_value = 50;    // Default 50% --> After mapping, output voltage ~? (see math below)
 int brightness_value = 10;  // Default 100%
 
 // =========================
 // Other Project Constants & Variables
 // =========================
 
 // Define battery life indicator pin
 #define BATTERY_ADC_PIN 26  //GPIO26 (ADC0)
 // Voltage divider ratio (for 100kΩ & 47kΩ)
 #define VOLTAGE_DIVIDER_RATIO 3.13  // Approximate ratio

 // GPIO Pins for charger status
 #define CHARGER_STAT1 13  // Status 1 (STAT1)
 #define CHARGER_STAT2 12  // Status 2 (STAT2)

// Battery Voltage to Percentage Mapping (LiPo Approximation)
int battery_percentage(float voltage) {
    if (voltage >= 3.2) return 100;  // Fully charged
    if (voltage >= 2.85) return 75 + ((voltage - 2.85) / 0.35) * 25;
    if (voltage >= 2.7) return 50 + ((voltage - 2.7) / 0.15) * 25;
    if (voltage >= 2.5) return 25 + ((voltage - 2.5) / 0.2) * 25;
    if (voltage >= 2.2) return ((voltage - 2.2) / 0.3) * 25;
    return 0;  // Battery low
}

// Read battery voltage through ADC
float get_battery_voltage() {
    adc_select_input(0); // Read from ADC0
    uint16_t raw_value = adc_read();  // 12-bit (0–4095)
    float v_measured = (raw_value / 4095.0) * 3.3; // Convert to voltage
    return v_measured * VOLTAGE_DIVIDER_RATIO; // Scale up
}

 // Define the full reel length (in feet) used to compute remaining length. REMOVED FOR BLUETOOTH INTEGRATION
// #define FULL_REEL_LENGTH 200
 
 // === LCD Pin Definitions === //
 #define PIN_RS  22    // Register Select: low for command, high for data.
 #define PIN_RW  27    // Read/Write: low for write.
 #define PIN_E   28    // Enable: triggers data read/write on falling edge.
 #define PIN_RES 21    // Reset pin for the LCD module.
 
 // 8-bit Data Bus for LCD: These GPIO pins connect to D0-D7 of the LCD.
 const int data_pins[8] = {0, 1, 2, 3, 4, 5, 6, 7};
 
 // Rotary Encoder Pins (using a Bourns PEC11 encoder).
 /*
 #define ENCODER_A   10  // Encoder channel A.
 #define ENCODER_B   11  // Encoder channel B.
 #define ENCODER_BTN 12  // Encoder push-button (active low).
 */
 #define ENCODER_A   10  // Encoder channel A.
 #define ENCODER_B   11  // Encoder channel B.
 #define ENCODER_BTN 9  // Encoder push-button (active low).
 // Additional Button and LED Pins.
 //#define BUTTON_26 2  // Fish biting simulator.
 //#define BUTTON_14 14  // Mute button.
 #define BUTTON_14 16
 //#define BUTTON_15 15  // Reel-Up button.
 #define BUTTON_15 17
 //#define LED_28    28  // Alarm LED.
 #define LED_28    18
 //#define LED_13    13  // Alarm buzzer.
 #define LED_13    19
 //#define LED_27    27  // Reel-Up LED indicator.
 
 // Global variables used for navigation and settings.
 volatile int menu_index = 0;            // Index for the currently selected menu item.
 volatile bool in_settings_menu = false;  // True when the settings menu is active.
 volatile uint32_t button_press_time = 0; // Timestamp for when the encoder button was pressed.
 volatile bool button_pressed = false;    // Flag indicating if the encoder button is pressed.
 
 // These flags are used for debouncing the rotary encoder.
 volatile bool cw_fall = false;           // Set when a falling edge on channel A is detected.
 volatile bool ccw_fall = false;          // Set when a falling edge on channel B is detected.
 
 int last_menu_index = -1;                // Used to prevent redundant LCD updates.
 volatile bool ignore_next_press = false; // When entering settings, ignore the very next encoder button press.
 volatile bool in_submenu = false;        // True when in a subpage (editing a specific setting).
 volatile uint32_t subpage_ignore_until = 0; // Time until which encoder button presses in subpage mode are ignored.
 
 // User-adjustable settings.
 int reel_speed = 50;       // Reel-Up speed (as a percentage, 0–100%) in 10% steps.
 int stop_length = 0;      // Auto stop length in feet (0–100 FT) in 1-ft steps.
 int temp_stop_length = 0; // Temporary variable for stop length during editing.
 bool metric_mode = false;  // false for Imperial units, true for Metric.
 bool alarm_enabled = true; // If false, the buzzer is disabled.
 float drag_value = 5.0f;   // Independent drag value (in ft–lbs); later can be updated separately.
 
 // Flag from the ISR to indicate that the subpage display needs updating.
 volatile bool update_subpage_flag = false;

 // Flag for Icons
 volatile bool alarm_icon_update_needed = false;
 volatile IconMode desired_alarm_icon_mode = ICON_CLEAR;

 // Bluetooth Variables and Shenanigans
 volatile int BT_Init = 0;
 volatile int FULL_REEL_LENGTH;
 extern volatile uint16_t line_length;
 extern volatile uint16_t drag_set;
 extern volatile uint8_t motor_status;
 extern volatile uint8_t motor_speed;
 extern volatile uint8_t fish_alarm;
 extern volatile uint8_t auto_stop_length;
 extern volatile uint8_t measurement_system;
 extern volatile int connection_status;
 volatile int fart = 0;
 volatile int fart2 = 0;
 //settings_t current_settings;

 /*
 #define SETTINGS_FLASH_OFFSET ((uint32_t)(&flash_settings) - XIP_BASE)
 #define SETTINGS_SECTOR_SIZE  FLASH_SECTOR_SIZE
 #define SETTINGS_PAGE_SIZE    FLASH_PAGE_SIZE

 const settings_t *saved_settings_ptr = (const settings_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);

 static void __not_in_flash_func(call_flash_range_erase)(void *param) {
    flash_range_erase((uint32_t)param, SETTINGS_SECTOR_SIZE);
 }

 static void __not_in_flash_func(call_flash_range_program)(void *param) {
    uint32_t offset = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
    flash_range_program(offset, data, FLASH_PAGE_SIZE);
 }

 bool save_settings(const settings_t *settings_in) {
     settings_t temp = *settings_in;
     temp.magic = SETTINGS_MAGIC;

    // Allocate a static buffer so it's in flash-eligible memory space
     static uint8_t page_buffer[FLASH_PAGE_SIZE];
     memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);  // Fill with 0xFF for a clean write
     memcpy(page_buffer, &temp, sizeof(settings_t));

    // Erase sector
     int rc = flash_safe_execute(call_flash_range_erase, (void *)SETTINGS_FLASH_OFFSET, UINT32_MAX);
     if (rc != PICO_OK) return false;

     // Program page
     uintptr_t params[] = { SETTINGS_FLASH_OFFSET, (uintptr_t)page_buffer };
     rc = flash_safe_execute(call_flash_range_program, params, UINT32_MAX);
     return rc == PICO_OK;
 }

 bool load_settings(settings_t *settings_out) {
     const settings_t *flash_data = (const settings_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);

     if (flash_data->magic != SETTINGS_MAGIC) {
         return false;  // No valid settings saved yet
     }
 
     memcpy(settings_out, flash_data, sizeof(settings_t));
     return true;
 }
     */

 // =========================
 // LCD Helper Functions
 // =========================
 
 // Writes an 8-bit value to the LCD data bus by updating each GPIO pin.
 void lcd_write_bus(uint8_t data) {
     for (int i = 0; i < 8; i++) {
         gpio_put(data_pins[i], (data >> i) & 1);
     }
 }
 
 // Generates an enable pulse for the LCD (as required by the controller).
 void lcd_pulse_enable(void) {
     gpio_put(PIN_E, 1);
     sleep_us(200);
     gpio_put(PIN_E, 0);
     sleep_us(200);
 }
 
 // Sends a command byte to the LCD.
 void lcd_send_command(uint8_t cmd) {
     gpio_put(PIN_RS, 0);  // RS low for command.
     gpio_put(PIN_RW, 0);  // RW low for writing.
     lcd_write_bus(cmd);
     lcd_pulse_enable();
     sleep_ms(2);          // Delay for command processing.
 }
 
 // Sends a data byte to the LCD.
 void lcd_send_data(uint8_t data) {
     gpio_put(PIN_RS, 1);  // RS high for data.
     gpio_put(PIN_RW, 0);  // RW low for writing.
     lcd_write_bus(data);
     lcd_pulse_enable();
     sleep_ms(1);
 }
 
 // Initializes the LCD using the standard initialization sequence.
 void lcd_init(void) {
     gpio_put(PIN_RES, 0);
     sleep_ms(50);
     gpio_put(PIN_RES, 1);
     sleep_ms(200);
     lcd_send_command(0x30); // Function set: 8-bit interface.
     sleep_ms(10);
     lcd_send_command(0x06); // Entry mode: auto-increment.
     sleep_ms(5);
     lcd_send_command(0x36); // Extended mode.
     sleep_ms(10);
     lcd_send_command(0x09); // Icon display on.
     sleep_ms(5);
     lcd_send_command(0x40); // Set SEGRAM address to 0x00 (Icon RAM)
     for (int i = 0; i < 16; i++) { // Initialize icons
     lcd_send_data(0x00); // Write 0x00 to clear each icon slot
     }
     lcd_send_command(0x30); // Back to basic instruction set.
     sleep_ms(10);
     lcd_send_command(0x0C); // Display on, cursor off.
     sleep_ms(5);
     lcd_send_command(0x01); // Clear display.
     sleep_ms(10);
     printf("[LCD] Initialization Complete!\n");
 }
 
 // Prints a string at the current cursor position.
 void lcd_print(const char *str) {
     while (*str) {
         lcd_send_data(*str++);
     }
 }
 
 // Prints a string starting at the beginning of a specified LCD line.
 void lcd_print_normal(const char *str, uint8_t line) {
     uint8_t addr = (line - 1) * 0x20;
     lcd_send_command(0x80 | addr);
     lcd_print(str);
 }
 
 // Prints a string centered on a given LCD line.
 void lcd_print_centered(const char *str, uint8_t line) {
     uint8_t pos = (20 - strlen(str)) / 2;
     uint8_t addr = (line - 1) * 0x20 + pos;
     lcd_send_command(0x80 | addr);
     lcd_print(str);
 }
 
 // =========================
 // Display Functions
 // =========================

 // Main screen display function.
 // In metric mode, it converts auto stop length from feet to meters and drag from ft-lbs to N·m.
 // In imperial mode, the remaining reel length is computed as FULL_REEL_LENGTH - stop_length.
 void display_main_screen(void) {
     lcd_send_command(0x01); // Clear display.
     if (metric_mode) {
         float meters = line_length * 0.3048f;   // Convert feet to meters.
         float drag_metric = drag_value * 0.453592f;   // Convert ft-lbs to N·m (approximate factor).
         char line1[21], line2[21], line3[21];
         snprintf(line1, sizeof(line1), "MAIN SCREEN");
         snprintf(line2, sizeof(line2), "%.1f METERS", meters);
         snprintf(line3, sizeof(line3), "%.1f KG", drag_metric);
         lcd_print_centered(line1, 1);
         lcd_print_centered(line2, 2);
         lcd_print_centered(line3, 3);
         lcd_send_command(0x80 | 0x60);  // Set cursor to the leftmost position of line 4
         if (connection_status == 1) {
            lcd_send_data(0);               // Print the custom character stored in slot 0
         }
     } else {
         int remaining = FULL_REEL_LENGTH; // Remaining reel length. (Removed - stop_length)
         printf("Remaining: %d, FULL_REEL_LENGTH: %d, stop_length: %d\n", remaining, FULL_REEL_LENGTH, stop_length);
         char line1[21], line2[21], line3[21];
         snprintf(line1, sizeof(line1), "MAIN SCREEN");
         snprintf(line2, sizeof(line2), "%03d FEET", remaining);
         snprintf(line3, sizeof(line3), "%.0f LBS", drag_value);
         lcd_print_centered(line1, 1);
         lcd_print_centered(line2, 2);
         lcd_print_centered(line3, 3);
         lcd_send_command(0x80 | 0x60);  // Set cursor to the leftmost position of line 4
         if (connection_status == 1) {
            lcd_send_data(0);               // Print the custom character stored in slot 0
         }
     }
     printf("[LCD] Main Screen Displayed.\n");
 }

 // Displays the settings menu.
 // The menu now contains seven options:
 // 1. Reel-Up Speed
 // 2. Auto Stop Length
 // 3. Metric/Imperial toggle
 // 4. Turn off Alarm
 // 5. Contrast
 // 6. Brightness
 // 7. Save Settings
 void display_settings_menu(int index) {
     const char *menu_items[] = {
         " 1.Reel-Up Speed",
         " 2.Auto Stop Length",
         " 3.Metric/Imperial",
         " 4.Turn off Alarm",
         " 5.Contrast",
         " 6.Brightness",
         " 7.Save Settings"
     };
     int start_index = index >= 4 ? index - 3 : 0;
     lcd_send_command(0x01); // Clear display.
     for (int i = 0; i < 4; i++) {
         lcd_send_command(0x80 | (i * 0x20));
         if (start_index + i < 7) {
             lcd_print(menu_items[start_index + i]);
             if (start_index + i == index) {
                 lcd_send_command(0x80 | ((i * 0x20)));
                 lcd_print(">");
             }
         }
     }
     printf("[LCD] Settings Menu Item: %s\n", menu_items[index]);
 }

void update_icon(uint8_t icon_index, IconMode mode) {
    uint8_t data;
    // Choose the data byte based on the mode.
    // According to the datasheet:
    // - Solid icon data is typically 0x10.
    // - Blinking icon data is typically 0x50.
    // - Clearing uses 0x00.
    switch(mode) {
        case ICON_SOLID:
            data = 0x10;
            break;
        case ICON_BLINK:
            data = 0x50;
            break;
        case ICON_CLEAR:
        default:
            data = 0x00;
            break;
    }
    // Use extended mode (0x36) to access the icon RAM,
    // then set the SEGRAM address (base 0x40 ORed with icon index),
    // write the data byte, and finally return to basic mode (0x30).
    lcd_send_command(0x36);  
    lcd_send_command(0x40 | (icon_index & 0x0F));
    lcd_send_data(data);
    lcd_send_command(0x30);
}
 void update_battery_icon() {
     float battery_voltage = get_battery_voltage();
     int battery_level = battery_percentage(battery_voltage);

     uint8_t icon_data;

     if (battery_level >= 80) {  // Full Battery
         icon_data = 0x1F;  // Solid icon
     } else if (battery_level >= 60) {
         icon_data = 0x1E;  // 4-bar icon
     } else if (battery_level >= 40) {
         icon_data = 0x1C;  // 3-bar icon
     } else if (battery_level >= 20) {
         icon_data = 0x18;  // 2-bar icon
     } else {
         icon_data = 0x50;  // 1-bar icon (low battery warning)
     }
 
     lcd_send_command(0x36);  // Switch to extended mode
     lcd_send_command(0x40 | 0x0F);  // Set SEGRAM address for battery icon
     lcd_send_data(icon_data);  // Write battery icon data
     lcd_send_command(0x30);  // Return to normal mode
 
     //printf("[BATTERY] Voltage: %.2fV | Level: %d%% | Icon: 0x%X\n", battery_voltage, battery_level, icon_data);
}

// Charging animation function
void charging_animation() {
    static uint8_t charge_stage = 0;  // Track charging cycle stage
    charge_stage = (charge_stage + 1) % 5;  // Loop through animation

    uint8_t icon_data;
    switch (charge_stage) {
        case 0: icon_data = 0x10; break;  // 1-bar (low battery)
        case 1: icon_data = 0x18; break;  // 2-bar
        case 2: icon_data = 0x1C; break;  // 3-bar
        case 3: icon_data = 0x1E; break;  // 4-bar
        case 4: icon_data = 0x1F; break;  // Full battery
    }

    // Update the LCD with the animated battery icon
    lcd_send_command(0x36);  // Switch to extended mode
    lcd_send_command(0x40 | 0x0F);  // Battery icon slot
    lcd_send_data(icon_data);
    lcd_send_command(0x30);  // Return to normal mode

    // printf("[CHARGING] Battery Animation Stage: %d | Icon: 0x%X\n", charge_stage, icon_data); REMOVED FOR EASIER DEBUGGING
}

// Check charging status
void check_charging_status() {
    bool stat1 = gpio_get(CHARGER_STAT1);  // Read STAT1
    bool stat2 = gpio_get(CHARGER_STAT2);  // Read STAT2

    if (!stat1 && stat2) {  // Charging: STAT1 = LOW, STAT2 = HIGH
        charging_animation();
    } else if (stat1 && !stat2) {  // Fully charged: STAT1 = HIGH, STAT2 = LOW
        lcd_send_command(0x36);
        lcd_send_command(0x40 | 0x0F);
        lcd_send_data(0x1F);  // Solid full battery icon
        lcd_send_command(0x30);
        printf("[CHARGING] Fully Charged - Solid Battery Icon\n");
    } else {  // No charger: Normal battery update
        update_battery_icon();
    }
}

void create_bluetooth_char(void) {
    // Set CGRAM address to 0 (for custom character 0).
    lcd_send_command(0x40);
    lcd_send_data(0x04);  // Row 0: 00100
    lcd_send_data(0x16);  // Row 1: 10110
    lcd_send_data(0x0D);  // Row 2: 01101
    lcd_send_data(0x06);  // Row 3: 00110
    lcd_send_data(0x06);  // Row 4: 00110
    lcd_send_data(0x0D);  // Row 5: 01101
    lcd_send_data(0x16);  // Row 6: 10110
    lcd_send_data(0x04);  // Row 7: 00100
}
 // =========================
 // Subpage Functions
 // =========================
 
 // In subpage mode, we display the current value for the selected setting.
 // For Reel-Up Speed, the value is shown as a percentage.
 // For Auto Stop Length, the value is shown in feet or converted to meters in metric mode.
 // For Contrast and Brightness, the values are shown as percentages.
 void update_subpage(int selection) {
     lcd_send_command(0x01); // Clear display.
     char buffer[21];
     switch (selection) {
         case 0:
             lcd_print_centered("Reel-Up Speed", 1);
             sprintf(buffer, "%d%%", reel_speed);
             lcd_print_centered(buffer, 2);
             lcd_print_normal("Press to Return.", 4);
             break;
         case 1:
             lcd_print_centered("Auto Stop Length", 1);
             if (metric_mode) {
                 float meters = temp_stop_length * 0.3048f;  // Convert feet to meters.
                 snprintf(buffer, sizeof(buffer), "%.1f M", meters);
             } else {
                 sprintf(buffer, "%d FT", temp_stop_length);
             }
             lcd_print_centered(buffer, 2);
             lcd_print_normal("Press to Return.", 4);
             break;
         case 2:
             lcd_print_centered("Metric/Imperial", 1);
             lcd_print_centered(metric_mode ? "Metric" : "Imperial", 2);
             lcd_print_normal("Turn to Toggle", 3);
             lcd_print_normal("Press to Return.", 4);
             break;
         case 3:
             lcd_print_centered("Alarm Toggle", 1);
             lcd_print_centered(alarm_enabled ? "Alarm On" : "Alarm Off", 2);
             lcd_print_normal("Turn to Toggle", 3);
             lcd_print_normal("Press to Return.", 4);
             break;
         case 4:
             lcd_print_centered("Contrast", 1);
             sprintf(buffer, "%d%%", contrast_value);
             lcd_print_centered(buffer, 2);
             lcd_print_normal("Press to Return.", 4);
             break;
         case 5:
             lcd_print_centered("Brightness", 1);
             sprintf(buffer, "%d%%", brightness_value);
             lcd_print_centered(buffer, 2);
             lcd_print_normal("Press to Return.", 4);
             break;
         default:
             lcd_print_centered("Undefined", 1);
             break;
     }
     printf("[SUBPAGE] Updated subpage for selection %d\n", selection);
 }
 
 // When a subsetting is selected, open its subpage.
 void open_subpage(int selection) {
     // Reset encoder debounce flags.
     cw_fall = false;
     ccw_fall = false;
     in_submenu = true;
     temp_stop_length = auto_stop_length; // Store the current stop length for editing
     update_subpage(selection);
     // Set a short ignore period (100 ms) to ignore stale encoder button presses
     // when exiting the subpage.
     subpage_ignore_until = to_ms_since_boot(get_absolute_time()) + 100;
     printf("[SUBPAGE] Opened subpage for selection %d; ignoring encoder input until %d ms.\n",
            selection, subpage_ignore_until);
 }
 
 // =========================
 // Button & Encoder Handling
 // =========================
 
 // Check the encoder button when in the settings menu.
 // A short press will either open a subpage or, if on Save Settings (option 7, index 6),
 // save all settings and return to the main screen.
 void check_settings_selection(void) {
     static bool was_pressed = false;
     static uint32_t press_time = 0;
     bool current = !gpio_get(ENCODER_BTN); // Active low.
     if (ignore_next_press) {
         if (gpio_get(ENCODER_BTN))
             ignore_next_press = false;
         return;
     }
     if (current && !was_pressed) {
         press_time = to_ms_since_boot(get_absolute_time());
         was_pressed = true;
     } else if (!current && was_pressed) {
         uint32_t duration = to_ms_since_boot(get_absolute_time()) - press_time;
         if (duration < 3000) { // Short press.
             // Option 7 (index 6) is Save Settings.
             if (menu_index == 6) {
                /*
                 current_settings.brightness = brightness_value;
                 current_settings.contrast = contrast_value;
                 current_settings.reel_speed = reel_speed;
                 current_settings.alarm_enabled = alarm_enabled;
                 printf("[DEBUG] About to save settings to flash...\n");
                 save_settings(&current_settings);
                 const uint8_t *raw = (const uint8_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
                 printf("First 16 bytes of saved flash:\n");
                 for (int i = 0; i < 16; i++) {
                     printf("%02X ", raw[i]);
                 }
                 printf("\n");
                 */
                 printf("[SAVE] Settings saved:\n");
                 printf("        Reel Speed: %d%%\n", reel_speed);
                 printf("        Auto Stop Length: %d FT\n", stop_length);
                 printf("        Metric Mode: %s\n", metric_mode ? "Metric" : "Imperial");
                 printf("        Alarm: %s\n", alarm_enabled ? "ON" : "OFF");
                 printf("        Contrast: %d%%\n", contrast_value);
                 printf("        Brightness: %d%%\n", brightness_value);
                 in_settings_menu = false;
                 lcd_send_command(0x01);
                 lcd_print_centered("Saving...", 2);
                 // Update icon slot 6 to display a blinking icon.
                 // Here, 0x50 is used as an example for the blink data.
                 update_icon(6, ICON_BLINK);
                 sleep_ms(1000);
                 display_main_screen();
                 update_icon(6, ICON_CLEAR);
                 printf("[SAVE] Returned to main screen after saving.\n");
             } else {
                 // Open the corresponding subpage.
                 open_subpage(menu_index);
             }
         }
         was_pressed = false;
     }
 }
 
 // In a subpage, a short press on the encoder button exits back to the settings menu.
 void check_submenu_exit(void) {
     // Wait until the ignore period expires.
     if (to_ms_since_boot(get_absolute_time()) < subpage_ignore_until)
         return;
     static bool was_pressed = false;
     bool current = !gpio_get(ENCODER_BTN); // Active low.
     if (current && !was_pressed) {
         was_pressed = true;
     } else if (!current && was_pressed) {
         was_pressed = false;
         in_submenu = false;
         // Write adjusted values back to server
        switch (menu_index) {
            case 1:
                stop_length = temp_stop_length;
                write_auto_stop_length(stop_length);
                break;
            case 2:
                write_measurement_system(metric_mode);
                break;
            case 3:
                break;
            
        }
         display_settings_menu(menu_index);
         last_menu_index = menu_index;
         printf("[SUBPAGE] Exiting subpage.\n");
     }
 }
 
 // --- Rotary Encoder Callback ---
 // This ISR uses a 10 ms rate limiter to reduce missed pulses.
 // When not in a subpage, it navigates the settings menu.
 // When in a subpage, it adjusts the current setting (in 10% or 1‑step increments).
 void encoder_callback(uint gpio, uint32_t events) {
     // Read the state of the encoder pins (bits for GPIO10 and GPIO11).
     uint32_t gpio_state = (gpio_get_all() >> 10) & 0b11;
     static uint32_t last_update = 0;
     uint32_t now = to_ms_since_boot(get_absolute_time());
     if (now - last_update < 10)
         return;
     last_update = now;
     
     if (!in_submenu) {
         // Navigation mode: update menu_index based on encoder turns.
         if (gpio == ENCODER_A) {
             if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
             if (ccw_fall && (gpio_state == 0b00)) {
                 ccw_fall = false;
                 cw_fall = false;
                 menu_index--;
                 if (menu_index < 0) menu_index = 0;
                 printf("[ENCODER] CCW - Menu index: %d\n", menu_index);
             }
         } else if (gpio == ENCODER_B) {
             if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
             if (cw_fall && (gpio_state == 0b00)) {
                 cw_fall = false;
                 ccw_fall = false;
                 menu_index++;
                 if (menu_index > 6) menu_index = 6;
                 printf("[ENCODER] CW - Menu index: %d\n", menu_index);
             }
         }
     } else {
         // Subpage mode: adjust the setting corresponding to the current menu_index.
         if (menu_index == 0) { // Reel-Up Speed.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     reel_speed -= 10;
                     if (reel_speed < 0) reel_speed = 0;
                     printf("[SUBPAGE] Decreased Speed: %d%%\n", reel_speed);
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     reel_speed += 10;
                     if (reel_speed > 100) reel_speed = 100;
                     printf("[SUBPAGE] Increased Speed: %d%%\n", reel_speed);
                 }
             }
         } else if (menu_index == 1) { // Auto Stop Length.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     temp_stop_length -= 1;
                     if (temp_stop_length < 0) temp_stop_length = 0;
                     write_auto_stop_length(temp_stop_length); // Write new stop length to server
                     printf("[SUBPAGE] Decreased Length: %d FT\n", temp_stop_length);
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     temp_stop_length += 1;
                     if (temp_stop_length > 100) temp_stop_length = 100;
                     write_auto_stop_length(temp_stop_length); // Write new stop length to server
                     printf("[SUBPAGE] Increased Length: %d FT\n", temp_stop_length);
                 }
             }
         } else if (menu_index == 2) { // Metric/Imperial toggle.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     metric_mode = false;
                     measurement_system = 0;
                     printf("[SUBPAGE] Set to Imperial Mode.\n");
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     metric_mode = true;
                     measurement_system = 1;
                     printf("[SUBPAGE] Set to Metric Mode.\n");
                 }
             }
         } else if (menu_index == 3) { // Alarm toggle.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     alarm_enabled = true;
                     desired_alarm_icon_mode = ICON_CLEAR;
                     alarm_icon_update_needed = true;
                     printf("[SUBPAGE] Alarm Enabled.\n");
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     alarm_enabled = false;
                     desired_alarm_icon_mode = ICON_SOLID;
                     alarm_icon_update_needed = true;
                     printf("[SUBPAGE] Alarm Disabled.\n");
                 }
             }
         } else if (menu_index == 4) { // Contrast.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     contrast_value -= 10;
                     if (contrast_value < 0) contrast_value = 0;
                     printf("[SUBPAGE] Decreased Contrast: %d%%\n", contrast_value);
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     contrast_value += 10;
                     if (contrast_value > 100) contrast_value = 100;
                     printf("[SUBPAGE] Increased Contrast: %d%%\n", contrast_value);
                 }
             }
             // Math for Contrast PWM:
             // We want the final (filtered) voltage to range from 2 V to 2.4 V after a 0.6 V drop.
             // Therefore, the PWM must output between 2.6 V (2 V + 0.6 V) and 3.0 V (2.4 V + 0.6 V).
             // Duty cycle = V_out / 3.3 V.
             // Thus, for contrast_value (0–100%), we set:
             //   V_out = 2.6 V + (contrast_value/100)*0.4 V.
             // Then, PWM level = (V_out/3.3) * 65535.
             pwm_set_gpio_level(CONTRAST_PWM_PIN,
                 (uint16_t)(((1.8f + (contrast_value / 100.0f) * 0.8f) / 3.3f) * 65535));
         } else if (menu_index == 5) { // Brightness.
             if (gpio == ENCODER_A) {
                 if (!cw_fall && (gpio_state == 0b10)) { cw_fall = true; }
                 if (ccw_fall && (gpio_state == 0b00)) {
                     ccw_fall = false;
                     cw_fall = false;
                     brightness_value -= 10;
                     if (brightness_value < 0) brightness_value = 0;
                     printf("[SUBPAGE] Decreased Brightness: %d%%\n", brightness_value);
                 }
             } else if (gpio == ENCODER_B) {
                 if (!ccw_fall && (gpio_state == 0b01)) { ccw_fall = true; }
                 if (cw_fall && (gpio_state == 0b00)) {
                     cw_fall = false;
                     ccw_fall = false;
                     brightness_value += 10;
                     if (brightness_value > 100) brightness_value = 100;
                     printf("[SUBPAGE] Increased Brightness: %d%%\n", brightness_value);
                 }
             }
             // Update PWM duty cycle for Brightness.
             // This uses the full 0–100% range.
             pwm_set_gpio_level(BRIGHTNESS_PWM_PIN,
                 (uint16_t)((brightness_value / 100.0f) * 65535));
         }
         update_subpage_flag = true; // Signal that LCD update is needed.
     }
 }
  
 // --- Check for Long Press to Enter Settings ---
 // Outside settings mode, holding the encoder button for more than 3 seconds enters the settings menu.
 void check_button_long_press(void) {
     if (!in_settings_menu && !gpio_get(ENCODER_BTN)) { // Button active low.
         gpio_put(LED_28, 0);
         gpio_put(LED_13, 0);
         write_fish_alarm(0);
         if (!button_pressed) {
             button_pressed = true;
             button_press_time = to_ms_since_boot(get_absolute_time());
         } else if (to_ms_since_boot(get_absolute_time()) - button_press_time > 3000) {
             in_settings_menu = true;
             menu_index = 0;
             display_settings_menu(menu_index);
             last_menu_index = menu_index;
             ignore_next_press = true; // Prevent immediate subpage open.
             printf("[SETTINGS] Entering settings menu via long press.\n");
         }
     } else if (gpio_get(ENCODER_BTN)) {
         button_pressed = false;
     }
 }
  
 // --- Handle Other Buttons and LED Feedback ---
 // BUTTON_26 and BUTTON_14 control LED_28 and BUZZER_13.
 // BUTTON_15 controls LED_27.
 void handle_buttons(void) {
     /*if (gpio_get(BUTTON_26)) {
         update_icon(1, ICON_SOLID);
         if (alarm_enabled) {
             gpio_put(LED_13, 1);
             gpio_put(LED_28, 1);
         }
     }*/
     if (gpio_get(BUTTON_14)) {
         gpio_put(LED_28, 0);
         gpio_put(LED_13, 0);
         write_fish_alarm(0);
         update_icon(1, ICON_CLEAR);
     }
     static uint32_t button_15_press_time = 0;
     static bool led_27_on = false;
     static bool button_15_released = true;
     if (gpio_get(BUTTON_15)) {
         gpio_put(LED_28, 0);
         gpio_put(LED_13, 0);
         if (button_15_press_time == 0 && button_15_released) {
             button_15_press_time = to_ms_since_boot(get_absolute_time());
             write_fish_alarm(0);
         } else if (to_ms_since_boot(get_absolute_time()) - button_15_press_time > 3000 && !led_27_on) {
             //gpio_put(LED_27, 1);
             update_icon(0x0C, ICON_BLINK);
             led_27_on = true;
             button_15_released = false;
             printf("Writing reel speed to motor: %d%%\n", reel_speed);
             write_motor_speed(reel_speed);
         }
     } else {
         if (button_15_press_time > 0) {
             if (!led_27_on) {
                 //gpio_put(LED_27, 0);
                 update_icon(0x0C, ICON_CLEAR);
                 // Stops the motor
                 write_motor_speed(0);
             } else {
                 led_27_on = false;
             }
             button_15_press_time = 0;
             button_15_released = true;
         }
     }
 }
  
 // --- GPIO Setup ---
 // Initialize all required GPIO pins, including LCD, encoder, buttons, and PWM channels.
 void gpio_setup(void) {
     int control_pins[] = {PIN_RS, PIN_RW, PIN_E, PIN_RES};
     for (int i = 0; i < 4; i++) {
         gpio_init(control_pins[i]);
         gpio_set_dir(control_pins[i], GPIO_OUT);
     }
     for (int i = 0; i < 8; i++) {
         gpio_init(data_pins[i]);
         gpio_set_dir(data_pins[i], GPIO_OUT);
     }
     gpio_init(ENCODER_A);
     gpio_set_dir(ENCODER_A, GPIO_IN);
     gpio_pull_up(ENCODER_A);
     gpio_init(ENCODER_B);
     gpio_set_dir(ENCODER_B, GPIO_IN);
     gpio_pull_up(ENCODER_B);
     gpio_init(ENCODER_BTN);
     gpio_set_dir(ENCODER_BTN, GPIO_IN);
     gpio_pull_up(ENCODER_BTN);
     // Set up encoder interrupts on both rising and falling edges.
     gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
     gpio_set_irq_enabled(ENCODER_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
     
     // Set up PWM for Contrast on GPIO 21.
     gpio_set_function(CONTRAST_PWM_PIN, GPIO_FUNC_PWM);
     uint slice_contrast = pwm_gpio_to_slice_num(CONTRAST_PWM_PIN);
     pwm_set_wrap(slice_contrast, 65535);
     // Mapping math for Contrast PWM:
     // Desired effective voltage after a 0.6 V drop: 2.0 V to 2.4 V.
     // Therefore, PWM must produce 2.6 V to 3.0 V.
     // V_out = 2.6 + (contrast_value/100)*0.4, then duty cycle = V_out / 3.3.
     pwm_set_chan_level(slice_contrast, pwm_gpio_to_channel(CONTRAST_PWM_PIN),
         (uint16_t)(((1.8f + (contrast_value / 100.0f) * 0.8f) / 3.3f) * 65535));
     pwm_set_enabled(slice_contrast, true);
     
     // Set up PWM for Brightness on GPIO 22.
     gpio_set_function(BRIGHTNESS_PWM_PIN, GPIO_FUNC_PWM);
     uint slice_brightness = pwm_gpio_to_slice_num(BRIGHTNESS_PWM_PIN);
     pwm_set_wrap(slice_brightness, 65535); // Adjusted from 65535 to 5000 for faster refresh
     // Brightness uses the full 0–100% range.
     pwm_set_chan_level(slice_brightness, pwm_gpio_to_channel(BRIGHTNESS_PWM_PIN),
         (uint16_t)((brightness_value / 100.0f) * 65535));
     pwm_set_enabled(slice_brightness, true);
     
     // Additional buttons.
     //gpio_init(BUTTON_26);
     //gpio_set_dir(BUTTON_26, GPIO_IN);
     //gpio_pull_down(BUTTON_26);
     gpio_init(BUTTON_14);
     gpio_set_dir(BUTTON_14, GPIO_IN);
     gpio_pull_down(BUTTON_14);
     gpio_init(BUTTON_15);
     gpio_set_dir(BUTTON_15, GPIO_IN);
     gpio_pull_down(BUTTON_15);
     
     // LED outputs.
     gpio_init(LED_28);
     gpio_set_dir(LED_28, GPIO_OUT);
     gpio_put(LED_28, 0);
     gpio_init(LED_13);
     gpio_set_dir(LED_13, GPIO_OUT);
     gpio_put(LED_13, 0);
     //gpio_init(LED_27);
     //gpio_set_dir(LED_27, GPIO_OUT);
     //gpio_put(LED_27, 0);

     // Initialize GPIO0 and GPIO1 as input for charger detection
     gpio_init(CHARGER_STAT1);
     gpio_set_dir(CHARGER_STAT1, GPIO_IN);
     gpio_pull_up(CHARGER_STAT1);  // Ensure stable input

     gpio_init(CHARGER_STAT2);
     gpio_set_dir(CHARGER_STAT2, GPIO_IN);
     gpio_pull_up(CHARGER_STAT2);
 }
  // Initializes the PWM outputs for both contrast and brightness to the default 50% setting.
void init_pwm_display_settings(void) {
    // For Contrast:
    // We want the effective (filtered) voltage to be between 2.0V and 2.4V after a 0.6V drop.
    // Therefore, the PWM must output between 2.6V and 3.0V.
    // The mapping: V_out = 2.6V + (contrast_value/100) * 0.4V.
    // For contrast_value = 50, V_out = 2.6 + 0.5*0.4 = 2.6 + 0.2 = 2.8 V.
    // The duty cycle is then 2.8/3.3, and the PWM level is that fraction times 65535.
    uint slice_contrast = pwm_gpio_to_slice_num(CONTRAST_PWM_PIN);
    uint16_t contrast_level = (uint16_t)(((1.8f + (contrast_value / 100.0f) * 0.8f) / 3.3f) * 65535);
    pwm_set_chan_level(slice_contrast, pwm_gpio_to_channel(CONTRAST_PWM_PIN), contrast_level);
    // For Brightness:
    // Brightness uses the full 0–100% range. At 50% the duty cycle is 0.5.
    uint slice_brightness = pwm_gpio_to_slice_num(BRIGHTNESS_PWM_PIN);
    uint16_t brightness_level = (uint16_t)((brightness_value / 100.0f) * 65535);
    pwm_set_chan_level(slice_brightness, pwm_gpio_to_channel(BRIGHTNESS_PWM_PIN), brightness_level);
}

// This function runs on core 1 and manages Bluetooth LE connection and notifications  
void BT_Core(void) {
    stdio_init_all();
    //sleep_ms(4000);

    // Initialized BluetoothLE stack and attempts connection
    init_bluetooth();

    // Flag to let LCD initialize with accurate values
    BT_Init = 1;

    while (true) {
        tight_loop_contents();
    }

}

 // --- Main Loop ---
 int main(void) {
     stdio_init_all();

     multicore_launch_core1(BT_Core);

     //sleep_ms(5000);

     printf("[SYSTEM] Starting UI Components\n");
     
     gpio_setup();

     /*
     if (!load_settings(&current_settings)) {
        printf("Saved settings not found! Loading defaults...\n");
        current_settings.brightness = 100;
        current_settings.contrast = 50;
        current_settings.reel_speed = 50;
        current_settings.alarm_enabled = true;
    } else {
        printf("Settings successfully loaded! Brightness: %d%%, Contrast: %d%%, Reel Speed: %d%%, Alarm: %s\n",
               current_settings.brightness, current_settings.contrast, current_settings.reel_speed,
               current_settings.alarm_enabled ? "ON" : "OFF");
    }

     // Sync to runtime variables
     brightness_value = current_settings.brightness;
     contrast_value = current_settings.contrast;
     reel_speed = current_settings.reel_speed;
     alarm_enabled = current_settings.alarm_enabled;
     */

     init_pwm_display_settings();
     lcd_init();
     adc_init();  // Initialize ADC for battery monitoring
     adc_gpio_init(BATTERY_ADC_PIN);  // Enable ADC on GPIO26
    
     display_main_screen();
     update_battery_icon();
     create_bluetooth_char();
     
     while (1) {
        
        stop_length = auto_stop_length;
         // If not in settings mode, check for a long press to enter settings.
         //printf("Temp Auto Stop: %d, Auto Stop (Client): %d, Auto Stop (Server): %d\n", temp_stop_length, stop_length, auto_stop_length);
         if (!in_settings_menu) {
            metric_mode = (measurement_system == 1);
             check_button_long_press();
             if (connection_status == 1 && fart == 0) (display_main_screen(), fart = 1);
             if (FULL_REEL_LENGTH != line_length) {
                FULL_REEL_LENGTH = line_length;
                display_main_screen();
             }
             if (drag_value != drag_set) {
                drag_value = drag_set;
                display_main_screen();
             }
             if (fish_alarm == 1) {
                    // If fish alarm is triggered, show the alarm icon and turn on the buzzer
                    update_icon(1, ICON_SOLID); // Update the alarm icon to solid
                    gpio_put(LED_13, 1); // Turn on Buzzer 13 for alarm indication
                    gpio_put(LED_28, 1); // Turn on LED 28 for alarm indication
                } else if (fish_alarm == 0) {
                    // If fish alarm is cleared, turn off the alarm icon and buzzer
                    update_icon(1, ICON_CLEAR);
                    gpio_put(LED_13, 0);
                    gpio_put(LED_28, 0);
             }
         }
         // If in settings mode, either check for subpage exit or selection.
         if (in_settings_menu) {
             if (in_submenu) {
                 check_submenu_exit();
             } else {
                 check_settings_selection();
                 if (menu_index != last_menu_index) {
                     display_settings_menu(menu_index);
                     last_menu_index = menu_index;
                 }
             }
         }

         // Check charging status every 0.5 seconds
         static uint32_t last_battery_update = 0;
             if (to_ms_since_boot(get_absolute_time()) - last_battery_update > 500) {
             check_charging_status();
             last_battery_update = to_ms_since_boot(get_absolute_time());
         }


         // If in a subpage and an update is flagged, update the subpage display.
         if (in_submenu && update_subpage_flag) {
             update_subpage(menu_index);
             update_subpage_flag = false;
         }

         // If Icon needs to be updated it will be flagged
         if (alarm_icon_update_needed) {
             update_icon(9, desired_alarm_icon_mode); // Use the appropriate icon slot.
             alarm_icon_update_needed = false;
         }
         handle_buttons();
         sleep_ms(50);
     }
 }
 
