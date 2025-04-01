#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"
#include "math.h"
#include "time.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "btstack.h"
#include "server_common.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "LCDTEST634.h"

// Multicore variables for handling encoder counts and calculations
// Pin Definitions
#define PWM_PIN 20       // PWM output pin
#define POT_PIN 26       // Potentiometer ADC input

// PWM resolution
#define PWM_RESOLUTION 65535  // Full 16-bit range

// Define Bluetooth LE Variables
static btstack_packet_callback_registration_t hci_event_callback_registration;

extern uint16_t line_length;
extern uint16_t drag_set;
extern uint8_t motor_status;
extern uint8_t motor_speed;
extern uint8_t fish_alarm;
extern uint8_t ping_test_status;
extern int ping_rate_count;
volatile int mobile_motor_control = 0; // 0 is no, 1 is yes
bool ping_start = false;
struct repeating_timer timer2;

extern hci_con_handle_t con_handle;
extern int le_notification_enabled;
extern volatile int send_update_flag;

// Define UI Variables
extern volatile bool in_settings_menu;
extern volatile int MaxSpeed;
extern volatile int MinSpeed;


// Define min and max duty cycle limits (default values)
volatile uint16_t min_duty = 0; // 0% duty cycle
volatile uint16_t max_duty = 65000; // 95% duty cycle

volatile int length = 0; 
volatile int drag = 0;
int count = 0;
int rotations = 0;
int count_drag = 0;
int oldVal = 0;
int newVal = 0;

// Function to calculate the length based on encoder rotations
int calculate_length(double Dmax, double Dmin, double rotations)
{
    double PI = 3.14;
    double part1 = (Dmax * PI) / 12.0; // Calculate part of the length formula
    double part2 = (Dmax - Dmin) / (12 * 2426.2 * pow((rotations / 0.9231), (5000.0 / 5119.0)) * pow(1, -0.985)); // Calculate other part
    int tempLength = (part1 - part2) * rotations; // Calculate total length
    return tempLength; // Return calculated length
}

// Update the drag value based on drag count from the encoder
int update_drag(double count_drag) {
    if (count_drag >= 0 && count_drag <= 5) {
        drag = 00;
    } else if (count_drag >= 6 && count_drag <= 10) {
        drag = 05;
    } else if (count_drag >= 11 && count_drag <= 15) {
        drag = 10;
    } else if (count_drag >= 16 && count_drag <= 20) {
        drag = 15;
    } else if (count_drag >= 21 && count_drag <= 25) {
        drag = 20;
    } else if (count_drag >= 26 && count_drag <= 30) {
        drag = 25;
    } else if (count_drag >= 31 && count_drag <= 35) {
        drag = 30;
    } else if (count_drag >= 36 && count_drag <= 40) {
        drag = 35;
    } else if (count_drag >= 41 && count_drag <= 45) {
        drag = 40;
    } else if (count_drag >= 46 && count_drag <= 50) {
        drag = 45;
    } else if (count_drag >= 51 && count_drag <= 55) {
        drag = 50;
    } else if (count_drag >= 56 && count_drag <= 60) {
        drag = 55;
    } else if (count_drag >= 61 && count_drag <= 65) {
        drag = 60;
    } else if (count_drag >= 66 && count_drag <= 70) {
        drag = 65;
    } else if (count_drag >= 71 && count_drag <= 75) {
        drag = 70;
    } 
    return drag; // Return the calculated drag
}

// Function to initialize PWM
void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_wrap(slice_num, PWM_RESOLUTION); // Full range 0-65535
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), 0);
    pwm_set_enabled(slice_num, true);
}

// Function to read ADC value
uint16_t read_potentiometer() {
    return adc_read();
}

// Function to set PWM duty cycle with limits
void set_pwm_duty(uint16_t duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    min_duty = (MinSpeed * PWM_RESOLUTION) / 100; // Scale to 16-bit
    max_duty = (MaxSpeed * PWM_RESOLUTION) / 100; // Scale to 16-bit
    if (max_duty > 65000) max_duty = 65000; // Ensure max duty cycle does not exceed 100%

    // Constrain duty cycle within min and max limits
    if (duty_cycle < min_duty) duty_cycle = min_duty;
    if (duty_cycle > max_duty) duty_cycle = max_duty;

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), duty_cycle);
}

// Function to update min/max limits dynamically
void update_limits(uint16_t new_min, uint16_t new_max) {
    if (new_min < new_max) {
        min_duty = new_min;
        max_duty = new_max;
    }
}

bool update_sender_callback(struct repeating_timer *t) {
    //printf("Sending BLE updates...\n");
    //printf("Line Length in Reel: %d , Line length in Server: %d\n", length, line_length);
    send_ble_updates();

    return true;
}

void core1() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0);
    

    // Setup PWM
    setup_pwm(PWM_PIN);

    // Initialize screen
    screen_setup();

    //sleep_ms(4000);

    // Initialize CYW43 driver
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43 driver\n");
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

    // Set up timer for sending updates
    struct repeating_timer update_timer;
    add_repeating_timer_ms(-200, update_sender_callback, NULL, &update_timer);
    uint16_t duty_cycle = 0;
    uint16_t duty_cycle_test = 0;

    while (true) {
        

        // PWM Code
        uint16_t pot_value = read_potentiometer();
        duty_cycle_test = (pot_value * PWM_RESOLUTION) / 4095; // Scale to 16-bit

        if (mobile_motor_control == 0 || duty_cycle_test > 1000 || in_settings_menu == false) { // Tests if reel is control 
            duty_cycle = duty_cycle_test;
        } else {
            duty_cycle = (motor_speed * PWM_RESOLUTION) / 100; // Scale to 16-bit
        }
        // Apply limits
        set_pwm_duty(duty_cycle);

        // Print values
        //float voltage = pot_value * (3.3f / 4095.0f);
        //float duty_percent = (duty_cycle * 100.0f) / PWM_RESOLUTION;
        //printf("ADC: %u, Voltage: %.2fV, Duty Cycle: %.2f%%\n", pot_value, voltage, duty_percent);

        // Example: Dynamically update limits (could be triggered by a button/UART)
        // Uncomment this line to change limits dynamically during execution
        // update_limits(20000, 45000);

        screen_update(length, drag);
    }
}

int main() {
    stdio_init_all(); // Initialize standard I/O'
    multicore_launch_core1(core1);
    // Initialize GPIO pins for input
    gpio_init(15); 
    gpio_set_dir(15, GPIO_IN); 
    gpio_init(14); 
    gpio_set_dir(8, GPIO_IN); 
    gpio_init(15); 
    gpio_set_dir(9, GPIO_IN); 
    gpio_init(10); 
    gpio_set_dir(10, GPIO_IN); 
    gpio_init(11); 
    gpio_set_dir(11, GPIO_IN);

    int New_Length = 0; //Change to new line count
    int Old_Length = 0; //Change to Old line count
    bool reg = true;
    // Define GPIO pins
    const int B1_PIN = 6;
    const int B2_PIN = 7;
    const int BUZZER_PIN = 8;  // Now used for a buzzer instead of an LED
    
    // Initialize Buttons as Inputs (Active High)
    gpio_init(B1_PIN);
    gpio_set_dir(B1_PIN, GPIO_IN);
    gpio_init(B2_PIN);
    gpio_set_dir(B2_PIN, GPIO_IN);
    
    // Initialize Buzzer as Output
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    double Dmax = 3.685;
    double Dmin = 2.00;
    int rotations = 0;
    int count = 0;
    int count_drag = 0;
    int oldVal = 0;
    int newVal = 0;
    int oldValdrag = 0;
    int newValdrag = 0;
    int buttonPressed = 0;
    int button = 0;
    int reg_count = 0;

    while (true) {
        /*if (gpio_get(15) == 1){ // If button is pressed, reset drag count
            count_drag = 0;
            drag = 0;
        }*/

        //printf("%d\n", count_drag);
        int chanA = gpio_get(10); // Read encoder channels for rotation count
        int chanB = gpio_get(11);
        int chanDragA = gpio_get(14); // Read encoder channels for drag count
        int chanDragB = gpio_get(15);

        // Update rotation count based on encoder signals
        if (chanA == 1) {
            if (chanB == 1){
                newVal = 11;
            } else {
                newVal = 10;
            }
        } else {
            if (chanB == 1){
                newVal = 01;
            } else {
                newVal = 00;
            }
        }

        // Update drag count based on encoder signals
        if (chanDragA == 1) {
            if (chanDragB == 1) {
                newValdrag = 11;
            } else {
                newValdrag = 10;
            }
        } else {
            if (chanDragB == 1) {
                newValdrag = 01;
            } else {
                newValdrag = 00;
            }
        }
        Old_Length = line_length;
        // Update rotation count and calculate line length
        if (newVal != oldVal) { 
             if (oldVal == 11) {
                 if (newVal == 10) {
                     gpio_put(22, 1);
                     count--;
                 } else {
                     gpio_put(22, 0);
                     count++;
                 }
             } else if (oldVal == 10) {
                 if (newVal == 00) {
                     gpio_put(22, 1);
                     count--;
                 } else {
                     gpio_put(22, 0);
                     count++;
                 }
             } else if (oldVal == 00) {
                 if (newVal == 01) {
                     gpio_put(22, 1);
                     count--;
                 } else {
                     gpio_put(22, 0);
                     count++;
                 }
             } else if (oldVal == 01) {
                 if (newVal == 11) {
                     gpio_put(22, 1);
                     count--;
                 } else {
                     gpio_put(22, 0);
                     count++;
                 }
             } 
             if (count==208){
                 rotations++;
                 count = 0; 
             }
             if (count == -208){  
                 --rotations;
                 count = 0;        
             }
             if (rotations >=1) {
                 length = calculate_length(Dmax, Dmin, rotations);
                 line_length = length;
                 //printf("Reel Line Length: %d\n", length);
             }
         }
         oldVal = newVal;
        New_Length = line_length;

        // Update drag count based on encoder
         if (newValdrag != oldValdrag) {
            if (oldValdrag == 11) {
                if (newValdrag == 10) {
                    gpio_put(22, 1);
                    count_drag--;
                } else {
                    gpio_put(22, 0);
                    count_drag++;
                }
            } else if (oldValdrag == 10) {
                if (newValdrag == 00) {
                    gpio_put(22, 1);
                    count_drag--;
                } else {
                    gpio_put(22, 0);
                    count_drag++;
                }
            } else if (oldValdrag == 00) {
                if (newValdrag == 01) {
                    gpio_put(22, 1);
                    count_drag--;
                } else {
                    gpio_put(22, 0);
                    count_drag++;
                }
            } else if (oldValdrag == 01) {
                if (newValdrag == 11) {
                    gpio_put(22, 1);
                    count_drag--;
                } else {
                    gpio_put(22, 0);
                    count_drag++;
                }
            }
            if (count_drag >=1) {
                drag =  update_drag(count_drag);
                drag_set = drag;
                printf("%d\n", drag);
                
                
            }
        }
        //update_drag();
        oldValdrag = newValdrag;
         //Clears queue and updates length value for core1    

         // Check Buzzer
        // Read buttons (Active High: 1 when pressed, 0 when not pressed)
        int B1 = gpio_get(B1_PIN);  
        int B2 = gpio_get(B2_PIN);
        // Toggle reg only when both buttons are pressed (Turns fish alarm functionality on/off)
        if (B1 == 1 && B2 ==1) { //THIS IS FOR MULTIPLE INPUTS
            reg_count++; // Debouncer without slowing code down
            if (reg_count >= 600000) {
                reg_count = 0; // Reset counter after toggling
                reg = !reg;
                //printf("Fish alarm toggled: %s\n", reg ? "ON" : "OFF");
            }
        }
        // Buzzer Logic
        if (reg && drag != 0 && New_Length > Old_Length) {
            fish_alarm = 1;
            //printf("Fish alarm on!\n");
        } else if (reg && drag != 0 && New_Length == Old_Length && fish_alarm == 1) {
            fish_alarm = 1;
            //rintf("Fish alarm still on!\n");
        } else {
            fish_alarm = 0;
        }
        gpio_put(BUZZER_PIN, fish_alarm);
     } 
}
