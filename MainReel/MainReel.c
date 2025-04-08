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
extern uint8_t auto_stop_length;
extern uint8_t measurement_system;
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
extern volatile int Position2[16];
extern volatile int AutoStopLen;
extern volatile int SpoolDiameter;
extern int count_drag;
extern volatile bool linereset;
extern volatile int alarmsensitivity;
extern volatile bool isImperial;


// Define min and max duty cycle limits (default values)
volatile uint16_t min_duty = 0; // 0% duty cycle
volatile uint16_t max_duty = 65000; // 95% duty cycle

volatile int length = 0; 
volatile int drag = 0;
//volatile int Position[16] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 65, 70, 75, 80};
int count = 0;
volatile int rotations = 0;
//int count_drag = 0;
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
    if (count_drag <= 8) { //position 1
        drag = Position2[0];
    } else if (count_drag >= 9 && count_drag <= 23) { //position 2
        drag = Position2[1];
    } else if (count_drag >= 24 && count_drag <= 33) { //position 2
        drag = Position2[2];
    } else if (count_drag >= 34 && count_drag <= 46) {
        drag = Position2[3];
    } else if (count_drag >= 47 && count_drag <= 58) {
        drag = Position2[4];
    } else if (count_drag >= 59 && count_drag <= 70) {
        drag = Position2[5];
    } else if (count_drag >= 71 && count_drag <= 84) {
        drag = Position2[6];
    } else if (count_drag >= 85 && count_drag <= 99) {
        drag = Position2[7];
    } else if (count_drag >= 100 && count_drag <= 110) {
        drag = Position2[8];
    } else if (count_drag >= 111 && count_drag <= 124) {
        drag = Position2[10];
    } else if (count_drag >= 125 && count_drag <= 139) {
        drag = Position2[11];
    } else if (count_drag >= 140 && count_drag <= 150) {
        drag = Position2[12];
    } else if (count_drag >= 151 && count_drag <= 165) {
        drag = Position2[13];
    } else if (count_drag >= 166 && count_drag <= 178) {
        drag = Position2[14];
    } else if (count_drag >= 179 && count_drag <= 250) {
        drag = Position2[15];
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

    if (line_length - auto_stop_length <= 0) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_PIN), pwm_gpio_to_channel(PWM_PIN), 0); // Stops motor if there is no more line
        return;
    }
    int activation_threshold = 5;
    // Temporary min/max speed init
    //MinSpeed = 5;
    //MaxSpeed = 100;

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint16_t adc_threshold = (activation_threshold * PWM_RESOLUTION) / 100; //
    //printf("Duty Cycle: %u, ADC Threshold: %u, MinSpeed: %d, MaxSpeed: %d, Mobile Motor Control: %d\n", duty_cycle, adc_threshold, MinSpeed, MaxSpeed, mobile_motor_control);
    if (duty_cycle < adc_threshold) {
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), 0);
        return;
    }

    min_duty = (MinSpeed * PWM_RESOLUTION) / 100; // Scale to 16-bit
    max_duty = (MaxSpeed * PWM_RESOLUTION) / 100; // Scale to 16-bit

    float scaled = (float)(duty_cycle - adc_threshold) / (PWM_RESOLUTION - adc_threshold);
    if (scaled > 1.0f) scaled = 1.0f;

    uint16_t output_duty = min_duty + (uint16_t)(scaled * (max_duty - min_duty));
    if (output_duty > 65000) output_duty = 65000;

    //printf( "Setting PWM Duty Cycle: %u (scaled from %u)\n", output_duty, duty_cycle);

    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), output_duty);
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

        if (duty_cycle_test > 1000) {
            in_settings_menu = false; // Exits settings menu
        }

        if (mobile_motor_control == 0 || duty_cycle_test > 1000 || in_settings_menu == true) { // Tests if reel is control 
            duty_cycle = duty_cycle_test;
        } else {
            duty_cycle = (motor_speed * PWM_RESOLUTION) / 100; // Scale to 16-bit
        }
        // Apply limits
        set_pwm_duty(duty_cycle);
        //printf("Pot Value: %u, Duty Cycle: %u\n", pot_value, duty_cycle);

        // Print values
        float voltage = pot_value * (3.3f / 4095.0f);
        float duty_percent = (duty_cycle * 100.0f) / PWM_RESOLUTION;
        //printf("ADC: %u, Voltage: %.2fV, Duty Cycle: %.2f%%\n", pot_value, voltage, duty_percent);

        // Example: Dynamically update limits (could be triggered by a button/UART)
        // Uncomment this line to change limits dynamically during execution
        // update_limits(20000, 45000);
        AutoStopLen = auto_stop_length;
        isImperial = (measurement_system == 0);
        if (!isImperial) printf("Metric Toggled\n");
        screen_update(length, drag);
    }
}

int main() {
    stdio_init_all(); // Initialize standard I/O'
    multicore_launch_core1(core1);
    // Initialize GPIO pins for input
    gpio_init(14); 
    gpio_set_dir(14, GPIO_IN); 
    gpio_init(15); 
    gpio_set_dir(15, GPIO_IN); 
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
    double Dmax = SpoolDiameter/10;
    double Dmin = 2.00;
    //int rotations = 0;
    int count = 0;
    //int count_drag = 0;
    int oldVal = 0;
    int newVal = 0;
    int oldValdrag = 0;
    int newValdrag = 0;
    int buttonPressed = 0;
    int button = 0;
    int reg_count = 0;

    int alarm_progress = 0; // Progress towards the alarm trigger
    int alarm_trigger = 0; // Holds value of length when sensitivity is tested

    while (true) {
        if (line_length - auto_stop_length <= 0) { 
            reg = true; // Reenable alarm after fish is reeled in
        }
        
        /*if (gpio_get(15) == 1){ // If button is pressed, reset drag count
            count_drag = 0;
            drag = 0;
        }*/

        //printf("%d\n", count_drag);
        int chanA = gpio_get(11); // Read encoder channels for rotation count
        int chanB = gpio_get(10);
        int chanDragA = gpio_get(15); // Read encoder channels for drag count
        int chanDragB = gpio_get(14);

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
        //printf("New Encoder Value: %d, Old Value: %d\n", newVal, oldVal);
        /*
         if (linereset = true){
                rotations = 0;
                linereset = false;
            }*/
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
             if (count==166){
                 rotations++;
                 count = 0; 
             }
             if (count == -166){  
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
        //printf("Line Length: %d\n", line_length);
        printf("Count: %d, Rotations: %d, Length: %d\n", count, rotations, length);
        
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
                //printf("%d\n", drag);
            }
            
        }
        //printf("Drag Count: %d, Drag Value: %d\n", count_drag, drag);
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
                printf("Fish alarm toggled: %s\n", reg ? "ON" : "OFF");
            }
        }
        
        // Buzzer Logic
        if (reg && drag != 0 && New_Length > Old_Length) {
            //if (alarm_trigger == 0) alarm_trigger = Old_Length, printf("resetting alarm trigger length\n");
            //if (New_Length - alarm_trigger >= alarmsensitivity) fish_alarm = 1, alarm_trigger = 1, printf("Trigger latched\n");
            //fish_alarm = 1; RE-ENABLE TO REMOVE ALARM SENSITIVITY
            //printf("Fish alarm on!\n");
            if (fish_alarm == 0) alarm_trigger++, printf("Incrementing Alarm Trigger!\n");
            //printf("Alarm sensitivity: %d, Alarm Trigger: %d, reg: %d, New Length: %d, Old Length: %d, Fish Alarm: %d\n", alarmsensitivity, alarm_trigger, reg, New_Length, Old_Length, fish_alarm);
            if (alarm_trigger >= alarmsensitivity) {
                //printf("Activating alarm...\n");
                fish_alarm = 1;
                alarm_trigger = 0;
            }
            
        } else if (reg && drag != 0 && New_Length == Old_Length && fish_alarm == 1) {
            fish_alarm = 1;
            //printf("Fish alarm still on!\n");
        } else {
            if (fish_alarm == 1) printf("Releasing trigger\n");
            fish_alarm = 0;
            //alarm_trigger = 0; // Reset the alarm trigger when reel is engaged or drag is 0
        
        }
        gpio_put(BUZZER_PIN, fish_alarm);
     } 
}
