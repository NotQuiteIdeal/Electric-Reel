#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

//Initializing pins
#define PWM_PIN 20         // Pin for PWM output (changed to GPIO 6)
#define POT_PIN 26        // Pin for potentiometer (ADC0)

//Initializing the PWM function and setting the output GPIO pin
void setup_pwm() {
    //selecting which GPIO pin the PWM function goes out of 
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    // Gets the slice number associated with the PWM GPIO pin for PWM functionality
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);


    pwm_set_wrap(slice_num, 255); 

    pwm_set_enabled(slice_num, true);
}

    // Read potentiometer value (ADC) and returns that value
uint16_t read_potentiometer() {
    // Read potentiometer value (ADC) 
    uint16_t adc_value = adc_read();
    // Returning that value
    return adc_value;
}

int main() {
    // Initialize stdio and ADC
    stdio_init_all();
    adc_init();

    // Select the potentiometer pin (ADC0) for the adc input
    adc_gpio_init(POT_PIN);
    adc_select_input(0);

    // calling the pwm function
    setup_pwm();

    while (true) {
        // setting the potentiometer value potentiometer value (0-4095)
        uint16_t pot_value = read_potentiometer();

        // Scale potentiometer value to 0-255 for PWM duty cycle
        uint8_t duty_cycle = (pot_value * 255) / 4095;  // Scale to 8-bit

        // Set PWM slices to set output PWM duty cycle
        uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
        pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle); // Set duty cycle for channel A

        // Value Conversions and Print
        const float conversion_factor = 3.3f / (1 << 12);
        const float DC_CONV = 100.0f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n, Duty Cycle: %f" , result, result * conversion_factor, result * DC_CONV);
        
        // Wait a bit before reading again
        sleep_ms(100);
    }

    return 0;
}
