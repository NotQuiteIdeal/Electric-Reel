#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"







int main()
{
    stdio_init_all();
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_init(8);
    gpio_set_dir(8, GPIO_IN);
    gpio_init(9);
    gpio_set_dir(9, GPIO_IN);
    
    int chanA = gpio_get(8);
    int chanB = gpio_get(9);
    int oldVal = 0;
    int newVal = 0;
    while (true) {
        chanA = gpio_get(8);
        chanB = gpio_get(9);
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
        
        if (newVal != oldVal) {
            if (oldVal == 11) {
                if (newVal == 10) {
                    gpio_put(22, 1);
                } else {
                    gpio_put(22, 0);
                }
            } else if (oldVal == 10) {
                if (newVal == 00) {
                    gpio_put(22, 1);
                } else {
                    gpio_put(22, 0);
                }
            } else if (oldVal == 00) {
                if (newVal == 01) {
                    gpio_put(22, 1);
                } else {
                    gpio_put(22, 0);
                }
            } else if (oldVal == 01) {
                if (newVal == 11) {
                    gpio_put(22, 1);
                } else {
                    gpio_put(22, 0);
                }
            }
        }
    oldVal = newVal;
    }
}
