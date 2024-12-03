/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

/* Example code to drive a 16x2 LCD panel via a I2C bridge chip (e.g. PCF8574)

   NOTE: The panel must be capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO 4 (pin 6)-> SDA on LCD bridge board
   GPIO 5 (pin 7)-> SCL on LCD bridge board
   3.3v (pin 36) -> VCC on LCD bridge board
   GND (pin 38)  -> GND on LCD bridge board
*/
// commands
const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;

const int LCD_ENABLE_BIT = 0x04;

// By default these LCD display drivers are on bus address 0x27
static int addr = 0x27;

// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0

#define MAX_LINES      2
#define MAX_CHARS      16

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val) {
#ifdef i2c_default
    i2c_write_blocking(i2c_default, addr, &val, 1, false);
#endif
}

void lcd_toggle_enable(uint8_t val) {
    // Toggle enable pin on LCD display
    // We cannot do this too quickly or things don't work
#define DELAY_US 600
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    i2c_write_byte(high);
    lcd_toggle_enable(high);
    i2c_write_byte(low);
    lcd_toggle_enable(low);
}

void lcd_clear(void) {
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

// go to location on LCD
void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, LCD_COMMAND);
}

static inline void lcd_char(char val) {
    lcd_send_byte(val, LCD_CHARACTER);
}

void lcd_string(const char *s) {
    while (*s) {
        lcd_char(*s++);
    }
}

void lcd_init() {
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);

    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}

void i2c_lcd_init() {
    #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
        #warning i2c/lcd_1602_i2c example requires a board with I2C pins
    #else
        // Initialize I2C
        i2c_init(i2c_default, 100 * 1000);
        gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
        gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

        lcd_init();
    #endif
}

int calculate_length(double Dmax, double Dmin, double rotations)
{
    double PI = 3.14;
    double part1 = (Dmax * PI) / 12.0;
    //double factor = pow((rotations / 0.9231), (0.977));
    //double part2 = (Dmax - Dmin) / (12 * 2426.2 * pow((rotations / 0.9231), (5000.0 / 5119.0)) * pow(1, -0.985));
    double part2 = (Dmax - Dmin) / (12 * 2426.2 * pow((rotations / 0.9231), (5000.0 / 5119.0)) * pow(1, -0.985));
    int tempLength = (part1 - part2) * rotations;
    
   // printf("PI: %d\n", PI);
   // printf("part1 (Dmax): %d\n", part1);
   // printf("part2: %d\n", part2);   
    return tempLength;
}

void display(int lineLength) {
    // Convert the integer to a string and assign it to message[1]
    char lineLengthStr[16];
    char *message[] = {      // Array to hold the messages to display
        "Line Length", ""    // Second string starts as empty
    };
    snprintf(lineLengthStr, sizeof(lineLengthStr), "%d", lineLength);
    message[1] = lineLengthStr;
    printf("length (in Disp): %d\n", lineLength);
    
    for (int line = 0; line < MAX_LINES; line++) {
        lcd_set_cursor(line, (MAX_CHARS / 2) - strlen(message[line]) / 2);
        lcd_string(message[line]);
    }
    
}

int main() {
    

   // Initialize I2C and LCD
    i2c_lcd_init();
    
    char *message[] =
            {
                    "Line Length", ""
            }; 
    
    
    stdio_init_all();
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_init(8);
    gpio_set_dir(8, GPIO_IN);
    gpio_init(9);
    gpio_set_dir(9, GPIO_IN);
    
    double Dmax = 3.685;
    double Dmin = 2.00;
    //float length = 0;
    int length = 0;
    //float rotations =0;
    int rotations =2;
    int count = 0;
    int newcount = 0;
    int rot = 0;
    int chanA = gpio_get(8);
    int chanB = gpio_get(9);
    int oldVal = 0;
    int newVal = 0;
    time_t dispWait = clock();
    display(length);
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
            printf("count: %d\n", count); 
            
            if (count==204){
                rotations++;
                count = 0;
                
            }
            //else if (count == -834){
            if (count == -204){  
                --rotations;
                //rotations == rotations - 1;
                count = 0;
                    
            }
            
            
            printf("rotations: %d\n", rotations);
            
            //if (length < 0);
            //{
            //    length = 0;
            //}  
            if (rotations >=1) {
                int length = calculate_length(Dmax, Dmin, rotations);
                printf("length: %d\n", length); 
                if (((double)(clock() - dispWait)) / CLOCKS_PER_SEC >= 1.0) {
                    printf("length (in if): %d\n", length);
                    display(length);
                    dispWait = clock();  // Reset the timer for the next update
                }
            
            }
            else if (rotations <=0) {
            
                length =0;
                printf("length: %d\n", length);
                if (((double)(clock() - dispWait)) / CLOCKS_PER_SEC >= 1.0) {
                    printf("length (in if): %d\n", length);
                    display(length);
                    dispWait = clock();  // Reset the timer for the next update
                } 
            }
            //int length = calculate_length(Dmax, Dmin, rotations);
            //printf("length: %lu\n", length); 
            //printf("length: %d\n", length); 
        }
     
     
      
        oldVal = newVal;
        
        
        
        
    }

}
