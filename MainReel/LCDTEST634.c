#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include <math.h>
#define I2C_PORT i2c0
#define SDA_PIN 4 // i2c data
#define SCL_PIN 5 // i2c clock
#define CFA634_I2C_ADDR 0x42  // I2C address (7-bit format)
#define MAX_LINE_LENGTH 20  // Max characters per line
#define ENCODER_A 0 // encoder a side
#define ENCODER_B 1 // encoder b side
#define ENCODER_BTN 2 // encoder button
#define RBTN 7 // right button
#define LBTN 6 // left button
#define PULSES_PER_UPDATE 2  // Update after every 4 pulses
#define DEBOUNCE_TIME_MS 35     // Short pause to allow proper pulse registration (in milliseconds)
 // Store previous values for menus
volatile int last_linelength = -1;// used to detremine if to update display
volatile int last_dragset = -1;// used to detremine if to update display
static int previous_encoder_value = 0;// used for reading encoder
volatile bool menuActive = false; // used to determine if menu is active for encoder
volatile bool button_pressed = false; //used to store if button is pressed or not
volatile uint32_t button_press_time = 0; // time button is pressed
volatile bool in_settings_menu = false;// if in settings menu
volatile int menu_index = 0; // index for tracking in setting
volatile bool cw_fall = false; // used for encoder
volatile bool ccw_fall = false;// used for encoder
int last_menu_index = -1; // to ensure the screen displays
volatile bool ignore_next_press = false; // ingore if less than that time and not in a settings menu
volatile bool in_submenu = false; // stores if in submen
volatile int selected_digit = 1; // which digit the cursors is on
volatile bool isImperial = true; //false = metric, true = imperial
volatile int AutoStopLen = 10;     // Store value for setting
volatile int MaxSpeed = 70;        // Store value for setting
volatile int MinSpeed = 10;        // Store value for setting
volatile int SpoolDiameter = 37;   // Store value for setting
volatile int selected_menu = 0; // which menu_index is selected
volatile bool right_pressed = false; // store if right button is pressed
volatile bool left_pressed = false; // store if left button is pressed
volatile double uplinecon = 0; // value used for conversion
volatile double updragcon = 0; // value used for conversion
volatile int linecon = 0; // value used for conversion
volatile int dragcon = 0; // value used for conversion
volatile bool encoder_btpress = false; // store if encoder is pressed
volatile bool long_press = false; // check if long press
volatile bool update_display = true; // sets flag to true until 
volatile bool value_changed = false; // Track if any value has changed
volatile int last_AutoStopLen = 10; // previous value next 4
volatile int last_MaxSpeed = 70;
volatile int last_MinSpeed = 10;
volatile int last_SpoolDiameter = 37;
volatile double SDia = 0.0; // used for calculating spool diameter to decimal to 1
volatile bool update_screen = false; // flag to determine to update screen
volatile int LineLength = 0;
volatile int Position1 = 1; // position for sorting through recal drag
volatile bool Pos0 = false; // flag to tell to go to recal drag call function
volatile int Position2[16] = {0, 1, 2, 4, 10, 20, 27, 36, 45, 58, 68, 70, 72, 76, 80, 84}; // drag recalibration and display
volatile int lastPosition2[16] = {0, 1, 2, 4, 10, 20, 27, 36, 45, 58, 68, 70, 72, 76, 80, 84}; // drag recalibration 
volatile bool DragNext = false; // flag to determine to go to next drag
volatile bool ISPOS0 = false; // False means it is not and true means it is
int alarmsensitivity = 1;
//int rotations = 0;
volatile bool alarmsen = false;
volatile bool linereset = false;
//leave alone
static int encoder_value = 0;
static uint32_t last_state = 0;
static uint32_t last_read_time = 0;
const uint32_t debounce_delay = 50;
volatile int last_encoder_A = 0;
volatile int last_encoder_B = 0;
volatile int pulse1=0;
volatile uint32_t last_interrupt_time = 0;
int count_drag = 0;

extern volatile int mobile_motor_control;
extern uint16_t line_length;
extern uint16_t drag_set;
extern uint8_t motor_status;
extern uint8_t motor_speed;
extern uint8_t fish_alarm;
extern uint8_t ping_test_status;
extern uint8_t measurement_system;
extern uint8_t auto_stop_length;


// Function to send a command to the LCD
void cfa634_send_command(uint8_t cmd) {
    uint8_t buffer[1] = {cmd};  // Store the command in a buffer
    int ret = i2c_write_blocking(I2C_PORT, CFA634_I2C_ADDR, buffer, sizeof(buffer), false);
    sleep_ms(25);
}
void setcursor(uint8_t col, uint8_t row) {
    if (row > 3) row = 3;  // Ensure row is within bounds

    uint8_t offset = col + (row * 0x00);  // Correct offset calculation
    uint8_t cmd = 0x10 | offset;          // Use addition, not bitwise OR
    //printf("Sending cursor command: 0x%02X\n", cmd);  // Debug print

    cfa634_send_command(cmd);
    sleep_ms(10);  // Small delay for stability
}
void cfa634_print(const char *text) {
    char formatted_text[MAX_LINE_LENGTH + 1];
    strncpy(formatted_text, text, MAX_LINE_LENGTH);
    formatted_text[MAX_LINE_LENGTH] = '\0';  

    for (size_t i = 0; i < MAX_LINE_LENGTH && formatted_text[i] != '\0'; i++) {
        // Filter out unwanted control characters
        if (formatted_text[i] >= 32 && formatted_text[i] <= 126) {  // Printable ASCII range
            uint8_t data = (uint8_t)formatted_text[i];
            i2c_write_blocking(I2C_PORT, CFA634_I2C_ADDR, &data, 1, false);
            //sleep_ms(10);  // Stability delay
        }
    }
}
void i2c_setup() {
    i2c_init(I2C_PORT, 100 * 1000);  // Set I2C speed to 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}
void cfa634_clear_screen() {
    uint8_t clear_cmd = 0x01;  // Clear screen command
    cfa634_send_command(clear_cmd);
    sleep_ms(3);
}
// End of Screen Set up
// Function called to allow for the user to recalibrate Drag
void RecalibrateDrag(int Position){
    printf("this is position initial value: %d", Position);
    if (Pos0){
        switch(Position-1){
            case 0:
                char display_value0[21];
                sprintf(display_value0, "         %02d         ", lastPosition2[1]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 1      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value0);
                setcursor(0, 3);
                cfa634_print("      R > NEXT      ");
                break;
            case 1:
                char display_value1[21];
                sprintf(display_value1, "         %02d         ", lastPosition2[2]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 2      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value1);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 2:
                char display_value2[21];
                sprintf(display_value2, "         %02d         ", lastPosition2[3]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 3      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value2);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 3:
                char display_value3[21];
                sprintf(display_value3, "         %02d         ", lastPosition2[4]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 4      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value3);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 4:
                char display_value4[21];
                sprintf(display_value4, "         %02d         ", lastPosition2[5]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 5      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value4);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 5:
                char display_value5[21];
                sprintf(display_value5, "         %02d         ", lastPosition2[6]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 6      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value5);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 6:
                char display_value6[21];
                sprintf(display_value6, "         %02d         ", lastPosition2[7]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 7      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value6);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 7:
                char display_value7[21];
                sprintf(display_value7, "         %02d         ", lastPosition2[8]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 8      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value7);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 8:
                char display_value8[21];
                sprintf(display_value8, "         %02d         ", lastPosition2[9]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 9      ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value8);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 9:
                char display_value9[21];
                sprintf(display_value9, "         %02d         ", lastPosition2[10]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 10     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value9);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 10:
                char display_value10[21];
                sprintf(display_value10, "         %02d         ", lastPosition2[11]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 11     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value10);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 11:
                char display_value11[21];
                sprintf(display_value11, "         %02d         ", lastPosition2[12]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 12     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value11);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 12:
                char display_value12[21];
                sprintf(display_value12, "         %02d         ", lastPosition2[13]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 13     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value12);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 13:
                char display_value13[21];
                sprintf(display_value13, "         %02d         ", lastPosition2[14]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 14     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value13);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            case 14:
                char display_value14[21];
                sprintf(display_value14, "         %02d         ", lastPosition2[15]);  // %02d ensures two-digit format
                setcursor(0, 0);
                cfa634_print("    POSITION 15     ");
                setcursor(0, 1);
                cfa634_print("  DRAG VALUE 0-99   ");
                setcursor(0, 2);
                cfa634_print(display_value14);
                setcursor(0, 3);
                cfa634_print("     R > NEXT       ");
                break;
            default:
                break;

        }
    }
}
// Function called to see the submenus
void selectedmenudisplay(int pos) {
    switch (pos) {
        case 0:
            setcursor(0, 0);
            cfa634_print("      SAVING        ");
            setcursor(0, 1);
            cfa634_print("                    ");
            setcursor(0, 2);
            cfa634_print("  RETURNING > MAIN  ");
            setcursor(0, 3);
            cfa634_print("                    ");
            sleep_ms(500); // delay to show message
            in_submenu = false; // this line and next tell to return to main
            in_settings_menu = false;

            // set values to update the main screen
            last_linelength = line_length;
            last_dragset = drag_set;
            break;
        case 1:
            setcursor(0, 0);
            cfa634_print("                    ");
            setcursor(0, 1);
            cfa634_print("   RESETTING LINE   ");
            setcursor(0, 2);
            cfa634_print("   LEFT TO CANCEL   ");
            setcursor(0, 3);
            cfa634_print("  R TO SAVE > RET   ");
            break;
        case 2: {
            char display_value[21];
            
            sprintf(display_value, "         %d          ", AutoStopLen);
            
            // Update display immediately
            
            setcursor(0, 0);
            cfa634_print("  ENTER AUTO STOP   ");
            setcursor(0, 1);
            cfa634_print("  LENG VALUE 0-99   ");
            setcursor(0, 2);
            cfa634_print(display_value);
            setcursor(0, 3);
            cfa634_print("  L > CNCL R > SAVE ");
            auto_stop_length = AutoStopLen;
            break;
        }
        case 3:
            setcursor(0, 0);
            cfa634_print(isImperial ? " CURRENT: IMPERIAL  " : "   CURRENT:METRIC   ");
            setcursor(0, 1);
            cfa634_print("   LEFT > METRIC    ");
            setcursor(0, 2);
            cfa634_print("   RIGHT > IMPER    ");
            setcursor(0, 3);
            cfa634_print("PRESS DIAL > RETURN ");
            measurement_system = isImperial;
            break;
        case 4: {
            char display_value1[21];
            sprintf(display_value1, "       %%%3d          ", MaxSpeed);  // Right-aligned, supports 0-999
                
            // Update display immediately
            setcursor(0, 0);
            cfa634_print("  ENTER MAX SPEED   ");
            setcursor(0, 1);
            cfa634_print("   VALUE 50-100     ");
            setcursor(0, 2);
            cfa634_print(display_value1);
            setcursor(0, 3);
            cfa634_print(" L > CNCL R > SAVE  ");
            break;
            }
        case 5: {
            char display_value2[21];
            sprintf(display_value2, "       %%%3d          ", MinSpeed);  // Right-aligned, supports 0-999
                
            // Update display immediately
            setcursor(0, 0);
            cfa634_print("  ENTER MIN SPEED   ");
            setcursor(0, 1);
            cfa634_print("    VALUE 0-40      ");
            setcursor(0, 2); 
            cfa634_print(display_value2);
            setcursor(0, 3);
            cfa634_print(" L > CNCL R > SAVE  ");
            break;
            }            
        case 6: {
            char display_value3[21];
            sprintf(display_value3, "        %.1f          ", SpoolDiameter/10.0);  // Casting to int for display
            
            // Update display immediately
            setcursor(0, 0);
            cfa634_print("  ENTER Spool DI-   ");
            setcursor(0, 1);
            cfa634_print("  AMETER VAL 0.0-9.9   ");
            setcursor(0, 2);
            cfa634_print(display_value3);
            setcursor(0, 3);
            cfa634_print(" L > CNCL R > SAVE  ");
            break;
        }
        case 7:
            setcursor(0, 0);
            cfa634_print("                    ");
            setcursor(0, 1);
            cfa634_print("    POSITION 0      ");
            setcursor(0, 2);
            cfa634_print("    LEFT < YES      ");
            setcursor(0, 3);
            cfa634_print("  PRESS DIAL > RET  ");
            break;
        case 8:
            char display_value4[21];
            sprintf(display_value4, "        %02d          ", alarmsensitivity);
            setcursor(0, 0);
            cfa634_print(" ALARM SENSITIVITY  ");
            setcursor(0, 1);
            cfa634_print("    FEET 1 TO 10    ");
            setcursor(0, 2);
            cfa634_print(display_value4);
            setcursor(0, 3);
            cfa634_print("   RIGHT TO EXIT    ");
        default: 
            break;
    }
}
// Function called to see the settings list
void settingsdisplay(int pos){
    //sleep_ms(25);
    switch (pos) {
        case 0:
            setcursor(0, 0);
            cfa634_print(">0.RETURN           ");
            setcursor(0, 1);
            cfa634_print(" 1.RESET LINE       ");
            setcursor(0, 2);
            cfa634_print(" 2.AUTO STOP LENGTH ");
            setcursor(0, 3);
            cfa634_print(" 3.IMPERIAL>METRIC  ");
            break;
        case 1:
            setcursor(0, 0);
            cfa634_print(" 0.RETURN           ");
            setcursor(0, 1);
            cfa634_print(">1.RESET LINE       ");
            setcursor(0, 2);
            cfa634_print(" 2.AUTO STOP LENGTH ");
            setcursor(0, 3);
            cfa634_print(" 3.IMPERIAL>METRIC  ");
            break;
        case 2:
            setcursor(0, 0);
            cfa634_print(" 0.RETURN           ");
            setcursor(0, 1);
            cfa634_print(" 1.RESET LINE       ");
            setcursor(0, 2);
            cfa634_print(">2.AUTO STOP LENGTH ");
            setcursor(0, 3);
            cfa634_print(" 3.IMPERIAL>METRIC  ");
            break;
        case 3:
            setcursor(0, 0);
            cfa634_print(" 0.RETURN           ");
            setcursor(0, 1);
            cfa634_print(" 1.RESET LINE       ");
            setcursor(0, 2);
            cfa634_print(" 2.AUTO STOP LENGTH ");
            setcursor(0, 3);
            cfa634_print(">3.IMPERIAL>METRIC  ");
            break;
        case 4:
            setcursor(0, 0);
            cfa634_print(">4.MAX SPEED        ");
            setcursor(0, 1);
            cfa634_print(" 5.MIN SPEED        ");
            setcursor(0, 2);
            cfa634_print(" 6.SPOOL DIAMETER   ");
            setcursor(0, 3);
            cfa634_print(" 7.RECAL DRAG       ");
            break;
        case 5:
            setcursor(0, 0);
            cfa634_print(" 4.MAX SPEED        ");
            setcursor(0, 1);
            cfa634_print(">5.MIN SPEED        ");
            setcursor(0, 2);
            cfa634_print(" 6.SPOOL DIAMETER   ");
            setcursor(0, 3);
            cfa634_print(" 7.RECAL DRAG       ");
            break;
        case 6:
            setcursor(0, 0);
            cfa634_print(" 4.MAX SPEED        ");
            setcursor(0, 1);
            cfa634_print(" 5.MIN SPEED        ");
            setcursor(0, 2);
            cfa634_print(">6.SPOOL DIAMETER   ");
            setcursor(0, 3);
            cfa634_print(" 7.RECAL DRAG       ");
            break;
        case 7:
            setcursor(0, 0);
            cfa634_print(" 4.MAX SPEED        ");
            setcursor(0, 1);
            cfa634_print(" 5.MIN SPEED        ");
            setcursor(0, 2);
            cfa634_print(" 6.SPOOL DIAMETER   ");
            setcursor(0, 3);
            cfa634_print(">7.RECAL DRAG       ");
            break;
        case 8:
            setcursor(0, 0);
            cfa634_print(">8.ALARM SENSITIVITY");
            setcursor(0, 1);
            cfa634_print("                    ");
            setcursor(0, 2);
            cfa634_print("                    ");
            setcursor(0, 3);
            cfa634_print("                    ");
        default: break;
    }
}
// Function to display main screen information for line and drag in either imperial or metric units
void cfa634_main(int line, int drag) {
    isImperial = (measurement_system == 0); // 1 for imperial, 0 for metric
    // Check if the current measurement system is imperial
    if (isImperial) {
        //cfa634_clear_screen(); // Clear the display
        setcursor(0, 0);
        cfa634_print("       MAIN         "); // Display "MAIN" on the first line
        setcursor(0, 1);
        // Format and display the line length in feet
        if (line < 10) { // If line is a single digit
            char formattedline[20];
            sprintf(formattedline, "  LINE: 000%d FEET   ", line);
            cfa634_print(formattedline);
        } else if (line < 100) { // If line is two digits
            char formattedline[20];
            sprintf(formattedline, "  LINE: 00%d FEET   ", line);
            cfa634_print(formattedline);
        } else if (line < 1000) { // If line is three digits
            char formattedline[20];
            sprintf(formattedline, "  LINE: 0%d FEET   ", line);
            cfa634_print(formattedline);
        } else { // If line is four digits or more
            char formattedline[20];
            sprintf(formattedline, "  LINE: %d FEET   ", line);
            cfa634_print(formattedline);
        }
        setcursor(0, 2);
        // Format and display the drag in foot-pounds
        if (drag < 10) { // If drag is a single digit
            char formatteddrag[20];
            sprintf(formatteddrag, "  DRAG: 00%d FT-LBS  ", drag);
            cfa634_print(formatteddrag);
        } else if (drag < 100) { // If drag is two digits
            char formatteddrag[20];
            sprintf(formatteddrag, "  DRAG: 0%d FT-LBS  ", drag);
            cfa634_print(formatteddrag);
        } else { // If drag is three digits or more
            char formatteddrag[20];
            sprintf(formatteddrag, "  DRAG: %d FT-LBS  ", drag);
            cfa634_print(formatteddrag);
        }
        setcursor(0, 3);
        cfa634_print("                    "); // Empty line for spacing
    } else if (!isImperial) { // If the current measurement system is metric
        // Convert line and drag to metric units
        uplinecon = line * 0.3048; // Convert feet to meters
        updragcon = drag * 1.3558179483; // Convert foot-pounds to newton-meters
        linecon = (int)round(uplinecon);
        dragcon = (int)round(updragcon);
        //cfa634_clear_screen(); // Clear the display
        setcursor(0, 0);
        cfa634_print("       MAIN         "); // Display "MAIN" on the first line
        // Format and display the line length in meters
        setcursor(0, 1);
        char formattedline[21];
        sprintf(formattedline, "   LINE: %04d M     ", linecon); 
        cfa634_print(formattedline);        // Display Line value from 0-9999 in meters
        // Format and display the drag in newton-meters
        setcursor(0, 2);
        char formatteddrag[21];
        sprintf(formatteddrag, "   DRAG: %03d NM     ", dragcon); 
        cfa634_print(formatteddrag);        // Display Drag value from 0-999 in newton-meters
        setcursor(0, 3);
        cfa634_print("                    "); // Empty line for spacing
        //printf("Line %d and Linecon %.2f\n", line, uplinecon);

    }
}
// Function to read the state of the left and right buttons
void read_btn() {
    static bool last_left_state = false;  
    static bool last_right_state = false;  

    bool left_state = gpio_get(LBTN);  
    bool right_state = gpio_get(RBTN);  

    left_pressed = false;  
    right_pressed = false;  

    if (left_state && !last_left_state) {  
        left_pressed = true;  
        mobile_motor_control = 0;
        fish_alarm = 0;
        //printf("Left button pressed\n"); 
        if (!in_settings_menu && !in_submenu && (ISPOS0 == false)){
            count_drag = 0;
            ISPOS0 = true;
        }
        if (in_submenu && menu_index != 0 && menu_index != 1 && menu_index != 3 && menu_index != 7) {  
            if (menu_index == 2) AutoStopLen = last_AutoStopLen;
            if (menu_index == 4) MaxSpeed = last_MaxSpeed;
            if (menu_index == 5) MinSpeed = last_MinSpeed;
            if (menu_index == 6) SpoolDiameter = last_SpoolDiameter;
            in_submenu = false;  
            settingsdisplay(menu_index);  // Exit submenu with saving
        } else if (menu_index ==1){
            in_submenu = false;
            settingsdisplay(menu_index); 
        } else if (menu_index == 3) {  
            isImperial = false;  // change to metric
        } else if (menu_index == 7){
            Pos0 = true;
            Position1 = 1;
            DragNext = false;
            RecalibrateDrag(Position1);
        }
    }

    if (right_state && !last_right_state) {  
        right_pressed = true;  
        mobile_motor_control = 0;
        fish_alarm = 0;
        //printf("Right button pressed\n");
        if (!in_settings_menu && !in_submenu && (ISPOS0 == false)){
            count_drag = 0;
            ISPOS0 = true;
        }        
        if (in_submenu && menu_index != 0 && menu_index != 1 && menu_index != 3 && menu_index != 7) {  
            // Save to last variable then exit
            if (menu_index == 2) last_AutoStopLen = AutoStopLen;
            if (menu_index == 4) last_MaxSpeed = MaxSpeed;
            if (menu_index == 5) last_MinSpeed = MinSpeed;
            if (menu_index == 6) last_SpoolDiameter = SpoolDiameter;
            in_submenu = false;  
            settingsdisplay(menu_index);  
        } else if (menu_index == 1) {
            in_submenu = false;
            settingsdisplay(menu_index); 
        } else if (menu_index == 3) {  
            isImperial = true;  
        } else if (menu_index == 7){
            if (Pos0 == true) {
                if ((Position1) < 15){
                    DragNext = true;
                    Position1 ++;
                    RecalibrateDrag(Position1);
                } else if (Position1 == 15){
                    in_submenu = false;
                    Pos0 = false;
                    settingsdisplay(menu_index);
                }
            }
        }
    }

    last_left_state = left_state;  
    last_right_state = right_state;  
}
// Function to handle  value update in submenus and settings list. Set up as an interrupt
///*
void encoder_isr(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_time = 0;
    static uint8_t last_state = 0b11;
    static int pulse_count = 0;
    mobile_motor_control = 0;
    fish_alarm = 0;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    // **Debounce filtering**
    if (now - last_interrupt_time < 15) return;
    last_interrupt_time = now;

    // **Read encoder state**
    int a = gpio_get(ENCODER_A);
    int b = gpio_get(ENCODER_B);
    uint8_t state = (a << 1) | b;

    // **Ignore duplicate readings**
    if (state == last_state) return;

    // **Process transitions correctly**
    if ((state == 0b10 && last_state == 0b11) ||  
        (state == 0b00 && last_state == 0b10) ||  
        (state == 0b01 && last_state == 0b00) ||  
        (state == 0b11 && last_state == 0b01)) {  
        pulse_count++;
        //printf("[CW] Pulse Incremented. Count: %d\n", pulse_count);
    }
    else if ((state == 0b01 && last_state == 0b11) ||  
             (state == 0b00 && last_state == 0b01) ||  
             (state == 0b10 && last_state == 0b00) ||  
             (state == 0b11 && last_state == 0b10)) {  
        pulse_count--;
        //printf("[CCW] Pulse Decremented. Count: %d\n", pulse_count);
    }

    last_state = state;  // **Update last state for next ISR call**

    // **Update menu index when threshold is reached**
    if (!in_submenu && in_settings_menu) {
        if (pulse_count >= 4) {
            menu_index = (menu_index < 8) ? menu_index + 1 : 8;
            //printf("[CW] Menu Index: %d\n", menu_index);
            pulse_count = 0;
            update_screen = true;
        } 
        else if (pulse_count <= -3) {
            menu_index = (menu_index > 0) ? menu_index - 1 : 0;
            //printf("[CCW] Menu Index: %d\n", menu_index);
            pulse_count = 0;
            update_screen = true;
        }
    } else if(in_submenu){
        if (pulse_count >=3) {
            if (menu_index == 2 && AutoStopLen < 99) {
                //AutoStopLen = last_AutoStopLen;
                AutoStopLen++;
                //printf("Auto stop length inc %d\n", AutoStopLen);
            } else if (menu_index == 4 && MaxSpeed < 100) {
                //MaxSpeed = last_MaxSpeed;
                MaxSpeed += 10;
            } else if (menu_index == 5 && MinSpeed < 45) {
                //MinSpeed = last_MinSpeed;
                MinSpeed += 10;
            } else if (menu_index == 6 && SpoolDiameter < 99) {
                //SpoolDiameter = last_SpoolDiameter;
                SpoolDiameter++;
            } else if (menu_index == 8 && alarmsensitivity < 10) {
                alarmsensitivity++;
                alarmsen = true;
            } else if (menu_index == 7) {
                // Update only the variable corresponding to Position1
                for (int i = 1; i < 15; i++) {
                    Position2[i] = lastPosition2[i];  // Copy each element individually
                } // this helps to save the value so it doesnt restart from zero
                if (Position1 == 1 && Position2[1] < 99) Position2[1]++;
                if (Position1 == 2 && Position2[2] < 99) Position2[2]++;
                if (Position1 == 3 && Position2[3] < 99) Position2[3]++;
                if (Position1 == 4 && Position2[4] < 99) Position2[4]++;
                if (Position1 == 5 && Position2[5] < 99) Position2[5]++;
                if (Position1 == 6 && Position2[6] < 99) Position2[6]++;
                if (Position1 == 7 && Position2[7] < 99) Position2[7]++;
                if (Position1 == 8 && Position2[8] < 99) Position2[8]++;
                if (Position1 == 9 && Position2[9] < 99) Position2[9]++;
                if (Position1 == 10 && Position2[10] < 99) Position2[10]++;
                if (Position1 == 11 && Position2[11] < 99) Position2[11]++;
                if (Position1 == 12 && Position2[12] < 99) Position2[12]++;
                if (Position1 == 13 && Position2[13] < 99) Position2[13]++;
                if (Position1 == 14 && Position2[14] < 99) Position2[14]++;
                if (Position1 == 14 && Position2[15] < 99) Position2[15]++;
                for (int i = 1; i < 15; i++) {
                    lastPosition2[i] = Position2[i];  // Copy each element individually
                }
            }
            menuActive = true;
            update_screen = true;
            pulse_count = 0;
        } else if (pulse_count <= -2) {
            if (menu_index == 2 && AutoStopLen > 0) {
                //AutoStopLen = last_AutoStopLen;
                AutoStopLen--;
                //printf("Auto stop length dec %d\n", AutoStopLen);
            } else if (menu_index == 4 && MaxSpeed > 45) {
                MaxSpeed = last_MaxSpeed;
                MaxSpeed -= 10;
            } else if (menu_index == 5 && MinSpeed > 0) {
                MinSpeed = last_MinSpeed;
                MinSpeed -= 10;
            } else if (menu_index == 6 && SpoolDiameter > 0) {
                SpoolDiameter = last_SpoolDiameter;
                SpoolDiameter--;
            } else if (menu_index == 8 && alarmsensitivity > 1) {
                alarmsensitivity--;
                alarmsen = true;
            } else if (menu_index == 7) {
                // Update only the variable corresponding to Position1
                for (int i = 1; i < 15; i++) {
                    Position2[i] = lastPosition2[i];  // Copy each element individually
                } // this helps to save the value so it doesnt restart from zero
                if (Position1 == 1 && Position2[1] < 99) Position2[1]--;
                if (Position1 == 2 && Position2[2] < 99) Position2[2]--;
                if (Position1 == 3 && Position2[3] < 99) Position2[3]--;
                if (Position1 == 4 && Position2[4] < 99) Position2[4]--;
                if (Position1 == 5 && Position2[5] < 99) Position2[5]--;
                if (Position1 == 6 && Position2[6] < 99) Position2[6]--;
                if (Position1 == 7 && Position2[7] < 99) Position2[7]--;
                if (Position1 == 8 && Position2[8] < 99) Position2[8]--;
                if (Position1 == 9 && Position2[9] < 99) Position2[9]--;
                if (Position1 == 10 && Position2[10] < 99) Position2[10]--;
                if (Position1 == 11 && Position2[11] < 99) Position2[11]--;
                if (Position1 == 12 && Position2[12] < 99) Position2[12]--;
                if (Position1 == 13 && Position2[13] < 99) Position2[13]--;
                if (Position1 == 14 && Position2[14] < 99) Position2[14]--;
                if (Position1 == 14 && Position2[15] < 99) Position2[15]--;
                for (int i = 1; i < 15; i++) {
                    lastPosition2[i] = Position2[i];  // Copy each element individually
                }
            }
            menuActive = true;
            update_screen = true;
            pulse_count = 0;
        }
    }
}
//*/
// Function to check the state of the encoder button and handle button presses
void check_encoder() {
    static bool button_was_pressed = false; 
    static uint32_t button_press_time = 0;
    static bool button_pressed = false; 

    if (gpio_get(ENCODER_BTN) == 1) {  // Button is pressed
        //printf( "Encoder button pressed\n");
        if (!button_pressed) {  
            mobile_motor_control = 0;
            button_pressed = true; 
            button_press_time = time_us_64(); 
        }
        else if ((time_us_64() - button_press_time > 3000000) && !button_was_pressed) {  
            in_settings_menu = true;  
            menu_index = 0;  
            settingsdisplay(menu_index);  
            last_menu_index = menu_index;  
            ignore_next_press = true;  
            button_was_pressed = true;  
        }
    } else {  // Button is released
        if (button_pressed) { 
            mobile_motor_control = 0; 
            uint32_t press_duration = time_us_64() - button_press_time; 
            if (press_duration < 1000000) {  
                if (in_settings_menu && !in_submenu) {  
                    selected_menu = menu_index;  
                    in_submenu = true;  
                    
                    // Store last known values before modifying
                    if (menu_index == 2) AutoStopLen = last_AutoStopLen;
                    if (menu_index == 4) MaxSpeed = last_MaxSpeed;
                    if (menu_index == 5) MinSpeed = last_MinSpeed;
                    if (menu_index == 6) SpoolDiameter = last_SpoolDiameter;

                    last_AutoStopLen = auto_stop_length;
                    
                    selectedmenudisplay(selected_menu);  
                } 
                // **Only close menu 3 with a short press**
                else if (in_submenu && (menu_index == 3 || menu_index ==7)) {
                    in_submenu = false;
                    settingsdisplay(menu_index);
                }
            }
            button_pressed = false;  
            button_was_pressed = false;  
        }
    }
}
// Function for Input and Output Set up
void gpio_setup() {
    // Encoder Reading
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A); // pull-up resistor for encoder a
    gpio_init(ENCODER_B);
    gpio_set_dir(ENCODER_B, GPIO_IN);
    gpio_pull_up(ENCODER_B); // pull-up resistor for encoder b
    gpio_init(ENCODER_BTN);
    gpio_set_dir(ENCODER_BTN, GPIO_IN);
    gpio_pull_down(ENCODER_BTN); // pull-down for encoder button
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled(ENCODER_B, GPIO_IRQ_EDGE_RISE, true);
    // Button Reading Left Then Right
    gpio_init(LBTN);
    gpio_set_dir(LBTN, GPIO_IN);
    gpio_pull_down(LBTN); // Pull-down for Left Button
    gpio_init(RBTN);
    gpio_set_dir(RBTN, GPIO_IN);
    gpio_pull_down(RBTN); //Pull-down for Right Button
}
// Function to handle all while loop changes
void screen_update(int linelength, int dragset) {
    read_btn(); // check button state
    check_encoder(); // check encoder button state

    //isImperial = (measurement_system == 0);
    //AutoStopLen = auto_stop_length;

    // Check if we're not in the settings menu or submenu
    /*
    if (!in_settings_menu && !in_submenu) {
        if (linelength != last_linelength || dragset != last_dragset) {
            cfa634_main(linelength, dragset);
            last_linelength = linelength;  // Store the new values
            last_dragset = dragset;
        }
    }*/

    if (!in_settings_menu && !in_submenu) {
        if (ISPOS0 == true){
            AutoStopLen = auto_stop_length;
            isImperial = (measurement_system == 0);
            cfa634_main(linelength, dragset);
        } else if(ISPOS0 == false) {
            setcursor(0, 0);
            cfa634_print("  SET DRAG POS AND  ");
            setcursor(0, 1);
            cfa634_print("   SPEED CTRL TO 0  ");
            setcursor(0, 2);
            cfa634_print("    LEFT OR RIGHT   ");
            setcursor(0, 3);
            cfa634_print("   BTN TO CONFIRM   ");
        }
        
    }

    // In settings mode, check for submenu or main menu
    if (in_settings_menu) {
        if (in_submenu) {
            mobile_motor_control = 0;
            // Check if any submenu value has changed, then update screen
            if (menuActive) {
                if (menu_index == 2 && AutoStopLen != last_AutoStopLen) { // Check if AutoStopLen changed
                    selectedmenudisplay(menu_index);
                    last_AutoStopLen = AutoStopLen;  // Store the new value
                } else if (menu_index == 4 && MaxSpeed != last_MaxSpeed) { // Check if MaxSpeed changed
                    selectedmenudisplay(menu_index);
                    last_MaxSpeed = MaxSpeed;  // Store the new value
                } else if (menu_index == 5 && MinSpeed != last_MinSpeed) { // Check if MinSpeed changed
                    selectedmenudisplay(menu_index);
                    last_MinSpeed = MinSpeed;  // Store the new value
                } else if (menu_index == 6 && SpoolDiameter != last_SpoolDiameter) { // Check if SpoolDiameter changed
                    selectedmenudisplay(menu_index);
                    last_SpoolDiameter = SpoolDiameter;  // Store the new value
                } else if (menu_index == 7 && (Position2 != lastPosition2 || DragNext == true)){
                    RecalibrateDrag(Position1);
                    printf("Position 1 %d\n", Position1);
                    DragNext = false;
                } else if (menu_index == 8 && alarmsen) {
                    selectedmenudisplay(menu_index);
                    alarmsen = false;
                }
            }
        } else {
            // Check if menu index has changed and update display
            if (menu_index != last_menu_index) {
                settingsdisplay(menu_index);
                last_menu_index = menu_index;
            }
        }
    }
    
    //sleep_ms(100); // Add small delay to prevent excessive updates
}
// function to handle the initialization of the screen
void screen_setup(){
    stdio_init_all();
    i2c_setup();
    gpio_setup();
    sleep_ms(500); // Wait for LCD to power up
    cfa634_clear_screen();
    cfa634_send_command(0x14);
}
/*
int main() {
    screen_setup();
    int linelength = 0;
    int dragset = 0;
    while (1) {
        screen_update(linelength, dragset);
    }
}
    */