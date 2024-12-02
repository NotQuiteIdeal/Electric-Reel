#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
//#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "math.h"

/*double calculate_length(double Dmax, double Dmin, double rotations)
{
    double PI = 3.14159265358979323846;
    double part1 = (Dmax / 12) * PI;
    double factor = pow((rotations / 0.9231), (5000.0 / 5119.0));
    double part2 = (Dmax - Dmin) / (29114.4 * pow(factor, -0.985));
    double length = (part1 - part2) * rotations;
    return length;    
}
*/
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
    



int main()
{

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
    int rotations =0;
    int count = 0;
    int newcount = 0;
    int rot = 0;
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
        if (rotations >=1)
        {
        int length = calculate_length(Dmax, Dmin, rotations);
        printf("length: %d\n", length); 
        }
        else if (rotations <=0){
         
            length =0;
            printf("length: %d\n", length); 
        }
        //int length = calculate_length(Dmax, Dmin, rotations);
        //printf("length: %lu\n", length); 
        //printf("length: %d\n", length); 
        }
     
     
      
    oldVal = newVal;
    
    }
  
}
