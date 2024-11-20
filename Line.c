#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

double calculate_length(double Dmax, double Dmin, double rotations)
{
    double PI = 3.14159265358979323846;
    double part1 = (Dmax * PI) / 12.0;
    double factor = pow((rotations / 0.9231), (5000.0 / 5119.0));
    double part2 = (Dmax - Dmin) / (12.0 * 2426.2 * pow(factor, -0.985));
    double Length = (part1 - part2) * rotations;
    return Length;    
}
int main()
{
    gpio_init(14);
    gpio_set_dir(14,GPIO_IN); //signialA

    gpio_init(15);
    gpio_set_dir(15, GPIO_IN); //signialB

    int counter=0;
    float rotations=0;
    float Length=0;
    double Dmax=3.685;
    double Dmin=2.00;
        
        if (gpio(14,1) && gpio(15,0));      //if pin14 is high and pin 15 is low
                counter++;                  //increment count
        if (gpio(15,1) && gpio(14,0));      //if pin15 is high and pin 14 is low
                --counter;                  //decrement count
        if (counter==48);
                counter==0;
                rotations++;
        if (counter==-48);
                counter==0;
                --rotations;
        
        double Length = calculate_Length(Dmax, Dmin, rotations);
        printf("Calculated Length: %.6f\n", Length);
        
        return 0;
    
}

///Length=(((Dmax*PI)/12)-((Dmax-Dmin)/(12*2426.2*((rotations/.9231)^(5000/5119))^(-.985))))*rotations;