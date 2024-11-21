#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

double calculate_length(double Dmax, double Dmin, double rotations)
{
    double PI = 3.14159265358979323846;
    double part1 = (Dmax * PI) / 12.0;
    double factor = pow((rotations / 0.9231), (5000.0 / 5119.0));
    double part2 = (Dmax - Dmin) / (12.0 * 2426.2 * pow(factor, -0.985));
    double length = (part1 - part2) * rotations;
    return length;    
}
int main()
{
    gpio_init(8);
    gpio_set_dir(8,GPIO_IN); //signialA

    gpio_init(9);
    gpio_set_dir(9, GPIO_IN); //signialB

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT); //text

    int counter=0;
    double rotations=0;
    int channelA =0;
    int channelB =0;
    double Dmax=3.685;
    double Dmin=2.00;
   
while(true)
{
    int counter=0;
    channelA=gpio_get(8);
    channelB=gpio_get(9);
        if (channelA ==1 ||channelB==0)
        {     //if pin14 is high and pin 15 is low
        //counter++;                  //increment count
            gpio_put(22,1);
            counter++;
        }
        else if (channelA ==0 || channelB==1)
        {
            gpio_put(22,0);
            --counter;
        }
        else if (channelA ==0 || channelB==0)
        {
            gpio_put(22,0);
            
        }
        else if (channelA ==1 || channelB==1)
        {
            gpio_put(22,0);
            
        }
    }
    printf("counter:" , counter);
}
    
    //if (gpio_get(15) && gpio(14,0));      //if pin15 is high and pin 14 is low
    //    --counter;                  //decrement count
    //if (counter==48);
     //   counter==0;
     //   rotations++;
    //if (counter==-48);
    //    counter==0;
    //    --rotations;
        
    //double length = calculate_length(Dmax, Dmin, rotations);
    //printf("Calculated length: %.6f\n", length);
        

    


///Length=(((Dmax*PI)/12)-((Dmax-Dmin)/(12*2426.2*((rotations/.9231)^(5000/5119))^(-.985))))*rotations;