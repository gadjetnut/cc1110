
#include "../../lib/delay.h"
#include "../../lib/cc1110.h"
#include "../../lib/cc1110-ext.h"


void main(void){
    /* Initialize P1_3 for LED output */
    P1SEL &= ~(BIT3);
    P1_3 = 1;
    P1DIR |= (BIT3);
    
    while (1){
               
       //1 second delay using milli-second delay routine
        delayms(1000);
        P1_3 ^= 1;  // Toggle LED

    }
}