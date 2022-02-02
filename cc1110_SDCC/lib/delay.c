//various delay routines
//delay routines are dependant on clock speeds of you application
//other than clk_wait routine below that works with all clock speeds

#include "cc1110.h"
#include "cc1110-ext.h"
#include "clk_mgmt.h"

//Millisecond delay
void delayms(int msec){
  for (int x=0;x<msec;x++){ //msec
    for (int y=0;y<500;y++){ //usec
      }
  }
  return;
}

//Microsecond delay
void delayus(int usec){
  for (int x=0;x<usec/3;x++){ //usec
  }
  return;
}


void clk_wait(uint8_t wait){

   uint32_t largeWait;

   if(wait == 0)
   {return;}

   largeWait = ((uint16_t) (wait << 7));
   largeWait += 59*wait;

   largeWait = (largeWait >> CLKSPD);
   while(largeWait--);

   return;
}