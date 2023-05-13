/*
 * MAINNNNN.C
 *
 *  Created on: May 5, 2023
 *      Author: mostafa
 */


#include "C:\Users\mosta\Desktop\ccs\tm4c123gh6pm.h"
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "C:\ti\TivaWare_C_Series-2.2.0.295\driverlib\sysctl.h"
#include "C:\ti\TivaWare_C_Series-2.2.0.295\driverlib\gpio.h"
#include "C:\ti\TivaWare_C_Series-2.2.0.295\driverlib\uart.h"
#define PI 3.14159265359
char lon [20]={0};
char lat [20]={0};
double destLat   = 0 ; //Enter your destination latitude
double destLon   = 0 ; //Enter your destination longitude
double y=0;

//-----------------------------------------------------------------------------------------------
void PortB_init(void){
SYSCTL_RCGCGPIO_R|=0X02;
while((SYSCTL_PRGPIO_R&0x02)==0);
GPIO_PORTB_LOCK_R= GPIO_LOCK_KEY;
GPIO_PORTB_CR_R|=0x01;
GPIO_PORTB_DIR_R |= 0x01; // set PB0 as output
GPIO_PORTB_DEN_R |= 0x01; // enable digital function for PB0
GPIO_PORTB_AFSEL_R &= ~0x01; // disable alternate function for PB0
GPIO_PORTB_AMSEL_R &= ~0x01; // disable analog function for PB0
GPIO_PORTB_PCTL_R &= ~0x0000000F; // configure PB0 as GPIO
}

//-----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
void SysTick_Init(void){
 NVIC_ST_CTRL_R = 0; // disable SysTick during setup
 NVIC_ST_CTRL_R = 0x00000005; // enable SysTick with core clock
}
//----------------------------------------------------------------------------------------------------
void SysTick_Wait1s(void){
 NVIC_ST_RELOAD_R = 16000000-1; // wait 1s as 16000000*62.5ns equals 1s
 NVIC_ST_CURRENT_R = 0; // any value written to CURRENT clears
 while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
    }
}
// clock rate = 16MHZ
// 16000000*62.5ns equals 1s
void delay(unsigned long n){ // n is the number of times that call SysTick_wait100ms so to wait 1s
 unsigned long i;
 for(i=0; i<n; i++){
 SysTick_Wait1s(); // wait 1s
    }
}
//-----------------------------------------------------------------------------------------
double Degconv (double val)
{
return (int)(val / 100) + (val - (int) (val /
100) * 100) / 60.0;}
//------------------------------------------------------------------------------------------
double deg2rad(float deg)
{

    return deg * (PI/180);
}

double CalcDist(double lat1,double lon1,double lat2,double lon2)
{
        int R = 6371;// Radius of the earth in km
        double dLat = deg2rad(lat2-lat1);// degree to radian
        double dLon = deg2rad(lon2-lon1);// degree to radian
        double a =
        sin(dLat/2) * sin(dLat/2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dLon/2) * sin(dLon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        double d = R*c*1000;//Distance in m
        return d;
}
//--------------------------------------------------------------------------------------

void led_output(double x){
    if (x>8){
        GPIO_PORTF_DATA_R=0x02; //The built-in LED will be turned on(red)when the target destination is far away by distance > 8 meters
    }

    else if(x>4&&x<=8){


        GPIO_PORTF_DATA_R=0x0A; //LED will be turned on(yellow) when the target destination is far away by distance < 8 meters and >4
    }
    else{
            GPIO_PORTF_DATA_R=0x08; //The built-in LED will be turned on(yellow) when the target destination is about to be reached < 4 meters.
    }
}

//--------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

void PortF_init(void){
  SYSCTL_RCGCGPIO_R|=0X20;
    while((SYSCTL_PRGPIO_R&0x20)==0);
    GPIO_PORTF_LOCK_R= GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R|=0x1F;                // Allow changes to PortF
    GPIO_PORTF_AFSEL_R &= ~0X1F;          // Disable alternate functions
    GPIO_PORTF_PCTL_R &= ~0XFFFFFFFF;     // Set pins to I/O
    GPIO_PORTF_AMSEL_R &= ~0X1F;          // Disable analog
    GPIO_PORTF_DIR_R |= 0X0E;               // Make LEDs output
    GPIO_PORTF_DEN_R |= 0X1F;             // Enable digital for all PortF pins
    GPIO_PORTF_DATA_R &= ~0x0E;             // Make LEDs off

}
//---------------------------------------------------------------------------------
void UART5_Init(void) {
    SYSCTL_RCGCUART_R |= 0x20;
    SYSCTL_RCGCGPIO_R |= 0x10;

    // Wait for GPIOE peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}

    // Configure PE4 and PE5 as UART3 pins
    GPIO_PORTE_AFSEL_R |= 0x30;
    GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0x00FFFF) | 0x110000;
    GPIO_PORTE_DEN_R |= 0x30;
    GPIO_PORTE_AMSEL_R &= ~0x30;
    UART5_CTL_R &= ~(0x0001);
    UART5_IBRD_R = 104;
    UART5_FBRD_R = 11;
    UART5_LCRH_R = 0x0070;
    UART5_CC_R=0;
    UART5_CTL_R |= 0x0301;
}


//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
int main(){
UART5_Init();
PortF_init();
SysTick_Init();
PortB_init();
while(1){
    char flag;
    char logname[] = "$GPRMC";
    char b;
    char nema[80]={0};
    char i=0;
    char n=0,m=0;
    char flag1=0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    do{
        flag = 1;
        for(i=0;i<6;i++){
              while ((UART5_FR_R&0x10)!=0); //RXFE
              b = (char)(UART5_DR_R&0xff);
              if(b!=logname[i]){
                flag=0;
                break;}
        }
    }while(flag==0);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    i=0;
    char c=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    do{

         while ((UART5_FR_R&0x10)!=0); //RXFE
         nema[i] = (char)(UART5_DR_R&0xff);
         i++;

    }while(nema[i]!='*');

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (i=0 ; i<80 ; i++)
                   {
                       if(nema[i]==',')
                       {
                           c++;
                       }

                       if (c==2)
                       {i++;
                           if (nema[i] == 'A'){
                           continue;}

                           else
                               break ;

                       }

                       if ((c==3))
                       {i+=1;
                           for (;nema[i]!=',';i++)
                           {
                               lat[n]=nema[i];
                               n+=1;
                           }
                           i-=1;
                       }

                       if ((c==5))
                       {i+=1;
                           for (;nema[i]!=',';i++)
                           {
                               lon[m]=nema[i];
                               m+=1;
                           }
                           break;

                       }

                   }




double latitude  = Degconv(atof(lat));
double longitude = Degconv(atof(lon));
double y = CalcDist(destLat,destLon,latitude,longitude);
led_output(y);
if (y>4 && y<=8){
    GPIO_PORTB_DATA_R ^=0x01;
    delay(1);
    GPIO_PORTB_DATA_R ^=0x01;
}
if (y<4 && flag1==0){
    GPIO_PORTB_DATA_R ^=0x01;
    delay(3);
    GPIO_PORTB_DATA_R ^=0x01;
    flag1=1;
}




    }
}
