#include "mbed.h"
#include "Elevator.h"
#include "Communication.h"
#include "Control.h"
#include "MMA8451Q.h"

/*!
 * Inicializacia RGB Led Diody - zelena farba. Ak vytah stoji, zelena dioda blika.
 */

DigitalOut led_free(LED_GREEN);
extern unsigned char address;
/*!
 * Premenna indukujuca aktualne poschodie, na ktorom sa vytah nachadza.
 */
volatile int actualFloor = 4;
/*!
 * Premenna indikujuca smer, v ktorom sa vytah aktualne pohybuje. 0 - stoji, 1 - smer nahor, -1 - smer nadol.
 */
volatile int direction = 0;
/*!
 * Pomocna premenna pre pozdrzanie spravania sa vytahu, ktora je navysovana kazdu sekundu. 
 */
volatile int waitVar = 0;
/*!
 * Pole priznakov poschodi. Ak false - vytah na danom poschodi nestoji, ak true - vytah na danom poschodi zastavi.
 */
volatile bool floors[5] = {false,false,false,false,false};
/*!
 * Ticker pre ovladanie posielania packetu pre watchDog.
 */
Ticker watchDg;
/*!
 * Ticker pre sledovanie adresy, z ktorej prisiel packet od programu.
 */
Ticker watchAddress;
/*!
 * Ticker pre sledovanie priznakov, ci ma vytah zastavit alebo nie.
 */
Ticker watchStop;
/*!
 * Ticker pre ovladanie vytahu, ked stoji.
 */
Ticker watchStopped;
/*!
 * Ticker pre informacny displej vytahu.
 */
Ticker watchDisp;

/*!
 * Obsluzna funkcia watchDg Tickeru. Periodicky posiela packet, ktory zabezpecuje reset watchDogu v programe ElevatorSim.
 */
void watchDog(){
    unsigned char wd[] = {0xA0,0xfe,0x00,0x01,0x05};
    sendPacket(wd,(sizeof(wd)/sizeof(wd[0])));    
}
/*!
 * Obsluzna funkcia watchDisp Tickeru. Periodicky posiela packet, ktory zabezpecuje zobrazenie aktualnych udajov na displeji v programe ElevatorSim.
 */
void watchDisplay(){  
 if (direction == -1){
         displayValueDown(actualFloor);
    }else if (direction == 1){
        displayValueUp(actualFloor);    
    }
    waitVar++;
}
/*!
 * Obsluzna funkcia watchAddress Tickeru. Ak je prijaty packet od tlacidiel alebo od switchu na poschodi, funkcia nastavi priznak floor na true, resp. nastavi premennu aktualneho poschodia.
 */
void readAddress(){
  
  switch(address){
     
     case 0xc4:floors[4] = true;ledOutOn(4);ledInOn(4);break;
     case 0xc3:floors[3] = true;ledOutOn(3);ledInOn(3);break;
     case 0xc2:floors[2] = true;ledOutOn(2);ledInOn(2);break;
     case 0xc1:floors[1] = true;ledOutOn(1);ledInOn(1);break;
     case 0xc0:floors[0] = true;ledOutOn(0);ledInOn(0);break;
     case 0xb4:floors[4] = true;ledOutOn(4);ledInOn(4);break;
     case 0xb3:floors[3] = true;ledOutOn(3);ledInOn(3);break;
     case 0xb2:floors[2] = true;ledOutOn(2);ledInOn(2);break;
     case 0xb1:floors[1] = true;ledOutOn(1);ledInOn(1);break;
     case 0xb0:floors[0] = true;ledOutOn(0);ledInOn(0);break;
     case 0xe0: actualFloor = 0; led_free=1; break;
     case 0xe1: actualFloor = 1;led_free=1;break;
     case 0xe2: actualFloor = 2;led_free=1;break;
     case 0xe3: actualFloor = 3;led_free=1;break;
     case 0xe4: actualFloor = 4;led_free=1;break;
     default: break;
     }
    }
 /*!
 * Obsluzna funkcia watchStop Tickeru. Periodicky sleduje, ci priznak poschodia, na ktorom sa momentalne vytah nachadza = true. Ak ano vytah zastavi.
 */
void stopCabin(){
   
    if (floors[actualFloor] == true) {
       floors[actualFloor] = false;
       direction = 0;
       Stop();
       CabinUnlock();
       displayValue(actualFloor);
       }
}
 /*!
 * Obsluzna funkcia watchStopeed Tickeru. Periodicky sleduje, ci vytah stoji. Ak ano, hlada najblizsii priznak poschodia true v smere jazdy. Ak nenajde zmeni hodnotu direction na opacnu a hlada znova.
 */
void elevatorStopped(){

    if (direction == 0){
        led_free = !led_free;
           if ((floors[actualFloor-1] == true)||(floors[actualFloor-2] == true) ||(floors[actualFloor-3] == true)||(floors[actualFloor-4] == true) ){//hladaj najblizsiu cestu smerom nadol
            if((waitVar % 5) == 0){ //cakaj
             ledOutOff(actualFloor);
             ledInOff(actualFloor);
             direction = -1;
             CabinLock();
             DownSpeedMotor();
            }                   
           }else if ((floors[actualFloor+1] == true)||(floors[actualFloor+2] == true) ||(floors[actualFloor+3] == true) ||(floors[actualFloor+4] == true)){    //hladaj najblizsiu cestu smerom nahor
             if((waitVar % 5) == 0){//cakaj
              ledOutOff(actualFloor);
              ledInOff(actualFloor);
              direction = 1;
              CabinLock();
              UpSpeedMotor();
             }
            }          
    }           

}


 /*!
 * Inicializacia riadenia vytahu. Na zaciatku vytah automaticky zide na 0 poschodie.
 */

void initSys(){
      
       led_free = 1; //zhasni led
       /*
       * Prirad obsluzne funkcie k Tickerom.
       */
       watchAddress.attach(&readAddress,0.1);
       watchStop.attach(&stopCabin,0.1);
       watchStopped.attach(&elevatorStopped,0.2);//zhasni led
       watchDg.attach(&watchDog,0.4);//zhasni led
       watchDisp.attach(&watchDisplay,1);//zhasni led
       
       
       
       CabinLock(); //zamkni kabinu
       DownSpeedMotor();//smer nadol
       wait(0.25);//pockaj
       floors[0] = true; //nastav priznak poschodia 0 na true
      
       ledOutOn(0); //zasviet led vonku na 0
       ledInOn(0);//zasviet led vnutri na 0
       direction = -1;
     
}
    
