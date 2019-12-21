#include "Elevator.h"
#include "Communication.h"
/*!
 * Posle packet s datami pre nastavenie aktualnej rychlosti vytahu smerom nahor.
 */
void UpSpeedMotor(){
    unsigned char packet[] = {0xA0,0xF1,0x00,0x05,0x02,0x00,0x00,0x00,0x64};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre nastavenie aktualnej rychlosti vytahu smerom nadol.
 */
void DownSpeedMotor(){   
    unsigned char packet[] = {0xA0,0xF1,0x00,0x05,0x02,0xff,0xff,0xff,0x9C};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre zastavenie vytahu.
 */
void Stop(){
    unsigned char packet[] = {0xA0,0xF1,0x00,0x05,0x02,0x00,0x00,0x00,0x00};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre zamknutie kabiny vytahu.
 */
void CabinLock(){
    unsigned char packet[] = {0xA0,0xF0,0x00,0x01,0x01};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre odomknutie kabiny vytahu.
 */
void CabinUnlock(){
    unsigned char packet[] = {0xA0,0xF0,0x00,0x01,0x00};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre ovladanie smerovej sipky displeja vytahu. Smer nahor.
 */
void DisplayUP(){
    unsigned char packet[] = {0xA0,0x30,0x00,0x01,0x01};
    unsigned char packet2[] = {0xA0,0x30,0x00,0x02,0x01,0x02};

    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
    sendPacket(packet2,(sizeof(packet2)/sizeof(packet2[0])));

}
/*!
 * Posle packet s datami pre ovladanie smerovej sipky displeja vytahu. Smer nadol.
 */
void displayDown(){
    unsigned char packet[] = {0xA0,0x30,0x00,0x01,0x02};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}


/*!
 * Posle packet s datami pre nastavenie ciselnej hodnoty poschodia na displeji vytahu.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void displayValue(int floor){
     
     unsigned char value;
      switch(floor){
    
        case 0: value = 0x50;break; 
        case 1: value = 0x31;break;
        case 2: value = 0x32;break;
        case 3: value = 0x33;break;
        case 4: value = 0x34;break;
    
    }
    
    unsigned char packet[] = {0xA0,0x30,0x00,0x02,0x03,value};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}


/*!
 * Posle packet s datami pre nastavenie ciselnej hodnoty poschodia na displeji vytahu a smerovej sipky nahor.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void displayValueUp(int floor){
     
     unsigned char value;
      switch(floor){
    
        case 0: value = 0x50;break; 
        case 1: value = 0x31;break;
        case 2: value = 0x32;break;
        case 3: value = 0x33;break;
        case 4: value = 0x34;break;
    
    }
    
    unsigned char packet[] = {0xA0,0x30,0x00,0x02,0x01,value};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}
/*!
 * Posle packet s datami pre nastavenie ciselnej hodnoty poschodia na displeji vytahu a smerovej sipky nadol.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void displayValueDown(int floor){
     
     unsigned char value;
      switch(floor){
    
        case 0: value = 0x50;break; 
        case 1: value = 0x31;break;
        case 2: value = 0x32;break;
        case 3: value = 0x33;break;
        case 4: value = 0x34;break;
    
    }
    
    unsigned char packet[] = {0xA0,0x30,0x00,0x02,0x02,value};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
}

/*!
 * Posle packet s datami pre zapnutie svetla na danom poschodi.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void ledOutOn(int floor){
    unsigned char address;
    switch(floor){
    
        case 0: address = 0x10;break; 
        case 1: address = 0x11;break;
        case 2: address = 0x12;break;
        case 3: address = 0x13;break;
        case 4: address = 0x14;break;
    
    }
    unsigned char packet[] = {0xA0,address,0x00,0x01,0x01};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));

}

/*!
 * Posle packet s datami pre vypnutie svetla na danom poschodi.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void ledOutOff(int floor){
    unsigned char address;
    switch(floor){
    
        case 0: address = 0x10;break; 
        case 1: address = 0x11;break;
        case 2: address = 0x12;break;
        case 3: address = 0x13;break;
        case 4: address = 0x14;break;
    
    }
    unsigned char packet[] = {0xA0,address,0x00,0x01,0x00};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
    
}
/*!
 * Posle packet s datami pre zapnutie svetla na danom poschodi v kabine.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void ledInOn(int floor){
    unsigned char address;
    switch(floor){
    
        case 0: address = 0x20;break; 
        case 1: address = 0x21;break;
        case 2: address = 0x22;break;
        case 3: address = 0x23;break;
        case 4: address = 0x24;break;
    
    }
    unsigned char packet[] = {0xA0,address,0x00,0x01,0x01};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
    
}
/*!
 * Posle packet s datami pre zapnutie svetla na danom poschodi v kabine.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void ledInOff(int floor){
    unsigned char address;
    switch(floor){
    
        case 0: address = 0x20;break; 
        case 1: address = 0x21;break;
        case 2: address = 0x22;break;
        case 3: address = 0x23;break;
        case 4: address = 0x24;break;
    
    }
    unsigned char packet[] = {0xA0,address,0x00,0x01,0x00};
    sendPacket(packet,(sizeof(packet)/sizeof(packet[0])));
    
}

/*!
 * Posle packet s datami pre zapnutie svetla na danom poschodi vonku aj v kabine.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void setLeds(int floor){
    ledInOn(floor);
    ledOutOn(floor);
}
/*!
 * Posle packet s datami pre vypnutie svetla na danom poschodi vonku aj v kabine.
 * @param floor Cislo poschodia. Iba celociselne hodnoty 0,1,2,3,4.
 */
void disableLeds(int floor){
    ledInOff(floor);
    ledOutOff(floor);    
}


    




