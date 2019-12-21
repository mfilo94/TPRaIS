#include "mbed.h"
#include "Control.h"
#include "Communication.h"
#include "Elevator.h"
/*
 * Inicializacia RGB Led Diody. Cervena. Ak nastane emergency break, v dosledku padu kabiny, zasvieti.
 */
DigitalOut led_emergency(LED_RED);

/*!
 * Obsluha prerusenia Acc, ak je detegovany FreeFall.
 */
void rob(){
    unsigned char emergency[] = {0xA0,0xf,0x00,0x01,0x01};
    sendPacket(emergency,(sizeof(emergency)/sizeof(emergency[0])));
    led_emergency = !led_emergency;

}
/*!
 * Inicializacia Acc MMA8451Q na freeFall detekciu.
 */
void initAccele(){
    
    MMA8451Q acce(SDA, SCL, MMA8451_I2C_ADDRESS);    
    acce.freeFallDetection(&rob);
}

/*!
 * Hlavna funkcia programu. Inicializacia komunikacie, inicializacia riadenia vytahu, inicializacia Acc. Nastavenie priority timerov.
 * Nekonecna slucka.
 */
int main()
{   

    led_emergency = 1; //zhasni cervenu led
    NVIC_SetPriority(LPTimer_IRQn,0);//priorita tickerov
    initCom();//inicializacia komunikacie
    initSys(); //inicializacia riadenia vytahu
    initAccele();//inicializacia Acc
    unsigned char emergencyClear[] = {0xA0,0xf,0x00,0x01,0x00};
    sendPacket(emergencyClear,(sizeof(emergencyClear)/sizeof(emergencyClear[0])));

    while(1){}
            
}








