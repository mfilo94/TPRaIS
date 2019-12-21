#include "Communication.h"
/*!
 * Inicializacia serioveho rozhrania.
 */

Serial pc(USBTX,USBRX,57600);
/*!
 * Pole prijatych dat cez seriove rozhranie.
 */
volatile unsigned char receivedData[256];
/*!
 * Premenna indikujuca v akom stave sa prijimanie dat nachadza.
 */
volatile int state = 0;
/*!
 * Premenna, v ktorej je ulozena aktualna hodnota adresy, z ktorej prisiel packet dat.
 */
volatile unsigned char address;
/*!
 * Premenna, v ktorej je ulozena dlzka prijatych dat.
 */
volatile unsigned char dataLength;

/*!
 * Funkcia, ktora vypocita hodnotu CRC8 pre dane pole dat.
 * @param data[] Pole dat z ktorych chceme vypocitat CRC8.
 * @param elements Velkost dat.
 * @return  unsigned data
 */
unsigned crc8(volatile unsigned char data[], size_t elements)
{
    
    
    static unsigned char crc8_tab[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};
    
    
    unsigned crc=0x00;
    for (int i=0; i<elements;i++){
                crc = crc8_tab[crc^data[i]];
    }
    
    return crc;
    
}

/*!
 * Pomocna funkcia, ktora vypocita CRC8 z prijatych dat.
 * @param packet[] Data, z ktorych chceme vypocitat CRC8.
 * @param elements Velkost dat.
 * @return  unsigned data
 */
unsigned doCrc(volatile unsigned char packet[], size_t elements){
    unsigned char crc[elements-3];
    crc[0] = packet[1];
    crc[1] = packet[2];
    
     for (int i=4; i<elements;i++){
        crc[i-2] = packet[i];    
    }
    
    return crc8(crc,elements-2);
}

/*!
 * Obsluzna funkcia prerusenia pre kazdy prijaty znak cez seriove rozhranie.
 * Funkcia kontroluje, ci prijate data maju spravnu formu. 
 * Funkcia porovnava prijate CRC8 s ocakavanym CRC8 prijatych dat.
 */
void readData(){
    if(pc.readable()) {
        if(pc.getc() == 0xA0) {
            receivedData[0] = 0xA0;
            if(pc.getc() == 0x00) {
                receivedData[1]= 0x00;
                address = pc.getc();
                receivedData[2] = address;
                receivedData[3] = pc.getc();
                for(int i = 4; i < 0x04 + receivedData[3]; i++) {
                    receivedData[i] = pc.getc();
                }
                if(pc.getc() == crc8(receivedData, receivedData[3] + 0x04)) {

                }
            }
        }
    }
}
/*!
 * Funkcia, ktora zabezpecuje poslanie packetu programu ElevatorSim.
 * @param packet[] Data na odoslanie.
 * @param elements Velkost dat na odoslanie.
 */
void sendPacket(volatile unsigned char packet[], size_t elements){    
        
    unsigned char crc[elements-3];
    crc[0] = packet[1];
    crc[1] = packet[2];
    
    for (int i=4; i<elements;i++){
        crc[i-2] = packet[i];    
    }
    
    for (int u=0; u<elements;u++){
     pc.putc(packet[u]);   
    }
    pc.putc(crc8(crc,elements-2));
    
    while (dataLength != 0x00) sendPacket(packet,elements); //riesenie ack

    
}
/*!
 * Inicializacna funkcia seriovej komunikacie medzi doskou KL-25Z a programom ElevatorSim.
 * Rychlost prenosu dat 57600 baud.
 * Priradenie funkcie k obsluhe prerusenia pri prijatom znaku cez seriove rozhranie.
 */
void initCom(){
    
    pc.baud(57600);
    pc.attach(&readData);


}





