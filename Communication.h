#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "mbed.h"


void readData();
void sendPacket(volatile unsigned char packet[], size_t elements);
void initCom();

#endif
