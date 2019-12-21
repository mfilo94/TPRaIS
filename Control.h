#ifndef CONTROL_H
#define CONTROL_H

#include "MMA8451Q.h"

#if   defined (TARGET_KL25Z) || defined (TARGET_KL46Z)
  PinName const SDA = PTE25;
  PinName const SCL = PTE24;
#elif defined (TARGET_KL05Z)
  PinName const SDA = PTB4;
  PinName const SCL = PTB3;
#elif defined (TARGET_K20D50M)
  PinName const SDA = PTB1;
  PinName const SCL = PTB0;
#else
  #error TARGET NOT DEFINED
#endif

/*!
 * Definicia adresy Acc.
 */
#define MMA8451_I2C_ADDRESS (0x1d<<1)

void initSys();

#endif
