/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MMA8451Q.h"

#define REG_WHO_AM_I      0x0D
#define REG_CTRL_REG_1    0x2A
#define REG_OUT_X_MSB     0x01
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Z_MSB     0x05
#define REG_FF_MT_CFG     0x15 // Freefall/Motion function block configuration
#define REG_FF_MT_SRC     0x16 // Freefall/Motion event source register
#define REG_FF_MT_THS     0x17 // Freefall/Motion threshold register
#define REG_FF_MT_CNT     0x18 // Freefall/Motion debounce counter
#define REG_CTRL_REG_4    0x2D
#define REG_CTRL_REG_5    0x2E
#define REG_INT_SRC       0x0C
#define UINT14_MAX        16383

void (*obsluha)(void);

//
InterruptIn MMA8451Q_Int1(PTA14);





MMA8451Q::MMA8451Q(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr)
{
    // activate the peripheral
    uint8_t data[2] = {REG_CTRL_REG_1, 0x01};
    writeRegs(data, 2);
}

MMA8451Q::~MMA8451Q() { }

uint8_t MMA8451Q::getWhoAmI()
{
    uint8_t who_am_i = 0;
    readRegs(REG_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

float MMA8451Q::getAccX()
{
    return (float(getAccAxis(REG_OUT_X_MSB))/4096.0);
}

float MMA8451Q::getAccY()
{
    return (float(getAccAxis(REG_OUT_Y_MSB))/4096.0);
}

float MMA8451Q::getAccZ()
{
    return (float(getAccAxis(REG_OUT_Z_MSB))/4096.0);
}

void MMA8451Q::getAccAllAxis(float * res)
{
    res[0] = getAccX();
    res[1] = getAccY();
    res[2] = getAccZ();
}

int16_t MMA8451Q::getAccAxis(uint8_t addr)
{
    int16_t acc;
    uint8_t res[2];
    readRegs(addr, res, 2);

    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;

    return acc;
}

void MMA8451Q::readRegs(int addr, uint8_t * data, int len)
{
    char t[1] = {addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}

void MMA8451Q::writeRegs(uint8_t * data, int len)
{
    m_i2c.write(m_addr, (char *)data, len);
}




void MMA8451Q::freeFallDetection( void(*paObsluha)(void))
{

    //Put the device in Standby Mode: Register 0x2A CTRL_REG1
    unsigned char data[2] = {REG_CTRL_REG_1, 0x20};
    writeRegs(data,2);

    //Configuration Register set for Freefall Detection enabling “AND” condition, OAE = 0, Enabling X, Y, Z and the Latch
    data[0] = REG_FF_MT_CFG;
    data[1] = 0xB8;
    writeRegs(data,2);

    //Threshold Setting Value for the resulting acceleration < 0.2 Note: The step count is 0.063g/count 0.2g/0.063g = 3.17 counts //Round to 3 counts
    data[0] = REG_FF_MT_THS;
    data[1] = 0x03;
    writeRegs(data,2);

    //Set the debounce counter to eliminate false positive readings for 50Hz sample rate with a requirement of 120 ms timer, assuming Normal Mode. Note: 120 ms/20 ms (steps) = 6 counts
    data[0] = REG_FF_MT_CNT;
    data[1] = 0x04;
    writeRegs(data,2);

    //Enable Motion/Freefall Interrupt Function in the System (CTRL_REG4)
    data[0] = REG_CTRL_REG_4;
    data[1] = 0x04;
    writeRegs(data,2);

    //Route the Motion/Freefall Interrupt Function to INT1 hardware pin (CTRL_REG5)
    data[0] = REG_CTRL_REG_5;
    data[1] = 0x04;
    writeRegs(data,2);

    //Put the device in Active Mode, 50 Hz
    data[0] = REG_CTRL_REG_1;
    data[1] = 0x21;
    writeRegs(data,2);

    obsluha = paObsluha;
    MMA8451Q_Int1.fall( this, &MMA8451Q::Fall_IRQ);
}

void MMA8451Q::Fall_IRQ( void)
{
    unsigned char t;
    //Determine source of the interrupt by first reading the system interrupt
    readRegs( REG_INT_SRC, &t, 1);
    if ( (t & 0x04) == 0x04) {
        //Read the Motion/Freefall Function to clear the interrupt
        readRegs( REG_FF_MT_SRC, &t, 1);
        // Run the user supplied function
        obsluha();
    }
}








