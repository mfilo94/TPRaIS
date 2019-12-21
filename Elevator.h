#ifndef ELEVATOR_H
#define ELEVATOR_H



    void UpSpeedMotor();
    void DownSpeedMotor();
    void Stop();
    void CabinLock();
    void CabinUnlock();
    void DisplayUP();
    void displayDown();
    void displayValue(int floor);
    void displayValueUp(int floor);
    void displayValueDown(int floor);
    void ledOutOn(int floor);
    void ledOutOff(int floor);
    void ledInOn(int floor);
    void ledInOff(int floor);
    void setLeds(int floor);
    void disableLeds(int floor);

    
    
#endif
