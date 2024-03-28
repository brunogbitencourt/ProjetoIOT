#pragma once
class DigitalLevelSensor
{
    private:
        int digitalPin;
        bool levelState; 

    public:
        DigitalLevelSensor(int digitalPin);
        ~DigitalLevelSensor();
        int getDigitalPin();
        bool getLevelState();
};