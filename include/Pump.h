#pragma once
class Pump
{
    private:
        int outputPin;
        int pwmOutput; 

    public:
        Pump(int inputPin, int pwmOutputPin);
        ~Pump();
        int getInputPin();    
        void turnOn(int pwmOutput);
        void turnOff();
        int getPwmOutput(); 

};