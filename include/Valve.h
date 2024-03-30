#pragma once

class Valve
{
    private:
        int digitalPin;
        bool valveState; 

    public:
        Valve(int digitalPin);
        ~Valve();
        int getDigitalPin();
        bool getValveState();
        void openValve();
        void closeValve();
        void toggleValve();
};