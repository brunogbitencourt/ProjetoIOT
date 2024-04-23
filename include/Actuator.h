#pragma once

#include <string>

using namespace std;


class Actuator{

    private:
        int id;
        string description;
        int type;         
        int outputPin;
        int state; 
        int pwmOutput; 
        


    public:
        Actuator(int id, string description, int typeW, int outputPin,  int state);
        ~Actuator();
        int getID();
        string getDescription();
        int getType();
        int getOutputPin();
        int getPwmOutput();
        int getState();
        void setState(int state);
        void setPwmOutput(int pwmOutput);
        void toggleActuator();


};