#pragma once

#include <string>

using namespace std;


class Actuator{

    private:
        string id;
        string description;
        int type;         
        int outputPin;
        int state; 
        int pwmOutput; 
        


    public:
        Actuator(string id, string description, int typeW, int outputPin,  int state);
        ~Actuator();
        string getID();
        string getDescription();
        int getType();
        int getOutputPin();
        int getPwmOutput();
        int getState();
        void setState(int state);
        void setPwmOutput(int pwmOutput);
        void toggleActuator();


};