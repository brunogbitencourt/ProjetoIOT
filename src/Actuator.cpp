#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "Actuator.h"

using namespace std;

Actuator::Actuator(string id, string description, int type, int outputPin,  int state){
    this->id = id;
    this->description = description;
    this->type = type;    
    this->outputPin = outputPin;
    this->state = state;
    this->pwmOutput = 0;
    pinMode(this->outputPin, OUTPUT);
}

Actuator::~Actuator(){
    //Destructor
}

string Actuator::getId(){
    return this->id;
}

string Actuator::getDescription(){
    string atuador[3] = {"Pump", "Motor", "Valve"};
    return this->description;
}

int Actuator::getType(){
    return this->type;    
}

int Actuator::getOutputPin(){    
    return this->outputPin;
}

int Actuator::getState(){
    return this->state;
}

int Actuator::getPwmOutput(){
    return this->pwmOutput;
}

void Actuator::setState(int state){
    this->state = state;
}

void Actuator::setPwmOutput(int pwmOutput){        
    pwmOutput = (this->type == 2 && pwmOutput != 0) ? 255 : pwmOutput;    
    analogWrite(this->outputPin, pwmOutput);
    this->pwmOutput = pwmOutput;
}

void Actuator::toggleActuator(){
    if(this->state == 0){
        this->state = 1;
        this->pwmOutput = 255;
    }else{
        this->state = 0;
        this->pwmOutput = 0;
    }
    analogWrite(this->outputPin, this->pwmOutput);
}
