#pragma once
#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "Actuator.h"

using namespace std;

Actuator::Actuator(int id, string description, int type, int outputPin,  int state){
    this->id = id;
    this->description = description;
    this->type = type;    
    this->outputPin = outputPin;
    this->state = state;
    this->pwmOutput = 0;
}

Actuator::~Actuator(){
    //Destructor
}

int Actuator::getID(){
    return this->id;
}

string Actuator::getDescription(){
    return this->description;
}

int Actuator::getType(){
    return this->type;
}

int Actuator::getOutputPin(){    
    return this->outputPin;
}

int Actuator::getState(){
    return (MODE == 1) ?  this->state : random(0, 3);
}

int Actuator::getPwmOutput(){
    return (MODE ==1) ? this->pwmOutput : random(0, 255);
}

void Actuator::setState(int state){
    this->state = state;
}

void Actuator::setPwmOutput(int pwmOutput){        
    pwmOutput = (this->type == 2) ? 255 : pwmOutput;    
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
