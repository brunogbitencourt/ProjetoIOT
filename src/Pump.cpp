#include <Arduino.h>
#include "Pump.h"

Pump::Pump(int inputPin, int pwmOutputPin)
{
    this->outputPin = outputPin;
    this->pwmOutput = pwmOutput;

    pinMode(this->outputPin,  OUTPUT);

}

Pump::~Pump()
{
}


int Pump::getInputPin()
{
    return this->outputPin;
}

void Pump::turnOn(int pwmOutput)
{
    this->pwmOutput = pwmOutput;
    digitalWrite(this->outputPin, this->pwmOutput);
}

void Pump::turnOff()
{
    this->pwmOutput = 0;
    digitalWrite(this->outputPin, LOW);
}

int Pump::getPwmOutput()
{
    return this->pwmOutput;
}


