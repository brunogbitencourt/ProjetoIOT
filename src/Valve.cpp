#include <Arduino.h>
#include "Valve.h"

Valve::Valve(int digitalPin)
{
    this->digitalPin = digitalPin;
    pinMode(digitalPin, OUTPUT);
    closeValve();
}

Valve::~Valve()
{
}

int Valve::getDigitalPin()
{
    return this->digitalPin;
}

bool Valve::getValveState()
{
    return this->valveState;
}

void Valve::openValve()
{
    digitalWrite(digitalPin, HIGH);
    this->valveState = true;
}

void Valve::closeValve()
{
    digitalWrite(digitalPin, LOW);
    this->valveState = false;
}

void Valve::toggleValve()
{
    if (this->valveState)
    {
        closeValve();
    }
    else
    {
        openValve();
    }
}