#include <Arduino.h>
#include "DigitalLevelSensor.h"

DigitalLevelSensor::DigitalLevelSensor(int digitalPin)
{
    this->digitalPin = digitalPin;
    pinMode(digitalPin, INPUT);
}

DigitalLevelSensor::~DigitalLevelSensor()
{
}

int DigitalLevelSensor::getDigitalPin()
{
    return this->digitalPin;
}

bool DigitalLevelSensor::getLevelState()
{
    return digitalRead(this->digitalPin);
}

