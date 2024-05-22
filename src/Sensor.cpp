#include "Sensor.h"
#include "GlobalConfig.h"

/**
 * Sensor type
 * type = 1 Sensor de nível analogico
 * type = 2 Sensor de nível digital
*/

Sensor::Sensor(int id, string description, int type, int outPutPin1, int outPutPin2)
{
    this->id = id;
    this->description = description;
    this->type - type;
    this->outPutPin1 = outPutPin1;
    this->outPutPin2 = outPutPin2;

    if (outPutPin2 != NULL)
    {
        pinMode(outPutPin1, OUTPUT);
    }
    else
    {
        pinMode(outPutPin1, INPUT);
        pinMode(outPutPin2, OUTPUT);
    }
}

Sensor::~Sensor() {}

int Sensor::getId()
{
    return id;
}

string Sensor::getDescription()
{
    return description;
}

int Sensor::getType()
{
    return type;
}

int Sensor::getOutPutPin1()
{
    return outPutPin1;
}

int Sensor::getOutPutPin2()
{
    return outPutPin2;
}

double Sensor::getAnalogValue()
{
    if (MODE == 0)
    {
        return getSimulatedAnalogValue();
    }
    else
    {
        if (type == 1)
        {
            long duration;

            digitalWrite(outPutPin1, LOW);
            vTaskDelay(pdMS_TO_TICKS(0.002));
            digitalWrite(outPutPin1, HIGH);
            vTaskDelay(pdMS_TO_TICKS(0.01));
            digitalWrite(outPutPin1, LOW);

            duration = pulseIn(outPutPin2, HIGH);

            analogValue = duration * SOUND_SPEED / 2;

            return this->analogValue;
        } else {
            return NULL;
        }
    }
}

bool Sensor::getDigitalValue()
{
    if (MODE == 0)
    {
        return getSimulatedDigitalValue();
    }
    else
    {
        if (type == 2)
        {
            return digitalRead(this->outPutPin1);
        } else {
            return NULL;
        }
    }
}

long Sensor::getTimeStamp()
{
    return timeStamp;
}

void Sensor::setAnalogValue(double value)
{
    analogValue = value;
}

void Sensor::setDigitalValue(bool value)
{
    digitalValue = value;
}

void Sensor::updateTimeStamp()
{
    timeStamp = millis();
}

double Sensor::getSimulatedAnalogValue()
{
    return random(ANALOG_MIN, ANALOG_MAX);
}

bool Sensor::getSimulatedDigitalValue()
{
    return random(0, 2); // Returns 0 or 1
}