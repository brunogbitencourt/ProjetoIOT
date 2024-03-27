#include <Arduino.h>
#include "USSensor.h"

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

USSensor::USSensor(int inputPin, int pwmOutputPin)
{
    this->echoPin = echoPin;
    this->triggerPin = triggerPin;

    pinMode(this->echoPin,  INPUT);
    pinMode(this->triggerPin,  OUTPUT);

}

USSensor::~USSensor()
{
}


int USSensor::getEchoPin()
{
    return this->echoPin;
}

int USSensor::getTriggerPin()
{
    return this->triggerPin;
}


float USSensor::getDistance()
{
    long duration;

    digitalWrite(triggerPin, LOW);
    // vTaskDelay(pdMS_TO_TICKS(0.02));
    delayMicroseconds(2);
    // Sets the triggerPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    // vTaskDelay(pdMS_TO_TICKS(0.01));
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    distance = duration * SOUND_SPEED/2;
    Serial.println(duration);

    return this->distance;
}


void USSensor::setDistance()
{
    // long duration;

    // digitalWrite(triggerPin, LOW);
    // vTaskDelay(pdMS_TO_TICKS(0.02));
    // // delayMicroseconds(2);
    // // Sets the triggerPin on HIGH state for 10 micro seconds
    // digitalWrite(triggerPin, HIGH);
    // vTaskDelay(pdMS_TO_TICKS(0.01));
    // // delayMicroseconds(10);
    // digitalWrite(triggerPin, LOW);

    // distance = duration * SOUND_SPEED/2;

    // duration = pulseIn(echoPin, HIGH);
}


