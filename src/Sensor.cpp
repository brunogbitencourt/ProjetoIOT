#include "Sensor.h"
#include "GlobalConfig.h"

/**
 * Sensor type
 * type = 1 Sensor de nível analógico
 * type = 2 Sensor de nível digital
*/

Sensor::Sensor(const std::string& id, const std::string& description, int type, int outPutPin1, int outPutPin2)
    : id(id), description(description), type(type), outPutPin1(outPutPin1), outPutPin2(outPutPin2),
      analogValue(0.0), digitalValue(false), timeStamp(millis()) {
    if (outPutPin2 != -1) {
        pinMode(outPutPin1, OUTPUT);
    } else {
        pinMode(outPutPin1, INPUT);
        pinMode(outPutPin2, OUTPUT);
    }
}

Sensor::~Sensor() {}

std::string Sensor::getId() const {
    return id;
}

std::string Sensor::getDescription() const {
    return description;
}

int Sensor::getType() const {
    return type;
}

int Sensor::getOutPutPin1() const {
    return outPutPin1;
}

int Sensor::getOutPutPin2() const {
    return outPutPin2;
}

double Sensor::getAnalogValue() {
    if (MODE == 0) {
        return getSimulatedAnalogValue();
    } else {
        if (type == 1) {
            long duration;

            digitalWrite(outPutPin1, LOW);
            vTaskDelay(pdMS_TO_TICKS(2)); // Delay in milliseconds
            digitalWrite(outPutPin1, HIGH);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay in milliseconds
            digitalWrite(outPutPin1, LOW);

            duration = pulseIn(outPutPin2, HIGH);

            analogValue = TANK_HEIGHT - duration * SOUND_SPEED / 2.0;

            return analogValue;
        } else {
            return -1; // Return an invalid value for non-analog sensors
        }
    }
}

bool Sensor::getDigitalValue() {
    if (MODE == 0) {
        return getSimulatedDigitalValue();
    } else {
        if (type == 2) {
            return digitalRead(outPutPin1);
        } else {
            return false; // Return false for non-digital sensors
        }
    }
}

long Sensor::getTimeStamp() const {
    return timeStamp;
}

void Sensor::setAnalogValue(double value) {
    analogValue = value;
}

void Sensor::setDigitalValue(bool value) {
    digitalValue = value;
}

void Sensor::updateTimeStamp() {
    // timeStamp = millis();
    timeStamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

double Sensor::getSimulatedAnalogValue() {
    return random(ANALOG_MIN, ANALOG_MAX);
}

bool Sensor::getSimulatedDigitalValue() {
    return random(0, 2); // Returns 0 or 1
}
