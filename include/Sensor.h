#pragma once

#include <Arduino.h>
#include <string>

using namespace std;

class Sensor {
public:
    Sensor(int id, string description, int type, int outPutPin1, int outPutPin2);
    ~Sensor();

    int getId();
    string getDescription();
    int getType();
    int getOutPutPin1();
    int getOutPutPin2();
    double getAnalogValue();
    bool getDigitalValue();
    long getTimeStamp();

    void setAnalogValue(double value);
    void setDigitalValue(bool value);
    void updateTimeStamp();
    double getSimulatedAnalogValue();
    bool getSimulatedDigitalValue();

private:
    int id;
    string description;
    int type;
    int outPutPin1;
    int outPutPin2;
    double analogValue;
    bool digitalValue;
    long timeStamp;  // Timestamp in milliseconds
};