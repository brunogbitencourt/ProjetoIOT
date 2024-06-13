#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <string>

class Sensor {
public:
    Sensor(const std::string& id, const std::string& description, int type, int outPutPin1, int outPutPin2);
    ~Sensor();

    std::string getId() const;
    std::string getDescription() const;
    int getType() const;
    int getOutPutPin1() const;
    int getOutPutPin2() const;
    double getAnalogValue();
    bool getDigitalValue();
    long getTimeStamp() const;

    void setAnalogValue(double value);
    void setDigitalValue(bool value);
    void updateTimeStamp();
    double getSimulatedAnalogValue();
    bool getSimulatedDigitalValue();

private:
    std::string id;
    std::string description;
    int type;
    int outPutPin1;
    int outPutPin2;
    double analogValue;
    bool digitalValue;
    long timeStamp;
};

#endif // SENSOR_H
