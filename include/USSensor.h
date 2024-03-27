#pragma once
class USSensor
{
    private:
        int echoPin;
        int triggerPin; 
        float distance;

    public:
        USSensor(int echoPin, int triggerPin);
        ~USSensor();
        int getEchoPin();    
        int getTriggerPin();    
        float getDistance();
        void setDistance();
};