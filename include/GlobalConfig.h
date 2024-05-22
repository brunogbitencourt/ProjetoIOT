//#define MODE 0 // Generates random values for the sensors and actuators
#define MODE 0 // Read sensors


//--------------------------------------------------------------------------------//
//------------------------------Actuators Variables-------------------------------//
//--------------------------------------------------------------------------------//
#define PUMP1_PIN  22
#define PUMP2_PIN  26
#define PUMP3_PIN  22

#define MOTOR1_PIN 25
#define MOTOR2_PIN 33

#define VALVE1_PIN 23
#define VALVE2_PIN 19
//--------------------------------------------------------------------------------//
//------------------------------Sensors Variables---------------------------------//
//--------------------------------------------------------------------------------//
#define SENSOR_D1_PIN 12
#define SENSOR_D2_PIN 13
#define SENSOR_A1_EPIN 2
#define SENSOR_A1_TPIN 26
#define SENSOR_A2_EPIN 32
#define SENSOR_A2_TPIN 15



//--------------------------------------------------------------------------------//
//------------------------------Wifi Variables------------------------------------//
//--------------------------------------------------------------------------------//
//#define WIFI_SSID "LIVE TIM_07A0_2G"
//#define WIFI_PASS "Brunolucas"
//#define WIFI_SSID "Justweb2_wifi"
//#define WIFI_PASS "IOx20=30?"
#define WIFI_SSID "POCO M3"
#define WIFI_PASS "hbjnhbjn"

//--------------------------------------------------------------------------------//
//------------------------------MQTT Variables------------------------------------//
//--------------------------------------------------------------------------------//
// Mosquitto
#define MOSQUITTO_BROKER_HOST "test.mosquitto.org"
#define MOSQUITTO_BROKER_PORT 1883
#define MOSQUITTO_USERNAME "IoT_Mosquitto"

// Definições para o MQTT
#define BROKER_MQTT "test.mosquitto.org"
#define BROKER_PORT 1883
#define ID_MQTT "IoT_PUC_SG_mqtt1"

// Defina o intervalo de valores aleatórios
#define ANALOG_MIN 0
#define ANALOG_MAX 1023
#define DIGITAL_VALUES {0, 1}

#define SOUND_SPEED 0.034