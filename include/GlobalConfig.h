//#define MODE 0 // Generates random values for the sensors and actuators
#define MODE 1 // Read sensors


//--------------------------------------------------------------------------------//
//------------------------------Actuators Variables-------------------------------//
//--------------------------------------------------------------------------------//
#define PUMP1_PIN  18
#define PUMP2_PIN  32
#define PUMP3_PIN  22

#define MOTOR1_PIN 4
#define MOTOR2_PIN 33

#define VALVE1_PIN 23
#define VALVE2_PIN 19
//--------------------------------------------------------------------------------//
//------------------------------Sensors Variables---------------------------------//
//--------------------------------------------------------------------------------//
#define SENSOR_D1_PIN 16
#define SENSOR_D2_PIN 17
#define SENSOR_A1_EPIN 21
#define SENSOR_A1_TPIN 5
#define SENSOR_A2_EPIN 27
#define SENSOR_A2_TPIN 14



//--------------------------------------------------------------------------------//
//------------------------------Wifi Variables------------------------------------//
//--------------------------------------------------------------------------------//
#define WIFI_SSID "LIVE TIM_07A0_2G"
#define WIFI_PASS "Brunolucas"
//#define WIFI_SSID "Justweb2_wifi"
// #define WIFI_PASS "IOx20=30?"
// #define WIFI_SSID "POCO M3"
// #define WIFI_PASS "hbjnhbjn"

//#define WIFI_SSID "iPhone B"
//#define WIFI_PASS "bruno123"


//--------------------------------------------------------------------------------//
//------------------------------MQTT Variables------------------------------------//
//--------------------------------------------------------------------------------//
// Mosquitto
#define TOPIC_SENSORS "sensors/data"
#define TOPIC_ACTUATORS "actuators/data"
#define TOPIC_ACTIONS "actions"

// Definições para o MQTT
#define BROKER_MQTT "test.mosquitto.org"
#define BROKER_PORT 1883
#define ID_MQTT "IoT_PUC_SG_mqtt1"

// Defina o intervalo de valores aleatórios
#define ANALOG_MIN 0
#define ANALOG_MAX 1023
#define DIGITAL_VALUES {0, 1}

#define SOUND_SPEED 0.034
#define TANK_HEIGHT 11.9 // Height of tank cm

// Configuração NTP
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 0 // Ajuste conforme seu fuso horário
#define DAYLIGHT_OFFSET_SEC 0