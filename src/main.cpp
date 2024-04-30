#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"

#include "Actuator.h"

using namespace std;

void actuatorTask(void *pvParameters);

//-----------------------------Actuators--------------------------------

Actuator pump1(1, "Pump 1 - Extenal", PUMP1_PIN, 0, 0);
Actuator pump2(2, "Pump 2 - Tank 1 Internal", PUMP2_PIN, 0, 0);
Actuator pump3(3, "Pump 3 - Tank 2 External", PUMP3_PIN, 0, 0);

Actuator motor1(4, "Motor 1 - Tank 1", MOTOR1_PIN, 2, 0);
Actuator motor2(5, "Motor 2 - Tank 2", MOTOR2_PIN, 2, 0);

Actuator valve1(6, "Valve 1", VALVE1_PIN, 2, 0);
Actuator valve2(7, "Valve 2", VALVE2_PIN, 2, 0);

//-----------------------------Sensors--------------------------------  

void setup() {
 
  Serial.begin(115200);

  xTaskCreate(actuatorTask, "Pump 01 Task", 1024, (void *)&pump1, 1, NULL);
  xTaskCreate(actuatorTask, "Pump 02 Task", 1024, (void *)&pump2, 1, NULL);
  xTaskCreate(actuatorTask, "Pump 03 Task", 1024, (void *)&pump3, 1, NULL);

  xTaskCreate(actuatorTask, "Motor 01 Task", 1024, (void *)&motor1, 1, NULL);
  xTaskCreate(actuatorTask, "Motor 02 Task", 1024, (void *)&motor2, 1, NULL);

  xTaskCreate(actuatorTask, "Valve 01 Task", 1024, (void *)&valve1, 1, NULL);
  xTaskCreate(actuatorTask, "Valve 02 Task", 1024, (void *)&valve2, 1, NULL);
 
  // vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}


void actuatorTask(void *pvParameters){

  Actuator *actuator = (Actuator *)pvParameters;

    // concatena id com a letra a
    string valor="a"+to_string(actuator->getID());
  
  while(1) { 
    if (Serial.available() > 0) {
      String arduinoString  = Serial.readString();
      string valSerial = string(arduinoString.c_str());

      if(valSerial == (valor+"l")){
        Serial.println(("Ligou o atuador "+valor).c_str());
        actuator->setPwmOutput(255);        
      } 
      else if(valSerial == (valor+"d")){
        Serial.println(("Desligou o atuador "+valor).c_str());
        actuator->setPwmOutput(0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Espera 5 segundos        
  }
}