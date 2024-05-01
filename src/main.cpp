#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"

#include "Actuator.h"

using namespace std;

void actuatorTask(void *pvParameters);

//-----------------------------Actuators--------------------------------

Actuator pump1(1, "Pump 1 - Tank 1", 0, PUMP1_PIN, 0);
Actuator pump2(2, "Pump 2 - Tank 1", 0, PUMP2_PIN, 0);
Actuator pump3(3, "Pump 3 - Tank 2", 0, PUMP3_PIN, 0);

Actuator motor1(4, "Motor 1 - Tank 1", 1, MOTOR1_PIN, 0);
Actuator motor2(5, "Motor 2 - Tank 2", 1, MOTOR2_PIN, 0);

Actuator valve1(6, "Valve 1 - Tank 1", 2, VALVE1_PIN, 0);
Actuator valve2(7, "Valve 2 - Tank 2", 2, VALVE2_PIN, 0);


//-----------------------------Sensors--------------------------------  

void setup() {
 
  Serial.begin(115200);

  Serial.println("Iniciando o sistema");

  xTaskCreate(actuatorTask, "Pump 01 Task", 4096, NULL, 1, NULL);
}
  /*xTaskCreate(actuatorTask, "Pump 02 Task",  4096, &pump2, 1, NULL);
  xTaskCreate(actuatorTask, "Pump 03 Task",  4096, &pump3, 1, NULL);

  xTaskCreate(actuatorTask, "Motor 01 Task", 4096, &motor1, 1, NULL);
  xTaskCreate(actuatorTask, "Motor 02 Task", 4096, &motor2, 1, NULL);

  xTaskCreate(actuatorTask, "Valve 01 Task", 4096, &valve1, 1, NULL);
  xTaskCreate(actuatorTask, "Valve 02 Task", 4096, &valve2, 1, NULL);
}*/
  // vTaskStartScheduler();}
void loop() {
  // put your main code here, to run repeatedly:
}


void actuatorTask(void *pvParameters){

     // concatena id com a letra a
  
  while(1) { 
    if (Serial.available() > 0) {      
      String arduinoString  = Serial.readString();
      string valSerial = string(arduinoString.c_str());

      if(valSerial == "a1l"){
        Serial.println("Pump 1 - Turn On");
        pump1.setPwmOutput(255);
      }      
      else if (valSerial == "a1d"){
        Serial.println("Pump 1 - Turn Off");
        pump1.setPwmOutput(0);        
      }
      else if (valSerial == "a2l"){
        Serial.println("Pump 2 - Turn On");
        pump2.setPwmOutput(255);
      }      
      else if (valSerial == "a2d"){
        Serial.println("Pump 2 - Turn Off");
        pump2.setPwmOutput(0);        
      }
      else if (valSerial == "a3l"){
        Serial.println("Pump 3 - Turn On");
        pump3.setPwmOutput(255);
      }      
      else if (valSerial == "a3d"){
        Serial.println("Pump 3 - Turn Off");
        pump3.setPwmOutput(0);        
      }
      else if (valSerial == "a4l"){
        Serial.println("Motor 1 - Turn On");
        motor1.setPwmOutput(255);
      }      
      else if (valSerial == "a4d"){
        Serial.println("Motor 1 - Turn Off");
        motor1.setPwmOutput(0);        
      }
      else if (valSerial == "a5l"){
        Serial.println("Motor 2 - Turn On");
        motor2.setPwmOutput(255);
      }      
      else if (valSerial == "a5d"){
        Serial.println("Motor 2 - Turn Off");
        motor2.setPwmOutput(0);        
      }
      else if (valSerial == "a6l"){
        Serial.println("Valve 1 - Turn On");
        valve1.setPwmOutput(255);
      }      
      else if (valSerial == "a6d"){
        Serial.println("Valve 1 - Turn Off");
        valve1.setPwmOutput(0);        
      }
      else if (valSerial == "a7l"){
        Serial.println("Valve 2 - Turn On");
        valve2.setPwmOutput(255);
      }      
      else if (valSerial == "a7d"){
        Serial.println("Valve 2 - Turn Off");
        valve2.setPwmOutput(0);
      }
      else{
        Serial.println("Invalid Command");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // Espera 5 segundos        
  }
}