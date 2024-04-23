#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "USSensor.h"
#include "DigitalLevelSensor.h"
#include "Actuator.h"

void pumpTask(void *pvParameters);

void usTask(void *pvParameters);

void valveTask(void *pvParameters);

void digitalLevelSensor(void *pvParameters);

void motorTask(void *pvParameters);

//-----------------------------Actuators--------------------------------

Actuator pump1(1, "Pump 1 - Extenal", PUMP1_PIN, 0, 0);
Actuator pump2(2, "Pump 2 - Tank 1 Internal", PUMP2_PIN, 0, 0);
Actuator pump3(3, "Pump 3 - Tank 2 External", PUMP3_PIN, 0, 0);

Actuator motor1(4, "Motor 1 - Tank 1", MOTOR1_PIN, 2, 0);
Actuator motor2(5, "Motor 2 - Tank 2", MOTOR2_PIN, 2, 0);

Actuator valve1(6, "Valve 1", VALVE1_PIN, 2, 0);
Actuator valve2(7, "Valve 2", VALVE2_PIN, 2, 0);

//-----------------------------Sensors--------------------------------




DigitalLevelSensor digitalSensor01(12);


void setup() {
 
  Serial.begin(115200);


  //xTaskCreate(pumpTask, "Pump Task", 1024, NULL, 1, NULL);
  
  //xTaskCreate(usTask, "Uss Task", 1024, NULL, 1, NULL);

  //xTaskCreate(digitalLevelSensor, "Digital Level Sensor", 1024, NULL, 1, NULL);

  xTaskCreate(valveTask, "Valve Task", 1024, NULL, 1, NULL);

  //xTaskCreate(motorTask, "Motor Task", 1024, NULL, 1, NULL);

  // vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:
}

/*
void motorTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        if (Serial.available() > 0) {
          char c = Serial.read();
          if (c == 'l') {
            //digitalWrite(23, HIGH);
            motor.turnOn(255);
            Serial.println("Ligou o motor");
            vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos
            //digitalWrite(23, LOW);
            motor.turnOff();
          } 
          else{
            Serial.println("Comando inválido");
          }
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Espera 5 segundos        

    }
}


void valveTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares

        // Le entrada serial se for s abre
        if (Serial.available() > 0) {
          char c = Serial.read();
          if (c == 'o') {
            valve1.setPwmOutput(255);
            Serial.println("Abriu a válvula");
          } else if (c == 'c') {
            valve.closeValve();
            Serial.println("Fechou a válvula");
          }
          else{
            Serial.println("Comando inválido");
          }
        }



        vTaskDelay(pdMS_TO_TICKS(200)); // Espera 5 segundos
    }
}


void digitalLevelSensor(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        // lw      
        if (digitalSensor01.getLevelState()){
          Serial.println("Nível Alto! Alerta!!!!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Espera 1 segundo
    }
}


void usTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        Serial.println("Uss Task");
        Serial.print("Distance cm: ");
        Serial.println(us_s1.getDistance());

        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 1 segundo
    }
}

void pumpTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        Serial.println("Pump Task");
        Serial.println("Ligando a bomba");
        pump.turnOn(255); // Ligar a bomba com saída PWM de 255 (100%)
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 1 segundo
        pump.turnOff(); // Desligar a bomba
        Serial.println("Desligando a bomba");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos
    }
  
}
*/