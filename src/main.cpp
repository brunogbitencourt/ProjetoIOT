#include <Arduino.h>
//#include <FreeRTOS.h>
#include "Pump.h"

// Defina os pinos de saída e de controle PWM
#define OUTPUT_PIN 2
#define PWM_OUTPUT3

void pumpTask(void *pvParameters);

Pump pump(OUTPUT_PIN, 0);

void setup() {
  // put your setup code here, to run once:


  xTaskCreate(pumpTask, "Pump Task", 1024, NULL, 1, NULL);

  vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:
}




void pumpTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        pump.turnOn(255); // Ligar a bomba com saída PWM de 255 (100%)
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 1 segundo
        pump.turnOff(); // Desligar a bomba
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos
    }
}