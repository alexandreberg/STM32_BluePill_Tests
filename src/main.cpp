// Sketch para STM32 BluePill - Piscar LED PC13 com mensagens seriais
// PlatformIO - STM32duino
#include <Arduino.h>

const int ledPin = PC13;  // LED na placa BluePill
unsigned long previousMillis = 0;
const long interval = 500;  // intervalo de 500ms
bool ledState = false;

void setup() {
  // Inicializa o pino do LED como saída
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // LED desligado (lógica invertida)
  
  // Inicializa comunicação serial
  Serial.begin(115200);
  
  // Aguarda a inicialização do serial (opcional)
  delay(1000);
  
  Serial.println("Sistema iniciado - LED na PC13");
  Serial.println("Piscando a cada 500ms");
  Serial.println("========================");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Verifica se passou o intervalo de tempo
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Alterna o estado do LED
    ledState = !ledState;
    
    if (ledState) {
      digitalWrite(ledPin, LOW);   // LED ligado (lógica invertida)
      Serial.println("LED ON");
    } else {
      digitalWrite(ledPin, HIGH);  // LED desligado (lógica invertida)
      Serial.println("LED OFF");
    }
  }
  
}