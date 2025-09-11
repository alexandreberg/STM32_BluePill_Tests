// Sketch para STM32 BluePill - Blink + Deep Sleep - Piscar LED PC13 com mensagens seriais e DeepSleep por 1 minuto
// PlatformIO - STM32duino
#include <Arduino.h>
#include <STM32LowPower.h>  // Biblioteca LowPower

const int ledPin = PC13;  // LED na placa BluePill
unsigned long previousMillis = 0;
const long interval = 500;  // intervalo de 500ms
bool ledState = false;
#define sleep_time 60

void setup() {
  // Inicializa o pino do LED como saída
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // LED desligado (lógica invertida)
  
  // Inicializa comunicação serial
  Serial.begin(115200);
  
  // Aguarda a inicialização do serial (opcional)
  delay(1000);
  
  Serial.println("");
  Serial.println("Sistema iniciado  - Blink + DeepSleep - LED na PC13");
  Serial.println("Piscando a cada " + String(interval) + " ms");
  Serial.println("========================");

  // Pisca 5 vezes antes de dormir
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, LOW);   // LED ON
    Serial.println("LED ON");
    delay(200);
    digitalWrite(ledPin, HIGH);  // LED OFF
    Serial.println("LED OFF");
    delay(200);
  }

  // Inicializa a lib LowPower
  LowPower.begin();

  // Entra em Shutdown Mode por 60 segundos
  Serial.println("Entrando em Deep Sleep por " + String(sleep_time) + " s");
  delay(500);

  LowPower.shutdown(1000 * sleep_time);

  // Quando acordar, o sistema reinicia do setup()
}

void loop() {
  // Nunca deve chegar aqui porque após acordar do shutdown,
  // o sistema reinicia e começa novamente pelo setup().
}