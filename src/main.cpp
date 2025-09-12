// Sketch para STM32 BluePill - LoRa Sender
// PlatformIO - STM32duino
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

int counter = 0;
int lora_startup_counter = 0;     // Counter to check if LoRa chip started communication propperly
long readingID = 0;               // Sending packet N°
#define sensor_id "Sensor_02"           // <<=== Sensor identification  ==>> CHANGE HERE!!
#define sensor_location "Bridge_01"     // <<=== Sensor location        ==>> CHANGE HERE!!
//define the pins used by the LoRa transceiver module
  #define SCK           PA5
  #define MISO          PA6
  #define MOSI          PA7
  #define SS            PA4
  #define RST           PA0
  #define DIO0          PA1
  const int csPin =     PA4;         // LoRa radio chip select
  const int resetPin =  PA0;         // LoRa radio reset
  const int irqPin =    PA1;         // Change for your board; must be a hardware interrupt pin of the STM32 Bluepill

  // Define LoRa Communication Band:
  #define BAND 915E6  
  String LoRaMessage = "";          // String to store the LoRa Message that should be sent

  void onTxDone() {
    #ifdef enableSerialLog
      Serial.println("TxDone");
    #endif
    // LoRa_rxMode();
  }

    boolean runEvery(unsigned long interval)
  {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      return true;
    }
    return false;
  }

  void LoRa_txMode(){
    LoRa.idle();                          // set standby mode
    LoRa.disableInvertIQ();               // normal mode
  }
    void LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and send it
  }


 void start_LoRa(){
    
    //Setup receiver para receber o update da hora:
    #ifdef enableSerialLog
      Serial.println("LoRa Sender test");
      Serial.println();
    #endif

    // register the receive callback
    // LoRa.onReceive(onReceive); 
    LoRa.onTxDone(onTxDone);
    // LoRa_rxMode();
  } //end start_LoRa

  void sendReadings() {
    if (runEvery(5000)) { // repeat every 5 sec 
    //TODO se recebe confirmação de recebimento do gateway, não pode enviar mais para economizar bateria ver email: Checagem de Retorno de mensagem LoRa

      //TODO: Do I know it the receiver received the LoRa message? how?
      LoRaMessage = String(sensor_id) + "/" + String(counter) + "&" + String(counter);

      //Send LoRa packet to receiver
      LoRa_sendMessage(LoRaMessage); // send a LoRaMessage

      #ifdef enableSerialLog
        Serial.print("Sending packet N°: ");   Serial.println(readingID);
        Serial.print("LoRaMessage: ");   Serial.println(LoRaMessage);
      #endif

      readingID++;
    }
  }

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(csPin, resetPin, irqPin);    //SPI LoRa pins

    while (!LoRa.begin(BAND) && lora_startup_counter < 10) {
      Serial.print(".");
      lora_startup_counter++;
      delay(500);
    }
    if (lora_startup_counter == 10) {
      Serial.println("LoRa initialization Failed!"); 
      delay (100);
    }
        if (lora_startup_counter < 10) {
          #ifdef enableSerialLog
            Serial.println("LoRa initialization OK!"); 
          #endif
    }

}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRaMessage = String(sensor_id) + "/" + String(counter) + "&" + String(counter);
  LoRa.beginPacket();
  // LoRa.print("hello ");
  LoRa.print(LoRaMessage);
  // LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}