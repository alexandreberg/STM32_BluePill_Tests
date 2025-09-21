/* Blue_Pill_Lora_Transmitter_with_RFM95_BPLTwR_23.09.2021-01
 * Código do sensor que está ativo na ponte pequena
 *
 * Alexandre Nuernbegr - alexandreberg@gmail.com
 *
 * Code available on: https://github.com/alexandreberg/SAPM_Sensor_BluePill
 *
 * BackupRegisters:
 * See https://community.st.com/t5/stm32-mcus/how-to-use-the-stm32-s-backup-registers/ta-p/49892
 *
 * Backup registers can be written/read and protected and have the option of being preserved in VBAT mode when the VDD domain is powered off.
 * The BackUp Registers are part of the RTC peripheral so we will need to enable the RTC to be able to access them.
 * The STM32 Blue Pill, which typically uses the STM32F103C8T6 microcontroller, has 10 backup registers.
 * Each register is 16 bits wide, providing a total of 20 bytes of data that can be stored in the backup domain.
 *
 * (!) To be able to preserve the backup registers through a power cycle, VBAT must remain powered when VDD is removed, this is called the VBAT mode.
 *
 * 23.11.2024 - Changing the code to read and hibernate for 1 minute between ultrasound readings.
 *             - 1min on and 1min off, adjust to not stay on for so long and turn off as soon as it transmits
 * 24.11.2024 - changing the readUltrasonic() function to work with the median
 *
 * 24.12.2024 - Cleaning and organizing the file
 *
 * TODO:
 * reactivate hibernation and make it sleep for 1min
 *
 *
BackupRegister Values:
Register - Value - Description
BR0 - último estado (logState)
BR1  →
BR2  → != 0 - indicates that  STM32 should to go into deepsleep
BR3  → != 0 -//indicates that  have to go into deepsleep
BR4  → contador de boots
BR5 e BR6 → timestamp (parte baixa/alta)
BR7 - FREE
BR8 - FREE
BR9 - Nuber of Rain Gauge pulses

Flag to enter in deep sleep mode:
goToSleep_flag = 0; i boot flag
goToSleep_flag = 1; //hibernation flag normal deepsleep ?????
goToSleep_flag = 2; //hibernation flag 1min ?????

Last state before reset/sleep:
goToSleep = 10 // vai dormir
loop = 20 // entrou no loop
onReceive = 30
sendReadings = 40

-06.08.2025 OK  - Cleaning and organizing the file
            OK  - Identing the file
            OK - Increasing RSSI signal for LoRa power to 20dBm
            - Correcting problem that sends lora message before ending US routines.
-27.08.2025 - Trying to correct the problem that freezes the return of the sensor after the hibernation leaving the holes in the graphic.
 * Debug-enhanced version of BluePill LoRa Transmitter
 * - Logs last execution state into BackupRegisters
 * - Handles new LoRa timestamp message format: "TS:<timestamp>"
 * - Prints backup registers at startup
 * 
BR1 / BR2 → timestamp (parte baixa/alta)

Sobre a lib low power:
https://github.com/stm32duino/STM32LowPower
void shutdown(uint32_t ms): enter in shutdown mode param ms (optional): number of milliseconds before to exit the mode. The RTC is used in alarm mode to wakeup the board in ms milliseconds.
Important:
RTC used as Wakeup source requires to have LSE or LSI as clock source. If one of them is used nothing is changed else it will configure it to use LSI clock source. One exception exists when SHUTDOWN_MODE is requested and PWR_CR1_LPMS is defined, in that case LSE is required. So, if the board does not have LSE, it will fail.
Eu uso o: rtc.setClockSource(STM32RTC::LSE_CLOCK); está correto.
The board will restart when exit shutdown mode.

Hardware state
shutdown mode: high wake-up latency (possible hundereds of ms or second timeframe), voltage supplies are cut except always-on domain, memory content are lost and system basically reboots.

21/09/25 - Adding pluviometer with LowPower.deepSleep instead of LowPower.shutdown
Tested on breadboard with lora pins:
#define LoRa_RST PB1  // it worked on breadboard
#define LoRa_DIO0 PB0 // it worked on breadboard
and pluviometer on PB8
Initial tests ok, passing to PCB...

TODO:
- software debouncing
- hardware debouncing
- activate BME280
- Battery Voltage measure 
*/

/*********************************************** Sensor Description ***********************************************/
#define sensor_id "Station_02"      // <<=== Sensor identification  ==>> CHANGE HERE!!
#define sensor_location "Bridge_01" // <<=== Sensor location        ==>> CHANGE HERE!!

/*********************************************** Macro Definitions ***********************************************/
// Enable (uncommenting) or disable (commenting out) services and periferals
#define enableSerialLog  // enable Serial debug on console
// #define enableWatchDog   // enable watchdog for deepsleep
// #define enableUltrasonic // enable Ultrasonic Sensor
#define enableRTCstm32   // using STM32 internal RTC Clock
#define enableRainGauge  // Enable Pluviometer
// #define enableTinyRTC         // TODO: not used because de SPI bus freezes and loose the connection!
#define enableLoRa // enable LoRa communication
#define enableDebug // Enable verbosity in debugging log

/*********************************************** Library Definitions ***********************************************/
#include <Arduino.h>
#include <stdlib.h>
#include <STM32LowPower.h> //Deep Sleep for STM32
#include <SPI.h>
#include <time.h>

#ifdef enableRTCstm32
#include <STM32RTC.h>
#endif

#ifdef enableWatchDog
#include <IWatchdog.h>
#endif

#ifdef enableTinyRTC
#include "RTClib.h" //Date and time functions using a DS1307 RTC connected via I2C and Wire lib (Not used because it is loosing connection with the SPI BUS)
#endif

#ifdef enableUltrasonic
#include <NewPing.h>
#endif

#ifdef enableLoRa
#include <LoRa.h>
#endif

/*********************************************** Global Variables ***********************************************/
String version = "System Version: SAPM_Sensor_BluePill_2025092101_Protoboard_pluviometer"; // ==> CHANGE HERE! <==


#ifdef enableRainGauge
// ==== Pluviômetro ====
#define BR9 9 // Bluepill tem até Backup Register 9
#define DEBOUNCE_MS 100
#define RAIN_PER_PULSE 0.25f // 0.25 mm por pulso
bool pulseFlag = 0;          // ISR Flag
volatile uint32_t rainPulses = 0;
// volatile int repetitions = 1;
volatile uint32_t accumulatedRainPulses = 0;
volatile uint32_t lastPulseTime = 0;
// Pin used to trigger a wakeup
#ifndef RAIN_GAUGE
#define RAIN_GAUGE pinNametoDigitalPin(SYS_WKUP1)
#endif
#define RAIN_GAUGE PB8 // rain gauge pin

#endif // enableRainGauge

#define  led_pin PC13 // TODO: Just to have visual information that it is working.

#ifdef enableTinyRTC // Not used
// Store date and time
int year = 0;
int month = 0;
int day = 0;
int hours = 0;
int minutes = 0;
int seconds = 0;
float timezone = 0;
String DATE_FULL3;

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
#endif // enableTinyRTC

#ifdef enableRTCstm32 // Working
/* Get the rtc object */
STM32RTC &rtc = STM32RTC::getInstance();
byte startUpMinute = 0;
/* Change these values to set the current initial time */
byte seconds = 0;
byte minutes = 0;
byte hours = 0;

/* Change these values to set the current initial date */
byte weekDay = 0;
byte day = 0;
byte month = 0;
byte year = 0;

#endif // enableRTCstm32

#ifdef enableUltrasonic
const unsigned int triggerPin = PA3;
const unsigned int echoPin = PA2;
// long lastEchoDistance = 0;             // We want to keep these values after reset
unsigned long pulseLength = 0;
unsigned long readingDistance = 0; // Measured distance in centimeters
// unsigned long maxReadingNumber = 0;    // Number of ultrasonic readings to do the calculation of mean and average
#endif // enableUltrasonic

int goToSleep_flag = 0; // Flag to enter in deep sleep mode

#ifdef enableLoRa
                        // define the pins used by the LoRa transceiver module
#define LoRa_SCK PA5
#define LoRa_MISO PA6
#define LoRa_MOSI PA7
#define LoRa_NSS PA4 // LoRa radio chip select
// #define LoRa_RST = PA8; // LoRa radio reset ==> New PCB
// #define LoRa_DIO0 = PA15; // Change for your board; must be a hardware interrupt pin of the STM32 Bluepill ==> New PCB
// #define LoRa_RST PA0 // LoRa radio reset ==> Old PCB in this works
// #define LoRa_DIO0 PA1 // Change for your board; must be a hardware interrupt pin of the STM32 Bluepill ==> Old PCB in this works
#define LoRa_RST PB1  // it worked on breadboard
#define LoRa_DIO0 PB0 // it worked on breadboard

// Define LoRa Communication Band:
#define BAND 915E6 /*  915E6 for Brazil (902-928 MHz) \
                       433E6 for Asia                 \
                       866E6 for Europe               \
                       915E6 for North America */

int lora_startup_counter = 0; // Counter to check if LoRa chip started communication propperly
long readingID = 0;           // Sending packet N°

String LoRaMessage = ""; // String to store the LoRa Message that should be sent

boolean onReceive_flag = 0;
bool distance_reading_done = false;

#endif                   // enableLoRa

/*********************************************** Function Prototypes ***********************************************/
void sketchSetup();
void logState(uint16_t code);

#ifdef enableTinyRTC
// void startTinyRTC();
// void setTime();
// void readTimeTinyRTC();
#endif // enableTinyRTC

#ifdef enableRTCstm32
void setupRTC();
void setTime();
void readTime();
#endif // enableRTCstm32

#ifdef enableUltrasonic
void ultrasonic_setup();
void readUltrasonic();
float calculateMedian(int *array, int arraySize);
int compareReadings(const void *a, const void *b);
#endif // enableUltrasonic

#ifdef enableLoRa
void LoRa_rxMode();
void LoRa_txMode();
void onReceive(int packetSize);
void LoRa_sendMessage(String message);
void onTxDone();
boolean runEvery(unsigned long interval);
void checkonReceive();
boolean runClockEvery(unsigned long interval);
void start_LoRa();
void sendReadings();
#endif // enableLoRa

#ifdef enableRainGauge
void rainISR();
void getRainPulses();
#endif //enableRainGauge

void goToSleep();

/*********************************************** End Function Prototypes *******************************************/
// TODO: Need to be better documented and clarified!!!!
void setup()
{
  // distance_reading_done = true; // emulates us sensor reading to test without US sensor
  sketchSetup();         // Setup of the Serial log and initial serial setup
  pinMode(led_pin, OUTPUT); 
  pinMode(RAIN_GAUGE, INPUT);                 

  // Blink pattern at startup
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(led_pin, HIGH);
    delay(150);
    digitalWrite(led_pin, LOW);
    delay(150);
  }

  // Contador de reinicializações no BackupRegister 4
  /* Cada vez que o setup() roda (seja por reset, watchdog ou wake-up), ele incrementa o valor armazenado no BR4.*/
  enableBackupDomain();
  uint16_t bootCounter = getBackupRegister(4);
  bootCounter++;
  setBackupRegister(4, bootCounter);
  disableBackupDomain();

  Serial.print("Boot counter (BR4): ");
  Serial.println(bootCounter);

#ifdef enableWatchDog
  enableBackupDomain(); // Function of .platformio\packages\framework-arduinoststm32\cores\arduino\stm32\backup.h

  if (getBackupRegister(2) != 0)
  { // indicates that  STM32 should to go into deepsleep
    Serial.println("Sistema reinicializado pelo WatchDog ... === Irá entrar em hibernação ... === BR2 = " + String(getBackupRegister(2)));
    setBackupRegister(2, 0);
    delay(100);
    setupRTC();
    LowPower.begin();
    goToSleep_flag = 2; // hibernation flag 1min
    goToSleep();
  }

  disableBackupDomain();
  IWatchdog.begin(10000000); // Init the watchdog timer with 10 seconds timeout
#endif

  // Enable the LoRa power supply
  // TODO: Disabled to test instability
  // pinMode(PB13, OUTPUT);
  // digitalWrite(PB13, HIGH);

  // delay(50); // Enable MP2307 in the MINI360 power regulator, it is needed 16ms to activate Vout

  Serial.println("Starting RTC with LSE Clock...");
  #ifdef enableWatchDog
  setupRTC();
  delay(50);
  #endif
  // Serial.println("setTime()");
  //   setTime();
  //   startUpMinute = rtc.getMinutes();
  // if (!rtc.isConfigured())
  // {
  //   Serial.println("Erro: RTC com LSE não está funcionando!");
  //   // Colocar alguma lógica alternativa ou travar o sistema
  //   while (1)
  //     ;
  // }
  Serial.println("DEBUG: Starting LowPower.begin()");
// Configure low power
  LowPower.begin();
  // Attach a wakeup interrupt on pin, calling rainISR when the device is woken up
  // Last parameter (LowPowerMode) should match with the low power state used: in this example LowPower.sleep()
  LowPower.attachInterruptWakeup(RAIN_GAUGE, rainISR, RISING, DEEP_SLEEP_MODE);
  /*void attachInterruptWakeup(uint32_t pin, voidFuncPtrVoid callback, uint32_t mode, LP_Mode LowPowerMode): Enable GPIO pin in interrupt mode. If the pin is a wakeup pin, it is configured as wakeup source (see board documentation). param pin: pin number
param callback: pointer to callback
param mode: interrupt mode (HIGH, LOW, RISING, FALLING or CHANGE) param LowPowerMode: Low power mode which will be used (IDLE_MODE, SLEEP_MODE, DEEP_SLEEP_MODE or SHUTDOWN_MODE). In case of SHUTDOWN_MODE only, Wakeup pin capability is activated. see: 
https://github.com/stm32duino/STM32LowPower/blob/main/examples/ExternalWakeup/ExternalWakeup.ino
*/

  // Serial.println("ultrasonic_setup()");
  #ifdef enableUltrasonic
  ultrasonic_setup();
  #endif
  Serial.println("start_LoRa()");
  start_LoRa();
}

/*********************************************** loop () ***********************************************/
void loop()
{
  logState(20); // entrou no loop
  getRainPulses(); // Checks if received a pulse

  #ifdef enableUltrasonic
  readUltrasonic();
  #endif

  sendReadings();

#ifdef enableWatchDog
  IWatchdog.reload();
#endif

  enableBackupDomain();
  setBackupRegister(2, 10);
  disableBackupDomain();
  delay(5000); // Espera 5 segundos entre transmissões

   // Triggers an infinite sleep (the device will be woken up only by the registered wakeup sources)
  // The power consumption of the chip will drop consistently
  Serial.println("Going to LowPower.deepSleep by 10sec");
  Serial.flush();
  LowPower.deepSleep(10000); // Deep sleep 10s
  // goToSleep();
}

/*********************************************** End loop () ***********************************************/

/*********************************************** Function Definitions ***********************************************/
//////////////////////////////////////////////////// sketchSetup ////////////////////////////////////////////////////
// Shows system infomation and configures serial interface
void sketchSetup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println("\nStarting Sensor: " + String(sensor_id) + " on " + String(sensor_location));
  Serial.println("\nIlha 3d");
  Serial.println("\nwww.ilha3d.com");
  Serial.println("\n");
  Serial.println(String(version));
  Serial.println("");

  Serial.println("=== Boot STM32 Sensor Node with New PCB Pins===");
  Serial.print("Last state before reset/sleep: ");
  Serial.println(getBackupRegister(0));
  Serial.print("BackupReg1 (TS low): ");
  Serial.println(getBackupRegister(5));
  Serial.print("BackupReg2 (TS high): ");
  Serial.println(getBackupRegister(6));

#ifdef enableDebug
  Serial.println("Last values of all Backup Registers:");
  Serial.println("BR0 = " + String(getBackupRegister(0)));
  Serial.println("BR1 = " + String(getBackupRegister(1)));
  Serial.println("BR2 = " + String(getBackupRegister(2)));
  Serial.println("BR3 = " + String(getBackupRegister(3)));
  Serial.println("BR4 = " + String(getBackupRegister(4)));
  Serial.println("BR5 = " + String(getBackupRegister(5)));
  Serial.println("BR6 = " + String(getBackupRegister(6)));
  Serial.println("BR7 = " + String(getBackupRegister(7)));
  Serial.println("BR8 = " + String(getBackupRegister(8)));
  Serial.println("BR9 = " + String(getBackupRegister(9)));
#endif
}

#ifdef enableTinyRTC
void startTinyRTC()
{
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    // abort();
  }

  if (!rtc.isrunning())
  {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    ////rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}

void setTime()
{
  // Set the time
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  rtc.adjust(DateTime(year - 2000, month, day, hours, minutes, seconds));
}
void readTimeTinyRTC()
{
  DateTime now = rtc.now();
  char DATE_FULL_RTC[] = "DD/MM/YYYY, hh:mm:ss";
  /*
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  */
  Serial.print("Data e hora atual: ");
  Serial.println(now.toString(DATE_FULL_RTC));
  // Prepara para atualizar o grafico a cada 30minutos
  /*if (now.minute() == 30 or now.minute() == 0) {
    //Serial.println(rtc.getHours());
    //Serial.println(rtc.getMinutes());
    Serial.println("Time of Transmission to Server");
  }*/
}
#endif // enableTinyRTC

#ifdef enableRTCstm32
void setupRTC()
{
  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  // rtc.setClockSource(STM32RTC::LSI_CLOCK); //3V3 ligado com diodo no VBAT
  rtc.setClockSource(STM32RTC::LSE_CLOCK); // 3V3 wired with a diode on VBAT
  rtc.begin();                             // initialize RTC 24H format
}

void setTime()
{
  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setWeekDay(weekDay);
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);
}

void readTime()
{
  // Print date...
  Serial.println("Data e hora armazenada no RTC Local");
  Serial.printf("%02d/%02d/%02d ", rtc.getDay(), rtc.getMonth(), rtc.getYear());

  // ...and time
  Serial.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds());
}
#endif // enableRTCstm32

#ifdef enableUltrasonic
// Ultrasonic Distance Sensor setup
void ultrasonic_setup()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
#ifdef enableSerialLog
  Serial.println("Setting up Ultrasonic Sensor.");
#endif
} // end ultrasonic_setup

// Function adapted to calculate the median of 11 readings from the ultrasonic sensor and display it as the read distance
void readUltrasonic()
{
  int readings[11]; // 11 readings (To calculate the median it is better to use an odd number)
  int ultrasonic_readings_array_size = sizeof(readings) / sizeof(readings[0]);

#ifdef enableSerialLog
  Serial.println("ultrasonic_readings_array_size = " + String(ultrasonic_readings_array_size));
#endif

  // Take 11 consecutive readings to calculate the median and eliminate undue readings and outliers due to ultrasound reflection:
  for (int i = 0; i < ultrasonic_readings_array_size; i++)
  {               // for1
  check_distance: // Label for goto
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(5);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    pulseLength = pulseIn(echoPin, HIGH);
    readingDistance = pulseLength / 58; // Measured distance in centimeters
    delay(50);

    if (readingDistance > 500 || readingDistance < 0)
    { // eliminate erroneous readings above or below the sensor range
      goto check_distance;
    }
    readings[i] = readingDistance;

#ifdef enableSerialLog
    Serial.println("readings-" + String(i) + " = " + String(readings[i]));
#endif
  } // for1

  float median = calculateMedian(readings, ultrasonic_readings_array_size); // Calculate the Median

  // TODO: Verifica se a median mudou significativamente
  // if (abs(median - lastEchoDistance) >= 1) { // check for change in distance só manda msg se mudar o valor > 1cm
  // lastEchoDistance = median;

#ifdef enableSerialLog
  Serial.println("Distância lida pelo sensor ultrassônico (mediana): " + String(median) + "cm");
  distance_reading_done = true;
  // Serial.println("distance_reading_done: " + String(distance_reading_done));
#endif
  // }

  delay(50); // para economizar bateria, pode-se reduzir esse tempo
}

// TODO: improve commenting
float calculateMedian(int *array, int arraySize)
{
  qsort(array, arraySize, sizeof(int), compareReadings);

// Print the sorted values
#ifdef enableSerialLog
  Serial.println("Leituras das distâncias ordenadas:");
  for (int i = 0; i < arraySize; i++)
  {
    Serial.println(array[i]);
  }
  Serial.println("\n");
#endif

  if (arraySize % 2 == 0)
  {
    return (float)(array[arraySize / 2 - 1] + array[arraySize / 2]) / 2;
  }
  else
  {
    return (float)array[arraySize / 2];
  }
}

int compareReadings(const void *a, const void *b)
{
  return (*(int *)a - *(int *)b);
}
#endif // enableUltrasonic

//////////////////////////////////////////////////// goToSleep() ////////////////////////////////////////////////////
void goToSleep()
{
  Serial.println("goToSleep()");
  logState(10); // vai dormir
  if (goToSleep_flag == 2)
  { // hibernará por 1 minuto
#ifdef enableWatchDog
    IWatchdog.reload();
#endif
    Serial.println("Hibernando por 1 minuto... goToSleep_flag == " + String(goToSleep_flag));
    delay(10);
    // LowPower.shutdown(1000 * 60); // hiberna por 1 min
    LowPower.deepSleep(1000 * 60); // for Rain Gauge
    // Ao acordar, reestabiliza a serial
    delay(100);
    Serial.flush();
    Serial.end();
    delay(50);
    Serial.begin(115200);
  }
  // Entra em Deep Sleep e acorda em horas cheias hh:00 ou hh:30
  if (goToSleep_flag == 1)
  {
    //   //DateTime now = rtc.now();
    //   //int sleepTime = 59 - now.minute(); //TinyRTC
#ifdef enableWatchDog
    IWatchdog.reload();
#endif
    Serial.println("Hibernando por 1 minuto... goToSleep_flag == " + String(goToSleep_flag));
    delay(10);
    // LowPower.shutdown(1000 * 60); // hiberna por 1 min
    LowPower.deepSleep(1000 * 60); // for Rain Gauge
    // Ao acordar, reestabiliza a serial
    delay(100);
    Serial.flush();
    Serial.end();
    delay(50);
    Serial.begin(115200);
  }
}

#ifdef enableLoRa
void LoRa_rxMode()
{
  LoRa.enableInvertIQ(); // active invert I and Q signals
  LoRa.receive();        // set receive mode
}

void LoRa_txMode()
{
  LoRa.idle();            // set standby mode
  LoRa.disableInvertIQ(); // normal mode
}

//==================================== Lora Callback void onReceive ===================================================
void onReceive(int packetSize)
{
  if (packetSize == 0) return;

  String LoRaData = LoRa.readString();
  Serial.print("LoRaData recebida: ");
  Serial.println(LoRaData);

  if (LoRaData.startsWith("TS:"))
  {
    String tsStr = LoRaData.substring(3, LoRaData.indexOf('|') > 0 ? LoRaData.indexOf('|') : LoRaData.length());
    unsigned long ts = tsStr.toInt();
    Serial.print("Timestamp recebido: ");
    Serial.println(ts);

    // ===== Conversão do timestamp para hora normal =====
    time_t rawtime = (time_t)ts;
    struct tm *timeinfo = gmtime(&rawtime); // ou localtime() se quiser considerar fuso

    char buffer[30];
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d",
            timeinfo->tm_mday,
            timeinfo->tm_mon + 1,
            timeinfo->tm_year + 1900,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec);

    Serial.print("Hora convertida: ");
    Serial.println(buffer);
    // ================================================

    logState(30); //onReceive
    enableBackupDomain();
    setBackupRegister(5, (uint16_t)(ts & 0xFFFF));
    setBackupRegister(6, (uint16_t)((ts >> 16) & 0xFFFF));
    disableBackupDomain();
  }
  else
  {
    Serial.println("Mensagem LoRa recebida em formato inesperado.");
  }
}


void LoRa_sendMessage(String message)
{
  LoRa_txMode();        // set tx mode
  LoRa.beginPacket();   // start packet
  LoRa.print(message);  // add payload
  LoRa.endPacket(true); // garante envio completo antes de continuar
  Serial.println("Transmissão concluída. Indo dormir...");
  goToSleep_flag = 2;    // força entrada no sono
}

void onTxDone()
{
#ifdef enableSerialLog
  Serial.println("TxDone - Transmissão completa.");
#endif

  // Define a flag para hibernar. Isso só será executado quando a transmissão for finalizada.
  goToSleep_flag = 2;

  // Retorna ao modo de recepção para o próximo ciclo
  LoRa_rxMode();
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

void checkonReceive()
{ // TODO ver se essa é a função que recebe o retorno do gateway
  if (onReceive_flag == 0)
  {
    int time = (rtc.getMinutes() - startUpMinute);
    Serial.print("time:   ");
    Serial.println(time);
    if (rtc.getMinutes() - startUpMinute >= 2)
    {
      // #ifdef enableSerialLog
      Serial.print("Did not receive the Date from the Gateway! Going to sleep for 30 seconds");
      delay(100);
      // #endif
      // vai dormir por 30 segundos...
      //  LowPower.shutdown(1000 * 30); //D.S por 1000ms* 30s * sleepTime/
    }
  }
}

boolean runClockEvery(unsigned long interval)
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

//=============================================================================================================
// Initialize LoRa module
void start_LoRa()
{
  LoRa.setTxPower(20); // Change LoRa transmission power to 20dBm
  Serial.println("Potência de Transmissão LoRa: 20dBm");

  LoRa.setPins(LoRa_NSS, LoRa_RST, LoRa_DIO0); // SPI LoRa pins
  // LoRa.setPins(Lora_SS, Lora_RST, Lora_DIO0); //pinos definidos diretamente na lib
  // SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_NSS); //pinos definidos diretamente na lib

  while (!LoRa.begin(BAND) && lora_startup_counter < 10)
  {
    Serial.print(".");
    lora_startup_counter++;
    delay(500);
  }
  if (lora_startup_counter == 10)
  {
    Serial.println("LoRa initialization Failed!");
    // delay (100);
  }
  if (lora_startup_counter < 10)
  {
#ifdef enableSerialLog
    Serial.println("LoRa initialization OK!");
#endif
  }

// Setup receiver para receber o update da hora:
#ifdef enableSerialLog
  Serial.println("LoRa Receiver Callback with LoRa Reset in " + String(LoRa_RST));
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();
#endif

  /* Endurecer a recepção: CRC + Sync Word + parâmetros idênticos
  Com CRC desativado e sync word default, qualquer ruído “LoRa-like” pode passar. Ative CRC e defina Sync Word e parametrização idêntica nos dois lados (sensor e gateway). 
  Acrescente no setup do LoRa em ambos:
  */
  LoRa.setSpreadingFactor(7);          /* igual nos dois
                                        Spreading Factor: Change the spreading factor of the radio.
                                        LoRa.setSpreadingFactor(spreadingFactor);
                                        spreadingFactor - spreading factor, defaults to 7
                                        Supported values are between 6 and 12. If a spreading factor of 6 is set, implicit header mode must be used to transmit and receive packets.*/
  LoRa.setSignalBandwidth(125E3);      /* igual nos dois
                                        Signal Bandwidth:  Change the signal bandwidth of the radio.
                                        LoRa.setSignalBandwidth(signalBandwidth);
                                        signalBandwidth - signal bandwidth in Hz, defaults to 125E3.
                                        Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.*/
  LoRa.setCodingRate4(5);              /* CR 4/5 (igual nos dois)
                                        Coding Rate: Change the coding rate of the radio.
                                        LoRa.setCodingRate4(codingRateDenominator);
                                        codingRateDenominator - denominator of the coding rate, defaults to 5
                                        Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4.*/
  LoRa.setPreambleLength(8);           /* igual nos dois
                                      Preamble Length: Change the preamble length of the radio.
                                      LoRa.setPreambleLength(preambleLength);
                                      preambleLength - preamble length in symbols, defaults to 8
                                      Supported values are between 6 and 65535.*/
  LoRa.setSyncWord(0x12);              /* igual nos dois (privado) — escolha um e padronize
                                      Sync Word: Change the sync word of the radio.
                                      LoRa.setSyncWord(syncWord);
                                      syncWord - byte value to use as the sync word, defaults to 0x12 */
  LoRa.enableCrc();                   /* ATIVAR CRC (nos dois)
                                      Enable or disable CRC usage, by default a CRC is not used.
                                      LoRa.enableCrc();
                                      LoRa.disableCrc();*/
                                      
  // register the receive callback
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
} // end start_LoRa

void sendReadings()
{

  Serial.println("DEBUG: inside sendReadings()");
  logState(40); // está no sendReadings
                  // if (runEvery(5000))
                  // { // repeat every 5 sec
  // TODO se recebe confirmação de recebimento do gateway, não pode enviar mais para economizar bateria ver email: Checagem de Retorno de mensagem LoRa
  if (distance_reading_done || (accumulatedRainPulses > 0) ) // Just sends after the US have done all the measurementes or The pluviometer has measurements
  {

    // #ifdef enableRainGauge
    // float rain = getRainVolume();
    
    // Serial.print("RainGauge pulses = ");
    // Serial.print(getRainPulses());
    // Serial.print(" | Accumulated Precipitation = ");
    // Serial.print(rain, 2);
    // Serial.println(" mm");
    // #endif

    // TODO: Do I know it the receiver received the LoRa message? how?
    // LoRaMessage = String(sensor_id) + "/" + String(readingDistance) + "&" + String(readingDistance);
    LoRaMessage = String(sensor_id) + "/" + String(accumulatedRainPulses) + "&" + String(accumulatedRainPulses);
    
    // LoRaMessage = String(sensor_id) + "/" + String(readingDistance) + "&" + String(rain, 2);

    // Send LoRa packet to receiver
    LoRa_sendMessage(LoRaMessage); // send a LoRaMessage
    distance_reading_done = false;
    accumulatedRainPulses = 0; // TODO: Resets after sending

#ifdef enableSerialLog
    Serial.print("Sending packet N°: ");
    Serial.println(readingID);
    Serial.print("LoRaMessage: ");
    Serial.println(LoRaMessage);
#endif
#ifdef enableWatchDog
    IWatchdog.reload();
#endif

    readingID++;
  }
  // }
}

#endif // enableLoRa

#ifdef enableRainGauge
void rainISR() //
{
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
  pulseFlag = 1; // Received a pulse
  ++rainPulses;
}

void getRainPulses()
{
  if (pulseFlag){
    Serial.println("Pulse: " + String(rainPulses)); 
    accumulatedRainPulses = rainPulses + accumulatedRainPulses;
    Serial.println("accumulatedRainPulses: " + String(accumulatedRainPulses)); 
    Serial.println("Accumulated Precipitation: " + String(RAIN_PER_PULSE*accumulatedRainPulses) + "mm");
    pulseFlag = 0; 
    rainPulses = 0;
  }
}
#endif //enableRainGauge

/*********************************************** Helpers ***********************************************/
void logState(uint16_t code)
{
  enableBackupDomain();
  setBackupRegister(0, code);
  disableBackupDomain();
}

/*********************************************** End Function Definitions ********************************************/