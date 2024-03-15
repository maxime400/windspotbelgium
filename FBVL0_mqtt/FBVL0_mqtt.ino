#include <ArduinoJson.h>
#include <Wire.h>


#include <string.h>
#include <esp_task_wdt.h>
// Activate mqtt debug
#define MQTT_DEBUG
// Configure Watch dog timer
#define WDT_TIMEOUT 120

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWRKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22


// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS




#include <TinyGsmClient.h>
#include <PubSubClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif


TinyGsmClient client(modem);
PubSubClient mqtt(client);

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);



#define uS_TO_S_FACTOR 1000000


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

int Array[16];


#include <bits/stdc++.h>
#include <iostream>
using namespace std;
// ...

//const char apn[]      = "internet.proximus.be"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char apn[]      = "TM";
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// SIM card PIN (leave empty, if not defined)
//const char simPIN[]   = "8490";
const char simPIN[]   = "1503";

// MQTT details
const char *broker = "windspotbelgium.com";

const char *topicLed = "GsmClientTest/led";
const char *topicInit = "GsmClientTest/init";
const char *topicLedStatus = "GsmClientTest/ledStatus";


// Variable definition for wind direction


unsigned char i;
unsigned char N ;
unsigned char NNE ;
unsigned char NE ;
unsigned char ENE ;
unsigned char E ;
unsigned char ESE ;
unsigned char SE ;
unsigned char SSE ;
unsigned char S ;
unsigned char SSO ;
unsigned char SO ;
unsigned char OSO ;
unsigned char O ;
unsigned char ONO ;
unsigned char NO ;
unsigned char NNO ;

// Variable definition for speedmeter
unsigned long counter = 0;
unsigned long counter_4s = 0;
unsigned long counter_10s = 0;
unsigned long counter_300s = 0;

volatile unsigned long rotation; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
volatile boolean countEnable;
//unsigned long rotation = 0;
float RealSpeed = 0;
unsigned long delta = 0;
boolean mem1 = 0;
boolean ReadDir = 0;
unsigned char table_speed[100];
unsigned short table_sensor[100];
unsigned char tsi = 0;
float speed_avg = 0;
float speed_min = 0;
float speed_max = 0;
float sensor_avg = 0;
float VoltageInputRealVal = 0;
float VoltageSolRealVal = 0;
float VoltageBatteryRealVal = 0;
unsigned char cnt_solar_low = 0 ;
unsigned char cnt_battery_low = 0 ;
unsigned char cnt_input_low = 0 ;
unsigned long timetosleep = 0;

unsigned short ConnectionSimServer;
char c;
int posA;
int posB;
int lenght;
String JSON_msg = "";
String CalValue = "";
String sleepstatus = "";
String SleepInfo = "";
int Sleeptime = 0;
float PwrLim = 0.0;
volatile unsigned long ContBncLim = 15;
int ResetDelay = 5;

unsigned long timeout = millis();

// Define counter to check live time
volatile unsigned int livetime_min = 0;   // counter



// Variables to track the last time the counter was reset
unsigned long lastCounterReset = millis();
unsigned long counterInterval = ResetDelay * 60 * 60 * 1000;

// mqtt stuff
uint32_t lastReconnectAttempt = 0;
int ledStatus = LOW;

//------  DEFINE I/O ESP32    ------------------------------------------------------------------------------//


// Analog input
const int Pin_batterie =  A11; //pin0
const int Pin_Davis_ana =  A16;  // pin14  (davis green wire)

// Digital input
const int Pin_Davis_pulse =  12;  // pin12(davis black wire)

// Digital output
const int Pin_Enable = 33;  // pin35
const int Pin_Reset = 4;  // pin4	Not used now
const int Pin_led = 13; // pin13



//----------------------------------------------------------------------------------------------------------//
//---------------------------------------      SETUP     -----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------//


void setup() {


  // ------------- Initialise GSM communication ------
  Serial.begin(115200);




  // ------------- Input/Output ------
  pinMode(Pin_Reset, OUTPUT); // Always 1 - if 0 reboot module
  pinMode(Pin_Davis_pulse, INPUT); // pulse capteur

  digitalWrite(Pin_Reset, LOW);

  pinMode(Pin_led, OUTPUT); // LED

  flashOutput(10, 600, Pin_led);

  // SIM modem initial setup
//  setupModem();

  //  // ------------- Read Calibration value for Girouette ------
  //   Serial.println("Read Girouette calibration value from Web Server");
  //
  //   String RequestData3 =      "api_key=" + apiKeyValue +
  //                              "";
  //   int i = 0;
  //
  //   Serial.println("Read Girouette calibration value from Web Server");
  //
  //   InitModem();
  //
  // 	// MQTT Broker setup
  // 	mqtt.setServer(broker, 1883);
  // 	mqtt.setCallback(mqttCallback);
  //
  //   mqtt_port
  //
  //
  //   Serial.println("--> Modem initialisation done");
  //   while ((CalValue == "") && (i <= 3)) {
  //
  //     Serial.print("--> try JSON from Web: ");
  //     Serial.println(i);
  //

  //     Serial.println("--> Calibration value done ");
  //     Serial.println("read girouette calibration =");
  //     Serial.println(CalValue);
  //     flashOutput(6, 400, Pin_led);
  //     i++;
  //
  //     if ((CalValue == "") && (i >= 1)) {
  //       Serial.println("reboot device by disable pin EN in 30 min ");
  //       flashOutput(30, 400, Pin_led);
  //       delay(1800000);	// pause 30 min
  //       // Pause the programm
  //       digitalWrite(Pin_Reset, HIGH);
  //       delay(10000);
  //          String SleepInfo = "Sleep mode activation because no able to initialise modem";
  //          int SleepTime = 120;
  //          Serial.print("Sleep mode actiovated because not able to initialise the modem for ");
  //          Serial.print(SleepTime);
  //          Serial.println(" minutes");
  //          Sleep(SleepInfo, SleepTime);
  //     }
  //   }
  //
  //
  //   StopModem();
  //   const size_t capacity2 = JSON_OBJECT_SIZE(2) + 100;
  //   DynamicJsonBuffer jsonBuffer2(capacity2);
  //
  //   JsonObject& root2 = jsonBuffer2.parseObject(CalValue);
  //   if (!root2.success()) {
  //     Serial.println("parseObject() failed");
  //   }
  int N_cal = 4095;
  int NNE_cal = 0;
  int NE_cal = 150;
  int ENE_cal = 405;
  int E_cal = 645;
  int ESE_cal = 915;
  int SE_cal = 1113;
  int SSE_cal = 1365;
  int S_cal = 1727;
  int SSO_cal = 1962;
  int SO_cal = 2198;
  int OSO_cal = 2446;
  int O_cal = 2745;
  int  ONO_cal = 3005;
  int NO_cal = 3340;
  int  NNO_cal = 3900;

  ContBncLim = 50;
  Serial.print("------>  ContBncLim  = ");
  Serial.println(ContBncLim);
  PwrLim = 0;
  Serial.print("------>  Power Limit threshold  = ");
  Serial.println(PwrLim);
  ResetDelay = 4;
  Serial.print("------>  ResetDelay  = ");
  Serial.println(ResetDelay);


  Array[0] = N_cal;
  Array[1] = NNE_cal;
  Array[2] = NE_cal;
  Array[3] = ENE_cal;
  Array[4] = E_cal;
  Array[5] = ESE_cal;
  Array[6] = SE_cal;
  Array[7] = SSE_cal;
  Array[8] = S_cal;
  Array[9] = SSO_cal;
  Array[10] = SO_cal;
  Array[11] = OSO_cal;
  Array[12] = O_cal;
  Array[13] = ONO_cal;
  Array[14] = NO_cal;
  Array[15] = NNO_cal;
  Serial.print("calibration value for North :");
  Serial.println(Array[0]);



  //   // ---------Initialise counter
  //   counter = millis() / 1000;
  //   counter_4s = counter;
  //   counter_300s = counter;
  //
  //   Serial.println("Configuring WDT...");
  //   esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  //   esp_task_wdt_add(NULL); //add current thread to WDT watch
  //
  //
  //
  //   // Define the interval for the reset counter in milliseconds
  //   counterInterval = ResetDelay * 60 * 60 * 1000; //  hours in milliseconds
  //   Serial.print("The board will auto-reset every ");
  //   Serial.print(ResetDelay);
  //   Serial.println(" hour");
  //
  //   flashOutput(6, 1000, Pin_led);
  //





}

//----------------------------------------------------------------------------------------------------------//
//---------------------------------------      MAIN LOOP     -----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------//






void loop() {

  
 attachInterrupt(digitalPinToInterrupt(Pin_Davis_pulse), isr_rotation, FALLING);
  countEnable = true;
  delay(4000);
  detachInterrupt(digitalPinToInterrupt(Pin_Davis_pulse));
  countEnable = false;

  esp_task_wdt_reset();
  //CheckVoltage();
  RecordData(4);


  // Check if it's time to reset the counter
  Serial.print("-->  lastCounterReset >= counterInterval:");
  Serial.print(millis() - lastCounterReset);
  Serial.print("  >  ");
  Serial.println(counterInterval);
  livetime_min = ((millis() - lastCounterReset) / (60 * 1000)); // calculate live time in minutes
  Serial.print("-------->  livetime_min:");
  Serial.println(livetime_min);

  if (millis() - lastCounterReset >= counterInterval) {
    // ----------------- Loop every 24 hour ----------------------------//
    SerialMon.print("Set Pin reset to high for 10 sec ");
    // Set the output pin HIGH
    digitalWrite(Pin_Reset, HIGH);
    Sleep("reboot auto because no hardware reset", 1);
    // Wait for a short duration (e.g., 1 second) to ensure a noticeable pulse
    delay(10000);
    SerialMon.print("Set Pin reset to off");  // Should not be executed because PinREset the module just before
    digitalWrite(Pin_Reset, LOW);
  }


  // ----------------- Loop every 5 minutes ----------------------------//




  delta = counter - counter_300s;



  if (delta > 300) {


    digitalWrite(Pin_led, HIGH);
    // -- Read sleeping mode
    Serial.println("-->  Init Modem for GETTING Sleep info ");
//    InitModem();
    Serial.println("-->  Start JSONFromWeb for Sleep info ");
//    String RequestData =      "api_key=" + apiKeyValue +
//                              "";
    sleepstatus = "";
    //    StopModem();
    Serial.println("-->  Stop Modem");
    Serial.print("-->  read sleeping status:");
    Serial.println(sleepstatus);


    const size_t capacity = JSON_OBJECT_SIZE(2) + 50;
    DynamicJsonBuffer jsonBuffer(capacity);

    JsonObject& root = jsonBuffer.parseObject(sleepstatus);
    if (!root.success()) {
      Serial.println("parseObject() failed");
    }
    String SleepOrAwake = root["SleepOrAwake"];
    String SleepTime_str = root["SleepTime"];     // SleepTime from WebServer is in minutes
    Serial.print("Json object tell us to sleep or to be awake; ");
    Serial.println(SleepOrAwake);
    Serial.print("Json object tell us to continue sleeping till; ");
    Serial.print(SleepTime_str);
    Serial.println(" hours");

    // if sleep
    if ( SleepOrAwake == "Sleep") {
      int SleepTime = SleepTime_str.toInt();
      String SleepInfo = "Scheduled sleep";
      Serial.print("ESP32 goes to sleep for ");
      Serial.print(SleepTime);
      Serial.println(" minutes");
//      String RequestData =      "api_key=" + apiKeyValue +
//                                "&SleepInfo=" + String(SleepInfo) +
//                                "&SleepTime=" + String(SleepTime) +
//                                "";
      String SendStatus = "";
      int flag_reboot = 0;
      while ((SendStatus == "") && (flag_reboot < 5)) {

        flag_reboot++;
        if (flag_reboot >= 5) {
          Serial.print("Reboot device by disable pin EN ");
          Sleep("reboot auto because no hardware reset", 1);
          digitalWrite(Pin_Reset, LOW);
        }
      }

      Serial.println("Go to sleep mode based on activation time:");
      Serial.println(SendStatus);
      Sleep(SleepInfo, SleepTime);
      // if awake
    } else {

      int n = sizeof(table_speed) / sizeof(table_speed[0]);
      speed_max = table_speed[0];
      speed_min = table_speed[0];


      for (i = 0 ; i < tsi ; i++) {
        speed_avg += table_speed[i];
        sensor_avg += table_sensor[i];
        if (table_speed[i] > speed_max) {
          speed_max = table_speed[i];
        }
        if (table_speed[i] < speed_min) {
          speed_min = table_speed[i];
        }
      }

      speed_avg = speed_avg / tsi;
      sensor_avg = sensor_avg / tsi;

      int SleepTime = 0;
      String SleepInfo = "Module is awake:";
      String RequestData =      String(N) +
                                String(NNE) +
                                String(NE) +
                                String(ENE) +
                                String(E) +
                                String(ESE) +
                                String(SE) +
                                String(SSE) +
                                String(S) +
                                String(SSO) +
                                String(SO) +
                                String(OSO) +
                                String(O) +
                                String(ONO) +
                                String(NO) +
                                String(NNO) +
                                String(speed_max) +
                                String(speed_min) +
                                String(speed_avg) +
                                String(sensor_avg) +
                                String(VoltageBatteryRealVal) +
                                String(SleepInfo) +
                                String(SleepTime) +
                                String(livetime_min)
                                ;
      String SendStatus = "";
      int flag_reboot = 0;
      while ((SendStatus == "") && (flag_reboot < 5)) {
        Serial.println("----> Init Modem to SEND data to webpage ");
        InitModem();
        Serial.println("----> Start MQTT to send wind data info ");
        if (mqttConnect("wind_data", "test plein de data")){
          SendStatus = "OK";
          Serial.println("---->  MQTT send successful... ");
        } else {
          Serial.println("---->  MQTT send fail... try again... ");
        }
        flag_reboot++;
        if (flag_reboot >= 3) {
          StopModem();
          Serial.print("reboot device by disable pin EN ");
          digitalWrite(Pin_Reset, HIGH);
                   Serial.println("------>  go to sleep mode for 120 minutes because not able to send data info");
                   SleepInfo = "Sleep mode activation because undervoltage on battery";
                   SleepTime = 120;
                   Serial.print("ESP32 goes to sleep for ");
                   Serial.print(SleepTime);
                   Serial.println(" minutes");
                   Sleep(SleepInfo, SleepTime);
        }
      }

      StopModem();
      Serial.println("-->  Stop Modem");
      Serial.print("Sending status data to Web Server:");
      Serial.println(SendStatus);
      Serial.println("Re-initialise variable");
      tsi = 0;
      N = 0;
      NNE = 0;
      NE = 0;
      ENE = 0;
      E = 0;
      ESE = 0;
      SE = 0;
      SSE = 0;
      S = 0;
      SSO = 0;
      SO = 0;
      OSO = 0;
      O = 0;
      ONO = 0;
      NO = 0;
      NNO = 0;

    }   //else  awake

    counter = millis() / 1000; // Re-write counter value due to delay in sending data
    counter_300s = counter;

    digitalWrite(Pin_led, LOW);

  }   // loop 300 sec



  counter = millis() / 1000;


}   //---------------  en main loop -----------------------------------------------------------------------//

//----------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------//

//---------------  FUNCTION -----------------------------------------------------------------------//



//---------------  FUNCTION -----------------------------------------------------------------------//



//---------------  FUNCTION -----------------------------------------------------------------------//

void  Sleep(String SleepInfo, unsigned int SleepTime) {
  //Serial.println("function Sleep is activated");

  unsigned int TIME_TO_SLEEP = (SleepTime * 60);    // Convert minutes in seconds
  uint64_t timetosleep = UINT64_C(TIME_TO_SLEEP * uS_TO_S_FACTOR); // Convert minutes in microseconds
  esp_sleep_enable_timer_wakeup(timetosleep);
  esp_deep_sleep_start();
}


//---------------  FUNCTION -----------------------------------------------------------------------//

//void CheckVoltage() {
//  int SleepTime = 0;
//  String SleepInfo = "";
//  // read voltage -- analog input = max 3.3V = 4095
//
//  // read voltage -- analog input = max 3.3V = 4095
//  int VoltageBatValue = analogRead(Pin_batterie);
//  VoltageBatteryRealVal = ((((float)VoltageBatValue / 4095) * 3.3) * 2); // rapport pont diviseur interne (experimental)
//  Serial.print("Voltage Battery Panel RAW Value = ");
//  Serial.println(VoltageBatValue);
//  Serial.print("Voltage Battery Panel Value = ");
//  Serial.println(VoltageBatteryRealVal);
//  Serial.print("Voltage Battery Limit voltage to sleep = ");
//  Serial.println(PwrLim);
//  if (VoltageBatteryRealVal < PwrLim) {
//    cnt_battery_low++;
//    Serial.println("Battery is under 3.3 Volt, after 10 ties it will sleep for 10 hour ");
//    if (cnt_battery_low > 10) {
//      SleepInfo = "Sleep mode activation because undervoltage on battery";
//      SleepTime = 600;
//      Serial.print("ESP32 goes to sleep for ");
//      Serial.print(SleepTime);
//      Serial.println(" minutes");
//    }
//  }
//  if ( SleepTime > 0) {
//    Serial.print("ESP32 goes to sleep for ");
//    Serial.print(SleepTime);
//    Serial.println(" minutes");
//    Serial.print("SleepInfo =");
//    Serial.println(SleepInfo);
//    String RequestData =      "api_key=" + apiKeyValue +
//                              "&SleepInfo=" + SleepInfo +
//                              "&SleepTime=" + String(SleepTime) +
//                              "&VoltageBatteryRealVal=" + String(VoltageBatteryRealVal) +
//                              "";
//    String SendStatus = "";
//
//    Serial.println("----> Init Modem to SEND sleep info ");
//    InitModem();
//    Serial.println("----> Start POST to send sleep info ");

//    Serial.println("---->  POST done... ");
//    StopModem();
//    Serial.println("-->  Stop Modem");
//
//    Sleep(SleepInfo, SleepTime);
//  }
//}

//---------------  FUNCTION -----------------------------------------------------------------------//

void RecordData(unsigned long delta) {

  // read the input on analog pin 0:
  int sensorValue = analogRead(Pin_Davis_ana);
  Serial.print("sensorValue raw analogRead = ");
  Serial.println(sensorValue);
  // Fill an array with the difference between Analog Value and each position
  int diffarray [16];
  diffarray[0] = abs(sensorValue - Array[0]);
  diffarray[1] = abs(sensorValue - Array[1]);
  diffarray[2] = abs(sensorValue - Array[2]);
  diffarray[3] = abs(sensorValue - Array[3]);
  diffarray[4] = abs(sensorValue - Array[4]);
  diffarray[5] = abs(sensorValue - Array[5]);
  diffarray[6] = abs(sensorValue - Array[6]);
  diffarray[7] = abs(sensorValue - Array[7]);
  diffarray[8] = abs(sensorValue - Array[8]);
  diffarray[9] = abs(sensorValue - Array[9]);
  diffarray[10] = abs(sensorValue - Array[10]);
  diffarray[11] = abs(sensorValue - Array[11]);
  diffarray[12] = abs(sensorValue - Array[12]);
  diffarray[13] = abs(sensorValue - Array[13]);
  diffarray[14] = abs(sensorValue - Array[14]);
  diffarray[15] = abs(sensorValue - Array[15]);

  int smallestdiff = diffarray[0];
  int j = 10;
  for (int i = 0 ; i < 16 ; i++) {

    if (diffarray[i] <= smallestdiff) {
      smallestdiff = diffarray[i];
      j = i;

    }
  }

  // Increase orientation variable corresponding to smallest value


  if (j == 0) {
    N++ ; // S
  }
  else if (j == 1) {
    NNE++ ; // E
  }
  else if (j == 2) {
    NE++ ; // N
  }
  else if (j == 3) {
    ENE++ ; // SE
  }
  else if (j == 4) {
    E++ ; // O
  }
  else if (j == 5) {
    ESE++ ; // NE
  }
  else if (j == 6) {
    SE++ ; // SO
  }
  else if (j == 7) {
    SSE++ ; // NO
  }
  else if (j == 8) {
    S++ ; // S
  }
  else if (j == 9) {
    SSO++ ; // E
  }
  else if (j == 10) {
    SO++ ; // N
  }
  else if (j == 11) {
    OSO++ ; // SE
  }
  else if (j == 12) {
    O++ ; // O
  }
  else if (j == 13) {
    ONO++ ; // NE
  }
  else if (j == 14) {
    NO++ ; // SO
  }
  else if (j == 15) {
    NNO++ ; // NO
  }
  else {
    unsigned charIssue = 0;
  }


  Serial.print("--- j = ");
  Serial.println(j);
  //Serial.print("Noorth calibration value = ");
  //Serial.println(Array[0]);
  //Serial.print("Difference for North ");
  //Serial.println(diffarray[0]);
  //Serial.print("North count3 : ");
  //Serial.println(N);
  //Serial.print("Smallest element in array : ");
  //Serial.println(j);
  //Serial.println("end");


  RealSpeed = ((3.6 * rotation) / (delta)); // ( davis6410 formula => V = P(2.25/T) (V[mph] -- P = nbr of pulse -- T = period [sec] )
  Serial.print("speed delta =:");
  Serial.println(delta);
  Serial.print("speed rotation =:");
  Serial.println(rotation);
  Serial.print("speed of the wind [km/h] :");
  Serial.println(RealSpeed);
  table_speed[tsi] = RealSpeed;
  table_sensor[tsi] = sensorValue;
  //   Serial.print("tsi = ");
  //   Serial.println(tsi);
  //Serial.print("]");
  //Serial.println(table_sensor[tsi]);
  rotation = 0;
  tsi++;
}

//---------------  FUNCTION -----------------------------------------------------------------------//

void InitModem() {


    // Set console baud rate
    SerialMon.begin(115200);

    delay(10);

    setupModem();

    SerialMon.println("Wait...");

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    delay(6000);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();
    // modem.init();

    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
    }
#endif

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected()) {
        SerialMon.println("GPRS connected");
    }

    // MQTT Broker setup
    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);

  
}



bool setupPMU()
{
    bool en = true;
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_SYS_CTL0);
    if (en) {
        Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    } else {
        Wire.write(0x35); // 0x37 is default reg value
    }
    return Wire.endTransmission() == 0;
}


void setupModem()
{

    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

}

void StopModem() {

  client.stop();
  //modem.shutdown();
  modem.poweroff();
  digitalWrite(MODEM_POWER_ON, LOW);
  Serial.println("Power off modem");
}

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation () {

  if (countEnable) {
    if ((millis() - ContactBounceTime) > ContBncLim ) { // debounce the switch contact.
      rotation++;
      SerialMon.println("++++");
      ContactBounceTime = millis();
    }
  }
}

// function to flsah an output
void flashOutput(int numOfFlashes, int periodTime, int outputPin) {
  for (int i = 0; i < numOfFlashes; i++) {
    digitalWrite(outputPin, HIGH);  // Turn the output on
    delay(periodTime / 2);          // Wait for half of the period
    digitalWrite(outputPin, LOW);   // Turn the output off
    delay(periodTime / 2);          // Wait for the remaining half of the period
  }
}


void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();

    // Only proceed if incoming message's topic matches
    if (String(topic) == topicLed) {
        ledStatus = !ledStatus;
        digitalWrite(Pin_led, ledStatus);
        mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
    }
}

boolean mqttConnect(char *topic, char *topic_data)
{
    SerialMon.print("Connecting to ");
    SerialMon.print(broker);

    // Connect to MQTT Broker
    boolean status = mqtt.connect(topic);

    // Or, if you want to authenticate MQTT:
    //boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

    if (status == false) {
        SerialMon.println(" fail");
        
        return false;
    }
    SerialMon.println(" success");
    mqtt.publish(topic, topic_data);
    return mqtt.connected();
}
