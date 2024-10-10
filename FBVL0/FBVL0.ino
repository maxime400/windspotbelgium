 #include <ArduinoJson.h>
#include <HTTPClient.h>
#include <string.h>
#include <esp_task_wdt.h>


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

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif


// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Serial.print("I2CPower.writ  write 0x37");
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Serial.print("I2CPower.writ     write 0x35");
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}
String resp_msg = "";

String StringArray[16];


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

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[] = "windspotbelgium.be"; // domain name: example.com, maker.ifttt.com, etc
const char* serverName = "http://windspotbelgium.be/FBVL0/post-data.php";
char resource[] = "/FBVL0/post-data.php";
char  serverSleepInfo[] = "/FBVL0/json/Sleep.php";
char  serverGirouetteInfo[] = "/FBVL0/json/GirouetteCal.php";
const int  port = 80;

// Keep this API Key value to be compatible with the PHP code provided in the project page.
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key
String apiKeyValue = "tPmAT5Ab3j7F9";

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

//------  DEFINE I/O ESP32    ------------------------------------------------------------------------------//


// Analog input
// const int Pin_batterie =  A11; //pin0
const int Pin_Davis_ana =  A6;  // pin34 (davis green wire)

// Digital input
const int Pin_Davis_pulse =  12;  // pin12(davis black wire)

// Digital output
const int Pin_Enable = 33;  // pin35
// const int Pin_Reset = 4;  // pin4	Not used now
const int Pin_led = 13; // pin13


//----------------------------------------------------------------------------------------------------------//
//---------------------------------------      SETUP     -----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------//

void setupModem()
{
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  //  pinMode(LED_PIN, OUTPUT);

  // Reset pin high
  digitalWrite(MODEM_RST, HIGH);

  // Turn on the Modem power first
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Pull down PWRKEY for more than 1 second according to manual requirements
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(200);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1200);
  digitalWrite(MODEM_PWRKEY, HIGH);


}

void setup() {


  // ------------- Initialise GSM communication ------
  Serial.begin(9600);




  // ------------- Input/Output ------
  pinMode(Pin_Enable, OUTPUT); // Always 1 - if 0 reboot module
  pinMode(Pin_Davis_pulse, INPUT); // pulse capteur

//   digitalWrite(Pin_Enable, HIGH);

  pinMode(Pin_led, OUTPUT); // LED

  flashOutput(10, 600, Pin_led);

  // SIM modem initial setup
  setupModem();

  // ------------- Read Calibration value for Girouette ------
  Serial.println("Read Girouette calibration value from Web Server");

  String RequestData3 =      "api_key=" + apiKeyValue +
                             "";
  int i = 0;

  Serial.println("Read Girouette calibration value from Web Server");

  InitModem();
  Serial.println("--> Modem initialisation done");
  while ((CalValue == "") && (i <= 3)) {

    Serial.print("--> try read girouette calibration: test number ");
    Serial.print(i);
    Serial.println(" ...");
    CalValue = JSONFromWeb(RequestData3, serverGirouetteInfo);
    Serial.print("result =");
    Serial.println(CalValue);
    flashOutput(6, 1000, Pin_led);
    i++;

    if ((CalValue == "") && (i >= 3)) {

		flashOutput(30, 400, Pin_led);
		delay(60000);	// pause 1 min
		 String SleepInfo = "Sleep mode activation for 2 hours because no able to read girouette calibration";
		 int SleepTime = 120;
		 Serial.print("Sleep mode activation for 2 hours because no able to read girouette calibration");
		 Serial.print(SleepTime);
		 Serial.println(" minutes");
		 Sleep(SleepInfo, SleepTime);
    }
  }


  StopModem();
  const size_t capacity2 = JSON_OBJECT_SIZE(2) + 100;
  DynamicJsonBuffer jsonBuffer2(capacity2);

  JsonObject& root2 = jsonBuffer2.parseObject(CalValue);
  if (!root2.success()) {
    Serial.println("parseObject() failed");
  }
  String N_cal = root2["N"];
  String NNE_cal = root2["NNE"];
  String NE_cal = root2["NE"];
  String ENE_cal = root2["ENE"];
  String E_cal = root2["E"];
  String ESE_cal = root2["ESE"];
  String SE_cal = root2["SE"];
  String SSE_cal = root2["SSE"];
  String S_cal = root2["S"];
  String SSO_cal = root2["SSO"];
  String SO_cal = root2["SO"];
  String OSO_cal = root2["OSO"];
  String O_cal = root2["O"];
  String  ONO_cal = root2["ONO"];
  String NO_cal = root2["NO"];
  String  NNO_cal = root2["NNO"];

  ContBncLim = root2["ContBncLim"];
  Serial.print("------>  ContBncLim  = ");
  Serial.println(ContBncLim);
  PwrLim = root2["PwrLim"];
  Serial.print("------>  Power Limit threshold  = ");
  Serial.println(PwrLim);
  ResetDelay = root2["ResetDelay"];
  Serial.print("------>  ResetDelay  = ");
  Serial.println(ResetDelay);


  StringArray[0] = N_cal;
  StringArray[1] = NNE_cal;
  StringArray[2] = NE_cal;
  StringArray[3] = ENE_cal;
  StringArray[4] = E_cal;
  StringArray[5] = ESE_cal;
  StringArray[6] = SE_cal;
  StringArray[7] = SSE_cal;
  StringArray[8] = S_cal;
  StringArray[9] = SSO_cal;
  StringArray[10] = SO_cal;
  StringArray[11] = OSO_cal;
  StringArray[12] = O_cal;
  StringArray[13] = ONO_cal;
  StringArray[14] = NO_cal;
  StringArray[15] = NNO_cal;
  Serial.print("calibration value for North :");
  Serial.println(StringArray[0]);

  // ---------Initialise counter
  counter = millis() / 1000;
  counter_4s = counter;
  counter_300s = counter;

  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch



  // Define the interval for the reset counter in milliseconds
  counterInterval = ResetDelay * 60 * 60 * 1000; //  hours in milliseconds
  Serial.print("The board will auto-reset every ");
  Serial.print(ResetDelay);
  Serial.println(" hour");

  flashOutput(6, 1000, Pin_led);






}

//----------------------------------------------------------------------------------------------------------//
//---------------------------------------      MAIN LOOP     -----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------//






void loop() {


  digitalWrite(Pin_Enable, HIGH); // Active pin which give power to davis6410
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
    SerialMon.println("Set Pin reset to high for 10 sec ");
    // Set the output pin HIGH
//     digitalWrite(Pin_Reset, HIGH);
//     Sleep("reboot auto because no hardware reset", 1);
    // Wait for a short duration (e.g., 1 second) to ensure a noticeable pulse
//     delay(10000);
//     SerialMon.print("Set Pin reset to off");  // Should not be executed because PinREset the module just before
//     digitalWrite(Pin_Reset, LOW);
  }


  // ----------------- Loop every 5 minutes ----------------------------//




  delta = counter - counter_300s;



  if (delta > 300) {


    digitalWrite(Pin_led, HIGH);
    // -- Read sleeping mode
    Serial.println("-->  Init Modem for GETTING Sleep info ");
    InitModem();
    Serial.println("-->  Start JSONFromWeb for Sleep info ");
    String RequestData =      "api_key=" + apiKeyValue +
                              "";
    sleepstatus = JSONFromWeb(RequestData, serverSleepInfo);
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
      String RequestData =      "api_key=" + apiKeyValue +
                                "&SleepInfo=" + String(SleepInfo) +
                                "&SleepTime=" + String(SleepTime) +
                                "";
      String SendStatus = "";
      int flag_reboot = 0;
       SendStatus = SendInfoToWeb(RequestData, resource);
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
      String RequestData =      "api_key=" + apiKeyValue +
                                "&N=" + String(N) +
                                "&NNE=" + String(NNE) +
                                "&NE=" + String(NE) +
                                "&ENE=" + String(ENE) +
                                "&E=" + String(E) +
                                "&ESE=" + String(ESE) +
                                "&SE=" + String(SE) +
                                "&SSE=" + String(SSE) +
                                "&S=" + String(S) +
                                "&SSO=" + String(SSO) +
                                "&SO=" + String(SO) +
                                "&OSO=" + String(OSO) +
                                "&O=" + String(O) +
                                "&ONO=" + String(ONO) +
                                "&NO=" + String(NO) +
                                "&NNO=" + String(NNO) +
                                "&speed_max=" + String(speed_max) +
                                "&speed_min=" + String(speed_min) +
                                "&speed_avg=" + String(speed_avg) +
                                "&sensor_avg=" + String(sensor_avg) +
                                "&VoltageBatteryRealVal=" + String(VoltageBatteryRealVal) +
                                "&SleepInfo=" + String(SleepInfo) +
                                "&SleepTime=" + String(SleepTime) +
                                "&livetime_min=" + String(livetime_min) +
                                "";
      String SendStatus = "";
      int flag_reboot = 0;
      while ((SendStatus == "") && (flag_reboot < 5)) {
        //        Serial.println("----> Init Modem to SEND data to webpage ");
        //        InitModem();
        Serial.print("----> Start POST to send wind data info, test ");
        Serial.print(i);
        Serial.println(" ...");
        SendStatus = SendInfoToWeb(RequestData, resource);
        Serial.println("---->  POST ended... ");
        Serial.print("---->  result =  ");
        Serial.println(SendStatus);
        flag_reboot++;
        if (flag_reboot >= 3) {
// 			  Serial.print("reboot device by disable pin EN ");
// 			  digitalWrite(Pin_Reset, HIGH);
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
      Serial.println("-->  Data has been sent successfully");
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


String SendInfoToWeb(String httpRequestData, char *webpage) {
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    delay(3000);
    return "";
  }
  else {
    Serial.println(" OK");
    Serial.print("Send Info to Web: ");
    Serial.print(server);
    Serial.print(" on webpage: ");
    Serial.println(webpage);
    if (!client.connect(server, port)) {
      Serial.println(" fail");
      return "";
    }
    else {

      Serial.print("httpRequestData: ");
      Serial.println(httpRequestData);
      client.print(String("POST ") + webpage + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);
      resp_msg = "";
      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 10000) {
        // Print available data (HTTP response from server)

        while (client.available()) {
          char c = client.read();
          resp_msg += c;
          timeout = millis();
        }
      }
      // Close client and disconnect
      client.stop();
      SerialMon.println(F("Server disconnected"));
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
      return "Ok";

    }

  }
}

//---------------  FUNCTION -----------------------------------------------------------------------//

String JSONFromWeb(String httpRequestData, char *webpage) {
  Serial.println("--------> start JSONFromWeb function");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("-------->  fail");
    //    delay(2000);
    return "";
  }
  else {
    Serial.println("-------->  OK");
    Serial.print("--------> Connecting to ");
    Serial.print(server);
    if (!client.connect(server, port)) {
      Serial.println("-------->  fail");
      return "";
    }
    else {
      Serial.println("-------->  OK");
      // Making an HTTP POST request
      // SerialMon.println("Read JSON from webpage:");
      // SerialMon.println(webpage);
      // Serial.print("httpRequestData: ");
      // Serial.println(httpRequestData);

      client.print(String("GET ") + webpage + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);
      timeout = millis();
      resp_msg = "";
      while (client.connected() && millis() - timeout < 10000) {
        // Print available data (HTTP response from server)
        // Serial.print("test2");

        while (client.available()) {
          c = client.read();
          resp_msg += c;
          timeout = millis();
          //  Serial.print("a");
        }
      }

      //Serial.println("-----++++++++++----client.read resutat =");
      //Serial.println(resp_msg);
      //Serial.println("-----++++++++++----client.read ENDING");
      JSON_msg = "";
      posA = resp_msg.indexOf("{");
      posB = resp_msg.indexOf("}");
      posB = posB + 1;
      lenght = resp_msg.length();
      JSON_msg = resp_msg.substring(posA, posB);
      return JSON_msg;

      // Close client and disconnect
      client.stop();
      SerialMon.println(F("--------> Server disconnected"));
      modem.gprsDisconnect();
      SerialMon.println(F("--------> GPRS disconnected"));

    }

  }
}

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
//    SendStatus = SendInfoToWeb(RequestData, resource);
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
  diffarray[0] = abs(sensorValue - StringArray[0].toInt());
  diffarray[1] = abs(sensorValue - StringArray[1].toInt());
  diffarray[2] = abs(sensorValue - StringArray[2].toInt());
  diffarray[3] = abs(sensorValue - StringArray[3].toInt());
  diffarray[4] = abs(sensorValue - StringArray[4].toInt());
  diffarray[5] = abs(sensorValue - StringArray[5].toInt());
  diffarray[6] = abs(sensorValue - StringArray[6].toInt());
  diffarray[7] = abs(sensorValue - StringArray[7].toInt());
  diffarray[8] = abs(sensorValue - StringArray[8].toInt());
  diffarray[9] = abs(sensorValue - StringArray[9].toInt());
  diffarray[10] = abs(sensorValue - StringArray[10].toInt());
  diffarray[11] = abs(sensorValue - StringArray[11].toInt());
  diffarray[12] = abs(sensorValue - StringArray[12].toInt());
  diffarray[13] = abs(sensorValue - StringArray[13].toInt());
  diffarray[14] = abs(sensorValue - StringArray[14].toInt());
  diffarray[15] = abs(sensorValue - StringArray[15].toInt());

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
  //Serial.println(StringArray[0]);
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


  // Reset pin high
  digitalWrite(MODEM_RST, HIGH);

  // Turn on the Modem power first
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Pull down PWRKEY for more than 1 second according to manual requirements
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(200);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1200);
  digitalWrite(MODEM_PWRKEY, HIGH);


  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);

  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  //  modem.restart();

  modem.init();
  //modem.stop();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    Serial.println("Unlock SIM...");
    modem.simUnlock(simPIN);
  }
  // modem.gprsConnect(apn, gprsUser, gprsPass);
  Serial.println(modem.getSimStatus());
  String modemInfo = modem.getModemInfo();
  String modemName = modem.getModemName();
  SerialMon.print(F("Connecting to "));
  SerialMon.println(apn);
  SerialMon.print("modem info ");
  SerialMon.println(modemInfo);
  SerialMon.print("modem name ");
  SerialMon.println(modemName);
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
