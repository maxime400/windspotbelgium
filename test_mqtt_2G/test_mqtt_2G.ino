/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * Use Mosquitto client tools to work with MQTT
 *   Ubuntu/Linux: sudo apt-get install mosquitto-clients
 *   Windows:      https://mosquitto.org/download/
 *
 * Subscribe for messages:
 *   mosquitto_sub -h test.mosquitto.org -t GsmClientTest/init -t GsmClientTest/ledStatus -q 1
 * Toggle led:
 *   mosquitto_pub -h test.mosquitto.org -t GsmClientTest/led -q 1 -m "toggle"
 *
 * You can use Node-RED for wiring together MQTT-enabled devices
 *   https://nodered.org/
 * Also, take a look at these additional Node-RED modules:
 *   node-red-contrib-blynk-ws
 *   node-red-dashboard
 *
 **************************************************************/

// Please select the corresponding model

#define SIM800L_IP5306_VERSION_20190610
// #define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

#include "utilities.h"

// Select your modem:
#define TINY_GSM_MODEM_SIM800

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Add a reception delay - may be needed for a fast processor at a slow baud rate
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN "1503"

// Your GPRS credentials, if any
const char apn[] = "TM";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* mqtt_server = "windspotbelgium.com";
const int mqtt_port = 1883; // Default MQTT port
const char* topic_data = "FBVL0/wind_data";
const char* topic_calibration = "FBVL0/calibration";

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


// Variable to store incoming message
String incomingMessage;
int counter = 0;

uint32_t lastReconnectAttempt = 0;


void callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages if needed
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  incomingMessage = ""; // Clear the previous one
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    incomingMessage += (char)payload[i];
  }
  Serial.println();
  // Assuming payload is a string, convert it to an integer

}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("ESP32Client")) {
      Serial.println("connected");

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup()
{
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

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);

}

void loop() {

    
  //--------------- WRITE MQTT data -------------------
  // Read MQTT data
  if (!mqtt.connected()) {
    reconnect();
  }
  incomingMessage = "";
  counter = 0;
  SerialMon.print("read mqtt data...");
  while ((incomingMessage == "") && (counter < 20)){
    SerialMon.print(".");
    mqtt.subscribe(topic_calibration);
    mqtt.loop();
    delay(500);   
    counter++;   
  }
  SerialMon.println(".");
  if (counter >= 20){
    SerialMon.println("mqtt subscribe topic_calibration connection issue --> goes to sleep");
  }
    
  SerialMon.print("inComing message = ");
  SerialMon.println(incomingMessage);
  //--------------------------------------------------
  //--------------- WRITE MQTT data -------------------
  if (!mqtt.connected()) {
    reconnect();
  }
  // Convert sensor value to string
  SerialMon.print("write mqtt data...");
  counter = 0;
  boolean success = false;
  String data = String("test1,rest2");
  // Publish the data to the MQTT topic
//  SerialMon.print("publish data to ");
//  SerialMon.println(topic_data);
//  SerialMon.print("data:  ");
//  SerialMon.println(data.c_str());
 // Attempt to publish data
   while ((success == false) && (counter < 20)){
        SerialMon.print(".");
        if (mqtt.connected()) {
          success = mqtt.publish(topic_data, data.c_str());
          if (success) {
            Serial.println("Data published successfully");
          } else {
            Serial.print(".");
          }
        }
        delay(1000);
        counter++;
  }
  SerialMon.println(".");
  if (counter >= 20){
    SerialMon.println("mqtt publish topic_data connection issue --> goes to sleep");
  }



  //--------------------------------------------------
  
  // Wait for some time before publishing again
  SerialMon.println("wait 30s...");
  delay(30000); // Adjust delay as needed
}
