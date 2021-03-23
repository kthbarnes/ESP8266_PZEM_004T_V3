#include <ArduinoOTA.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include "settingsPZEM.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <SoftwareSerial.h>  //  ( NODEMCU ESP8266 )
SoftwareSerial pzem(14, 12); // (RX,TX) connect to TX,RX of PZEM for NodeMCU | D5-> 14 | D6-> 12|
//SoftwareSerial pzem(D7,D8);  // (RX,TX) connect to TX,RX of PZEM
#include <ModbusMaster.h>
ModbusMaster node;
SimpleTimer timer;

int timerTask1;
double U_PR, I_PR, P_PR, PPR, PR_F, PR_PF, PR_alarm; // PZEM registers
uint8_t result;
uint16_t data[6];

int analogPin = A0;
float Vo = 0;
float Vin = 0;
float Vr1 = 0;
float Iin = 0;
float Pin = 0;
float Vinit = 0;

const char *ssid = "";
const char *password = "";

const char *mqttServer = "35.192.157.47"; //mqtt server IP
const int mqttPort = 1883;                // Mqtt port number
const char *mqttUser = "";
const char *mqttPassword = "";

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200);
  Serial.println("Start serial");
  pzem.begin(9600);
  Serial.println("Start PZEM serial");
  node.begin(1, pzem);
  Serial.println("Start PZEM"); // 1 = ID MODBUS

  WiFi.begin(ssid, password);

  // wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  // mqtt connection
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {

      Serial.println("connected");
    }
    else
    {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void reconnect_mqtt()
{
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {

      Serial.println("connected");
    }
    else
    {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void reconnect_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

void loop()
{
  if (!client.connected())
  {
    // If we're not, attempt to reconnect to MQTT Broker
    reconnect_mqtt();
  }
  
  if (WiFi.status() != WL_CONNECTED)
  {
    // If we're not, attempt to reconnect to MQTT Broker
    reconnect_wifi();
  }
  
  // read calulated values from PZEM
  result = node.readInputRegisters(0x0000, 10);
  if (result == node.ku8MBSuccess)
  {
    U_PR = (node.getResponseBuffer(0x00) / 10.0f);
    I_PR = (node.getResponseBuffer(0x01) / 1000.000f);
    P_PR = (node.getResponseBuffer(0x03) / 10.0f);
    PPR = (node.getResponseBuffer(0x05) / 1000.0f);
    PR_F = (node.getResponseBuffer(0x07) / 10.0f);
    PR_PF = (node.getResponseBuffer(0x08) / 100.0f);
    PR_alarm = (node.getResponseBuffer(0x09));
  }
   
  Vinit = analogRead(analogPin);
  Vo = Vinit * (3.3 / 1023.0);
  Vin = Vo * 7.1;
  Vr1 = Vin - Vo;
  Iin = Vr1 / 180;
  Pin = Vin * Iin;
  
  // convert values to JSON format so that we can send a single json file to the broker

  StaticJsonDocument<200> JSONbuffer;
  JSONbuffer["Energy"] = PPR;
  JSONbuffer["Power"] = P_PR;
  JSONbuffer["Voltage"] = U_PR;
  JSONbuffer["Current"] = I_PR;
  JSONbuffer["Pin"] = Pin;



  serializeJsonPretty(JSONbuffer, Serial); // print json on serial monitor
  Serial.println("");

  // creating a string - buffer , since only string can be published to mqtt
  char buffer[256];
  serializeJson(JSONbuffer, buffer);
  client.publish("esp/pzem", buffer);

  if (client.publish("esp/pzem", buffer) == true)
  {
    Serial.println("Success sending message");
  }
  else
  {
    Serial.println("Error sending message");
  }

  delay(10000);
}