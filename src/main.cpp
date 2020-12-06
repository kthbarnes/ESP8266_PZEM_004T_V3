#include <ArduinoOTA.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include "settingsPZEM.h"
#include <ArduinoJson.h>


#include <SoftwareSerial.h>  //  ( NODEMCU ESP8266 )
SoftwareSerial pzem(14, 12); // (RX,TX) connect to TX,RX of PZEM for NodeMCU
//SoftwareSerial pzem(D7,D8);  // (RX,TX) connect to TX,RX of PZEM
#include <ModbusMaster.h>
ModbusMaster node;
SimpleTimer timer;

int timerTask1;
double U_PR, I_PR, P_PR, PPR, PR_F, PR_PF, PR_alarm;
uint8_t result;
uint16_t data[6];

void setup()
{
  Serial.begin(115200);
  Serial.println("Start serial");
  pzem.begin(9600);
  Serial.println("Start PZEM serial");
  node.begin(1, pzem);
  Serial.println("Start PZEM"); // 1 = ID MODBUS
  


  

  //  timerTask1 = timer.setInterval(1000, updateBlynk);
}

void loop()
{

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

  Serial.print("Voltage: ");
  Serial.print(U_PR); // V
  Serial.println(" V");


  Serial.print("Current: ");
  Serial.print(I_PR, 3); //  A
  Serial.println(" A");
  
  Serial.print("Power: ");
  Serial.print(P_PR); //  W
  Serial.println(" W");
  
  Serial.print("Energy: ");
  Serial.print(PPR, 3); // kWh
  Serial.println(" KWh");
  
  
  Serial.print("Frequency: ");
  Serial.print(PR_F); // Hz
  Serial.println(" Hz");
  
  Serial.print("Power Factor: ");
  Serial.println(PR_PF);
  
  Serial.println("====================================================");

  StaticJsonDocument<200> doc;
  
  doc["Voltage"] = U_PR;
  doc["Current"] = I_PR;
  doc["Power"] = P_PR;
  doc["Energy"] = PPR;
  doc["Frequency"] = PR_F;

  serializeJsonPretty(doc, Serial);

  



  delay(1000);
}