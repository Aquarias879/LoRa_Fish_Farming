#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

#define ss 15  //GPIO 15
#define rst 4  //GPIO 16
#define dio0 2  //GPIO 4

#define LED 21

const char* ssid = "JWISDOM";
const char* password = "062340161";

byte MasterNode = 0xFF;     // address of this device
byte Node1 = 0xBB;      // destination to send to
byte Node2 = 0xCC;
byte Node3 = 0xDD;

String SenderNode = "";
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
String incoming = "";
String httpgetdata = "";
bool state1 = false;
bool state2 = false;
bool state3 = false;

// Tracks the time since last event fired
unsigned long previousMillis = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs = 0;
unsigned long currentMillis = 0;
int interval = 1 ; // updated every 1 second
int Secs = 0;

int temperature;
int humidity;
int soilmoisturepercent;
int soilMoistureValue;

WiFiClient wifiClient;
HTTPClient http;

void wf_state() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}

void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  pinMode(LED, OUTPUT);

  Serial.println("LoRa Master Node");

  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (millis() > 10000) {
      Serial.println("Wi-Fi connection failed. Restarting...");
      ESP.restart();
    }
  }
  //LoRa.setSpreadingFactor(8);
  Serial.println("\nWi-Fi connected");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

void loop() {
  wf_state();
  currentMillis = millis();
  currentsecs = currentMillis / 1000;
  if ((unsigned long)(currentsecs - previoussecs) >= interval) {
    Secs = Secs + 1;
    if (Secs % 800 == 0)
    {
      String message = "10";
      Serial.println(message);
      sendMessage(message,MasterNode,Node2);
      delay(100);
      message = "";
      
    }
    if (Secs % 900 == 0)
    {
      String message = "20";
      Serial.println(message);
      sendMessage(message,MasterNode,Node1);
      delay(100);
      message = "";
      
    }
    if (Secs % 1000 ==0)
    {
      String message = "30";
      Serial.println(message);
      sendMessage(message,MasterNode,Node3);
      delay(100);
      message = "";
      
    }
    if (Secs % 500 == 0) {  // Check if it's been a month (30 days)
      getRequest();
    }
    if (Secs % 2592000 == 0) //200000
    {
      ESP.restart();
    }
    else
    { 
      Serial.println(Secs);
    }
    previoussecs = currentsecs;
  }
  
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  memset(&incoming, 0, sizeof(incoming));
}


void sendMessage(String outgoing, byte MasterNode ,byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(otherNode);              // add destination address
  LoRa.write(MasterNode);             // add sender address
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  byte recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }

  if (recipient != Node2 && recipient != MasterNode) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }

  if (recipient != Node3 && recipient != MasterNode) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message: " + incoming);
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, incoming);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  // Extract the values
  String dev = doc["std_ID"].as<String>();
  String temp = doc["temp"].as<String>();
  String pH = doc["pH"].as<String>();
  String orp = doc["orp"].as<String>();
  String do_v = doc["do"].as<String>();
  String ec = doc["ec"].as<String>();
  String salinity = doc["salinity"].as<String>();

  Serial.print("Dev: ");
  Serial.println(dev);
  Serial.print("Temperature: ");
  Serial.println(temp);
  Serial.print("pH: ");
  Serial.println(pH);
  Serial.print("DO: ");
  Serial.println(do_v);
  Serial.print("ORP: ");
  Serial.println(orp);
  Serial.print("EC: ");
  Serial.println(ec);
  Serial.print("Salinity: ");
  Serial.println(salinity);
  
  if (WiFi.status() == WL_CONNECTED){

    HTTPClient http;
    http.begin(wifiClient, "http://wqm.jw-app.com.tw/WaterQuality/DeviceSave");
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(incoming);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    http.end();
    //sendMessage(outgoing,MasterNode,Node1);
    //delay(10000);
  }
}

void getRequest(){
  if (WiFi.status() == WL_CONNECTED) {
    http.begin(wifiClient, "http://192.168.2.140:5000");
    int httpCode = http.GET();
    Serial.print("httpCode=");
    Serial.println(httpCode);
    if (httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
        if ( payload == "50e0d" ) //S/N last 5 digit
            {
              SenderNode = "Node1:";
              String outgoing = "CALEC";
              sendMessage(outgoing,MasterNode,Node1);
            }
        
        if ( payload == "e2815" ) //S/N last 5 digit
            {
              SenderNode = "Node2:";
              String outgoing = "CALEC";
              sendMessage(outgoing,MasterNode,Node2);
            }
      }
    } 
  }