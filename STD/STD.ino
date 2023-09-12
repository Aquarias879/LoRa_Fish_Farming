#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <LoRa.h>
#include <ArduinoUniqueID.h>
#include <Wire.h>
#define ARDUINOJSON_ENABLE_STRING_DEDUPLICATION 1
#include <ArduinoJson.h>
#include "DFRobot_ORP_PRO.h"
#include "DFRobot_EC10.h"
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

#define DO_PIN A0
#define PIN_ORP A1
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution

#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (0) //Current water temperature ℃, Or temperature sensor function

#define CAL1_V (550) //mv
#define CAL1_T (25)   //℃

#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

#define SS_PIN 4
#define RST_PIN 3
#define DIO0_PIN 2

#define SLAVE_ID 1
#define SERIAL_BAUDRATE 9600
ModbusMaster modbusNode;

#define RX_PIN 6
#define TX_PIN 5
#define SOFTWARE_SERIAL_BAUDRATE 9600
SoftwareSerial softwareSerial(RX_PIN, TX_PIN);

#define EC_PIN A2

// Set the LCD address (0x27 for most common I2C LCD)
LiquidCrystal_I2C lcd(0x27, 20, 2);


const int batteryPin = A3; // Analog input pin connected to battery voltage
float batteryVoltage; // To store the battery voltage value
int batteryLevel; // To store the battery level percentage

float voltage, ecValue;
float conversionFactor = 0.5;
DFRobot_EC10 ec;
bool calibrationMode = false;
String jsonString;

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
unsigned int ADC_ORP_voltage;
uint16_t DO;
float ORP_VAL, practical_salinity, DOValue,salinity,temperature,pH,ad,SalinityValue,ECValue,ph;
int DO_Temp, DO_Raw, DO_Voltage;


String payload;
String ArduinoID = "";
String uniqueID = "";

String outgoing, Mymessage;
byte msgCount = 0;
byte Node2 = 0xCC;
byte MasterNode = 0xFF;
bool state = false;
bool calibrate = false;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)temperature_c * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

DFRobot_ORP_PRO ORP(-2480);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  lcd.init();  
  lcd.backlight();

  while (!Serial)
    ;
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
  }
  Serial.println("LoRa init succeeded.");
  //LoRa.setSpreadingFactor(8);
  softwareSerial.begin(SERIAL_BAUDRATE);
  modbusNode.begin(SLAVE_ID, softwareSerial);
  ec.begin();
}

void(* resetFunc) (void) = 0; //製造重啟命令 

void createJson()
{
  
  // Read temperature from the sensor
  temperature = readTemperature();
  // Read pH value from the sensor
  pH = readPH();
  // Read ORP value from the sensor
  ORP_VAL = readORP();
  // Read DO value from the sensor
  DOValue = readDOValue();
  // Read EC value from the sensor
  ECValue = readECValue();
  SalinityValue = ECValue * conversionFactor ;

  

  StaticJsonDocument<128> json;
  json["std_ID"] = STD_ID().substring(11, 16);
  json["temp"] = String(temperature);
  json["pH"] = String(pH);
  json["orp"] = String(ORP_VAL);
  json["do"] = String(DOValue);
  json["ec"] = String(ECValue);
  json["salinity"] = String(SalinityValue);

  // Serialize JSON to a string
 
  serializeJson(json, jsonString);

  return serializeJson;
}

void loop()
{
  onReceive(LoRa.parsePacket());
  //batt_percentage();
}

String STD_ID(){
  for (size_t i = 0; i < 8; i++)
  {
    if (UniqueID8[i] < 0x10)
    {
      uniqueID = 0 + String(UniqueID8[i], HEX);
    }
    else
    {
      uniqueID = String(UniqueID8[i], HEX);
    }
    ArduinoID += uniqueID;
  }
  return ArduinoID;
}

float readTemperature()
{
  uint8_t result;
  uint16_t data[3];
  result = modbusNode.readHoldingRegisters(0x00, 3);
  if (result == modbusNode.ku8MBSuccess)
  {
     float temperature = modbusNode.getResponseBuffer(0x00) / 100.0;
  }
  return temperature ;
}

float readPH()
{
  uint8_t result;
  uint16_t data[3];
  result = modbusNode.readHoldingRegisters(0x00, 3);
  if (result == modbusNode.ku8MBSuccess)
  {
    ad = modbusNode.getResponseBuffer(0x02);
    ph = (modbusNode.getResponseBuffer(0x01) + ad) / 100.0;
  }
  return ph;
}

float readORP()
{
  ADC_ORP_voltage = ((unsigned long)analogRead(PIN_ORP) * VREF + ADC_RES / 2) / ADC_RES;
  ORP_VAL = ORP.getORP(ADC_ORP_voltage);
  return ORP_VAL;
}

float readDOValue()
{
  DO_Raw = analogRead(DO_PIN);
  DO_Voltage = VREF * (float)DO_Raw / ADC_RES;
  DO_Temp = readTemperature();
  DOValue = readDO(DO_Voltage, DO_Temp);
  return DOValue;
}

float readECValue()
{
  float temperature = readTemperature();
  voltage = analogRead(EC_PIN) /  1024.0 * 5000;
  ecValue = ec.readEC(voltage, temperature); ;
  return ecValue;
}

void sendMessage(String outgoing,byte MasterNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(MasterNode);               // add destination address
  LoRa.write(Node2);                    // add sender address
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket(true);                     // finish packet and send it
}

void batt_percentage(){
  
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  byte recipient = LoRa.read();
  byte sender = LoRa.read();


  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();

  if (recipient != Node2 && recipient != MasterNode) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  Serial.println("Message: " + incoming);
  int Val = incoming.toInt();
  if (Val == 10 and sender == MasterNode)
  {
    createJson();
    Serial.println(jsonString);
    sendMessage(jsonString,MasterNode);
    delay(500);

    resetFunc(); //重啟程序開始
    //state = true;
  }
  if (incoming == "CALEC" and calibrate == false){
    ec.calibration(voltage, temperature,"ENTEREC");
    delay(5000);
    Serial.print(payload);
    sendMessage(payload,MasterNode);
    delay(5000);
    ec.calibration(voltage, temperature,"CALEC");
    delay(5000);
    ec.calibration(voltage, temperature,"EXITEC");
    delay(5000);
    ORP.getCalibration();
    Serial.print("calibration is: ");
    Serial.print(ORP.getCalibration());
    Serial.println("mV");
    delay(1000);
    calibrate = true;
    }
  }
}
