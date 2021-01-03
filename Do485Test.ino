
#include "REG_SINTROL_203.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

String deviceToken = "iXo3cyAe9N6HpHrjw6OC";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";

String doVal = "";
String tempVal = "";
ModbusMaster node;


const long interval = 10000;  //millisecond
unsigned long previousMillis = 0;


float HexTofloat(uint32_t x) {
  return (*(float*)&x);
}

uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}
//------------------------------------------------

float Read_Meter_float(char addr , uint16_t  REG) {
  unsigned int i = 0;
  uint32_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  node.begin(addr, modbus);
  result = node.readHoldingRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
  delay(500);
//  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      Serial.println(data[j]);
    }

    doVal =  data[0];
    tempVal = data[1] ;

    Serial.println(doVal);
    Serial.println("Connec modbus Ok.");
    return i;
//  } else {
//    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
//    delay(1000);
//    return 0;
//  }
}

void getProbe() {     // Update read all data
  //  delay(1000);                              // เคลียบัสว่าง
  for (char i = 0; i < Total_of_Reg ; i++) {
    DATA_METER [i] = Read_Meter_float(ID_meter, Reg_addr[i]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
  }
}

//**************************************************************************************************************
void setup() {
  
  Serial.begin(115200);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println(F("Test DO Water Monitor"));
  // communicate with Modbus slave ID 1 over Serial (port 2)

  AISnb.debug = true;


  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();
  Serial.println();
  Serial.println();
  Serial.println(F("****************Test Disolved Oxegen*******************"));
}
//String getResult( unsigned int x_high, unsigned int x_low)
//{
//  String hex2 = "";
//
//  hex2.concat(x_high);
//  hex2.concat(x_low);
//  Serial.print("hex:");
//  Serial.println(hex2);
//  Serial.print("dec:");
//  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
//  return String(hexToDec(hex2));
//}
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    getProbe();

    String json = "";
    json.concat("{\"Tn\":\"");
    json.concat(deviceToken);
    json.concat("\",\"DO\":");
    json.concat(doVal);
    json.concat(",\"tem\":");
    json.concat(tempVal);
    json.concat("}");
    Serial.println(json);
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);

    previousMillis = currentMillis;

    UDPReceive resp = AISnb.waitResponse();
  }
}


String decToHex(int decValue) {

  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}
