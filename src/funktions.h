#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

uint16_t posStep = 5;

String intArraytoString(int intarray[2])
{
  int intArray[] = {1, 2, 3, 4, 5};
  String str = "";

  for (int i = 0; i < sizeof(intArray) / sizeof(intArray[0]); i++) {
      str += String(intArray[i]);
      if (i < sizeof(intArray) / sizeof(intArray[0]) - 1) {
          str += ", "; // Add a separator
      }
  }
  //Serial.println(str);
  return str;
}

int* stringToIntArray(String str)
{
  // Convert the string back to an integer array
  int startIndex = 0;
  int endIndex = str.indexOf(',');
  int* intArray = new int[2];
  int arrayIndex = 0;

  while (endIndex != -1) {
    intArray[arrayIndex++] = str.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 2; // Move past the comma and space
    endIndex = str.indexOf(',', startIndex);
  }
  intArray[arrayIndex] = str.substring(startIndex).toInt(); // Add the last number

  return intArray;
}

int readJoystick(int xPin,int yPin)
{
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);
  
  if((xValue < 1300))
  {
    Serial.println("Left");
    return 1;
  }
  else if((xValue > 2700))
  {
    Serial.println("Right");
    return 2;
  }
  else if((yValue > 2900))
  {
    Serial.println("Down");
    return 3;
  }
  else if((yValue < 1300))
  {
    Serial.println("Up");
    return 4;
  }else{
    return 0;
  }
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

void LoRa_rxMode(){
  //Serial.println("LoRa_rxMode");
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  //Serial.println("LoRa_txMode");
  LoRa.idle();  
  //Serial.println("idle done");          // set standby mode;
  LoRa.disableInvertIQ();              // normal mode
  //Serial.println("disableInvertIQ done");
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                       // set tx mode
  //Serial.println("Begin message");
  LoRa.beginPacket();                   // start packet
  //Serial.println("Begin packet done");
  LoRa.print(message);                  // add payload
  //Serial.println("Print done");
  LoRa.endPacket(false);                 // finish packet and send it
  //Serial.println("End packet done");
}

void sendJoystick(String joypos){

  Serial.println(joypos);

  LoRa_sendMessage(joypos);
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

void manageSend(uint8_t joyLeft, uint8_t joyRight)
{
  int msgArray[2];
  static uint8_t taskLeft = 0;
  static uint8_t taskRight = 0;
  if(!taskLeft)
  {
    msgArray[0] = 100;
    msgArray[1] = joyLeft;
    sendJoystick(intArraytoString(msgArray));
    taskLeft = joyLeft;
  }else if(taskLeft != joyLeft)
  {
    taskLeft = 0;
  }
  
  if(!taskRight)
  {
    msgArray[0] = 101;
    msgArray[1] = joyRight;
    sendJoystick(intArraytoString(msgArray));
    taskRight = joyRight;
  }else if(taskRight != joyRight)
  {
    taskRight = 0;
  }
}

