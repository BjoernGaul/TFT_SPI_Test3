#include <Arduino.h>

uint16_t posStep = 5;
extern void onDataReceive();


int posADD(int pos)
{
  if(pos < 270)
  {
    pos+=posStep;
  }
  return pos;
}

int posSUB(int pos)
{
  if(pos > 0)
  {
    pos-=posStep;
  }
  return pos;
}

int readJoystick(int xPin,int yPin)
{
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);
  
  if(xValue < 500)
  {
    Serial.println("Left");
    return 1;
  }
  else if(xValue > 3500)
  {
    Serial.println("Right");
    return 2;
  }
  else if(yValue < 500)
  {
    Serial.println("Up");
    return 3;
  }
  else if(yValue > 3500)
  {
    Serial.println("Down");
    return 4;
  }
}