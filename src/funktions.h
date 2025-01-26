#include <Arduino.h>


uint16_t posStep = 5;

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
