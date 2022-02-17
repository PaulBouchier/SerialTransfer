#include "SerialTransfer.h"


SerialTransfer myTransfer;

struct STRUCT {
  char z;
  float y;
} __attribute__((packed)) testStruct;


void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  myTransfer.begin(Serial1);
}


void loop()
{
  if(myTransfer.available())
  {
    myTransfer.rxObj(testStruct);
    Serial.print(testStruct.z);
    Serial.println(testStruct.y);
  }
}
