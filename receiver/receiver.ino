
/*
 * GND------------------------------GND  
 * D11------------------------------Data
 * 5V-------------------------------VCC
 */


#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

unsigned long parse_long (uint8_t* num , int start_byte ,int BytesCount);
int parse_int_char(const char ch);

RH_ASK driver;
// RH_ASK driver(2000, 2, 4, 5); // ESP8266: do not use pin 11

double Latitude, Longitude;
bool Reading_updated = false;

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    uint8_t buf[50];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      unsigned long  big_lat = parse_long (buf , 0 ,9);
      unsigned long  big_lon = parse_long (buf , 10 ,9); 
      Latitude = (double)big_lat  / 10000000;
      Longitude = (double)big_lon / 10000000;
      Reading_updated = true;
    }
    
    if(Reading_updated){
      Reading_updated = false;
      Serial.print("Latitude : ");  Serial.print(Latitude,8);
      Serial.print("\t Longitude : ");  Serial.print(Longitude,8);
      Serial.println();
    }
}


unsigned long parse_long (uint8_t* num , int start_byte ,int BytesCount){

  bool isNum = false;
  unsigned long result = 0;
  bool is_negative = false;
  int digit=0;
  for (int i=start_byte ; i < (BytesCount+start_byte) ; i++)
  {
    char c = num[i];
//    Serial.print(c); Serial.print(",");
    if (c=='-')
    {
      is_negative=true;
    }
    else
    {
      int n = parse_int_char(c);
      if (n>-1)
       {
         result= (result*10)+n;
         isNum = true;
       }
      else
        if (isNum)    break;
    }
  }

  if (is_negative) result*=-1;

  return result;

}


int parse_int_char(const char ch){

  if (ch=='0')
    return 0;
  else if (ch=='1')
    return 1;
  else if (ch=='2')
    return 2;
  else if (ch=='3')
    return 3;
  else if (ch=='4')
    return 4;
  else if (ch=='5')
    return 5;
  else if (ch=='6')
    return 6;
  else if (ch=='7')
    return 7;
  else if (ch=='8')
    return 8;
  else if (ch=='9')
    return 9;

  else return -1;
}
