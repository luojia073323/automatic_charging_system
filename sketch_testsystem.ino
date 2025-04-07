#include <IRremote.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <string.h>

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte data_buf[16];
int i;

int Ix[4];
int Iy[4];
int s;
unsigned long startTime = 0; 
bool currentDetected = false;

IRsend irsend;

SoftwareSerial mySerial(13, 12);          //RX,TX连ESP的TX，RX
String comdata;

//battery
SoftwareSerial batterySerial(8, 7);       //RX, TX连TTL的TX，RX
unsigned char g[] = {0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77};
String data = "";
float getVoltage(String voltage);
float getCurrent(String current);
int getMargin(String margin);


void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

void setup()
{   
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only 
    }
    Serial.println("Hello World");
    
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(9600);
    Wire.begin();
    // IR sensor initialize
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
    
    //ESP
    mySerial.begin(115200);
    batterySerial.begin(9600);              //9600
    mySerial.println("AT");
    delay(2000);
    mySerial.println("AT+CWMODE=1");
    delay(1500);    
    // while (mySerial.readString().indexOf("OK")==NULL)
    // {
    // Serial.println("error");
    // }
    // Serial.println("success");
    mySerial.println("AT+RST");
    delay(2000); 

    mySerial.println("AT+CWJAP=\"test001\",\"19360211620\"");
    delay(3000);

    mySerial.println("AT+CIPMODE=0");
    delay(1500);   
    mySerial.println("AT+CIPMUX=1");
    delay(2000);
    connectToServer(0,"192.168.4.1", 8082);
    connectToServer(1,"172.20.10.4", 8081);

    mySerial.println("AT+CIPSEND=0,1");
    delay(2000);
    // mySerial.println("AT+CIPSEND=1,8");
    // delay(2000);

    // while (mySerial.readString().indexOf("OK")==NULL){}
    // mySerial.println("AT+CIPSTART=\"TCP\",\"172.20.10.4\",8081");
    // delay(3500);

    // while (mySerial.readString().indexOf("OK")==NULL){}
    // Serial.println("linked");
    // mySerial.println("AT+CIPMODE=1");
    // delay(1500);
    
    // while (mySerial.readString().indexOf("OK")==NULL){}
    // // mySerial.println("AT+CIPSTATUS");
    // // Serial.println("good"); 
    //   mySerial.println("AT+CIPSEND");
    //   delay(2000);

}

void loop()
{
    // irsend.sendRC5(0x0, 8); //send 0x0 code (8 bits)
    // delay(200);

    // irsend.sendRC5(0x1, 8);
    // delay(200);

    //IR sensor read
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

    Wire.requestFrom(slaveAddress, 4);        // Request the 2 byte heading (MSB comes first)
    for (i=0;i<4;i++) { data_buf[i]=0; }
    i=0;
    while(Wire.available() && i < 4) { 
        data_buf[i] = Wire.read();
        i++;
    }

    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    // if( getCurrent(data) = 0)
    // {
      for(i=0; i<1; i++)
      {
        if (Ix[i] < 1000)
          Serial.print("");
        if (Ix[i] < 100)  
          Serial.print("");
        if (Ix[i] < 10)  
          Serial.print("");
          Serial.print( int(Ix[i]) );
          Serial.print(",");
        if (Iy[i] < 1000)
          Serial.print("");
        if (Iy[i] < 100)  
          Serial.print("");
        if (Iy[i] < 10)  
          Serial.print("");
          Serial.print( int(Iy[i]) );
          // if (i<1)
          //   Serial.print(",");
       // }
      Serial.println("");
      delay(15);
      }
    // else 
    // {
    //   Serial.println("0,0");
    // }

    
    //ESP
     if( ((Ix[0] != 1023) || (Iy[0] != 1023)) && getCurrent(data) = 0 )
    {
      while (mySerial.readString().indexOf(">")==NULL){}
      mySerial.print("a");

    }
    //if(((Ix[0] = 1023) && (Iy[0] = 1023)) || getMargin(data) > 99) 
    //  if(((Ix[0] = 1023) && (Iy[0] = 1023))) 
    // {
    //   // mySerial.println("AT+CIPSEND");
    //   // delay(1000);
    //   // while (mySerial.readString().indexOf(">")==NULL){}
    //   // Serial.println("send ok2");
    //   mySerial.println("A");
    //   delay(5000);
    // }


    //battery
    batterySerial.write(g,7);
    delay(500);

    data = "";
    while(batterySerial.available())
    {
      unsigned char r = (unsigned char)batterySerial.read();
      data += r;
      data += ',';
    }   

    // if (data.length() > 0) { 
      // Serial.println();
      // Serial.println(data);    
      // Serial.print("Voltage:");
      // Serial.println(getVoltage(data));
      // Serial.print("Current:");
      // Serial.println(getCurrent(data));
      // Serial.print("Margin:");
      // Serial.println(getMargin(data));
                          // }    
      mySerial.println("AT+CIPSEND=1,16");
      delay(2000);
      while (mySerial.readString().indexOf(">")==NULL){}

      mySerial.println();
      mySerial.print(getVoltage(data));
      mySerial.print(",");
      mySerial.print(getCurrent(data));
      mySerial.print(",");
      mySerial.print(getMargin(data));
      delay(5000);            
    
}

//function
float getVoltage(String temp) 
{
int myPosition = -1;
String info[38];                   //38字节
for (int j = 0; j < 38; j++) {
  myPosition = temp.indexOf(',');
  if (myPosition != -1)
  {
    info[j] = temp.substring(0, myPosition);
    temp = temp.substring(myPosition + 1, temp.length());
  }
  else {
    if (temp.length() > 0) { 
      info[j] = temp.substring(0, myPosition);
    }
      }         
                             }
return (info[4].toInt() * 256 + info[5].toInt()) / 100.0;
}

float getCurrent(String temp) {
int myPosition = -1;
String info[38]; 
for (int j = 0; j < 38; j++) {
myPosition = temp.indexOf(',');
if (myPosition != -1)
{
info[j] = temp.substring(0, myPosition);
temp = temp.substring(myPosition + 1, temp.length());
}
else {
if (temp.length() > 0) { 
info[j] = temp.substring(0, myPosition);
                       }
    }
                             }
return (65536-(info[6].toInt() * 256 + info[7].toInt())) / 100.0;
}

int getMargin(String temp) {
int myPosition = -1;
String info[38]; 
for (int j = 0; j < 38; j++) {
myPosition = temp.indexOf(',');
if (myPosition != -1)
{
info[j] = temp.substring(0, myPosition);
temp = temp.substring(myPosition + 1, temp.length());
}
else {
if (temp.length() > 0) { 
info[j] = temp.substring(0, myPosition);
                       }
    }
                             }
return (info[23].toInt());
}

void connectToServer(int id, const char* serverIP, int serverPort) 
{
  while (mySerial.readString().indexOf("OK")==NULL){}
  mySerial.println("AT+CIPSTART=" +  String(id) + ",\"TCP\",\"" + String(serverIP) + "\"," + String(serverPort));
  delay(3000);

  // while (mySerial.readString().indexOf("OK")==NULL){}
  // Serial.println("linked");
  // mySerial.println("AT+CIPMODE=1");
  // delay(1500);
    
  // while (mySerial.readString().indexOf("OK")==NULL){}
  // // mySerial.println("AT+CIPSTATUS");
  // // Serial.println("good"); 
  // mySerial.println("AT+CIPSEND");
  // delay(2000);
}
  if(getCurrent(data) > 0)
  {
    if (!currentDetected) 
    {
      startTime = millis();  
      currentDetected = true;
    }
  }
  
  if (millis() - startTime > 8000 && currentDetected && getCurrent(data) = 0)       
  {  
       mySerial.print("a");                 // 闭合继电器
  }
  else 
  {
    currentDetected = false;               // 重置电流检测标记
    mySerial.print("A");                   // 继电器断开
  }
