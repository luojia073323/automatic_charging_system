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

IRsend irsend;
SoftwareSerial mySerial(13, 12);  //RX,TX
String comdata;
 
void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
    Serial.begin(115200);
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
    mySerial.println("AT");
    delay(1000);
    // Serial.println(mySerial.readString());
    delay(1000);
    mySerial.println("AT+CWMODE=1");
    delay(1000);    
    // Serial.println(mySerial.readString());
    while (mySerial.readString().indexOf("OK")==NULL){
    Serial.println("error");
    }
    // Serial.println("success");
    mySerial.println("AT+RST");
    delay(1500); 
    // Serial.println(mySerial.readString());
    mySerial.println("AT+CWJAP=\"test001\",\"19360211620\"");
    delay(3000);

    while (mySerial.readString().indexOf("OK")==NULL){Serial.println("error!!!");}
    Serial.println("yes");
    // Serial.println(mySerial.readString());
    mySerial.println("AT+CIPSTART=\"TCP\",\"192.168.4.1\",8082");
    delay(2000);

    while (mySerial.readString().indexOf("OK")==NULL){Serial.println("TCP error");}
    Serial.println("TCP LINKED");
    // Serial.println(mySerial.readString());
    mySerial.println("AT+CIPMODE=1");
    delay(1500);

    while (mySerial.readString().indexOf("OK")==NULL){}
    // mySerial.println("AT+CIPSTATUS");   
      mySerial.println("AT+CIPSEND");
      delay(2000); 
}
 
void loop() {

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

    //ESP
    //  if( ((Ix[0] != 1023) || (Iy[0] != 1023)) && getMargin(data) < 90)
    if( ((Ix[0] != 1023) || (Iy[0] != 1023)))
    {
      // mySerial.println("AT+CIPSEND");
      // delay(1000);
      while (mySerial.readString().indexOf(">")==NULL){}
      // Serial.println("send ok1");
      mySerial.println("a");
      delay(5000);
    }
    // if(((Ix[0] = 1023) && (Iy[0] = 1023)) || getMargin(data) > 99) 
    if(((Ix[0] = 1023) && (Iy[0] = 1023))) 
    {
      // mySerial.println("AT+CIPSEND");
      // delay(1000);
      while (mySerial.readString().indexOf(">")==NULL){}
      // Serial.println("send ok2");
      mySerial.println("A");
      delay(5000);
    }

    //test
    // while (mySerial.readString().indexOf(">")==NULL){}
    // // Serial.println("send ok1");
    // mySerial.println("A");
    // delay(5000);

    // while (mySerial.readString().indexOf(">")==NULL){}
    // // Serial.println("send ok2");
    // mySerial.println("a");
    // delay(5000);
}
