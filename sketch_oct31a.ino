#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
 
#define APSSID "test001"
#define APPSK  "19360211620"
 
/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;
#define MAX_SRV_CLIENTS 1
 
WiFiServer server(8082);
WiFiClient serverClients[1];
 
String comdata;
 
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);
  // WiFi.mode(WIFI_AP);
 
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
 
  server.begin();
  server.setNoDelay(true);
  Serial.println("Server started");
  // digitalWrite(LED_BUILTIN,HIGH);
}

void loop(){
  uint8_t i;
  //检测服务器端是否有活动的客户端连接
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //查找空闲或者断开连接的客户端，并置为可用
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial.print("New client: "); Serial.println(i);
        continue;
      }
    }
    //若没有可用客户端，则停止连接
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
 
  //检查客户端的数据
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      if(serverClients[i].available()){
        //从Telnet客户端获取数据，并推送到URAT端口
        while(serverClients[i].available()){
          comdata += char(serverClients[i].read());
          delay(2);         
        }
      }
    }
  }
 
  if(comdata.length() > 0){    
    comdata.trim();
        Serial.print(comdata);
    if (comdata.endsWith("A")){
      digitalWrite(LED_BUILTIN,LOW);
      // Serial.println("RELAY OFF");
      // Serial.write("A");
    }
    if (comdata.endsWith("a")){
      digitalWrite(LED_BUILTIN,HIGH);
      // Serial.println("RELAY ON");
      // Serial.write("a");
    }
    comdata = "";
  }  
  //  yield();
}
