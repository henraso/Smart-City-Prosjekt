
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// Timere for sending av data til Zumo
unsigned long sendTimer1 = millis();
unsigned long sendTimer2 = 0;


unsigned long previousMillis;
unsigned long currentMillis;

WiFiClient espClient;
PubSubClient client(espClient);


// SSID og Password for router
const char* ssid = "HENRIK-NETT";
const char* password = "84451F&r";
const char* mqtt_server = "gruppe26.kicks-ass.net";
WebServer server(80);

// All data som skal sendes til Zumo + array de skal lagres i.
int emgStatusToZumo = 0;
int enterChargeToZumo = 0;
int changeBatteryToZumo = 0;
int serviceBatteryToZumo = 0;
int addCashToZumo = 0;
int redLightToZumo = 0;
int sendToZumo[6];




// Verdier som sendes til MQTT
int sendValues[] = {0, 0, 0, 0, 0, 0};




void setup() {


  Serial.begin(9600);
// Serial for kommunikasjon med Zumo
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
// Starter opp MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  

  //Kobler til WiFi
  connectWiFi();
  
}
void loop() {

//MQTT

// Reconnecter MQTT hvis man disconnecter
  if(!client.connected()) {
    reconnectMQTT();
  }
  
  client.loop();
  Serial.print("FØR SEND  ");
  Serial.println(addCashToZumo);
//  Sender speedometerverdier til MQTT
  char sendSpeed[8];
  dtostrf(sendValues[0],1,2, sendSpeed);
  client.publish("esp32/speedometer", sendSpeed);

//  Sender batteriprosent
  char sendBattery[8];
  dtostrf(sendValues[1],1,2, sendBattery);
  client.publish("esp32/batterypercentage", sendBattery);
  

//  Totalavstand til MQTT
  char sendDistance[8];
  dtostrf(sendValues[2],1 , 2, sendDistance);
  client.publish("esp32/distance", sendDistance);

//  Siste minutts gjennomsnittshastighet til MQTT
  char sendLastMinAvg[8];
  dtostrf(sendValues[3], 1, 2, sendLastMinAvg);
  client.publish("esp32/lastminavg", sendLastMinAvg);
  
//  Batterihelse til MQTT
  char sendBatteryHealth[8];
  dtostrf(sendValues[4], 1, 2, sendBatteryHealth);
  client.publish("esp32/batteryhealth", sendBatteryHealth);

//  Oppdatering av saldo til MQTT
  char sendCurrentBalance[8];
  dtostrf(sendValues[5], 1, 2, sendCurrentBalance);
  client.publish("esp32/accountBalance", sendCurrentBalance);

 // Serial.println("after publish");


  //SERIAL TIL ZUMO
  sendToZumo[0] = emgStatusToZumo;
  sendToZumo[1] = enterChargeToZumo;
  sendToZumo[2] = changeBatteryToZumo;
  sendToZumo[3] = serviceBatteryToZumo;
  sendToZumo[4] = addCashToZumo;
  sendToZumo[5] = redLightToZumo;


  Serial.print(sendToZumo[0]); Serial.print("||||");
  Serial.print(sendToZumo[1]); Serial.print("||||");
  Serial.print(sendToZumo[2]); Serial.print("||||");
  Serial.print(sendToZumo[3]); Serial.print("||||");
  Serial.print(sendToZumo[4]); Serial.print("||||");
  Serial.println(sendToZumo[5]);


 // Serial.println("Before sending");
  
  //Serial.println(emgStatusToZumo);
// Sender data til Zumo via serial
  sendTimer1 = millis();
  if(sendTimer1>sendTimer2){
    //Serial.println("before for loop");
    sendTimer2 = sendTimer1 + 100;
    for(int j = 0; j < 6 ; j++){
      Serial1.print(sendToZumo[j]);
//      Serial.println(sendToZumo[1]);
      Serial1.print(">");
    }
  }


//  Serial.println("OUT OF FOR-LOOP");
//  Mottar data fra serial.
  int idx = 0;
  while(Serial1.available()>0){
    
    //Serial.println("IN WHILE");
    String incoming = Serial1.readStringUntil('>');
    //Serial.println(incoming);
    sendValues[idx] = incoming.toInt();
//    Serial.println(sendValues[idx]);
    idx++;
    
    if(idx > 5){
        idx = 0;
      }
      
 }


//    Serial.print(sendValues[0]);Serial.print("--");
//    Serial.print(sendValues[1]);Serial.print("--");
//    Serial.print(sendValues[2]);Serial.print("--");
//    Serial.print(sendValues[3]);Serial.print("--");
//    Serial.print(sendValues[4]);Serial.print("--");
//    Serial.println(sendValues[5]);
    
    
}


void connectWiFi() {
  Serial.print("Prøver å koble til: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }

  Serial.println(" ");
  Serial.println("WiFi tilkobling vellykket!");
  Serial.print("IP adressen er: ");
  Serial.println(WiFi.localIP());
  delay(200);
}

// Callback tar imot data fra MQTT som skal sendes videre til Zumo
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Innkommende melding omhandler: ");
  Serial.print(topic);
  String messageFromMQTT;


  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageFromMQTT += (char)message[i];
  }
  Serial.println();
//  Nødlading
  if(String(topic) == "esp32/emergencyCharging"){
    if(messageFromMQTT == "on1"){
      emgStatusToZumo = 1;
    }
    else if(messageFromMQTT == "off1"){
      emgStatusToZumo = 0;
    } 
  }
//  Gå til lading
  if(String(topic) == "esp32/enterCharging") {
    if(messageFromMQTT == "on2") {
      enterChargeToZumo = 1;       
    }
    else if(messageFromMQTT == "off2") {
      enterChargeToZumo = 2;
    }
  }
//  ENDRE BATTERI
  if(String(topic) == "esp32/changeBattery"){
    if(messageFromMQTT == "on3") {
      changeBatteryToZumo = 1;
    }
    else if(messageFromMQTT == "off3") {
      changeBatteryToZumo = 0;
    }
  }
  
//  SERVICE BATTERI
  if(String(topic) == "esp32/batteryService"){
    if(messageFromMQTT == "on4") {
      serviceBatteryToZumo = 1;
            
    }
    else if(messageFromMQTT == "off4") {
     serviceBatteryToZumo = 0; 
    }
  }
//  LEGG TIL SALDO PÅ ZUMO
  if(String(topic) == "esp32/solarMoneytoZumo"){
    if(messageFromMQTT != "off8") {
      String solarBuffer = messageFromMQTT;
      addCashToZumo = solarBuffer.toInt();
    }
      
    else if(messageFromMQTT == "off8"){
      addCashToZumo = 0;
    }
  }
// RØDT LYS
  if(String(topic) == "esp32/redLightToZumo"){
    if(messageFromMQTT == "on9") {               //RØDT LYS
      redLightToZumo = 1;
            
    }
    else if(messageFromMQTT == "off9") {
      redLightToZumo = 0;
    }
  }


} //Callback end

void reconnectMQTT() {
  //kjører om igjen til tilkoblingen er etablert
  while(!client.connected()) {
    Serial.print("Prøver å etablere kobling til MQTT...");
    //Prøver å koble til
    if(client.connect("zumoESP")) {
      Serial.println("Tilkobling vellykket!"); 
      //subscribe
      client.subscribe("esp32/enterCharging");
      client.subscribe("esp32/emergencyCharging");
      client.subscribe("esp32/solarMoneytoZumo");
      client.subscribe("esp32/changeBattery");
      client.subscribe("esp32/batteryService");
      client.subscribe("esp32/redLightToZumo");
      
    } else {
      Serial.print("feilet, rc=");
      Serial.print(client.state());
      Serial.println(" prøv på nytt om 5 sekunder");
      //vent 5 sekunder
      delay(5000);
    }
  }
}//reconnect MQTT end
