#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <stdlib.h> 

int connectionStatus = 0;

int milliseconds;
int seconds;
int minutes;

const int receiver = 33;
const int stopButton = 32;

unsigned long currentMillis;
unsigned long previousMillis;

int counter;
int x;

uint8_t sensorState;
uint8_t buttonState;

WiFiClient espClient;
PubSubClient client(espClient);

// SSID og Password for router
const char* ssid = "HENRIK-NETT";
const char* password = "84451F&r";//"84451F&r";

const char* mqtt_server = "gruppe26.kicks-ass.net";
//const char* mqtt_server = "broker.hivemq.com";//"gruppe26.kicks-ass.net"; 
WebServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(receiver, INPUT);
  pinMode(stopButton, INPUT);
  
  client.setServer(mqtt_server, 1883);
  
  connectWiFi();
}

void loop() {
  if(!client.connected()) {
    reconnectMQTT();
    connectionStatus = 0;
  }
  client.loop();
  
  sensorState = digitalRead(receiver);
  buttonState = digitalRead(stopButton);
  
  currentMillis = millis();
  if(currentMillis - previousMillis >= 10000) {
    previousMillis = currentMillis;
    Serial.println("VENTER PÅ STARTSIGNAL");
  }

  if(connectionStatus == 1) {
    client.publish("hivemq/connectionStatus", "Tilkoblet");
  }
  
  if(sensorState == LOW) {
    x = 1;
  }

  while(x==1) {
    sensorState = digitalRead(receiver);
    buttonState = digitalRead(stopButton);
    timer();
    
    if(sensorState == LOW && counter >= 5) {
      Serial.println((String)"SLUTTID: " + minutes + ":" + seconds + ":" + milliseconds);
      //resetTimer();
      stopTimer();
      counter = 0;
      x = 0;
      delay(2000);
    }
  } 
}

void sendData() {
  String endTimer = ((String)minutes + ":" + seconds + ":" + milliseconds);
  
  char endtimeString[8];
  endTimer.toCharArray(endtimeString, 10);
  client.publish("esp32/timerEnd", endtimeString);

  //NOT CONNECTED TO EDUROAM
  //client.publish("hivemq/timerEnd", endtimeString);
}

void timer() {
  currentMillis = millis();
  if(currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    milliseconds = milliseconds + 10;
    displayTimer();
  }

  if(milliseconds >= 1000) {
    milliseconds = 0;
    seconds++;
    counter++;
    sendData();
  }
  
  if(seconds >= 60) {
    seconds = 0;
    minutes++;
    Serial.println("MINUTTLAGTTIL");
  }
}

void resetTimer() {
  sendData();
  minutes = 0;
  seconds = 0;
  Serial.println("RESET");  
  sendData();
}

void stopTimer() {
  sendData();
  milliseconds = 0;
  minutes = 0;
  seconds = 0;
  counter = 0;
  Serial.println("STOPPET");
}

void displayTimer() {
  Serial.println((String)"TIDTAKER: " + minutes + ":" + seconds + ":" + milliseconds);
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

void reconnectMQTT() {
  //kjører om igjen til tilkoblingen er etablert
  while(!client.connected()) {
    Serial.print("Prøver å etablere kobling til MQTT...");
    //Prøver å koble til
    if(client.connect("timerClient")) {
      Serial.println("Tilkobling vellykket!"); 
      connectionStatus = 1;
      //subscribe
    } else {
      Serial.print("feilet, rc=");
      Serial.print(client.state());
      Serial.println(" prøv på nytt om 5 sekunder");
      //vent 5 sekunder
      delay(5000);
    }
  }
}//reconnect MQTT end
