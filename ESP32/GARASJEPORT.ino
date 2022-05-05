//inkluderer nødvendige biblioteker
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

//definerer tilkobling for servo
const int servoPin = 32;
Servo servo;

//definerer variabel for statusen til servoen
int servoState;
int servoChange

//definerer variabler for tidtaking med millis()
unsigned long previousMillis;
unsigned long currentMillis;

//oppsett for MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// SSID og Passord for hotspot
const char* ssid = "HENRIK-NETT";
const char* password = "84451F&r";
const char* mqtt_server = "gruppe26.kicks-ass.net";
WebServer server(80);

void setup() {
  //Starter serial monitor
  Serial.begin(115200);
  servo.attach(servoPin); //tilkobler servoen

  //koblier til MQTT-server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  servo.write(0); //setter servoen til posisjon 0

  //Kobler til WiFi
  connectWiFi();
} //void setup end

void loop() {
  //Sjekker tilkoblingen til MQTT, og kjører loop helt til tilkobling
  if(!client.connected()) {
    reconnectMQTT();
  }
  client.loop(); //looper for å sjekke etter meldinger tilsendt

  char garageString[8]; //lager en variabel som skal lagre data som skal sendes
  dtostrf(servoState, 1, 2, garageString); //gjør om til string(char)
  client.publish("esp32/garageDoor", garageString); //sender data på gitt topic
}

//egendefinert funksjon for å koble til internett
void connectWiFi() {
  Serial.print("Prøver å koble til: ");
  Serial.println(ssid); //printer ssid
  WiFi.begin(ssid, password); //tilkobler gitt ssid og med gitt passort

  //looper til tilkobling er gjennomført
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }

  Serial.println(" ");
  Serial.println("WiFi tilkobling vellykket!");
  Serial.print("IP adressen er: ");
  Serial.println(WiFi.localIP()); //printer IP-adressen 
  delay(200);
}

//egendefinert funksjon for å motta meldinger over MQTT fra Node-RED
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Innkommende melding omhandler: ");
  Serial.print(topic); //viser hvilken topic det blir sendt på 
  Serial.print(". Melding: ");
  String messageGarage; //definerer en variabel som lagrer meldingen

  //meldingen kommer som chars, så må legges til stringen, som er en array av chars
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageGarage += (char)message[i];
  }
  Serial.println();

  //sjekker om topic er lik ønsket topic å motta meldinger fra
  if(String(topic) == "esp32/garageOutput") {
    Serial.print("Endrer tilstand til ");
    if(messageGarage == "open") { //hvis "open" blir mottatt, endre servoens tilstand
      Serial.println("ÅPEN");
      servo.write(90);
      servoChange = 1;
      servoState = 1;
    }
    else if(messageGarage == "closed") { //hvis "closed" blir mottatt, endre servoens tilstand
      Serial.println("STENGT");
      servo.write(00);
      servoState = 0;
    }
  }
} //Callback end

//egendefinert funksjon som tilkobler til MQTT
void reconnectMQTT() {
  //kjører om igjen til tilkoblingen er etablert
  while(!client.connected()) {
    Serial.print("Prøver å etablere kobling til MQTT...");
    //Prøver å koble til
    if(client.connect("garageClient")) {
      Serial.println("Tilkobling vellykket!"); 
      //abonnerer på topic som skal sende data fra Node-RED
      client.subscribe("esp32/garageOutput");
      
    } else {
      Serial.print("feilet, rc=");
      Serial.print(client.state()); //Serialprinter status, og hvorfor tilkoblingen feilet
      Serial.println(" prøv på nytt om 5 sekunder");
      //vent 5 sekunder
      delay(5000);
    }
  }
}//reconnect MQTT end
