//inkluderer nødvendige biblioteker
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

//påloggingsinformasjonene for å koble til hotspot fra egen PC
const char* ssid = "HENRIK-NETT";
const char* password = "84451F&r";
const char* mqtt_server = "gruppe26.kicks-ass.net";

//konfigurerer MQTT
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;

//variabler for å lagre data hentet fra sensoren
Adafruit_BME280 bme;
float temp = 0;
float humidity = 0;
float pressure = 0;

void setup() {
  Serial.begin(115200); //setter baud rate til 115200

  //kjører til BME-sensoren blir detektert over I2C
  if(!bme.begin(0x76)) { //definerer hvilken adresse I2C er tilkoblet
    Serial.println("Finner ikke BME280-sensoren, sjekk at alt er riktig koblet!");
    while(1); //loop
  }
  //kobler til WiFi
  connectWiFi();

  //setter verdier for tilkobling til MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
} //void setup end

void loop() {
  //Sjekker tilkoblingen til MQTT
  if(!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  //sender data hvert 5. sekund
  long currentMillis = millis();
  if(currentMillis - previousMillis > 5000) {
    previousMillis = currentMillis;

    //henter temperatur i celsius
    temp = bme.readTemperature();

    //temperaturdata som skal sendes til dashboard
    char tempString[10]; //lager en char som holder dataen som skal sendes
    dtostrf(temp, 1, 2, tempString); //gjør om fra float til string, og beholder 2 desimaler
    Serial.print("Temperatur: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString); //sender over MQTT til Node-RED på topic "esp32/temperature"

    //henter luftfuktighet fra sensoren og lagres
    humidity = bme.readHumidity();
    
    char humidityString[10]; //char som inneholder data
    dtostrf(humidity, 1, 2, humidityString); //gjør om til string
    Serial.print("Luftfuktighet: "); //Serialprinter
    Serial.println(humidityString);
    client.publish("esp32/humidity", humidityString); //Sender på gitt topic

    //lagrer verdi for lufttrykk, og gjør om så verdien er i hPa
    pressure = bme.readPressure()/100;
    
    char pressureString[10]; //char som inneholder data
    dtostrf(pressure, 1, 2, pressureString); //gjør om til string (char)
    Serial.print("Lufttrykk: "); //Serialprinter
    Serial.println(pressureString);
    client.publish("esp32/pressure", pressureString); //sender på gitt topic til Node-RED
  }
} //void loop end

//Egen funksjon som kobler til hotspot
void connectWiFi() {
  Serial.print("Prøver å koble til: "); //printer til Serial for debugging
  Serial.println(ssid); //printer navnet på nettet som blir forsøkt tilkoblet
  WiFi.begin(ssid, password); //starter tilkoblingen med gitt ssid og passord

  while (WiFi.status() != WL_CONNECTED){ //looper helt til tilkoblingen er gjennomført
    delay(1000);
    Serial.print(".");
  }

  //printes hvis tilkobling er vellykket
  Serial.println(" ");
  Serial.println("WiFi tilkobling vellykket!");
  Serial.print("IP adressen er: ");
  Serial.println(WiFi.localIP());  //viser IP-adressen som er tilkoblet
  delay(200);
} //connectWiFi end

//egendefinert funksjon som tilkobler MQTT
void reconnectMQTT() {
  //kjører om igjen til tilkoblingen er etablert
  while(!client.connected()) {
    Serial.print("Prøver å etablere kobling til MQTT...");
    //Prøver å koble til med et definert klient-navn
    if(client.connect("bme280Client")) {
      Serial.println("Tilkobling vellykket!"); 
    } else {
      Serial.print("feilet, rc=");
      Serial.print(client.state()); //printer verdi som forteller hvorfor tilkoblingen mislykkes, for debugging
      Serial.println(" prøv på nytt om 5 sekunder");
      //vent 5 sekunder
      delay(5000);
    }
  }
}//reconnect MQTT end
