
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

const int greenLedPin = 21;
const int yellowLedPin = 19;
const int redLedPin = 18;
const int buttonPin = 22;
const int buttonLedPin = 25;
int buttonState;
int lightState = 1;
int lightChange = 1;
int currentLightState = 1;
int solarPanelValue  = 0;
int solarPanelEnergy = 0;
const int solarPanelPin = 32;

unsigned long previousMillis = 0;
unsigned long currentMillis =  0;
unsigned long currentMillisLight = 0;
unsigned long previousMillisSolar = 0;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// SSID og Password for router
const char* ssid = "HENRIK-NETT";
const char* password = "84451F&r";
const char* mqtt_server = "gruppe26.kicks-ass.net";
WebServer server(80);



void setup() {
  // Starter seriellkommunikasjon og velger PIN
  Serial.begin(9600);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(solarPanelPin, INPUT);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  digitalWrite(greenLedPin, HIGH);


// Kobler til WiFi
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
  greenLight();

} //Setup end

// Kjører callback
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Innkommende melding omhandler: ");
  Serial.print(topic);
  Serial.print(". Melding: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/outputStreetLight") {
    Serial.print("Endrer utgang til ");
    if(messageTemp == "on7"){
      Serial.println("PÅ");
      digitalWrite(buttonLedPin, HIGH);
      lightChange = 0;
      updateButtonState();
    }
    else if(messageTemp == "off"){
      Serial.println("AV");
      //digitalWrite(ledPin, LOW);
    }
  }
} //Callback end


// Kober til MQTT på Raspberry Pi
void reconnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32ClientTrafficLightSolar")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/outputStreetLight");
      client.subscribe("esp32/outputSolarPanel");
      client.subscribe("esp32/outputSolarEnergy");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}// reconnect MQTT end


// Rødt lys
void redLight(){
  currentMillisLight = millis();
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(yellowLedPin, HIGH);
  currentLightState = 0;
  // Current light state er 0 om det ikke er lovt å kjøre.
  delay(2000);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(redLedPin, HIGH);
}

// Grønt lys
void greenLight(){
  digitalWrite(redLedPin, HIGH);
  digitalWrite(yellowLedPin, HIGH);
  delay(2000);
  digitalWrite(redLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  currentLightState = 1;
  // Current light state er 1 om lyset er grønt.
}

void updateLightState(){
    char lightString[8];
    dtostrf(lightState, 1, 2, lightString);
    client.publish("esp32/trafficLightState", lightString);
}

void updateButtonState(){
  char buttonString[8];
  dtostrf(lightChange, 1, 2, buttonString);
  client.publish("esp32/trafficButtonPressed", buttonString);
}

// Funksjon for å sende verdien av effekt fra solcellepanelet til Node-Red
// Endrer int til char og sender som en string som kan mottas i Node-Red
void updateSolarPanelValue(){
  char powerString[8];
  dtostrf(solarPanelValue, 1, 0, powerString);
  client.publish("esp32/solarPanelValue", powerString);
  Serial.println(powerString);
}

void updateSolarPanelEnergy(){
  char energyString[8];
  dtostrf(solarPanelEnergy, 1, 0, energyString);
  client.publish("esp32/solarPanelEnergy", energyString);
  Serial.println(energyString);
}

void loop() {
  unsigned long currentMillis = millis();
  buttonState = (digitalRead(buttonPin));
  if (buttonState == 1 && lightChange == 1){
    lightChange = 0;
    updateButtonState();
    digitalWrite(buttonLedPin, HIGH);
  }
  
  //Kobler til MQTT ved frakobling og sjekker kommunikasjon
  if (!client.connected()) {
      reconnectMQTT();
    }
  client.loop();

  // Laster også opp ny solcelleverdi hvert 10. sekund
  if (currentMillis - previousMillisSolar >= 10000){
      previousMillisSolar = currentMillis;
      solarPanelValue = analogRead(solarPanelPin);
      solarPanelEnergy = solarPanelValue;
      updateSolarPanelValue();
      updateSolarPanelEnergy();
  }

      
  // Kjører hvert tiende millisekund for å endre tilstand på lys.
   if (currentMillis - previousMillis >= 10){
    previousMillis = currentMillis;
    if (currentLightState == 1 && lightChange == 0){
    redLight();
    lightState = 0;
    lightChange = 1;
    digitalWrite(buttonLedPin, LOW);
    currentMillisLight = currentMillis;
    updateLightState();
    updateButtonState();
    
    }
  if (currentLightState == 0 && currentMillis - currentMillisLight >= 5000){
    greenLight();
    lightState = 1;
    lightChange = 1;
    digitalWrite(buttonLedPin, LOW);
    updateLightState();
    updateButtonState();
    }

  }

}
