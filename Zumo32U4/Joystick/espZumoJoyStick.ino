#include <esp_now.h>
#include <WiFi.h>


int rightControl;
int leftControl;
int initManual;
// Struct som skal motta data
// Må være lik senderens struct
typedef struct struct_message {
  int rightControl;
  int leftControl;
  int initManual;
} struct_message;

// lager en struct kalt myData
struct_message myData;

// callback-funksjon som kjøres når det mottas data
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
//  Data sendes direkte til Zumo. ¨'>' er delimiter.
//  Kommer alltid i riktig rekkefølge.
  Serial.print(myData.leftControl);Serial.print("------------------");
  Serial1.print(myData.leftControl);
  Serial1.print(">");
  Serial.println(myData.rightControl);
  Serial1.print(myData.rightControl);
  Serial1.print(">");

}
 
void setup() {
  // Start Serial monitor
  Serial.begin(115200);
  // Snakker med Zumo via Serial1
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  
  // Sett enhet som WiFi-stasjon
  WiFi.mode(WIFI_STA);
  
  // Start ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ved initialisering av ESP-NOW");
    return;
  }
  
  // Når ESP-NOW kjører setter vi recv CB for å motta datapakker
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

//  Ikke nødvendig med noe i loopen, siden callback
//  -funksjonen sender alt til Zumo via Serial1.  

} 
