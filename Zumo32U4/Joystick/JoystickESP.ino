#include <esp_now.h>
#include <WiFi.h>

const int buttonPin = 33;
const int ledPin = 32;

const int dimensionY = 35;
const int dimensionX = 34;

int buttonState;
byte lastButtonState = LOW;
byte ledState = LOW;
int initManual;

int midX, midY;
int x, y = 0;

// Kaller alle variabler som skal brukes i joystick-funksjonen. 
int xVal, yVal, mainMotor, rightMotor, leftMotor;
// mainMotor og diffMotor bestemmer hvilket hjul som skal være hovedhjulet og hvilket som skal være sidehjulet.
float diffDeg, diffValue, diffMotor;

// Mottakers Mac Adresse  44:17:93:5E:49:C0

uint8_t broadcastAddress[] = {0x44, 0x17, 0x93, 0x5E, 0x49, 0xC0};

// struct som inneholder data som skal sendes
// Matches the receiver
typedef struct struct_message {
  int rightControl;
  int leftControl;
  int initManual;
} struct_message;

// struct_message myData som holder dataen som skal sendes
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback når data blir sendt
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus på siste sendte datapakke:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Datapakke sendt" : "Datapakke ble ikke sendt");
}//end onDataSent

void setup() {
  Serial.begin(115200);
  pinMode(dimensionY, INPUT);
  pinMode(dimensionX, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Joystickene har litt variasjon i hvilken verdi de returnerer når de er i nøytral posisjon, så disse variablene finner midtverdien.
  midX = analogRead(dimensionX);
  midY = analogRead(dimensionY);

  // Setter ESP32-en som en WiFi-stasjon
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Feil ved initialisering av ESP-NOW");
    return;
  }

  // Når ESP-NOW er satt opp, vil vi registrere for Send CB for å få
  // status på datapakken som ble sendt
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

//Funksjon som leser av en knapp i en pull-down krets
void initButton() {
  buttonState = digitalRead(buttonPin);

  //sammenligner forrige og nåværende status for knappen
  if(buttonState != lastButtonState) { //hvis de verdiene ikke er like, kjør if-setning
    lastButtonState = buttonState; //setter lastButtonState til avlest buttonState
    if((buttonState == LOW)) { 
      ledState = (ledState == HIGH) ? LOW: HIGH;
      digitalWrite(ledPin, ledState); //setter ledlys høy for å signalisere at data sendes
      initManual = (initManual == 1) ? 0: 1; //verdi som brukes videre
    }
  }
}

void loop() {

  
  initButton(); //kjører egendefinert funksjon

  //når knappen er trykket, skal koden sendes
  while(initManual == 1) {
    initButton();
    // Leser de analoge verdiene som joysticken gir ut.
    x = analogRead(dimensionX);
    y = analogRead(dimensionY);
    joyStick();

    myData.leftControl = leftMotor;
    myData.rightControl = rightMotor;
    myData.initManual = 1;

    Serial.println(myData.leftControl);
    Serial.println(myData.rightControl);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
    delay(10);
  }
  Serial.println("Venter på signal!"); //når knappen trykkes på nytt går den ut av while-loopen
}

int previousMotor, currentMotor;

void joyStick() {
  // Arduinoen har en standardleseresolusjon på 10 bits, som betyr at de leste verdiene kommer til å være mellom 0-1024. Vi mappet verdiene til å bli mellom 0-400, som er rangen til bilfarten.
  xVal = map(x, midX, 1023, 0, 400);
  yVal = map(y, midY, 0, 0, 400);
  // For å finne et forhold mellom hjulene når joysticken ikke peker rett frem, bak eller til siden lagde vi en variabel "diffDeg" som finner vinkelen mellom x og y.
  diffDeg = atan2(yVal, xVal)*180/3.1415;

  // atan2() kommandoen leser kun fra 0-180 grader, før den går over til negative verdier, så denne funksjonen gjør at vi kan lese verdier på 360grader.
  if (diffDeg < 0) {
    diffDeg += 360;
  }

  previousMotor = mainMotor;


  
  // I denne if... else if-funksjonen bestemmes det hvor sterk sidehjulet skal være i forhold til hovedhjulet, og hva som skal avgjøre størrelsen på hovedhjulet.
  if (diffDeg < 90) {
    // Når gradene er mellom 0-90 skal differensialverdien være en faktor 0-1 av hovedhjulet.
    diffValue = map(diffDeg, 0, 90, 0, 100);
    // Denne lille if-else funskjonen bestemmer om det er xVal eller yVal som skal være farten til hovedmotoren. Siden de allerede er mappa mellom 0-400, som er samme fart som hjula kan ha, kan de settes lik hverandre.
    if (diffDeg < 45) {
      mainMotor = xVal;
    } else {
      mainMotor = yVal;
    }
  } else if (diffDeg < 190) {
    diffValue = map(diffDeg, 90, 180, 100, 0);
    if (diffDeg < 135) {
      mainMotor = abs(yVal);
    } else {
      mainMotor = abs(xVal);
    }
  } else if (diffDeg < 270) {
    diffValue = map(diffDeg, 180, 270, 0, 100);
    if (diffDeg < 225) {
      mainMotor = xVal;
    } else {
      mainMotor = yVal;
    }
  } else {
    diffValue = map(diffDeg, 270, 360, 100, 0);
    if (diffDeg < 315) {
      mainMotor = yVal;
    } else {
      mainMotor = -xVal;
    }
  }

  mainMotor = constrain(mainMotor, -400, 400);
  // Lager en liten buffer for små verdier sånn at den ikke beveger seg når den skal stå stille
  if (abs(mainMotor) < 15) {
    mainMotor = 0; 
  }
  // Regnestykket bestemmer hvor sterk sidehjulet skal være. Sidehjulfarten er hovedhjulet ganget med differensialfaktoren.
  diffMotor = mainMotor*diffValue/100;

  // If-else løkka bestemmer hvilket hjul som er hovedhjul og hvilket hjul som er sidehjul.
  if (diffDeg > 90 && diffDeg < 270) {
    rightMotor = diffMotor;
    leftMotor = mainMotor;
  } else {
    rightMotor = mainMotor;
    leftMotor = diffMotor;
  }

  currentMotor = mainMotor

  // Forrigling for å hindre slit på motorene. Når motoren går fra å kjøre framover til bakover eller motsatt er det satt på et 200ms delay hvor motoren står stille før den endrer retning.
  if (currentMotor < 0 && previousMotor > 0 || currentMotor > 0 && previousMotor < 0) {
    myData.leftControl = 0;
    myData.rightControl = 0;
    myData.initManual = 1;

    Serial.println(myData.leftControl);
    Serial.println(myData.rightControl);
    delay(200);
  }

//  Serial.print(rightMotor);
//  Serial.print(",");
//  Serial.println(leftMotor);
}
