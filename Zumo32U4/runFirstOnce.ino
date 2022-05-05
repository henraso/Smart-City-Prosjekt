/* Dette programmet er ment for å kjøres en gang før vi begynner å bruke hovedprogrammet
 * Programmet fjerner all tidligere data fra EEPROM-en og setter batterihelse mtp. produksjonsfeil. 
 * På denne måten slipper vi å kalibrere linjesensorene hver gang vi skal starte bilen.
 */

#include <EEPROM.h>

int eeAddress = 0;    //Addressen EEPROMmen starter fra når den lagrer data.
float maxBattery = 54000;   //Maksverdi batteriet kan ha, denne verdien kan defineres som vi vil.

struct ZumoCar {    //Alle verdiene som er spesifikke for bilen og som enten endrer seg under kjøring eller som blir skapt her og skal bli henta i hovedprogrammet får en variabel i denne strukturen.
  unsigned long totalDistance;    //totalDistansen blir kontinuerlig oppdatert under kjøring.
  unsigned long batteryDistance;    //batteriDistansen er lik som totalDistansen, men resettes under batteri- vedlikehold/bytte.
  long failureChance;   //Tall mellom 0-1000 som fjerner 50% av batterihelsa hvis det blir truffet under en randomsjekk i hovedprogrammet.
  float failHit;    //Verdi som sier om en produksjonsfeil har inntruffet.
  int charges;    //Antall ladinger bilen har gjort.
  int superCharges;   //Antall nødladinger bilen har gjort.
  int underFivePercent;   //Antall ganger bilbatteriet har gått under 5%.
  unsigned int totalBattery;    //Verdi batteriet har etter produksjonsfeil er tatt i betraktning.
  float batteryHealth;    //Batterihelse, blir påvirket av flere faktorer.
  int balance;    //Bankkonto.
};


// clearEEPROM() Setter hver verdi i flash-minnet til 0.
void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}

ZumoCar ourCar;   //Kaller et objekt av struktur ZumoCar, kalt ourCar.

void setup() {
  clearEEPROM();
  randomSeed(analogRead(0));    //Randomiserer 
  int prodError = random(90,101);   //Tilfeldig tall mellom 90-100, gjør at batterihelsa og totalbatteriet har en produksjonsfeilverdi på 0-10%.
  ourCar.failureChance = random(1,1001);    //Tilfeldig tall mellom 1-1000, hvis den blir truffet av en annen rng-sjekk i hovedprogrammet blir batterihelsa redusert med 50%.
  ourCar.totalBattery = maxBattery*prodError/100;   //totalBatteriet.
  ourCar.batteryHealth = ourCar.totalBattery/ourCar.totalBattery*100;   //Batterihelsa er en prosent av totalBatteri.
  ourCar.failHit = 1;
  ourCar.balance = 700;   //Startkapital.
  ourCar.batteryDistance = 900000;
  EEPROM.put(eeAddress, ourCar);    //Setter strukturen i EEPROMmen.
  Serial.begin(9600);
  Serial.println("Done!!");
  Serial.println(ourCar.totalDistance);
}


void loop() {
  
}
