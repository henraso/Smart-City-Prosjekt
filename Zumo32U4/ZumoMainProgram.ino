#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>

Zumo32U4Motors motors; 
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD display;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors;

// Strukturen til bilen vår ble laget i runOnce()-programmet, med verdier lagret i EEPROMmen. Strukturen blir repetert her for å lagre verdiene.
struct ZumoCar {    //Alle verdiene som er spesifikke for bilen og som skal lagres mens programmet ikke kjører defineres i denne strukturen.
  unsigned long totalDistance;    //totalDistansen blir kontinuerlig oppdatert under kjøring.
  unsigned long batteryDistance;    //batteriDistansen er lik som totalDistansen, men resettes under batteri- vedlikehold/bytte.
  long failureChance;   //Tall mellom 0-1000 som fjerner 50% av batterihelsa hvis det blir truffet under en randomsjekk i hovedprogrammet.
  float failHit;    //Verdi som sier om batteriet har mista 50% av batteriHelsa.
  int charges;    //Antall ladinger bilen har gjort.
  int superCharges;   //Antall nødladinger bilen har gjort.
  int underFivePercent;   //Antall ganger bilbatteriet har gått under 5%.
  unsigned int totalBattery;    //Verdi batteriet har etter produksjonsfeil er tatt i betraktning.
  float batteryHealth;    //Batterihelse, blir påvirket av flere faktorer.
  int balance;    //Bankkonto.
};

ZumoCar ourCar;   //ourCar er et objekt av type ZumoCar, og det er her verdiene spesifikt til bilen vår blir lagret.

//Linjesensorer
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

//  Variabler nødvendig for å motta data fra mqtt via serial fra esp32
//  Liste der all data lagres
int valuesFromMQTT[5];
int changeBattery = 0;    // Data for å fortelle at det blir foretatt batteribytte
int serviceBattery = 0;   // Samme bare for service
int addCash = 0;          //Legg til cash 
int redLight = 0;         //Melding om at det er rødt lys

// RØDT LYS SONE
bool redZone=false;

//  Variabler og verdier nødvendig for å sende til mqtt
int sendValues[4];
int sendTimer1 = millis();
int sendTimer2 = 0;



//SWITCHTILSTANDER
int state = 0;
int nextState = 0; //Nødvendig for å ikke endre state når du er inne i en case


//PD-regulator
int wantedValue = 2000;
float Kp=0.5;               
float Td=6;
int lastError = 0;

//LADING
int enterCharging = 0;                  // Variabler som skal sendes til MQTT
int emergencyCharge = 0;
// True helt til emergencyCharge blir brukt en gang. Resettes ved ladning på stasjon
bool emergencyAvailable = true;        
bool charging = false;
unsigned long chargeTimer1;
unsigned long chargeTimer2=0;;



// Variabler nødvendig for egne svingefunksjoner.
bool turning=false;
unsigned long turnCounter1 = 0;
unsigned long turnCounter2 = 0;

// Addressen EEPROMmen begynner å hente verdier fra.
int eeAddress = 0;

// Timere og verdier brukt i batteriHelse-funksjonen.
unsigned long currentMillis, failureMillis;
long int totalBattery, currentBattery;

//variabler brukt i saldofunksjonen.
int currentBalance;
unsigned long balanceTimer1, balanceTimer2 = 0;

void setup() {
  failureMillis = millis();
  
  EEPROM.get(eeAddress, ourCar);   //Verdiene fra EEPROMmen blir lagt inn i ourCar-objektet.
  totalBattery = ourCar.totalBattery;   //Batterikapasitet (selvbestemt)
  currentBattery = totalBattery;  //Batterinivå akkurat nå. Sendes til MQTT for dashbord update.
  currentBalance = ourCar.balance;    //Setter bankkontoen lik den som allerede er på EEPROMmen.
  //Starter serial1 for kommunikasjon med ESP32
  Serial1.begin(115200);
  Serial.begin(9600);
  
  
  //KALLER KALIBRERINGSFUNKSJON
  calFunc();

}


void loop() {
  currentMillis = millis();
 // Serial.print("state:"); Serial.println(state);

    
// DATA SOM SENDES TIL MQTT
  int currentSpeed = speedFunc();
  int currentBatPercent = batteryStatus();
  int currentTotalDistance = distanceFunc()/100;    //Totaldistansen er i cm, og vi deler på 100 for å vise totaldistanse i meter på dashboardet, dette fordi verdien fort overstiger maksverdien på en int, og vi ikke vil sende større verdier enn int mellom Zumo og MQTT.
  int lastMinaverageSpeed = averageSpeedFunc();
  int currentBatteryHealth = ourCar.batteryHealth;
  
  
//Oppdaterer alle data som skal sendes
  sendValues[0] = currentSpeed;
  sendValues[1] = currentBatPercent;
  sendValues[2] = currentTotalDistance;
  sendValues[3] = lastMinaverageSpeed;
  sendValues[4] = currentBatteryHealth;
  sendValues[5] = currentBalance;
  
  // DATA SENDES TIL MQTT VIA ESP32
  // ">" brukes som en delimiter.
  sendTimer1 = millis();
  if(sendTimer1>sendTimer2){
    sendTimer2 = sendTimer1 + 400;
    for(int i = 0; i<6; i++){
      Serial1.print(sendValues[i]);
      //Serial.print(sendValues[i]);
      Serial1.print(">");
      //Serial.print(">");
    }
  }
// DATA SOM KOMMER FRA MQTT VIA ESP32
// While løkken kjører når det blir sendt data. Data blir lest og lagret fram til egen satt
// delimiter som her er :">". Deretter legges data i faste posisjoner i en liste. 

  int index  = 0;                   // index skal alltid være 0 ved ny lesing av data
  
  while(Serial1.available()>0){
  //Serial.println("enter while");
    String incoming = Serial1.readStringUntil('>');
    valuesFromMQTT[index] = incoming.toInt();
    Serial.println(valuesFromMQTT[1]);
    index++;
    if(index>5){
    index = 0;
    }
  }
  
  

  

// GIR EGNE VARIABLER VERDIER FRA MQTT
// SENDES ALLTID I FAST REKKEFØLGE
  emergencyCharge = valuesFromMQTT[0];
  enterCharging = valuesFromMQTT[1];
  changeBattery = valuesFromMQTT[2];
  serviceBattery = valuesFromMQTT[3];
  addCash = valuesFromMQTT[4];
  redLight = valuesFromMQTT[5];

  balanceTimer1 = millis();
  if (balanceTimer1>balanceTimer2 && addCash != 0) {
    balanceTimer2 = balanceTimer1 + 500;   //Vi har en timer på 2s mellom hver transaksjon slik at den ikke registrerer samme sending flere ganger.
    currentBalance += addCash;    //Kontoen øker med verdien som er spesifisert i MQTT.
    
  }

//  Serial.print(valuesFromMQTT[0]); Serial.print("---");
//  Serial.print(valuesFromMQTT[1]); Serial.print("---");
//  Serial.print(valuesFromMQTT[2]); Serial.print("---");
//  Serial.print(valuesFromMQTT[3]); Serial.print("---");
//  Serial.print(valuesFromMQTT[4]); Serial.print("---");
//  Serial.println(valuesFromMQTT[5]); 


  // Tilstandsendring for switch
  // nextState oppdateres i caser
  state = nextState;

// LINJEFØLGING
// HENTER VERDIER FRA LINJESENSOR
  int positionVal = lineSensors.readLine(lineSensorValues);
  //Avvik
  int errorValue = positionVal - wantedValue;
  //hastighetsforskjell
  int speedDifference = errorValue*Kp + Td*(errorValue-lastError);
//  lagrer "forrige" error
  lastError = errorValue;
  int leftSpeed = 400 + speedDifference;
  int rightSpeed = 400 - speedDifference;

  int outerSensors = lineSensorValues[0] + lineSensorValues[4];
  if(lineSensorValues[3] > 800 && lineSensorValues[2]> 800 && lineSensorValues[1] > 800 && outerSensors < 300){
    redZone = !redZone;
    buzzer.playFrequency(600,100,25);
  }

  if(redZone && redLight == 1){
    state = 7;
  }

  //Serial.print("state before switch:");Serial.println(state);
  
  switch(state){

//  Første case er selve linjefølginga.
    case 0:
      
    leftSpeed = constrain(leftSpeed,0, 400);
    rightSpeed = constrain(rightSpeed, 0, 400);
 
    motors.setSpeeds(leftSpeed, rightSpeed);
    
//  State endring hvis yttersensor til venstre oppdager 90 grader
//  sving.
    if(lineSensorValues[0]>800 && lineSensorValues[4]<200){
      nextState = 1;
      }

//  State endring når yttersensor oppdager brå høyresving og enterCharging == 1.
//  Da skal Zumo til ladestasjon
    if(lineSensorValues[4]>800 && lineSensorValues[0]<200 && enterCharging == 1){
      nextState = 2;
    }
//  Går i lading når den møter T-kryss.    
    if(enterCharging == 1 && lineSensorValues[0]>950 && lineSensorValues[4]>950 && turning == false && charging == false){
      nextState = 3;
    }
//  Kjøring ut fra ladestasjon. Vet den skal fortsette kjørebane til høyre ved første T-kryss
    if(enterCharging == 0 && lineSensorValues[0]>950 && lineSensorValues[4]>950 && charging ==false){
      nextState = 2;
    }
//  NØDLADNING - RESERVEBATTERI. KAN BARE BENYTTES 1 GANG FØR MAN GÅR INN TIL STASJON
    if(emergencyAvailable && emergencyCharge == 1 && currentBalance >= 30){
      currentBattery += 0.1 * totalBattery;
      emergencyAvailable = false;
      ourCar.superCharges ++;
      currentBalance -= 30;   //Hver nødlading koster 30kr.
    }
    

//  RESETTING AV SVINGE-BOOL. VET AT MAN IKKE ER I SVING 500MS ETTER SVINGEN HAR STARTET (90GRADER)
    if(turning){
      turnCounter1 = millis();
      if(turnCounter1 > turnCounter2){
        turning = false;
      }
    }
    break;

    case 1:
//  Svinger 90 grader til venstre    
    turning = true;
    motors.setSpeeds(0,0);
    delay(60);
    motors.setSpeeds(-400, 400);
    delay(150);
    motors.setSpeeds(0,0);
    delay(30);
    nextState = 0;                    // Tilbake til linjefølging etter sving
    turnCounter1 = millis();          // Timere som skal resette turning boolen
    turnCounter2 = turnCounter1 + 500;
    break;

    case 2:
//  Svinger 90 grader til høyre
    turning = true;
    motors.setSpeeds(0,0);
    delay(60);
    motors.setSpeeds(400, -400);
    delay(150);
    motors.setSpeeds(0,0);
    delay(30);
    nextState = 0;                  // Går tilbake til linjefølging etter sving
    turnCounter1 = millis();        // Timere som skal resette turning boolen
    turnCounter2 = turnCounter1 + 500;
    break;
    
//  LADEMODUS    
    case 3:
    motors.setSpeeds(0,0);
    charging = true;
    emergencyAvailable = true;
    chargeTimer1 = millis();
//  Lader 1 prosent hvert andre sekund. 
    if(chargeTimer1>chargeTimer2 && currentBalance >= 3){
      currentBattery += 0.01*totalBattery;
      if (currentBattery < totalBattery*0.99) {
        currentBalance -= 3;    //For hver prosent bilen lader, koster det 3kr.
      }
      chargeTimer2 = chargeTimer1 + 2000;
    }
    //Når det blir gjort service på batteriet blir batterihelsa bedre.
    if(serviceBattery==1 && currentBalance >= 250 && balanceTimer1>balanceTimer2) {
      balanceTimer2 = balanceTimer1 + 1000;
      ourCar.batteryDistance -= 100000;
      ourCar.charges -= 5;
      ourCar.superCharges -= 2;
      ourCar.underFivePercent -= 1;
      ourCar.failHit = 1;
      if (ourCar.charges < 0) {
        ourCar.charges = 0;
      }
      if (ourCar.superCharges < 0) {
        ourCar.superCharges = 0;
      }
      if (ourCar.underFivePercent < 0) {
        ourCar.underFivePercent = 0;
      }
      if (ourCar.batteryDistance < 0) {
        ourCar.batteryDistance = 0;
      }
      currentBalance -= 100;
    }
    //Når batteriet endres blir det helt nytt.
    if (changeBattery==1 && currentBalance >= 500 && balanceTimer1>balanceTimer2) {
      balanceTimer2 = balanceTimer1 + 1000;
      ourCar.batteryDistance = 0;
      ourCar.charges = 0;
      ourCar.superCharges = 0;
      ourCar.underFivePercent = 0;
      ourCar.failHit = 1;
      currentBalance -= 500;
    }
//   FORLAT LADESTASJON
    if(enterCharging == 2){
      nextState = 4;
      charging = false;
      ourCar.charges ++;
      enterCharging = 0;
    }
    break;

//  Rygger ut fra ladestasjon og snur
//  begynner linjefølging igjen
    case 4:
    turning = true;
    motors.setSpeeds(-200, -200);
    delay(700);
    motors.setSpeeds(200,-200);
    delay(615);
    motors.setSpeeds(0,0);
    delay(50);
    nextState = 0;
    break;

//  Kjør rett fram ved stort hull i teip
    case 5:
    motors.setSpeeds(350,350);
    delay(350);
    nextState = 0;
    break;

    case 6:
    motors.setSpeeds(0,0);
    if(redLight == 0){
      nextState = 0;
    }
    break;
  }
  ourCar.balance = currentBalance;    //Setter relevant saldo tilbake inn i bilobjektet vårt.
  batteryHealthFunc();    //Finner ny batterihelse
  underXpercentBattery();
  EEPROM.put(eeAddress, ourCar);    //Lagrer nye verdier i EEPROM.
}


//FUNKSJONER
//KALIBRERING
int calFunc(){
  int _ticks = 0;
  
  //TEKSTINSTRUKSJONER FOR KALIBRERING
  /*display.clear();
  display.gotoXY(0, 0);
  display.print("Press A");
  display.gotoXY(0, 1);
  display.print("to CAL");*/
  
//  VENTER PÅ KNAPPETRYKK FOR KALIBRERING
  buttonA.waitForButton();
  lineSensors.initFiveSensors();
  
//  KALIBRERING:
  
  while(_ticks < 3500){ // Kalibrerer helt til encoderne har telt 3500 ganger.
    motors.setSpeeds(150, -150);
    lineSensors.calibrate();
    _ticks = encoders.getCountsLeft();
  }
  _ticks = 0;
  while(_ticks < 3500){
    motors.setSpeeds(-150, 150);
    lineSensors.calibrate();
    _ticks = encoders.getCountsRight();
    
  }
//  Slår av motorene før den skal starte.
  motors.setSpeeds(0,0);
//  Delay før start  
  delay(4000);
}


//SPEEDOMETER
//speedFunc variabler som ikke kan stå inne i funksjonen
float velocity;
int count = 0;
float ticks = 0;
long int t2 = 0;
float refresh = 200.0; //refreshrate for LCD-screen

int speedFunc(){
  //Timer for oppdateringsfrekvens på utregning av fart
  long int t1 = millis();
  
  
  if(t1 > t2 && turning == false){
    count += 1;

    
    t2 = t1 + refresh;
    ticks = encoders.getCountsAndResetLeft();
    float distance = (ticks) / (67.96);           // Ticks til cm
    velocity = (distance)/ (refresh/1000.0);      // Deler refresh på 1000 for å få ms til s.
        
    //Writing to LCD
    /*
    display.clear();
    display.gotoXY(0, 0);
    display.print(int(velocity)); 
    display.print("CM/s");*/      
  }
  return int(velocity);               // Returnerer en int som kan skrives til dashbord på nettsida.
}

//TOTAL DISTANSE
//VARIABLER SOM MÅ STÅ UTENFOR FUNKSJON
long int t4 = 0;
int distanceTraveled;
int distanceFunc(){
  long int t3 = millis();
  if(t3>t4){
    t4 = t3+refresh;  //Bruker samme refresh-rate som speedometer
    
    //Bruker speedometer funksjonen for å regne ut distanse.
    //Da vil distanse stoppe når farten er 0. Bruker absoluttverdi, siden 
    //rygging er også distanse kjørt
    distanceTraveled = abs(speedFunc())*(refresh/1000.0);
    ourCar.totalDistance += distanceTraveled;
    ourCar.batteryDistance += distanceTraveled;
    display.clear();
    display.print(int(distanceTraveled));display.print("cm");
  }

  return ourCar.totalDistance;
}
//Funksjon for gjennomsnitthastighet siste minutt
unsigned long t6 = 0;
int previousDistance = 0;
int averageSpeedFunc(){
  int averageSpeed;
  unsigned long t5 = millis();
  if(t5 > t6){
    t6 = t5 + 60000;
    averageSpeed = (distanceFunc() - previousDistance)/60;
    previousDistance = distanceFunc();
  }
  return averageSpeed;
}

//Batteribrukfunksjon
//Hviletilstand = 10mAh f(0) = 10
//makshastighet = 90mAh f(40) = 90
//Batteri = 15mAh
int totalDrain;
int batteryDrain(){
  int constantDrain = 10;
  int variableDrain = (speedFunc())*2;
  totalDrain = constantDrain + variableDrain;
  if(charging){
    totalDrain = 0;
  }
  return totalDrain;
}

//Funksjon for batteriprosent
unsigned long batteryTimer2 = 0;
int batteryStatus(){
  unsigned long batteryTimer = millis();
  

//vanlig kjøring
  if(batteryTimer > batteryTimer2){
    currentBattery = currentBattery - batteryDrain();
    batteryTimer2 = batteryTimer + 1000;
  }

  
  //currentBattery kan ikke være høyere enn totalBattery
  if(currentBattery > totalBattery){
    currentBattery = totalBattery;
  }
  if(currentBattery <= 0){
    currentBattery = 0;    // Batteri kan ikke være mindre enn 0
    motors.setSpeeds(0,0); // Kan ikke kjøre uten batteri
  }
  
  float batteryPercentage = (float(currentBattery) / float(totalBattery))*100.0;
    //display.clear();
    //display.print(int(batteryPercentage)); display.print("%");
  
    
  return batteryPercentage;
}


//funksjon for stopping ved under 5 og 10 prosent strøm
long int currentTime = 15000;
long int spentTime = millis();
bool firstTimeBuzzer = false;             
bool firstTimeUnder5 = false;                                                   
int buzzCounter = 0;
unsigned long buzzTimer1 = millis();
unsigned long buzzTimer2 = 0;

void underXpercentBattery(){
  
  if (((currentBattery) <= (totalBattery*0.10)) && ((currentBattery) >= (totalBattery*0.05))){
//  Skal pipe to ganger hvert 15 sekund hvis batteriet ligger på mellom 5 og 10 prosent.
    spentTime = millis();
    buzzTimer1 = millis();
    if(buzzTimer1>buzzTimer2 && firstTimeBuzzer==false){
      buzzer.playFrequency(250,250,10);
      buzzTimer2 = buzzTimer1 + 500;
      buzzCounter ++;
      if(buzzCounter == 2){
        firstTimeBuzzer = true;
      }
    }
    if(spentTime>currentTime){
      firstTimeBuzzer = false;
      currentTime = spentTime + 15000;
      buzzCounter = 0;
    } 
  }

  else if((currentBattery) < (totalBattery*0.05)){
//  Hvis batteriet er under 5 prosent strøm skal det gjøre det samme som over, men stoppe motorene i tillegg. 
    spentTime = millis();
    buzzTimer1 = millis();
    if(buzzTimer1>buzzTimer2 && firstTimeUnder5==false){
        buzzer.playFrequency(250,250,10);
        buzzTimer2 = buzzTimer1 + 500;
        buzzCounter ++;
        motors.setSpeeds(0,0);
        delay(10);
        if(buzzCounter == 2){
          firstTimeUnder5 = true;
          ourCar.underFivePercent ++;
        }
      }
      if(spentTime>currentTime){
        firstTimeUnder5 = false;
        currentTime = spentTime + 15000;
        buzzCounter = 0;
      }
  }
}

float batteryEquation;
float batteryHealthPercent = ourCar.batteryHealth;    //Henter opprinnelig verdi fra EEPROM
float topHealth, batteryDecay;    //batteryDecay er korrosjonen som skjer i batteriet og blir på virket av antall ladinger, nødladinger ganger bilen har gått under fem prosent og distanse kjørt på batteriet.
int failCheck;
float failHit = 1;

//BATTERIHELSEFUNKSJON
void batteryHealthFunc() {
  batteryDecay = (3*(ourCar.superCharges) + 0.4*(ourCar.charges) + (10*ourCar.underFivePercent))*exp(pow(10,-5)*ourCar.batteryDistance);
  if (currentMillis - failureMillis >= 2000) {
    failCheck = random(1,1000);   
    if (failCheck == ourCar.failureChance) {    //Annenhvert sekund er det en 0,1% sjanse for at batterihelsa faller med 50%.
      failHit = 0.5;
    }
    if (batteryEquation < 0) {
      batteryEquation = 0;
    }
    failureMillis = currentMillis;
  }
    batteryEquation = ourCar.totalBattery*failHit - batteryDecay;    //Regner ut batterikorrosjon.

  batteryHealthPercent = batteryEquation/float(totalBattery)*100;
  totalBattery = ourCar.totalBattery*batteryHealthPercent;
  ourCar.batteryHealth = batteryHealthPercent;    //Lagrer ny batterihelse-verdi i ourCar-objektet.
}
