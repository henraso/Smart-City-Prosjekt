// NB! DETTE ER IKKE SISTE VERSJON AV KODEN. DENNE
// KODEN ER BARE LAGT TIL HER FOR Å VISE AT VI HAR 
// LAGET EN LØSNING PÅ Å DETEKTERE HAKK I LINJA
// PÅ LINJEFØGEREN.

// LEGG OGSÅ MERKE TIL AT KODEN IKKE ER FERDIG KOMMENTERT
// OG DET ER OGSÅ FORDI DETTE IKKE ER EN FERDIG KODE! SE 
// ZumoMainProgram FOR DEN FERDIGE KODEN! 

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors; 
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD display;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors;

//Linjesensorer
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

//  variabler nødvendig for å motta data fra mqtt via serial fra esp32
String inStr;
String inStr2;
int valuesFromMQTT[5];
int changeBattery = 0;
int serviceBattery = 0;
int addCash = 0;
int redLight = 0;


//  variabler og ting nødvendig for å sende til mqtt
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
int enterCharging = 0;
int emergencyCharge = 0;
bool charging = false;
unsigned long chargeTimer1;
unsigned long chargeTimer2=0;;

long int totalBattery = 54000;          //Batterikapasitet
long int currentBattery = totalBattery;  //Batterinivå

//hull i linjefølginga
int whiteCounter = 0;
unsigned long countTimer1=0;
unsigned long countTimer2 = 0;
bool checkCount = false;
bool failedCount = false;
int currentPos;
//Sving
bool turning;
unsigned long turnCounter1 = 0;
unsigned long turnCounter2 = 0;



void setup() {
  //starter serial for sending av informasjon til esp
  Serial1.begin(115200);
  Serial.begin(9600);
  
  //KALLER KALIBRERINGSFUNKSJON
  calFunc();

}

void loop() {
  
 // Serial.print("state:"); Serial.println(state);

    
// DATA SOM SENDES TIL MQTT
  int currentSpeed = speedFunc();
  int currentBatPercent = batteryStatus();
  int currentTotalDistance = distanceFunc();
  int lastMinAvrageSpeed = avrageSpeedFunc();
  int currentBatteryHealth = 100;
  int currentBalance = 500;
  
//Oppdaterer alle data som skal sendes
  sendValues[0] = currentSpeed;
  sendValues[1] = currentBatPercent;
  sendValues[2] = currentTotalDistance;
  sendValues[3] = lastMinAvrageSpeed;
  sendValues[4] = currentBatteryHealth;
  sendValues[5] = currentBalance;
  
  //DATA SENDES TIL MQTT VIA ESP32
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
  int index  = 0;
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
  if(index>5){
    index = 0;
  
 }
 // Serial.println("leave while");
  //GIR EGNE VARIABLER VERDIER FRA MQTT
  emergencyCharge = valuesFromMQTT[0];
  enterCharging = valuesFromMQTT[1];
  changeBattery = valuesFromMQTT[2];
  serviceBattery = valuesFromMQTT[3];
  addCash = valuesFromMQTT[4];
  redLight = valuesFromMQTT[5];
  

//  Serial.print(valuesFromMQTT[0]); Serial.print("---");
//  Serial.print(valuesFromMQTT[1]); Serial.print("---");
//  Serial.print(valuesFromMQTT[2]); Serial.print("---");
//  Serial.print(valuesFromMQTT[3]); Serial.print("---");
//  Serial.print(valuesFromMQTT[4]); Serial.print("---");
//  Serial.println(valuesFromMQTT[5]); 


  
  //Tilstandsendring for switch
  state = nextState;
  
  //HENTER VERDIER FRA LINJESENSOR
  int positionVal = lineSensors.readLine(lineSensorValues);
  //Avvik
  int errorValue = positionVal - wantedValue;
  //hastighetsforskjell
  int speedDifference = errorValue*Kp + Td*(errorValue-lastError);

  //lagrer "forrige" error
  lastError = errorValue;

  int leftSpeed = 400 + speedDifference;
  int rightSpeed = 400 - speedDifference;
  
//OPPDAGING AV HULL I LINJA
  currentPos= + lineSensorValues[2] + lineSensorValues[3];
 
  if(currentPos < 25 && checkCount == false && failedCount==false &&turning == false){
    countTimer1 = millis();
    countTimer2 = millis() + 200;
    checkCount = true;
  }
  if(checkCount){
    countTimer1 = millis();
    if (countTimer1 < countTimer2){
      if (lineSensorValues[2] > 800){
        whiteCounter=1;
        checkCount=false;
      }
    }
    else{
        checkCount = false;
        failedCount = true;
    }
  }

  if(failedCount && lineSensorValues[2] > 800){
    failedCount = false;
  }


  if (whiteCounter == 1){
    buzzer.playFrequency(600, 200, 20);
    state = 5;
  }
  //Serial.print("state before switch:");Serial.println(state);
  
  switch(state){
    case 0:
      
    leftSpeed = constrain(leftSpeed,0, 400);
    rightSpeed = constrain(rightSpeed, 0, 400);
 
    motors.setSpeeds(leftSpeed, rightSpeed);
    
  
     //VEKSLING HVIS YTTERSENSOR TIL VENSTRE OPPDAGER BRÅ SVING
    if(lineSensorValues[0]>800 && lineSensorValues[4]<200){
      nextState = 1;
      }

    //VEKSLING HVIS YTTERENSOR TIL HØYRE OPPDAGER BRÅ SVING
    if(lineSensorValues[4]>800 && lineSensorValues[0]<200 && enterCharging == 1){
      buzzer.playFrequency(300, 200, 5);
      nextState = 2;
    }
    if(enterCharging == 1 && lineSensorValues[0]>950 && lineSensorValues[4]>950 && turning == false && charging == false){
      nextState = 3;
    }
    //Kjøring ut fra ladestasjon
    if(enterCharging == 0 && lineSensorValues[0]>950 && lineSensorValues[4]>950 && charging ==false){
      nextState = 2;
    }

    if(turning){
      turnCounter1 = millis();
      if(turnCounter1 > turnCounter2){
        turning = false;
      }
    }
    break;

    case 1:
    turning = true;
    //Brå venstre sving
    motors.setSpeeds(0,0);
    delay(60);
    motors.setSpeeds(-400, 400);
    delay(150);
    motors.setSpeeds(0,0);
    delay(30);
    nextState = 0;
    turnCounter1 = millis();
    turnCounter2 = turnCounter1 + 500;
    
    break;

    case 2:
    turning = true;
    motors.setSpeeds(0,0);
    delay(60);
    motors.setSpeeds(400, -400);
    delay(150);
    motors.setSpeeds(0,0);
    delay(30);
    nextState = 0;
    turnCounter1 = millis();
    turnCounter2 = turnCounter1 + 500;
    
    break;
      
    case 3:
    buzzer.playFrequency(440, 200, 10);
    motors.setSpeeds(0,0);
    delay(1000);
    nextState = 6;
    break;

    case 4://Forlat charging:
    
    turning = true;
    motors.setSpeeds(-200, -200);
    delay(700);
    motors.setSpeeds(200,-200);
    delay(615);
    motors.setSpeeds(0,0);
    delay(50);
    nextState = 0;
    break;

    case 5:
    motors.setSpeeds(350,350);
    delay(350);
    nextState = 0;
    whiteCounter = 0;
    break;

    case 6:
    charging = true;
    chargeTimer1 = millis();
    if(chargeTimer1>chargeTimer2){
      currentBattery += 0.01*totalBattery;
      chargeTimer2 = chargeTimer1 + 2000;
    }
    
    motors.setSpeeds(0,0);
    if(enterCharging == 0){
      nextState = 4;
      charging = false;
    }
    
    break;

    case 7:
    
    break;
    
  }
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
  
  //VENTER PÅ KNAPPETRYKK FOR KALIBRERING
  buttonA.waitForButton();
  lineSensors.initFiveSensors();
  
  //KALIBRERING:
  
  while(_ticks < 3500){
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
  
  motors.setSpeeds(0,0);
  buzzer.playFrequency(440, 200, 5);
  
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
  long int t1 = millis();
  
  
  if(t1 > t2 && turning == false){
    count += 1;

    
    t2 = t1 + refresh;
    ticks = encoders.getCountsAndResetLeft();
    float distance = (ticks) / (67.96);           // Ticks to cm
    velocity = (distance)/ (refresh/1000.0); //Convert from mm to
        
    //Writing to LCD
    /*
    display.clear();
    display.gotoXY(0, 0);
    display.print(int(velocity)); 
    display.print("CM/s");*/

    //kalles batteristatus hvert sekund
//    if(count == 5){
//      batteryStatus();
//      count = 0;
//    }
      
  }
  return int(velocity);
}

//TOTAL DISTANSE
//VARIABLER SOM MÅ STÅ UTENFOR FUNKSJON
long int t4 = 0;
int distanceTraveled = 0;
int distanceFunc(){
  long int t3 = millis();
  if(t3>t4){
    t4 = t3+refresh;  //Bruker samme refresh-rate som speedometer
    
    //Bruker speedometer funksjonen for å regne ut distanse.
    //Da vil distanse stoppe når farten er 0. Bruker absoluttverdi, siden 
    //rygging er også distanse kjørt
    distanceTraveled += abs(speedFunc())*(refresh/1000.0);;
    display.clear();
    display.print(int(distanceTraveled));display.print("cm");
  }

  return distanceTraveled;
}
//Funksjon for gjennomsnitthastighet siste minutt
unsigned long t6 = 0;
int previousDistance = 0;
int avrageSpeedFunc(){
  int avrageSpeed;
  unsigned long t5 = millis();
  if(t5 > t6){
    t6 = t5 + 60000;
    avrageSpeed = (distanceFunc() - previousDistance)/60;
    previousDistance = distanceFunc();
  }
  return avrageSpeed;
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
  
  //Ryggelademodus
  //Når Zumo ikke beveger bruker den fortsatt batteri. Vi vet at når den bruker mindre enn 10 rygger bilen.
  //Må også ha initiert lademodus
//  if (batteryDrain()<10 && reverseChargingStatus){
//  //batteryDrain blir negativ hvis man rygger fort nok. Ladningsraten velger vi å være 20% av utladningsraten
//    currentBattery = currentBattery - batteryDrain()*0.20;
//  }
  
//Superchargemodus
//  else if(batteryDrain()<10 && superChargingStatus){
//
//    //superCharging gjør ryggeladningen 10X mer effektiv
//    currentBattery = currentBattery - batteryDrain()*0.20*10.0;
//  }
//  
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
    currentBattery = 0;
    motors.setSpeeds(0,0); //Kan ikke kjøre uten batteri
  }
  
  float batteryPercentage = (float(currentBattery) / float(totalBattery))*100.0;
    //display.clear();
    //display.print(int(batteryPercentage)); display.print("%");
  
    
  return batteryPercentage;
}


//funksjon for stopping ved under 5 og 10 prosent strøm
long int currentTime = 15000;
long int spentTime = millis();
bool firstTimeBuzzer = false;               //Sjekker om det er første gangen den går inn i loopen
bool firstTimeUnder5 = false;               //med batteri under 10%. Dette for å sette i gang currentTime                                       
int buzzCounter = 0;
unsigned long buzzTimer1 = millis();
unsigned long buzzTimer2 = 0;

void underXpercentBattery(){
  if (((currentBattery) <= (totalBattery*0.10)) && ((currentBattery) >= (totalBattery*0.05))){
  
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
        }
      }
      if(spentTime>currentTime){
        firstTimeUnder5 = false;
        currentTime = spentTime + 15000;
        buzzCounter = 0;
      }
  }
}
