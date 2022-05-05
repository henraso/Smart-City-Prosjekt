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

// Motorverdier basert på joystick fra ESPNOW.
int motorVal[1];



void setup() {
//  Snakker med ESP gjennom Serial 1
  Serial1.begin(115200);
  motors.setSpeeds(100,100);
  Serial.begin(115200);
  delay(5000);

}

void loop() {

//Enkel while løkke for å lese informasjon som kommer via Serial og pakker det i en array.
  int index  = 0;                   // index skal alltid være 0 ved ny lesing av data
  
  while(Serial1.available()>0){
    String incoming = Serial1.readStringUntil('>'); 
   // Serial.println(incoming);
    motorVal[index] = incoming.toInt();
    index++;
    if(index>1){
      index = 0;
    }
  }
  delay(5);
  int left = motorVal[0];
  int right = motorVal[1];
  Serial.print(left); Serial.print("      ");
  Serial.println(right);

//  Mottar verdier for hvilke belter som skal kjøre fra serial.
   motors.setSpeeds(left, right);
}
