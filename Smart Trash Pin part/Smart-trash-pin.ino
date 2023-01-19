#include <Servo.h>
Servo myServo; // define our Servo.


// 1: ultrasonic sensor that is used to measure the amount of trash inside.
// 2: ultrasonic sensor that is used to detect the trash in front of the trash can.
#define ECHOPIN1 13
#define TRIGPIN1 12
#define ECHOPIN2 11
#define TRIGPIN2 10


// The LEDs that represent the trash level. More lights will be on when the trash is higher.
#define LED1 7
#define LED2 6
#define LED3 5
#define LED4 4
#define LED5 3


// duration is the time since the ultrasonic sensor sends an ultrasonic wave until the wave reflects after colliding with a certain object , by using that we can measure the distance.


long duration1, distance1;
long duration2, distance2;


void setup() {
  // ultrasonic sensor 1 setup
  Serial.begin(9600);
  pinMode(ECHOPIN1, INPUT);
  pinMode(TRIGPIN1, OUTPUT);
  // LED setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);


  // ultrasonic sensor 2 setup
  myServo.attach(9); // servo on digital pin 9
  myServo.write(180);
  pinMode(ECHOPIN2, INPUT);
  pinMode(TRIGPIN2, OUTPUT);
}


void loop() {
  // LED control
  digitalWrite(TRIGPIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN1, HIGH);
  delayMicroseconds(10);


  digitalWrite(ECHOPIN1, LOW);
  duration1 = pulseIn(ECHOPIN1, HIGH);
  // speed = 344 m/s based on the equation of speed of sound with the assumption that the temperature is 20 and the humidity is 50. Duration is measured in milliseconds.
  distance1 = int(duration1*0.0344/2);  // widely recommended equation to measure the distance using duration, based on 2 * distance = speed * duration (2* distance is from transmitter to receiver and from receiver back to transmitter)   
  // the trash can's height is 18cm
  if (distance1 < 20) {
    digitalWrite(LED1, 150);
  } else {
    digitalWrite(LED1, 0);
  }  


  if (distance1 < 16) {
    digitalWrite(LED2, 150);
  } else {
    digitalWrite(LED2, 0);
  }
   
  if (distance1 < 12) {
    digitalWrite(LED3, 150);
  } else {
    digitalWrite(LED3, 0);
  }
  if (distance1 < 8) {
    digitalWrite(LED4, 150);
  } else {
    digitalWrite(LED4, 0);
  }
   
  if (distance1 < 4) {
    digitalWrite(LED5, 150);
  } else {
    digitalWrite(LED5, 0);
  }


  // Servo control
  digitalWrite(TRIGPIN2, LOW);
  delay(2);  
  digitalWrite(TRIGPIN2, HIGH);
  delayMicroseconds(10);  
  digitalWrite(TRIGPIN2, LOW);  
  duration2 = pulseIn(ECHOPIN2, HIGH);  
  distance2 = int(duration2*0.0344/2);  


  // The lid will only be open when there is trash in front of the trash can and there is space for more trash.
  if (distance2 < 10) {
    if (distance1 > 4) {
      myServo.write(0);  // Turn Servo to open the lid
      delay(3000); // Wait for the user to put trash in
      //myServo.write(0); 
    }
  } else {
     myServo.write(180); // Close the lid
     delay(50);
  }
}