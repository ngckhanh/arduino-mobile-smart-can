#include <NewPing.h>            //Ultrasonic sensor function library.
#include <Servo.h>              //Servo motor library.
#include <LiquidCrystal_I2C.h>  //LiquidCrystal I2C library
#include <Wire.h>               //Wire library

//our L298N control pins
#define motorPin1 4     //digital input 4
#define motorPin2 5     //digital input 5
#define motorPin3 6     //digital input 6
#define motorPin4 7     //digital input 7

//sensor pins
#define trigger_pin A0  //analog input 1
#define echo_pin A1     //analog input 2
#define max_distance 600

//button for LC
#define buttonPin 2    //digital input 2

//set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2); 

//checkin the state again for exactly button
boolean buttonState = HIGH;
boolean debounceButton(boolean state) {
  boolean stateNow = digitalRead(buttonPin);
  if (state != stateNow) //if there previous state of button difference with the present state 
  { 
    delay(10);
    stateNow = digitalRead(buttonPin);
  }
  return stateNow;
}

//set conditions of these variables:
int pressed = 0;
int cm = 0;
int duration = 0;
int iterations = 5;
int distance = 0;
int distanceRight = 0;
int distanceLeft = 0;
char val;

//servo pin
int servo_pin = 9;
Servo myservo; //Our servo name is myservo 

NewPing sonar(trigger_pin, echo_pin, max_distance);  //Ultrasonic sensor function
                                      
void setup() {
  pinMode(trigger_pin, OUTPUT); 
  pinMode(echo_pin, INPUT);
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); 
  Serial.begin(9600); //establish serial communication between the Arduino board and another device

  lcd.init();  //initialize the lcd
  lcd.backlight(); //turn on the backlight

  myservo.attach(servo_pin);  //our servor pin

  myservo.write(150);  //adjust the servo's initial rotation
  delay(2000);
}

void loop() {
  if (debounceButton(buttonState) == LOW && buttonState == HIGH) {
    pressed++;
    buttonState = LOW;
  } else if (debounceButton(buttonState) == HIGH && buttonState == LOW) {
    buttonState = HIGH;
  }

  //Begin the project without any pressed button (pressed = 0)
  if (pressed == 0) 
  {
    Serial.println("Press the button to choose");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press the button to choose");
    delay(100);
  }

  //Button has been pressed one time (pressed = 1) leads to Bluetooth mode
  if (pressed == 1) {
    Serial.println("Bluetooth Mode");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bluetooth");
    bluetoothCar();
  }

  //Button has been pressed two times (pressed = 2) leads to Obstacle Avoiding mode
  if (pressed == 2) {
    Serial.println("Obstacle Avoiding Mode");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Obstacle Avoiding");
    obstacleAvoiding();
  }

  //Button has been pressed more than three times it will set back to zero pressed
  if (pressed >= 3)
    pressed = 0;

}

void obstacleAvoiding() //avoiding obstacle function
{
  distance = readPing(); 
  Serial.print("Distance Ahead is: ");
  Serial.println(distance);
  if (distance <= 20)
  {
    Stop();
    delay(50);
    moveBackward();
    delay(400);
    Stop();
    delay(50);

    distanceRight = lookRight();
    Serial.print("Distance Right is: ");
    Serial.println(distanceRight);
    delay(300);

    distanceLeft = lookLeft();
    Serial.print("Distance Left is: ");
    Serial.println(distanceLeft);
    delay(300);

    if (distanceRight >= distanceLeft) {
      turnRight();
      delay(700);
      Stop();
      
    } else {
      turnLeft();
      delay(700);
      Stop();
    }
  } else {
    moveForward();
  }
}

void bluetoothCar() //Bluetooth function
{
  if (Serial.available() > 0) //check the state of serial
  {
    val = Serial.read();
  }

  if (val == 'F')  //move forward
  {
    moveForward();
  } 

  else if (val == 'B')  //move backward
  {
    moveBackward();
  } 

  else if (val == 'S')  //stop moving
  {
    Stop();
  } 
  
  else if (val == 'R')  //turn to right side
  {
    turnRight();
  } 
  
  else if (val == 'L')  //turn to left side
  {
    turnLeft();
  } 
  
  else if (val == 'I')  //Forward Right side
  {
    forwardRight();
  } 
  
  else if (val == 'J')  //Backward Right side
  {
    backwardRight();
  } 
  
  else if (val == 'G')  //Forward Left side
  {
    forwardLeft();
  } 
  
  else if (val == 'H')  //Backward Left side
  {
    backwardLeft();
  }
}

int readPing()  //Calculate the distance for ultrasonic sensor
{
  delay(70);
  duration = sonar.ping_median(iterations);
  cm = (duration / 2) * 0.0343;  //convert duration variable into centimetres
  if (cm >= 400) {
    cm = 400;
  }
  if (cm < 2) {
    cm = 400;
  }
  return cm;
}

int lookLeft() //Servo look its left side function
{
  myservo.write(175); //adjust it to 175 degrees
  delay(500);
  distance = readPing();
  delay(100);
  return distance;
  delay(100);
}

int lookRight() //Servo look its left side function
{
  myservo.write(20); //adjust it to 20 degrees
  delay(500);
  distance = readPing();
  delay(100);
  return distance;
  delay(100);
}

void Stop() //Stop function 
{
  Serial.println("Stop");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Stop");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void moveForward() //Move Forward function
{
  Serial.println("Move Forward");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Move Forward");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}

void moveBackward() //Move Backward function
{
  Serial.println("Move Backward");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Move Backward");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}

void turnRight() //Turn right function
{
  Serial.println("Turn Right");
  Serial.println("");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Turn Right");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}

void turnLeft() //Turn right function
{
  Serial.println("Turn Left");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Turn Left");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void forwardLeft() //Turn left and move forward function
{
  Serial.println("Forward Left");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Forward Left");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}

void forwardRight() //Turn right and move forward function
{
  Serial.println("Forward Right");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Forward Right");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void backwardRight() //Turn right and move backward function
{
  Serial.println("Backward Right");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Backward Right");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}

void backwardLeft() //Turn left and move backward function
{
  Serial.println("Backward Left");
  lcd.setCursor(0, 1); //places the cursor at (0,1) on the screen
  lcd.print("Backward Right");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}