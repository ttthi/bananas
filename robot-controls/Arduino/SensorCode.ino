#include <Servo.h>

Servo servo;
const int sensorPin = A0;
const int servoPin = 8;
const int trigPin = 9;
const int echoPin = 10;
const int obsPin = 7;

int sensorCutOff = 100; // Value between min force 0 and max force 1023
int closingSpeed = 2; // Value between 1 and 180
bool graplerStat = true; // True for open, False for closed
String command = "";

// Functions for opening and closing the arm
void open();
void closed();
// Function for reading distance from the ultrasonic sensor
int distance();
// Function for counting steps determined visual obstruction sensor 
bool countdown();

// Setup pins
void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  open(); // Set grapler to default (open)
}

// Program main loop
void loop() {
  command = Serial.readStringUntil('\n'); // Read command from Python

  if (command.startsWith("open")) {
    if (!graplerStat) {
      open();
      graplerStat = true;
    }
  } else if (command.startsWith("close")) {
    if (graplerStat) {
      close();
      graplerStat = false;
    }
  } else if (command.startsWith("distance")) {
    Serial.println(distance());
  } else if (command.startsWith("countdown")) {
  int colonIndex = command.indexOf(':');
  if (colonIndex != -1) {
    String valueStr = command.substring(colonIndex + 1);
    int countdownValue = valueStr.toInt();
    Serial.println(countdown(countdownValue));
  }
}

  delay(100);
}

void open() {
  int angle = servo.read();
  while (angle >= 0) {
    servo.write(angle);
    delay(100);
    angle -= closingSpeed;
  } 
}

void close() {
  int sensorValue = 0;
  int angle = 0;
  while (angle <= 180) {
    delay(100); // Wait so the sensor can read
    sensorValue = analogRead(sensorPin);
    if (sensorValue >= sensorCutOff) {
      break;
    }

    servo.write(angle);
    angle += closingSpeed;
  } 
}

int distance() {
  long duration = 0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

bool countdown(int target) {
  int sensorValue = digitalRead(obsPin);
  int b = sensorValue;
  int count = 0;
  while (count < target) {
    sensorValue = digitalRead(obsPin);
    if (sensorValue == 1 && b == 0) {
      b = 1;
      count += 1;
    } else if (sensorValue == 0 && b == 1) {
      b = 0;
    }
  }
  return true;
}
