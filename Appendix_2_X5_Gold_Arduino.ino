#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>

// PINS
//  encoder
const int encoder = 3;

//  Left motor connections
int enL = A3;
int in1 = 8;
int in2 = 9;

// Right motor connections
int enR = A2;
int in3 = 5;
int in4 = 4;

//US eyes
const int trigPin = A0;
const int echoPin = A1;


// Variables

// WiFi
const char SSID[] = "HotWheels";
const char PASSWORD[] = "KaChow67";
WiFiServer server1(5700);
WiFiServer server2(6800);
bool start = false;
char ch, ch2;

//  gyroscope values
float x, y, z;

//  angle values
float change;
float angle = 0;

//  time values
unsigned long currTime, elapsedTime;
unsigned long prevTime = 0;

//  encoder pulse count
volatile int LpulseCount;

//  encoder speed output
double RPM;
int buggySpeed;

//  motor speed bits
int badSpeed;

//  PID
unsigned long currentTime;
unsigned long previousTime = 0;
double error;
double lastError;
double input, speed;
double Setpoint = 23;
double cumError, rateError;
int c_speed;

//  PID constants
double kp = 15;
double ki = 0;
double kd = .2;

//  speed and angle to height_mm
float angleRads;
double sinVal;
double height_mm = 0;
double height_cm, angleRes;

//  Sending
String message;

void setup() {
  Serial.begin(9600);
  // WiFi
  WiFi.begin(SSID, PASSWORD);
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  server1.begin();
  server2.begin();

  // Set all the motor control pins to outputs
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // US pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // FOR ENCODER
  pinMode(encoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder), Lincrement, CHANGE);
  LpulseCount = 0;



  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in °/second");

  STOP();
}

void loop() {
  // WiFi
  WiFiClient Client1 = server1.available();
  WiFiClient Client2 = server2.available();

  if (Client1.connected()) {
    Client1.write('p');
    ch = Client1.read();
    Serial.print(ch);
    if (ch == 'w') {
      start = true;
    }

    if (ch == 's') {
      start = false;
    }

    if (ch == 'r') {
      RESET();
    }
    Serial.println(ch);
  }

  if (start == true) {
    // MOVE
    MOVE();

  } else if (start == false) {
    STOP();
  }

  //  all calculations every 50ms
  currTime = millis();
  elapsedTime = currTime - prevTime;
  if (elapsedTime >= 50) {
    calcVelocity();
    calcAngle();
    calcHeight_mm();
    height_cm = height_mm / 10;
    angleRes = angle;
    if (angleRes > -2 && angleRes < 2) { angleRes = 0; }
    prevTime = currTime;
  }

  // Send data
  String message = String(buggySpeed) + "," + String(angleRes) + "," + String(height_cm);
  if (Client2.connected()) {
    ch2 = Client2.read();
    server2.println(message);
    Serial.print(ch2);
    // Serial.print("  ");
    Serial.println(message);
  }
}

// FUNCTIONS

// RESET
void RESET() {
  angle = 0;
  height_mm = 0;
  height_cm = 0;
  buggySpeed = 0;
}
//  Encoder count
void Lincrement() {
  LpulseCount++;
}
//  Encoder velocity calculation
void calcVelocity() {
  RPM = ((LpulseCount / 8) * (60000 / elapsedTime)) / 120;
  buggySpeed = (6.75 * 3.14159 * RPM) / 60;
  Serial.println(LpulseCount);
  Serial.println(buggySpeed);
  LpulseCount = 0;
}

//  IMU angle calculation
void calcAngle() {
  if (IMU.gyroscopeAvailable()) { IMU.readGyroscope(x, y, z); }
  //  angle change accounting for error
  change = x - 0.4;
  if (change > 0.15 || change < -0.15) {
    if (digitalRead(in1) == LOW && digitalRead(in2) == LOW) {
      angle = angle + (change * 3.6 / elapsedTime);
    }
    if (digitalRead(in1) != LOW || digitalRead(in2) != LOW) {
      angle = angle + (change * 9.4 / elapsedTime);
    }
    if (angle > 360) { angle = angle - 360; };
    if (angle < 0.1 && angle > -0.1) { angle = 0; };
  }
}

// Calculation of current height_mm above start point
void calcHeight_mm() {
  if (angle < 2 && angle > -2) {
    height_mm = height_mm;
  } else {
    angleRads = angle * (3.14159 / 180);
    sinVal = (sin(angleRads));
    height_mm = height_mm + ((buggySpeed * elapsedTime / 100) * sinVal);
  }
/*  Serial.print(sinVal);
  Serial.println(" sin");
  Serial.print(buggySpeed);
  Serial.println("cm/s");
  Serial.print(elapsedTime);
  Serial.println("ms");
  Serial.print(height_mm);
  Serial.println("mm"); */
}
// Standard movement fucntion
void MOVE() {
  int distance = poll_distance();
  badSpeed = computePID(distance);
  speed = constrain(badSpeed, 0, 255);

  if (distance >= 15) {
    analogWrite(enL, (255 - speed));
    analogWrite(enR, (255 - speed));
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    STOP();
  }
}


// STOP function
void STOP() {
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

//  Polling
float poll_distance() {
  // polling
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  return (duration * 0.034 / 2);
}

double computePID(double inp) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation

  error = (Setpoint - inp);                       // determine error
  cumError += (error * elapsedTime);              // compute integral
  rateError = (error - lastError) / elapsedTime;  // compute derivative

  double out = (kp * error) + (ki * cumError) + (kd * rateError);  //PID output

  lastError = error;           //remember current error
  previousTime = currentTime;  //remember current time

  return out;  //have function return the PID output
}
