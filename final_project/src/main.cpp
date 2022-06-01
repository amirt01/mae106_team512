#include <Arduino.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>
#include <float.h>

#define REED_SWITCH_PIN 2
#define SOLENOID_PIN 3
#define BUTTON_PIN 4
#define SERVO_PIN 5

// Initialize Library Objects
Servo servo;
LSM303 compass;

// Initialize Compass Calibration Values
LSM303::vector<int16_t> running_min = {32767, 32767, 32767};
LSM303::vector<int16_t> running_max = {-32768, -32768, -32768};
unsigned long calibrationStartTime;
const unsigned short callibrationDurration = 10000;

// Power Variables
bool reedSwitchState = 0;  // initially the piston is at the top

// Steering Variables
int servoDirection, constrainedServoDirection;
float currentHeading, desiredHeading, deltaHeading;
float theta, beta;
const unsigned short Kp = 5;
const unsigned short STEERING_CENTER = 92;
const unsigned short STEERING_DELTA = 10;
const unsigned short STEERING_UPPER_BOUND = STEERING_CENTER + STEERING_DELTA;
const unsigned short STEERING_LOWER_BOUND = STEERING_CENTER - STEERING_DELTA;  // upper and lower bound for steering
float targetDistance = 1.5 * 150 * 10;  // 5ft in mm

// Power Timing
unsigned long stopTime = 0;
unsigned long fireTime = 25;

// Competition Timing
unsigned long runningStartTime = 0;
unsigned long runningDurration = 60000;
unsigned long launchDelay = 15000;
unsigned long blinkTime = 0;
unsigned short blinkDelay = 15;

// Controls
float rotations = -0.5;  // work around to solve the interupt hit issue
float distance = 0;
bool going = false;

enum State{
  Callibration,
  PositionAssignment,
  Running,
  Finish,
} controlState;

// This function will be called every time the reed switch is triggered
void increment() {
  if (controlState != State::Running) return;  // only fire the piston when we are in the running mode

  Serial.println("Firing!");

  digitalWrite(SOLENOID_PIN, HIGH);
  stopTime = millis() + fireTime;

  rotations += 0.5;  // gear ration is 2:1
  distance = rotations / 2 * M_PI * 69;
  // TODO: calculate speed for error calculation
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), increment, FALLING);

  Serial.begin(9600);

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);

  controlState = State::Callibration;
  calibrationStartTime = millis();
}

void loop() {
  switch(controlState) {
    case Callibration:
      blinkDelay = 1000;

      // Callibrate Compass
      compass.read();
      running_min.x = min(running_min.x, compass.m.x);
      running_min.y = min(running_min.y, compass.m.y);
      running_min.z = min(running_min.z, compass.m.z);
      running_max.x = max(running_max.x, compass.m.x);
      running_max.y = max(running_max.y, compass.m.y);
      running_max.z = max(running_max.z, compass.m.z);

      // Callibrate for 10 seconds, then assign compass min and max
      if (millis() > calibrationStartTime + callibrationDurration) {
        compass.m_min = running_min;
        compass.m_max = running_max;
        calibrationStartTime = millis();

        controlState = State::Running;
        blinkDelay = 100;
      }
      break;
      
    case PositionAssignment:
      // TODO: Determines statrting point based on competition 
      break;
    
    case Running:
      if (digitalRead(BUTTON_PIN)) {
        Serial.println("Starting to Go!");
        delay(100);  // debounce the button

        going = true;
        blinkDelay = 15;

        // update the compass heading
        compass.read();
        desiredHeading = compass.heading();
       
        digitalWrite(LED_BUILTIN, HIGH);
        delay(launchDelay);  // wait 15 seocnds before launching

        runningStartTime = millis();       
        increment();
      } else if (!going) break;  // skip everything if we are still in standby
      
      /* STEERING */
      // this snipet solves the servo twitch on startup problem
      if (!servo.attached()) { // if we haven't attatched the servo yet
        servo.attach(SERVO_PIN);  // attatch the servo
        Serial.println("Servo Attatched!");
      }

      // Instead of implementing a low pass filter, we only change heading when we aren't firing the piston.
      // This is not only more accurate, but also less resource intensive
      if (!digitalRead(SOLENOID_PIN)) {
        Serial.println("Steering!");
        
        compass.read();
        currentHeading = compass.heading();

        if (distance > targetDistance) {
          desiredHeading -= 90;  // TODO: make left/right turn option
          targetDistance = FLT_MAX_EXP;  // 25ft in mm (all the way down the channel)
        }

        // Calculate the smaller angle between desired heading and current heading
        // i.e. should we turn left or right?
        theta = desiredHeading - currentHeading;
        beta = 360 - abs(theta);
        deltaHeading = abs(theta) < beta ? theta : beta;  // pick the smaller angle difference
        servoDirection = map(Kp * deltaHeading, -180, 180, STEERING_LOWER_BOUND, STEERING_UPPER_BOUND);
        constrainedServoDirection = constrain(servoDirection, STEERING_LOWER_BOUND, STEERING_UPPER_BOUND);
        servo.write(servoDirection);
      }

      /* POWER */
      if (millis() > stopTime)
        digitalWrite(SOLENOID_PIN, LOW);

      // Hard stop after 1 minute
      if (millis() > runningStartTime + runningDurration) {
        digitalWrite(SOLENOID_PIN, HIGH);
        controlState = State::Finish;
        blinkDelay = 100;
      }
      break;

    case Finish:
      break;
  }

  // This snippet allows us to indicate what mode we are in with the built in LED without
  // adding any delay.
  if (millis() > blinkTime) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    blinkTime = millis() + blinkDelay;
  }
}