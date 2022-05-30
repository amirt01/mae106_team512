#include <Arduino.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

Servo servo;

char report[80];

// power variables
int solenoidPin = 2; //pin solenoid is attahced to
int reedSwitchPin = 3;
int solenoidState = LOW;
bool reedSwitchState = 0;  // initially the piston is at the top

float rotations = -0.5;  // work around to solve the interupt hit issue
float distance = 0;

// steering variables
int servoPin = 6; //define servo pin attached 
int servoDirection; // direction servo is pointing
float currentHeading, desiredHeading, deltaHeading;
float theta, beta;
int Kp = 5;
int upperBound = 50, lowerBound = 0;  // upper and lower bound for steering
float targetDistance = 1524;  // 5ft in mm

// control variables
int buttonPin = 4;
bool going = false;
int buttonState = 0;

enum State{
  Callibration,
  PositionAssignment,
  Running,
  Finish,
} state;

unsigned long previous_time = 0;

// Time Variables
unsigned long lastmilli = 0; // time since solenoid opened
unsigned long currentmiilli;
const long interval = 1000; // interval to turn on and off solenoid
float revolutions = 0;

unsigned long stopTime = 0;
unsigned long offset = 50;  // milliseconds

void increment() {
  // only fire the piston when we are in the running mode
  if (state != State::Running) return;

  digitalWrite(solenoidPin, HIGH);  // switch the solenoid state
  stopTime = millis() + offset;

  rotations += 0.5;  // gear ration is 2:1
  distance = rotations / 2 * M_PI * 69;
  // TODO: calculate speed for error calculation
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(reedSwitchPin, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);

  previous_time = millis();

  state = State::Callibration;  // CHANGE FOR FINAL

  attachInterrupt(digitalPinToInterrupt(reedSwitchPin), increment, FALLING);
}

void loop() {
  switch(state) {
    case Callibration:digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      // Callibrate Compass
      compass.read();
      
      // Update min values
      running_min.x = min(running_min.x, compass.m.x);
      running_min.y = min(running_min.y, compass.m.y);
      running_min.z = min(running_min.z, compass.m.z);

      // Update max values
      running_max.x = max(running_max.x, compass.m.x);
      running_max.y = max(running_max.y, compass.m.y);
      running_max.z = max(running_max.z, compass.m.z);

      sprintf(report, "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        running_min.x, running_min.y, running_min.z,
        running_max.x, running_max.y, running_max.z);
      Serial.println(report);

      // Continue callibration for 10 seconds and then assign compass min and max
      if (millis() > previous_time + 10000) {
        compass.m_min = running_min;
        compass.m_max = running_max;
        previous_time = millis();
        state = State::Running;
      }
      break;
    case PositionAssignment:   // Determines statrting point based on competition 
      break;
    case Running:
      // skip everything if we are still in standby
      if (digitalRead(buttonPin)) {
        Serial.println("Button Pressed!");
        delay(25);  // wait 1 second
        going = !going;
        if (going) {
          compass.read();
          desiredHeading = compass.heading();  // set the compass heading to follow
        }
      }
      if (!going) break;
      
      digitalWrite(LED_BUILTIN, HIGH);
      delay(15);
      digitalWrite(LED_BUILTIN, LOW);
 
      /* STEERING */
      // this snipet solves the servo twitch on startup problem
      if (!servo.attached()) { // if we haven't attatched the servo yet
        servo.attach(servoPin);  // attatch the servo
        Serial.println("Servo Attatched!");
      }

      // TODO: check if this is necessary/works
      if (!digitalRead(solenoidPin)) {  // we only change heading when we aren't firing the piston
        // get the updated compass value
        compass.read();
        currentHeading = compass.heading();

        if (distance > targetDistance) {
          desiredHeading += 90;  // TODO: make left/right turn option
          targetDistance = 7620;  // 25ft in mm (all the way down the channel)
        }

        // calculate the smaller angle between desired heading and current heading
        // i.e. should we turn left or right?
        theta = desiredHeading - currentHeading;
        beta = 360 - abs(theta);
        deltaHeading = abs(theta) < beta ? theta : beta;
        servoDirection = constrain(map(Kp * deltaHeading, 180, -180, lowerBound, upperBound), lowerBound, upperBound);
        servo.write(servoDirection);
      }

      /* POWER */
      if (millis() > stopTime)
        digitalWrite(solenoidPin, LOW);

      break;
    case Finish:
      // Finish run
      break;
  }
  delay(100);
}