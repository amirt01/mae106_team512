#include <Arduino.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

Servo servo;


// power variables
int solenoidPin = 2; //pin solenoid is attahced to
int reedSwitchPin = 10;
bool pistonPosition = true;  // true is top, false is bottom
int solenoidState = LOW;
int reedSwitchState; // variable storing position of reed switch

// steering variables
int servoPin; //define servo pin attached
int servoDirection; // direction servo is pointing
int currentHeading, desiredHeading, newHeading;
int Kp = 3;

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

void setup() {
  servo.attach(servoPin);
  pinMode(solenoidPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(reedSwitchPin, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);

  previous_time = millis();

  state = State::Running;  // CHANGE FOR FINAL
}

void loop() {
  switch(state) {
    case Callibration:
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
    
      // Continue callibration for 10 seconds and then assign compass min and max
      if (millis() > previous_time + 10000) {
        compass.m_min = running_min;
        compass.m_max = running_max;
        previous_time = millis();
      }
      break;
    case PositionAssignment:
      break;
    case Running:
      // skip everything if we are still in standby
      if (digitalRead(buttonPin)) {
        going = !going;
        delay(15);
      }
      if (!going) break;

      /* STEERING */
      // compass.read();
      // currentHeading = compass.heading();
      // newHeading = Kp * (currentHeading - desiredHeading);
      // servoDirection = map(newHeading, compass.m_min.x, compass.m_max.x, 0.0, 25.0);  // TODO: CHANGE TO OTHER DIMENSION
      // analogWrite(servoPin, servoDirection);

      /* POWER */
      // Serial.println(reedSwitchState);
      // TODO: change to use interupts instead
      Serial.println(digitalRead(reedSwitchPin));
      bool reedSwitchState = digitalRead(reedSwitchPin);
      if (!reedSwitchState) { // if the magnet is near the reedswitch
        digitalWrite(solenoidPin, !digitalRead(solenoidPin));
        while(digitalRead(reedSwitchPin) == reedSwitchState);
      }

      /* REED SWITCH */

      // reedSwitchState = digitalRead(reedSwitchPin);

      break;
    case Finish:
      // Finish run
      break;
  }
  delay(100);
}