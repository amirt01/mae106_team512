#include <Arduino.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

Servo servo;

int servoPin; //define servo pin attached
int solenoid; //pin solenoid is attahced to
int switchPin; //pin that the switch is connected to
int reedSwitch; // variable storing position of reed switch
int servoDirection; // direction servo is pointing
int robotPosition; // stores position of the robot itself
int solenoidState = LOW;
int potentOne;
int potentTwo;
int pistonMode = 1;
int DesiredSteeringAngle;
int TimeBetweenFires;
int PullUpSwitch;
int sensorValue;
int error;
bool going = false;

// power variables
int pistonPosition = 1;
// steering variables
int currentHeading, desiredHeading, newHeading;
int Kp = 3;

// control variables
int ButtonPin = 4;  //TODO: MUST BE A PWM PIN

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
  pinMode(solenoid, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(reedSwitch, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);

  previous_time = millis();
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
      if (!going) {
        if (analogRead(ButtonPin)) going = true;
        else break;
      } else break;

      /* STEERING */
      compass.read();
      currentHeading = compass.heading();
      newHeading = Kp * (currentHeading - desiredHeading);
      servoDirection = map(newHeading, compass.m_min.x, compass.m_max.x, 0.0, 25.0);  // TODO: CHANGE TO OTHER DIMENSION
      analogWrite(servoPin, servoDirection);

      /* POWER */

      // TODO: change to use interupts instead
      if (digitalRead(reedSwitch)) { // if the magnet is near the reedswitch
      if (digitalRead(reedSwitch)) { // if the magnet is near the reedswitch
        pistonPosition++;
        if (pistonPosition > 4) pistonPosition = 1;
      }

      if (pistonPosition < 3)
          solenoidState = HIGH;
      else
          solenoidState = LOW;
      digitalWrite(solenoid, solenoidState);

      /* REED SWITCH */

      reedSwitch = digitalRead(switchPin);

      break;
    case Finish:
      // Finish run
      break;
  }
}
