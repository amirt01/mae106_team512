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
int reedSwitchPin = 10;
bool pistonPosition = true;  // true is top, false is bottom
int solenoidState = LOW;
bool reedSwitchState = 0;  // initially the piston is at the top

// steering variables
int servoPin = 2; //define servo pin attached
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
float revolutions = 0;

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

  state = State::Callibration;  // CHANGE FOR FINAL
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

      snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        running_min.x, running_min.y, running_min.z,
        running_max.x, running_max.y, running_max.z);
      Serial.println(report);

      // Continue callibration for 10 seconds and then assign compass min and max
      if (millis() > previous_time + 10000) {
        compass.m_min = running_min;
        compass.m_max = running_max;
        previous_time = millis();
        state = State::Running;
        desiredHeading = compass.heading();
      }
      break;
    case PositionAssignment:   // Determines statrting point based on competition 
      break;
    case Running:
      // skip everything if we are still in standby
      if (digitalRead(buttonPin)) {
        going = !going;
        delay(15);
      }
      if (!going) break;

      /* STEERING */
      compass.read();
      currentHeading = compass.heading();
      newHeading = Kp * (currentHeading - desiredHeading);
      servoDirection = map(newHeading, compass.m_min.y, compass.m_max.y, 180, 0);  // TODO: CHANGE TO OTHER DIMENSION
      servo.write(servoDirection);
      Serial.println(servoDirection);

      /* POWER */
      // Serial.println(reedSwitchState);   // prints to the serial monitor for debugging
      // TODO: change to use interupts instead
      // Serial.println(digitalRead(reedSwitchPin)); 
      reedSwitchState = digitalRead(reedSwitchPin);   // writes the reedswitch to the pin 
      
      // reed switch is 1 when no magnet, 0 when magnet
      if (!reedSwitchState) // if the magnet is near the reedswitch
        if (reedSwitchState != digitalRead(reedSwitchPin)) {  // prevent double firing
          digitalWrite(solenoidPin, !digitalRead(solenoidPin));  // switch the solenoid state
          revolutions += 0.5;
        }

      /* REED SWITCH */

       reedSwitchState = digitalRead(reedSwitchPin);

      /* VERIFICATION 2 */
      // Serial.print("Distance traveled (mm): ");  // print distance traveled (mm)
      // Serial.println(revolutions / 2 * 69 * M_PI);  // calculate the distance traveled

      break;
    case Finish:
      // Finish run
      break;
  }
  delay(100);
}