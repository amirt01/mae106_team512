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
bool pistonPosition = true;  // true is top, false is bottom
int solenoidState = LOW;
bool reedSwitchState = 0;  // initially the piston is at the top

float rotations = -0.5;  // work around to solve the interupt hit issue
float distance = 0;

// steering variables
int servoPin = 6; //define servo pin attached
int servoDirection; // direction servo is pointing
float currentHeading, desiredHeading, newHeading;
int Kp = 1;
int upperBound = 50, lowerBound = 0;  // upper and lower bound for steering

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

void increment() {
  if (state != State::Running) return;

  digitalWrite(solenoidPin, HIGH);  // switch the solenoid state
  // delay(50);
  // digitalWrite(solenoidPin, LOW);

  rotations += 0.5;
  distance = rotations / 2 * M_PI * 69;
  
   // Serial.print("Rotations: ");
   // Serial.println(rotations);  // calculate the distance traveled
   Serial.print("Distance Traveled (mm): ");
   Serial.println(distance);
}

void setup() {
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

  attachInterrupt(digitalPinToInterrupt(reedSwitchPin), increment, FALLING);
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
 
      /* STEERING */
      // this snipet solves the servo twitch on startup problem
      // if (!servo.attached())  // if we haven't attatched the servo yet
      //   servo.attach(servoPin);  // attatch the servo

      // // get the updated compass value
      // compass.read();
      // currentHeading = compass.heading();

      // // calculate the updated heading TODO: there is a bug somewhere here...
      // newHeading = constrain(Kp * (currentHeading - desiredHeading), -180, 180);
      // servoDirection = map(newHeading, -180, 180, lowerBound, upperBound);  // TODO: CHANGE TO OTHER DIMENSION
      // servo.write(servoDirection);

      /* POWER */
      // digitalWrite(solenoidPin, LOW);
      // Serial.println(reedSwitchState);   // prints to the serial monitor for debugging
      // TODO: change to use interupts instead
      // Serial.println(digitalRead(reedSwitchPin)); 

      /* VERIFICATION 2 */
      // Serial.print("Distance traveled (mm): ");  // print distance traveled (mm)
      // Serial.println(revolutions / 2 * 69 * M_PI);  // calculate the distance traveled
      // Serial.print("rotations: ");
      // Serial.println(rotations);

      break;
    case Finish:
      // Finish run
      break;
  }
  delay(100);
}