#include <Servo.h>
#include "pitches.h"
#include "constants.h"

/////////////////////////////////////////////////////////////////////////
// CONTROLS /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
int IRpan = IRpanCenter;
int IRtilt = IRtiltCenter;

/////////////////////////////////////////////////////////////////////////
// SETUP ////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void setup(void) {
  // initialize pins
  pinMode(ML_S, OUTPUT);
  pinMode(MR_S, OUTPUT);
  pinMode(ML_D, OUTPUT);
  pinMode(MR_D, OUTPUT);

  pinMode(ML_E, INPUT);
  pinMode(MR_E, INPUT);

  pinMode(CE_I, OUTPUT);

  pinMode(BZ, OUTPUT);

  // initialize head servos
  S_IRpan.attach(SV_B);
  S_IRpan.write(IRpanCenter);
  S_IRtilt.attach(SV_T);
  S_IRtilt.write(IRtiltCenter);

  // play tune on powerup / reset
  if (soundOn) {
    int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
    int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
    for (byte Note = 0; Note < 8; Note++)
    {
      int noteDuration = 1000 / noteDurations[Note];
      tone(BZ, melody[Note], noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
    }
  }

  Serial.begin(9600);
}

/////////////////////////////////////////////////////////////////////////
// MAIN EXECUTION LOOP //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void loop(void) {

  // allow for manual robot control
  manualMotorControl();

  // dead reckoning speed
  float speedReadingL, speedReadingR;
  readMotorSpeed(&speedReadingL, &speedReadingR);
  if (verboseMotorEncoders) {
    Serial.print("L [");
    Serial.print(speedReadingL);
    Serial.print("],  R [");
    Serial.print(speedReadingR);
    Serial.println("]");
  }

  // read the eye sensors
  int leftIRvalue, rightIRvalue, topIRvalue, bottomIRvalue;
  readIReye(&leftIRvalue, &rightIRvalue,
    &topIRvalue, &bottomIRvalue);
  if (verboseIReye) {
    Serial.print("left = ");
    Serial.print(leftIRvalue);
    Serial.print(", right = ");
    Serial.print(rightIRvalue);
    Serial.print(", top = ");
    Serial.print(topIRvalue);
    Serial.print(", bottom = ");
    Serial.println(bottomIRvalue);
  }

  // move the head and body to follow an object
  calcHeadPosition(leftIRvalue, rightIRvalue,
    topIRvalue, bottomIRvalue);
  S_IRpan.write(IRpan);
  S_IRtilt.write(IRtilt);

  /*
  // read distance from ultrasonic rangefinder
  float cm;
  readUSdistance(&cm);
  //Serial.println(cm);
  if(cm <= 5.0) {
  motorsStop();
  }
  */

  if (verboseControls) {
    Serial.print("pan [ctrl = ");
    Serial.print(IRpan);
    Serial.print(", real = ");
    Serial.print(S_IRpan.read());
    Serial.print("]     tilt [ctrl = ");
    Serial.print(IRtilt);
    Serial.print(", real = ");
    Serial.print(S_IRtilt.read());
    Serial.println("]");
  }
}

/////////////////////////////////////////////////////////////////////////
// MANUAL MOTOR CONTROL /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void manualMotorControl() {
  if (Serial.available() >= 1) {
    char val = Serial.read();

    int leftspeed = maxSpeedL;
    int rightspeed = maxSpeedR;

    switch (val) { // Perform an action depending on the command
    case 'v'://Move Forward
      motorsForward(maxSpeedL, maxSpeedR);
      break;
    case 'i'://Move Backwards
      motorsReverse(maxSpeedL, maxSpeedR);
      break;
    case 'u'://Turn Left
      motorsLeft(maxSpeedL, maxSpeedR);
      break;
    case 'a'://Turn Right
      motorsRight(maxSpeedL, maxSpeedR);
      break;
    case 't'://Test

      break;
    case 'z':
      IRpan = S_IRpan.read();
      S_IRpan.write(IRpan - 5);
      break;
    case 'b':
      IRpan = S_IRpan.read();
      S_IRpan.write(IRpan + 5);
      break;
    case 'h':
      IRtilt = S_IRtilt.read();
      S_IRtilt.write(IRtilt + 5);
      break;
    case 'n':
      IRtilt = S_IRtilt.read();
      S_IRtilt.write(IRtilt - 5);
      break;
    default:
      motorsStop();
      break;
    }
  }
}


/////////////////////////////////////////////////////////////////////////
// MOTOR ENCODERS ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void readMotorSpeed(float *speedReadingL, float *speedReadingR) {
  // persistent variables
  static long previousMillis = 0;  // to establish a fake concurrency
  static long interval = 1000;     // (i.e. a thread that doesn't block everything)
  static int encoderCountL = 0;
  static int encoderCountR = 0;
  static bool encoderStateOldL = false;
  static bool encoderStateOldR = false;

  // other variables
  bool encoderStateNewL, encoderStateNewR;
  int directionReadingL = 0, directionReadingR = 0;

  int encoderValueL = analogRead(ML_E);
  int encoderValueR = analogRead(MR_E);

  if (encoderValueL < 600) //Min value is 400 and max value is 800, so state chance can be done at 600.
    encoderStateNewL = true;
  else
    encoderStateNewL = false;

  if (encoderValueR < 600)
    encoderStateNewR = true;
  else
    encoderStateNewR = false;

  if (encoderStateNewL != encoderStateOldL)
    encoderCountL++;
  encoderStateOldL = encoderStateNewL;

  if (encoderStateNewR != encoderStateOldR)
    encoderCountR++;
  encoderStateOldR = encoderStateNewR;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    directionReadingL = digitalRead(ML_D) * 2 - 1;
    directionReadingR = digitalRead(MR_D) * 2 - 1;

    *speedReadingL = float(-directionReadingL * encoderCountL) / 16.0;
    *speedReadingR = float(-directionReadingR * encoderCountR) / 16.0;
    encoderCountL = 0;
    encoderCountR = 0;
  }
}


/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (STOP) ////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void motorsStop(void) { //Stop
  digitalWrite(ML_S, LOW);
  digitalWrite(MR_S, LOW);
}

/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (TRACKS ROTATE IN FORWARD DIRECTION) //////////////////
/////////////////////////////////////////////////////////////////////////
void motorsForward(int l, int r) {
  analogWrite(ML_S, l);
  digitalWrite(ML_D, LOW);
  analogWrite(MR_S, r);
  digitalWrite(MR_D, LOW);
}

/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (TRACKS ROTATE IN BACKWARD DIRECTION) /////////////////
/////////////////////////////////////////////////////////////////////////
void motorsReverse(int l, int r) {
  analogWrite(ML_S, l);
  digitalWrite(ML_D, HIGH);
  analogWrite(MR_S, r);
  digitalWrite(MR_D, HIGH);
}

/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (TRACKS CAUSE LEFT ROTATION) //////////////////////////
/////////////////////////////////////////////////////////////////////////
void motorsLeft(int l, int r) {
  analogWrite(ML_S, l);
  digitalWrite(ML_D, HIGH);
  analogWrite(MR_S, r);
  digitalWrite(MR_D, LOW);
}

/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (TRACKS CAUSE RIGHT ROTATION) /////////////////////////
/////////////////////////////////////////////////////////////////////////
void motorsRight(int l, int r) {
  analogWrite(ML_S, l);
  digitalWrite(ML_D, LOW);
  analogWrite(MR_S, r);
  digitalWrite(MR_D, HIGH);
}

/////////////////////////////////////////////////////////////////////////
// ROBOT MOVEMENT (GENERAL SPEED COMMAND) ///////////////////////////////
/////////////////////////////////////////////////////////////////////////
void motorsGeneralCmd(int l, int r) {
  if (l > 0 && r > 0)
    motorsForward(l, r);
  if (l > 0 && r < 0)
    motorsRight(l, -r);
  if (l < 0 && r > 0)
    motorsLeft(-l, r);
  if (l < 0 && r < 0)
    motorsReverse(-l, -r);
}


/////////////////////////////////////////////////////////////////////////
// ULTRASONIC RANGEFINDER ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
float readUSdistance(float *distance) {
  // This code is based on one of the Arduino example sketches.
  // Establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(US, OUTPUT);
  digitalWrite(US, LOW);
  delayMicroseconds(2);
  digitalWrite(US, HIGH);
  delayMicroseconds(15);
  digitalWrite(US, LOW);
  delayMicroseconds(20);
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(US, INPUT);
  duration = pulseIn(US, HIGH);

  // Convert the time reading into a distance:
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  *distance = float(duration) / 29.0 / 2.0;
}

/////////////////////////////////////////////////////////////////////////
// INFRARED COMPOUND EYE: READING ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// from Dagu Adventure Robot
// Simple IR sensors such as those used on Mr. General cannot measure
// distance accurately and can be tricked by objects of different colours.
// For example a small bright white object in the distance and a large
// dark object nearby can give the same readings depending on how well
// they reflect IR. Sunlight has a lot of infrared light an can easily
// trick or blind these sensors. Background light such as this is called
// ambient light.
// To help overcome some of these limitations we use the method proposed
// by Frits of reading the sensors twice. The first time we read the
// sensor with the IR LEDs on. This gives us a reading that is equal to
// the ambient light plus IR LED light that has reflected from a nearby
// object. The second reading is with the LEDs off so that we read only
// the ambient light. When we subtract the second reading (ambient IR)
// from the first (ambient IR + reflected IR) we are left with a value
// that represents only the IR light that was reflected from a nearby
// object. This technique allows the robot to track your hand movements
// while ignoring the IR light coming in from a nearby window.
void readIReye(int *leftIRvalue, int *rightIRvalue, int *topIRvalue, int *bottomIRvalue)
{
  // persistent variables
  static byte noise = 0;

  // other variable declarations
  int IRaccumulatedReading;

  digitalWrite(CE_I, HIGH);           // turn on IR LEDs to read TOTAL IR LIGHT (ambient + reflected)
  delay(2);                           // allow time for phototransistors to respond (may not be needed)
  *leftIRvalue = analogRead(CE_L);   // TOTAL IR = AMBIENT IR + LED IR REFLECTED FROM OBJECT
  *rightIRvalue = analogRead(CE_R);   // TOTAL IR = AMBIENT IR + LED IR REFLECTED FROM OBJECT
  *topIRvalue = analogRead(CE_T);   // TOTAL IR = AMBIENT IR + LED IR REFLECTED FROM OBJECT
  *bottomIRvalue = analogRead(CE_B);   // TOTAL IR = AMBIENT IR + LED IR REFLECTED FROM OBJECT

  digitalWrite(CE_I, LOW);                            // turn off IR LEDs to read AMBIENT IR LIGHT (IR from indoor lighting and sunlight)
  delay(2);                                           // allow time for phototransistors to respond (may not be needed)
  *leftIRvalue = *leftIRvalue - analogRead(CE_L);     // REFLECTED IR = TOTAL IR - AMBIENT IR
  *rightIRvalue = *rightIRvalue - analogRead(CE_R);    // REFLECTED IR = TOTAL IR - AMBIENT IR
  *topIRvalue = *topIRvalue - analogRead(CE_T);      // REFLECTED IR = TOTAL IR - AMBIENT IR
  *bottomIRvalue = *bottomIRvalue - analogRead(CE_B);   // REFLECTED IR = TOTAL IR - AMBIENT IR

  IRaccumulatedReading = (*leftIRvalue
    + *rightIRvalue
    + *topIRvalue
    + *bottomIRvalue) / 4;   // distance of object is average of reflected IR

  noise++;   // count program loops
  if (noise > 100 && soundOn)
  {
    tone(BZ, IRaccumulatedReading * 5 + 100, 5);    // produce sound every eighth loop - high pitch = close distance
    noise = 0;
  }
}

/////////////////////////////////////////////////////////////////////////
// INFRARED COMPOUND EYE: OBJECT TRACKING ///////////////////////////////
/////////////////////////////////////////////////////////////////////////
// from Dagu Adventure Robot
// Using the pan servo as an example the robot must first read the left
// and right compound eye sensors and subtract the ambient light as
// previously mentioned.
// The distance/reflectivity of the object must be taken into account to
// prevent servo overcorrection. This is done by using a variable called
// "panscale" wich is the average of the left and right readings divided
// by LRscalefactor which is adjusted to allow for servo speed and tracking
// sensitivity. Too sensitive and the servos will overshoot causing them
// to jitter.
// A value called "leftright" is the absolute difference between the left
// sensor and the right sensor divided by the panscale to allow for
// distance and servo speed. This value is then added or subtracted from
// the old servo position to provide a new servo position.
// Tilting of the head works the same way as the panning of the neck.
// Arduino users must forgive the clumbsy code as it has been translated
// from Picaxe basic which has limited math functions and no negative
// numbers.
//
// In order to track an object properly the robot needs to do more than
// turn it's head. It must follow with the body. If the neck pans too
// far left or right then the wheels are driven causing the robot to turn
// towards the object.
// Distance is calculated by averaging the readings of all 4 sensors
// (up, down, left and right). This is not an accurate measurement of
// distance but is good enough for the robot to judge things such as which
// direction is blocked and which direction has room to move. The robot
// ries to maintain a set distance from the object it's tracking so as not
// to loose it's lock on the object while avoiding a collision. The wheels
// are driven forward or backward as required.
void calcHeadPosition(int leftIRvalue, int rightIRvalue, int topIRvalue, int bottomIRvalue)
{
  int speedL = 0; // start with motors set to a speed of 0
  int speedR = 0;

  int pan = S_IRpan.read(); // start with head's current position
  int tilt = S_IRtilt.read();

  int panscale, tiltscale, leftright, updown;
  int panExtremum, motionDirection;

  int IRaccumulatedReading = (leftIRvalue
    + rightIRvalue
    + topIRvalue
    + bottomIRvalue) / 4;   // distance of object is average of reflected IR

  // if there is no object in range, drive the head back
  // to its default position
  if (IRaccumulatedReading < IRobstacleThreshold)
  {
    if (IRpan > IRpanCenter)
      IRpan = IRpan - 1;
    if (IRpan < IRpanCenter)
      IRpan = IRpan + 1;
    if (IRtilt > IRtiltCenter)
      IRtilt = IRtilt - 1;
    if (IRtilt < IRtiltCenter)
      IRtilt = IRtilt + 1;
    motorsStop();
  }
  else // track object with pan & tilt unit
  {
    // to prevent servos from overreacting (jittering)
    panscale = (leftIRvalue + rightIRvalue) / LRscalefactor;
    tiltscale = (topIRvalue + bottomIRvalue) / TBscalefactor;

    leftright = (leftIRvalue - rightIRvalue) / panscale;
    pan = pan + leftright;

    updown = (topIRvalue - bottomIRvalue) / tiltscale;
    tilt = tilt - updown;

    IRpan = constrain(pan, IRpanMin, IRpanMax);
    IRtilt = constrain(tilt, IRtiltMin, IRtiltMax);

    // move forward or backward to follow object
    motionDirection = sign(IRoptimumThreshold - IRaccumulatedReading);
    speedL = motionDirection * maxSpeedL;
    speedR = motionDirection * maxSpeedR;

    // turn the body to follow the object if the "head" pan angle
    // is about to reach its minimum ("right" in body frame)
    // or maximum ("left" in body frame)
    panExtremum = IRpanMax - IRpan;
    if (panExtremum < turnBodyThresholdAngle) {
      switch (motionDirection) {
      case 0:
        speedL = -maxSpeedL + panExtremum;
        speedR = maxSpeedR - panExtremum;
        break;
      case 1:
        speedL = maxSpeedL - panExtremum;
        speedR = maxSpeedR;
        break;
      case -1:
        speedL = -maxSpeedL;
        speedR = -maxSpeedR + panExtremum;
        break;
      }
    }
    panExtremum = IRpan - IRpanMin;
    if (panExtremum < turnBodyThresholdAngle) {
      switch (motionDirection) {
      case 0:
        speedL = maxSpeedL - panExtremum;
        speedR = -maxSpeedR + panExtremum;
        break;
      case 1:
        speedL = maxSpeedL;
        speedR = maxSpeedR - panExtremum;
        break;
      case -1:
        speedL = -maxSpeedL + panExtremum;
        speedR = -maxSpeedR;
        break;
      }
    }

    if(speedL < 0)
        speedL = constrain(speedL, -maxSpeedL, -minSpeedL);
    if(speedL > 0)
        speedL = constrain(speedL, minSpeedL, maxSpeedL);
    if(speedR < 0)
        speedR = constrain(speedR, -maxSpeedR, -minSpeedR);
    if(speedR > 0)
        speedR = constrain(speedR, minSpeedR, maxSpeedR);

    motorsGeneralCmd(speedL, speedR);
  } // END else (leftIRvalue > rightIRvalue)

} // END function

/////////////////////////////////////////////////////////////////////////
// SIGN /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
int sign(int var) {
  if (var > 0)
    return 1;
  if (var < 0)
    return -1;
  return 0;
}
