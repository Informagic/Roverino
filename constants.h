/////////////////////////////////////////////////////////////////////////
// GENERAL //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
const bool soundOn = false;
const bool verboseMotorEncoders = false;
const bool verboseIReye = false;
const bool verboseControls = false;

/////////////////////////////////////////////////////////////////////////
// PINS /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
const int ML_S = 6; // Left Motor Speed Control (digital, PWM)
const int MR_S = 5; // Right Motor Speed Control (digital, PWM)
const int ML_D = 8; // Left Motor Direction Control (digital)
const int MR_D = 7; // Right Motor Direction Control (digital)
const int ML_E = 0; // Left Motor Encoder (analog)
const int MR_E = 1; // Right Motor Encoder (analog)

const int CE_T = 4; // IR Compound Eye, top (analog)
const int CE_L = 5; // IR Compound Eye, left (analog)
const int CE_B = 2; // IR Compound Eye, bottom (analog)
const int CE_R = 3; // IR Compound Eye, right (analog)
const int CE_I = 11; // IR Compound Eye, IR LEDs (digital, PWM)

const int US = 4; // Ultrasonic Sensor (digital)

const int BZ = 3; // Buzzer (digital, PWM)

const int SV_B = 9; // Bottom Servo of Pan & Tilt Unit (digital, PWM disabled by servo library)
const int SV_T = 10; // Top Servo of Pan & Tilt Unit 

/////////////////////////////////////////////////////////////////////////
// IR COMPOUND EYE //////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// http://letsmakerobots.com/node/24348
Servo S_IRpan;
Servo S_IRtilt;

// values are degrees
const int IRpanCenter = 90;
const int IRpanMax = 160;
const int IRpanMin = 20;
const int IRtiltCenter = 120;
const int IRtiltMax = 180;
const int IRtiltMin = 70;

// needed for object tracking
const short LRscalefactor = 15;
const short TBscalefactor = 15;
const int IRobstacleThreshold = 200;
const int IRoptimumThreshold = 500;
const int turnBodyThresholdAngle = 30;

/////////////////////////////////////////////////////////////////////////
// PROPULSION SYSTEM ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
const int maxSpeedL = 255; //255 is maximum speed
const int maxSpeedR = 255;
const int minSpeedL = 240;
const int minSpeedR = 240;
