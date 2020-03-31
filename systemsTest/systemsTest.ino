#include<Stepper.h>

// Define number of steps per rotation for small steppers:
const int stepsPerRevolution = 2048;
const int thirdTurn = 683; // one third turn to fire a dart
// steps for big steppers - everything is factored by 4 because of the gear ratio
const float degreesPerStep = 0.45; // 1.8 deg / 4
const int stepsPerRotation = 800; // 200 steps * 4
const int stepDelay = 5; // this determines the rpm

const int NUM_MAGS = 4;

// PWM
// firing motor
const int SPIN_UP = 8;

// signals when to stop moving the mag holder
const int MAG_POSITION = 16;

// limit switch for the two vertical ends
const int VERTICAL_TOP = 52;
const int VERTICAL_BOTTOM = 53;

// firing pin
const int FIRING_PIN1 = 38;
const int FIRING_PIN2 = 39;
const int FIRING_PIN3 = 40;
const int FIRING_PIN4 = 41;

// Mag motor
const int MAG_STEPPER1 = 42;
const int MAG_STEPPER2 = 43;
const int MAG_STEPPER3 = 44;
const int MAG_STEPPER4 = 45;

// Horizontal motor
const int HORIZONTAL_DIRECTION = 24;
const int HORIZONTAL_STEP = 25;

// Vertical motor
const int VERTICAL_DIRECTION = 22;
const int VERTICAL_STEP = 23;

// Create stepper objects, note the pin order: 1 3 2 4
Stepper MagStepper = Stepper(stepsPerRevolution, MAG_STEPPER1, MAG_STEPPER3, MAG_STEPPER2, MAG_STEPPER4);
Stepper FiringPin = Stepper(stepsPerRevolution, FIRING_PIN1, FIRING_PIN3, FIRING_PIN2, FIRING_PIN4);

struct gunCoords
{
  float horizontal;
  float vertical;
};

/********************************************
 * single step
 *********************************************/
void stepSingle(int pin)
{
  digitalWrite(pin, HIGH);
  delay(stepDelay);
  digitalWrite(pin, LOW);
}

/********************************************
 * multiple steps
 *********************************************/
void stepMultiple(int pin, int steps, int checkPin = 0)
{
  for (int i = 0; i < steps; i++)
  {
    stepSingle(pin);
    if (checkPin != 0)
    {
      if (!digitalRead(checkPin))
      {
        return;
      }
    }
    delay(stepDelay);
  }
}

/*************************************************
 * turret aiming
 **************************************************/
void aim(gunCoords coords)
{
  int horizontalDirection = coords.horizontal < 0 ? LOW : HIGH;
  int verticalDirection = coords.vertical < 0 ? LOW : HIGH;
  int checkPin = verticalDirection == LOW ? VERTICAL_BOTTOM : VERTICAL_TOP;

  // determine how many steps to take
  float horizontalSteps = (int)(abs(coords.horizontal) / degreesPerStep);
  float verticalSteps = (int)(abs(coords.vertical) / degreesPerStep);

  // set the direction
  digitalWrite(HORIZONTAL_DIRECTION, horizontalDirection);
  digitalWrite(VERTICAL_DIRECTION, verticalDirection);
  delay(10);
  // take the steps
  stepMultiple(HORIZONTAL_STEP, horizontalSteps);
  stepMultiple(VERTICAL_STEP, verticalSteps, checkPin);
}

/*****************************************
 *
 ******************************************/
void fireDart(int numDarts = 1)
{
  int pos = 0;
  for (int i = 0; i < numDarts; i++)
  {
    // move the firing pin forward
    FiringPin.step(thirdTurn);
  }
}


/********************************************************
 * Sleep while nothing happening
 *******************************************************/
void fireSequence(bool fire, gunCoords coords)
{
  if (fire)
  {
    SpinUp();
  }
  aim(coords);
  if (fire)
  {
    fireDart();
    SpinDown();
  }
}

/***************************************************
 * Move the mag. Detect whether at the end or not.
 **************************************************/
void moveMagHolder()
{
  while (!digitalRead(MAG_POSITION))
  {
    MagStepper.step(1);
  }
  while (digitalRead(MAG_POSITION))
  {
    MagStepper.step(1);
  }
}

/***************************************************
 * Reset the mag holder.
 **************************************************/
void resetMagHolder()
{
  int mag = 1;
  while (mag != NUM_MAGS)
  {
    if (!digitalRead(MAG_POSITION))
    {
      mag++;
      while(!digitalRead(MAG_POSITION) && mag != NUM_MAGS)
      {
        MagStepper.step(-1);
      }
    }
    MagStepper.step(-1);
  }
}

/**********************************************
 * Used for spinning up the firing motor
 **********************************************/
void SpinUp()
{
  analogWrite(SPIN_UP, 256);
}

/**********************************************
 * Used for spinning down the firing motor
 **********************************************/
void SpinDown()
{
  digitalWrite(SPIN_UP, LOW);
}

/******************************************************************************
 * setup for all modes
 ******************************************************************************/
void setup() 
{
  // Set the speed to 13rpm:
  MagStepper.setSpeed(5);
  FiringPin.setSpeed(3);

  // output pins
  pinMode(SPIN_UP, OUTPUT);
  pinMode(HORIZONTAL_STEP, OUTPUT);
  pinMode(HORIZONTAL_DIRECTION, OUTPUT);
  pinMode(VERTICAL_STEP, OUTPUT);
  pinMode(VERTICAL_DIRECTION, OUTPUT);

  // initialize the outputs just to be sure
  digitalWrite(SPIN_UP, LOW);
  digitalWrite(HORIZONTAL_STEP, LOW);
  digitalWrite(HORIZONTAL_DIRECTION, LOW);
  digitalWrite(VERTICAL_STEP, LOW);
  digitalWrite(VERTICAL_DIRECTION, LOW);

  // input pins
  pinMode(VERTICAL_TOP, INPUT_PULLUP);
  pinMode(VERTICAL_BOTTOM, INPUT_PULLUP);
  pinMode(MAG_POSITION, INPUT_PULLUP);

}


void loop() {
  // move 45 degrees without firing
  gunCoords coords;
  coords.horizontal = 45;
  coords.vertical = 0;
  fireSequence(false, coords);

  // move back 45 degrees and fire
  coords.horizontal = -45;
  fireSequence(true, coords);
  moveMagHolder();

  // move up 15 degrees and fire
  coords.horizontal = 0;
  coords.vertical = 15;
  fireSequence(true, coords);
  moveMagHolder();

  // move back down 15 degrees and fire
  coords.vertical = -15;
  fireSequence(true, coords);
  moveMagHolder();

  // stay put and fire
  coords.vertical = 0;
  fireSequence(true, coords);
  moveMagHolder();

  // reset mag holder
  resetMagHolder();

  // delay 10 seconds
  delay(10000);
  
}
