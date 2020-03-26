// Includes
#include <Stepper.h>
#include <avr/sleep.h>

//#define ENSURE_FIRE
//#define POWER_DOWN // this feature still needs some work
#define DEBUG
// Define number of steps per rotation for small steppers:
const int stepsPerRevolution = 2048;
const int thirdTurn = 683; // one third turn to fire a dart
// steps for big steppers - everything is factored by 4 because of the gear ratio
const float degreesPerStep = 0.45; // 1.8 deg / 4
const int stepsPerRotation = 800; // 200 steps * 4
const int stepDelay = 5; // this determines the rpm

const int DARTS_PER_MAG = 12;
const int NUM_MAGS = 4;
const int firingSpeed = 100;

// codes to send back to controlling device from Arduino
const int NOT_READY = 1;
const int READY = 2;

int numDartsFired;
int numMagsUsed;
bool magsLoaded;

volatile bool joystickControl;
volatile bool fired;

/*********** PINS *************/
// UART
const int TX = 14;
const int RX = 15;

// INTERRUPTS
const int JOYSTICK_SWITCH = 2;
// save pin 3 for possible future interrupt
const int FIRE_FEEDBACK = 3; // if we choose to detect a misfire
// interrupt for when reloading mags is finished -- also used for power down
const int RELOAD_N_POWER_DOWN = 18;
// communication interrupt
const int COMMUNICATION_INTERRUPT = 19;

// PWM
// firing motor
const int SPIN_UP = 8;

// DIGITAL PINS

// joystick button for firing
const int FIRE = 10;

// signals when to stop moving the mag holder
const int MAG_POSITION = 35;

// stop pins for the two vertical ends
const int VERTICAL_TOP = 36;
const int VERTICAL_BOTTOM = 37;

// firing pin
const int FIRING_PIN1 = 38;
const int FIRING_PIN2 = 39;
const int FIRING_PIN3 = 40;
const int FIRING_PIN4 = 41;

// Mag motor
const int MAG_STEPPER1 = 44;
const int MAG_STEPPER2 = 45;
const int MAG_STEPPER3 = 46;
const int MAG_STEPPER4 = 47;

// Horizontal motor
const int HORIZONTAL_DIRECTION = 50;
const int HORIZONTAL_STEP = 51;

// Vertical motor
const int VERTICAL_DIRECTION = 52;
const int VERTICAL_STEP = 53;

// Analog pins
const int XAXIS = A0;
const int YAXIS = A1;

/***************** END OF PINS*********************/

// Create stepper objects, note the pin order: 1 3 2 4
Stepper MagStepper = Stepper(stepsPerRevolution, MAG_STEPPER1, MAG_STEPPER3, MAG_STEPPER2, MAG_STEPPER4);
Stepper FiringPin = Stepper(stepsPerRevolution, FIRING_PIN1, FIRING_PIN3, FIRING_PIN2, FIRING_PIN4);

struct serializedGunCoords
{
  uint8_t horizontal[4];
  uint8_t vertical[4];
};

struct gunCoords
{
  float horizontal;
  float vertical;
};

/*******************************************
 * ISR to start aiming/firing process
 ********************************************/
void wakeupISR(void* param)
{
  sleep_disable();
}

/***************************************************************
 * ISR to toggle between facial recognition and joystick modes
 **************************************************************/
void toggleMode(void* param)
{
  joystickControl = !joystickControl;
}

#ifdef POWER_DOWN
/***************************************************
 * Power Down - always start/end in the same place.
 **************************************************/
 void powerDown(void* param)
{
  digitalWrite(VERTICAL_DIRECTION, LOW);
  while(digitalRead(VERTICAL_BOTTOM))
  {
    stepSingle(VERTICAL_STEP);
  }
  // wait for power to turn off - not the best way to do this. need to look into a better way.
  while(1) {}
}
#endif

#ifdef ENSURE_FIRE
/********************************************
 * ISR to ensure dart firing.
 ********************************************/
void dartFired(void* param)
{
  fired = true;
}
#endif

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
    fired = false;
    // move the firing pin forward
    FiringPin.step(thirdTurn);

#ifdef ENSURE_FIRE
  if (fired == false)
  {
    // set i back one to retry firing
    i--;
  }
  else
  {
    numDartsFired++;
  }
#else
  numDartsFired++;
#endif
  }
}

/***************************************************
 * Move the mag. Detect whether at the end or not.
 **************************************************/
void moveMagHolder()
{
  numMagsUsed++;
  if (numMagsUsed == NUM_MAGS)
  {
    magsLoaded = false;
  }
  else
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
  numMagsUsed = 0;
  magsLoaded = true;
}

/**********************************************
 * Used for spinning up the firing motor
 **********************************************/
void SpinUp()
{
  analogWrite(SPIN_UP, firingSpeed);
}

/**********************************************
 * Used for spinning down the firing motor
 **********************************************/
void SpinDown()
{
  digitalWrite(SPIN_UP, LOW);
}

/********************************************************
 * Sleep while nothing happening
 *******************************************************/
void sleep(int interruptPin)
{
#ifdef DEBUG
  Serial.println("Going to sleep");
  Serial.flush();
#endif
  Serial3.flush();
  // sleep the arduino until the interrupt is triggered
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPin), wakeupISR, RISING);
  sleep_cpu();
  // after waking
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  
#ifdef DEBUG
  Serial.println("I'm awake!");
#endif
}

/******************************************************************************
 * setup for all modes
 ******************************************************************************/
void setup() 
{
  Serial3.begin(9600); // we can probably make this faster
  while (!Serial3) {}
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {}
#endif
  // Set the speed to 13rpm:
  MagStepper.setSpeed(13);
  FiringPin.setSpeed(13);

#ifdef DEBUG
  Serial.println("started stepper motors");
#endif

  // output pins
  pinMode(SPIN_UP, OUTPUT);
  pinMode(HORIZONTAL_STEP, OUTPUT);
  pinMode(HORIZONTAL_DIRECTION, OUTPUT);
  pinMode(VERTICAL_STEP, OUTPUT);
  pinMode(VERTICAL_DIRECTION, OUTPUT);

#ifdef DEBUG
  Serial.println("initialized outputs");
#endif

  // initialize the outputs just to be sure
  digitalWrite(SPIN_UP, LOW);
  digitalWrite(HORIZONTAL_STEP, LOW);
  digitalWrite(HORIZONTAL_DIRECTION, LOW);
  digitalWrite(VERTICAL_STEP, LOW);
  digitalWrite(VERTICAL_DIRECTION, LOW);

#ifdef DEBUG
  Serial.println("set outputs low");
#endif

  // input pins
  pinMode(COMMUNICATION_INTERRUPT, INPUT);
  pinMode(VERTICAL_TOP, INPUT_PULLUP);
  pinMode(VERTICAL_BOTTOM, INPUT_PULLUP);
  pinMode(FIRE_FEEDBACK, INPUT);
  pinMode(MAG_POSITION, INPUT_PULLUP);
  pinMode(RELOAD_N_POWER_DOWN, INPUT);
  pinMode(JOYSTICK_SWITCH, INPUT);
  pinMode(XAXIS, INPUT);
  pinMode(YAXIS, INPUT);
  pinMode(FIRE, INPUT);

#ifdef DEBUG
  Serial.println("initialized inputs");
#endif

  numDartsFired = 0;
  numMagsUsed = 0;
#ifdef DEBUG
  Serial.println("initialized inputs");
#endif

  magsLoaded = false;
  joystickControl = false;

#ifdef DEBUG
  Serial.println("attaching interrupts");
#endif

  // interrupts
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_SWITCH), toggleMode, RISING);
#ifdef ENSURE_FIRE
  attachInterrupt(digitalPinToInterrupt(FIRE_FEEDBACK), dartFired, RISING);
#endif
#ifdef POWER_DOWN
  attachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN), powerDown, RISING);
#endif

#ifdef DEBUG
  Serial.println("finished setup");
#endif
}

/*******************************************************
 * loop for all modes
 ******************************************************/
void loop() {
  if (!magsLoaded)
  {
    // detach all interrupts so we don't accidentally wake up before wanted
#ifdef POWER_DOWN
    detachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN));
#endif
#ifdef ENSURE_FIRE
    detachInterrupt(digitalPinToInterrupt(FIRE_FEEDBACK));
#endif
    detachInterrupt(digitalPinToInterrupt(JOYSTICK_SWITCH));
  
    Serial3.write(NOT_READY); // send a code back to wait while waiting for load
    // sleep until reloaded
    sleep(RELOAD_N_POWER_DOWN);

    //re-attach all interrupts now that we're awake
#ifdef POWER_DOWN
    attachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN), powerDown, RISING);
#endif
#ifdef ENSURE_FIRE
    attachInterrupt(digitalPinToInterrupt(FIRE_FEEDBACK), dartFired, RISING);
#endif
    attachInterrupt(digitalPinToInterrupt(JOYSTICK_SWITCH), toggleMode, RISING);

    // reset the mag holder to the first mag
    resetMagHolder();
  }

  // only do one of the modes. Can't do both at once.
  // This gives a "free hand" option of using a joystick to
  // fire the darts wherever it is desired.
  gunCoords coords;
  bool fire;
  if (joystickControl)
  {
    fire = joystickLoop(coords);
  }
  else
  {
    fire = faceLoop(coords);
  }

  
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
  
  // change mag if needed
  if (numDartsFired == DARTS_PER_MAG)
  {
    moveMagHolder();
  }
}
