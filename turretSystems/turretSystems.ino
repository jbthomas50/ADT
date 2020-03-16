// Includes
#include <Stepper.h>
#include <avr/sleep.h>

//#define ENSURE_FIRE
//#define POWER_DOWN // this feature still needs some work
#define DEBUG
// Define number of steps per rotation for small steppers:
const int stepsPerRevolution = 2048;
const int thirdTurn = stepsPerRevolution / 3;
const int stepDelay = 10;
// steps for big steppers - everything is factored by 4 because of the gear ratio
const int degreesPerStep = 0.45; // 1.8 deg / 4
const int stepsPerRotation = 800; // 200 steps * 4

const int DARTS_PER_MAG = 12;
const int NUM_MAGS = 4;
const int firingSpeed = 100;

// codes to send back to controlling device from Arduino
const int NOT_READY = 1;
const int READY = 2;

int numDartsFired;
int numMagsUsed;
bool magsLoaded;

volatile bool fired;

/*********** PINS *************/
// UART
const int TX = 14;
const int RX = 15;

// INTERRUPTS
// communication interrupt
const int COMMUNICATION_INTERRUPT = 2;
// save pin 3 for possible future interrupt
const int FIRE_FEEDBACK = 3; // if we choose to detect a misfire
// interrupt for when reloading mags is finished -- also used for power down
const int RELOAD_N_POWER_DOWN = 18;

// PWM
// firing motor
const int SPIN_UP = 8;

// DIGITAL PINS
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
} previousCoords;

/*******************************************
 * ISR to start aiming/firing process
 ********************************************/
void wakeupISR(void* param)
{
  sleep_disable();
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
  while(1);
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
  int horizontalSteps = coords.horizontal / degreesPerStep;
  int verticalSteps = coords.vertical / degreesPerStep;

  // set the direction
  digitalWrite(HORIZONTAL_DIRECTION, horizontalDirection);
  digitalWrite(VERTICAL_DIRECTION, verticalDirection);
  
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

/***************************************************************
 * Serial input is read in bytes, so we have to deserialize it
 **************************************************************/
template <class t>
t deserialize(uint8_t *serialData)
{
    t data;
    memcpy(&data, (void*)serialData, sizeof(t));
    return data;
}

/*******************************************************************************************************
 * get the start frame. make sure we are syncing up with the sender so we recieve correct information
 * TODO: check for correct data in start frame before starting the rest of the recieve process.
 *******************************************************************************************************/
bool getStartFrame()
{
  Serial3.read();
  Serial3.read();
  return true;
}

/******************************************************************
 * empty the buffer
 * TODO: check for end frameto ensure data is received correctly
 *****************************************************************/
void getEndFrame()
{
  while (Serial3.available())
  {
    Serial3.read();
  }
}

/********************************************************
 * Get the coordinates and the command to fire or not
 *******************************************************/
bool getInput(serializedGunCoords & ser_coords)
{
  bool fire = false;
  while (!Serial3.available()){}
  if(getStartFrame())
  {
  // Horizontal angle
  ser_coords.horizontal[0] = Serial3.read();
  ser_coords.horizontal[1] = Serial3.read();
  ser_coords.horizontal[2] = Serial3.read();
  ser_coords.horizontal[3] = Serial3.read();
  // vertical angle
  ser_coords.vertical[0] = Serial3.read();
  ser_coords.vertical[1] = Serial3.read();
  ser_coords.vertical[2] = Serial3.read();
  ser_coords.vertical[3] = Serial3.read();
  // fire or no fire
  fire = Serial3.read();
  // do we want to send # of darts?
  }
  getEndFrame();
  return fire;
}

/********************************************************
 * Sleep while nothing happening
 *******************************************************/
void sleep(int interruptPin)
{
#ifdef DEBUG
  Serial.println("Going to sleep");
#endif
  // sleep the arduino until the interrupt is triggered
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(interruptPin), wakeupISR, FALLING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
  // after waking
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  
#ifdef DEBUG
  Serial.println("I'm awake!");
#endif
}

/******************************************************************************
 *
 ******************************************************************************/
void setup() 
{
  Serial3.begin(9600); // we can probably make this faster
  while (!Serial3) {}
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {}
#endif
  // Set the speed to 15 rpm:
  MagStepper.setSpeed(13);
  FiringPin.setSpeed(13);

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
  pinMode(COMMUNICATION_INTERRUPT, INPUT_PULLUP);
  pinMode(VERTICAL_TOP, INPUT_PULLUP);
  pinMode(VERTICAL_BOTTOM, INPUT_PULLUP);
  pinMode(FIRE_FEEDBACK, INPUT);
  pinMode(MAG_POSITION, INPUT_PULLUP);
  pinMode(RELOAD_N_POWER_DOWN, INPUT_PULLUP);

  numDartsFired = 0;
  numMagsUsed = 0;
  magsLoaded = false;

  // interrupts
#ifdef ENSURE_FIRE
  attachInterrupt(digitalPinToInterrupt(FIRE_FEEDBACK), dartFired, RISING);
#endif
#ifdef POWER_DOWN
  attachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN), powerDown, FALLING);
#endif
}

/**************************************************************************
 *
 *************************************************************************/
void loop()
{
  if (!magsLoaded)
  {
#ifdef POWER_DOWN
    detachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN));
#endif
    Serial3.write(NOT_READY); // send a code back to wait while waiting for load
    sleep(RELOAD_N_POWER_DOWN);
    resetMagHolder();
    Serial3.write(READY); // send code to begin aiming
#ifdef POWER_DOWN
    attachInterrupt(digitalPinToInterrupt(RELOAD_N_POWER_DOWN), powerDown, FALLING);
#endif
  }
  
  sleep(COMMUNICATION_INTERRUPT);
  
  serializedGunCoords ser_coords;
  bool fire = getInput(ser_coords);

  gunCoords coords;
  coords.horizontal = deserialize<float>(ser_coords.horizontal);
  coords.vertical = deserialize<float>(ser_coords.vertical);
  
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
  previousCoords = coords;
  
  // change mag if needed
  if (numDartsFired == DARTS_PER_MAG)
  {
    moveMagHolder();
  }
}
