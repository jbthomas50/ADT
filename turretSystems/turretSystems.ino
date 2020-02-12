// 1 Arduino for aiming
// 1 Arduino for firing control

// Includes
#include <Stepper.h>
#include <Servo.h>
#include <avr/sleep.h>
// Define number of steps per rotation:
const int stepsPerRevolution = 2048;

/*********** PINS *************/
// UART
const int RX = 0;
const int TX = 1;
// communication interrupt
const int COMMUNICATION_INTERRUPT = 2;
// save pin 3 for possible future interrupt
const int FIRE_FEEDBACK = 3; // if we choose to detect a misfire
// Z axis motor
const int Z_IN1 = 4;
const int Z_IN2 = 5;
const int Z_IN3 = 6;
const int Z_IN4 = 7;
// firing pin
const int FIRING_PIN = 8;
// enable for the motor to move the mag holder
const int MOTOR_ENABLE = 9;
// XY axis motor
const int XY_IN1 = 10;
const int XY_IN2 = 11;
const int XY_IN3 = 12;
const int XY_IN4 = 13;

// firing motor. Only need one pin since it only goes one way
const int SPIN_UP = A0;
// motor control for the mag holder. Need more pins because this one spins two ways
const int MAG_MOVER = A1;
const int MAG_RESET = A2;
// signals when to stop moving the mag holder
const int MAG_POSITION = A3;
const int HORIZONTAL_CALIBRATION = A4;
const int VERTICAL_CALIBRATION = A5;

/***************** END OF PINS*********************/

const float GEAR_RATIO = 360.0 * 4.0; // four full rotations for a 360 degree turn
const float STEP_PER_DEG = stepsPerRevolution / GEAR_RATIO;
const int DARTS_PER_MAG = 12;
const int NUM_MAGS = 4;
// Create stepper objects, note the pin order: 1 3 2 4
Stepper HorizontalStepper = Stepper(stepsPerRevolution, XY_IN1, XY_IN3, XY_IN2, XY_IN4);
Stepper VerticalStepper = Stepper(stepsPerRevolution, Z_IN1, Z_IN3, Z_IN2, Z_IN4);
Servo firingPin;

int xCenter;
int yCenter;

int numDartsFired;
int numMagsUsed;

bool spinning;
volatile bool fired;

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

/********************************************
 * ISR to ensure dart firing.
 ********************************************/
void dartFired(void* param)
{
  fired = true;
}

/********************************************
 * Initial calibration
*********************************************/
void calibrate()
{
  while (!digitalRead(HORIZONTAL_CALIBRATION))
  {
    HorizontalStepper.step(1);
  }
  previousCoords.horizontal = 0.0;
  while(!digitalRead(VERTICAL_CALIBRATION))
  {
    VerticalStepper.step(1);
  }
  previousCoords.vertical = 0.0;
}

/*************************************************
 * turret control functions
**************************************************/
void aim(gunCoords coords)
{
  // determine how many degrees away we are
  // choose the way that is shorter
  float right = coords.horizontal - previousCoords.horizontal;
  float left = previousCoords.horizontal - coords.horizontal;
  float horizontalDegrees = abs(right) < abs(left) ? right : left;
  // this is very important to get right since we don't have a full 360 degree turn. We can't go too far either way.
  float up = coords.vertical - previousCoords.vertical;
  float down = previousCoords.vertical - coords.vertical;
  float verticalDegrees = abs(up) < abs(down) ? up : down;
  
  // determine how many steps to take
  int horizontalSteps = STEP_PER_DEG * horizontalDegrees;
  int verticalSteps = STEP_PER_DEG * verticalDegrees;
  
  // take the steps
  HorizontalStepper.step(horizontalSteps);
  VerticalStepper.step(verticalSteps);
}

/********************************************************************************/
void fireOrIntimidate(bool fire, int numDarts = 1)
{
  
  if (fire)
  {
    int pos = 0;
    for (int i = 0; i < numDarts; i++)
    {
      fired = false;
      // move the firing pin forward
      for (pos; pos <= 180; pos += 10) {
        firingPin.write(pos);
        delay(50);
      }
      // move firing pin back
      for (pos; pos >= 0; pos -= 10) {
        firingPin.write(pos);
        delay(50); 
      }
      firingPin.write(0);
      delay(200);
      
      if (fired == false)
      {
        // set i back one to retry firing
        //i--;
      }
      else
      {
        numDartsFired++;
      }
    }
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
    // We've used all the mags so we have to reset
    digitalWrite(MAG_MOVER, LOW);
    digitalWrite(MAG_RESET, HIGH);
    digitalWrite(MOTOR_ENABLE, HIGH);
    int mag = 1;
    while (mag != NUM_MAGS)
    {
      if (!digitalRead(MAG_POSITION))
      {
        mag++;
        while(!digitalRead(MAG_POSITION) && mag != NUM_MAGS)
        {
          // wait to pass current mag  
        }
      }
    }
    digitalWrite(MOTOR_ENABLE, HIGH);
    numMagsUsed = 0;
  }
  else
  {
    // Move to the next mag
    digitalWrite(MAG_MOVER, LOW);
    digitalWrite(MAG_RESET, HIGH);
    digitalWrite(MOTOR_ENABLE, HIGH);
    while (!digitalRead(MAG_POSITION))
    {
      // make sure we get all the way past the current mag
    }
    while (digitalRead(MAG_POSITION))
    {
      // get to the next mag
    }
    digitalWrite(MOTOR_ENABLE, HIGH);
  }
}


/**********************************************
 * Used for spinning up the firing motor
 **********************************************/
void toggleSpinUp()
{
  spinning = !spinning;
  digitalWrite(SPIN_UP, spinning);
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
  Serial.read();
  Serial.read();
  return true;
}

/******************************************************************
 * empty the buffer
 * TODO: check for end frameto ensure data is received correctly
 *****************************************************************/
void getEndFrame()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

/********************************************************
 * Get the coordinates and the command to fire or not
 *******************************************************/
bool getInput(serializedGunCoords & ser_coords)
{
  bool fire = false;
  while (!Serial.available()){}
  if(getStartFrame())
  {
  // Horizontal angle
  ser_coords.horizontal[0] = Serial.read();
  ser_coords.horizontal[1] = Serial.read();
  ser_coords.horizontal[2] = Serial.read();
  ser_coords.horizontal[3] = Serial.read();
  // vertical angle
  ser_coords.vertical[0] = Serial.read();
  ser_coords.vertical[1] = Serial.read();
  ser_coords.vertical[2] = Serial.read();
  ser_coords.vertical[3] = Serial.read();
  // fire or no fire
  fire = Serial.read();
  // do we want to send # of darts?
  }
  getEndFrame();
  return fire;
}

/********************************************************************************/
void setup() 
{
  Serial.begin(9600); // we can probably make this faster
  // Set the speed to 15 rpm:
  HorizontalStepper.setSpeed(15);
  VerticalStepper.setSpeed(15);

  // start the servo at 0
  firingPin.attach(FIRING_PIN);
  firingPin.write(0);

  // output pins
  pinMode(SPIN_UP, OUTPUT);
  pinMode(MAG_MOVER, OUTPUT);
  pinMode(MAG_RESET, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  // initialize the outputs just to be sure
  digitalWrite(SPIN_UP, LOW);
  digitalWrite(MAG_MOVER, LOW);
  digitalWrite(MAG_RESET, LOW);
  digitalWrite(MOTOR_ENABLE, LOW);

  // input pins
  pinMode(COMMUNICATION_INTERRUPT, INPUT_PULLUP);
  pinMode(HORIZONTAL_CALIBRATION, INPUT);
  pinMode(VERTICAL_CALIBRATION, INPUT);
  pinMode(FIRE_FEEDBACK, INPUT);
  pinMode(MAG_POSITION, INPUT_PULLUP);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMMUNICATION_INTERRUPT), wakeupISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FIRE_FEEDBACK), dartFired, RISING);

  spinning = false;
  numDartsFired = 0;
  numMagsUsed = 0;

  // calibrate the gun angles
  calibrate();
}

/********************************************************************************/
void loop()
{
  // sleep the arduino until the interrupt is triggered
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
  Serial.println("I'm awake!");
  serializedGunCoords ser_coords;
  bool fire = getInput(ser_coords);

  gunCoords coords;
  coords.horizontal = deserialize<float>(ser_coords.horizontal);
  coords.vertical = deserialize<float>(ser_coords.vertical);
  
  if (fire)
  {
    toggleSpinUp();    
  }
  aim(coords);
  fireOrIntimidate(fire);
  if (fire)
  {
    toggleSpinUp();    
  }
  previousCoords = coords;
  
  // change mag if needed
  if (numDartsFired == DARTS_PER_MAG)
  {
    moveMagHolder();
  }
}
