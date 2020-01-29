// 1 Arduino for aiming
// 1 Arduino for firing control

// Includes
#include <Stepper.h>
#include <Servo.h>
#include <avr/sleep.h>
// Define number of steps per rotation:
const int stepsPerRevolution = 2048;
// UART
#define RX 0
#define TX 1
// communication interrupt
#define COMMUNICATION_INTERRUPT 2
// interrupt to calibrate the turret angle
#define FIRING_PIN 3
// Z axis motor
#define Z_IN1 4
#define Z_IN2 5
#define Z_IN3 6
#define Z_IN4 7
// XY axis motor
#define XY_IN1 8
#define XY_IN2 9
#define XY_IN3 10
#define XY_IN4 11
// firing pin
#define HORIZONTAL_CALIBRATION 12
#define VERTICAL_CALIBRATION 13

#define SPIN_UP A0

const float GEAR_RATIO = 360.0 / 4.0; // four full rotations for a 360 degree turn
const float STEP_PER_DEG = stepsPerRevolution / GEAR_RATIO;
// Create stepper objects, note the pin order: 1 3 2 4
Stepper HorizontalStepper = Stepper(stepsPerRevolution, XY_IN1, XY_IN3, XY_IN2, XY_IN4);
Stepper VerticalStepper = Stepper(stepsPerRevolution, Z_IN1, Z_IN3, Z_IN2, Z_IN4);
Servo firingPin;

int xCenter;
int yCenter;

bool spinning;

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
  Serial.println("calibrating");
}

/*************************************************
 * turret control functions
**************************************************/
void aim(gunCoords coords)
{
  Serial.println("Aiming");
  
  // determine how many degrees away we are
  float horizontalDegrees = coords.horizontal - previousCoords.horizontal;
  float verticalDegrees = coords.vertical - previousCoords.vertical;
  
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
    Serial.println("Firing");
    int pos = 0;
    for (int i = 0; i < numDarts; i++)
    {
      // move the firing pin forward
      for (pos = 0; pos <= 180; pos += 10) {
        firingPin.write(pos);
        delay(15);
      }
      // move firing pin back
      for (pos = 180; pos >= 0; pos -= 10) {
        firingPin.write(pos);
        delay(15); 
      }
      firingPin.write(0);
      delay(200);
    }
    
  }
  else
  {
    Serial.println("Intimidating");
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
  digitalWrite(SPIN_UP, LOW);

  // input pins
  pinMode(COMMUNICATION_INTERRUPT, INPUT_PULLUP);
  pinMode(HORIZONTAL_CALIBRATION, INPUT);
  pinMode(VERTICAL_CALIBRATION, INPUT);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMMUNICATION_INTERRUPT), wakeupISR, RISING);

  spinning = false;

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
}
