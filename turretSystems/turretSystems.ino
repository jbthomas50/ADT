// Includes
#include <Stepper.h>
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

const float DEG_PER_STEP = 360 / stepsPerRevolution;
const float STEP_PER_DEG = stepsPerRevolution / 360;
// Create stepper objects, note the pin order:
Stepper HorizontalStepper = Stepper(stepsPerRevolution, XY_IN1, XY_IN3, XY_IN2, XY_IN4);
Stepper VerticalStepper = Stepper(stepsPerRevolution, Z_IN1, Z_IN3, Z_IN2, Z_IN4);

int xCenter;
int yCenter;
// Use this to get serial input byte by byte and use as a float
union byteChipper
{
  float angle;
  char choppedAngle[4];
};

struct gunCoords
{
  byteChipper horizontal;
  byteChipper vertical;
};

gunCoords previousCoords;

// ISR to start aiming/firing process
void wakeupISR(void* param)
{
  sleep_disable();
}

// Initial calibration
void calibrate()
{
  while (!digitalRead(HORIZONTAL_CALIBRATION))
  {
    HorizontalStepper.step(1);
  }
  previousCoords.horizontal.angle = 0;
  while(!digitalRead(VERTICAL_CALIBRATION))
  {
    VerticalStepper.step(1);
  }
  previousCoords.vertical.angle = 0;
}

// turret control functions
void aim(gunCoords coords)
{
  Serial.println("Aiming");
  // determine how many degrees away we are
  float horizontalDegrees = coords.horizontal.angle - previousCoords.horizontal.angle;
  float verticalDegrees = coords.vertical.angle - previousCoords.vertical.angle;
  // determine how many steps to take
  int horizontalSteps = STEP_PER_DEG * horizontalDegrees;
  int verticalSteps = STEP_PER_DEG * verticalDegrees;
  // take the steps
  HorizontalStepper.step(horizontalSteps);
  VerticalStepper.step(verticalSteps);
}

void fireOrIntimidate(bool fire)
{
  if (fire)
  {
    Serial.println("Firing");
    digitalWrite(FIRING_PIN, HIGH);
    delay(500);
    digitalWrite(FIRING_PIN, LOW);
  }
  else
  {
    Serial.println("Intimidating");
  }
}

bool getInput(gunCoords & coords)
{
  while (!Serial.available()){}
  for (int i = 0; i < 4; i++)
  {
    coords.horizontal.choppedAngle[i] = Serial.read();
  }
  for (int i = 0; i < 4; i++)
  {
    coords.vertical.choppedAngle[i] = Serial.read();
  }
  bool fire = Serial.read();
  return fire;
}

void setup() 
{
  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);
  // Set the speed to 15 rpm:
  HorizontalStepper.setSpeed(15);
  VerticalStepper.setSpeed(15);

  // output pins
  pinMode(FIRING_PIN, OUTPUT);

  // input pins
  pinMode(COMMUNICATION_INTERRUPT, INPUT_PULLUP);
  pinMode(HORIZONTAL_CALIBRATION, INPUT);
  pinMode(VERTICAL_CALIBRATION, INPUT);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMMUNICATION_INTERRUPT), wakeupISR, RISING);

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
  gunCoords coords;
  bool fire = getInput(coords);
  
  Serial.println(coords.horizontal.angle);
  Serial.println(coords.vertical.angle);
  Serial.println(fire);
  
  aim(coords);
  fireOrIntimidate(fire);
  previousCoords = coords;
}
