const int RX = 0;
const int TX = 1;
const int SEND_START = 8;

const int START_FRAME = 0xDEAD;
const int END_FRAME = 0x5AFE;

boolean fire;
byte horizontal[4];
byte vertical[4];

template <class t>
uint8_t* serialize(t *data)
{
  uint8_t *serialData = new uint8_t [sizeof(t)];
  memcpy((void *)serialData, (void*)data, sizeof(t));
  return serialData;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(SEND_START, OUTPUT);
  fire = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  float h = ((float)random(0, 720)) / 2.0;
  float v = ((float)random(0, 180)) / 2.0;

  uint8_t * ser_h = serialize(&h);
  uint8_t * ser_v = serialize(&v);
  uint8_t * startFrame = serialize(&START_FRAME);
  uint8_t * endFrame = serialize(&END_FRAME);

  digitalWrite(SEND_START, HIGH);
  delay(50);
  digitalWrite(SEND_START, LOW);
  // start frame
  Serial.write(startFrame, 2);
  // data
  Serial.write(ser_h, 4);
  Serial.write(ser_v, 4);
  Serial.write(fire);
  // end frame
  Serial.write(endFrame, 2);
  
  delay(1000);
  fire = !fire;

  delay(50000);
}
